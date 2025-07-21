#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <Arduino.h>

#define MOTOR1_PIN1 18
#define MOTOR1_PIN2 5
#define ENABLE1_PIN 19
#define MOTOR2_PIN1 16
#define MOTOR2_PIN2 4
#define ENABLE2_PIN 17

#define TRIG_PIN 21
#define ECHO_PIN 22
#define SOUND_SPEED 0.034
#define NUM_SAMPLES 3
#define SAFE_THRESHOLD_CM 15

#define BUZZER_PIN 23
#define BUZZER_CHANNEL 0
#define BUZZER_RESOLUTION 8 

// ROS 2
rcl_publisher_t movement_pub;
rcl_publisher_t distance_pub;
rcl_subscription_t movement_sub;
std_msgs__msg__Int32 cmd_msg;
std_msgs__msg__Int32 dist_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Micro-ROS error checking macros
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { while(1){ delay(100); } } }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) {} }

// FreeRTOS Task Handles and Shared Variables
TaskHandle_t buzzerTaskHandle = NULL;
volatile bool buzzer_active = false; // Flag to control buzzer task activity

// Function for the buzzer FreeRTOS task
void buzzer_task(void *arg) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 2000 / portTICK_PERIOD_MS; // Update frequency every 2 seconds

  // Initialize xLastWakeTime with the current tick count
  xLastWakeTime = xTaskGetTickCount();

  while (1) {
    // Only perform buzzer actions if it's active
    if (buzzer_active) {
      int freq = random(2000, 10001); 
      ledcSetup(BUZZER_CHANNEL, freq, BUZZER_RESOLUTION); // Re-setup for frequency change
      ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);
      ledcWrite(BUZZER_CHANNEL, 128); // 50% duty cycle
    } else {
      // If not active, turn off the buzzer and detach the pin
      ledcWrite(BUZZER_CHANNEL, 0);
    }
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

// Movement and control of the robot
void move_robot(int cmd) {
  switch (cmd) {
    case 1: // Forward
      digitalWrite(MOTOR1_PIN1, HIGH); digitalWrite(MOTOR1_PIN2, LOW);
      digitalWrite(MOTOR2_PIN1, HIGH); digitalWrite(MOTOR2_PIN2, LOW);
      break;

    case 2: // Backward
      digitalWrite(MOTOR1_PIN1, LOW); digitalWrite(MOTOR1_PIN2, HIGH);
      digitalWrite(MOTOR2_PIN1, LOW); digitalWrite(MOTOR2_PIN2, HIGH);
      break;

    case 3: // Left
      digitalWrite(MOTOR1_PIN1, LOW); digitalWrite(MOTOR1_PIN2, HIGH);
      digitalWrite(MOTOR2_PIN1, HIGH); digitalWrite(MOTOR2_PIN2, LOW);
      break;

    case 4: // Right
      digitalWrite(MOTOR1_PIN1, HIGH); digitalWrite(MOTOR1_PIN2, LOW);
      digitalWrite(MOTOR2_PIN1, LOW); digitalWrite(MOTOR2_PIN2, HIGH);
      break;

    case 5: // Buzzer On
      buzzer_active = true; // Set flag to activate buzzer
      // No return here, allow motors to be enabled/controlled
      break;

    case 6: // Buzzer Off
      buzzer_active = false; // Set flag to deactivate buzzer
      // No return here, allow motors to be enabled/controlled
      break;

    default: // Stop
      digitalWrite(MOTOR1_PIN1, LOW); digitalWrite(MOTOR1_PIN2, LOW);
      digitalWrite(MOTOR2_PIN1, LOW); digitalWrite(MOTOR2_PIN2, LOW);
      break;
  }

  digitalWrite(ENABLE1_PIN, HIGH);
  digitalWrite(ENABLE2_PIN, HIGH);
}

// ROS subscription callback function
void subscription_callback(const void *msgin) {
  const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
  Serial.print("Received command: ");
  Serial.println(msg->data);
  move_robot(msg->data); // Process the received command

  // Publish the received command back to /movement2 (for feedback/logging)
  cmd_msg.data = msg->data;
  rcl_publish(&movement_pub, &cmd_msg, NULL);
}

// Ultrasonic sensor reading FreeRTOS task
void ultrasonic_task(void *arg) {
  float distance_samples[NUM_SAMPLES] = {0};
  int sample_index = 0;

  while (1) {
    // Trigger the sensor
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2); 
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10); 
    digitalWrite(TRIG_PIN, LOW);

    // Measure echo time with a timeout
    long duration = pulseIn(ECHO_PIN, HIGH, 30000); // Timeout after 30ms

    // Calculate distance (in cm)
    float distance = duration * SOUND_SPEED / 2.0;

    // Validate distance and average samples
    if (distance > 0 && distance < 400) { // Valid range for common HC-SR04
      distance_samples[sample_index] = distance;
      sample_index = (sample_index + 1) % NUM_SAMPLES;

      float sum = 0;
      for (int i = 0; i < NUM_SAMPLES; i++) {
        sum += distance_samples[i];
      }
      float avg_distance = sum / NUM_SAMPLES;

      // Publish distance: 0 if below safe threshold, otherwise the averaged distance
      dist_msg.data = (avg_distance < SAFE_THRESHOLD_CM) ? 0 : (int)avg_distance;
      RCSOFTCHECK(rcl_publish(&distance_pub, &dist_msg, NULL)); 

      Serial.print("Averaged Distance (cm): ");
      Serial.println(avg_distance);
    } else {
      Serial.println("Distance out of range or timeout.");
    }
    vTaskDelay(500 / portTICK_PERIOD_MS); // Run every 500ms
  }
}

void setup() {
  set_microros_transports(); // Initialize micro-ROS transport layer
  Serial.begin(115200);      // Start serial communication for debugging

  // Setup motor control pins as outputs
  pinMode(MOTOR1_PIN1, OUTPUT); pinMode(MOTOR1_PIN2, OUTPUT); pinMode(ENABLE1_PIN, OUTPUT);
  pinMode(MOTOR2_PIN1, OUTPUT); pinMode(MOTOR2_PIN2, OUTPUT); pinMode(ENABLE2_PIN, OUTPUT);
  // Setup ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);

  // Initialize LEDC (PWM) for buzzer but keep it off initially
  ledcSetup(BUZZER_CHANNEL, 2000, BUZZER_RESOLUTION); 
  ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);
  ledcWrite(BUZZER_CHANNEL, 0);

  
  xTaskCreatePinnedToCore(
    buzzer_task,        // Function 
    "Buzzer Task",      // A descriptive name for the task
    2048,               // Stack size (bytes)
    NULL,               // Task parameters
    1,                  // Priority 
    &buzzerTaskHandle,  // Task handle to reference the task
    0                   // Core to run on 
  );
  buzzer_active = false; // Ensure buzzer starts inactive

  delay(2000); // Allow time for micro-ROS agent to connect

  // Initialize ROS 2
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "motor_ultrasonic_node", "", &support));

  // Create ROS 2 publishers
  RCCHECK(rclc_publisher_init_default(
    &movement_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "movement2"));

  RCCHECK(rclc_publisher_init_default(
    &distance_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "distance"));

  // Create ROS 2 subscription
  RCCHECK(rclc_subscription_init_default(
    &movement_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "movement"));

  // Configure ROS 2 executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator)); // 1 handle
  RCCHECK(rclc_executor_add_subscription(&executor, &movement_sub, &cmd_msg, &subscription_callback, ON_NEW_DATA));

  // Start ultrasonic sensor task on Core 1 
  xTaskCreatePinnedToCore(
    ultrasonic_task,    // Function 
    "Ultrasonic Task",  // A descriptive name for the task
    4096,               // Stack size (bytes) 
    NULL,               // Task parameter (not used here)
    1,                  // Priority
    NULL,               // Task handle
    1                   // Core to run on (Core 1)
  );
}

void loop() {
  delay(10); 
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10))); // Spin the executor for 10ms
}