#include <Wire.h>

#define I2C_ADDR        0x34
#define MOTOR_TYPE_ADDR               20
#define MOTOR_ENCODER_POLARITY_ADDR   21
#define MOTOR_FIXED_SPEED_ADDR        51

#define TURN_DURATION 1000  // Turn right for 1 second
#define FORWARD_DURATION 8000 // Move forward for 8 seconds

uint8_t MotorType = 3; // MOTOR_TYPE_JGB37_520_12V_110RPM
uint8_t MotorEncoderPolarity = 0;

void setup() {
    Wire.begin();
    Serial.begin(115200);
    
    delay(200);
    WireWriteDataArray(MOTOR_TYPE_ADDR, &MotorType, 1);
    delay(5);
    WireWriteDataArray(MOTOR_ENCODER_POLARITY_ADDR, &MotorEncoderPolarity, 1);
    Serial.println("Basic Movement Mode Active");
}

// Movement Commands (Motor 1 = Left, Motor 2 = Right)
int8_t car_forward[4]  = {-23, -23, 0, 0};   // Move Forward
int8_t car_turnRight[4] = {-23, 23, 0, 0};   // Turn Right
int8_t car_stop[4]     = {0, 0, 0, 0};       // Stop

void loop() {
    // Move forward for 8 seconds
    WireWriteDataArray(MOTOR_FIXED_SPEED_ADDR, car_forward, 4);
    Serial.println("Moving Forward");
    delay(FORWARD_DURATION);
    
    // Turn right for 1 second
    WireWriteDataArray(MOTOR_FIXED_SPEED_ADDR, car_turnRight, 4);
    Serial.println("Turning Right");
    delay(TURN_DURATION);
    
    // Stop movement
    WireWriteDataArray(MOTOR_FIXED_SPEED_ADDR, car_stop, 4);
    Serial.println("Stopping");
    
    while (1); // Break the loop, stopping execution
}

// I2C Communication Function
bool WireWriteDataArray(uint8_t reg, uint8_t *val, unsigned int len) {
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(reg);
    for (unsigned int i = 0; i < len; i++) {
        Wire.write(val[i]);
    }
    return (Wire.endTransmission() == 0);
}
