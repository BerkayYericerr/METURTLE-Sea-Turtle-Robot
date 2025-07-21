from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import rclpy

class CoordinatePublisher(Node):
    def __init__(self):
        super().__init__('coordinate_publisher')
        self.publisher = self.create_publisher(Float64MultiArray, 'coordinates', 10)

        self.subscription2 = self.create_subscription(
            Float64MultiArray,
            'coordinatessimulated',
            self.manipulate_coordiantes,
            10)
        
        self.latitude = 35.3342  # Approximate latitude range of Turkey
        self.longitude = 33.4914 # Approximate longitude range of Turkey
         

        self.timer = self.create_timer(0.5, self.publish_coordinates)  
        
    def manipulate_coordiantes(self, msg):
        
        self.latitude = float(msg.data[0])
        self.longitude = float(msg.data[1])





    def publish_coordinates(self):
        # Generate random latitude and longitude within Turkey's approximate bounds
        #latitude = random.uniform(36.0, 42.0)  # Approximate latitude range of Turkey
        #longitude = random.uniform(26.0, 45.0) # Approximate longitude range of Turkey
        #heading = random.uniform(0.0, 365.0)


        msg = Float64MultiArray()
        msg.data = [float(self.latitude), float(self.longitude)]
        self.publisher.publish(msg)
        self.get_logger().info(f'Coordinates published: Latitude {self.latitude}, Longitude {self.longitude}')

def main(args=None):
    rclpy.init(args=args)
    node = CoordinatePublisher()
    rclpy.spin(node)
    rclpy.shutdown()
