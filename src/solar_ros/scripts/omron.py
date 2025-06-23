
# from solar_ros.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray,Int32

class OmronNode(Node):
    def __init__(self):
        super().__init__('omron_node')
        self.publisher_ = self.create_publisher(Float32MultiArray, "/cmd_vel", 10)
        self.publisher_B = self.create_publisher(Int32, "/cubemx_publisher_Brush", 10)
        self.publisher_L = self.create_publisher(Float32MultiArray, "/lidar_state", 10)
        self.create_subscription(Int32, "/cubemx_publisher_Omron", self.feedback_omron_callback, 10)
    
    def feedback_omron_callback(self, msg:Int32):
        if msg.data == 1:
            ros_message = Float32MultiArray()
            ros_message.data = [float(0), float(0)]
            B_msg = Int32()
            B_msg.data = int(0)
            self.publisher_B.publish(B_msg)
            self.publisher_.publish(ros_message)
            L_msg = Float32MultiArray()
            L_msg.data = [float(0), float(0), float(0)]
            self.publisher_L.publish(L_msg)
            self.get_logger().info("Stop all cannot detect panel!!")

def main(args=None):
    rclpy.init(args=args)
    node = OmronNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()