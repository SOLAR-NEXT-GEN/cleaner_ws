#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32, Float32


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.timer = self.create_timer(0.01, self.timer_callback)
        self.create_subscription(Float32MultiArray, "/cmd_vel", self.cmd_callback, 10)

        self.speed_l_publisher = self.create_publisher(Float32, '/cubemx_publisher_L', 10)
        self.speed_r_publisher = self.create_publisher(Float32, '/cubemx_publisher_R', 10)
        self.servo_publisher = self.create_publisher(Int32, '/cubemx_publisher_servo', 10)
        self.brush_publisher = self.create_publisher(Int32, '/cubemx_publisher_Brush', 10)

        self.cmd_vel = [0.0, 0.0]
        self.servo = 1
        self.brush = 2

    def timer_callback(self):
        speed_l = Float32()
        speed_l.data = self.cmd_vel[0]
        self.speed_l_publisher.publish(speed_l)

        speed_r = Float32()
        speed_r.data = self.cmd_vel[1]
        self.speed_r_publisher.publish(speed_r)

        servo = Int32()
        servo.data = self.servo
        self.servo_publisher.publish(servo)

        brush = Int32()
        brush.data = self.brush
        self.brush_publisher.publish(brush)        

    def cmd_callback(self, msg:Float32MultiArray):
        self.cmd_vel[0] = msg.data[0]
        self.cmd_vel[1] = msg.data[1]

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
