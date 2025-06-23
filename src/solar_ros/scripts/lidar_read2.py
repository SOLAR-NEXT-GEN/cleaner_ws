#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
from std_msgs.msg import Float32MultiArray, Int32


class LidarReadNode(Node):
    def __init__(self):
        super().__init__('lidar_read_node')

        self.lim_pos_publisher = self.create_publisher(LaserScan, '/scan2', 10)
        self.brush_publisher = self.create_publisher(Int32, '/cubemx_publisher_servo', 10)
        self.cmd_vel_publisher = self.create_publisher(Float32MultiArray, '/cmd_vel', 10)
        self.feedback_lidar_publisher = self.create_publisher(Int32, '/feedback_lidar', 10)

        #subscription
        self.create_subscription(LaserScan, "/scan", self.lidar_callback, 10)

        self.create_timer(0.01, self.timer_callback)

        #variable
        self.max_speed = 2.0
        self.kp = [2.0, 70.0]
        self.stack_pos = []
        self.deg = [270, 90]
        self.dh = [0.24, 0.46]
        self.prev_distance = [0,0]
        self.side = 'none'
        self.servo_state = 0
        self.servo_timecount = 0
        self.side = 0
        self.error_count = 0

    def timer_callback(self):
        if self.error_count >= 500:
            feedback_msg = Int32()
            feedback_msg.data = 0
            self.feedback_lidar_publisher.publish(feedback_msg)
        else:
            self.error_count += 1
            feedback_msg = Int32()
            feedback_msg.data = 1
            self.feedback_lidar_publisher.publish(feedback_msg)
        if self.servo_state == 1 and self.servo_timecount < 500:
            self.servo_timecount += 1
        elif self.servo_state == 1:
            self.servo_state = 0
            self.servo_timecount = 0
            brush_msg = Int32()
            brush_msg.data = 0
            self.brush_publisher.publish(brush_msg)


    def cmd_pub(self, distance):
        msg = Float32MultiArray()
        #for change direction
        a = 1

        #if have solarcell below
        if abs(distance[0]) >= 0.005 and abs(distance[1]) >= 0.005:

            # check if solarcell shift left or right
            error = distance[0] + distance[1]
            if abs(error) < 0.1:
                # if solarcell in the middle
                error = distance[0] - self.prev_distance[0]
                #check if solarcell rotate
                if abs(error) > 0.001:

                    #กันเวลาหมุนกลับแล้วค่า previous ไม่ได้เปลี่ยนเพราะเดินหน้าปกติ ให้เดินหน้าปกติก่อนค่อยหมุนใหม่
                    if (self.side != 'left' and error > 0) or (self.side != 'right' and error < 0):
                        v = [-(self.kp[1] * error), +(self.kp[1] * error)]
                        if error > 0:
                            self.side = 'right'
                        else:
                            self.side = 'left'
                    else:
                        v = [self.max_speed, self.max_speed]
                else:
                    v = [self.max_speed, self.max_speed]
                    self.side = 'none'
            else:
                v = [self.max_speed-(self.kp[0] * error), self.max_speed+(self.kp[0] * error)]
                self.side = 'none'

            #check limit real max speed = 1.5 * max_speed
            if abs(v[0]) > self.max_speed*1.5:
                v[0] = self.max_speed * 1.5 * (v[0]/abs(v[0]))
            if abs(v[1]) > self.max_speed*1.5:
                v[1] = self.max_speed * 1.5 * (v[1]/abs(v[1]))
            self.prev_distance = distance
        else:
            v = [0.0, 0.0]

        msg.data = [v[0], v[1]]
        self.cmd_vel_publisher.publish(msg)
        self.get_logger().info(f'Publishing speed data =  {v}')


    def publish_lidar_data(self, msg:LaserScan, deg):
        scan = msg
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.angle_min = np.deg2rad(deg[0])
        scan.angle_max = np.deg2rad(deg[1])
        scan.ranges = self.lim_deg
        self.lim_pos_publisher.publish(scan)

    def cal_something(self,msg:LaserScan):
        self.stack_pos = []
        self.dx = []
        self.dy = []      
        for i in range(len(msg)):
            self.stack_pos.append(round(msg[i],2))
            self.dx.append(round(np.sin(np.deg2rad(i*0.5))*msg[i],5))
            self.dy.append(round(np.cos(np.deg2rad(i*0.5))*msg[i],5))
        
    def lim_deg_pos(self, first, last):
        self.lim_deg = []
        self.lim_dx = []
        self.lim_dy = []
        if first < last:
            self.lim_deg = self.stack_pos[first*2:last*2]
            self.lim_dx = self.dx[first*2:last*2]
            self.lim_dy = self.dy[first*2:last*2]
        else:
            self.lim_deg = self.stack_pos[first*2:720]
            self.lim_deg = self.lim_deg[:] + self.stack_pos[0:last*2]
            self.lim_dx = self.dx[first*2:720]
            self.lim_dx = self.lim_dx[:] + self.dx[0:last*2]
            self.lim_dy = self.dy[first*2:720]
            self.lim_dy = self.lim_dy[:] + self.dy[0:last*2]
        # print('dy')
        # print(self.lim_dy)
        # print('dx')
        # print(self.lim_dx)

    def distance_check(self, dy, dx, dh):
        distance = [0,0]
        switch__l = 0
        switch__r = 0
        range_left = [dy[len(dy)//2 -1]-0.02, dy[len(dy)//2 -1]+0.02]
        range_right = [dy[len(dy)//2]-0.02, dy[len(dy)//2]+0.02]
        a = 1
        while(abs(range_left[0])==np.inf or abs(range_right[0])==np.inf):
            if abs(range_left[0])==np.inf:
                range_left = [dy[len(dy)//2 -1 - a]-0.02, dy[len(dy)//2 -1 - a]+0.02]
            if abs(range_right[0])==np.inf:
                range_right = [dy[len(dy)//2 + a]-0.02, dy[len(dy)//2 + a]+0.02]
            a+=1
        stack_left = 1
        stack_right = 1

        for i in range(len(dy)//2):
            left = len(dy)//2 -1 - i
            right = len(dy)//2 + i

            if abs(dy[left]) != np.inf:
                if min(range_left) <= dy[left] <= max(range_left):
                    range_left = [dy[left]-(0.02*stack_left), dy[left]+(0.02*stack_left)]
                    stack_left = 1
                else:
                    if distance[0] == 0 and switch__l != 1:
                        print(range_left, dy[left])
                        distance[0] = dx[left+stack_left]
                        switch__l = 1 
                        print('left',dy[left], dy[left+stack_left], left)
            else:
                stack_left += 1

            if abs(dy[right]) != np.inf:
                if min(range_right) <= dy[right] <= max(range_right):
                    range_right = [dy[right]-(0.02*stack_right), dy[right]+(0.02*stack_right)]
                    stack_right = 1
                else:
                    if distance[1] == 0:
                        distance[1] = dx[right-stack_right]
                        print('right',dy[right], dy[right-stack_right], right)
            else:
                stack_right += 1

            if distance[0] != 0 and distance[1] != 0:
                break
        self.get_logger().info(f'distance data =  {distance}')
        return distance
            
    def lidar_callback(self, msg:LaserScan):
        self.error_count = 0
        range = msg.ranges
        self.cal_something(range)
        self.lim_deg_pos(self.deg[0],self.deg[1])
        d = self.distance_check(self.lim_dy, self.lim_dx, self.dh)
        self.cmd_pub(d)
        self.publish_lidar_data(msg, self.deg)
        self.brush_check(self.lim_dy, self.dh)
    
    def brush_check(self, dy, dh):
        # print("brush", dy[(len(dy)//2)-2:(len(dy)//2)+3]) 
        # print((dy[len(dy)//2] > max(dh) or dy[len(dy)//2] is None))
        # print((dy[len(dy)//2+1] > max(dh) or dy[len(dy)//2+1] is None))
        # print((dy[len(dy)//2-1] > max(dh) or dy[len(dy)//2-1] is None))
        # print((dy[len(dy)//2+2] > max(dh) or dy[len(dy)//2+2] is None))
        # print((dy[len(dy)//2-2] > max(dh) or dy[len(dy)//2-2] is None))
        # print((dy[len(dy)//2+2] > max(dh) or dy[len(dy)//2+3] is None))
        # print((dy[len(dy)//2-2] > max(dh) or dy[len(dy)//2-3] is None))
        # print((dy[len(dy)//2+2] > max(dh) or dy[len(dy)//2+4] is None))
        # print((dy[len(dy)//2-2] > max(dh) or dy[len(dy)//2-4] is None))
        # print(self.servo_state)
        if ((dy[len(dy)//2] > max(dh) or dy[len(dy)//2] is None) and 
            (dy[(len(dy)//2)+1] > max(dh) or dy[(len(dy)//2)+1] is None) and 
            (dy[(len(dy)//2)-1] > max(dh) or dy[(len(dy)//2)-1] is None) and 
            (dy[(len(dy)//2)+2] > max(dh) or dy[(len(dy)//2)+2] is None) and 
            (dy[(len(dy)//2)-2] > max(dh) or dy[(len(dy)//2)-2] is None) and
            (dy[(len(dy)//2)+3] > max(dh) or dy[(len(dy)//2)+2] is None) and 
            (dy[(len(dy)//2)-3] > max(dh) or dy[(len(dy)//2)-2] is None) and
            (dy[(len(dy)//2)+4] > max(dh) or dy[(len(dy)//2)+2] is None) and 
            (dy[(len(dy)//2)-4] > max(dh) or dy[(len(dy)//2)-2] is None) and
            self.servo_state == 0):
            # print("----------------------------------------------")
            brush_msg = Int32()
            brush_msg.data = 1
            self.brush_publisher.publish(brush_msg)
            self.servo_state = 1


def main(args=None):
    rclpy.init(args=args)
    node = LidarReadNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
