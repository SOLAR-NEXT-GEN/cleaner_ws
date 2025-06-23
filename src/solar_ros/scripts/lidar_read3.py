#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud
import numpy as np
from std_msgs.msg import Float32MultiArray, Int32
from geometry_msgs.msg import Point32


class LidarReadNode(Node):
    def __init__(self):
        super().__init__('lidar_read_node')

        self.lim_pos_publisher = self.create_publisher(LaserScan, '/scan2', 10)
        self.edge_pos_publisher = self.create_publisher(PointCloud, '/edge_pos', 10)
        self.brush_publisher = self.create_publisher(Int32, '/cubemx_publisher_servo', 10)
        self.cmd_vel_publisher = self.create_publisher(Float32MultiArray, '/cmd_vel', 10)

        #subscription
        self.create_subscription(LaserScan, "/scan", self.lidar_callback, 10)
        self.create_subscription(Float32MultiArray, "/lidar_state", self.state_callback, 10)

        self.create_timer(0.01, self.timer_callback)

        #variable
        self.max_speed = 8.0
        self.direction = -1 #ทิศการเดิน
        self.kp = 2.0
        self.stack_pos = []
        self.deg = [270, 90]
        self.servo_state = 0
        self.servo_timecount = 0
        self.side = 0 #เปลี่ยนฝั่ง LiDAR
        self.lidar_error = 0.06
        self.dh = 0.5
        self.isActive = 0

    def timer_callback(self):
        if self.servo_state == 1 and self.servo_timecount < 500:
            self.servo_timecount += 1
        elif self.servo_state == 1:
            self.servo_state = 0
            self.servo_timecount = 0
            brush_msg = Int32()
            brush_msg.data = 0
            # self.brush_publisher.publish(brush_msg)

    def state_callback(self, msg:Float32MultiArray):
        self.isActive = msg.data[0]
        self.max_speed = msg.data[1]
        self.kp = msg.data[2]


    def cmd_pub(self, distance):
        msg = Float32MultiArray()

        #if have solarcell below
        if abs(distance[0]) >= 0.005 and abs(distance[1]) >= 0.005:

            # check if solarcell shift left or right
            error = distance[0] + distance[1]
            if abs(error) < 0.1:
                # if solarcell in the middle
                v = [self.max_speed, self.max_speed]
            else:
                v = [self.max_speed-(self.kp * error), self.max_speed+(self.kp * error)]

            #check limit real max speed = 1.5 * max_speed
            if abs(v[0]) > self.max_speed*1.5:
                v[0] = self.max_speed * 1.5 * (v[0]/abs(v[0]))
            if abs(v[1]) > self.max_speed*1.5:
                v[1] = self.max_speed * 1.5 * (v[1]/abs(v[1]))
        else:
            v = [self.max_speed, self.max_speed]

        msg.data = [v[0]*self.direction, v[1]*self.direction]
        self.cmd_vel_publisher.publish(msg)
        self.get_logger().info(f'Publishing speed data =  {msg.data}')


    def publish_lidar_data(self, msg:LaserScan, deg):
        scan = msg
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.angle_min = np.deg2rad(deg[0])
        scan.angle_max = np.deg2rad(deg[1])
        scan.ranges = self.lim_deg
        self.lim_pos_publisher.publish(scan)

    def publish_pose(self, pose):
        msg = PointCloud()
        msg.header.frame_id = "laser"
        msg.header.stamp = self.get_clock().now().to_msg()

        for i in pose:
            temp = Point32()
            temp.x = i[0]
            temp.y = i[1]
            temp.z = 0.0
            msg.points.append(temp)

        self.edge_pos_publisher.publish(msg)

    def cal_something(self,msg:LaserScan):
        self.stack_pos = []
        self.dx = []
        self.dy = []
        if self.side == 1:
            for i in range(len(msg)):
                self.stack_pos.insert(0, round(msg[i],2))
                self.dx.insert(0, round(np.sin(np.deg2rad(i*0.5))*msg[i],4))
                self.dy.insert(0, round(np.cos(np.deg2rad(i*0.5))*msg[i],4))
        else:
            for i in range(len(msg)):
                self.stack_pos.append(round(msg[i],2))
                self.dx.append(round(np.sin(np.deg2rad(i*0.5))*msg[i],4))
                self.dy.append(round(np.cos(np.deg2rad(i*0.5))*msg[i],4))
        
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

    def distance_check(self, dy, dx):
        distance = [0,0]
        range_left = [-np.inf, np.inf]
        range_right = [-np.inf, np.inf]
        prev_pos = [None, None]
        diff = [0,0]

        for i in range(len(dy)//2):
            left = len(dy)//2 -1 - i
            right = len(dy)//2 + i

            if dy[left] != np.inf and dy[left] != -np.inf:
                if min(range_left) < dy[left] < max(range_left):
                    if prev_pos[0] != None:
                        diff[0] = (dy[left] - dy[prev_pos[0]])/abs(prev_pos[0]-left)
                        # if diff_l >= 0:
                        #     range_left[0] = dy[left] - self.lidar_error
                        #     range_left[1] = dy[left] + diff_l + self.lidar_error
                        # else:
                        #     range_left[0] = dy[left] + diff_l - self.lidar_error
                        #     range_left[1] = dy[left] + self.lidar_error
                        range_left[0] = dy[left] + diff[0] - self.lidar_error
                        range_left[1] = dy[left] + diff[0] + self.lidar_error
                        if range_left[1] == -np.inf:
                            print(dy[left], diff[0], self.lidar_error)
                else:
                    if -0.01 <= distance[0] <= 0.01:
                        distance[0] = prev_pos[0]
                prev_pos[0] = left
            else:
                range_left[0] += diff[0]
                range_left[1] += diff[0]

            if dy[right] != np.inf and dy[right] != -np.inf:
                if min(range_right) < dy[right] < max(range_right):
                    if prev_pos[1] != None:
                        diff[1] = (dy[right] - dy[prev_pos[1]])/abs(prev_pos[1]-right)
                        range_right[0] = dy[right] + diff[1] - self.lidar_error
                        range_right[1] = dy[right] + diff[1] + self.lidar_error
                else:
                    if -0.01 <= distance[1] <= 0.01:
                        distance[1] = prev_pos[1]
                prev_pos[1] = right
            else:
                range_right[0] += diff[1]
                range_right[1] += diff[1]

            if abs(distance[0]) > 0.01 and abs(distance[1]) > 0.01:
                break
        self.publish_pose([[dy[distance[0]], dx[distance[0]]], [dy[distance[1]], dx[distance[1]]]])
        self.get_logger().info(f'distance data =  {dx[distance[0]], dx[distance[1]]}')
        return [dx[distance[0]], dx[distance[1]]]
            
    def lidar_callback(self, msg:LaserScan):
        range = msg.ranges
        self.cal_something(range)
        self.lim_deg_pos(self.deg[0],self.deg[1])
        d = self.distance_check(self.lim_dy, self.lim_dx)
        if self.isActive != 0:
            self.cmd_pub(d)
        self.publish_lidar_data(msg, self.deg)
        self.brush_check(self.lim_dy, self.dh)
    
    def brush_check(self, dy, dh):
        if ((dy[len(dy)//2] > dh or dy[len(dy)//2] is None) and 
            (dy[(len(dy)//2)+1] > dh or dy[(len(dy)//2)+1] is None) and 
            (dy[(len(dy)//2)-1] > dh or dy[(len(dy)//2)-1] is None) and 
            (dy[(len(dy)//2)+2] > dh or dy[(len(dy)//2)+2] is None) and 
            (dy[(len(dy)//2)-2] > dh or dy[(len(dy)//2)-2] is None) and
            (dy[(len(dy)//2)+3] > dh or dy[(len(dy)//2)+3] is None) and 
            (dy[(len(dy)//2)-3] > dh or dy[(len(dy)//2)-3] is None) and
            (dy[(len(dy)//2)+4] > dh or dy[(len(dy)//2)+4] is None) and 
            (dy[(len(dy)//2)-4] > dh or dy[(len(dy)//2)-4] is None) and
            self.servo_state == 0):
            print("----------------------------------------------")
            brush_msg = Int32()
            brush_msg.data = 1
            # self.brush_publisher.publish(brush_msg)
            self.servo_state = 1


def main(args=None):
    rclpy.init(args=args)
    node = LidarReadNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()