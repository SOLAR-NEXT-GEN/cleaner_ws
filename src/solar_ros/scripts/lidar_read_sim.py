#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
from std_msgs.msg import Float64MultiArray


class LidarReadNode(Node):
    def __init__(self):
        super().__init__('lidar_read_node')

        self.lim_pos_publisher = self.create_publisher(LaserScan, '/scan2', 10)
        self.cmd_vel_publisher = self.create_publisher(Float64MultiArray, '/velocity_controllers/commands', 10)

        #subscription
        self.create_subscription(LaserScan, "/lidar/out", self.lidar_callback, 10)

        #variable
        self.max_speed = 65.0
        self.kp = [65.0, 2210.0]
        self.stack_pos = []
        self.deg = [270, 90]
        self.dh = [0.25, 0.43]
        self.prev_distance = [0,0]
        self.side = 'none'

    def cmd_pub(self, distance):
        msg = Float64MultiArray()
        #for change direction
        a = -1

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
        msg.data = [a*x/10 for x in [v[0], v[0], -v[1], -v[1]]]
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

    def distance_check(self, dy, dx, dh):
        distance = [0,0]
        range_left = [dh[0], dh[1]]
        range_right = [dh[0], dh[1]]

        for i in range(len(dy)//2):
            left = len(dy)//2 -1 - i
            right = len(dy)//2 + i

            if min(range_left) < dy[left] < max(range_left):
                if min(range_left) == range_left[0]:
                    range_left[1] = dy[left] + 0.005
                else:
                    range_left[1] = dy[left] - 0.005
            else:
                if distance[0] == 0:
                    distance[0] = dx[left]

            if min(range_right) < dy[right] < max(range_right):
                if min(range_right) == range_right[1]:
                    range_right[0] = dy[right] + 0.005
                else:
                    range_right[0] = dy[right] - 0.005
            else:
                if distance[1] == 0:
                    distance[1] = dx[right]
            if distance[0] != 0 and distance[1] != 0:
                break
        self.get_logger().info(f'distance data =  {distance}')
        return distance
            
    def lidar_callback(self, msg:LaserScan):
        range = msg.ranges
        self.cal_something(range)
        self.lim_deg_pos(self.deg[0],self.deg[1])
        d = self.distance_check(self.lim_dy, self.lim_dx, self.dh)
        self.cmd_pub(d)
        self.publish_lidar_data(msg, self.deg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarReadNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
