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

        self.f_lim_pos_publisher = self.create_publisher(LaserScan, '/front/scan2', 10)
        self.f_edge_pos_publisher = self.create_publisher(PointCloud, '/front/edge_pos', 10)
        self.b_lim_pos_publisher = self.create_publisher(LaserScan, '/back/scan2', 10)
        self.b_edge_pos_publisher = self.create_publisher(PointCloud, '/back/edge_pos', 10)
        self.brush_publisher = self.create_publisher(Int32, '/cubemx_publisher_servo', 10)
        self.cmd_vel_publisher = self.create_publisher(Float32MultiArray, '/cmd_vel', 10)
        self.brush1_publisher = self.create_publisher(Int32, '/cubemx_sub_encoder1', 10)
        self.brush2_publisher = self.create_publisher(Int32, '/cubemx_sub_encoder2', 10)

        #subscription
        self.create_subscription(LaserScan, "/front/scan", self.front_lidar_callback, 10)
        self.create_subscription(LaserScan, "/back/scan", self.back_lidar_callback, 10)
        self.create_subscription(Float32MultiArray, "/lidar_state", self.state_callback, 10)
        self.create_subscription(Int32, "/cubemx_publisher_encoder1", self.encoder1_callback, 10)
        self.create_subscription(Int32, "/cubemx_publisher_encoder2", self.encoder2_callback, 10)

        self.create_timer(0.01, self.timer_callback)

        #variable
        self.lidar_distance = 0.32 #ระยะห่างระหว่าง LiDAR
        self.max_speed = 0.2
        self.direction = 1 #ทิศการเดิน
        self.deg = [270, 90]
        self.lidar_error = 0.032
        self.isActive = 0
        self.f_mid_pose = 0
        self.b_mid_pose = 0
        self.look_ahead = 1.0
        self.R = 0.05
        self.L = 1.59
        self.prev_vl = 0.0
        self.prev_vr = 0.0
        self.lim = 0.01
        self.start_position = [0, 0]
        self.f_dy = [0, 0]
        self.b_dy = [0, 0]
        self.current_encoder = [0, 0]

    def timer_callback(self):
        if self.isActive:
            self.cmd_pub()

    def state_callback(self, msg:Float32MultiArray):
        self.isActive = msg.data[0]
        self.max_speed = abs(msg.data[1])
        self.direction = msg.data[1]
        self.look_ahead = msg.data[2]
        # if msg.data[0] == 0:
        #     cmd_msg = Float32MultiArray()
        #     cmd_msg.data = [0.0, 0.0]
        #     self.cmd_vel_publisher.publish(cmd_msg)

    def encoder1_callback(self, msg:Int32):
        self.current_encoder[0] = msg.data

    def encoder2_callback(self, msg:Int32):
        self.current_encoder[2] = msg.data

    def pub_brush_target(self):
        # publish brush target
        target_position = [min(self.f_dy[0], self.b_dy[0]), min(self.f_dy[1], self.b_dy[1])]
        brush_msg1 = Int32()
        brush_msg1.data = target_position[0] - self.start_position[0]
        brush_msg2 = Int32()
        brush_msg2.data = target_position[1] - self.start_position[1]
        self.brush1_publisher.publish(brush_msg1)
        self.brush2_publisher.publish(brush_msg2)

    def publish_lidar_data(self, msg:LaserScan, deg, lim_deg, side):
        scan = msg
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.angle_min = np.deg2rad(deg[0])
        scan.angle_max = np.deg2rad(deg[1])
        scan.ranges = lim_deg

        if side == "front":
            self.f_lim_pos_publisher.publish(scan)
        elif side == "back":
            self.b_lim_pos_publisher.publish(scan)

    def publish_pose(self, pose, side):
        msg = PointCloud()
        msg.header.frame_id = "laser"
        msg.header.stamp = self.get_clock().now().to_msg()

        for i in pose:
            temp = Point32()
            temp.x = i[0]
            temp.y = i[1]
            temp.z = 0.0
            msg.points.append(temp)

        if side == "front":
            self.f_edge_pos_publisher.publish(msg)
        elif side == "back":
            self.b_edge_pos_publisher.publish(msg)

    def stack_data(self,msg:list):
        stack_pos = []
        dx = []
        dy = []
        for i in range(len(msg)):
            stack_pos.append(round(msg[i],5))
            dx.append(round(np.sin(np.deg2rad(i*0.5))*msg[i],5))
            dy.append(round(np.cos(np.deg2rad(i*0.5))*msg[i],5))

        return stack_pos, dx, dy
        
        
    def lim_deg_pos(self, first, last, stack_pos, dx, dy):
        lim_deg = []
        lim_dx = []
        lim_dy = []
        if first < last:
            lim_deg = stack_pos[first*2:last*2]
            lim_dx = dx[first*2:last*2]
            lim_dy = dy[first*2:last*2]
        else:
            lim_deg = stack_pos[first*2:720]
            lim_deg = lim_deg[:] + stack_pos[0:last*2]
            lim_dx = dx[first*2:720]
            lim_dx = lim_dx[:] + dx[0:last*2]
            lim_dy = dy[first*2:720]
            lim_dy = lim_dy[:] + dy[0:last*2]

        return lim_deg, lim_dx, lim_dy

    def distance_check(self, dy, dx):
        distance = [len(dy)//2, len(dy)//2]
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
                        #     range_left[1] = dy[left] + diff01_l + self.lidar_error
                        # else:
                        #     range_left[0] = dy[left] + diff_l - self.lidar_error
                        #     range_left[1] = dy[left] + self.lidar_error
                        range_left[0] = dy[left] + diff[0] - self.lidar_error
                        range_left[1] = dy[left] + diff[0] + self.lidar_error
                        if range_left[1] == -np.inf:
                            print(dy[left], diff[0], self.lidar_error)
                else:
                    if -0.01 <= dx[distance[0]] <= 0.01:
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
                    if -0.01 <= dx[distance[1]] <= 0.01:
                        distance[1] = prev_pos[1]
                prev_pos[1] = right
            else:
                range_right[0] += diff[1]
                range_right[1] += diff[1]

            if abs(dx[distance[0]]) > 0.01 and abs(dx[distance[1]]) > 0.01:
                break
        # self.publish_pose([[dy[distance[0]], dx[distance[0]]], [dy[distance[1]], dx[distance[1]]]])
        # self.get_logger().info(f'distance data =  {dx[distance[0]], dx[distance[1]]}')
        return [dx[distance[0]], dx[distance[1]]], [dy[distance[0]], dy[distance[1]]]
            
    def front_lidar_callback(self, msg:LaserScan):
        range = msg.ranges
        stack, dx, dy = self.stack_data(range)
        lim_raw, lim_dx, lim_dy = self.lim_deg_pos(self.deg[0],self.deg[1], stack, dx, dy)
        dx, dy = self.distance_check(lim_dy, lim_dx)
        if abs(dx[0]) > 0.005 and abs(dx[1]) > 0.005 and abs(dx[0]) != np.inf and abs(dx[1]) != np.inf and 0.9 < np.sqrt(((dx[0]-dx[1])**2)+((dy[0]-dy[1])**2)) < 1.1:
            self.f_mid_pose = (dx[0] + dx[1])/2 
            self.f_dy = dy
        self.publish_pose([[dy[0], dx[0]], [dy[1], dx[1]]], "front")
        self.publish_lidar_data(msg, self.deg, lim_raw, "front")

    def back_lidar_callback(self, msg:LaserScan):
        range = msg.ranges
        stack, dx, dy = self.stack_data(range)
        dx = [x*-1 for x in dx]
        lim_raw, lim_dx, lim_dy = self.lim_deg_pos(self.deg[0],self.deg[1], stack[::-1], dx[::-1], dy[::-1])
        dx, dy = self.distance_check(lim_dy, lim_dx)
        if abs(dx[0]) > 0.005 and abs(dx[1]) > 0.005 and abs(dx[0]) != np.inf and abs(dx[1]) != np.inf and 0.9 < np.sqrt(((dx[0]-dx[1])**2)+((dy[0]-dy[1])**2)) < 1.1 :
            self.b_mid_pose = (dx[0] + dx[1])/2
            self.b_dy = dy
        self.publish_pose([[dy[0], dx[0]], [dy[1], dx[1]]], "back")
        self.publish_lidar_data(msg, self.deg, lim_raw, "back")

    def cmd_pub(self):
        if self.direction >= 0:
            robot_x = (self.f_mid_pose + self.b_mid_pose)/2
            robot_theta = np.arcsin((self.f_mid_pose-self.b_mid_pose)/self.lidar_distance)
        elif self.direction < 0:
            robot_x = -(self.f_mid_pose + self.b_mid_pose)/2
            robot_theta = np.arcsin((self.f_mid_pose-self.b_mid_pose)/self.lidar_distance)
        print(robot_x, robot_theta)

        target_x = 0
        target_y = self.look_ahead

        dx = target_x - robot_x
        dy = target_y
        # In robot frame: Y forward, X right
        target_x_r = -np.cos(robot_theta) * dx + np.sin(robot_theta) * dy
        target_y_r = np.sin(robot_theta) * dx + np.cos(robot_theta) * dy

        target_dist = np.sqrt(target_x_r**2 + target_y_r**2)
        curvature = 0.0
        if target_dist > 1e-6:  # avoid division by zero
            curvature = 2 * target_x_r / (target_dist * target_dist)
        
        # Determine angular velocity (for differential drive, angular = curvature * linear velocity)
        if curvature > 1.0:
            curvature = 1.0
        elif curvature < -1.0:
            curvature = -1.0
        angular_z = self.max_speed * curvature

        print(angular_z)

        msg = Float32MultiArray()
        v_r = self.max_speed + (angular_z * self.L / 2.0)
        v_l = self.max_speed - (angular_z * self.L / 2.0)

        diff_r = v_r - self.prev_vr
        diff_r = np.clip(diff_r, -self.lim, self.lim)
        v_r = self.prev_vr + diff_r

        diff_l = v_l - self.prev_vl
        diff_l = np.clip(diff_l, -self.lim, self.lim)
        v_l = self.prev_vl + diff_l

        self.prev_vr = v_r
        self.prev_vl = v_l

        if self.direction >= 0:
            msg.data = [v_l, v_r]
        elif self.direction < 0:
            msg.data = [v_r*-1, v_l*-1]
        self.cmd_vel_publisher.publish(msg)
        self.get_logger().info(f'Publishing speed data =  {msg.data}')
        # self.pub_brush_target()


def main(args=None):
    rclpy.init(args=args)
    node = LidarReadNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()