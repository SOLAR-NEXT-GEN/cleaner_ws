#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray,Int32
import paho.mqtt.client as mqtt
import os
import subprocess

# Define the MQTT settings
BROKER = "broker.emqx.io"
PORT = 1883
CLIENT_ID = "ros2-mqtt-bridge"
TOPIC_SUBSCRIBE = "test/topic/solar/fibo"
TOPIC_PUB = "test/topic/solar/fibo_feedback"
MQTT_USERNAME = ""  # Replace with actual username
MQTT_PASSWORD = ""  # Replace with actual password

class mqtt2ros(Node):
    def __init__(self):
        super().__init__("mqtt2ros")
        self.publisher_ = self.create_publisher(Float32MultiArray, "/cmd_vel", 10)
        self.publisher_brush_spin = self.create_publisher(Int32, "/brush_spin", 10)
        self.publisher_L_encoder = self.create_publisher(Int32, "/L_encoder", 10)
        self.publisher_R_encoder = self.create_publisher(Int32, "/R_encoder", 10)
        self.publisher_water_pump= self.create_publisher(Int32, "/water_pump", 10)
        self.publisher_lidar = self.create_publisher(Float32MultiArray, "/lidar_state", 10)
        self.publisher_mode = self.create_publisher(Int32,"/mode",10)
        # self.publisher_Stop = self.create_publisher(Int32,"/stop",10)
        self.get_logger().info("ROS 2 Node initialized, setting up MQTT client...")
        
        self.create_subscription(Float32MultiArray, "/feedback_odr", self.feedback_odr_callback, 10)
        msg = Int32()
        msg.data = 0
        self.publisher_Stop.publish(msg=msg)

        # Initialize MQTT client
        self.mqtt_client = mqtt.Client(client_id=CLIENT_ID)
        self.mqtt_client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message

        # Connect to the MQTT broker.+++++++++++++++++++++++++++++++++.0
        try:
            self.mqtt_client.connect(BROKER, PORT, 60)
            self.get_logger().info(f"Connected to MQTT broker at {BROKER}:{PORT}")
            self.mqtt_client.loop_start()  # Start MQTT loop in a separate thread
        except Exception as e:
            self.get_logger().error(f"Failed to connect to MQTT broker: {e}")

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info(f"Connected to MQTT broker, subscribing to '{TOPIC_SUBSCRIBE}'")
            self.mqtt_client.subscribe(TOPIC_SUBSCRIBE)
        else:
            self.get_logger().error(f"Failed to connect to MQTT broker, return code {rc}")

    def on_message(self, client, userdata, msg):
        mqtt_message = msg.payload.decode()
        self.get_logger().info(f"Received MQTT message: '{mqtt_message}' on topic '{msg.topic}'")

        # Publish the message to the ROS 2 topic
        split_msg = mqtt_message.split()

        if int(split_msg[0]) == 9 : # Stop

            msg = Int32()
            msg.data = 0
            self.publisher_mode.publish(msg)

            ros_message = Float32MultiArray()
            ros_message.data = [float(0), float(0)]
            self.publisher_.publish(ros_message)

            brush_spin_msg = Int32()
            brush_spin_msg.data = int(0)
            self.publisher_brush_spin.publish(brush_spin_msg)

            lidar_msg = Float32MultiArray()
            lidar_msg.data = [float(0), float(0), float(0)]
            self.publisher_lidar.publish(lidar_msg)
            

        elif int(split_msg[0]) == 0 : # Auto

            msg = Int32()
            msg.data = 1
            self.publisher_mode.publish(msg=msg)

            lidar_msg = Float32MultiArray()
            lidar_msg.data = [float(1), float(split_msg[1]), float(split_msg[2])]
            self.publisher_lidar.publish(lidar_msg)


        elif int(split_msg[0]) == 1 : # Manual

            msg = Int32()
            msg.data = 1
            self.publisher_mode.publish(msg=msg)
            
            lidar_msg = Float32MultiArray()
            lidar_msg.data = [float(0), float(0), float(0)]
            self.publisher_lidar.publish(lidar_msg)

            ros_message = Float32MultiArray()
            ros_message.data = [float(split_msg[1]), float(split_msg[2])]

            brush_spin_msg = Int32()
            brush_spin_msg.data = int(split_msg[3])
            self.publisher_brush_spin.publish(brush_spin_msg)

            L_encoder_msg = Int32()
            L_encoder_msg.data = int(split_msg[4])
            self.publisher_L_encoder.publish(L_encoder_msg)

            R_encoder_msg = Int32()
            R_encoder_msg.data = int(split_msg[5])
            self.publisher_R_encoder.publish(R_encoder_msg)

            water_pump_msg = Int32()
            water_pump_msg.data = int(split_msg[6])
            self.publisher_water_pump.publish(water_pump_msg)
            
            self.publisher_.publish(ros_message)
            self.get_logger().info(f"Republished to ROS topic: 'ros_topic'")

        elif int(split_msg[0]) == 2 : #### Reset 
            self.get_logger().info("Reset command received. Closing other terminals and launching drive.sh.")
            self.close_other_terminals()
            self.launch_new_terminal()

    def feedback_odr_callback(self,msg:Float32MultiArray):
        # print(msg.data)
        time = float(msg.data[2])
        formatted_time = round(time, 2)
        temp = str(int(msg.data[0])) + " " + str(int(msg.data[1])) + " "+ str(formatted_time)
        self.mqtt_client.publish(TOPIC_PUB, temp)
    
    def close_other_terminals(self):
        try:
            # Get the window ID of the current terminal
            current_window_id = subprocess.check_output(['xdotool', 'getactivewindow']).strip()

            # Get a list of all terminal window IDs (assuming gnome-terminal)
            terminal_window_ids = subprocess.check_output(['xdotool', 'search', '--class', 'gnome-terminal']).split()

            for window_id in terminal_window_ids:
                if window_id != current_window_id:
                    # Close the terminal window
                    subprocess.call(['xdotool', 'windowclose', window_id])
                    print(f"Closed terminal window ID: {window_id.decode()}")
        except Exception as e:
            print(f"Error closing terminal windows: {e}")

    def launch_new_terminal(self):
        try:
            subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', '/home/solar/drive.sh; exec bash'])
            print("Launched new terminal with drive.sh.")
        except Exception as e:
            print(f"Failed to launch drive.sh: {e}")
    
def main(args=None):
    rclpy.init(args=args)
    node = mqtt2ros()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node.")
    finally:
        node.mqtt_client.loop_stop()  # Stop the MQTT loop
        node.mqtt_client.disconnect()  # Disconnect from MQTT broker
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
