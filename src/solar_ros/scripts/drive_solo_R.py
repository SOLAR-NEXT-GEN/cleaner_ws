#!/usr/bin/python3

# from solar_ros.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray,Int32

import SoloPy as solo
import time


class SoloRNode(Node):
    def __init__(self):
        super().__init__('solo_node_R')
        self.initial_solo()
        self.create_timer(0.01, self.solo_loop)
        self.create_subscription(Int32,"/stop",self.stop_callback,10)
        self.publisher_feedback = self.create_publisher(Float32MultiArray, "/feedback_odr_R", 10)
        self.publisher_L = self.create_publisher(Float32MultiArray, "/lidar_state", 10)
        self.create_subscription(Float32MultiArray, "/cmd_vel", self.velo_callback, 10)
        self.create_subscription(Float32MultiArray, "/feedback_odr_L", self.feedback_callback, 10)
        self.right_speed = 0
        
        self.count = 0
        self.start = 0
        self.feedback_L = 0.0
        self.feedback_R = 0.0
        self.stop = 0
        
    def initial_solo(self):
        pwmFrequency = 40
        numberOfPoles = 8
        currentLimit = 20

        # self.odrv = odrive.find_any()
        self.solo_R = self.detect_solo_ports()
        # ports = ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2", "/dev/ttyACM3"]
        # self.solo_L = solo.SoloMotorControllerUart(ports[1], 1, solo.UartBaudRate.RATE_115200,autoConnect=1)
        # self.solo_R = solo.SoloMotorControllerUart(ports[0], 0, solo.UartBaudRate.RATE_115200,autoConnect=1)

       
        time.sleep(3)
        # self.solo_R.serial_error_handler()
        print("Trying to Connect To SOLO")
        communication_is_working_R = False
        eror,data = self.solo_R.overwrite_error_register()
        while communication_is_working_R is False:
            time.sleep(1)
            communication_is_working_R, error = self.solo_R.communication_is_working()
            print(error)
            eror,data = self.solo_R.overwrite_error_register()
            # self.solo_L.reset_error_register()

        print("Communication Right SOLO succuessfully!")

        # Initial Configurations
        self.solo_R.set_output_pwm_frequency_khz(pwmFrequency)
        self.solo_R.set_current_limit(currentLimit)
        self.solo_R.set_command_mode(solo.CommandMode.DIGITAL)
        self.solo_R.set_motor_type(solo.MotorType.BLDC_PMSM)
        self.solo_R.set_motor_poles_counts(numberOfPoles)
        # self.solo_sensorless(self.solo_R)
        # self.solo_R.set_feedback_control_mode(2)
        self.solo_hal_speed(self.solo_R)
        self.solo_R.set_motion_profile_mode(solo.MotionProfileMode.STEP_RAMP_RESPNSE)
        # self.solo_R.set_motion_profile_variable1(18)
        # self.solo_R.set_motion_profile_variable2(100)
        self.solo_R.motor_parameters_identification(solo.Action.START)
        # self.solo_R.set_speed_acceleration_value(20.0)
        # self.solo_R.set_speed_deceleration_value(1000.0)
        time.sleep(3)

        print("Finished setup Solo")

    def detect_solo_ports(self):
        ports = ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2", "/dev/ttyACM3","/dev/ttyACM4"]
        solo_R = None
        while solo_R == None:
            for port in ports:
                try:
                    if solo_R == None :
                        solo1 = solo.SoloMotorControllerUart(port, 0, solo.UartBaudRate.RATE_115200)
                        is_working, _ = solo1.communication_is_working()
                        if is_working:
                            print(f"{port} responds to address 0 (Right)")
                            solo_R = solo1
                            continue
                except :
                    print("R not sucess")
                    pass
            time.sleep(1)
            print("search for port")
        return solo_R
    
    def solo_hal_speed(self, solo_drive):
        speedControllerKp = 0.04
        speedControllerKi = 0.006
        solo_drive.set_feedback_control_mode(solo.FeedbackControlMode.HALL_SENSORS)
        solo_drive.set_speed_controller_kp(speedControllerKp)
        solo_drive.set_speed_controller_ki(speedControllerKi)
        solo_drive.set_control_mode(solo.ControlMode.SPEED_MODE)


    def solo_sensorless(self, solo_drive):
        speedControllerKp = 0.04
        speedControllerKi = 0.006
        solo_drive.set_feedback_control_mode(solo.FeedbackControlMode.SENSORLESS_HSO)
        solo_drive.set_control_mode(solo.ControlMode.SPEED_MODE)
        solo_drive.set_speed_controller_kp(speedControllerKp)
        solo_drive.set_speed_controller_ki(speedControllerKi)
        solo_drive.set_sensorless_transition_speed(500)
        
    def solo_loop(self):
        error_value_R, error_type_R = self.solo_R.get_error_register()

        if error_type_R != solo.Error.NO_ERROR_DETECTED:
            print(f"Right: Failed to get error register: {error_type_R}")
            self.feedback_R = 0.0
        else:
            if error_value_R != -1:
                # print("Left: No errors detected.")
                self.feedback_R = 1.0
            else:
                print(f"Left: error_value: {error_value_R:#010x}")
                self.feedback_R = 0.0

        current_time = self.get_clock().now().nanoseconds / 1e9
        
        if self.count == 0 :
            self.start = current_time
            self.count += 1

        current_time = current_time - self.start

        feedback_msg = Float32MultiArray()
        print(self.feedback_L, self.feedback_R,current_time)
        feedback_msg.data = [self.feedback_R,current_time]
        self.publisher_feedback.publish(feedback_msg)

    def feedback_callback(self, msg:Float32MultiArray):
        self.feedback_L = msg.data[0]

        if self.feedback_L == 0.0 or self.feedback_R == 0.0:
            self.solo_R.set_speed_reference(0)
            self.right_speed = 0
            # L_msg = Float32MultiArray()
            # L_msg.data = [float(0), float(0), float(0)]
            # self.publisher_L.publish(L_msg)

    def stop_callback(self,msg:Int32):
        self.stop = msg.data
        print(self.stop)

    def velo_callback(self, msg:Float32MultiArray):
        if self.stop == 0:
            self.solo_R.set_speed_reference(0)
            print(f"set speed to {0}")
            return
        self.right_speed = msg.data[1]/0.02*60
        if self.feedback_L == 1.0 and self.feedback_R == 1.0:
            if self.right_speed > 0:
                self.solo_R.set_motor_direction(solo.Direction.CLOCKWISE)
                self.solo_R.set_speed_reference(self.right_speed)
                print(f"set speed to {self.right_speed}")
            else:
                self.solo_R.set_motor_direction(solo.Direction.COUNTERCLOCKWISE)
                self.solo_R.set_speed_reference(-self.right_speed)

def main(args=None):
    rclpy.init(args=args)
    node = SoloRNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
