#!/usr/bin/python3

# from solar_ros.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import SoloPy as solo
import time


class SoloNode(Node):
    def __init__(self):
        super().__init__('solo_node')
        self.initial_solo()
        self.create_timer(0.01, self.solo_loop)
        self.publisher_feedback = self.create_publisher(Float32MultiArray, "/feedback_odr", 10)
        self.publisher_L = self.create_publisher(Float32MultiArray, "/lidar_state", 10)
        self.create_subscription(Float32MultiArray, "/cmd_vel", self.velo_callback, 10)
        self.left_speed = 0
        self.right_speed = 0
        self.count = 0
        self.start = 0
        
    def initial_solo(self):
        pwmFrequency = 40
        numberOfPoles = 8
        currentLimit = 20

        # self.odrv = odrive.find_any()
        self.solo_L, self.solo_R = self.detect_solo_ports()
        # ports = ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2", "/dev/ttyACM3"]
        # self.solo_L = solo.SoloMotorControllerUart(ports[1], 1, solo.UartBaudRate.RATE_115200,autoConnect=1)
        # self.solo_R = solo.SoloMotorControllerUart(ports[0], 0, solo.UartBaudRate.RATE_115200,autoConnect=1)

       
        time.sleep(3)
        # self.solo_R.serial_error_handler()
        print("Trying to Connect To SOLO")
        communication_is_working_L = False
        communication_is_working_R = False
        eror,data = self.solo_L.overwrite_error_register()
        eror,data = self.solo_R.overwrite_error_register()
        while communication_is_working_L is False:
            time.sleep(1)
            communication_is_working_L, error = self.solo_L.communication_is_working()
            print(error)
            eror,data = self.solo_L.overwrite_error_register()
            # self.solo_L.reset_error_register()

        print("Communication Left SOLO succuessfully!")
        while communication_is_working_R is False:
            time.sleep(1)
            communication_is_working_R, error = self.solo_R.communication_is_working()
            print(error)
            eror,data = self.solo_R.overwrite_error_register()
            # self.solo_R.reset_error_register()
        print("Communication Right SOLO succuessfully!")

        # Initial Configurations
        self.solo_L.set_output_pwm_frequency_khz(pwmFrequency)
        self.solo_L.set_current_limit(currentLimit)
        self.solo_L.set_command_mode(solo.CommandMode.DIGITAL)
        self.solo_L.set_motor_type(solo.MotorType.BLDC_PMSM)
        self.solo_L.set_motor_poles_counts(numberOfPoles)
        # self.solo_sensorless(self.solo_L)
        # self.solo_L.set_feedback_control_mode(2)
        self.solo_hal_speed(self.solo_L)
        self.solo_L.set_motion_profile_mode(solo.MotionProfileMode.TIME_OPTIMAL_STCURVE)
        self.solo_L.set_motion_profile_variable1(18)
        self.solo_L.set_motion_profile_variable2(100)
        self.solo_L.motor_parameters_identification(solo.Action.START)
        # self.solo_L.set_speed_acceleration_value(20.0)
        # self.solo_L.set_speed_deceleration_value(1000.0)
        time.sleep(3)

        self.solo_R.set_output_pwm_frequency_khz(pwmFrequency)
        self.solo_R.set_current_limit(currentLimit)
        self.solo_R.set_command_mode(solo.CommandMode.DIGITAL)
        self.solo_R.set_motor_type(solo.MotorType.BLDC_PMSM)
        self.solo_R.set_motor_poles_counts(numberOfPoles)

        # self.solo_sensorless(self.solo_R)
        # self.solo_R.set_feedback_control_mode(2)
        self.solo_hal_speed(self.solo_R)
        self.solo_R.set_motion_profile_mode(solo.MotionProfileMode.TIME_OPTIMAL_STCURVE)
        self.solo_R.set_motion_profile_variable1(18)
        self.solo_R.set_motion_profile_variable2(100)
        self.solo_R.motor_parameters_identification(solo.Action.START)

        # self.solo_R.set_speed_acceleration_value(20.0)
        # self.solo_R.set_speed_deceleration_value(1000.0)
        time.sleep(3)

        print("Finished setup Solo")

    def detect_solo_ports(self):
        ports = ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2", "/dev/ttyACM3","/dev/ttyACM4"]
        solo_L = None
        solo_R = None
        while solo_L == None or solo_R == None :
            for port in ports:
                try:
                    if solo_L == None :
                        solo1 = solo.SoloMotorControllerUart(port, 1, solo.UartBaudRate.RATE_115200)
                        is_working, _ = solo1.communication_is_working()
                        if is_working:
                            print(f"{port} responds to address 1 (Left)")
                            solo_L = solo1
                            continue
                except :
                    print("L not sucess")
                    pass

                try:
                    if solo_R == None :
                        solo2 = solo.SoloMotorControllerUart(port, 0, solo.UartBaudRate.RATE_115200)
                        is_working, _ = solo2.communication_is_working()
                        if is_working:
                            print(f"{port} responds to address 0 (Right)")
                            solo_R = solo2
                            continue
                except :
                    print("R not sucess")
                    pass
            time.sleep(1)
            print("search for port")
        return solo_L, solo_R
    
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
        error_value_L, error_type_L = self.solo_L.get_error_register()
        error_value_R, error_type_R = self.solo_R.get_error_register()

        if error_type_L != solo.Error.NO_ERROR_DETECTED:
            print(f"Left: Failed to get error register: {error_type_L}")
            feedback_L = 0.0
        else:
            if error_value_L != -1:
                # print("Left: No errors detected.")
                feedback_L = 1.0
            else:
                print(f"Left: error_value: {error_value_L:#010x}")
                feedback_L = 0.0

        if error_type_R != solo.Error.NO_ERROR_DETECTED:
            print(f"Right: Failed to get error register: {error_type_R}")
            feedback_R = 0.0
        else:
            if error_value_R != -1:
                # print("Right: No errors detected.")
                feedback_R = 1.0
            else:
                print(f"Right: error_value: {error_value_R:#010x}")
                feedback_R = 0.0

        current_time = self.get_clock().now().nanoseconds / 1e9
        
        if self.count == 0 : 
            self.start = current_time
            self.count += 1

        current_time = current_time - self.start

        feedback_msg = Float32MultiArray()
        print(feedback_L,feedback_R,current_time)
        feedback_msg.data = [feedback_L,feedback_R,current_time]
        self.publisher_feedback.publish(feedback_msg)

        if feedback_L == 1 and feedback_R == 1:
            if self.left_speed > 0:
                self.solo_L.set_motor_direction(solo.Direction.COUNTERCLOCKWISE)
                self.solo_L.set_speed_reference(self.left_speed)
            # elif self.left_speed == 0 :
            #     pass
            else:
                self.solo_L.set_motor_direction(solo.Direction.CLOCKWISE)
                self.solo_L.set_speed_reference(-self.left_speed)

            if self.right_speed > 0:
                self.solo_R.set_motor_direction(solo.Direction.CLOCKWISE)
                self.solo_R.set_speed_reference(self.right_speed)
            # elif self.right_speed == 0 :
            #     pass
            else:
                self.solo_R.set_motor_direction(solo.Direction.COUNTERCLOCKWISE)
                self.solo_R.set_speed_reference(-self.right_speed)
        else:
            self.solo_L.set_speed_reference(0)
            
            self.solo_R.set_speed_reference(0)
            self.right_speed = 0
            self.left_speed = 0
            L_msg = Float32MultiArray()
            L_msg.data = [float(0), float(0), float(0)]
            self.publisher_L.publish(L_msg)

    def velo_callback(self, msg:Float32MultiArray):
        self.left_speed = msg.data[0]/0.02*60
        self.right_speed = msg.data[1]/0.02*60


def main(args=None):
    rclpy.init(args=args)
    node = SoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
