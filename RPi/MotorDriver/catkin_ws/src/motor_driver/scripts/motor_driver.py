#!/usr/bin/python

import rospy
from geometry_msgs.msg import Twist

import zumo_lib as motors
import threading        

class I2CMotorController: 

    def __init__(self):
        motors.init_i2c_bus()

    def send_speed(self, speed_l, speed_r, log = False):
        motors.send_speed(speed_l, speed_r)
        if log:
            rospy.loginfo("Speed Left: %d, Speed Right: %d", speed_l, speed_r)

class MotorController:
    def __init__(self, motor_interface):
        self.sub = rospy.Subscriber("motors/motor_twist", Twist, self.data_processing_callback)
        self.vel_max = 200
        self.l_val = 0
        self.r_val = 0
        self.rate = rospy.Rate(10)
        self.thread = threading.Thread(target=self.i2c_thread)
        self.mutex = threading.Lock()
        self.motor_interface = motor_interface

    def data_processing_callback(self, twist):
        with self.mutex:
            self.l_val = int((twist.linear.x - twist.angular.z)*self.vel_max/2)
            self.r_val = int((twist.linear.x + twist.angular.z)*self.vel_max/2)

    def i2c_thread(self):
        l_val = 0
        r_val = 0
        while not rospy.is_shutdown():
            with self.mutex:
                l_val = self.l_val
                r_val = self.r_val
            self.motor_interface.send_speed(l_val, r_val)
            self.rate.sleep()

    def star_thread(self):
        self.thread.start()

def main():
    i2c_motor_controller = I2CMotorController()
    rospy.init_node('motor_driver')
    motor_controller = MotorController(i2c_motor_controller)
    motor_controller.star_thread()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()
    