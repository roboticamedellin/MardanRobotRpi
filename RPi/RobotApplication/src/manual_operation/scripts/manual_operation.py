#!/usr/bin/python

import rospy
from geometry_msgs.msg import Twist

class MotorController:
    def __init__(self):
        self.pub = rospy.Publisher('motors/motor_twist', Twist, queue_size=1000)
        self.sub = rospy.Subscriber("joy_driver/joy", Twist, self.callback)

    
    def callback(self, twist):
        self.pub.publish(twist)


def main():
    rospy.init_node('manual_operation')
    obc = MotorController()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()
