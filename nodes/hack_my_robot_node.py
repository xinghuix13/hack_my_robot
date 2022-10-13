#!/usr/bin/env python

import rospy

from hack_my_robot.hack_my_robot_routine import smartCommonClass

# Define a main function and then call it
def main():
    # Initialize node and name it
    rospy.init_node("hack_my_robot")
    rospy.loginfo("Hack My Robot main routine")
    # Go to class functions that do all the heavy lifting
    try:
        smartCommonClass()
    except rospy.ROSInterruptException:
        pass
    # Allow ROS to go to all callbacks
    rospy.spin()

if __name__ == '__main__':
    main()