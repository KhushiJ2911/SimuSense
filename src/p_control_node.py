#!/usr/bin/env python

# ROS Python Node for Proportional (P) Control (The Control Module)
#
# Logic:
# 1. Subscribes to the error values published by the perception node.
# 2. Implements a Proportional (P) control law to calculate necessary linear (x) and angular (z) velocities.
# 3. Publishes the calculated velocities as a geometry_msgs/Twist message to the robot's command topic.
#
# STATUS: P-CONTROLLER LOGIC BEING FINALIZED

import rospy
from geometry_msgs.msg import Twist
# from custom_msgs.msg import TargetError # Custom message to be defined
# import rosparam # For loading configuration gains

class PControlNode:
    def __init__(self):
        rospy.init_node('p_control_node', anonymous=True)
        # self.error_sub = rospy.Subscriber("/object_detection/error", TargetError, self.error_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Load P-Gains from config/p_gains.yaml (Will be implemented later)
        self.KP_LINEAR = 0.005 # Placeholder value
        self.KP_ANGULAR = 0.003 # Placeholder value
        rospy.loginfo("P-Control Node Initialized. Ready for errors...")

    def error_callback(self, error_msg):
        twist = Twist()
        
        # --- Placeholder for P-Control Calculation ---
        # 1. Calculate Linear Velocity (Forward/Backward movement)
        # twist.linear.x = self.KP_LINEAR * error_msg.dist_error
        
        # 2. Calculate Angular Velocity (Turning left/right)
        # twist.angular.z = self.KP_ANGULAR * error_msg.x_error
        
        # 3. Clamp speeds for safety (optional but good practice)
        # ----------------------------------------------------

        self.cmd_vel_pub.publish(twist)

if __name__ == '__main__':
    try:
        PControlNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
