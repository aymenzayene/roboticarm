#!/usr/bin/env python3
import rospy
from interbotix_xs_modules.arm import InterbotixManipulatorXS
import numpy as np
from cv_basics.srv import Centroid, CentroidResponse
from time import sleep, time

# Global variable for the robot
bot = None
last_processed_time = 0
processing_interval = 5  # in seconds

def clamp(value, min_value, max_value, joint_name):
    if value < min_value:
        rospy.loginfo(f"Joint {joint_name} value {value} is below the minimum limit of {min_value}. Clamping to minimum.")
        return min_value
    elif value > max_value:
        rospy.loginfo(f"Joint {joint_name} value {value} is above the maximum limit of {max_value}. Clamping to maximum.")
        return max_value
    return value

def perform_pick_and_place(cx, cy):
    # Calculate dynamic waist position with centroid coordinate and clamp it within limits
   #dynamic_waist = -0.79288521 + 0.00244743 * cx - 0.00032535 * cy
    dynamic_waist = -1.208291239413795 + 0.00258455697229715266 * cx + 0.00129699620668963693 * cy
    
    bot.arm.go_to_home_pose()
    
    sleep(2.0)
    bot.arm.set_ee_pose_components(x=0.3, z=0.2)
   
    sleep(2.0)
    bot.arm.set_single_joint_position("waist", dynamic_waist)
    #bot.arm.set_single_joint_position("shoulder", dynamic_shoulder)
    bot.gripper.open()
    bot.arm.set_ee_cartesian_trajectory(x=0.1, z=-0.16)
    bot.gripper.close()
    bot.arm.set_ee_cartesian_trajectory(x=-0.1, z=0.16)
    
    bot.arm.set_single_joint_position("waist", -np.pi/2.0)
   
    
    bot.arm.set_ee_cartesian_trajectory(pitch=1.5)
    bot.arm.set_ee_cartesian_trajectory(pitch=-1.5)
    bot.arm.set_single_joint_position("waist", np.pi/2.0)
    bot.arm.set_ee_cartesian_trajectory(x=0.1, z=-0.16)
    bot.gripper.open()
    bot.arm.set_ee_cartesian_trajectory(x=-0.1, z=0.16)
    bot.arm.go_to_home_pose()
    bot.arm.go_to_sleep_pose()
    

def handle_centroid(req):
    global bot, last_processed_time, processing_interval
    current_time = time()
    if current_time - last_processed_time < processing_interval:
        return CentroidResponse(success=False)  # Ignore if interval not passed

    cx, cy = req.cx, req.cy
    rospy.loginfo(f"Received centroid data: cx = {cx}, cy = {cy}")
    perform_pick_and_place(cx, cy)
    last_processed_time = current_time
    return CentroidResponse(success=True)

def main():
    global bot
    #rospy.init_node('robotic_arm_control')
    bot = InterbotixManipulatorXS("wx250s", "arm", "gripper")
    s = rospy.Service('centroid_service', Centroid, handle_centroid)
    rospy.spin()

if __name__ == '__main__':
    main()

