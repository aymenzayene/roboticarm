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
    global bot
   # Calculate dynamic waist position and clamp it within limits
   #dynamic_waist = -0.79288521 + 0.00244743 * cx - 0.00032535 * cy
    dynamic_waist = -1.208291239413795 + 0.00258455697229715266 * cx + 0.00129699620668963693 * cy
    dynamic_shoulder = -1.208291239413795- 0.004949365901311755 * cx + 0.0021293099284373407 * cy
    dynamic_elbow = -1.20829139413795 + 0.0004456043414232663 * cx - 0.0016755823305856727 * cy
    dynamic_wrist_angle = -0.0077659804792566955 -1.3953274884882079 * dynamic_shoulder  -0.8178526903149 * dynamic_elbow
    dynamic_wrist_angle = clamp(dynamic_wrist_angle, -1.745329, 2.146755, "wrist_angle")

    dynamic_forearm_roll=0
    dynamic_wrist_rotate=0
    joint_positions = [dynamic_waist,dynamic_shoulder ,dynamic_elbow, 0,  dynamic_wrist_angle,0]
    #bot = InterbotixManipulatorXS("wx250s", "arm", "gripper")
    bot.arm.go_to_home_pose()
    #bot.arm.set_joint_positions(joint_positions)
    sleep(2.0)
    bot.arm.set_ee_pose_components(x=0.3, z=0.2)
    bot.arm.set_ee_pose_components(y=0.3, z=0.2)
    sleep(2.0)
    bot.arm.set_single_joint_position("waist", dynamic_waist)
    bot.arm.set_single_joint_position("shoulder", dynamic_shoulder)
   
    
    bot.gripper.open()
    
    #bot.arm.set_joint_positions(joint_positions)
    #bot.arm.set_ee_pose_components(x=0.1, z=-0.16)
    bot.arm.set_ee_cartesian_trajectory(x=0.1, z=-0.16)
    bot.gripper.close()
    bot.arm.set_ee_cartesian_trajectory(x=-0.1, z=0.16)
    #bot.arm.set_ee_pose_components(x=-0.1, z=0.16)
    bot.arm.set_single_joint_position("waist", -np.pi/2.0)
    #set the object position
    #Setting the object position
    #x, y, z = cx, cy, 0.16

    # Approach the object
    #bot.arm.set_ee_pose_components(x=x, y=y, z=z+0.05, pitch=0.5)
    #sleep(2.0)  # Allow time for the arm to move

    # Move down to the object
    #bot.arm.set_ee_pose_components(x=x, y=y, z=z, pitch=0.5)
    #sleep(2.0)  # Allow time for the arm to move
    
    #bot.arm.set_ee_pose_components(x=cx/100,y=cy/100, z=0.2,pitch=0.5)
    #bot.arm.set_ee_cartesian_trajectory(z=cy/100, y=-0.02)
    
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

