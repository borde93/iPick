#Swiftpro Arm Control system
#------------------------------
#This code controls a Swiftpro Arm using ROS2 nodes.
#
#This node in particular is responsible for broadcasting 
#the trnsformation of the arm gripper with respect to the base
#due to arm limitations, and to speed up the development the 



#!/usr/bin/env python3
import math
import sys
import numpy as np
import os
import time

#from std_msgs.msg import String

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from swiftpro_interfaces.msg import ArmPointDetection
from swiftpro_interfaces.action import GrabStrawberry
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import TransformStamped, PointStamped, Point, PoseStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
import tf2_geometry_msgs
from uarm.wrapper import SwiftAPI 
from uarm.utils.log import logger
from rclpy.duration import Duration

from tf2_ros import TransformBroadcaster

MOVE_SPEED = 60

CENTER_S = 165.0
CENTER_R = 90.0
CENTER_H = 120.0

HOMING_S = 150.0
HOMING_R = 50.0
HOMING_H = 100.0 

BASKET_S = 180.0
BASKET_R = 130.0
BASKET_H = 40.0 


ARM_GRIPPER_X = 160.0   #Offset from arm tcp to griper tcp
CAM_ARM_Z = 14      #14mm from camera screw to arm tcp coordinates 
ARM_GRIPPER_Z = 33  #From arm tcp to actual gripper tcp z offset (mm)

CAM_MIN_X = 200.0 #0.200 #should be 0.150
MAX_ERROR = 10.0

#Software limits for the arm sadly, they don't have a good implementation in their software.
MAX_R = 138
MIN_R = 45
MAX_S = 300
MIN_S = 140 
MIN_H =35 #5 #35 is to avoid touching the table while not on rover
MAX_H = 165 #145

PICK_Z = 35.0

UNIT_CONV = 1000




#logger.setLevel(logger.VERBOSE)

class swiftpro_node(Node):
    def __init__(self):
        super().__init__("swiftpro_state")
      
        self.swift_ = SwiftAPI(filter={'hwid': 'USB VID:PID=2341:0042'})
        self.swift_.waiting_ready(timeout=3)
        time.sleep(2)
        # Initialize the transform broadcaster
        self.base_tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.declare_parameter("publish_period", 0.2)
      
        #Move arm to initial position
        self.move_arm_polar(CENTER_S, CENTER_R, CENTER_H)
        self.move_arm_polar(HOMING_S, HOMING_R, HOMING_H)
        time.sleep(3)
        self.is_moving_ = False
        self.unreachCtr = 0
        self.t = self.get_transform()
        self.move_speed = 20

        #Start the action server
        self.servicing_goal = False
        self.grab_strawberry_server_ = ActionServer(
            self, GrabStrawberry, "grab_strawberry", 
            goal_callback = self.goal_callback, 
            cancel_callback = self.cancel_callback,
            execute_callback = self.execute_callback, 
            callback_group=ReentrantCallbackGroup())
        
        self.get_logger().info("Grab server has been started.")

        #Setting up timer
        self.timer_period_ = self.get_parameter("publish_period").value
        self.position_timer_ = self.create_timer(self.timer_period_, self.publish_transform)
        

        #------------------- TRANSFORM PUBLISHER--------------
    def publish_transform(self):
        if not self.is_moving_:
            self.t = self.get_transform()
            self.base_tf_broadcaster.sendTransform(self.t)
        #self.base_tf_broadcaster.sendTransform(self.t)
    
        #-------------------------- ACTIION SERVER ----------------------
    def goal_callback(self, goal_request: GrabStrawberry.Goal):
        self.get_logger().info("Received a goal")
        
        #t_goal = goal_request.target.point
        #self.get_logger().info(f"{goal_request.target}")
        #If we are still completing the previous action, reject the target
        if self.servicing_goal:
            #self.get_logger().warn(f"Rejecting goal, self.servicing staus=: {self.servicing_goal}")
            return GoalResponse.REJECT
        
        
        self.get_logger().info("Accepting the goal")
        return GoalResponse.ACCEPT
        
        # Any cancel request will be processed here, we can accept or reject it
    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Received a cancel request")
        return CancelResponse.ACCEPT

    # If a goal has been accepted, it will then be executed in this callback
    # After we are done with the goal execution we set a final state and return the result
    # When executing the goal we also check if we need to cancel it
    # tc -> taget in camera frame coordinates
    # ta -> target in arm frame coordinates
    def execute_callback(self, goal_handle: ServerGoalHandle):
        result = GrabStrawberry.Result()
        strwbsCoord : ArmPointDetection = goal_handle.request.target.strawberry 
        self.servicing_goal = True
       # base_coord : PointStamped = goal_handle.request.target
        is_centered = False
        

        try:
            t_camera = self.tf_buffer.lookup_transform(target_frame='camera_bottom_screw_frame', source_frame='arm_base',time=rclpy.time.Time(), timeout=Duration(seconds=5))
        except Exception as e:
            self.get_logger().warn(f"Target to camera frame transform failed: {e}")          
            goal_handle.abort()
            self.servicing_goal = False 
            return result
        
        self.get_logger().warn(f"Passed the cameraTransform")
        for target in strwbsCoord:            
            base_coord = target.pose.position
            self.get_logger().warn(f"Base coordinates: {base_coord}")

            # #converting m to mm for x y z base coordinates
            ta = [int(v * 1000) for v in (
                base_coord.x,
                base_coord.y,
                base_coord.z
            )]
            
            #If the point is reachable, procees to next steps
            S, R, H = self.coord_to_polar(ta[0], ta[1], ta[2])
            if self.can_reach(S - ARM_GRIPPER_X, R, H + ARM_GRIPPER_Z):

            #Center the strawberry to the camera
        
                target_camera = do_transform_pose(target.pose, t_camera)
                tc = [int(v * 1000) for v in (
                target_camera.position.x,
                target_camera.position.y,
                target_camera.position.z
            )]
                
                break
            
            else:
                result.move_completed = False 

        self.get_logger().info(f"Strawberry arm coordinates: {ta}")

 
        if self.can_reach(S - ARM_GRIPPER_X, R, H + ARM_GRIPPER_Z):
            #Check and center the strawberry to the camera
            is_centered, result.move_completed = self.center_strawberry(tc, ta)

            #Pick the strawberry
            if is_centered:
                pick_completed = self.pick_strawberry(ta)
                if not pick_completed:
                    result.move_completed = False
        else:
            result.move_completed = False

            #after 10 times target is unreacheable, HOME the arm. (To recover in case of bad centering)
            if self.unreachCtr > 10:
                self.move_arm_polar(HOMING_S, HOMING_R, HOMING_H)
                self.unreachCtr = 0

        if result.move_completed:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        self.servicing_goal = False             #ready to receive next goal
        return result
  

    
    #Checks if the frame is centered and at min distance for final grabbing 
    #Inputs, tc -> camera coordinates of the target
    #        ta -> arm coordinates of the targer
    #Outputs isCentered and moveCompleted
    def center_strawberry(self, tc: list, ta: list) -> tuple[bool, bool]:
        isCentered = True         
        moveCompleted = True
        tc_x, tc_y, tc_z = tc       #Target in camera coordinates
        ta_x, ta_y, ta_z = ta       #Target in Arm coordinates

        #self.get_logger().info(f"center coordinates: x: {tc_x}, y: {tc_y} z: {tc_z}")
        
        S, R, H = self.coord_to_polar(ta_x, ta_y, ta_z)

        errX = abs(tc_x - CAM_MIN_X)
        errY = abs(tc_y)
        errZ = abs(tc_z)

        self.get_logger().info(f"errX: {errX}, errY: {errY}, errZ: {errZ}")

        if ((errX > MAX_ERROR) or (errZ > MAX_ERROR) or (errY >  MAX_ERROR)):
            self.get_logger().info("WE ARE CENTERING THE STRAWBERRY")
            moveCompleted = self.move_arm_polar(S - CAM_MIN_X, R, H - CAM_ARM_Z)
            isCentered = False

        return isCentered, moveCompleted
    

    #Reaches the strawberry, grips it, removes from the stem and drops it to the basket
    #Input: Absolute coordinates
    #Return: Move completed (True if arm completed all actions)
    def pick_strawberry(self, ta: list) -> bool:
        ta_x, ta_y, ta_z = ta
        S, R, H = self.coord_to_polar(ta_x, ta_y, ta_z)
        S = S - ARM_GRIPPER_X       #Gripper tcp coordinates
        H = H + ARM_GRIPPER_Z       

        self.get_logger().info(f"Picking, Strawberry coordinates: x: {ta_x}, y: {ta_y} z: {ta_z}")   
        move_completed = self.move_arm_polar(S+5, R, H)                       #Move gripper tcp to strawberry
        if move_completed:
            grip_success = self.swift_.set_gripper(True, timeout=5,wait=True)
            time.sleep(3)
            if grip_success.lower() == 'ok':
                    move_completed = True
        if move_completed:
            move_completed = self.move_arm_polar(S, R, (H - PICK_Z))          #Lower arm 
            time.sleep(1)
        if move_completed:
            move_completed = self.move_arm_polar(CENTER_S, CENTER_R, CENTER_H)           #move to center
            time.sleep(1)
        if move_completed:
            move_completed = self.move_arm_polar(BASKET_S, BASKET_R, BASKET_H)           #move to basket
            time.sleep(1)

        grip_success = self.swift_.set_gripper(False, timeout=5, check=True)         #drop strawberry
        time.sleep(3)  
        move_completed = self.move_arm_polar(HOMING_S, HOMING_R, HOMING_H)                #Move to strawberry picking area
        time.sleep(3)
        return move_completed


    # Move arm using Polar coordinates, S = Stretch, R = Rotation, H = Height 
    # Returns wether the move was succesfull or not
    def move_arm_polar(self, S: float, R: float, H: float) -> bool:
        self.is_moving_ = True
        move_completed = False
            
        if not self.can_reach(S, R, H):
            self.is_moving_ = False
            return move_completed
        
        self.get_logger().info(f"moving:  Strech: {S}, Rotation: {R}, Height: {H}") 
        move_finished = self.swift_.set_polar(S, R, H, speed=MOVE_SPEED, wait=True, timeout=10)
        
        self.t = self.get_transform()
        self.base_tf_broadcaster.sendTransform(self.t)
        self.is_moving_ = False
        time.sleep(0.3)
        self.get_logger().info(f"move finished: {move_finished}")
        
        if move_finished.lower() == 'ok':
            move_completed = True

        return move_completed
    

    #function to move the arm on x, y ,z coordinates replaced by move_arm_polar)
    def move_arm(self, ta_x: int, ta_y: int, ta_z: int) -> bool:
        self.is_moving_ = True
        move_completed = False
        
        if self.swift_.check_pos_is_limit([ta_x, ta_y, ta_z]):
            self.get_logger().warning("Rejecting the goal, target is out of reach")
            return move_completed
        self.get_logger().info(f"move coordinates:x: {ta_x},t: {ta_y}, z: {ta_z}") 
        move_finished = self.swift_.set_position(x=ta_x, y=ta_y, z=ta_z, speed=20, wait=True, timeout=10, cmd='G0')
        
        self.t = self.get_transform()
        self.base_tf_broadcaster.sendTransform(self.t)
        self.is_moving_ = False
        time.sleep(0.2)
        self.get_logger().info(f"move finished: {move_finished}")
        
        if move_finished.lower() == 'ok':
            move_completed = True

        return move_completed
       
    #convert x, y, z coordinates into Stretch, Rotation, Height
    def coord_to_polar(self, x, y, z):
        self.get_logger().info(f"Converting xyz to polar: x: {x}, y: {y} z: {z}")
        S = round(np.hypot(x, y), 4)
        R = round(np.rad2deg(np.arctan2(y, x)) + 90, 4)
        H = round(z, 4)
        self.get_logger().info(f"Converted to: S: {S}, R: {R} H: {H}")
        
        return S, R, H
    
    #Given polar coordinates, check if the point is reachable
    def can_reach(self, S: float, R: float, H: float) -> bool:
        over_limit = False
        canReach = True
        S, R, H = [round(v, 4) for v in (S, R, H)]

        over_limit = not(MIN_S < S < MAX_S
                         and MIN_R < R < MAX_R 
                         and MIN_H < H < MAX_H)
        
        if not MIN_S < S < MAX_S:
            self.get_logger().warn(f"S: {S} should be between {MIN_S} and {MAX_S}")
            over_limit = True

        if not MIN_H < H < MAX_H:
            self.get_logger().warn(f"H: {H} should be between {MIN_H} and {MAX_H}")
            over_limit = True

        if not MIN_R < R < MAX_R:
            self.get_logger().warn(f"R: {R} should be between {MIN_R} and {MAX_R}")
            over_limit = True

        if self.swift_.check_pos_is_limit([S, R, H], is_polar=True, timeout=3) or over_limit:       #S > MAX_S or S < MIN_S or R > MAX_R or R < MIN_R
            canReach = False
            self.unreachCtr += 1
        
        return canReach                


    

    #Calculates the quaternion for the transformation of the gripper link with respect to the base

    def get_transform(self):
        t = TransformStamped()
        
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'arm_base'
        t.child_frame_id = 'gripper_link'


        
        readAbs = self.swift_.get_position(wait = True, timeout = 10)
        readPolar = self.swift_.get_polar(wait = True, timeout = 10)
            
        arm_x = readAbs[0]
        arm_y = readAbs[1]
        arm_z = readAbs[2]
    
        baseRagAng = np.deg2rad(readPolar[1] - 90.0)

        quat = self.quaternion_from_euler(0, 0, baseRagAng)
        
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        

        t.transform.translation.x= arm_x / 1000     #/1000 for m
        t.transform.translation.y= arm_y / 1000
        t.transform.translation.z= arm_z / 1000 

        # self.base_tf_broadcaster.sendTransform(self.t)

        
        return t
    
        # This function is a stripped down version of the code in
    # https://github.com/matthew-brett/transforms3d/blob/f185e866ecccb66c545559bc9f2e19cb5025e0ab/transforms3d/euler.py
    # Besides simplifying it, this version also inverts the order to return x,y,z,w, which is
    # the way that ROS prefers it.
    def quaternion_from_euler(self, ai, aj, ak):
        ai /= 2.0
        aj /= 2.0
        ak /= 2.0
        ci = math.cos(ai)
        si = math.sin(ai)
        cj = math.cos(aj)
        sj = math.sin(aj)
        ck = math.cos(ak)
        sk = math.sin(ak)
        cc = ci*ck
        cs = ci*sk
        sc = si*ck
        ss = si*sk

        q = np.empty((4, ))
        q[0] = cj*sc - sj*cs
        q[1] = cj*ss + sj*cc
        q[2] = cj*cs - sj*sc
        q[3] = cj*cc + sj*ss

        return q

    


def main(args=None):
    rclpy.init(args=args)
    node = swiftpro_node()
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()


if __name__ == "__main__":
    main()