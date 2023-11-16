#!/usr/bin/env python3

#---------------Import Libraries and msgs---------------:

import rospy
from robot_control_simulation.srv import * #response msg type
import time
import math
import numpy as np
from std_msgs.msg import *
from geometry_msgs.msg import *
import tf2_msgs.msg
from tf.transformations import euler_from_quaternion
import tf2_ros

#---------------Define Functions---------------:
class UR_Controller():

    def __init__(self):
        self.max_acc = 1.0 #0.5
        self.max_vel = 1.0 #0.5

		
    def get_pose(self): 
        """ Use TF publisher to get robot pose """
        # Create TF buffer to store last 10 transforms
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        currentTransform = None
        rate = rospy.Rate(10.0)
        # Within those transforms try to find base - tool frames
        while currentTransform == None and not rospy.is_shutdown():
            try:
                currentTransform = tfBuffer.lookup_transform('base', 'tool0', rospy.Time(0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()
                continue
    	#convert from tf2msg to geometry msg pose
        my_pose = Pose()
        my_pose.position = currentTransform.transform.translation
        my_pose.orientation = currentTransform.transform.rotation
        return my_pose


    def check_errors(self, robot_pose):
        """ Wait for ball pos service, and get out errors """
        rospy.wait_for_service('ball_traj')
        try:
            my_ball_traj = rospy.ServiceProxy('ball_traj', ball_traj)
            errors = my_ball_traj(robot_pose)
            return errors
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s"%e)


    def convert_to_euler(self, quat):
        """ Take our quat and convert it into x y z, also matches the orientation that the UR robot uses as a convention"""
        x = quat.x
        y = quat.y
        z =  quat.z
        w =  quat.w
        my_quat = [x,y,z,w] 
        (roll, pitch, yaw) = euler_from_quaternion(my_quat)
        ## Convert to correct yaw roll pitch orientation
        # Get Yaw matrix
        yawMatrix = np.matrix([
        [math.cos(yaw), -math.sin(yaw), 0],
        [math.sin(yaw), math.cos(yaw), 0],
        [0, 0, 1]
        ])
        # Get Pitch matrix
        pitchMatrix = np.matrix([
        [math.cos(pitch), 0, math.sin(pitch)],
        [0, 1, 0],
        [-math.sin(pitch), 0, math.cos(pitch)]
        ])
        # Get Roll matrix
        rollMatrix = np.matrix([
        [1, 0, 0],
        [0, math.cos(roll), -math.sin(roll)],
        [0, math.sin(roll), math.cos(roll)]
        ])
        # Get scalers / coefficients 
        R = yawMatrix * pitchMatrix * rollMatrix
        theta = math.acos(((R[0, 0] + R[1, 1] + R[2, 2]) - 1) / 2)
        multi = 1 / (2 * math.sin(theta))
        # Get yaw roll pitch
        rx = multi * (R[2, 1] - R[1, 2]) * theta
        ry = multi * (R[0, 2] - R[2, 0]) * theta
        rz = multi * (R[1, 0] - R[0, 1]) * theta
        return rx, ry, rz
        

    def generate_move_l(self, pose):
        """ The URSIM-ROS drivers require us to send strings that it decodes and uses for data in it's inverse kinematics, so we convert our pose msg into a string before sending it off.
        This is a special case, usually it's good practise to use msgs when sending data around in ROS, instead of sending everything off as strings. """
        header = "def myProg():"
        footer = "\nend"
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        rx, ry, rz = self.convert_to_euler(pose.orientation)
        pose_str = '['+str(x)+ ',' +str(y)+ ',' +str(z)+ ',' +str(rx)+ ',' +str(ry)+ ',' +str(rz) +']'
        command = header + '\n\tmovel(p{}, a={}, v={}, t=0, r=0)'.format(pose_str, self.max_acc, self.max_vel) + footer
        return command


    def generate_move_j(self, waypoint, sequence = False, pose_msg = False):
        """ Use waypoint or waypoints list to generate Move L command"""
        header = "def myProg():"
        footer = "\nend"
        move_msg = ""
        # Use pose msg or joint states
        if pose_msg:
            x = waypoint.position.x
            y = waypoint.position.y
            z = waypoint.position.z
            rx, ry, rz = UR_Controller().convert_to_euler(waypoint.orientation)
            pose_str = '['+str(x)+ ',' +str(y)+ ',' +str(z)+ ',' +str(rx)+ ',' +str(ry)+ ',' +str(rz) +']'
            move_msg = '\n\tmovej(p{}, a={}, v={}, t=0, r=0)'.format(pose_str, self.max_acc, self.max_vel)
        else:
            # If we have a list of waypoints, concatenate them all
            if sequence:
                for item in waypoint:
                    move ="\nmovej({},a={},v={},t={},r={})".format(item, self.max_acc ,self.max_vel,0,0)
                    move_msg += move
            # Otherwise just send it off
            else:
                move_msg ="\nmovej({},a={},v={},t={},r={})".format(waypoint, self.max_acc ,self.max_vel,0,0)
        command = header + move_msg + footer
        return command


    def rotate_tool(self, rx, ry, rz):
        """Function that rotates using tool frame instead of base frame, so we can easily rotate in angles we're familar with"""
        header = "def myProg():"
        footer = "\nend"	
        msg = """
        global pose_wrt_tool = p[{}, {}, {}, {}, {}, {}]
        global pose_wrt_base = pose_trans(get_forward_kin(), pose_wrt_tool)
        movel( pose_wrt_base, a={}, v={})""".format(0, 0, 0, rx, ry, rz, self.max_acc, self.max_vel)
        command = header + msg + footer
        return command

    def tar_check(self,tar_pos):
        # check if we have reached target pos, ignore orientation
        i = 0
        tar_reached = False
        while not tar_reached:
            current_pos = self.get_pose()
            tar_reached = (round(current_pos.position.x, 3) == round(tar_pos.position.x, 3)) and (round(current_pos.position.y, 3) == round(tar_pos.position.y, 3)) and (round(current_pos.position.z, 3) == round(tar_pos.position.z, 3))
            i+= 1
        return True


