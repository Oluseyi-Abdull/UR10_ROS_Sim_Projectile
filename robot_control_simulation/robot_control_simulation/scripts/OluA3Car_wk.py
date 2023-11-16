#!/usr/bin/env python3

#------------------------------Import Libraries and msgs----------------------------------------:

import rospy
import time
import math
import numpy as np
from std_msgs.msg import *
from geometry_msgs.msg import *
import tf2_msgs.msg
from tf.transformations import euler_from_quaternion
from collections import Counter
from control_lib import UR_Controller


#-----------------------------------------Initialise--------------------------------------------#

print("Please Wait While System Starts Up...")
rospy.init_node("Olu_A3", anonymous=False)
ur_script = rospy.Publisher('/ur_hardware_interface/script_command', String, queue_size=10)
ur_con = UR_Controller()
time.sleep(2)
print("...System Started")

# -------------------------------------------VARIABLES----------------------------------------- #

# Declaring the starting position for the robot. Using Radians 
# Potentially can create a radian to degree converter if necessary

max_acc = 1.0 # value for UR10 maximum accelleration
max_vel = 1.0 # value for UR10 maximum velocity

#Defining the axes below, storing them in lists and naming the list components as strings 
# rot = ["roll", "pitch", "yaw"]

# axes = ["x", "y", "z"]

# Add a variable to catch ball specific number of times. balls_caught = 0 before program runs. 

#potentially change the values 
home_waypoint = [1.58825, -1.71042, -2.19911, -0.802851, 1.58825, -0.03106686] 
#command = ur_con.generate_move_j(home_waypoint)
#ur_script.publish(command)
#time.sleep(5) # TO DO replace this with check

#-----------------------------------------------Functions---------------------------------------#

def home_robot():  ### Change the home co-orddinates to my desired position###
	header = "def myProg():"
	footer = "\nend"
	home_waypoint = [1.58825, -1.71042, -2.19911, -0.802851, 1.58825, -0.03106686]
	move_msg ="\nmovej({},a={},v={},t={},r={})".format(home_waypoint, max_acc ,max_vel,0,0)
	command = header + move_msg + footer
	return command

def move_j_pose(pose): ## Move J is preffered###
	header = "def myProg():"
	footer = "\nend"
	x = pose.position.x
	y = pose.position.y
	z = pose.position.z
	rx, ry, rz = UR_Controller().convert_to_euler(pose.orientation)
	pose_str = '['+str(x)+ ',' +str(y)+ ',' +str(z)+ ',' +str(rx)+ ',' +str(ry)+ ',' +str(rz) +']'
	command = header + '\n\tmovej(p{}, a={}, v={}, t=0, r=0)'.format(pose_str, max_acc, max_vel) + footer
	return command
	
	# Check Error
	my_pose = ur_con.get_pose()
	errors = ur_con.check_errors(my_pose)
	print("Cartesian error:", (errors.ee_trans_error),"m") # Directional error in meters
	print("Orientational error:", errors.ee_rot_error, "degrees") # Rot error in degrees
	print("Ball caught:",errors.ball_caught) # Boolean tells us if the ball was caught or not

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

def tar_check(self,tar_pos):
        # check if we have reached target pos, ignore orientation
        i = 0
        tar_reached = False
        while not tar_reached:
            current_pos = self.get_pose()
            tar_reached = (round(current_pos.position.x, 3) == round(tar_pos.position.x, 3)) and (round(current_pos.position.y, 3) == round(tar_pos.position.y, 3)) and (round(current_pos.position.z, 3) == round(tar_pos.position.z, 3))
            i+= 1
        return True

#-------Function to determine error values based on Rotational Error values Thresholds---------#

def thresh_car(): 
	my_pose = ur_con.get_pose()
	errors = ur_con.check_errors(my_pose)
	rot_error = errors.ee_rot_error # Check for rotational error
	car_err = errors.ee_trans_error #Checking cartesian Error
	
	if car_err >= 1.50: # If the cartesian error is above a cetain value then 
		thresh_car.mag = 1.50 # Set cartesian magnitude value to a particular number. 
	elif car_err >= 1.0: 
		thresh_car.mag = 1.0
	elif car_err >= 0.50: 
		thresh_car.mag = 0.40
	elif car_err >= 0.25: 
		thresh_car.mag = 0.20
	elif car_err >= .10: 
		thresh_car.mag = 0.10
	elif car_err >= 0.05:
		thresh_car.mag = 0.02
	elif car_err < 0.05: 
		print(car_err,"Cartesian Threshold met!")	
	return thresh_car.mag

#-------Function to determine error values based on Rotational Error values Thresholds---------#

def thresh_rot():
	my_pose = ur_con.get_pose()
	errors = ur_con.check_errors(my_pose)
	rot_error = errors.ee_rot_error # Check for rotational error
	car_err = errors.ee_trans_error #Checking cartesian Error
	pi = 3.142
	
	if rot_error >= 150: #If rotational error is below a particular preset value then
		angle = 2.0944
	elif rot_error >= 100: # To actuate the robot grasper
		angle = 1.5708
	elif rot_error >= 50: 
		angle = 0.698132
	elif rot_error >= 25: 
		angle = 0.349066
	elif rot_error >= 10: 
		angle = 0.139626
	elif rot_error >= 5: 
		angle = 0.0349066
	elif rot_error <= 5:
		print("Rotational Threshold met!")
#	elif rot_error >= 0:
#	
#		print(angle, "degrees")
#		print(rad,"Radians")	
	return angle 
	
#-------- Exploration function that decides the moves the robot arm using x,y,z Axis'-----------#

####Additional preventative measures are needed to make sure the simulation does not break, limitations to the max movement of the manipulator ** Cartesian Specifically 


#For the exploration functions they may only ever need to move in all axis by 1 cm, by comparing the initial error value and the new value and finding out which one is reduces error the most a immediate best direction can be determined fairly quickly 

#Potentially make a list that gets appended to with the initial error value that compares with the appended value if it the error becomes smaller pop the value that is the increase in error/ return the list int of the value that is the smallest. 

def explore_c(dist):
	explore_c.car_error_values = [] #create empty list [x,-x,y,-y,z,-z] list index = next move 
	#Find smallest number in the list and its respective index 
	
#####------------------------------------EXPLORE +X AXIS--------------------------------###	
	
	###--- Check Cartesian Error ---###
	my_pose = ur_con.get_pose()
	errors = ur_con.check_errors(my_pose)
	cat_err = errors.ee_trans_error
	print("Cartesian error:", (cat_err),"m") #Directional error in meters
	
	###--- Move +X using lib and pose ---###
	my_pose.position.x += dist
	command = ur_con.generate_move_j(my_pose, pose_msg=True)
	ur_script.publish(command)
	ur_con.tar_check(my_pose)
	
	###--- Check Cartesian Error ---###
	my_pose = ur_con.get_pose() # Check Pose for better error check
	errors = ur_con.check_errors(my_pose)
	cat2_err = errors.ee_trans_error
	print("+X: Cartesian error:", (cat2_err),"m") #Directional error in meters
	#if cat2_err < 0.05:
#		break
	#Append Error after movement to populate list
	explore_c.car_error_values.append(cat2_err) 
	
#####-------------------------RETURN TO INITIAL POSITION--------------------------------###	

###----Move to inital position using lib and pose----###

	my_pose = ur_con.get_pose()
	my_pose.position.x -= dist
	command = ur_con.generate_move_j(my_pose, pose_msg=True)
	ur_script.publish(command)
	ur_con.tar_check(my_pose)

#####------------------------------------EXPLORE -X AXIS--------------------------------###	

	###--- Check Cartesian Error ---###
	my_pose = ur_con.get_pose()
	errors = ur_con.check_errors(my_pose)
	cat_err = errors.ee_trans_error
#	print("Cartesian error:", (cat_err),"m") #Directional error in meters
	
	##Move -X using lib and pose
	my_pose.position.x -= dist
	command = ur_con.generate_move_j(my_pose, pose_msg=True)
	ur_script.publish(command)
	ur_con.tar_check(my_pose)
	
	###--- Check Cartesian Error ---###
	my_pose = ur_con.get_pose() # Check Pose for better error check
	errors = ur_con.check_errors(my_pose)
	cat2_err = errors.ee_trans_error
	print("-X: Cartesian error:", (cat2_err),"m") #Directional error in meters
	explore_c.car_error_values.append(cat2_err)

#####-------------------------RETURN TO INITIAL POSITION--------------------------------###
	
###----Move to inital position using lib and pose----###
	
	my_pose = ur_con.get_pose()
	my_pose.position.x += dist
	command = ur_con.generate_move_j(my_pose, pose_msg=True)
	ur_script.publish(command)
	ur_con.tar_check(my_pose)

#####------------------------------------EXPLORE +Y AXIS--------------------------------###	

###----Move to inital position using lib and pose----###
	my_pose = ur_con.get_pose()
	errors = ur_con.check_errors(my_pose)
	cat_err = errors.ee_trans_error
#	print("Cartesian error:", (cat_err),"m") #Directional error in meters
	
	###--- Move +Y using lib and pose ---###
	my_pose.position.y += dist
	command = ur_con.generate_move_j(my_pose, pose_msg=True)
	ur_script.publish(command)
	ur_con.tar_check(my_pose)
	
	###--- Check Cartesian Error ---###
	my_pose = ur_con.get_pose() # Check Pose for better error check
	errors = ur_con.check_errors(my_pose)
	cat2_err = errors.ee_trans_error
	print("+Y: Cartesian error:", (cat2_err),"m") #Directional error in meters
	explore_c.car_error_values.append(cat2_err)

#####-------------------------RETURN TO INITIAL POSITION--------------------------------###

###----Move to inital position using lib and pose----###

	my_pose = ur_con.get_pose()
	my_pose.position.y -= dist
	command = ur_con.generate_move_j(my_pose, pose_msg=True)
	ur_script.publish(command)
	ur_con.tar_check(my_pose)

#####------------------------------------EXPLORE -Y AXIS--------------------------------###	

	##Move using lib and pose
	my_pose = ur_con.get_pose()
	errors = ur_con.check_errors(my_pose)
	cat_err = errors.ee_trans_error
#	print("Cartesian error:", (cat_err),"m") #Directional error in meters
	
	###--- Move -Y using lib and pose ---###
	my_pose.position.y -= dist
	command = ur_con.generate_move_j(my_pose, pose_msg=True)
	ur_script.publish(command)
	ur_con.tar_check(my_pose)
	
	###--- Check Cartesian Error ---###
	my_pose = ur_con.get_pose() # Check Pose for better error check
	errors = ur_con.check_errors(my_pose)
	cat2_err = errors.ee_trans_error
	print("-Y: Cartesian error:", (cat2_err),"m") #Directional error in meters
	explore_c.car_error_values.append(cat2_err)
	
#####-------------------------RETURN TO INITIAL POSITION--------------------------------###

###----Move to inital position using lib and pose----###
	
	my_pose = ur_con.get_pose()
	my_pose.position.y += dist
	command = ur_con.generate_move_j(my_pose, pose_msg=True)
	ur_script.publish(command)
	ur_con.tar_check(my_pose)

#####------------------------------------EXPLORE +Z AXIS--------------------------------###	

	##Move using lib and pose
	my_pose = ur_con.get_pose()
	errors = ur_con.check_errors(my_pose)
	cat_err = errors.ee_trans_error
#	print("Cartesian error:", (cat_err),"m") #Directional error in meters
	
	###--- Move +Z using lib and pose ---###
	my_pose.position.z += dist
	command = ur_con.generate_move_j(my_pose, pose_msg=True)
	ur_script.publish(command)
	ur_con.tar_check(my_pose)
	
	###--- Check Cartesian Error ---###
	my_pose = ur_con.get_pose() # Check Pose for better error check
	errors = ur_con.check_errors(my_pose)
	cat2_err = errors.ee_trans_error
	print("+Z: Cartesian error:", (cat2_err),"m") #Directional error in meters
	explore_c.car_error_values.append(cat2_err)

#####-------------------------RETURN TO INITIAL POSITION--------------------------------###

###----Move to inital position using lib and pose----###
	
	my_pose = ur_con.get_pose()
	my_pose.position.z -= dist
	command = ur_con.generate_move_j(my_pose, pose_msg=True)
	ur_script.publish(command)
	ur_con.tar_check(my_pose)

#####------------------------------------EXPLORE -Z AXIS--------------------------------###	

	##Move on -Z using lib and pose
	my_pose = ur_con.get_pose()
	errors = ur_con.check_errors(my_pose)
	cat_err = errors.ee_trans_error
	#print("Cartesian error:", (cat_err),"m") #Directional error in meters
	
	###--- Move -Z using lib and pose ---###
	my_pose.position.z -= dist
	command = ur_con.generate_move_j(my_pose, pose_msg=True)
	ur_script.publish(command)
	ur_con.tar_check(my_pose)
	
	###--- Check Cartesian Error ---###
	my_pose = ur_con.get_pose() # Check Pose for better error check
	errors = ur_con.check_errors(my_pose)
	cat2_err = errors.ee_trans_error
	print("-Z: Cartesian error:", (cat2_err),"m") #Directional error in meters
	explore_c.car_error_values.append(cat2_err)
	
#####-------------------------RETURN TO INITIAL POSITION--------------------------------###

###----Move to inital position using lib and pose----###
	
	my_pose = ur_con.get_pose()
	my_pose = ur_con.get_pose()
	my_pose.position.z += dist
	command = ur_con.generate_move_j(my_pose, pose_msg=True)
	ur_script.publish(command)
	ur_con.tar_check(my_pose)
	print("Cartesian Exploration Complete!")
	print(explore_c.car_error_values)
	return explore_c.car_error_values #Populated list [x,-x,y,-y,z,-z] index = next move 

#--------Exploration function that decides the moves the robot arm using Pitch,Roll,Yaw Axis'---#
################################################################################################

def explore_r():
	###----Create list for rotational error---###### HEEEEEEEEEELP
	rot_error_values = [] # [r,-r,p,-p,y,-y] 
	angle = thresh_rot()
###--------------------------------------------EXPLORE +ROLL----------------------------------###
	
	###---GET ERROR---###
	my_pose = ur_con.get_pose()
	errors = ur_con.check_errors(my_pose)
	rot_error = errors.ee_rot_error
#	print("Rotational error:", (rot_error),"degrees") #Directional error in meters

	###---Explore the +Roll Axis---###
	command = ur_con.rotate_tool(angle, 0, 0) #Roll,Pitch, Yaw
	ur_script.publish(command) # The Amount the robot will move and which direction
	time.sleep(2)
####	print("+Roll:", angle, "radian")
	
	###--- Check ROTATIONAL Error ---###
	my_pose = ur_con.get_pose() # Check Pose for better error check
	errors = ur_con.check_errors(my_pose)
	rot2_err = errors.ee_rot_error
#	print("Rotational error:", (rot2_err),"degrees") #Directional error in degrees
	
	##---ADD VALUE TO LIST---###
	###### HEEEEEEEEEELP
	print("+Roll" ,rot2_err, "degrees")
	
	rot_error_values.append(rot2_err)
#	print("+Roll",rot_error_values)
	
###------------------------------RETURN TO INITIAL POSITION----------------------------------###
	
	###---Return to initial position---###
	my_pose = ur_con.get_pose # Check Pose for better error check
	command = ur_con.rotate_tool(-angle, 0, 0) #Roll,Pitch, Yaw
	ur_script.publish(command) # The Amount the robot will move and which direction
	time.sleep(2)
#	print("-Roll", -angle, "r")

###--------------------------------------------EXPLORE -ROLL----------------------------------###
	
	###---GET ERROR---###
	my_pose = ur_con.get_pose()
	errors = ur_con.check_errors(my_pose)
	rot_error = errors.ee_rot_error
#	print("Rotational error:", (rot_error),"degrees") #Directional error in meters

	###---Explore the -Roll Axis---###
	command = ur_con.rotate_tool(-angle, 0, 0) #Roll,Pitch, Yaw
	ur_script.publish(command) # The Amount the robot will move and which direction
	time.sleep(2)
####	print("-Roll:", -angle, "radian")
	
	###--- Check ROTATIONAL Error ---###
	my_pose = ur_con.get_pose() # Check Pose for better error check
	errors = ur_con.check_errors(my_pose)
	rot2_err = errors.ee_rot_error
#	print("Rotational error:", (rot2_err),"degrees") #Directional error in meters
	
	##---ADD VALUE TO LIST---###
	###### HEEEEEEEEEELP
	print("-Roll",rot2_err, "degrees")
	rot_error_values.append(rot2_err)
	
#	print("-Roll",rot_error_values)
	
###------------------------------RETURN TO INITIAL POSITION----------------------------------###
	
	###---Return to initial---###
	my_pose = ur_con.get_pose # Check Pose for better error check
	command = ur_con.rotate_tool(angle, 0, 0) #Roll,Pitch, Yaw
	ur_script.publish(command) # The Amount the robot will move and which direction
	time.sleep(2)
#	print("+Roll", angle, "r")
	
###--------------------------------------------EXPLORE +Pitch----------------------------------###
	
	###---GET ERROR---###
	my_pose = ur_con.get_pose()
	errors = ur_con.check_errors(my_pose)
	rot_error = errors.ee_rot_error
#	print("Rotational error:", (rot_error),"degrees") #Directional error in meters

	###---Explore the +Pitch Axis---###
	command = ur_con.rotate_tool(0,angle, 0) #Roll,Pitch, Yaw
	ur_script.publish(command) # The Amount the robot will move and which direction
	time.sleep(2)
#	print("+Pitch:", angle, "radian")
	
	###--- Check ROTATIONAL Error ---###
	my_pose = ur_con.get_pose() # Check Pose for better error check
	errors = ur_con.check_errors(my_pose)
	rot2_err = errors.ee_rot_error
#	print("Rotational error:", (rot2_err),"degrees") #Directional error in meters
	
	##---ADD VALUE TO LIST---###
	###### HEEEEEEEEEELP
	print("+Pitch",rot2_err, "degrees")
	rot_error_values.append(rot2_err)
#	print("+Pitch",rot_error_values)
	
###------------------------------RETURN TO INITIAL POSITION----------------------------------###
	
	###---Return to initial position---###
	my_pose = ur_con.get_pose # Check Pose for better error check
	rot_error # Check error
	command = ur_con.rotate_tool(0, -angle, 0) #Roll,Pitch, Yaw
	ur_script.publish(command) # The Amount the robot will move and which direction
	time.sleep(2)
#	print("-Pitch", -angle, "r")

###--------------------------------------------EXPLORE -Pitch----------------------------------#
	
	###---GET ERROR---###
	my_pose = ur_con.get_pose()
	errors = ur_con.check_errors(my_pose)
	rot_error = errors.ee_rot_error
#	print("Rotational error:", (rot_error),"degrees") #Directional error in meters

	###---Explore the +Pitch Axis---###
	command = ur_con.rotate_tool(0, -angle, 0) #Roll,Pitch, Yaw
	ur_script.publish(command) # The Amount the robot will move and which direction
	time.sleep(2)
#	print("-Pitch:", -angle, "radian")
	
	###--- Check ROTATIONAL Error ---###
	my_pose = ur_con.get_pose() # Check Pose for better error check
	errors = ur_con.check_errors(my_pose)
	rot2_err = errors.ee_rot_error
#	print("Rotational error:", (rot2_err),"degrees") #Directional error in meters
	
	##---ADD VALUE TO LIST---###
	###### HEEEEEEEEEELP
	print("-Pitch: " ,rot2_err, "degrees")
	rot_error_values.append(rot2_err)
#	print("-Pitch",rot_error_values)

###------------------------------RETURN TO INITIAL POSITION----------------------------------###
	
	###---Return to initial---###
	my_pose = ur_con.get_pose # Check Pose for better error check
	command = ur_con.rotate_tool(0, angle, 0) #Roll,Pitch, Yaw
	ur_script.publish(command) # The Amount the robot will move and which direction
	time.sleep(2)
#	print("+Pitch", angle, "r")
	
###--------------------------------------------EXPLORE +Yaw----------------------------------#
	
	###---GET ERROR---###
	my_pose = ur_con.get_pose()
	errors = ur_con.check_errors(my_pose)
	rot_error = errors.ee_rot_error
#	print("Rotational error:", (rot_error),"degrees") #Directional error in meters

	###---Explore the +Yaw Axis---###
	command = ur_con.rotate_tool(0, 0, angle) #Roll,Pitch, Yaw
	ur_script.publish(command) # The Amount the robot will move and which direction
	time.sleep(2)
#	print("+Yaw:", angle, "radian")
	
	###--- Check ROTATIONAL Error ---###
	my_pose = ur_con.get_pose() # Check Pose for better error check
	errors = ur_con.check_errors(my_pose)
	rot2_err = errors.ee_rot_error
#	print("Rotational error:", (rot2_err),"degrees") #Directional error in meters
	
	##---ADD VALUE TO LIST---###
	###### HEEEEEEEEEELP
	print("+Yaw: ",rot2_err, "degrees")
	rot_error_values.append(rot2_err)
#	print("+Yaw",rot_error_values)

###------------------------------RETURN TO INITIAL POSITION----------------------------------###
	
	###---Return to initial position---###
	my_pose = ur_con.get_pose # Check Pose for better error check
	command = ur_con.rotate_tool(0, 0, -angle) #Roll,Pitch, Yaw
	ur_script.publish(command) # The Amount the robot will move and which direction
	time.sleep(2)
#	print("-Yaw", -angle, "r")

###--------------------------------------------EXPLORE -Yaw----------------------------------#
	
	###---GET ERROR---###
	my_pose = ur_con.get_pose()
	errors = ur_con.check_errors(my_pose)
	rot_error = errors.ee_rot_error
#	print("Rotational error:", (rot_error),"degrees") #Directional error in meters

	###---Explore the +Yaw Axis---###
	command = ur_con.rotate_tool(0, 0, -angle) #Roll,Pitch, Yaw
	ur_script.publish(command) # The Amount the robot will move and which direction
	time.sleep(2)
#	print("-Yaw:", -angle, "radian")
	
	###--- Check ROTATIONAL Error ---###
	my_pose = ur_con.get_pose() # Check Pose for better error check
	errors = ur_con.check_errors(my_pose)
	rot2_err = errors.ee_rot_error
#	print("Rotational error:", (rot2_err),"degrees") #Directional error in meters
	
	##---ADD VALUE TO LIST---###
	###### HEEEEEEEEEELP
	print("-Yaw: ",rot2_err, "degrees")
	rot_error_values.append(rot2_err)
#	print("-Yaw",rot_error_values)

###------------------------------RETURN TO INITIAL POSITION----------------------------------###
	
	###---Return to initial---###
	my_pose = ur_con.get_pose # Check Pose for better error check
	command = ur_con.rotate_tool(0, 0, angle) #Roll,Pitch, Yaw
	ur_script.publish(command) # The Amount the robot will move and which direction
	time.sleep(2)
#	print("+Yaw", angle, "r")
	print("Rotational Exploration Complete!")
	
	return rot_error_values #Populated list [r,-r,p,-p,y,-y] index = next move

#---------------------------------------------------------------------------------------------#

### Need to spit out value associated with axis that produced the highest reduction of error for both functions!!!###

#-------------------Function for classifying the possible catesian actions----------------------#

####May need to define move as a function for cartesian since code is almost identical for all if statements ** Just select the Axis [ie. x/-x]###

def carr_actions(cmove,dist): 
	cmove = cmove + 1 # to ensure correct option is chosen 
	completed_car_actions = [] # Actions taken #Possible moves [x,-x,y,-y,z,-z]
	my_pose = ur_con.get_pose() # Check Pose for better error check

	if cmove == 1:
		
		###---Move in the +X Axis---###
		print ("Moving in the +X direction.")
		
		###---Move---###
		my_pose.position.x += dist
		command = ur_con.generate_move_j(my_pose, pose_msg=True)
		ur_script.publish(command)
		ur_con.tar_check(my_pose)
		
		###---Check Errors---###
		my_pose = ur_con.get_pose() # Check Pose for better error check
		errors = ur_con.check_errors(my_pose)
		cat2_err = errors.ee_trans_error
		print("+X: Cartesian error:", (cat2_err),"m") #Directional error in meters 			
		###---carr actions in list---###
		completed_car_actions.append("+X")
		completed_car_actions.append(thresh_car.mag)
		print("Moved in +X!")
		print(completed_car_actions)

		
	if cmove == 2:
	
		###---Move in the -X Axis---###
		print ("Moving in the -X direction.")
		
		###---Move---###
		my_pose.position.x -= dist
		command = ur_con.generate_move_j(my_pose, pose_msg=True)
		ur_script.publish(command)
		ur_con.tar_check(my_pose)
		
		###---Check Errors---###
		my_pose = ur_con.get_pose() # Check Pose for better error check
		errors = ur_con.check_errors(my_pose)
		cat2_err = errors.ee_trans_error
		print("-X: Cartesian error:", (cat2_err),"m") #Directional error in meters 			
		###---carr actions in list---###
		completed_car_actions.append("-X")
		completed_car_actions.append(thresh_car.mag)
		print("Moved in -X!")
		print(completed_car_actions)


	if cmove == 3:
	
		###---Move in the +Y Axis---###
		print ("Moving in the +Y direction.")
		
		###---Move---###
		my_pose.position.y += dist
		command = ur_con.generate_move_j(my_pose, pose_msg=True)
		ur_script.publish(command)
		ur_con.tar_check(my_pose)
		
		###---Check Errors---###
		my_pose = ur_con.get_pose() # Check Pose for better error check
		errors = ur_con.check_errors(my_pose)
		cat2_err = errors.ee_trans_error
		print("+Y: Cartesian error:", (cat2_err),"m") #Directional error in meters 			
		###---carr actions in list---###
		completed_car_actions.append("+Y")
		completed_car_actions.append(thresh_car.mag)
		print("Moved in +Y!")
		print(completed_car_actions)

	if cmove == 4:
		
		###---Move in the -Y Axis---###
		print ("Moving in the -X direction.")
		
		###---Move---###
		my_pose.position.y -= dist
		command = ur_con.generate_move_j(my_pose, pose_msg=True)
		ur_script.publish(command)
		ur_con.tar_check(my_pose)
		
		###---Check Errors---###
		my_pose = ur_con.get_pose() # Check Pose for better error check
		errors = ur_con.check_errors(my_pose)
		cat2_err = errors.ee_trans_error
		print("-Y: Cartesian error:", (cat2_err),"m") #Directional error in meters 			
		###---carr actions in list---###
		completed_car_actions.append("-Y")
		completed_car_actions.append(thresh_car.mag)
		print("Moved in -Y!")
		print(completed_car_actions)		

	if cmove == 5:
		
		###---Move in the +Z Axis---###
		print ("Moving in the +Z direction.")
		
		###---Move---###
		my_pose.position.z += dist
		command = ur_con.generate_move_j(my_pose, pose_msg=True)
		ur_script.publish(command)
		ur_con.tar_check(my_pose)
		
		###---Check Errors---###
		my_pose = ur_con.get_pose() # Check Pose for better error check
		errors = ur_con.check_errors(my_pose)
		cat2_err = errors.ee_trans_error
		print("+Z: Cartesian error:", (cat2_err),"m") #Directional error in meters 			
		###---carr actions in list---###
		completed_car_actions.append("+Z")
		completed_car_actions.append(thresh_car.mag)
		print("Moved in +Z!")
		print(completed_car_actions)		

	if cmove == 6:
		
		###---Move in the -Z Axis---###
		print ("Moving in the -Z direction.")
		
		###---Move---###
		my_pose.position.z -= dist
		command = ur_con.generate_move_j(my_pose, pose_msg=True)
		ur_script.publish(command)
		ur_con.tar_check(my_pose)
		
		###---Check Errors---###
		my_pose = ur_con.get_pose() # Check Pose for better error check
		errors = ur_con.check_errors(my_pose)
		cat2_err = errors.ee_trans_error
		print("-Z: Cartesian error:", (cat2_err),"m") #Directional error in meters 			
		###---carr actions in list---###
		completed_car_actions.append("-Z")
		completed_car_actions.append(thresh_car.mag)
		print("Moved in -Z!")
		print(completed_car_actions)		
	
	return cat2_err

###---------Function for actions Rotation [pitch,roll,yaw] and respective inverses----------### 

def rmove(rmove,angle):

	rmove = rmove + 1 # to ensure correct option is chosen 
	completed_rot_actions = [] # Actions taken #Possible moves [r,-r,p,-p,y,-y]
	my_pose = ur_con.get_pose() # Check Pose for better error check
	
	if rmove == 1: #+Roll

		###---Move in the -Roll Axis---###
		command = ur_con.rotate_tool(angle, 0, 0) #Roll,Pitch, Yaw
		ur_script.publish(command) # The Amount the robot will move and which direction
		time.sleep(2)
		
		###---Check Errors---###
		my_pose = ur_con.get_pose() # Check Pose for better error check
		errors = ur_con.check_errors(my_pose)
		rot2_err = errors.ee_rot_error
		print("+Roll:", (rot2_err),"degrees") #Rotational error		
		
		
		##Append to list###
		completed_rot_actions.append("+Roll: ")
		completed_rot_actions.append(angle)
		print(completed_rot_actions)
	
	if rmove == 2:#-Roll

		###---Move in the -Roll Axis---###
		command = ur_con.rotate_tool(-angle, 0, 0) #Roll,Pitch, Yaw
		ur_script.publish(command) # The Amount the robot will move and which direction
		time.sleep(2)
		#print("Moved -Roll:", -angle, "radians")
	
		###---Check Errors---###
		my_pose = ur_con.get_pose() # Check Pose for better error check
		errors = ur_con.check_errors(my_pose)
		rot2_err = errors.ee_rot_error
		print("-Roll:", (rot2_err),"degrees") #Rotational error	
	
		###---Append to list---###
		completed_rot_actions.append("-Roll: ")
		completed_rot_actions.append(angle)
		print(completed_rot_actions)
		
	if rmove == 3:#+Pitch

		###---Move in the +Pich Axis---###
		command = ur_con.rotate_tool(0, angle, 0) #Roll,Pitch, Yaw
		ur_script.publish(command) # The Amount the robot will move and which direction
		time.sleep(2)
		#print("Moved +Pitch:", angle, "radians")
		
		###---Check Errors---###
		my_pose = ur_con.get_pose() # Check Pose for better error check
		errors = ur_con.check_errors(my_pose)
		rot2_err = errors.ee_rot_error
		print("+Pitch:", (rot2_err),"degrees") #Rotational error			
		
		###---Append to list---###
		completed_rot_actions.append("+Pitch: ")
		completed_rot_actions.append(angle)
		print(completed_rot_actions)
	
	if rmove == 4:#-Pitch
	
		###---Move in the -Pitch Axis---###
		command = ur_con.rotate_tool(0, -angle, 0) #Roll,Pitch, Yaw
		ur_script.publish(command) # The Amount the robot will move and which direction
		time.sleep(2)
		#print("Moved -Pitch:", -angle, "radians")
		
		
		###---Check Errors---###
		my_pose = ur_con.get_pose() # Check Pose for better error check
		errors = ur_con.check_errors(my_pose)
		rot2_err = errors.ee_rot_error
		print("-Pitch:", (rot2_err),"degrees") #Rotational error			
		###---Append to list---###
		completed_rot_actions.append("-Pitch: ")
		completed_rot_actions.append(angle)
		print(completed_rot_actions)
	
	if rmove == 5:#+Yaw
	
		###---Move in the +Yaw Axis---###
		command = ur_con.rotate_tool(0, 0, angle) #Roll,Pitch, Yaw
		ur_script.publish(command) # The Amount the robot will move and which direction
		time.sleep(2)
		#print("Moved +Yaw:", angle, "radians")
		
		
		###---Check Errors---###
		my_pose = ur_con.get_pose() # Check Pose for better error check
		errors = ur_con.check_errors(my_pose)
		rot2_err = errors.ee_rot_error
		print("+Yaw:", (rot2_err),"degrees") #Rotational error	
		
		###---Append to list---###
		completed_rot_actions.append("+Yaw: ")
		completed_rot_actions.append(angle)
		print(completed_rot_actions)
	
	if rmove == 6:#-Yaw
	
		###---Move in the -Yaw Axis---###
		command = ur_con.rotate_tool(0, 0, -angle) #Roll,Pitch, Yaw
		ur_script.publish(command) # The Amount the robot will move and which direction
		time.sleep(2)
		print("Moved -Yaw:", -angle, "radians")
		
		###---Check Errors---###
		my_pose = ur_con.get_pose() # Check Pose for better error check
		errors = ur_con.check_errors(my_pose)
		rot2_err = errors.ee_rot_error
		print("-Yaw:", (rot2_err),"degrees") #Rotational error	
		
		###---Append to list---###
		completed_rot_actions.append("-Yaw: ")
		completed_rot_actions.append(angle)
		print(completed_rot_actions)
	
	return rot2_err

#-------------------------------------------------Main Code-------------------------------------:

#------------------------------------------Put Robot in Home Position---------------------------:

#Set Robot to home position

my_pose = ur_con.get_pose() # Get Robot Pose
command = home_robot() # Robot to default home position
ur_script.publish(command) # Send the command message to the manipulator
time.sleep(10)
my_pose = ur_con.get_pose() # After each movement command, Get Robot Pose for Accuracy

print("Robot sent home")
###--------------------------------------------VARIABLES--------------------------------------###

###-----------------------------------------ERROR CHECKING------------------------------------###

# Check Pose for better error check
my_pose = ur_con.get_pose() 
errors = ur_con.check_errors(my_pose)
# Check for rotational error
rot_error = errors.ee_rot_error 
#Checking cartesian Error
car_err = errors.ee_trans_error 
##----Ball caught Bool----###
ball_caught = errors.ball_caught
#print("Is Ball Caught!?",ball_caught)
#print("Cartesian Error: ",car_err)
#print("Rotational Error: ",rot_error)		

###---Cartesian Loop---###

###if car_err > 0.05:
while car_err > 0.05:
	thresh_car()
	print("Exploring" , thresh_car.mag , "m in every axis")
	explore_c(thresh_car.mag)
	min_cvalue = min(explore_c.car_error_values) # get minimum value of the list 
	min_cindex = explore_c.car_error_values.index(min_cvalue)
	print("list position index: []", min_cindex , " Number in the list" ,min_cindex + 1)
	new_err = carr_actions(min_cindex,thresh_car.mag)
	if new_err < 0.05:
		# Check Pose for better error check
		my_pose = ur_con.get_pose() 
		errors = ur_con.check_errors(my_pose)
		# Check for rotational error
		rot_error = errors.ee_rot_error 
		#Checking cartesian Error
		car_err = errors.ee_trans_error 
		##----Ball caught Bool----###
		ball_caught = errors.ball_caught
		print("Cartesian Error: ",car_err,"Cartesian Threshold MET!")
		print(rot_error)
		print(ball_caught)	
'''
###if car_err > 0.05 and rot_error > 5:	
while rot_error > 5:
	my_pose = ur_con.get_pose()
	errors = ur_con.check_errors(my_pose)
	#Rotational error
	rot_error = errors.ee_rot_error 
	#Checking cartesian Error
	car_err = errors.ee_trans_error 
	##----Ball caught Bool----###
	ball_caught = errors.ball_caught
	angle = thresh_rot()
	print("Rotational Error: ",rot_error)
	print("Exploring" , angle , "Radians in every axis")
	rot_err_list = explore_r()
	print(rot_err_list)
	min_rvalue = min(rot_err_list) # get minimum value of the list 
	min_rindex = rot_err_list.index(min_rvalue)
	print("list position index: []", min_rindex , " Number in the list: " ,min_rindex + 1)
	#print("moving")
	rot_err = rmove(min_rindex,angle)
	print("Moved!")
	#time.sleep(2)
	if rot_err < 5:
		###--- Print---###
		print("Final Rotational Error: ",rot_err)
		print("Rotational Threshold MET!")
		print("Ball caught", ball_caught)
		break
'''


'''
	my_pose = ur_con.get_pose()
	errors = ur_con.check_errors(my_pose)
	#Rotational error
	rot_error = errors.ee_rot_error 
	#Checking cartesian Error
	car_err = errors.ee_trans_error 
	##----Ball caught Bool----###
	ball_caught = errors.ball_caught
	angle = thresh_rot()
	print("Rotational Error: ",rot_error)
	print("Exploring" , angle , "Radians in every axis")
	rot_err_list = explore_r()
	print(rot_err_list)
	min_rvalue = min(rot_err_list) # get minimum value of the list 
	min_rindex = rot_err_list.index(min_rvalue)
	print("list position index: []", min_rindex , " Number in the list: " ,min_rindex + 1)
	print("moving")
	rot_err = rmove(min_rindex,angle)
	print("Moved!")
	###--- Print---###
	#print("Rotational Error: ",rot_error)
	print("Ball caught", ball_caught)
	#time.sleep(2)
	if rot_err < 5:

		print("Rotational Threshold MET!")
		break
		'''
'''

	my_pose = ur_con.get_pose()
	errors = ur_con.check_errors(my_pose)
	#Rotational error
	rot_error = errors.ee_rot_error 
	#Checking cartesian Error
	car_err = errors.ee_trans_error 
	##----Ball caught Bool----###
	ball_caught = errors.ball_caught
	angle = thresh_rot()
	print("Rotational Error: ",rot_error)
	print("Exploring" , angle , "Radians in every axis")
	rot_err_list = explore_r()
	print(rot_err_list)
	min_rvalue = min(rot_err_list) # get minimum value of the list 
	min_rindex = rot_err_list.index(min_rvalue)
	print("list position index: []", min_rindex , " Number in the list: " ,min_rindex + 1)
	#print("moving")
	rot_err = rmove(min_rindex,angle)
	print("Moved!")
	#time.sleep(2)
	if rot_err < 5:
		###--- Print---###
		print("Final Rotational Error: ",rot_err)
		print("Rotational Threshold MET!")
		print("Ball caught", ball_caught)
		break


'''
#----------------------------------------Runs the Algorithm-------------------------------------

###---Testing Functions---###
'''
#command = home_robot()
#print("Robot sent home")
#move_j_pose(command)
#deg_to_rad(10)
thresh_car()
print("Exploring" , thresh_car.mag , "m in every axis")
#thresh_rot()
explore_c(thresh_car.mag)
#-explore_r()
min_cvalue = min(explore_c.car_error_values) # get minimum value of the list 
min_cindex = explore_c.car_error_values.index(min_cvalue)
print("list position index: []", min_cindex , " Number in the list" ,min_cindex + 1)
carr_actions(min_cindex,thresh_car.mag)
#-print(carr_actions.compled_carr_actions)
#-min_rindex()
#-rot_actions(min_rindex.min_index)
#-print(rot_actions.compled_rot_actions)

my_pose = ur_con.get_pose() # Check Pose for better error check
errors = ur_con.check_errors(my_pose)
##----Ball caught Bool----###
ball_caught = errors.ball_caught
print(ball_caught)
'''

###--------------------------------------LOOP TESTS-------------------------------------###

'''
#command = home_robot()
#print("Robot sent home")
#move_j_pose(command)
#deg_to_rad(10)
thresh_car()
print("Exploring" , thresh_car.mag , "m in every axis")
#thresh_rot()
explore_c(thresh_car.mag)
#-explore_r()
min_cvalue = min(explore_c.car_error_values) # get minimum value of the list 
min_cindex = explore_c.car_error_values.index(min_cvalue)
print("list position index: []", min_cindex , " Number in the list" ,min_cindex + 1)
carr_actions(min_cindex,thresh_car.mag)
#-print(carr_actions.compled_carr_actions)
#-min_rindex()
#-rot_actions(min_rindex.min_index)
#-print(rot_actions.compled_rot_actions)

my_pose = ur_con.get_pose() # Check Pose for better error check
errors = ur_con.check_errors(my_pose)
##----Ball caught Bool----###
ball_caught = errors.ball_caught
print(ball_caught)
'''

""" ######------- Used to code the program ------#####
Sort of Psuedo code 
while E_BC_P.ball_caught != True:
	### LOOP CONDITIONS TO INITIATE ALGORITHM ###
	if car_err_values > 0.05 and rot_err_values > 5: ###If thresholds are above ball catch 		loop is initiated.
	
	###---Go into CARTESIAN LOOP---###
		while car_err > 0.05:
		#Perform the explore_c , qction selector then carr_action
			E_BC_P()# Acquire and print the error values 
			###---get variables for exploration---###
			thresh_car()  #Function to get magnitude based on cartesian error
			###--- Explore ---###
			explore_c(mag)
			###----Return index of the greatest reduction of error---###
			min_index(car_error_values)
			###---use the minimum index to decide what move to take next---### 
			carr_actions(min_index,mag)
			###---Should do until error is below 0.05m---###
			#if cartesian error is lower than desired threshold then
			if car_err < 0.05 and rot_err > 5:
				###---GO INTO ROTATION LOOP---###
				while rot_error > 5:
					#Perform the explore_r , action selector then rot_action
					E_BC_P() # Acquire and print the error values 
					###---get variables for exploration---###
					thresh_rot()  #Get magnitude based on cartesian error
					deg_to_rad(angle) # convert degrees to radians
					###--- Explore ---###
					explore_r(rad)
					###----Return index of the lowest error---###
					min_rindex(car_err_values)
					###---use the minimum index to decide next move ---### 
					rot_actions(min_rindex,rad)
					###---Should do until cart error is below 0.05m---###
					if rot_error_values > 5: #if rotational error is lower 							than desired threshold then Break
						break
				### LOOP CONDITIONS TO BREAK CART###
			if car_err_values < 0.05 and rot_err_values < 5: ### IF met loop is 			stopped.
				break #break main loop	
				
	### LOOP CONDITIONS TO BREAK ALGORITHM ###
	if car_err_values < 0.05 and rot_err_values < 5: ### If thresholds are met ball is 	caught loop is stopped.
		break #break main loop
				
#Potentially needs variable to catch ball a specific number of times. balls_caught += 1	

#print results!	
print("Cartesian and Rotational thresholds have been met!")
print("Ball is caught")

# Return to home position

#Potentially add function or code to enable Endless ball catching or ball catching to a particular amount ie catching the ball 5 times. 
"""

