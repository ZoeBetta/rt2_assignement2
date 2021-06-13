#! /usr/bin/env python

## @package rt2_assignment2
#
#  \file go_to_point.py
#  \brief This file implements the behaviour that allows the robot to reach a goal.
#
#  \author Zoe Betta
#  \version 1.0
#  \date 12/06/2021
#  \details
#  
#  Subscribes to: <BR>
#	 /odom
#    /vel
#
#  Publishes to: <BR>
#	 /cmd_vel 
#
#  Services: <BR>
#    None
#
#  Action Services: <BR>
#    /go_to_point
#
#  Description: <BR>
#    This node allows the robot to reach a position with a given orientation.
#	 Firstly it orients the robot in the direction of the goal and moves towards it.
#	 Once the robot has reached the correct x and y coordinates it rotates to reach
#	 the correct orientation. The velocities, both angular and linear are set by
#	 the node user_interface and are received on the topic /vel. If the goal
# 	 is set cancelled by the client of the action server then the velocities are
#	 all set to zero and the action server is set preempted.  


import rospy
from geometry_msgs.msg import Twist, Point, Pose
from nav_msgs.msg import Odometry
from tf import transformations
import math
import actionlib
import actionlib.msg
import rt2_assignment2.msg
from std_msgs.msg import String, Float64

# robot state variables
position_ = Point()
pose_=Pose()
yaw_ = 0
position_ = 0
state_ = 0
pub_ = None

desired_position_= Point()
desired_position_.z=0
success = False
# parameters for control
yaw_precision_ = math.pi / 9  # +/- 20 degree allowed
yaw_precision_2_ = math.pi / 90  # +/- 2 degree allowed
dist_precision_ = 0.1
kp_a = -3.0 
kp_d = 0.2
ub_a = 0.6
lb_a = -0.5
ub_d = 0.6
Vel=Twist()

#action server
act_s=None

##
#	\brief This function is called when new data are available on the topic /vel
#	\param msg: the data received on the topic /vel
#	\return : None
# 	
#	This function saves the data received by the subscriber on the topic /vel
#	in a global variable of type Twist, Vel, this variable will be used when
#	there is the need to set a new velocity.
def clbk_vel(msg):
    global Vel
    Vel.linear.x=msg.linear.x
    Vel.angular.z=msg.angular.z

##
#	\brief This function is called when new data are available on the topic /odom
#	\param msg: the data received on the topic /odom
#	\return : None
# 	
#	This function saves the data received by the subscriber on the topic /odom
#	in the global variable position for the information about the current position
#	of the robot. It then changes the format of the orientation from quaternions angles
#	to euler angles; it is the extracted the third elemen of the vector and it is saved
#	on the global variable yaw_
def clbk_odom(msg):
    #This is called when new data are available on /odom
    global position_
    global pose_
    global yaw_
	
    # position
    position_ = msg.pose.pose.position

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]

##
#    \brief This function changes the state
#	\param state: the state of the robot
#	\return : None
# 	
#	This function receives the new state and assigns its value to the global 
#	variable states_. Then a message is printed to know which one is the
#	new state.
def change_state(state):
    #Changes the state to a new one
    global state_
    state_ = state
    print ('State changed to [%s]' % state_)

##
#	\brief This function normalizes angles
#	\param angle: the angle I want to normalize
#	\return : angle, the normalized angle
# 	
#	This function normalizes the angle received as input, it doesn't change 
#	the sign but it reduces the magnitude to less than one full circle
def normalize_angle(angle):
    #This makes calculations on angles
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

##
#	\brief This function orients the robot in the beginning
#	\param des_pos: the desired x and y coordinates
#	\return : None
# 	
#	This function calculates the desired orientation to reach the x,y point
#	and set the angular velocity to reach it. If the orientation error is 
#	less than a given threshold then the state is changed to the behaviour
#	go straight.
def fix_yaw(des_pos):
    #This orients the robot in direction of the goal
    global yaw_, pub, yaw_precision_2_, state_,Vel
    des_yaw = math.atan2(desired_position_.y - position_.y, desired_position_.x - position_.x)
    err_yaw = normalize_angle(des_yaw - yaw_)
    rospy.loginfo(err_yaw)
    
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = Vel.angular.z
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = Vel.angular.z
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = Vel.angular.z
    pub_.publish(twist_msg)
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        print ('Yaw error: [%s]' % err_yaw)
        change_state(1)

##
#	\brief This function moves the robot in a straight line
#	\param des_pos: the desired x and y coordinates
#	\return : None
# 	
#	This function calculates the desired orientation to reach the x,y point
#	and the distance between the goal both linear and angular. It then sets
#	the linear velocity. It also set an angular velocity proportional to the
#	error to slightly correct the direction of the line when needed. If the
#	distance between the goal is less than a given threshold the state is changed
#	to the fix final orientation behaviour.
def go_straight_ahead(des_pos):
    #This makes the robot go in a straight line towards the goal
    global yaw_, pub, yaw_precision_, state_,Vel
    des_yaw = math.atan2(desired_position_.y - position_.y, desired_position_.x - position_.x)
    err_yaw = des_yaw - yaw_
    err_pos = math.sqrt(pow(desired_position_.y - position_.y, 2) + pow(desired_position_.x - position_.x, 2))
    err_yaw = normalize_angle(des_yaw - yaw_)
    rospy.loginfo(err_yaw)

    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = Vel.linear.x
        if twist_msg.linear.x > ub_d:
            twist_msg.linear.x = Vel.linear.x

        twist_msg.angular.z = Vel.angular.z*err_yaw
        pub_.publish(twist_msg)
    else: # state change conditions
        print ('Position error: [%s]' % err_pos)
        change_state(2)

    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        print ('Yaw error: [%s]' % err_yaw)
        change_state(0)

##
#	\brief This function orients the robot in the end
#	\param des_yaw: the desired orientation
#	\return : None
# 	
#	This function calculates the error between the current orientation and the 
#	desired one. It then sets the angular velocity to obtain the correct orientation.
#	If the error is less than a given threshold then the state is changed to done.
def fix_final_yaw(des_yaw):
    global Vel
    # It orients the robot in the final desired orientation 
    err_yaw = normalize_angle(des_yaw - yaw_)
    rospy.loginfo(err_yaw)
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = Vel.angular.z
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = Vel.angular.z
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = Vel.angular.z
    pub_.publish(twist_msg)
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(3)
     
##
#	\brief This function stops the robot
#	\param : None
#	\return : None
# 	
#	This function puts to zero all the velocities, angular and linear, and sets
#	the goal as succeeded.  
def done():
    #I put every velocity to zero before setting the goal to completed
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub_.publish(twist_msg)
    success = True
    act_s.set_succeeded()

##
#	\brief This function implements the server behaviour
#	\param goal: the desired position and orientation to obtain
#	\return : None
# 	
#	This function is called when a request to the server is made. It sets 
#	all the global variables needed, then it enteres a while loop. In the while
#	loop it always check if the goal is preempted and if that is the case it
#	sets all the velocities to zero and it set the goal as preempted. If 
#	the action server is not preempted it checks which is the state and it 
#	calls the corresponding function.     
def go_to_point(goal):
    #Implements the logic to go to the goal
    global state_, desired_position_, act_s, success
    desired_position_.x = goal.target_pose.pose.position.x
    desired_position_.y = goal.target_pose.pose.position.y
    des_yaw = goal.target_pose.pose.position.z
    change_state(0)
    while True:
        # if the action is preempted
        if act_s.is_preempt_requested():
            rospy.loginfo('Goal was preempted')
            # I set the velocity to zero
            twist_msg = Twist()
            twist_msg.linear.x = 0
            twist_msg.angular.z = 0
            pub_.publish(twist_msg)
            # I set the goal as preempted
            act_s.set_preempted()
            success=False 
            break
        #if the state is 0 I orient the robot towards the goal    
        elif state_ == 0:
            fix_yaw(desired_position_)
        #if the state is 1 I go straight    
        elif state_ == 1:
            go_straight_ahead(desired_position_)
        #if the state is 2 the robot orients itself in the final orientation    
        elif state_ == 2:
            fix_final_yaw(des_yaw)
        #if the state is 3 I set the goal as achieved    
        elif state_ == 3:
            done()
            break
    return True

##
#	\brief This function implements the ros node
#	\param : None
#	\return : None
# 	
#	This function is called when the program is first requested to run. It
#	initializes all the publishers, subscribers, services and then waits for
#	a request for the action server that should come from the user_interface
#	node.
def main():
    global pub_, active_, act_s
    #I initialize the goal
    rospy.init_node('go_to_point')
    #I initialize the publisher for the velocity
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    #I initialize the subscriber on odometry
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    sub_linVel = rospy.Subscriber('/vel', Twist, clbk_vel)
    #I initialize the action server
    act_s = actionlib.SimpleActionServer('/go_to_point', rt2_assignment2.msg.PlanningAction, go_to_point, auto_start=False)
    act_s.start()
    rospy.spin()

if __name__ == '__main__':
    main()
