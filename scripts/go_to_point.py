#! /usr/bin/env python
"""
.. module:: go_to_point
    :platform: Unix
    :synopsis: This file implements the behaviour that allows the robot to reach a goal.

.. moduleauthor:: Zoe Betta

This node allows the robot to reach a position with a given orientation.
Firstly it orients the robot in the direction of the goal and moves towards it.
Once the robot has reached the correct x and y coordinates it rotates to reach
the correct orientation. The velocities, both angular and linear are set by
the node *user_interface* and are received on the topic */vel*. If the goal
is set cancelled by the client of the action server then the velocities are
all set to zero and the action server is set preempted.  
 
Subscribes to:

/odom topic where the simulator publishes the robot position

/vel where the *user_interface* publishes the requested velocities

Publishes to:

/cmd_vel the desired robot velocity, it depends on the state

Service :

/go_to_point to start the robot motion.
"""

import rospy
from geometry_msgs.msg import Twist, Point, Pose
from nav_msgs.msg import Odometry
import math
import actionlib
import actionlib.msg
import rt2_assignment2.msg
from std_msgs.msg import String, Float64

# robot state variables
position_ = Point()
"""Point: actual robot position
"""
pose_=Pose()
"""Pose: actual robot orientation
"""
yaw_ = 0
"""Float: actual robot angle
"""
position_ = 0
state_ = 0
"""Int: state of the server
"""
pub_ = None
"""To publish on the */cmd_vel* topic
"""
desired_position_= Point()
"""Point: desired robot position
"""
desired_position_.z=0
"""Float: desired robot orientation
"""
success = False
"""Bool: to store wheater the goal has been reached
"""
yaw_precision_ = math.pi / 9 
"""Float: +/- 20 degrees of precision allowed
"""
yaw_precision_2_ = math.pi / 90
"""Float: +/- 2 degrees of precision allowed
"""
dist_precision_ = 0.1
"""Float: precision of the linear distace allowed
"""
kp_a = -3.0
"""Float: coefficent
""" 
kp_d = 0.2
"""Float: coefficent
"""
ub_a = 0.6
"""Float: coefficent
"""
lb_a = -0.5
"""Float: coefficent
"""
ub_d = 0.6
"""Float: coefficent
"""
Vel=Twist()
"""Twist: the requested velocities
"""

act_s=None
"""To initialize the action server
"""

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    
    Args :
        x,y,z,w (Float): the elements of the quaternion
    
    Returns :
        Roll_x,pitch_y,yaw_z (Float) : the euler angles
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
     
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
     
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
     
    return roll_x, pitch_y, yaw_z # in radians

def clbk_vel(msg):
    """
    This function saves the data received by the subscriber on the topic */vel*
    in a global variable of type Twist, Vel, this variable will be used when
    there is the need to set a new velocity.
    
    Args :
        msg(Twist): the data received on the topic */vel*
    
    Returns :
        None
    """
    global Vel
    Vel.linear.x=msg.linear.x
    Vel.angular.z=msg.angular.z

def clbk_odom(msg):
    """
    This function saves the data received by the subscriber on the topic */odom*
    in the global variable position for the information about the current position
    of the robot. It then changes the format of the orientation from quaternions angles
    to euler angles; it is the extracted the third elemen of the vector and it is saved
    on the global variable *yaw_*
    
    Args :
        msg (Odometry): the data received on the topic */odom*
    
    Returns :
        None
    """
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
    euler = euler_from_quaternion(quaternion[0],quaternion[1],quaternion[2],quaternion[3])
    yaw_ = euler[2]

def change_state(state):
    """
    This function receives the new state and assigns its value to the global 
    variable *states_*. Then a message is printed to know which one is the
    new state.
    
    Args :
        state(Int): the state of the robot
    
    Returns :
        None
    """
    #Changes the state to a new one
    global state_
    state_ = state
    print ('State changed to [%s]' % state_)

def normalize_angle(angle):
    """
    This function normalizes the angle received as input, it doesn't change 
    the sign but it reduces the magnitude to less than one full circle
    
    Args :
        angle(Float): the angle I want to normalize
    
    Returns :
        angle(Float): the normalized angle.
    """
    #This makes calculations on angles
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

def fix_yaw(des_pos):
    """
    This function calculates the desired orientation to reach the x,y point
    and set the angular velocity to reach it. If the orientation error is 
    less than a given threshold then the state is changed to the behaviour
    go straight.
   
    Args :
        des_pos (Point): the desired x and y coordinates
    
    Returns :
        None
    """
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

def go_straight_ahead(des_pos):
    """
	This function calculates the desired orientation to reach the x,y point and the distance between the goal both linear and angular, It then sets the linear velocity, It also set an angular velocity proportional to the error to slightly correct the direction of the line when needed, If the distance between the goal is less than a given threshold the state is changed to the fix final orientation behaviour.
    
    Args :
        des_pos (Point): the desired x and y coordinates
    
    Returns :
        None
    """
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

def fix_final_yaw(des_yaw):
    """
    This function calculates the error between the current orientation and the 
    desired one. It then sets the angular velocity to obtain the correct orientation.
    If the error is less than a given threshold then the state is changed to done.
    
    Args :
        des_yaw(Float): the desired orientation
    
    Returns :
        None
    """
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
      
def done():
    """
    This function puts to zero all the velocities, angular and linear, and sets
    the goal as succeeded.  
    
    Args :
        None
    
    Returns :
        None
    """
    #I put every velocity to zero before setting the goal to completed
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub_.publish(twist_msg)
    success = True
    act_s.set_succeeded()
    
def go_to_point(goal):
    """
    This function is called when a request to the server is made. It sets 
    all the global variables needed, then it enteres a while loop. In the while
    loop it always check if the goal is preempted and if that is the case it
    sets all the velocities to zero and it set the goal as preempted. If 
    the action server is not preempted it checks which is the state and it 
    calls the corresponding function.  
    
    Args :
        goal: the desired position and orientation to obtain
    
    Returns :
        None
    """
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

def main():
    """
    This function is called when the program is first requested to run. It
    initializes all the publishers, subscribers, services and then waits for
    a request for the action server that should come from the user_interface
    node.
    
    Args :
        None
    
    Returns :
        None
    """
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
