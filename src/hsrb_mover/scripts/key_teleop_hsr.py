#!/usr/bin/env python

from __future__ import print_function

import rospy

from geometry_msgs.msg import Twist
import trajectory_msgs.msg

import sys
import select
import termios
import tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
        i
   j    k    l
For Head movement:
---------------------------
        w
   a    s    d
For Torso movement:
---------------------------
	f	h
anything else : stop
CTRL-C to quit
"""

moveBindings = {
		'i': (1, 0, 0, 0),
		'j': (0, 0, 0, 1),
		'l': (0, 0, 0, -1),
		'k': (-1, 0, 0, 0),
		'I': (1, 0, 0, 0),
		'J': (0, 1, 0, 0),
		'L': (0, -1, 0, 0),
        'K': (-1, 0, 0, 0),
		 }

headBindings = {
		'w': (1, 0),
		's': (-1, 0),
		'd': (0, -1),
		'a': (0, 1),
	     }

torsoBindings = {
    'f':1.0,
    'h':-1.0,
}


def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key


def moveHead(pan, tilt, pub):
    # fill ROS message

    traj = trajectory_msgs.msg.JointTrajectory()
    traj.joint_names = ["head_pan_joint", "head_tilt_joint"]
    p = trajectory_msgs.msg.JointTrajectoryPoint()
    p.positions = [pan, tilt]
    p.velocities = [0, 0]
    p.time_from_start = rospy.Time(1)
    traj.points = [p]

    # publish ROS message
    pub.publish(traj)

def moveTorso(height,pub):

    traj = trajectory_msgs.msg.JointTrajectory()
    traj.joint_names = ["arm_lift_joint", "arm_flex_joint",
                        "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
    p = trajectory_msgs.msg.JointTrajectoryPoint()
    p.positions = [height, -0.0, -1.57, -1.57 , 0.0]
    p.velocities = [0, 0, 0, 0, 0]
    p.time_from_start = rospy.Time(1)
    traj.points = [p]
    # publish ROS message
    pub.publish(traj)


def vels(speed, turn):
	return "currently:\tspeed %s\tturn %s " % (speed, turn)

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    pub = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=1)
    rospy.init_node('hsr_keyboard_teleop')
    speed = rospy.get_param("~speed", 1.0)
    turn = rospy.get_param("~turn", 0.5)
    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    pub_head = rospy.Publisher('/hsrb/head_trajectory_controller/command',
    trajectory_msgs.msg.JointTrajectory, queue_size=10)

    pan_step = rospy.get_param("~pan_step", 0.25)
    tilt_step = rospy.get_param("~tilt_step", 0.25)

    pan = 0; tilt = 0;

    # wait to establish connection between the controller
    while pub_head.get_num_connections() == 0:
        rospy.sleep(0.1)

    pub_torso = rospy.Publisher('/hsrb/arm_trajectory_controller/command',
                          trajectory_msgs.msg.JointTrajectory, queue_size=10)
    torso = 0;
    torso_step= rospy.get_param("~torso_step",0.1)
    while pub_torso.get_num_connections() == 0:
        rospy.sleep(0.1)

    try:
	print("Starting keyboard Teleop for HSR-Takeshi by Coyo-Soft")
    	print(msg)
    	print(vels(speed, turn))
    	while(1):
    		key = getKey()
    		if key in moveBindings.keys():
    			x = moveBindings[key][0]
    			y = moveBindings[key][1]
    			z = moveBindings[key][2]
    			th = moveBindings[key][3]
    		elif key in headBindings.keys():
		        pan+=headBindings[key][1]*pan_step
		        tilt+=headBindings[key][0]*tilt_step
		        if tilt > 0.47:
		            tilt = 0.47
		        elif tilt < -0.7:
		            tilt = -0.7
		        if pan > 1.570:
		            pan= 1.570
		        elif pan < -1.570:
		            pan= -1.570
			#Stop when turning head.
			x = 0
			y = 0
			z = 0
			th = 0

		elif key in torsoBindings.keys():
			torso+=torsoBindings[key]*torso_step
			if torso> 1.0:
				torso=1.0
			elif torso < 0.0:
				torso=0.0
#			print("debug")
			#Stop when rising torso.
			x = 0
			y = 0
			z = 0
			th = 0
            	else:
    			x = 0
    			y = 0
    			z = 0
    			th = 0
    			if (key == '\x03'):
    				break

    		twist = Twist()
    		twist.linear.x = x * speed;
    		twist.linear.y = y *speed;
            	twist.linear.z = z * speed;
            	twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th * turn
    		pub.publish(twist)
           	moveHead(pan,tilt,pub_head)
		moveTorso(torso,pub_torso)

    except Exception as e:
	print("Error:")
    	print(e)

    finally:
    	twist = Twist()
    	twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    	twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
    	pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
