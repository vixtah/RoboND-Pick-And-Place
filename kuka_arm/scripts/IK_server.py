#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
import numpy as np

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Initialize service response
        joint_trajectory_list = []

        # Define DH param symbols


        # Joint angle symbols
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') # theta 1  
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

        # Modified DH params
        s = {alpha0:     0,  a0:      0,  d1:  0.75,  q1:       q1,
             alpha1: -pi/2.,  a1:   0.35,  d2:     0,  q2:  q2-pi/2.,
             alpha2:     0,  a2:   1.25,  d3:     0,  q3:       q3,
             alpha3: -pi/2.,  a3: -0.054,  d4:   1.5,  q4:       q4,
             alpha4:  pi/2.,  a4:      0,  d5:     0,  q5:       q5,
             alpha5: -pi/2.,  a5:      0,  d6:     0,  q6:       q6,
             alpha6:     0,  a6:      0,  d7: 0.303,  q7:        0}



        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Define DH param symbols
            q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') # theta 1  
            d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
            a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
            alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

            # Joint angle symbols


            # Modified DH params



            # Define Modified DH Transformation matrix



            # Create individual transformation matrices
            def TF_Matrix(alpha, a, d, q):
                TF = Matrix([[            cos(q),           -sin(q),           0,             a],
                             [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                             [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                             [                 0,                 0,           0,             1]])
                return TF

            T0_1 = TF_Matrix(alpha0,a0,d1,q1).subs(s)
            T1_2 = TF_Matrix(alpha1,a1,d2,q2).subs(s)
            T2_3 = TF_Matrix(alpha2,a2,d3,q3).subs(s)
            T3_4 = TF_Matrix(alpha3,a3,d4,q4).subs(s)
            T4_5 = TF_Matrix(alpha4,a4,d5,q5).subs(s)
            T5_6 = TF_Matrix(alpha5,a5,d6,q6).subs(s)
            T6_G = TF_Matrix(alpha6,a6,d7,q7).subs(s)


            
            # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
     
            # Calculate joint angles using Geometric IK method

	    r, p, y = symbols('r p y')
            X_rot = Matrix([[            1,           0,           0],
                            [            0,   cos(r),  -sin(r)],
                            [            0,   sin(r),   cos(r)]])
            Y_rot = Matrix([[   cos(p),           0,  sin(p)],
                            [            0,           1,           0],
                            [  -sin(p),           0,  cos(p)]])
            Z_rot = Matrix([[     cos(y),   -sin(y),           0],
                            [     sin(y),    cos(y),           0],
                            [            0,           0,           1]])

            rotEE = Z_rot * Y_rot * X_rot
            rotError = Z_rot.subs(y, radians(180)) * Y_rot.subs(p, radians(-90))
	    rotEE = rotEE * rotError
            rotEE = rotEE.subs({'r': roll, 'p': pitch, 'y': yaw})
            w_x = px - .303 * rotEE[0,2]
            w_y = py - .303 * rotEE[1,2]
            w_z = pz - .303 * rotEE[2,2]

            theta1 = atan2(w_y,w_x)

            J2_WC_x = abs(sqrt(w_x**2 + w_y**2) - .35)
            J2_WC_y = abs(w_z - .75)

            A = 1.501
            B = sqrt(J2_WC_x**2 + J2_WC_y**2)
            C = 1.25

            a = acos((-A**2+B**2+C**2)/(2*B*C))
            theta2 = atan(J2_WC_x/J2_WC_y) - a
            b = acos((A**2+C**2-B**2)/(2*A*C))
            theta3 = np.pi/2-b-.036
            print("theta1,2,3: ",theta1, theta2, theta3)
            R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
            R0_3 = R0_3.evalf(subs={
                q1: theta1,
                q2: theta2,
                q3: theta3
            })
            R3_6 = R0_3.inv("LU") * rotEE

            theta4 = atan2(R3_6[2,2],-R3_6[0,2])
            theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]),R3_6[1,2])
            theta6 = atan2(-R3_6[1,1],R3_6[1,0])

            if(sin(theta5))<0:
                theta4 = atan2(-R3_6[2,2],R3_6[0,2])
                theta6 = atan2(R3_6[1,1],-R3_6[1,0])

            print("theta4,5,6: ",theta4, theta5, theta6)


            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            if (len(req.poses) - x > 2):
                joint_trajectory_point.positions = [theta1, theta2, theta3, 0, -pi/2, 0]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
