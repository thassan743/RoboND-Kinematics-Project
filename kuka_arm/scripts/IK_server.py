#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya
# Modified by: Taariq Hassan

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
import numpy as np
import sys
import time

def handle_calculate_IK(req):
    count = 1
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Initialize service response
        joint_trajectory_list = []
        
        # Define DH param symbols
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
        
        # Joint angle symbols
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')

  
        # Modified DH params
        s = {alpha0:      0, a0:      0, d1:  0.75,
             alpha1:  -pi/2, a1:   0.35, d2:     0, q2: q2-pi/2,
             alpha2:      0, a2:   1.25, d3:     0,
             alpha3:  -pi/2, a3: -0.054, d4:   1.5,
             alpha4:   pi/2, a4:      0, d5:     0,
             alpha5:  -pi/2, a5:      0, d6:     0,
             alpha6:      0, a6:      0, d7: 0.303, q7: 0}
        
        # Create individual transformation matrices
        T0_1 = Matrix([[             cos(q1),            -sin(q1),            0,              a0 ],
                       [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1 ],
                       [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1 ],
                       [                   0,                   0,            0,               1 ]])
        T0_1 = T0_1.subs(s)
        
        T1_2 = Matrix([[             cos(q2),            -sin(q2),            0,              a1 ],
                       [ sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2 ],
                       [ sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2 ],
                       [                   0,                   0,            0,               1 ]])
        T1_2 = T1_2.subs(s)
        
        T2_3 = Matrix([[             cos(q3),            -sin(q3),            0,              a2 ],
                       [ sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3 ],
                       [ sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3 ],
                       [                   0,                   0,            0,               1 ]])
        T2_3 = T2_3.subs(s)
        
        T3_4 = Matrix([[             cos(q4),            -sin(q4),            0,              a3 ],
                       [ sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4 ],
                       [ sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4 ],
                       [                   0,                   0,            0,               1 ]])
        T3_4 = T3_4.subs(s)
        
        T4_5 = Matrix([[             cos(q5),            -sin(q5),            0,              a4 ],
                       [ sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5 ],
                       [ sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5 ],
                       [                   0,                   0,            0,               1 ]])
        T4_5 = T4_5.subs(s)
        
        T5_6 = Matrix([[             cos(q6),            -sin(q6),            0,              a5 ],
                       [ sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6 ],
                       [ sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6 ],
                       [                   0,                   0,            0,               1 ]])
        T5_6 = T5_6.subs(s)
        
        T6_G = Matrix([[             cos(q7),            -sin(q7),            0,              a6 ],
                       [ sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7 ],
                       [ sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7 ],
                       [                   0,                   0,            0,               1 ]])
        T6_G = T6_G.subs(s)
        
        #Transform matrix for first three joints
        T0_3 = T0_1 * T1_2 * T2_3
        
        # Total Homogeneous Transform between base_link and gripper_link
        T0_G = T0_3 * T3_4 * T4_5 * T5_6 * T6_G
        
        # Correction to transform gripper URDF frame to DH frame
        R_z = Matrix([[     cos(pi), -sin(pi),          0,  0],
                      [     sin(pi),  cos(pi),          0,  0],
                      [           0,        0,          1,  0],
                      [           0,        0,          0,  1]])
        R_y = Matrix([[  cos(-pi/2),        0, sin(-pi/2),  0],
                      [           0,        1,          0,  0],
                      [ -sin(-pi/2),        0, cos(-pi/2),  0],
                      [           0,        0,          0,  1]])
        R_corr = R_z * R_y
        
        # Corrected Homogeneous Transform
        T_total = T0_G * R_corr
        
        # save current time for measuring IK time
        start_time = time.time()
        
        for x in xrange(0, len(req.poses)):
            #Print status to console, updates pose count and elapsed time
            print 'Calculating IK for pose ', count, '/', len(req.poses), 'Elapsed Time (s): {0:.1f}'.format(time.time()-start_time), '\r',
            if (x == len(req.poses) - 1):
                print ''
            sys.stdout.flush()
            count += 1
            
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            
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
            
            #End effector rotation matrices
            R_x = Matrix([[          1,         0,          0],
                          [          0, cos(roll), -sin(roll)],
                          [          0, sin(roll),  cos(roll)]])
            R_y = Matrix([[ cos(pitch),         0, sin(pitch)],
                          [          0,         1,          0],
                          [-sin(pitch),         0, cos(pitch)]])
            R_z = Matrix([[   cos(yaw), -sin(yaw),          0],
                          [   sin(yaw),  cos(yaw),          0],
                          [          0,         0,          1]])
            
            R0_G = R_x * R_y * R_z
            
            #Calculate wrist centre position
            WC = Matrix([px, py, pz]) - d7 * R0_G * Matrix([1,0,0])
            
            #Calculate theta1 from WC x and y coordinates
            theta1 = atan2(WC[1],WC[0])
            theta1 = theta1.subs(s)
            
            #Calculate new WC position relative to joint2 origin O2 in the plane of the arm.
            WC_x = sqrt(WC[0]**2 + WC[1]**2) - a1
            WC_z = WC[2] - d1
            
            #Calculate the lengths of the sides of the triangle formed by joints j2,j3,j5
            #This is used to calculate theta3 using the cosine rule.
            d_j2j5_2 = WC_x**2 + WC_z**2
            d_j3j5 = sqrt(a3**2 + d4**2)
            
            D = (d_j2j5_2 - a2**2 - d_j3j5**2) / (2 * a2 * d_j3j5)
            
            #Error check for the case when D is greater than 1 resulting in complex numbers
            if D.subs(s) > 1:
                D = 1
            
            #angle between l2 and the line from j3 to j5
            angle = atan2(sqrt(1-D**2), D)
            
            #angle adjustment to account for a3 offset
            adjust = atan2(abs(a3), d4)
            
            # Calculate theta3
            theta3 = angle - adjust - pi/2
            theta3 = theta3.subs(s)
            
            #Calculate theta2
            theta2 = atan2(WC_x, WC_z) - atan2(d_j3j5 * sin(angle), a2 + d_j3j5 * cos(angle))
            theta2 = theta2.subs(s)
            
            #Using the calculated first 3 joint angles, compute the numerical rotation matrix from j0 to j3
            R0_3 = T0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})[0:3, 0:3]
            
            #The rotation matrix of the sperical wrist can be calculated from the
            #known EE rotation and the calculated R0_3 rotation
            R3_6 = R0_3.transpose() * R0_G
            
            #Adjust for the difference in frames of the joints and EE
            yaw_rot = -pi/2
            pitch_rot = -pi/2
            R_y = Matrix([[ cos(pitch_rot), 0, sin(pitch_rot)],
                          [              0, 1,              0],
                          [-sin(pitch_rot), 0, cos(pitch_rot)]])

            R_z = Matrix([[ cos(yaw_rot), -sin(yaw_rot), 0],
                          [ sin(yaw_rot),  cos(yaw_rot), 0],
                          [            0,             0, 1]])
            R3_6 = R3_6 * R_z * R_y
            
            #Calculate the last 3 joint angles using tf library
            theta4, theta5, theta6 = tf.transformations.euler_from_matrix(np.array(R3_6).astype(np.float64), "ryzy")
            
            #Limit joint angles to range of motion defined in the URDF
            dtr = np.pi/180
            theta1 = np.clip(theta1,-185*dtr,185*dtr)
            theta2 = np.clip(theta2, -45*dtr, 85*dtr)
            theta3 = np.clip(theta3,-210*dtr, 65*dtr)
            theta4 = np.clip(theta4,-350*dtr,350*dtr)
            theta5 = np.clip(theta5,-125*dtr,125*dtr)
            theta6 = np.clip(theta6,-350*dtr,350*dtr)

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        print ''
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
