#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya (project template code) | Braulio Rios (student code) 

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from sympy import *
from mpmath import pi


## Helper rotation functions
def rot_x(angle):
    return N(Matrix([[   1,           0,           0, 0],
                     [   0,  cos(angle), -sin(angle), 0],
                     [   0,  sin(angle),  cos(angle), 0],
                     [   0,           0,           0, 1]]))


def rot_z(angle):
    return N(Matrix([[   cos(angle), -sin(angle), 0, 0],
                     [   sin(angle),  cos(angle), 0, 0],
                     [            0,           0, 1, 0],
                     [            0,           0, 0, 1]]))


def rot_y(angle):
    return N(Matrix([[    cos(angle),  0, sin(angle), 0],
                     [             0,  1,          0, 0],
                     [   -sin(angle),  0, cos(angle), 0],
                     [             0,  0,          0, 1]]))


# Calculates the norm only for the 3 first coordinates (useful for 4d vecs)
def norm3d(vec):
    return Matrix(vec[0:3]).norm()

# Variable DH params (related to joint angles)
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')  # theeta values of dh-params
joint_vals = {q1: 0, q2: -pi/2, q3: 0, q4: 0, q5: 0, q6: 0, q7: 0}  # updated when q changes

# Kuka KR210 constant DH params
a0, a1, a2, a3, a4, a5, a6 = 0, 0.35, 1.25, -0.054, 0, 0, 0
d1 = 0.33 + 0.427  # floor to joint 2 height
d4 = 0.96 + 0.54  # z3 to wrist center horizontal distance
d7 = 0.11 + 0.193  # wrist center to end-effector distance
d2, d3, d5, d6 = 0, 0, 0, 0
p0, p1, p2, p3, p4, p5, p6 = 0, -pi/2, 0, -pi/2, pi/2, -pi/2, 0

# some useful distances
d3_wc = sqrt(a3**2 + d4**2)  # joint 3 to wrist-center distance
d2_3 = a2  # distance from joint 2 to 3

# Transformation matrices (most of them could be very simplified)
T0_1 = Matrix([[cos(q1),         -sin(q1),               0,      a0    ],
               [sin(q1)*cos(p0), cos(q1)*cos(p0), -sin(p0), -sin(p0)*d1],
               [sin(q1)*sin(p0), cos(q1)*sin(p0),  cos(p0), cos(p0)*d1 ],
               [              0,               0,        0,     1      ]])

T1_2 = Matrix([[cos(q2),         -sin(q2),               0,      a1    ],
               [sin(q2)*cos(p1), cos(q2)*cos(p1), -sin(p1), -sin(p1)*d2],
               [sin(q2)*sin(p1), cos(q2)*sin(p1),  cos(p1), cos(p1)*d2 ],
               [              0,               0,        0,     1      ]])

T2_3 = Matrix([[cos(q3),         -sin(q3),               0,      a2    ],
               [sin(q3)*cos(p2), cos(q3)*cos(p2), -sin(p2), -sin(p2)*d3],
               [sin(q3)*sin(p2), cos(q3)*sin(p2),  cos(p2), cos(p2)*d3 ],
               [              0,               0,        0,     1      ]])

T3_4 = Matrix([[cos(q4),         -sin(q4),               0,      a3    ],
               [sin(q4)*cos(p3), cos(q4)*cos(p3), -sin(p3), -sin(p3)*d4],
               [sin(q4)*sin(p3), cos(q4)*sin(p3),  cos(p3), cos(p3)*d4 ],
               [              0,               0,        0,     1      ]])

T4_5 = Matrix([[cos(q5),         -sin(q5),               0,      a4    ],
               [sin(q5)*cos(p4), cos(q5)*cos(p4), -sin(p4), -sin(p4)*d5],
               [sin(q5)*sin(p4), cos(q5)*sin(p4),  cos(p4), cos(p4)*d5 ],
               [              0,               0,        0,     1      ]])

T5_6 = Matrix([[cos(q6),         -sin(q6),               0,      a5    ],
               [sin(q6)*cos(p5), cos(q6)*cos(p5), -sin(p5), -sin(p5)*d6],
               [sin(q6)*sin(p5), cos(q6)*sin(p5),  cos(p5), cos(p5)*d6 ],
               [              0,               0,        0,     1      ]])

T6_7 = Matrix([[cos(q7),         -sin(q7),               0,      a6    ],
               [sin(q7)*cos(p6), cos(q7)*cos(p6), -sin(p6), -sin(p6)*d7],
               [sin(q7)*sin(p6), cos(q7)*sin(p6),  cos(p6), cos(p6)*d7 ],
               [              0,               0,        0,     1      ]])

# Correction matrices from/to Gazebo coords to DH coords
R_gazebo_to_dh = N(simplify(rot_x(pi) * rot_y(pi/2)))
R_dh_to_gazebo = N(simplify(rot_z(pi) * rot_y(-pi/2)))

# Extract rotation matrices from the transformation matrices
R0_1 = T0_1[0:3, 0:3]
R1_2 = T1_2[0:3, 0:3]
R2_3 = T2_3[0:3, 0:3]
R3_4 = T3_4[0:3, 0:3]
R4_5 = T4_5[0:3, 0:3]
R5_6 = T5_6[0:3, 0:3]
R6_7 = T6_7[0:3, 0:3]


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ################### FK code ########

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
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

	    ### Your IK code here
	    p_end_effector = N(Matrix([[px], [py], [pz], [1]]))

	    # Calculate joint angles using Geometric IK method
	    Rrpy_gazebo = N(rot_z(yaw) * rot_y(pitch) * rot_x(roll))
	    versor_x_gazebo = Rrpy_gazebo[:, 0]
	    p_wc = N(p_end_effector - versor_x_gazebo * d7)  # wrist-center position
	    theta1 = atan2(p_wc[1], p_wc[0])
	    # Position of joint-2 (x, y, z and placeholder for 4th coord)
	    p_j2 = Matrix([a1 * cos(theta1), a1 * sin(theta1), d1, 0])
	    vec_j2_to_wc = p_wc - p_j2  # vector from joint-2 to wrist-center
	    d2_wc = norm3d(vec_j2_to_wc)  # distance from joint 2 to wrist-center

	    # Calculate inner angles from triangle between joint 2, joint 3 and wrist-center
	    inner_j2 = acos((d2_3**2 + d2_wc**2 - d3_wc**2)/(2*d2_3*d2_wc))
	    inner_j3 = acos((d2_3**2 + d3_wc**2 - d2_wc**2)/(2*d2_3*d3_wc))

	    # Having triangle inner angles, calculate needed joint angles
	    angle_wc_j2 = asin(vec_j2_to_wc[2]/d2_wc)  # angle between WC and the horiz-plane at j2
	    theta2 = pi/2 - (inner_j2 + angle_wc_j2)  # theta2 = 0 is 90 degrees with horiz-plane
	    angle_offset_wc_j3 = atan2(-a3, d4)
	    theta3 = pi/2 - (inner_j3 + angle_offset_wc_j3)

	    dh_theta1 = theta1
	    dh_theta2 = theta2 - pi/2
	    dh_theta3 = theta3
	    R0_3 = N(R0_1.subs({q1: dh_theta1}))\
		   * N(R1_2.subs({q2: dh_theta2}))\
		   * N(R2_3.subs({q3: dh_theta3}))

	    Rrpy_dh = Rrpy_gazebo * R_dh_to_gazebo
	    R0_3_inv = R0_3.transpose()  # orthonormal matrix: inv() = transpose()
	    R3_6 = R0_3_inv * Rrpy_dh[0:3, 0:3]
	    theta5 = N(atan2(sqrt(R3_6[0, 2]**2 + R3_6[2, 2]**2), R3_6[1, 2]))
	    theta4 = atan2(R3_6[2,2], -R3_6[0,2])
	    theta6 = atan2(-R3_6[1,1],R3_6[1,0])

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
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
