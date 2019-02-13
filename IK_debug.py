from sympy import *
from time import time
from mpmath import radians, pi
import tf

'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generated test cases can be added to the test_cases dictionary.
'''

test_cases = {1:[[[2.16135,-1.42635,1.55109],
                  [0.708611,0.186356,-0.157931,0.661967]],
                  [1.89451,-1.44302,1.69366],
                  [-0.65,0.45,-0.36,0.95,0.79,0.49]],
              2:[[[-0.56754,0.93663,3.0038],
                  [0.62073, 0.48318,0.38759,0.480629]],
                  [-0.638,0.64198,2.9988],
                  [-0.79,-0.11,-2.33,1.94,1.14,-3.68]],
              3:[[[-1.3863,0.02074,0.90986],
                  [0.01735,-0.2179,0.9025,0.371016]],
                  [-1.1669,-0.17989,0.85137],
                  [-2.99,-0.12,0.94,4.06,1.29,-4.12]],
              4:[],
              5:[]}

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


def test_code(test_case):
    ## Set up code
    ## Do not modify!
    x = 0
    class Position:
        def __init__(self,EE_pos):
            self.x = EE_pos[0]
            self.y = EE_pos[1]
            self.z = EE_pos[2]
    class Orientation:
        def __init__(self,EE_ori):
            self.x = EE_ori[0]
            self.y = EE_ori[1]
            self.z = EE_ori[2]
            self.w = EE_ori[3]

    position = Position(test_case[0][0])
    orientation = Orientation(test_case[0][1])

    class Combine:
        def __init__(self,position,orientation):
            self.position = position
            self.orientation = orientation

    comb = Combine(position,orientation)

    class Pose:
        def __init__(self,comb):
            self.poses = [comb]

    req = Pose(comb)
    start_time = time()
    
    ########################################################################################
    ## 

    ## Insert IK code here!
    px = req.poses[0].position.x
    py = req.poses[0].position.y
    pz = req.poses[0].position.z
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
        [req.poses[x].orientation.x, req.poses[x].orientation.y,
            req.poses[x].orientation.z, req.poses[x].orientation.w])

    ### Your IK code here
    p_end_effector = N(Matrix([[px], [py], [pz], [1]]))

    # Calculate joint angles using Geometric IK method
    R_rpy_gazebo = N(rot_z(yaw) * rot_y(pitch) * rot_x(roll))
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
    dh_theta2 = - (pi/2 - theta2)
    dh_theta3 = theta3
    R0_3 = N(R0_1.subs({q1: dh_theta1}))\
           * N(R1_2.subs({q2: dh_theta2}))\
           * N(R2_3.subs({q3: dh_theta3}))

    Rrpy_dh = R_rpy_gazebo * R_dh_to_gazebo
    R3_6 = R0_3.inv("LU") * Rrpy_dh[0:3, 0:3]
    print(R3_6)
    print(simplify(3_4 * R4_5 * R5_6))
    theta4, theta5, theta6 = 0, 0, 0

    ## 
    ########################################################################################
    
    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]

    ## (OPTIONAL) YOUR CODE HERE!

    ## End your code input for forward kinematics here!
    ########################################################################################

    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    your_wc = [1,1,1] # <--- Load your calculated WC values in this array
    your_ee = [1,1,1] # <--- Load your calculated end effector value from your forward kinematics
    ########################################################################################

    ## Error analysis
    print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))

    # Find WC error
    if not(sum(your_wc)==3):
        wc_x_e = abs(your_wc[0]-test_case[1][0])
        wc_y_e = abs(your_wc[1]-test_case[1][1])
        wc_z_e = abs(your_wc[2]-test_case[1][2])
        wc_offset = sqrt(wc_x_e**2 + wc_y_e**2 + wc_z_e**2)
        print ("\nWrist error for x position is: %04.8f" % wc_x_e)
        print ("Wrist error for y position is: %04.8f" % wc_y_e)
        print ("Wrist error for z position is: %04.8f" % wc_z_e)
        print ("Overall wrist offset is: %04.8f units" % wc_offset)

    # Find theta errors
    t_1_e = abs(theta1-test_case[2][0])
    t_2_e = abs(theta2-test_case[2][1])
    t_3_e = abs(theta3-test_case[2][2])
    t_4_e = abs(theta4-test_case[2][3])
    t_5_e = abs(theta5-test_case[2][4])
    t_6_e = abs(theta6-test_case[2][5])
    print ("\nTheta 1 error is: %04.8f" % t_1_e)
    print ("Theta 2 error is: %04.8f" % t_2_e)
    print ("Theta 3 error is: %04.8f" % t_3_e)
    print ("Theta 4 error is: %04.8f" % t_4_e)
    print ("Theta 5 error is: %04.8f" % t_5_e)
    print ("Theta 6 error is: %04.8f" % t_6_e)
    print ("\n**These theta errors may not be a correct representation of your code, due to the fact \
           \nthat the arm can have muliple positions. It is best to add your forward kinmeatics to \
           \nconfirm whether your code is working or not**")
    print (" ")

    # Find FK EE error
    if not(sum(your_ee)==3):
        ee_x_e = abs(your_ee[0]-test_case[0][0][0])
        ee_y_e = abs(your_ee[1]-test_case[0][0][1])
        ee_z_e = abs(your_ee[2]-test_case[0][0][2])
        ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
        print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
        print ("End effector error for y position is: %04.8f" % ee_y_e)
        print ("End effector error for z position is: %04.8f" % ee_z_e)
        print ("Overall end effector offset is: %04.8f units \n" % ee_offset)




if __name__ == "__main__":
    # Change test case number for different scenarios
    test_case_number = 1

    test_code(test_cases[test_case_number])
