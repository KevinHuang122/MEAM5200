import numpy as np
from math import pi
import math

class FK():

    def __init__(self):

        # TODO: you may want to define geometric parameters here that will be
        # useful in computing the forward kinematics. The data you will need
        # is provided in the lab handout
        self.DH_Param = np.array([[0,  0.141,  0,  0],
                                  [0, 0.192,  0, -pi/2],
                                  [0,   0,    0,  pi/2],
                                  [0, 0.316,  0.0825,  pi/2],
                                  [0,   0,    -0.0825, -pi/2],
                                  [0, 0.384,  0,  pi/2],
                                  [0,   0, 0.088, pi/2],
                                  [0, 0.210,  0,  0]])

    def homogeneousTransform(self, theta, d, a, alpha):

        Rot_z = np.array([[math.cos(theta), -1*math.sin(theta), 0,  0],
                          [math.sin(theta),    math.cos(theta), 0,  0],
                          [     0,                   0,         1,  0],
                          [     0,                   0,         0,  1]])

        Trans_z = np.array([[1, 0, 0, 0],
                            [0, 1, 0, 0],
                            [0, 0, 1, d],
                            [0, 0, 0, 1]])

        Trans_a = np.array([[1, 0, 0, a],
                            [0, 1, 0, 0],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])

        Rot_x = np.array([[     1,              0,                0,          0],
                          [     0,      math.cos(alpha), -1*math.sin(alpha),  0],
                          [     0,      math.sin(alpha),    math.cos(alpha),  0],
                          [     0,              0,                0,          1]])

        ht_matrix = Rot_z @ Trans_z @ Trans_a @ Rot_x 

        return ht_matrix


    def forward(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        jointPositions -8 x 3 matrix, where each row corresponds to a rotational joint of the robot or end effector
                  Each row contains the [x,y,z] coordinates in the world frame of the respective joint's center in meters.
                  The base of the robot is located at [0,0,0].
        T0e       - a 4 x 4 homogeneous transformation matrix,
                  representing the end effector frame expressed in the
                  world frame
        """

        # Your Lab 1 code starts here
                             
        jointPositions = np.zeros((8,3))
        T = np.identity(4)
        jointPositions[0, :] = [0,0,0] #base position
        self.DH_Param[1:8, 0] = q

        for i in range(0, 8):
            theta, d, a, alpha = self.DH_Param[i]

            T = T @ self.homogeneousTransform(theta, d, a, alpha) # Calculate T(i-1)i

            #add the offset
            if i == 2: #joint 3
                T2 = T @ np.array([[1, 0, 0, 0],
                                   [0, 1, 0, 0],
                                   [0, 0, 1, 0.195],
                                    [0, 0, 0, 1]])
                jointPositions[i,:] = T2[:3, -1]

            elif i == 4: #joint 5
                T4 = T @np.array([[1, 0, 0, 0],
                                  [0, 1, 0, 0],
                                  [0, 0, 1, 0.125],
                                  [0, 0, 0, 1]])
                jointPositions[i,:] = T4[:3, -1]

            elif i == 5: #joint 6
                T5 = T @np.array([[1, 0, 0, 0],
                                  [0, 1, 0, 0],
                                  [0, 0, 1, 0.015],
                                  [0, 0, 0, 1]])
                jointPositions[i,:] = T5[:3, -1]
            
            else:
                jointPositions[i, :] = T[:3, -1]
                
        T0e = T

        # Your code ends here

        return jointPositions, T0e

    # feel free to define additional helper methods to modularize your solution for lab 1

    
    # This code is for Lab 2, you can ignore it ofr Lab 1
    def get_axis_of_rotation(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        axis_of_rotation_list: - 3x7 np array of unit vectors describing the axis of rotation for each joint in the
                                 world frame

        """
        # STUDENT CODE HERE: This is a function needed by lab 2

        return()
    
    def compute_Ai(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        Ai: - 4x4 list of np array of homogenous transformations describing the FK of the robot. Transformations are not
              necessarily located at the joint locations
        """
        # STUDENT CODE HERE: This is a function needed by lab 2

        return()
    
if __name__ == "__main__":

    fk = FK()

    # matches figure in the handout
    q = np.array([0,0,0,-pi/2,0,pi/2,pi/4])

    joint_positions, T0e = fk.forward(q)
    
    print("Joint Positions:\n",joint_positions)
    print("End Effector Pose:\n",T0e)
