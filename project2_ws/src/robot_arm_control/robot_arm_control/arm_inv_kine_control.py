#!/usr/bin/env python3

from sympy import *
import sympy as sp
import numpy as np
from scipy import linalg
import scipy as scpy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import sys
import select
import tty
import termios
from numpy.linalg import pinv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from geometry_msgs.msg import PoseStamped
# %matplotlib inline

# from sympy import pi


# pi = np.pi
#Defining the joint variables

# q1,q2,q3,q4,q5 = symbols('theta1,theta2,theta3,theta4,theta5')

#Creating a state matrix "q" from the joint variables
q1 = 0.0
q2 = 1.57
q3 = 0.0
q4 = -1.57
q5 = 0.0

# q = np.array([q1,q2,q3,q4,q5])


def dh_Trans(q):
    
    #Frame 0 to 1
    T_0_1 = Matrix([[cos(q[0]),-sin(q[0])*cos(pi/2),sin(q[0])*sin(pi/2),0],
                    [sin(q[0]),cos(q[0])*cos(pi/2),-cos(q[0])*sin(pi/2),0],
                    [0,sin(pi/2),cos(pi/2),150],
                    [0,0,0,1]])
    #Frame 1 to 2
    T_1_2 = Matrix([[cos(q[1]+pi/2),-sin(q[1]+pi/2)*cos(0),sin(q[1]+pi/2)*sin(0),150*cos(q[1]+pi/2)],
                    [sin(q[1]+pi/2),cos(q[1]+pi/2)*cos(0),-cos(q[1]+pi/2)*sin(0),150*sin(q[1]+pi/2)],
                    [0,sin(0),cos(0),-9.43],
                    [0,0,0,1]])
    #Frame 2 to 3
    T_2_3 = Matrix([[cos(q[2]),-sin(q[2])*cos(0),sin(q[2])*sin(0),100*cos(q[2])],
                    [sin(q[2]),cos(q[2])*cos(0),-cos(q[2])*sin(0),100*sin(q[2])],
                    [0,sin(0),cos(0),0],
                    [0,0,0,1]])
    #Frame 3 to 4
    T_3_4 = Matrix([[cos(q[3]-pi/2),-sin(q[3]-pi/2)*cos(-pi/2),sin(q[3]-pi/2)*sin(-pi/2),0],
                    [sin(q[3]-pi/2),cos(q[3]-pi/2)*cos(-pi/2),-cos(q[3]-pi/2)*sin(-pi/2),0],
                    [0,sin(-pi/2),cos(-pi/2),0],
                    [0,0,0,1]])
    #Frame 4 to 5
    T_4_5 = Matrix([[cos(q[4]),-sin(q[4])*cos(0),sin(q[4])*sin(0),0],
                    [sin(q[4]),cos(q[4])*cos(0),-cos(q[4])*sin(0),0],
                    [0,sin(0),cos(0),119],
                    [0,0,0,1]])
    #Base Frame to EndEffector / 0 to 6
    Tf = T_0_1*T_1_2*T_2_3*T_3_4*T_4_5
    
    return Tf

def dh_Jacobian(q):
    
        #Frame 0 to 1
    T_0_1 = Matrix([[cos(q[0]),-sin(q[0])*cos(pi/2),sin(q[0])*sin(pi/2),0],
                    [sin(q[0]),cos(q[0])*cos(pi/2),-cos(q[0])*sin(pi/2),0],
                    [0,sin(pi/2),cos(pi/2),150],
                    [0,0,0,1]])
    #Frame 1 to 2
    T_1_2 = Matrix([[cos(q[1]+pi/2),-sin(q[1]+pi/2)*cos(0),sin(q[1]+pi/2)*sin(0),150*cos(q[1]+pi/2)],
                    [sin(q[1]+pi/2),cos(q[1]+pi/2)*cos(0),-cos(q[1]+pi/2)*sin(0),150*sin(q[1]+pi/2)],
                    [0,sin(0),cos(0),-9.43],
                    [0,0,0,1]])
    #Frame 2 to 3
    T_2_3 = Matrix([[cos(q[2]),-sin(q[2])*cos(0),sin(q[2])*sin(0),100*cos(q[2])],
                    [sin(q[2]),cos(q[2])*cos(0),-cos(q[2])*sin(0),100*sin(q[2])],
                    [0,sin(0),cos(0),0],
                    [0,0,0,1]])
    #Frame 3 to 4
    T_3_4 = Matrix([[cos(q[3]-pi/2),-sin(q[3]-pi/2)*cos(-pi/2),sin(q[3]-pi/2)*sin(-pi/2),0],
                    [sin(q[3]-pi/2),cos(q[3]-pi/2)*cos(-pi/2),-cos(q[3]-pi/2)*sin(-pi/2),0],
                    [0,sin(-pi/2),cos(-pi/2),0],
                    [0,0,0,1]])
    #Frame 4 to 5
    T_4_5 = Matrix([[cos(q[4]),-sin(q[4])*cos(0),sin(q[4])*sin(0),0],
                    [sin(q[4]),cos(q[4])*cos(0),-cos(q[4])*sin(0),0],
                    [0,sin(0),cos(0),119],
                    [0,0,0,1]])
    
    
    T = [T_0_1,T_0_1*T_1_2,T_0_1*T_1_2*T_2_3,T_0_1*T_1_2*T_2_3*T_3_4,T_0_1*T_1_2*T_2_3*T_3_4*T_4_5]
    
    return T

def cross(M1, M2):
    c = [M1[1] * M2[2] - M1[2] * M2[1],
         M1[2] * M2[0] - M1[0] * M2[2],
         M1[0] * M2[1] - M1[1] * M2[0]]
    return c

#Defining a function to calculate the jacobian

def jacobian(T):

  # Exctracting the Z vectors and appending them to a matrix
    z = [sp.Matrix([0,0,1])]
    for i in T:
        z.append((i[:3,2]))

  # Exctracting the origin vectors and appending them to a matrix
    o =[sp.Matrix([0,0,0])]
    for i in T:
        o.append((i[:3,3]))

  # Inintializing a Jacobian matrix of 6x6 with zeroes
    J = sp.zeros(6, 5)

  # The first three rows of the Jacobian are the cross product of z vectors and difference of end-effector and joint origins
    for i in range(5):
        J[0, i] = sp.Matrix(cross(z[i], [o[-1][0] - o[i][0], o[-1][1] - o[i][1], o[-1][2] - o[i][2]]))

  # The last three rows of the Jacobian are simply the z vectors for rotational joints
    for i in range(5):
        J[3:6, i] = z[i]
    
#     sp.pprint(J)
    
    return J

class ArmControlNode(Node):

    def __init__(self):
        super().__init__('arm_control_node')

        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        self.settings = termios.tcgetattr(sys.stdin)

        self.ef_x = 0.0
        self.ef_y = 0.0 
        self.ef_z = 0.0

        # Subscribe to the /odom topic
        self.subscription = self.create_subscription(
            PoseStamped,
            '/odom',
            self.odom_callback,
            10  # QoS profile, adjust as needed
        )

        self.get_logger().info("Subscribed to /odom topic. Waiting for messages...")

    def odom_callback(self, msg):
        # Extracting X, Y, Z position from PoseStamped message
        self.ef_x = msg.pose.position.x
        self.ef_y = msg.pose.position.y
        self.ef_z = msg.pose.position.z
        
    def run_arm_control(self):

        q1 = 0.0
        q2 = 0.0
        q3 = 0.0
        q4 = 0.0
        q5 = 0.0
        
        ef_x = 0.0
        ef_y = 0.0 
        ef_z = 0.0

        q = sp.Matrix([[q1],[q2],[q3],[q4],[q5]])
        
        joint_positions = Float64MultiArray()

        # joint_positions.data = [q1,1.57,q3,-1.57,q5]

        # self.joint_position_pub.publish(joint_positions)

        def inv_kine(q):
   
            q1 = 0.0
            q2 = 0.0
            q3 = 0.0
            q4 = 0.0
            q5 = 0.0

            q = sp.Matrix([[q1],[q2],[q3],[q4],[q5]])

            target_position = input("Enter three numbers separated by spaces: ")

            # Split the input string by spaces and convert the substrings to floats
            target_position_vector = np.array([float(num) for num in target_position.split()])
            np.append(target_position_vector,[0,0,0])    
            
            # present_ef_trans = dh_Trans(q)
            present_ef_pos = [ef_x, ef_y, ef_z]

            print(present_ef_pos)
            print("\n")
            print(target_position_vector)
            distance_to_move = []

            for i in range(3):
                distance_to_move.append(target_position_vector[i] - present_ef_pos[i])

            print("Distance to move in X direction: ",distance_to_move[0],"\n")
            print("Distance to move in Y direction: ",distance_to_move[1],"\n")
            print("Distance to move in Z direction: ",distance_to_move[2],"\n")

            total_time_to_move = 5
            dt = 0.001
            time_step = np.arange(0,total_time_to_move,dt)

            x_points_to_plot = []
            y_points_to_plot = []
            z_points_to_plot = []

            fig =plt.figure()
            ax = fig.add_subplot(111, projection='3d')

            for i in time_step:
            
                x_dot = (target_position_vector[0] - present_ef_pos[0])/total_time_to_move
                y_dot = (target_position_vector[1] - present_ef_pos[1])/total_time_to_move
                z_dot = (target_position_vector[2] - present_ef_pos[2])/total_time_to_move

                X_dot_bar = sp.Matrix([x_dot, y_dot, z_dot, 0, 0, 0])

                T = dh_Jacobian(q)
                J = jacobian(T)
                # J_inv = np.linalg.pinv(np.array(J,dtype = float))
                
                J_array = np.array(J).astype(float)
                J_inv = pinv(J_array)
                # print(J_inv)    
                q_dot = J_inv * X_dot_bar

                q = q + q_dot * dt
                
                q1 = float(q[0])                           
                q2 = float(q[1])
                q3 = float(q[2])
                q4 = float(q[3])
                q5 = float(q[4])

    
                joint_positions.data = (q1,q2,q3,q4,q5)

                self.joint_position_pub.publish(joint_positions)

                T_ef = dh_Trans(q)

                x_points_to_plot.append(T_ef[0, 3])
                y_points_to_plot.append(T_ef[1, 3])
                z_points_to_plot.append(T_ef[2, 3])
                
                print("X: ", T_ef[0,3], " Y: ", T_ef[1,3], " Z: ", T_ef[2,3])

            ax.plot3D(x_points_to_plot, y_points_to_plot, z_points_to_plot)
            ax.set_title("End-Effector Path")
            ax.set_xlabel('x-axis')
            ax.set_ylabel('y-axis')
            ax.set_zlabel('z-axis')
            plt.show()

            # print("x: ",x_points_to_plot,"\n")
            # print("y: ",y_points_to_plot,"\n")
            # print("z: ",z_points_to_plot,"\n")
        inv_kine(q)
        
def main(args=None):
    rclpy.init(args=args)
    node = ArmControlNode()
    node.run_arm_control()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



