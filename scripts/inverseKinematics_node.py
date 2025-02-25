#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from InverseKinemeticsArm import ArmInverseKinematics
import rospy
from std_msgs.msg import Float64, Float64MultiArray
from InverseKinematicsLeg import LegInverseKinematics
from sensor_msgs.msg import JointState

class WalkingNode:
    def __init__(self):
        # Inicializa o nó ROS
        rospy.init_node('inverse_kinematics_node')

        # Define a taxa de execução (10 Hz)
        self.rate = rospy.Rate(100)  # 10 Hz


        self.publishers = {
            'left_leg_pub': rospy.Publisher('/marta/left_leg/command', Float64MultiArray, queue_size=10),
            'right_leg_pub': rospy.Publisher('/marta/right_leg/command', Float64MultiArray, queue_size=10),
            'left_arm_pub': rospy.Publisher('/marta/arm_l_head/command', Float64MultiArray, queue_size=10),
            'right_arm_pub': rospy.Publisher('/marta/arm_r/command', Float64MultiArray, queue_size=10)
        }

        # self.publishers = {
        #     'left_leg_pub': rospy.Publisher('/marta/left_leg/state', Float64MultiArray, queue_size=10),
        #     'right_leg_pub': rospy.Publisher('/marta/right_leg/state', Float64MultiArray, queue_size=10),
        #     'left_arm_pub': rospy.Publisher('/marta/arm_l_head/state', Float64MultiArray, queue_size=10),
        #     'right_arm_pub': rospy.Publisher('/marta/arm_r/state', Float64MultiArray, queue_size=10)
        # }

        # Subscrição para receber as coordenadas das pernas
        rospy.Subscriber('/coordinates_input', JointState, self.coordinates_callback)

        # Parâmetros físicos do robô carregados do servidor de parâmetros ROS
        self.physical_parameters = self.load_physical_parameters()

        # Inicializa a cinemática inversa
        self.leg_inverse_kinematics = LegInverseKinematics(self.physical_parameters)
        self.arm_inverse_kinematics = ArmInverseKinematics(self.physical_parameters)

    def coordinates_callback(self, msg):
        # Receber as coordenadas das pernas
        x_left_leg = msg.position[0]
        y_left_leg = msg.position[1]
        z_left_leg = msg.position[2]
        theta_left_leg = msg.position[3]
        
        x_right_leg = msg.position[4]
        y_right_leg = msg.position[5]
        z_right_leg = msg.position[6]
        theta_right_leg = msg.position[7]
        
        # Receber as coordenadas dos braços
        x_left_arm = msg.position[8]
        y_left_arm = msg.position[9]
        z_left_arm = msg.position[10]

        x_right_arm = msg.position[11]
        y_right_arm = msg.position[12]
        z_right_arm = msg.position[13]

        # Calcular os ângulos das articulações usando a cinemática inversa
        joints_leg = self.leg_inverse_kinematics.compute_joints_targets(x_left_leg, y_left_leg, z_left_leg, theta_left_leg, 
                                                                x_right_leg, y_right_leg, z_right_leg, theta_right_leg)
        
        joints_arm = self.arm_inverse_kinematics.compute_joint_targets(x_left_arm, y_left_arm, z_left_arm, 
                                                               x_right_arm, y_right_arm, z_right_arm)

        # Publicar os ângulos calculados nos tópicos corretos
        self.publish_angles_leg(joints_leg)
        self.publish_angles_arm(joints_arm)

    def publish_angles_leg(self, joints):
        # Publicar os ângulos da perna esquerda
        left_leg = joints['left_leg']
        right_leg = joints['right_leg']
        

        left_leg_msg = Float64MultiArray()
        left_leg_msg.data =  [left_leg['hip_yaw'], left_leg['hip_roll'], 
                                left_leg['hip_pitch'], left_leg['knee_pitch'], 
                                left_leg['ankle_pitch'], left_leg['ankle_roll']]
        right_leg_msg = Float64MultiArray()
        right_leg_msg.data = [right_leg['hip_yaw'], right_leg['hip_roll'], 
                                right_leg['hip_pitch'], right_leg['knee_pitch'], 
                                right_leg['ankle_pitch'], right_leg['ankle_roll']]
        
        self.publishers['left_leg_pub'].publish(left_leg_msg)
        self.publishers['right_leg_pub'].publish(right_leg_msg)

        print("left_leg_angles: ", left_leg)
        print("right_leg_angles: ", right_leg)

    def publish_angles_arm(self, joints):
        # Publicar os ângulos do braço esquerdo
        left_arm = joints['left_arm']
        right_arm = joints['right_arm']
        
        left_arm_msg = Float64MultiArray()
        left_arm_msg.data = [0, 0,left_arm['shoulder_pitch'], left_arm['shoulder_roll'], left_arm['elbow_pitch']]
        right_arm_msg = Float64MultiArray()
        right_arm_msg.data = [right_arm['shoulder_pitch'], right_arm['shoulder_roll'], right_arm['elbow_pitch']]

        self.publishers['left_arm_pub'].publish(left_arm_msg)
        self.publishers['right_arm_pub'].publish(right_arm_msg)

        print("left_arm_angles: ", left_arm)
        print("right_arm_angles: ", right_arm)

    def spin(self):
        while not rospy.is_shutdown():
            rospy.sleep(0.01)  # Intervalo para atingir a frequência de 100 Hz

    def load_physical_parameters(self):
        # Carregar os parâmetros do servidor de parâmetros ROS
            parameters = {
            'upper_leg_length': rospy.get_param('/walking_params/upper_leg_length', 0.1),
            'lower_leg_length': rospy.get_param('/walking_params/lower_leg_length', 0.1),
            'hip_pitch0': rospy.get_param('/walking_params/hip_pitch0', 0.0),
            'knee_pitch0': rospy.get_param('/walking_params/knee_pitch0', 0.0),
            'ankle_pitch0': rospy.get_param('/walking_params/ankle_pitch0', 0.0),
            'torso_to_left_hip': rospy.get_param('/walking_params/torso_to_left_hip', [0.007, 0.040, -0.103]),
            'left_ankle_to_left_foot': rospy.get_param('/walking_params/left_ankle_to_left_foot', [0.001, -0.001, -0.038]),

            'upper_arm_length': rospy.get_param('/arm_params/upper_arm_length', 0.1136),
            'lower_arm_length': rospy.get_param('/arm_params/lower_arm_length', 0.1550),
            'shoulder_pitch0': rospy.get_param('/arm_params/shoulder_pitch0', 0.0),
            'shoulder_roll0': rospy.get_param('/arm_params/shoulder_roll0', 0.0),
            'elbow_pitch0': rospy.get_param('/arm_params/elbow_pitch0', 0.0)
            }
            rospy.loginfo(f"Loaded physical parameters: {parameters}")
            return parameters


if __name__ == '__main__':
    try:
        node = WalkingNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
