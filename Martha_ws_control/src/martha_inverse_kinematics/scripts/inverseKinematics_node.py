#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64
from InverseKinematics import InverseKinematics
from sensor_msgs.msg import JointState

class WalkingNode:
    def __init__(self):
        # Inicializa o nó ROS
        rospy.init_node('walking_node')

        # Publishers para os ângulos das articulações
        self.publishers = {
            'hip_pitch_left': rospy.Publisher('/martha/l_hip_pitch_position/command', Float64, queue_size=10),
            'knee_pitch_left': rospy.Publisher('/martha/l_knee_position/command', Float64, queue_size=10),
            'ankle_pitch_left': rospy.Publisher('/martha/l_ank_pitch_position/command', Float64, queue_size=10),
            'hip_roll_left': rospy.Publisher('/martha/l_hip_roll_position/command', Float64, queue_size=10),
            'ankle_roll_left': rospy.Publisher('/martha/l_ank_roll_position/command', Float64, queue_size=10),
            'hip_yaw_left': rospy.Publisher('/martha/l_hip_yaw_position/command', Float64, queue_size=10),
            'hip_pitch_right': rospy.Publisher('/martha/r_hip_pitch_position/command', Float64, queue_size=10),
            'knee_pitch_right': rospy.Publisher('/martha/r_knee_position/command', Float64, queue_size=10),
            'ankle_pitch_right': rospy.Publisher('/martha/r_ank_pitch_position/command', Float64, queue_size=10),
            'hip_roll_right': rospy.Publisher('/martha/r_hip_roll_position/command', Float64, queue_size=10),
            'ankle_roll_right': rospy.Publisher('/martha/r_ank_roll_position/command', Float64, queue_size=10),
            'hip_yaw_right': rospy.Publisher('/martha/r_hip_yaw_position/command', Float64, queue_size=10),
        }

        # Subscrição para receber as coordenadas das pernas
        rospy.Subscriber('/coordinates_input', JointState, self.coordinates_callback)

        # Parâmetros físicos do robô carregados do servidor de parâmetros ROS
        self.physical_parameters = self.load_physical_parameters()

        # Inicializa a cinemática inversa
        self.inverse_kinematics = InverseKinematics(self.physical_parameters)

    def coordinates_callback(self, msg):
        # Receber as coordenadas das pernas
        x_left = msg.position[0]
        y_left = msg.position[1]
        z_left = msg.position[2]
        theta_left = msg.position[3]
        
        x_right = msg.position[4]
        y_right = msg.position[5]
        z_right = msg.position[6]
        theta_right = msg.position[7]

        # Calcular os ângulos das articulações usando a cinemática inversa
        joints = self.inverse_kinematics.compute_joints_targets(x_left, y_left, z_left, theta_left, 
                                                                x_right, y_right, z_right, theta_right)

        # Publicar os ângulos calculados nos tópicos corretos
        self.publish_angles(joints)

    def publish_angles(self, joints):
        # Publicar os ângulos da perna esquerda
        left_leg = joints['left_leg']
        self.publishers['hip_pitch_left'].publish(left_leg['hip_pitch'])
        self.publishers['knee_pitch_left'].publish(left_leg['knee_pitch'])
        self.publishers['ankle_pitch_left'].publish(left_leg['ankle_pitch'])
        self.publishers['hip_roll_left'].publish(left_leg['hip_roll'])
        self.publishers['ankle_roll_left'].publish(left_leg['ankle_roll'])
        self.publishers['hip_yaw_left'].publish(left_leg['hip_yaw'])

        # Publicar os ângulos da perna direita
        right_leg = joints['right_leg']
        self.publishers['hip_pitch_right'].publish(right_leg['hip_pitch'])
        self.publishers['knee_pitch_right'].publish(right_leg['knee_pitch'])
        self.publishers['ankle_pitch_right'].publish(right_leg['ankle_pitch'])
        self.publishers['hip_roll_right'].publish(right_leg['hip_roll'])
        self.publishers['ankle_roll_right'].publish(right_leg['ankle_roll'])
        self.publishers['hip_yaw_right'].publish(right_leg['hip_yaw'])

        print("left_leg_angles: ", left_leg)
        print("right_leg_angles: ", right_leg)

    def spin(self):
        rospy.spin()

    def load_physical_parameters(self):
        # Carregar os parâmetros do servidor de parâmetros ROS
            parameters = {
            'upper_leg_length': rospy.get_param('/walking_params/upper_leg_length', 0.1),
            'lower_leg_length': rospy.get_param('/walking_params/lower_leg_length', 0.1),
            'hip_pitch0': rospy.get_param('/walking_params/hip_pitch0', 0.0),
            'knee_pitch0': rospy.get_param('/walking_params/knee_pitch0', 0.0),
            'ankle_pitch0': rospy.get_param('/walking_params/ankle_pitch0', 0.0),
            'torso_to_left_hip': rospy.get_param('/walking_params/torso_to_left_hip', [0.02, 0.037, -0.02]),
            'left_ankle_to_left_foot': rospy.get_param('/walking_params/left_ankle_to_left_foot', [0.0, 0.0105, -0.0335]),
            'period': rospy.get_param('/walking_params/period', 0.5),
            'doubleSupportRatio': rospy.get_param('/walking_params/doubleSupportRatio', 0.2),
            'zCom': rospy.get_param('/walking_params/zCom', 0.25),
            'zStep': rospy.get_param('/walking_params/zStep', 0.05),
            'armCompensationFactor': rospy.get_param('/walking_params/armCompensationFactor', 1.0),
            'comSwingFactor': rospy.get_param('/walking_params/comSwingFactor', 0.5)
            }
            rospy.loginfo(f"Loaded physical parameters: {parameters}")
            return parameters


if __name__ == '__main__':
    try:
        node = WalkingNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
