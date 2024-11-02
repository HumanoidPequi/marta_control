#!usr/bin/env python3
# -*- coding: utf-8 -*-
import math
from utils import *

class InverseKinematics:
    def __init__(self, physical_parameters):
        self.upper_leg_length = physical_parameters['upper_leg_length']
        self.lower_leg_length = physical_parameters['lower_leg_length']
        self.hip_pitch0 = physical_parameters['hip_pitch0']
        self.knee_pitch0 = physical_parameters['knee_pitch0']
        self.ankle_pitch0 = physical_parameters['ankle_pitch0']
        self.torso_to_left_hip = physical_parameters['torso_to_left_hip']
        self.left_ankle_to_left_foot = physical_parameters['left_ankle_to_left_foot']
        self.math_utils = MathUtils()

    def compute_joints_targets_one_leg(self, x, y, z, theta):
        # Distância no plano yz
        yz = math.sqrt(y ** 2 + z ** 2)
        # Distância total do quadril ao pé
        c = math.sqrt(x ** 2 + y ** 2 + z ** 2)
        # Comprimento dos segmentos da perna
        a = self.upper_leg_length
        b = self.lower_leg_length

        # print("a=", a,", b=", b, ", c=", c)


        if((a ** 2 + b ** 2 - c ** 2) / (2.0 * a * b) > 1 or (a ** 2 + b ** 2 - c ** 2) / (2.0 * a * b) < -1):
            print("parametros quebram lei dos cossenos, valor=",(a ** 2 + b ** 2 - c ** 2) / (2.0 * a * b))
            return {
            'hip_pitch': 0,
            'knee_pitch': 0,
            'ankle_pitch': 0,
            'hip_roll': 0,
            'ankle_roll': 0,
            'hip_yaw': 0
        }

        

        # Usar a lei dos cossenos para calcular os ângulos
        gamma = math.acos((a ** 2 + b ** 2 - c ** 2) / (2.0 * a * b))  # Ângulo do joelho
        beta = math.acos((a ** 2 + c ** 2 - b ** 2) / (2.0 * a * c))   # Ângulo do quadril
        alpha = math.acos((b ** 2 + c ** 2 - a ** 2) / (2.0 * b * c))  # Ângulo do tornozelo

        # gamma = self.math_utils.normalize_angle_radians(gamma)
        # beta = self.math_utils.normalize_angle_radians(beta)
        # alpha = self.math_utils.normalize_angle_radians(alpha)

        print(" alpha = ", alpha, ", beta = ", beta, ", gamma = ", gamma)
        print(" y = ", y, " z = ", z)

        # Ajustar os ângulos para os ângulos base do robô
        hip_pitch = -(math.atan2(x, yz) + beta) - self.hip_pitch0
        knee_pitch = math.pi - gamma - self.knee_pitch0
        ankle_pitch = math.atan(x/yz) - alpha - self.ankle_pitch0 

        # Calcular os ângulos de rotação lateral
        hip_roll = -math.atan(y/z)
        ankle_roll = -hip_roll

        return {
            'hip_pitch': hip_pitch,
            'knee_pitch': knee_pitch,
            'ankle_pitch': ankle_pitch,
            'hip_roll': hip_roll,
            'ankle_roll': ankle_roll,
            'hip_yaw': theta
        }

    def compute_joints_targets(self, x_left, y_left, z_left, theta_left, 
                               x_right, y_right, z_right, theta_right):
        
        # print("left_leg_init: ", x_left, y_left, z_left, theta_left)
        # print("right_leg_init: ", x_right, y_right, z_right, theta_right)

        # Ajuste com base nas posições físicas do robô
        x_left -= self.torso_to_left_hip[0]
        y_left -= self.torso_to_left_hip[1]
        x_right -= self.torso_to_left_hip[0]
        y_right += self.torso_to_left_hip[1]

        z_left -= self.torso_to_left_hip[2]
        z_right -= self.torso_to_left_hip[2]

        # print()
        # print("left_leg_desloc: ", x_left, y_left, z_left, theta_left)
        # print("right_leg_desloc: ", x_right, y_right, z_right, theta_right)

        # Ajustar a rotação
        x_left, y_left = self._rotate(x_left, y_left, theta_left)
        x_right, y_right = self._rotate(x_right, y_right, theta_right)

        # Ajustar para o tornozelo
        x_left -= self.left_ankle_to_left_foot[0]
        y_left -= self.left_ankle_to_left_foot[1]
        x_right -= self.left_ankle_to_left_foot[0]
        y_right += self.left_ankle_to_left_foot[1]

        z_left -= self.left_ankle_to_left_foot[2]
        z_right -= self.left_ankle_to_left_foot[2]
        
        print()
        # print("left_leg_desloc: ", x_left, y_left, z_left, theta_left)
        # print("right_leg_desloc: ", x_right, y_right, z_right, theta_right)

        # Calcular os ângulos para ambas as pernas
        left_leg = self.compute_joints_targets_one_leg(x_left, y_left, z_left, theta_left)
        right_leg = self.compute_joints_targets_one_leg(x_right, y_right, z_right, theta_right)

        return {
            'left_leg': left_leg,
            'right_leg': right_leg
        }

    def _rotate(self, x, y, theta):
        """Aplica uma rotação no plano 2D"""
        x_new = x * math.cos(theta) + y * math.sin(theta)
        y_new = -x * math.sin(theta) + y * math.cos(theta)
        return x_new, y_new