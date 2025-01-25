#!usr/bin/env python3
# -*- coding: utf-8 -*-
import math

class ArmInverseKinematics:
    def __init__(self, physical_parameters):
        self.upper_arm_length = physical_parameters['upper_arm_length']
        self.lower_arm_length = physical_parameters['lower_arm_length']
        self.shoulder_pitch0 = physical_parameters['shoulder_pitch0']
        self.shoulder_roll0 = physical_parameters['shoulder_roll0']
        self.elbow_pitch0 = physical_parameters['elbow_pitch0']

    def compute_joint_targets_one_arm(self, x, y, z):
        """
        Calcula os ângulos para uma posição alvo (x, y, z) no espaço 3D.
        Retorna os ângulos para as articulações do ombro e cotovelo.
        """
        # Distância total do ombro à posição alvo
        c = math.sqrt(x**2 + y**2 + z**2)

        # Comprimento dos segmentos do braço
        a = self.upper_arm_length
        b = self.lower_arm_length

        # Verificar se o alvo é alcançável
        if c > a + b or c < abs(a - b):
            print("Posição alvo fora do alcance do braço.")
            return {
                'shoulder_pitch': 0,
                'shoulder_roll': 0,
                'elbow_pitch': 0
            }

        # Lei dos cossenos para calcular os ângulos
        gamma = math.acos((a**2 + b**2 - c**2) / (2 * a * b))  # Ângulo do cotovelo
        beta = math.acos((a**2 + c**2 - b**2) / (2 * a * c))   # Ângulo do ombro

        # Ângulo de pitch do ombro
        shoulder_roll = (math.atan2(y, x) - self.shoulder_roll0) * -1
        # Ângulo de pitch do ombro
        shoulder_pitch = (math.atan2( math.sqrt(x**2 + y**2), z) + beta - self.shoulder_pitch0 )-math.pi
        
        # Ângulo de pitch do cotovelo
        elbow_pitch = (math.pi - gamma - self.elbow_pitch0) * -1 

        return {
            'shoulder_pitch': shoulder_pitch,
            'shoulder_roll': shoulder_roll,
            'elbow_pitch': elbow_pitch
        }

    def compute_joint_targets(self, x_left, y_left, z_left, x_right, y_right, z_right):
        """
        Calcula os ângulos para ambos os braços.
        """
        left_arm = self.compute_joint_targets_one_arm(x_left, y_left, z_left)
        right_arm = self.compute_joint_targets_one_arm(x_right, y_right, z_right)

        return {
            'left_arm': left_arm,
            'right_arm': right_arm
        }
