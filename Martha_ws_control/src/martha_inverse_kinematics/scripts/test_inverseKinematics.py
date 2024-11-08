#!usr/bin/env python3
# -*- coding: utf-8 -*-
from InverseKinematics import InverseKinematics

# Definindo os parâmetros físicos do robô
physical_parameters = {
    'upper_leg_length': 0.1,
    'lower_leg_length': 0.1,
    'hip_pitch0': 0.0,
    'knee_pitch0': 0.0,
    'ankle_pitch0': 0.0,
    'torso_to_left_hip': [0.02, 0.037, -0.02],
    'left_ankle_to_left_foot': [0.0, 0.0105, -0.0335]
}


# esse eh o zero
# rostopic pub /coordinates_input sensor_msgs/JointState "{position: [0.0, -0.032, -0.2235, 0, 0.0, 0.032, -0.2235, 0]}"

# esse eh o chute sla
# rostopic pub /coordinates_input sensor_msgs/JointState "{position: [0.02, 0.0475, 0.2135, 0.0, 0.02, 0.0475, 0.2535, 0.0]}"

# Criando o objeto de cinemática inversa
ik = InverseKinematics(physical_parameters)

# Cálculo dos ângulos das articulações para as duas pernas
joints = ik.compute_joints_targets(
    x_left=0.02, y_left=0.0475, z_left=-0.2535, theta_left=0.0,
    x_right=0.02, y_right=-0.0475, z_right=-0.2535, theta_right=0.0
)

# Resultado
print(joints)
