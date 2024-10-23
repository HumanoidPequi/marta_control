import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Funções de rotação
def rotation_matrix_x(theta):
    return np.array([
        [1, 0, 0],
        [0, np.cos(theta), -np.sin(theta)],
        [0, np.sin(theta), np.cos(theta)]
    ])

def rotation_matrix_y(theta):
    return np.array([
        [np.cos(theta), 0, np.sin(theta)],
        [0, 1, 0],
        [-np.sin(theta), 0, np.cos(theta)]
    ])

def rotation_matrix_z(theta):
    return np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta), np.cos(theta), 0],
        [0, 0, 1]
    ])

# Definindo as variáveis de comprimento
# Definir os pontos no espaço
CoM = np.array([0, 0, 0])  # Centro de Massa na origem
torso_to_left_hip = np.array([0.02, 0.037, -0.02])  # Vetor do torso ao quadril esquerdo
upper_leg_length = 0.1
lower_leg_length = 0.1
left_ankle_to_left_foot = np.array([0.0, 0.0105, -0.0335])  # Vetor do tornozelo para o pé esquerdo







# Definir ângulos para todas as juntas (em radianos, todos inicialmente 0)
theta_lhr, theta_lhp, theta_lhy = 0, 0, 0  # Left Hip (Roll, Pitch, Yaw)
theta_rhr, theta_rhp, theta_rhy = 0, 0, 0  # Right Hip (Roll, Pitch, Yaw)
theta_lk, theta_rk = 0, 0  # Left Knee, Right Knee (Pitch)
theta_lar, theta_lap = 0, 0  # Left Ankle (Roll, Pitch)
theta_rar, theta_rap = 0, 0  # Right Ankle (Roll, Pitch)

torso_to_right_hip = np.array([torso_to_left_hip[0], -torso_to_left_hip[1], torso_to_left_hip[2]])  # Vetor do torso ao quadril direito (simétrico)
right_ankle_to_right_foot = np.array([left_ankle_to_left_foot[0], -left_ankle_to_left_foot[1], left_ankle_to_left_foot[2]])  # Vetor do tornozelo para o pé direito (simétrico)

# Calculando a rotação das pernas
# Vetor da left upper leg com rotação
left_upper_leg = rotation_matrix_x(theta_lhr) @ rotation_matrix_y(theta_lhp) @ rotation_matrix_z(theta_lhy) @ np.array([0, 0, -upper_leg_length])
# Vetor da right upper leg com rotação (simétrica à esquerda)
right_upper_leg = rotation_matrix_x(theta_rhr) @ rotation_matrix_y(theta_rhp) @ rotation_matrix_z(theta_rhy) @ np.array([0, 0, -upper_leg_length])
# Vetor da left lower leg com rotação apenas em pitch
left_lower_leg = rotation_matrix_y(theta_lk) @ np.array([0, 0, -lower_leg_length])
# Vetor da right lower leg com rotação apenas em pitch (simétrica à esquerda)
right_lower_leg = rotation_matrix_y(theta_rk) @ np.array([0, 0, -lower_leg_length])
# Vetor do left ankle to foot com rotação
left_ankle_to_foot_rotated = rotation_matrix_x(theta_lar) @ rotation_matrix_y(theta_lap) @ left_ankle_to_left_foot
# Vetor do right ankle to foot com rotação (simétrico)
right_ankle_to_foot_rotated = rotation_matrix_x(theta_rar) @ rotation_matrix_y(theta_rap) @ right_ankle_to_right_foot



# Posição do quadril esquerdo e direito
left_hip = CoM + torso_to_left_hip
right_hip = CoM + torso_to_right_hip
# Posicionamento da perna esquerda
left_knee = left_hip + left_upper_leg
left_ankle = left_knee + left_lower_leg
left_foot = left_ankle + left_ankle_to_foot_rotated
# Posicionamento da perna direita
right_knee = right_hip + right_upper_leg
right_ankle = right_knee + right_lower_leg
right_foot = right_ankle + right_ankle_to_foot_rotated

# Criação do gráfico 3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
# Plotar as posições do robô (perna esquerda)
ax.plot([CoM[0], left_hip[0]], [CoM[1], left_hip[1]], [CoM[2], left_hip[2]], label='Torso to Left Hip', color='r')
ax.plot([left_hip[0], left_knee[0]], [left_hip[1], left_knee[1]], [left_hip[2], left_knee[2]], label='Left Upper Leg', color='g')
ax.plot([left_knee[0], left_ankle[0]], [left_knee[1], left_ankle[1]], [left_knee[2], left_ankle[2]], label='Left Lower Leg', color='b')
ax.plot([left_ankle[0], left_foot[0]], [left_ankle[1], left_foot[1]], [left_ankle[2], left_foot[2]], label='Left Ankle to Foot', color='k')
# Plotar as posições do robô (perna direita)
ax.plot([CoM[0], right_hip[0]], [CoM[1], right_hip[1]], [CoM[2], right_hip[2]], label='Torso to Right Hip', color='r', linestyle='--')
ax.plot([right_hip[0], right_knee[0]], [right_hip[1], right_knee[1]], [right_hip[2], right_knee[2]], label='Right Upper Leg', color='g', linestyle='--')
ax.plot([right_knee[0], right_ankle[0]], [right_knee[1], right_ankle[1]], [right_knee[2], right_ankle[2]], label='Right Lower Leg', color='b', linestyle='--')
ax.plot([right_ankle[0], right_foot[0]], [right_ankle[1], right_foot[1]], [right_ankle[2], right_foot[2]], label='Right Ankle to Foot', color='k', linestyle='--')
# Configurar rótulos e títulos
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('3D Visualization of Both Legs with Joint Angles')
# Legenda
ax.legend()
# Ajustar limites dos eixos
ax.set_xlim([-0.5, 0.5])
ax.set_ylim([-0.5, 0.5])
ax.set_zlim([-0.5, 0.1])
# Mostrar o gráfico
plt.show()

