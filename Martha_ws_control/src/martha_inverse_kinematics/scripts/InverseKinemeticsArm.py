def compute_joint_targets_one_arm(self, x, y, z, is_left=True):
    """
    Calcula os ângulos para uma posição alvo (x, y, z) no espaço 3D.
    `is_left` define se o braço é esquerdo (True) ou direito (False).
    """
    # Inverte o sinal do eixo Y para simetria entre braços
    y = y if is_left else -y

    # Distância total do ombro à posição alvo
    c = math.sqrt(x**2 + y**2 + z**2)

    a = self.upper_arm_length
    b = self.lower_arm_length

    if c > a + b or c < abs(a - b):
        print("Posição alvo fora do alcance do braço.")
        return {
            'shoulder_pitch': 0,
            'shoulder_roll': 0,
            'elbow_pitch': 0
        }

    gamma = math.acos((a**2 + b**2 - c**2) / (2 * a * b))  # cotovelo
    beta = math.acos((a**2 + c**2 - b**2) / (2 * a * c))   # ombro

    # Ângulo de roll do ombro (giro em torno do eixo do braço)
    shoulder_roll = (math.atan2(y, x) - self.shoulder_roll0) * (-1 if is_left else 1)

    # Ângulo de pitch do ombro (levantar o braço)
    shoulder_pitch = (math.atan2(math.sqrt(x**2 + y**2), z) + beta - self.shoulder_pitch0) - math.pi

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
    left_arm = self.compute_joint_targets_one_arm(x_left, y_left, z_left, is_left=True)
    right_arm = self.compute_joint_targets_one_arm(x_right, y_right, z_right, is_left=False)

    return {
        'left_arm': left_arm,
        'right_arm': right_arm
    }
