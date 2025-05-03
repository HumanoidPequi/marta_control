#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import JointState
import sys
import select
import termios
import tty

# Instruções para o usuário
instructions = """
Controle das pernas e braços:
---------------------------
Perna Esquerda:
  w : aumentar x
  x : diminuir x
  a : aumentar y
  d : diminuir y
  s : aumentar z
  c : diminuir z

Perna Direita:
  i : aumentar x
  m : diminuir x
  j : aumentar y
  l : diminuir y
  k : aumentar z
  , : diminuir z

Braço Esquerdo:
  1 : aumentar x
  2 : aumentar y
  3 : aumentar z
Braço Direito:
  7 : aumentar x
  8 : aumentar y
  9 : aumentar z
Para os bracos, apertar '-' para inverter o sentido

Pressione Esc para sair
"""

# Incremento para ajustar as coordenadas
STEP_SIZE = 0.001


class TeleopLegArmControl:
    def __init__(self):
        rospy.init_node('teleop_leg_arm_control')
        self.DIRECTION = 1

        # [0.02, 0.0485, 0.08999999999999986, 0.0, 0.018, -0.04850000000000007, 0.1454999999999999, 0.0, 0.0, 0.0, -0.269, 0.0, 0.0, -0.269]

        # pos_init = [0, 0.0945, -0.1455, 0.0000,  0.0280, -0.1115, -0.1415, 0.0000, 0.0, 0.0, -0.269, 0.0, 0.0, -0.269]
        pos_init = [-0.013, -0.032, -0.255, 0, -0.013, 0.032, -0.255, 0, 0.0, 0.000, -0.2690, 0.0, 0.000, -0.2690] 
        # ['0.012', '-0.032', '-0.146', '0.000', '0.012', '0.032', '-0.146', '0.000', '0.237', '0.000', '-0.126', '0.000', '0.000', '-0.233'] ultima posicao

        # Posições iniciais ajustadas
        self.leg_position = pos_init[0:8]
        self.arm_position = pos_init[8:14]

        # Publisher combinado para pernas e braços
        self.coordinate_publisher = rospy.Publisher('/coordinates_input', JointState, queue_size=10)

        # Configuração do terminal para leitura de teclas
        self.settings = termios.tcgetattr(sys.stdin)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        print(instructions)
        try:
            while not rospy.is_shutdown():
                key = self.get_key()

                # Verificar se a tecla Esc foi pressionada
                if key == '\x1b':  # Código ASCII para Esc
                    print("Saindo do teleop.")
                    break

                # Controles para a perna esquerda
                if key == 'w':
                    self.leg_position[0] += STEP_SIZE  # Aumenta x
                elif key == 'x':
                    self.leg_position[0] -= STEP_SIZE  # Diminui x
                elif key == 'a':
                    self.leg_position[1] += STEP_SIZE  # Aumenta y
                elif key == 'd':
                    self.leg_position[1] -= STEP_SIZE  # Diminui y
                elif key == 's':
                    self.leg_position[2] += STEP_SIZE  # Aumenta z
                elif key == 'c':
                    self.leg_position[2] -= STEP_SIZE  # Diminui z

                # Controles para a perna direita
                elif key == 'i':
                    self.leg_position[4] += STEP_SIZE  # Aumenta x
                elif key == 'm':
                    self.leg_position[4] -= STEP_SIZE  # Diminui x
                elif key == 'j':
                    self.leg_position[5] += STEP_SIZE  # Aumenta y
                elif key == 'l':
                    self.leg_position[5] -= STEP_SIZE  # Diminui y
                elif key == 'k':
                    self.leg_position[6] += STEP_SIZE  # Aumenta z
                elif key == ',':
                    self.leg_position[6] -= STEP_SIZE  # Diminui z

                # Controles para o braço esquerdo
                elif key == '1':
                    self.arm_position[0] += STEP_SIZE * self.DIRECTION  # Aumenta x
                elif key == '2':
                    self.arm_position[1] += STEP_SIZE * self.DIRECTION  # Aumenta y
                elif key == '3':
                    self.arm_position[2] += STEP_SIZE * self.DIRECTION  # Aumenta z

                # Controles para o braço direito
                elif key == '7':
                    self.arm_position[3] += STEP_SIZE * self.DIRECTION  # Aumenta x
                elif key == '8':
                    self.arm_position[4] += STEP_SIZE * self.DIRECTION  # Aumenta y
                elif key == '9':
                    self.arm_position[5] += STEP_SIZE * self.DIRECTION  # Aumenta z

                if key == '-':
                    self.DIRECTION *= -1

                # Publica as novas posições das pernas e braços combinadas
                combined_position = self.leg_position + self.arm_position
                msg = JointState()
                msg.position = combined_position

                # Formatar as posições com 3 casas decimais
                formatted_position = [f"{pos:.3f}" for pos in combined_position]
                # rospy.loginfo("Combined: %s", formatted_position)
                print(f"Combined: {formatted_position}")
                self.coordinate_publisher.publish(msg)

        except Exception as e:
            print(e)

        finally:
            # Restaura as configurações do terminal
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            print("Teleop encerrado.")


if __name__ == '__main__':
    try:
        teleop = TeleopLegArmControl()
        teleop.run()
    except rospy.ROSInterruptException:
        pass
