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

# arm: 0.1590, 0.0000, -0.2160, 0.1590, 0.0000, -0.2160

class TeleopLegArmControl:
    def __init__(self):
        rospy.init_node('teleop_leg_arm_control')
        self.DIRECTION = 1

        # Posições iniciais
        #self.leg_position = [0.02, 0.0475, -0.2535, 0.0, 0.02, -0.0475, -0.2535, 0.0]
        #self.arm_position = [0.0, 0.0, -0.269, 0.0, 0.0, -0.269]
        self.leg_position = [0.1130, 0.0475, -0.1395, 0.0000,  0.1140, -0.0475, -0.1395, 0.0000]
        self.arm_position = [0.1590, 0.0000, -0.2160, 0.1590, 0.0000, -0.2160]

        # Publishers para enviar as coordenadas
        self.leg_publisher = rospy.Publisher('/coordinates_input', JointState, queue_size=10)
        self.arm_publisher = rospy.Publisher('/arm_coordinates_input', JointState, queue_size=10)

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
                    self.arm_position[0] += STEP_SIZE * self.DIRECTION # Aumenta x
                elif key == '2':
                    self.arm_position[1] += STEP_SIZE * self.DIRECTION # Aumenta y
                elif key == '3':
                    self.arm_position[2] += STEP_SIZE * self.DIRECTION # Aumenta z

                # Controles para o braço direito
                elif key == '7':
                    self.arm_position[3] += STEP_SIZE * self.DIRECTION # Aumenta x
                elif key == '8':
                    self.arm_position[4] += STEP_SIZE * self.DIRECTION # Aumenta y
                elif key == '9':
                    self.arm_position[5] += STEP_SIZE * self.DIRECTION # Aumenta z
                    
                if key == '-':
                    self.DIRECTION *= -1

                # Publica a nova posição das pernas
                leg_msg = JointState()
                leg_msg.position = self.leg_position
                rospy.loginfo("Leg: [ %.4f, %.4f, %.4f, %.4f,  %.4f, %.4f, %.4f, %.4f]", *self.leg_position)
                self.leg_publisher.publish(leg_msg)

                # Publica a nova posição dos braços
                arm_msg = JointState()
                arm_msg.position = self.arm_position
                rospy.loginfo("Arm: [ %.4f, %.4f, %.4f, %.4f, %.4f, %.4f]", *self.arm_position)
                self.arm_publisher.publish(arm_msg)

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
