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
Controle das pernas:
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

Pressione Esc para sair
"""

# Incremento para ajustar as coordenadas
STEP_SIZE = 0.001

class TeleopLegControl:
    def __init__(self):
        rospy.init_node('teleop_leg_control')

        # Posições iniciais
        self.position = [-0.013, -0.032, -0.255, 0, -0.013, 0.032, -0.255, 0]

        # Publisher para enviar as coordenadas
        self.publisher = rospy.Publisher('/coordinates_input', JointState, queue_size=10)

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
                    self.position[0] += STEP_SIZE  # Aumenta x
                elif key == 'x':
                    self.position[0] -= STEP_SIZE  # Diminui x
                elif key == 'a':
                    self.position[1] += STEP_SIZE  # Aumenta y
                elif key == 'd':
                    self.position[1] -= STEP_SIZE  # Diminui y
                elif key == 's':
                    self.position[2] += STEP_SIZE  # Aumenta z
                elif key == 'c':
                    self.position[2] -= STEP_SIZE  # Diminui z

                # Controles para a perna direita
                elif key == 'i':
                    self.position[4] += STEP_SIZE  # Aumenta x
                elif key == 'm':
                    self.position[4] -= STEP_SIZE  # Diminui x
                elif key == 'j':
                    self.position[5] += STEP_SIZE  # Aumenta y
                elif key == 'l':
                    self.position[5] -= STEP_SIZE  # Diminui y
                elif key == 'k':
                    self.position[6] += STEP_SIZE  # Aumenta z
                elif key == ',':
                    self.position[6] -= STEP_SIZE  # Diminui z

                # Publica a nova posição
                msg = JointState()
                msg.position = self.position

                rospy.loginfo("[ %.4f, %.4f, %.4f, %.4f,  %.4f, %.4f, %.4f, %.4f]", msg.position[0], msg.position[1], msg.position[2], msg.position[3], msg.position[4], msg.position[5], msg.position[6], msg.position[7])
            
                self.publisher.publish(msg)

                #rospy.loginfo(f"Nova posição: {self.position}")

        except Exception as e:
            print(e)

        finally:
            # Restaura as configurações do terminal
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            print("Teleop encerrado.")

if __name__ == '__main__':
    try:
        teleop = TeleopLegControl()
        teleop.run()
    except rospy.ROSInterruptException:
        pass
