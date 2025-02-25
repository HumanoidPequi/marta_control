#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
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

        # Posições iniciais ajustadas para as pernas e braços
        self.leg_position = [0.007, 0.040, -0.341, 0.0000,  0.007, -0.040, -0.341, 0.0000]
        self.arm_position = [0.0, 0.0, -0.269, 0.0, 0.0, -0.269]

        # Publisher para as coordenadas combinadas (joints)
        self.coordinate_publisher = rospy.Publisher('/coordinates_input', JointState, queue_size=10)
        # Publisher para os markers que serão exibidos no RViz
        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

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

                # Publica as novas posições combinadas
                combined_position = self.leg_position + self.arm_position
                joint_msg = JointState()
                joint_msg.position = combined_position
                print("Combined:", [f"{pos:.3f}" for pos in combined_position])
                self.coordinate_publisher.publish(joint_msg)

                # Cria e publica marker para a perna esquerda
                marker_left = Marker()
                marker_left.header.frame_id = "base_link"  # Ajuste o frame conforme sua configuração
                marker_left.header.stamp = rospy.Time.now()
                marker_left.ns = "teleop_point_left"
                marker_left.id = 0
                marker_left.type = Marker.SPHERE
                marker_left.action = Marker.ADD
                marker_left.pose.position.x = self.leg_position[0]
                marker_left.pose.position.y = self.leg_position[1]
                marker_left.pose.position.z = self.leg_position[2]
                marker_left.pose.orientation.x = 0.0
                marker_left.pose.orientation.y = 0.0
                marker_left.pose.orientation.z = 0.0
                marker_left.pose.orientation.w = 1.0
                marker_left.scale.x = 0.005
                marker_left.scale.y = 0.005
                marker_left.scale.z = 0.005
                marker_left.color.a = 1.0  # Totalmente opaco
                marker_left.color.r = 1.0
                marker_left.color.g = 0.0
                marker_left.color.b = 0.0  # Vermelho

                # Cria e publica marker para a perna direita
                marker_right = Marker()
                marker_right.header.frame_id = "base_link"  # Ajuste o frame conforme sua configuração
                marker_right.header.stamp = rospy.Time.now()
                marker_right.ns = "teleop_point_right"
                marker_right.id = 1
                marker_right.type = Marker.SPHERE
                marker_right.action = Marker.ADD
                marker_right.pose.position.x = self.leg_position[4]
                marker_right.pose.position.y = self.leg_position[5]
                marker_right.pose.position.z = self.leg_position[6]
                marker_right.pose.orientation.x = 0.0
                marker_right.pose.orientation.y = 0.0
                marker_right.pose.orientation.z = 0.0
                marker_right.pose.orientation.w = 1.0
                marker_right.scale.x = 0.005
                marker_right.scale.y = 0.005
                marker_right.scale.z = 0.005
                marker_right.color.a = 1.0  # Totalmente opaco
                marker_right.color.r = 1.0
                marker_right.color.g = 0.0
                marker_right.color.b = 0.0  # Vermelho

                # Publica ambos os markers
                self.marker_pub.publish(marker_left)
                self.marker_pub.publish(marker_right)

        except Exception as e:
            print(e)

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            print("Teleop encerrado.")

if __name__ == '__main__':
    try:
        teleop = TeleopLegArmControl()
        teleop.run()
    except rospy.ROSInterruptException:
        pass
