#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64MultiArray

class MartaStateListener:
    def __init__(self):
        # Armazena os dados mais recentes de cada tópico
        self.left_leg_data = []
        self.right_leg_data = []
        self.left_arm_data = []
        self.right_arm_data = []

        # Subscribers
        rospy.Subscriber("/marta/left_leg/state", Float64MultiArray, self.left_leg_cb)
        rospy.Subscriber("/marta/right_leg/state", Float64MultiArray, self.right_leg_cb)
        rospy.Subscriber("/marta/arm_l_head/state", Float64MultiArray, self.left_arm_cb)
        rospy.Subscriber("/marta/arm_r/state", Float64MultiArray, self.right_arm_cb)

    def left_leg_cb(self, msg):
        self.left_leg_data = msg.data
        self.print_all()

    def right_leg_cb(self, msg):
        self.right_leg_data = msg.data
        self.print_all()

    def left_arm_cb(self, msg):
        self.left_arm_data = msg.data
        self.print_all()

    def right_arm_cb(self, msg):
        self.right_arm_data = msg.data
        self.print_all()

    def print_all(self):
    # Verifica se temos dados em todas as listas
        if self.left_leg_data and self.right_leg_data and self.left_arm_data and self.right_arm_data:
            # 1) Perna Esquerda
            # 2) Perna Direita
            # 3) Braço Esquerdo (pulando os 2 primeiros elementos)
            # 4) Braço Direito

            # Concatena todas as listas em uma só, pulando os 2 primeiros elementos do braço esquerdo
            combined_list = (list(self.left_leg_data)
                                + list(self.right_leg_data)
                                + list(self.left_arm_data)[2:]
                                + list(self.right_arm_data))

            print("Lista combinada [Perna E, Perna D, Braço E (pulados 2), Braço D]:")
            print(combined_list)


def main():
    rospy.init_node("marta_state_listener", anonymous=True)
    MartaStateListener()
    rospy.spin()

if __name__ == "__main__":
    main()
