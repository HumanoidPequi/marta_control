#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64MultiArray

def publish_default_values():
    # Inicializa o nó ROS
    rospy.init_node('default_state_publisher', anonymous=True)
    
    # Criação dos publishers para os tópicos
    right_leg_pub = rospy.Publisher('/marta/right_leg/state', Float64MultiArray, queue_size=10)
    left_leg_pub = rospy.Publisher('/marta/left_leg/state', Float64MultiArray, queue_size=10)
    arm_r_pub = rospy.Publisher('/marta/arm_r/state', Float64MultiArray, queue_size=10)
    arm_l_head_pub = rospy.Publisher('/marta/arm_l_head/state', Float64MultiArray, queue_size=10)
    
    # Frequência de publicação
    rate = rospy.Rate(10)  # 10 Hz

    # Valores padrão
    right_leg_values = [0, 1, 0, 0, 0, 0]
    left_leg_values = [0, 0, 0.3, 0, 0, 0]
    arm_r_values = [0, 0, 0.2]
    arm_l_head_values = [0, 0, 0, 0, 0]

    while not rospy.is_shutdown():
        # Cria as mensagens
        right_leg_msg = Float64MultiArray()
        left_leg_msg = Float64MultiArray()
        arm_r_msg = Float64MultiArray()
        arm_l_head_msg = Float64MultiArray()

        # Atribui os valores às mensagens
        right_leg_msg.data = right_leg_values
        left_leg_msg.data = left_leg_values
        arm_r_msg.data = arm_r_values
        arm_l_head_msg.data = arm_l_head_values

        # Publica as mensagens
        right_leg_pub.publish(right_leg_msg)
        left_leg_pub.publish(left_leg_msg)
        arm_r_pub.publish(arm_r_msg)
        arm_l_head_pub.publish(arm_l_head_msg)

        # Espera até a próxima iteração
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_default_values()
    except rospy.ROSInterruptException:
        pass
