#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64, Float64MultiArray

def callback_arm_l_head(msg):
    """
    Expande os valores do tópico /marta/arm_l_head/command para os tópicos individuais.
    """
    topics = [
        "/marta/head_tilt_position/command",
        "/marta/head_pan_position/command",
        "/marta/l_sho_pitch_position/command",
        "/marta/l_sho_roll_position/command",
        "/marta/l_el_position/command"
    ]
    for i, value in enumerate(msg.data):
        pub = rospy.Publisher(topics[i], Float64, queue_size=10)
        pub.publish(value)

def callback_arm_r(msg):
    """
    Expande os valores do tópico /marta/arm_r/command para os tópicos individuais.
    """
    topics = [
        "/marta/r_sho_pitch_position/command",
        "/marta/r_sho_roll_position/command",
        "/marta/r_el_position/command"
    ]
    for i, value in enumerate(msg.data):
        pub = rospy.Publisher(topics[i], Float64, queue_size=10)
        pub.publish(value)

def callback_left_leg(msg):
    """
    Expande os valores do tópico /marta/left_leg/command para os tópicos individuais.
    """
    topics = [
        "/marta/l_hip_yaw_position/command",
        "/marta/l_hip_roll_position/command",
        "/marta/l_hip_pitch_position/command",
        "/marta/l_knee_position/command",
        "/marta/l_ank_pitch_position/command",
        "/marta/l_ank_roll_position/command"
    ]
    for i, value in enumerate(msg.data):
        pub = rospy.Publisher(topics[i], Float64, queue_size=10)
        pub.publish(value)

def callback_right_leg(msg):
    """
    Expande os valores do tópico /marta/right_leg/command para os tópicos individuais.
    """
    topics = [
        "/marta/r_hip_yaw_position/command",
        "/marta/r_hip_roll_position/command",
        "/marta/r_hip_pitch_position/command",
        "/marta/r_knee_position/command",
        "/marta/r_ank_pitch_position/command",
        "/marta/r_ank_roll_position/command"
    ]
    for i, value in enumerate(msg.data):
        pub = rospy.Publisher(topics[i], Float64, queue_size=10)
        pub.publish(value)

def main():
    rospy.init_node('topic_expander', anonymous=True)

    # Subscrições nos tópicos Float64MultiArray
    rospy.Subscriber("/marta/arm_l_head/command", Float64MultiArray, callback_arm_l_head)
    rospy.Subscriber("/marta/arm_r/command", Float64MultiArray, callback_arm_r)
    rospy.Subscriber("/marta/left_leg/command", Float64MultiArray, callback_left_leg)
    rospy.Subscriber("/marta/right_leg/command", Float64MultiArray, callback_right_leg)

    rospy.spin()

if __name__ == '__main__':
    main()
