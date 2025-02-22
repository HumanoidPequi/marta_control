#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
import time

def publish_coordinates(coordinates_matrix, delays):
    pub_leg = rospy.Publisher('/coordinates_input', JointState, queue_size=10)
    pub_arm = rospy.Publisher('/arm_coordinates_input', JointState, queue_size=10)
    rospy.init_node('coordinates_publisher', anonymous=True)
    msg_leg = JointState()
    msg_arm = JointState()
    rate = 100
    steps = []
    for delay in delays:
        steps.append(rate * delay)
    print(steps)

    # Verifica se o tamanho da lista de atrasos corresponde ao número de publicações
    if len(coordinates_matrix) != len(delays):
        rospy.logerr("A lista de atrasos não corresponde ao número de conjuntos de coordenadas.")
        return

    position_buffered = coordinates_matrix[0]
    for i, position in enumerate(coordinates_matrix):
        if i > 0:
            for j in range(int(steps[i])):
                position_interpolated = [
                    position_buffered[k] + (position[k] - position_buffered[k]) * (j/steps[i]) for k in range(len(position))
                ]
                msg_leg.position = position_interpolated[0:8]
                pub_leg.publish(msg_leg)
                msg_arm.position = position_interpolated[8:15]
                rospy.loginfo(f"Publicando: {position_interpolated}")
                pub_arm.publish(msg_arm)
                rospy.sleep(1/rate)
            position_buffered = position
        if i == 0:
            msg_leg.position = position[0:8]
            pub_leg.publish(msg_leg)
            msg_arm.position = position[8:15]
            rospy.loginfo(f"Publicando: {position}")
            pub_arm.publish(msg_arm)

        # Aplica o atraso correspondente
        # rospy.sleep(delays[i])

if __name__ == '__main__':
    try:
        # Matriz com todas as coordenadas tanto para a perna quanto para o braço
        coordinates_matrix = [
            [ -0.0130, 0.0475, -0.2550, 0.0000,  -0.0130, -0.0475, -0.2550, 0.0000, 0.0, 0.0, -0.2690, 0.0, 0.0, -0.2690],
            [ -0.0130, 0.0475, -0.2550, 0.0000,  -0.0130, -0.0475, -0.2550, 0.0000, 0.028, 0.0, -0.0590, 0.028, 0.0, -0.0590],
            [ -0.0220, 0.0475, -0.1905, 0.0000,  -0.0220, -0.0475, -0.1905, 0.0000, 0.1000, 0.0000, 0.0590, 0.1000, 0.0000, 0.0590],
           # #[-0.0020, 0.1795, -0.0975, 0.0000,  -0.0020, -0.1795, -0.1005, 0.0000, 0.2550, 0.0000, 0.0590, 0.2550, 0.0000, 0.0590],
           # #[-0.0020, 0.1795, -0.0975, 0.0000,  -0.0020, -0.1795, -0.1005, 0.0000,  0.2080, 0.0000, -0.1680, 0.2080, 0.0000, -0.1680],
           # #[-0.0020, 0.1795, -0.0975, 0.0000,  -0.0020, -0.1795, -0.1005, 0.0000, 0.1590, 0.0000, -0.2160, 0.1590, 0.0000, -0.2160],
           # #[0.0330, 0.1795, -0.0975, 0.0000,  0.0280, -0.1795, -0.1005, 0.0000, 0.0, 0.0, -0.269, 0.0, 0.0, -0.269]
           # [0.0010, 0.0475, -0.1575, 0.0000,  0.0010, -0.0475, -0.1575, 0.0000, 0.2550, 0.0000, 0.0590, 0.2550, 0.0000, 0.0590],
           # [0.0140, 0.0475, -0.1575, 0.0000,  0.0140, -0.0475, -0.1575, 0.0000, 0.2080, 0.0000, -0.1680, 0.2080, 0.0000, -0.1680],
           # [0.0310, 0.0475, -0.1365, 0.0000,  0.0310, -0.0475, -0.1365, 0.0000, 0.1590, 0.0000, -0.2160, 0.1590, 0.0000, -0.2160],
           # [0.02, 0.0475, -0.2535, 0.0, 0.02, -0.0475, -0.2535, 0.0, 0.1590, 0.0000, -0.2160, 0.1590, 0.0000, -0.2160] 
        ]
        # Lista de atrasos entre cada publicação (em segundos)
        delays = [2.0, 2.0, 1.0, ]#0.1, 0.1, 1.0, 2.0]

#0.2630, 0.0000, 0.0510, 0.2630, 0.0000, 0.0510 braço reto
#0.0330, 0.0945, -0.1455, 0.0000,  0.0280, -0.1115, -0.1415, 0.0000 perna aberta
#0.0330, 0.1795, -0.0975, 0.0000,  0.0280, -0.1795, -0.1005, 0.0000 perna mais aberta ainda
        publish_coordinates(coordinates_matrix, delays)

    except rospy.ROSInterruptException:
        pass
