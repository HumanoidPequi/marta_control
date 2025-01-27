#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
import time

def publish_coordinates(coordinates_matrix, delays):
    pub_leg = rospy.Publisher('/coordinates_input', JointState, queue_size=10)
    pub_arm = rospy.Publisher('/arm_coordinates_input', JointState, queue_size=10)
    rospy.init_node('coordinates_publisher', anonymous=True)
    msg = JointState()
    # msg_arm = JointState()
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
                msg.position = position_interpolated
                pub_leg.publish(msg)
                # msg_arm.position = position_interpolated[8:15]
                rospy.loginfo(f"Publicando: {position_interpolated}")
                pub_arm.publish(msg)
                rospy.sleep(1/rate)
            position_buffered = position

        # Aplica o atraso correspondente
        rospy.sleep(delays[i])

if __name__ == '__main__':
    try:
        # Matriz com todas as coordenadas tanto para a perna quanto para o braço
        coordinates_matrix = [
            [ -0.0130, -0.0320, -0.2550, 0.0000,  -0.0130, 0.0320, -0.2550, 0.0000, 0.0, 0.0, -0.2690, 0.0, 0.0, -0.2690],
            [ -0.0130, -0.0320, -0.2550, 0.0000,  -0.0130, 0.0320, -0.2550, 0.0000, 0.028, 0.0, -0.0590, 0.028, 0.0, -0.0590]

        ]
        # Lista de atrasos entre cada publicação (em segundos)
        delays = [0.2, 4.0]


        publish_coordinates(coordinates_matrix, delays)

    except rospy.ROSInterruptException:
        pass
