#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
import time

def publish_coordinates(coordinates_matrix, delays):
    pub = rospy.Publisher('/coordinates_input', JointState, queue_size=10)
    rospy.init_node('coordinates_publisher', anonymous=True)
    msg = JointState()

    # Verifica se o tamanho da lista de atrasos corresponde ao número de publicações
    if len(coordinates_matrix) != len(delays):
        rospy.logerr("A lista de atrasos não corresponde ao número de conjuntos de coordenadas.")
        return

    for i, position in enumerate(coordinates_matrix):
        msg.position = position
        rospy.loginfo(f"Publicando: {position}")
        pub.publish(msg)

        # Aplica o atraso correspondente
        rospy.sleep(delays[i])

if __name__ == '__main__':
    try:
        # Matriz com todas as coordenadas
        coordinates_matrix = [
            #[ -0.0030, -0.0320, -0.2210, 0.0000,  -0.0030, 0.0350, -0.2210, 0.0000],
            #[-0.0380, -0.0320, -0.1830, 0.0000, -0.0380, 0.0320, -0.1980, 0.0000],
            #[0.1670, -0.0320, -0.1390, 0.0000, -0.0380, 0.0320, -0.2130, 0.0000],

            [ -0.0130, -0.0320, -0.2480, 0.0000,  -0.0130, 0.0320, -0.2480, 0.0000], 
            [ -0.0130, -0.0320, -0.2380, 0.0000,  -0.0130, 0.0320, -0.2380, 0.0000],
            [ -0.0130, -0.0320, -0.2310, 0.0000,  -0.0130, 0.0330, -0.2310, 0.0000],
            [ -0.0130, -0.0320, -0.2290, 0.0000,  -0.0130, 0.0330, -0.2290, 0.0000],
            #[ -0.0050, -0.0320, -0.2250, 0.0000,  -0.0050, 0.0330, -0.2250, 0.0000],
            #[ -0.0000, -0.0320, -0.2220, 0.0000,  -0.0000, 0.0330, -0.2220, 0.0000],
            #[ 0.0020, -0.0320, -0.2200, 0.0000,  0.0020, 0.0330, -0.2200, 0.0000],
            #[ 0.0030, -0.0320, -0.2200, 0.0000,  0.0030, 0.0330, -0.2200, 0.0000],
            #[ 0.0060, -0.0320, -0.2200, 0.0000,  0.0060, 0.0330, -0.2200, 0.0000],
            [0.1670, -0.0320, -0.1390, 0.0000, -0.0380, 0.0320, -0.2000, 0.0000]  

        ]
        # Lista de atrasos entre cada publicação (em segundos)
        delays = [0.2, 0.2, 0.2, 0.2, 6.0]

        rospy.sleep(20)

        publish_coordinates(coordinates_matrix, delays)

    except rospy.ROSInterruptException:
        pass
