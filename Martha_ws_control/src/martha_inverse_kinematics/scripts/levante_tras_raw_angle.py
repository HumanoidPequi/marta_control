#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
import time

def publish_coordinates(coordinates_matrix, delays):
    pub_left_leg = rospy.Publisher('/marta/left_leg/command', Float64MultiArray, queue_size=10)
    pub_right_leg = rospy.Publisher('/marta/right_leg/command', Float64MultiArray, queue_size=10)
    pub_arm_l_head = rospy.Publisher('/marta/arm_l_head/command', Float64MultiArray, queue_size=10)
    pub_arm_r = rospy.Publisher('/marta/arm_r/command', Float64MultiArray, queue_size=10)
    
    rospy.init_node('coordinates_publisher', anonymous=True)
    
    rate = 100
    steps = [int(rate * delay) for delay in delays]
    print(steps)

    if len(coordinates_matrix) != len(delays):
        rospy.logerr("A lista de atrasos não corresponde ao número de conjuntos de coordenadas.")
        return

    position_buffered = coordinates_matrix[0]
    for i, position in enumerate(coordinates_matrix):
        
        for j in range(steps[i]):
            position_interpolated = [
                position_buffered[k] + (position[k] - position_buffered[k]) * (j / steps[i]) 
                for k in range(len(position))
            ]
            
            msg_left_leg = Float64MultiArray(data=position_interpolated[0:6])
            msg_right_leg = Float64MultiArray(data=position_interpolated[6:12])
            msg_arm_l_head = Float64MultiArray(data=[0, 0] + position_interpolated[12:15])
            msg_arm_r = Float64MultiArray(data=position_interpolated[15:18])
            
            pub_left_leg.publish(msg_left_leg)
            pub_right_leg.publish(msg_right_leg)
            pub_arm_l_head.publish(msg_arm_l_head)
            pub_arm_r.publish(msg_arm_r)
            
            # rospy.loginfo(f"Publicando: {position_interpolated}")
            rospy.sleep(1 / rate)
        position_buffered = position
        
            

if __name__ == '__main__':
    
    # rospy.sleep(10.0)
    try:
        coordinates_matrix = [
            
            #0.0
            #posicao inicial
            [0, 0, 0, 0, 0, 0, #perna esquerda
             0, 0, 0, 0, 0, 0, #perna direita
             0, 0, 0, #braço esquerdo
             0, 0, 0], #braço direito
            
            #0.1
            #coloca os bracos retraidos junto ao corpo
            [0, 0, 0, 0, 0, 0, 
             0, 0, 0, 0, 0, 0, 
             1.2983921955579856, -0.0, -2.625252736660332,
             1.2983921955579856, -0.0, -2.625252736660332,
             ],
            
            
            #0.2
            #estica braco esquerdo para tras e joga perna esquerda para frente
            [0.0 , 0.0 , -1.6461875707843674 , 1.0116060284533992 , 0.6345815423309678 , -0.0,
             0, 0, 0, 0, 0, 0,
            1.57, 0.0, 0.0,
            0, 0.0, 0.0,
            ],
            
            #0.3
            #braco esquerdo para frente, direito para tras, alternancia de perna
            [0.0 , 0.0 , -1.6461875707843674 , 1.0116060284533992 , 0.6345815423309678 , -0.0,
             0.0 , -0.7188299996216256 , 0.24246731907521746 , 1.3423648760613114 , -1.5848321951365287 , 0.7188299996216256,
             -1.57, 0.0, 0.0,
             1.57, 0.0, 0.0,
            ],
            
            #0.4
            #braco direito para tras, perna esquerda para o zero
            [0, 0, 0, 0, 0, 0, 
             0.0 , -0.7188299996216256 , 0.24246731907521746 , 1.3423648760613114 , -1.5848321951365287 , 0.7188299996216256,
             0, 0.0, 0.0,
             1.57, 0, 0
            ],
            
            #0.5
            #corrige posicao do torso
            [0, 0, 0, 0, 0, 0, 
             0, 0, 0, 0, 0, 0, 
             0, 0, 0, 
             0, 0, 0], 
            
            #0
            #posicao inicial
            [0, 0, 0, 0, 0, 0, #perna esquerda
             0, 0, 0, 0, 0, 0, #perna direita
             0, 0, 0, #braço esquerdo
             0, 0, 0], #braço direito
            
            #1
            #braco aberto 90 graus
            [0.0 , 0.0 , -0.0 , 0.0 , -0.0 , -0.0,
             0.0 , 0.0 , -0.0 , 0.0 , -0.0 , -0.0,
             0.0, -1.57, -0.0,
             0.0, 1.57, -0.0],
            
            #2
            #braco rotacionado no roll em 360
            [0.0 , 0.0 , -0.0 , 0.0 , -0.0 , -0.0,
             0.0 , 0.0 , -0.0 , 0.0 , -0.0 , -0.0,
             -3.05, -1.57, -0.0,
             -3.05, 1.57, -0.0
            ],

            #3
            #braco levantado para cima
            [0.0 , 0.0 , -0.0 , 0.0 , -0.0 , -0.0,
             0.0 , 0.0 , -0.0 , 0.0 , -0.0 , -0.0,
             -3.05, -0.0, -0.0,
             -3.05, -0.0, -0.0
            ],
            
            #4
            #recolhe perna e entra em posicao de prancha
            [0.0 , 0.0 , -1.347510130324382 , 2.1587986945091084 , -1.511288564184727 , -0.0,
             0.0 , 0.0 , -1.347510130324382 , 2.1587986945091084 , -1.511288564184727 , -0.0,
             -0.6723776154747352, -0.0, -2.4289171419559303,
             -0.6723776154747352, -0.0, -2.4289171419559303,
            ],
            
            #5
            #estende braco e prepara perna para posicao de levante
            [0.0 , 0.0 , -1.1543614716039874 , 2.1600611688726437 , -1.2056996972686567 , -0.0,
             0.0 , 0.0 , -1.1543614716039874 , 2.1600611688726437 , -1.2056996972686567 , -0.0,
             -1.0384443839794026, -0.0, -0.0757396801147956,
             -1.0384443839794026, -0.0, -0.0757396801147956,
            ],

            #6
            #recolhe braco para posicao de levante
            [0.0 , 0.0 , -1.1003592173949751 , 2.0007184347899503 , -1.0003592173949751 , -0.0,
             0.0 , 0.0 , -1.1003592173949751 , 2.0007184347899503 , -1.0003592173949751 , -0.0,
             0.037053355859876014, -0.0, -0.6190954824624666,
             0.037053355859876014, -0.0, -0.6190954824624666
            ],
            
            #7
            #recolhe mais o braco
            [0.0 , 0.0 , -1.186399552299258 , 2.372799104598516 , -1.186399552299258 , -0.0,
             0.0 , 0.0 , -1.186399552299258 , 2.372799104598516 , -1.186399552299258 , -0.0,
             1.9, 0.0, -1.3,
             1.9, 0.0, -1.3,
            ],
            
            #8
            #senta no proprio pe (libera torque no servo)
            [0.0 , 0.0 , -1.186399552299258 , 2.372799104598516 , -1.186399552299258 , -0.0,
             0.0 , 0.0 , -1.186399552299258 , 2.372799104598516 , -1.186399552299258 , -0.0,
             0.6080424270510827, -0.0, 0.1,
             0.6080424270510827, -0.0, 0.1,
            ],
            
            #9
            #prepara braco para posicao de levante
            [0.0 , 0.0 , -1.186399552299258 , 2.372799104598516 , -1.186399552299258 , -0.0,
             0.0 , 0.0 , -1.186399552299258 , 2.372799104598516 , -1.186399552299258 , -0.0,
             0.6080424270510827, -0.0, 0.0400333406175881,
             0.6080424270510827, -0.0, 0.0400333406175881,
            ],
            
            #10
            #apoia a posicao de levante com pe e braco
            [0.0 , 0.0 , -0.7595238660168626 , 2.041484801262796 , -1.2819609352459336 , -0.0,
             0.0 , 0.0 , -0.7595238660168626 , 2.041484801262796 , -1.2819609352459336 , -0.0,
             0, 0, 0, 
             0, 0, 0],
            
            #11
            #reajusta posicao para levantar completamente
            [0.0 , 0.0 , -0.7137951169038818 , 1.8685777565411072 , -1.1547826396372258 , -0.0,
             0.0 , 0.0 , -0.7137951169038818 , 1.8685777565411072 , -1.1547826396372258 , -0.0,
             0, 0, 0, 
             0, 0, 0],
        
            
        ]
        # delays = [0.5, 0.5, 0.5, 0.5, 2.0, 2.0, 5.0, 0.5, 1.0, 2.0, 1.0, 3.0]
        delays = [0.5, 2.0, 1.0, 0.5, 0.5, 1.0, 0.5, 0.5, 0.5, 0.5, 2.0, 2.0, 5.0, 0.5, 1.0, 2.0, 1.0, 3.0]
        publish_coordinates(coordinates_matrix, delays)
    except rospy.ROSInterruptException:
        pass
    
    
    
    #0.0 , 0.0 , -0.08642436199007525 , 0.5048438712708507 , -0.41841950928077526 , -0.0
