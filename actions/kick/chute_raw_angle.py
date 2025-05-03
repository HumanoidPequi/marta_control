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
            
            
            [0.0 , 0.0 , -0.829831624591877 , 1.659663249183754 , -0.829831624591877 , -0.0,
             0.0 , 0.0 , -0.829831624591877 , 1.659663249183754 , -0.829831624591877 , -0.0,
             1.0 , -0.3 , 0.0,
             1.0 , 0.3 , 0.0],
            
            [0.0 , 0.0 , -0.7189820319622575 , 1.791944522251201 , -1.0729624902889434 , -0.0,
             0.0 , 0.0 , -0.7189820319622575 , 1.791944522251201 , -1.0729624902889434 , -0.0,
             1.0 , -0.3 , 0.0,
             1.0 , 0.3 , 0.0],
            
        
            [0.0 , 0.0 , -0.5189820319622575 , 1.791944522251201 , -0.8729624902889434 , -0.0,
             0.0 , 0.0 , -0.5189820319622575 , 1.791944522251201 , -0.8729624902889434 , -0.0,
             1.0 , -0.3 , 0.0,
             1.0 , 0.3 , 0.0],
            
            
            [0.0 , 0.0 , -1.557096128713183 , 0.6127839660850278 , 0.9443121626281561 , -0.0,
             0.0 , 0.0 , -0.5189820319622575 , 1.791944522251201 , -0.8729624902889434 , -0.0,
             1.0 , -0.3 , 0.0,
             1.0 , 0.3 , 0.0],
            
            
            [0.0 , 0.0 , -1.545907418328543 , 1.9353689081800114 , -0.11946148985146845 , -0.0,
             0.0 , 0.0 , -0.5189820319622575 , 1.791944522251201 , -0.8729624902889434 , -0.0,
             1.0 , -0.3 , 0.0,
             1.0 , 0.3 , 0.0],
            
            [0.0 , 0.0 , -0.5189820319622575 , 1.791944522251201 , -0.4729624902889434 , -0.0,
             0.0 , 0.0 , -0.5189820319622575 , 1.791944522251201 , -0.8729624902889434 , -0.0,
             1.0 , -0.3 , 0.0,
             1.0 , 0.3 , 0.0],
            
        ]
        
        delays = [2.0, 0.8, 0.5, 2.0, 0.1, 1.0, 2.0]
        publish_coordinates(coordinates_matrix, delays)
    except rospy.ROSInterruptException:
        pass
    
    
    
    #0.0 , 0.0 , -0.08642436199007525 , 0.5048438712708507 , -0.41841950928077526 , -0.0
