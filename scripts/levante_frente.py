#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray, Float64
from marta_msgs.srv import Levantar, LevantarResponse
import time

class LevantarParaFrenteServer:
    def __init__(self):
        # Inicializa os publishers
        self.init_publishers()
        
        # Define o serviço
        self.service = rospy.Service('/levantar_para_frente', Levantar, self.handle_levantar_para_frente)
        self.isReal_ = rospy.get_param("~isReal", True)
        
        rospy.loginfo("Serviço 'levantar_para_frente' pronto.")
    
    def init_publishers(self):
        # Publishers para robô real (MultiArray)
        self.pub_left_leg = rospy.Publisher('/marta/left_leg/command', Float64MultiArray, queue_size=10)
        self.pub_right_leg = rospy.Publisher('/marta/right_leg/command', Float64MultiArray, queue_size=10)
        self.pub_arm_l_head = rospy.Publisher('/marta/arm_l_head/command', Float64MultiArray, queue_size=10)
        self.pub_arm_r = rospy.Publisher('/marta/arm_r/command', Float64MultiArray, queue_size=10)
        
        # Publishers para simulação (tópicos individuais)
        self.pub_cabeca_pitch = rospy.Publisher('/marta/head_tilt_position/command', Float64, queue_size=10)
        self.pub_pescoco_yaw = rospy.Publisher('/marta/head_pan_position/command', Float64, queue_size=10)
        
        self.pub_pe_esq_pitch = rospy.Publisher('/marta/l_ank_pitch_position/command', Float64, queue_size=10)
        self.pub_pe_esq_roll = rospy.Publisher('/marta/l_ank_roll_position/command', Float64, queue_size=10)
        self.pub_quadril_esq_pitch = rospy.Publisher('/marta/l_hip_pitch_position/command', Float64, queue_size=10)
        self.pub_quadril_esq_roll = rospy.Publisher('/marta/l_hip_roll_position/command', Float64, queue_size=10)
        self.pub_quadril_esq_yaw = rospy.Publisher('/marta/l_hip_yaw_position/command', Float64, queue_size=10)
        self.pub_joelho_esq_pitch = rospy.Publisher('/marta/l_knee_position/command', Float64, queue_size=10)
        
        self.pub_pe_dir_pitch = rospy.Publisher('/marta/r_ank_pitch_position/command', Float64, queue_size=10)
        self.pub_pe_dir_roll = rospy.Publisher('/marta/r_ank_roll_position/command', Float64, queue_size=10)
        self.pub_quadril_dir_pitch = rospy.Publisher('/marta/r_hip_pitch_position/command', Float64, queue_size=10)
        self.pub_quadril_dir_roll = rospy.Publisher('/marta/r_hip_roll_position/command', Float64, queue_size=10)
        self.pub_quadril_dir_yaw = rospy.Publisher('/marta/r_hip_yaw_position/command', Float64, queue_size=10)
        self.pub_joelho_dir_pitch = rospy.Publisher('/marta/r_knee_position/command', Float64, queue_size=10)
        
        self.pub_cotovelo_esq_pitch = rospy.Publisher('/marta/l_el_position/command', Float64, queue_size=10)
        self.pub_ombro_esq_pitch = rospy.Publisher('/marta/l_sho_pitch_position/command', Float64, queue_size=10)
        self.pub_ombro_esq_roll = rospy.Publisher('/marta/l_sho_roll_position/command', Float64, queue_size=10)
        
        self.pub_cotovelo_dir_pitch = rospy.Publisher('/marta/r_el_position/command', Float64, queue_size=10)
        self.pub_ombro_dir_pitch = rospy.Publisher('/marta/r_sho_pitch_position/command', Float64, queue_size=10)
        self.pub_ombro_dir_roll = rospy.Publisher('/marta/r_sho_roll_position/command', Float64, queue_size=10)
    
    def handle_levantar_para_frente(self, req):
        try:
            self.publish_coordinates()
            return LevantarResponse(True, "Movimento 'levantar para frente' executado com sucesso")
        except Exception as e:
            return LevantarResponse(False, f"Erro ao executar movimento: {str(e)}")
    
    def publish_coordinates(self):
        rospy.loginfo("Publicando coordenadas para levantar para frente...")
        coordinates_matrix = [
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
            #recolhe perna e entra em posicao de espacate
            [0.04448544234037399, 1.4940972328186035, -0.09664078801870346, 0.18254370987415314, -0.0782330185174942, -1.5784661769866943,
              -0.07363107800483704, -1.685844898223877, -0.09817477315664291, 0.10431069135665894, -0.03528155758976936, 1.5155730247497559,
             -0.6723776154747352, -0.0, -2.4289171419559303,
             -0.6723776154747352, -0.0, -2.4289171419559303,
            ],
            
            
            #6
            # posicao de levante 1
            [0.003067961661145091, 0.9387962222099304, -1.052310824394226, 2.3408546447753906, -1.257864236831665, -0.9249904155731201, 
             -0.0015339808305725455, -1.0569127798080444, -1.1535536050796509, 2.356194496154785, -1.1765632629394531, 0.9909515976905823, 
             -0.3574175238609314, 0.026077672839164734, -0.0951068103313446,
             -0.3574175238609314, 0.026077672839164734, -0.0951068103313446
            ],
            
            [0.26998060941696167, 0.7424467206001282, -1.2839418649673462, 1.780951738357544, -0.8805049657821655, -0.7424467206001282, 
             -0.07209710031747818, -0.685689389705658, -1.2363885641098022, 1.90827214717865, -1.035436987876892, 0.7102330923080444,
             -0.3574175238609314, 0.026077672839164734, -0.0951068103313446,
             -0.3574175238609314, 0.026077672839164734, -0.0951068103313446
            ],

            #posicao inicial
            [-0.30,  0.06, -0.68,  1.56, -0.96, -0.06,   # perna esquerda
            -0.30, -0.25, -0.46,  1.41, -1.04,  0.25,   # perna direita
            0.00, -0.87,  0.00,                       # braço esquerdo
            -0.05,  0.87,  0.00]                        # braço direito
            
        ]
        delays = [0.5, 0.5, 0.5, 2.0, 2.0, 3.0, 3.0, 0.01]
        
        rate = 100
        steps = [int(rate * delay) for delay in delays]

        position_buffered = coordinates_matrix[0]
        for i, position in enumerate(coordinates_matrix):
            for j in range(steps[i]):
                position_interpolated = [
                    position_buffered[k] + (position[k] - position_buffered[k]) * (j / steps[i]) 
                    for k in range(len(position))
                ]

                if self.isReal_:
                    # Publica para robô real (MultiArray)
                    msg_left_leg = Float64MultiArray(data=position_interpolated[0:6])
                    msg_right_leg = Float64MultiArray(data=position_interpolated[6:12])
                    msg_arm_l_head = Float64MultiArray(data=[0, 0] + position_interpolated[12:15])
                    msg_arm_r = Float64MultiArray(data=position_interpolated[15:18])
                    
                    self.pub_left_leg.publish(msg_left_leg)
                    self.pub_right_leg.publish(msg_right_leg)
                    self.pub_arm_l_head.publish(msg_arm_l_head)
                    self.pub_arm_r.publish(msg_arm_r)
                else:
                    # Publica para simulação (tópicos individuais)
                    msg = Float64()
                    
                    # Cabeça (não está nas coordenadas, mantém 0)
                    msg.data = 0
                    self.pub_cabeca_pitch.publish(msg)
                    self.pub_pescoco_yaw.publish(msg)
                    
                    # Pernas esquerda
                    msg.data = position_interpolated[0]   # quadril yaw
                    self.pub_quadril_esq_yaw.publish(msg)
                    msg.data = position_interpolated[1]   # quadril roll
                    self.pub_quadril_esq_roll.publish(msg)
                    msg.data = position_interpolated[2]   # quadril pitch
                    self.pub_quadril_esq_pitch.publish(msg)
                    msg.data = position_interpolated[3]   # joelho pitch
                    self.pub_joelho_esq_pitch.publish(msg)
                    msg.data = position_interpolated[4]   # tornozelo pitch
                    self.pub_pe_esq_pitch.publish(msg)
                    msg.data = position_interpolated[5]   # tornozelo roll
                    self.pub_pe_esq_roll.publish(msg)
                    
                    # Pernas direita
                    msg.data = position_interpolated[6]   # quadril yaw
                    self.pub_quadril_dir_yaw.publish(msg)
                    msg.data = position_interpolated[7]   # quadril roll
                    self.pub_quadril_dir_roll.publish(msg)
                    msg.data = position_interpolated[8]   # quadril pitch
                    self.pub_quadril_dir_pitch.publish(msg)
                    msg.data = position_interpolated[9]   # joelho pitch
                    self.pub_joelho_dir_pitch.publish(msg)
                    msg.data = position_interpolated[10]  # tornozelo pitch
                    self.pub_pe_dir_pitch.publish(msg)
                    msg.data = position_interpolated[11]  # tornozelo roll
                    self.pub_pe_dir_roll.publish(msg)
                    
                    # Braços esquerdo (arm_l_head)
                    msg.data = position_interpolated[12]  # ombro pitch
                    self.pub_ombro_esq_pitch.publish(msg)
                    msg.data = position_interpolated[13]  # ombro roll
                    self.pub_ombro_esq_roll.publish(msg)
                    msg.data = position_interpolated[14]  # cotovelo pitch
                    self.pub_cotovelo_esq_pitch.publish(msg)
                    
                    # Braços direito (arm_r)
                    msg.data = position_interpolated[15]  # ombro pitch
                    self.pub_ombro_dir_pitch.publish(msg)
                    msg.data = position_interpolated[16]  # ombro roll
                    self.pub_ombro_dir_roll.publish(msg)
                    msg.data = position_interpolated[17]  # cotovelo pitch
                    self.pub_cotovelo_dir_pitch.publish(msg)
                
                rospy.sleep(1 / rate)
            position_buffered = position

if __name__ == '__main__':
    rospy.init_node('levantar_para_frente_server')
    server = LevantarParaFrenteServer()
    rospy.spin()