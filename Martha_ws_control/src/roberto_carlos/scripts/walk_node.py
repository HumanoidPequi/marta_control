#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from walk_calculations import WalkZmpCalculator
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState  # Importando o tipo de mensagem JointState

class TimeIterationNodeWithXYZ:
    def __init__(self):
        # Inicializa o nó ROS
        rospy.init_node('walk_node')

        # Cria o publisher para publicar resultados de cálculos, se necessário
        # self.publisher = rospy.Publisher('/calculated_value', Float64, queue_size=10)

         # Cria o publisher para enviar mensagens JointState para o tópico /coordinates_input
        self.publisher = rospy.Publisher('/coordinates_input', JointState, queue_size=10)

        # Variável para armazenar o tempo inicial
        self.start_time = rospy.Time.now()

        # Variáveis de controle para valores recebidos
        self.v_x = 0.0 # x
        self.v_y = 0.0 # y
        self.v_z = 0.0 # angular

        # Define a taxa de execução (10 Hz)
        self.rate = rospy.Rate(20)  # 10 Hz


        self.calculator = WalkZmpCalculator()

        # Parâmetros físicos do robô carregados do servidor de parâmetros ROS
        self.physical_parameters = self.load_physical_parameters()
        self.walking_params = self.calculator.get_walking_params()

        

        # self.v_zero_pe = [0.02, 0.0475, -0.2135, 0.0, 0.02, -0.0475, -0.2335, 0.0]
        
        # zero_pe define a posicao inicial do pe da marta em relacao ao CG. nesse caso ela so coloca as coordenadas de x e y como a soma dos vetores de translacao do torso_hip e do pe
        # ja para o z do pe deve ser levado em conta o eixo
        self.x_zero = self.physical_parameters['torso_to_left_hip'][0]
        self.y_zero = self.physical_parameters['torso_to_left_hip'][1] + self.physical_parameters['left_ankle_to_left_foot'][1]

        if ( abs(self.physical_parameters['torso_to_left_hip'][2]) + abs(self.physical_parameters['left_ankle_to_left_foot'][2]) + abs(self.physical_parameters['upper_leg_length']) + abs(self.physical_parameters['lower_leg_length']) < self.physical_parameters['zCom'] ):
            rospy.logfatal("invalid zCoM")
            self.z_zero = -(abs(self.physical_parameters['torso_to_left_hip'][2]) + abs(self.physical_parameters['left_ankle_to_left_foot'][2]) + abs(self.physical_parameters['upper_leg_length']) + abs(self.physical_parameters['lower_leg_length']) - 0.02)
        else:
            self.z_zero = -(self.physical_parameters['zCom'])
        

        self.v_zero_pe = [  self.x_zero,
                            self.y_zero,
                            self.z_zero,
                            0,
                            self.x_zero,
                            -self.y_zero,
                            self.z_zero,
                            0]            

        self.left_is_swing_foot = True

        


        self.torso_begin_x = 0


        
    def load_physical_parameters(self):
        # Carregar os parâmetros do servidor de parâmetros ROS
            parameters = {
            'upper_leg_length': rospy.get_param('/walking_params/upper_leg_length', 0.1),
            'lower_leg_length': rospy.get_param('/walking_params/lower_leg_length', 0.1),
            'hip_pitch0': rospy.get_param('/walking_params/hip_pitch0', 0.0),
            'knee_pitch0': rospy.get_param('/walking_params/knee_pitch0', 0.0),
            'ankle_pitch0': rospy.get_param('/walking_params/ankle_pitch0', 0.0),
            'torso_to_left_hip': rospy.get_param('/walking_params/torso_to_left_hip', [0.02, 0.037, -0.02]),
            'left_ankle_to_left_foot': rospy.get_param('/walking_params/left_ankle_to_left_foot', [0.0, 0.0105, -0.0335]),
            'period': rospy.get_param('/walking_params/period', 0.5),
            'doubleSupportRatio': rospy.get_param('/walking_params/doubleSupportRatio', 0.2),
            'zCom': rospy.get_param('/walking_params/zCom', 0.25),
            'zStep': rospy.get_param('/walking_params/zStep', 0.05),
            'armCompensationFactor': rospy.get_param('/walking_params/armCompensationFactor', 1.0),
            'comSwingFactor': rospy.get_param('/walking_params/comSwingFactor', 0.5),
            }
            rospy.loginfo(f"Loaded physical parameters: {parameters}")
            return parameters

    def xyz_callback(self, msg):
        """
        Callback que é chamado quando uma mensagem do tipo Point é recebida.
        Armazena os valores de x, y e z.
        """
        self.v_x = msg.x   # Valor no eixo x
        self.v_y = msg.y   # Valor no eixo y
        self.v_z = msg.z   # Valor no eixo z

        rospy.loginfo("Recebido: x=%.2f, y=%.2f, z=%.2f", self.v_x, self.v_y, self.v_z)

    def run(self):
        while not rospy.is_shutdown():
            # Obtém o tempo decorrido desde o início do nó
            current_time = rospy.Time.now()
            elapsed_time = (current_time - self.start_time).to_sec()

            # reseta o elapsed_time para fazer outro passo
            if (elapsed_time >= self.physical_parameters['period']):
                self.start_time = rospy.Time.now()
                # troca perna
                self.left_is_swing_foot = not self.left_is_swing_foot
                # print("troca perna")

            # pega o ponto desejado dado as velocidades e ajusta para o sistema rotacionado em v_z*periodo
            dx, dy, drot = self.rotation_coordinates(self.v_x, self.v_y, self.v_z, self.physical_parameters['period'])

            # phi = tempo/periodo --> adimensional de tempo
            phi = self.calculator.calculate_phi(elapsed_time) 
            
            if (phi > 1.0):
                phi = 1.0


            # Função que utiliza os valores recebidos para realizar algum cálculo
            coordinates_foot_z = self.rise_feet(self.left_is_swing_foot, phi)
            coordinates_foot_y = self.swing_feet(self.left_is_swing_foot, phi)
            #coordinates_foot_x = self.stride_feet(self.left_is_swing_foot, phi)

            coordinates_foot_x = [0,0,0,0, 0,0,0,0]

            foot_coordinates = self.matrix_add_3(coordinates_foot_z, coordinates_foot_y, coordinates_foot_x)
            # publica mensagem
            msg = JointState()
            msg.position = self.matrix_add(foot_coordinates, self.v_zero_pe)
            # print position
            print(self.v_zero_pe)
            # rospy.loginfo("[ %.4f, %.4f, %.4f, %.4f,  %.4f, %.4f, %.4f, %.4f]", foot_coordinates[0], foot_coordinates[1], foot_coordinates[2], foot_coordinates[3], foot_coordinates[4], foot_coordinates[5], foot_coordinates[6], foot_coordinates[7])
            self.publisher.publish(msg)
            # Espera até a próxima iteração de acordo com a taxa definida (10 Hz)
            self.rate.sleep()

    def rise_feet(self, left_is_swing_foot, phi):
        """
        Função que executa uma ação com base nos valores recebidos.
        Aqui, você pode implementar a lógica desejada usando x, y e z.
        """
        point_swing_foot = [0 , 0 , self.walking_params['zStep']*self.calculator.calculate_zFrac(phi), 0]


        if(left_is_swing_foot):
            coordinates_foot = point_swing_foot + [0, 0, 0, 0] #TODO: coordenada pe de suporte
        else:
            coordinates_foot = [0, 0, 0, 0] + point_swing_foot

        # Publica o resultado no tópico /calculated_value
        return coordinates_foot

    def swing_feet(self, left_is_swing_foot, phi):
        # parametros de posicao do zmp
        p_0 = 0
        p_s = self.y_zero
        p_f = 0

        # md1, md2 = self.calculator.calculate_m1m2(p_s, p_0, p_f)
        # if (phi < t1):
        #     p_t = p_0 + md1*phi*period
        #     # print("primeiro if")
        # elif (phi < t2):
        #     p_t = p_s
        # else:
        #     p_t = p_s + md2*(phi - t2)*period

        # print(p_t, phi)


        aPy, aNy = self.calculator.calculate_params_ZMP(p_s, p_0, p_f, 0, 0)
        y_support_foot = self.calculator.get_CoM(phi, aPy, aNy, p_s, p_0, p_f)*self.calculator.calculate_yFrac(phi)
        # y_support_foot = p_t

        # if (phi > 1.0):
        #     y_support_foot = 0
        # print(y_support_foot)

        # se o pe esquerdo levanta, coloca o zmp no direito. vice versa
        if (left_is_swing_foot):
            coordinates_foot = [0, -y_support_foot, 0, 0] + [ 0, -y_support_foot, 0, 0]
        else:
            coordinates_foot = [ 0, y_support_foot, 0, 0] + [0, y_support_foot, 0, 0]

        return coordinates_foot

    def stride_feet(self, left_is_swing_foot, phi):
        self.v_x = 0.001
        period = self.physical_parameters['period']

        p_x_0 = 0
        p_x_s = self.v_x*period
        p_x_f = self.v_x*2*period

        aPy, aNy = self.calculator.calculate_params_ZMP(p_x_s, p_x_0, p_x_f, 0, p_x_f)
        x_support_foot = self.calculator.get_CoM(phi, aPy, aNy, p_x_s, p_x_0, p_x_f)*self.calculator.calculate_xFrac(phi)

        print(x_support_foot)
        # se o pe esquerdo levanta, avanca o esquerdo e recua direito. vice versa
        if (left_is_swing_foot):
            coordinates_foot = [x_support_foot, 0, 0, 0] + [ -x_support_foot, 0, 0, 0]
        else:
            coordinates_foot = [-x_support_foot, 0, 0, 0] + [x_support_foot, 0, 0, 0]

        # coordinates_foot = [0,0,0,0, 0,0,0,0]
        return coordinates_foot



    def matrix_add(self, mat1, mat2):
        mat_buffer = []
        for i in range(len(mat1)):
            mat_buffer.append(mat1[i] + mat2[i])
        return mat_buffer
    
    def matrix_add_3(self, mat1, mat2, mat3):
        mat_buffer = []
        for i in range(len(mat1)):
            mat_buffer.append(mat1[i] + mat2[i] + mat3[i])
        return mat_buffer
    
    def rotation_coordinates(self, v_x, v_y, v_z, period):
        x =   v_x*period * math.cos(v_z*period) + v_y*period * math.sin(v_z*period)
        y = - v_x*period * math.sin(v_z*period) + v_y*period * math.cos(v_z*period)
        rot = v_z*period
        return x, y, rot


if __name__ == '__main__':
    try:
        # Cria o nó e começa a iterar
        node = TimeIterationNodeWithXYZ()
        node.run()
    except rospy.ROSInterruptException:
        pass
