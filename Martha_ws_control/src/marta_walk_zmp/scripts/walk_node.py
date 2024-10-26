#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from walk_calculations import WalkZmpCalculator
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState  # Importando o tipo de mensagem JointState

class TimeIterationNodeWithXYZ:
    def __init__(self):

        

        # Inicializa o nó ROS
        rospy.init_node('time_iteration_with_xyz_node', anonymous=True)

        # Cria o publisher para publicar resultados de cálculos, se necessário
        self.publisher = rospy.Publisher('/calculated_value', Float64, queue_size=10)

         # Cria o publisher para enviar mensagens JointState para o tópico /coordinates_input
        self.publisher = rospy.Publisher('/coordinates_input', JointState, queue_size=10)

        # Variável para armazenar o tempo inicial
        self.start_time = rospy.Time.now()

        # Variáveis de controle para valores recebidos
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        # Define a taxa de execução (10 Hz)
        self.rate = rospy.Rate(20)  # 10 Hz


        self.calculator = WalkZmpCalculator()

        # Parâmetros físicos do robô carregados do servidor de parâmetros ROS
        self.physical_parameters = self.load_physical_parameters()
        self.walking_params = self.calculator.get_walking_params()



        

        # self.zero_pe = [0.02, 0.0475, -0.1535, 0.0, 0.02, -0.0475, -0.1535, 0.0]
        self.zero_pe = [self.physical_parameters['torso_to_left_hip'][0],
                        self.physical_parameters['torso_to_left_hip'][1] + self.physical_parameters['left_ankle_to_left_foot'][1],
                        -(self.physical_parameters['torso_to_left_hip'][2] + self.physical_parameters['left_ankle_to_left_foot'][2] + self.physical_parameters['upper_leg_length'] + self.physical_parameters['lower_leg_length']) ,
                        0,
                        self.physical_parameters['torso_to_left_hip'][0],
                        -(self.physical_parameters['torso_to_left_hip'][1] + self.physical_parameters['left_ankle_to_left_foot'][1]),
                        -(self.physical_parameters['torso_to_left_hip'][2] + self.physical_parameters['left_ankle_to_left_foot'][2] + self.physical_parameters['upper_leg_length'] + self.physical_parameters['lower_leg_length']) ,
                        0]

        self.left_is_swing_foot = True

        
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
            'comSwingFactor': rospy.get_param('/walking_params/comSwingFactor', 0.5)
            }
            rospy.loginfo(f"Loaded physical parameters: {parameters}")
            return parameters

    def xyz_callback(self, msg):
        """
        Callback que é chamado quando uma mensagem do tipo Point é recebida.
        Armazena os valores de x, y e z.
        """
        self.x = msg.x   # Valor no eixo x
        self.y = msg.y   # Valor no eixo y
        self.z = msg.z   # Valor no eixo z

        rospy.loginfo("Recebido: x=%.2f, y=%.2f, z=%.2f", self.x, self.y, self.z)

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
            
            # Função que utiliza os valores recebidos para realizar algum cálculo
            self.rise_feet(elapsed_time, self.left_is_swing_foot)

            # Espera até a próxima iteração de acordo com a taxa definida (10 Hz)
            self.rate.sleep()

    def rise_feet(self, elapsed_time, left_is_swing_foot):
        """
        Função que executa uma ação com base nos valores recebidos.
        Aqui, você pode implementar a lógica desejada usando x, y e z.
        """
        # phi = tempo/periodo --> adimensional de tempo
        phi = self.calculator.calculate_phi(elapsed_time) 
        point_swing_foot = [0 , 0 , self.walking_params['zStep']*self.calculator.calculate_zFrac(phi), 0]

        if(left_is_swing_foot):
            coordinates_swing_foot = point_swing_foot + [0, 0, 0, 0] #TODO: coordenada pe de suporte
        else:
            coordinates_swing_foot = [0, 0, 0, 0] + point_swing_foot

        # Publica o resultado no tópico /calculated_value
        msg = JointState()
        msg.position = self.matrix_add(self.zero_pe , coordinates_swing_foot)
        print(msg.position)
        self.publisher.publish(msg)



    def matrix_add(self, mat1, mat2):
        mat_buffer = []
        for i in range(len(mat1)):
            mat_buffer.append(mat1[i] + mat2[i])
        return mat_buffer

if __name__ == '__main__':
    try:
        # Cria o nó e começa a iterar
        node = TimeIterationNodeWithXYZ()
        node.run()
    except rospy.ROSInterruptException:
        pass
