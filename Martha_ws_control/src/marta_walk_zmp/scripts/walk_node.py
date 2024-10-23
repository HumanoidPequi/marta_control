#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from walk_calculations import WalkZmpCalculator
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState  # Importa o tipo de mensagem JointState
from geometry_msgs.msg import Twist

class TimeIterationNodeWithTwist:
    def __init__(self):

        self.calculator = WalkZmpCalculator()

        # Inicializa o nó ROS
        rospy.init_node('time_iteration_with_twist_node', anonymous=True)

        # Cria o publisher para publicar resultados de cálculos, se necessário
        self.publisher = rospy.Publisher('/calculated_value', Float64, queue_size=10)

        # Cria o publisher para o tópico /coordinates_input (JointState)
        self.coordinates_publisher = rospy.Publisher('/coordinates_input', JointState, queue_size=10)

        # Cria o subscriber para receber as mensagens do tipo Twist
        rospy.Subscriber('/cmd_vel', Twist, self.twist_callback)

        # Variável para armazenar o tempo inicial
        self.start_time = rospy.Time.now()

        # Variáveis de controle de velocidades recebidas
        self.v_x = 0.0
        self.v_y = 0.0
        self.v_angular = 0.0

        # Variáveis de z_inicial e z_final
        self.z_inicial = 0.2535  # ou qualquer valor inicial adequado
        self.z_final = 0.2035    # ou qualquer valor final adequado

        # Define a taxa de execução (10 Hz)
        self.rate = rospy.Rate(10)  # 10 Hz


    def twist_callback(self, msg):
        """
        Callback que é chamado quando uma mensagem do tipo Twist é recebida.
        Armazena os valores de v_x, v_y, e v_angular.
        """
        self.v_x = msg.linear.x   # Velocidade no eixo x
        self.v_y = msg.linear.y   # Velocidade no eixo y
        self.v_angular = msg.angular.z  # Velocidade angular (yaw)

        rospy.loginfo("Recebido: v_x=%.2f, v_y=%.2f, v_angular=%.2f", self.v_x, self.v_y, self.v_angular)

    def run(self):
        while not rospy.is_shutdown():
            # Obtém o tempo decorrido desde o início do nó
            current_time = rospy.Time.now()
            elapsed_time = (current_time - self.start_time).to_sec()

            # Função que utiliza os valores recebidos para realizar algum cálculo
            self.perform_action(elapsed_time)

            # Espera até a próxima iteração de acordo com a taxa definida (10 Hz)
            self.rate.sleep()

    def perform_action(self, elapsed_time):
        """
        Função que executa uma ação com base nos valores de velocidade recebidos.
        Aqui, você pode implementar a lógica desejada usando v_x, v_y e v_angular.
        """
        walkingTime = elapsed_time
        if (walkingTime >= self.calculator.params['period']):
            walkingTime = 0
            self.start_time = rospy.Time.now()

        self.compute_one_step(walkingTime)
        

    def compute_one_step(self, elapsed_time):
        # Obtém o valor de zFrac a partir do WalkZmpCalculator

        zFrac = self.calculator.calculate_zFrac(elapsed_time)

        # Calcula o novo valor de z
        z = self.z_inicial + (self.z_final - self.z_inicial) * zFrac


        # Log para ver o progresso de z
        # rospy.loginfo("Tempo: %.2f, zFrac: %.2f, z_inicial: %.2f, z_final: %.2f, z: %.2f", elapsed_time, zFrac, self.z_inicial, self.z_final, z)

        # Publica o resultado no tópico /calculated_value
        msg = Float64()
        msg.data = z
        self.publisher.publish(msg)

        # Prepara a mensagem JointState para o tópico /coordinates_input
        joint_state_msg = JointState()

        # Inicializa as posições com 0 ou com valores padrão (modifique conforme necessário)
        joint_state_msg.name = ['x_left', 'y_left', 'z_left', 'theta_left', 'x_right', 'y_right', 'z_right', 'theta_right']
        joint_state_msg.position = [0.02, 0.0475, z, 0.0, 0.02, 0.0475, -0.2535, 0.0]  # Apenas z_left é atualizado com o valor de z

        # Publica a mensagem JointState no tópico /coordinates_input
        self.coordinates_publisher.publish(joint_state_msg)

        # Log do valor calculado para JointState
        rospy.loginfo("Publicando JointState: z_left=%.2f", z)


if __name__ == '__main__':
    try:
        # Cria o nó e começa a iterar
        node = TimeIterationNodeWithTwist()
        node.run()
    except rospy.ROSInterruptException:
        pass
