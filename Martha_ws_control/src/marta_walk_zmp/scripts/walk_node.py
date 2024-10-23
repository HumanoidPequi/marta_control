#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from walk_calculations import WalkZmpCalculator
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class TimeIterationNodeWithTwist:
    def __init__(self):

        self.calculator = WalkZmpCalculator()

        # Inicializa o nó ROS
        rospy.init_node('time_iteration_with_twist_node', anonymous=True)

        # Cria o publisher para publicar resultados de cálculos, se necessário
        self.publisher = rospy.Publisher('/calculated_value', Float64, queue_size=10)

        # Cria o subscriber para receber as mensagens do tipo Twist
        rospy.Subscriber('/cmd_vel', Twist, self.twist_callback)

        # Variável para armazenar o tempo inicial
        self.start_time = rospy.Time.now()

        # Variáveis de controle de velocidades recebidas
        self.v_x = 0.0
        self.v_y = 0.0
        self.v_angular = 0.0

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

        params = self.calculator.get_walking_params()
        rospy.loginfo(params['period'])

        # Exemplo: Executar algum cálculo com as velocidades e o tempo decorrido
        # Neste caso, apenas somando as velocidades e o tempo para um exemplo simples
        result = self.v_x * elapsed_time + self.v_y * elapsed_time + self.v_angular * elapsed_time

        # Publica o resultado no tópico /calculated_value
        msg = Float64()
        msg.data = result
        self.publisher.publish(msg)

        # Log do valor calculado
        # rospy.loginfo("Tempo: %.2f, Resultado calculado: %.2f", elapsed_time, result)

if __name__ == '__main__':
    try:
        # Cria o nó e começa a iterar
        node = TimeIterationNodeWithTwist()
        node.run()
    except rospy.ROSInterruptException:
        pass
