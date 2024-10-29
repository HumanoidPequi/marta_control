#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
import math

class WalkZmpCalculator:
    def __init__(self):
        # Inicializa o ROS para acessar os parâmetros e funcionalidades do sistema
        # rospy.init_node('walk_zmp_calculator', anonymous=True)

        # Obtém os parâmetros ao inicializar a classe
        self.params = self.get_walking_params()

    # Função para obter parâmetros do ROS Parameter Server
    def get_walking_params(self):
        try:
            # Obtém todos os parâmetros necessários do namespace /walking_params
            period = rospy.get_param('/walking_params/period')
            zStep = rospy.get_param('/walking_params/zStep')
            doubleSupportRatio = rospy.get_param('/walking_params/doubleSupportRatio')
            zCom = rospy.get_param('/walking_params/zCom')
            comSwingFactor = rospy.get_param('/walking_params/comSwingFactor')
            ySeparation = rospy.get_param('/walking_params/ySeparation', 0.0475)

            GRAVITY = 9.80665
            tZMP = math.sqrt(zCom*comSwingFactor/GRAVITY)**(-1)


            # Retorna os parâmetros como um dicionário
            return {
                'period': period,
                'zStep': zStep,
                'doubleSupportRatio': doubleSupportRatio,
                'zCom' : zCom,
                'comSwingFactor': comSwingFactor,
                'tZMP' : tZMP,
                'ySeparation' : ySeparation 
            }

        except KeyError as e:
            # Se algum parâmetro não for encontrado, exibe uma mensagem de erro
            print(f"Erro: Parâmetro {e} não encontrado no ROS Parameter Server.")
            exit(1)

    # Função para calcular o valor de phi (t/T)
    def calculate_phi(self, elapsed_time):
        period = self.params['period']
        phi = elapsed_time / period
        return phi
    
    def calculate_m1m2(self, y_zmp_s, y_zmp_0, y_zmp_f):
        period = self.params['period']
        doubleSupportRatio = self.params['doubleSupportRatio']
    
        T1 = period*doubleSupportRatio/2                            # isto é na verdade T_b
        T2 = period - T1                                            # isto é na verdade T_e

        m1 = (y_zmp_s - y_zmp_0)/ T1                                          # isto é na verdade m_d1
        m2 = -(y_zmp_s - y_zmp_f)/(period - T2)                               # isto é na verdade m_d2
        return m1, m2

    # Função para calcular T1 e T2 baseados no período e no doubleSupportRatio
    def calculate_ZMP(self, y_zmp_s, y_zmp_0, y_zmp_f, x1, x2):
        period = self.params['period']
        doubleSupportRatio = self.params['doubleSupportRatio']
        tZMP = self.params['tZMP']

        T1 = period*doubleSupportRatio/2                            # isto é na verdade T_b
        T2 = period - T1                                            # isto é na verdade T_e

        m1 = (y_zmp_s - y_zmp_0)/ T1                                          # isto é na verdade m_d1
        m2 = -(y_zmp_s - y_zmp_f)/(period - T2)                               # isto é na verdade m_d2

        c1 = x1 - y_zmp_0 + tZMP * m1 * math.sinh(-T1/tZMP)              # isto é na verdade K_0
        c2 = x2 - y_zmp_f + tZMP * m2 * math.sinh((period - T2)/tZMP)    # isto é na verdade K_f

        expTStep = math.exp(period/tZMP)

        aP = (c2 - c1/expTStep) / (expTStep - 1/expTStep)           # isto é na verdade a_s
        aN = (c1 * expTStep - c2) / (expTStep - 1/expTStep)         # isto é na verdade b_s
        return aP, aN
    
    def calculate_xFrac(self, t):
        t1 = self.params['doubleSupportRatio']/2
        t2 = 1 - t1
        if ( t < t1):
            return 0.0
        if ( t < t2):
            return 0.5 * (1.0 - math.cos(math.pi * (t - t1) / (t2 - t1)))
        return 1.0
    def calculate_yFrac(self, t):
        t1 = self.params['doubleSupportRatio']/2
        t2 = 1 - t1
        if ( t < t1):
            return 0.0
        if ( t < t2):
            return 0.5 * (1.0 - math.cos(math.pi * (t - t1) / (t2 - t1)))
        return 1.0
    def calculate_zFrac(self, t):
        t1 = self.params['doubleSupportRatio']/2
        t2 = 1 - t1
    
        if ( t > t1 and t < t2):
            return 0.5 * (1.0 - math.cos( 2 * math.pi * (t - t1) / (t2 - t1)))
        return 0.0



# Exemplo de uso do WalkZmpCalculator
if __name__ == '__main__':
    # Instancia a classe
    calculator = WalkZmpCalculator()
    # Tempo inicial (o começo da caminhada)
    start_time = time.time()
    # Tempo atual (pode ser substituído por qualquer outro cálculo de tempo)
    current_time = time.time()

    # Cálculo de phi = t / T, onde T é o período da caminhada
    phi = calculator.calculate_phi(start_time)
    # Cálculo de T1 e T2

    # Exibe o valor de phi, T1, e T2
    print(f"Valor de phi: {phi:.2f}")

    # Continue fazendo outros cálculos ou utilize os parâmetros conforme necessário
