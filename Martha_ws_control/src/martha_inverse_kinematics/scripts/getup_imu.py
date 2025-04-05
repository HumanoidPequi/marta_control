#!/usr/bin/env python3
import rospy
import actionlib
import tf
from sensor_msgs.msg import Imu
from my_robot_msgs.msg import RaiseAction, RaiseFeedback, RaiseResult
import subprocess

current_angle = 0.0

def imu_callback(data):
    global current_angle
    # Extrai o quaternion da mensagem do IMU e converte para ângulos de Euler
    q = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(q)
    current_angle = pitch
    rospy.logdebug("Pitch atualizado: {:.3f} rad".format(pitch))

def call_raise_front():
    rospy.loginfo("Chamando nó de levantamento pela frente...")

    #subprocess.Popen(["rosrun", "martha_inverse_kinematics", "levante_frente_raw_angle.py"])

def call_raise_back():
    rospy.loginfo("Chamando nó de levantamento por trás...")

    #subprocess.Popen(["rosrun", "martha_inverse_kinematics", "levante_tras_raw_angle.py"])

def execute_callback(goal):
    rospy.loginfo("Goal recebido. Verificando orientação do IMU...")
    # Aguarda brevemente para garantir atualização dos dados do IMU
    rospy.sleep(0.5)
    angle = current_angle
    THRESHOLD = 0.2  # Ajuste o threshold conforme necessário

    result = RaiseResult()
    if angle > THRESHOLD:
        rospy.loginfo("Ângulo positivo acima do threshold. Chamando levantamento pela frente.")
        call_raise_front()
        result.success = True
        result.message = "Levantamento pela frente acionado."
    elif angle < -THRESHOLD:
        rospy.loginfo("Ângulo negativo abaixo do threshold. Chamando levantamento por trás.")
        call_raise_back()
        result.success = True
        result.message = "Levantamento por trás acionado."
    else:
        rospy.logwarn("Ângulo dentro do threshold; nenhuma ação executada.")
        result.success = False
        result.message = "Ângulo dentro do threshold; ação não executada."
    
    server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node('raise_action_server')
    
    # Inscreve-se no tópico onde o Arduino publica os dados do IMU
    rospy.Subscriber('/imu', Imu, imu_callback)
    
    # Cria o Action Server (certifique-se de que a definição da RaiseAction esteja correta)
    server = actionlib.SimpleActionServer('raise_action', RaiseAction, execute_callback, auto_start=False)
    server.start()
    
    rospy.loginfo("Action Server 'raise_action' iniciado. Aguardando goals...")
    rospy.spin()
