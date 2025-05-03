#!/usr/bin/env python3
import rospy
import actionlib
import tf
from sensor_msgs.msg import Imu
from marta_msgs.msg import RaiseAction, RaiseFeedback, RaiseResult
import subprocess
import json

current_angle = None  # Inicialmente sem leitura válida

def imu_callback(data):
    global current_angle
    q = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(q)
    current_angle = pitch
    rospy.logdebug("Pitch atualizado: {:.3f} rad".format(pitch))

def call_raise_front():
    rospy.loginfo("Chamando nó de levantamento pela frente...")
    subprocess.Popen(["rosrun", "martha_inverse_kinematics", "levante_frente_raw_angle.py"])

def call_raise_back():
    rospy.loginfo("Chamando nó de levantamento por trás...")
    subprocess.Popen(["rosrun", "martha_inverse_kinematics", "levante_tras_raw_angle.py"])

def execute_callback(goal):
    rospy.loginfo("Goal recebido. Aguardando leitura válida do IMU...")

    try:
        goal_data = json.loads(goal.json)
        rospy.loginfo("Goal decodificado: %s", goal_data)
    except json.JSONDecodeError:
        rospy.logerr("Goal recebido não é um JSON válido.")
        result = RaiseResult()
        result.json = json.dumps({
            "success": False,
            "message": "JSON inválido recebido no goal."
        })
        server.set_aborted(result)
        return

    # Espera até o IMU fornecer um valor válido (máx 5s)
    wait_time = 0
    while current_angle is None and wait_time < 5.0:
        rospy.sleep(0.1)
        wait_time += 0.1

    if current_angle is None:
        rospy.logerr("Falha ao obter leitura do IMU.")
        result = RaiseResult()
        result.json = json.dumps({
            "success": False,
            "message": "Falha ao obter leitura do IMU."
        })
        server.set_aborted(result)
        return

    rospy.sleep(0.5)  # Garante leitura estável
    angle = current_angle
    THRESHOLD = 0.2

    # Feedback com valor atual de pitch
    feedback = RaiseFeedback()
    feedback.json = json.dumps({"pitch": angle})
    server.publish_feedback(feedback)

    result = RaiseResult()

    if angle > THRESHOLD:
        rospy.loginfo("Ângulo > +{:.2f} rad. Levantando pela frente.".format(THRESHOLD))
        call_raise_front()
        result_dict = {
            "success": True,
            "message": "Levantamento pela frente acionado.",
            "pitch": angle
        }
    elif angle < -THRESHOLD:
        rospy.loginfo("Ângulo < -{:.2f} rad. Levantando por trás.".format(THRESHOLD))
        call_raise_back()
        result_dict = {
            "success": True,
            "message": "Levantamento por trás acionado.",
            "pitch": angle
        }
    else:
        rospy.logwarn("Ângulo dentro do threshold. Nenhuma ação executada.")
        result_dict = {
            "success": False,
            "message": "Ângulo dentro do threshold; ação não executada.",
            "pitch": angle
        }

    result.json = json.dumps(result_dict)
    server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node('raise_action_server')
    rospy.Subscriber('/imu', Imu, imu_callback)
    server = actionlib.SimpleActionServer('raise_action', RaiseAction, execute_callback, auto_start=False)
    server.start()
    rospy.loginfo("Action Server 'raise_action' iniciado. Aguardando goals...")
    rospy.spin()
