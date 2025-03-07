import rospy
import actionlib
import actionlib_tutorials.msg


class RaiseAction(object):
  _feedback = actionlib_tutorials.msg.RaiseFeedback()
  _result = actionlib_tutorials.msg.RaiseResult()

  def __init__(self, name):
    self._action_name = name
    self._as = actionlib.SimpleActionServer(self._action_name, actionlib_tutorials.msg.RaiseAction, execute_cb=self.execute_cb, auto_start = False)
    self._as.start()

  def execute_cb(self, goal):
    r = rospy.Rate(1)
    success = True

    #info
    rospy.loginfo("Goal received")

    
    #feedback
    self._feedback.current_angle = 0
    self._as.publish_feedback(self._feedback)
    r.sleep()

if __name__ == '__main__':
  rospy.init_node('raise_action')
  server = RaiseAction(rospy.get_name())
  rospy.spin()



  


