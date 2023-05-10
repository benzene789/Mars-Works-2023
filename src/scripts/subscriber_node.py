#!/user/bin/env
import rospy
from std_msgs.msg import String
from panela.msg import Position

def callback(data):
	rospy.loginfo("%s X: %d Y: %d", data.message, data.x, data.y)

def listener():
	rospy.init_node("Subscriber_Node", anonymous = True)
	rospy.Subscriber('talking_topic', Position, callback)
	rospy.spin()

if __name__ == '__main__':
	try:
		listener()
	except rospy.ROSInterruptException:
		pass