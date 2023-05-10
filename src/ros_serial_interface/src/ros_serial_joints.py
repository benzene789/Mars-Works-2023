#!/user/bin/env
# OLD OLd OLD OLD OLD
# SEE ROS_SERIAL_JOINTS_1.PY
import serial
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import math
import time

#  0 or 1
arduino = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=.1)

class Subscriber():
    move_the_joints = True
    current_goal_joints = None
    def write_read(self,x):
        arduino.write(bytes(x, 'utf-8'))
        time.sleep(1)
        data = arduino.read_all()
        return data

    def callback(self, data):
        y = [int(math.degrees(x)) for x in data.position]
        offseted_y = [40, 20, y[0], 115-y[2], 116-y[1], y[3], y[4], 0]
        t = tuple(offseted_y)
        self.joints = str(offseted_y)[1:-1]
        if self.current_goal_joints is None or self.current_goal_joints != self.joints :
            self.move_the_joints = True
            self.current_goal_joints != self.joints
        else:
            self.move_the_joints = False
        # rospy.loginfo(t)

    def __init__(self):
        self.node_name = "Subscriber_Node"
        topic_name = "rover_arm/joint_states"
        self.pub = rospy.Publisher(topic_name, String, queue_size=10)
        rospy.init_node(self.node_name, anonymous=True)
        self.sub = rospy.Subscriber(topic_name, JointState, self.callback)
        
        ######################################################################
        self.rate = rospy.Rate(10) # hz
                
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook) 
        
        rospy.loginfo(f"The '{self.node_name}' node is active...")
        ######################################################################

    def shutdownhook(self):
        print(f"Stopping the '{self.node_name}' node at: {rospy.get_time()}")
        self.ctrl_c = True

    def main_loop(self):
        if not self.ctrl_c:
            # num = input("data: ") # Taking input from user
            value = self.write_read(self.joints)
            rospy.loginfo(self.joints)
            # print(int(value)) # printing the value
            # publisher_message = f"{str(self.value)}"
            # self.pub.publish(publisher_message)
            self.rate.sleep()
        # rospy.spin()

if __name__ == '__main__':
    subscriber_instance = Subscriber()
    time.sleep(1)
    subscriber_instance.main_loop()