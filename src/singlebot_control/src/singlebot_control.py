#!/usr/bin/env python
import roslib; roslib.load_manifest('singlebot_control')
import rospy, yaml, sys
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import JointState , Imu
from std_msgs.msg import Float64
from rospy import numpy_msg
import numpy as np

class SpringLoadedInvertedPendulum():
    def __init__(self):
        nn = Imu()
        self.phase = 0 # 0 for flight, 1 for stance
        self.pos = np.array([0,0,0])
        self.orientation = nn.orientation
        self.angular_velocity = nn.angular_velocity
        self.linear_accelration = nn.linear_acceleration

    def updateState(self, states):
        self.orientation = states.orientation
        self.angular_velocity = states.angular_velocity
        self.linear_accelration = states.linear_acceleration

    def subIMU(self, data):
        self.orientation = data.orientation
        self.angular_velocity = data.angular_velocity
        self.linear_accelration = data.linear_acceleration

    def subState(self, data):
        for i in range(len(data.name)):
            if data.name[i] == 'base_link':
                self.pos[0] = data.pose[i].position.x
                self.pos[1] = data.pose[i].position.y
                self.pos[2] = data.pose[i].position.z

    def pubJointPos(jtn_num, val, pubber):
        # rate = rospy.Rate(10) # 10hz
        # while not rospy.is_shutdown():
        hello_val = val
        pubber.publish(hello_val)
        # rate.sleep()


if __name__ == '__main__':
    SLIP = SpringLoadedInvertedPendulum()
    pub1 = rospy.Publisher('/singlebot/joint1_position_controller/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('/singlebot/joint2_position_controller/command', Float64, queue_size=10)
    pub3 = rospy.Publisher('/singlebot/joint3_position_controller/command', Float64, queue_size=10)
    rospy.init_node('singlebot_controller', anonymous=True)
    # SLIP.pubJointPos(2, 0.5, pub2)

    # subscribe for all needed information
    rospy.Subscriber("imu", Imu, SLIP.subIMU)
    rospy.Subscriber("/gazebo/model_states", ModelStates, SLIP.subState)

    # since IMU is "real time" use imu signal for control

    rospy.spin()