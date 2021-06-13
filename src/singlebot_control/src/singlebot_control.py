#!/usr/bin/env python
import roslib; roslib.load_manifest('singlebot_control')
import rospy, yaml, sys
from gazebo_msgs.msg import ModelStates, ContactsState
from sensor_msgs.msg import JointState , Imu
from std_msgs.msg import Float64
from rospy import numpy_msg
import math
import numpy as np

class SpringLoadedInvertedPendulum():
    def __init__(self):
        nn = Imu()
        self.phase = 'flight'
        self.pos = [0,0,0]
        self.vel = [0,0,0]
        self.vel_desired = [0,0,0]
        self.angular_velocity_desired = [0,0,0]
        self.orientation = nn.orientation
        self.angular_velocity = nn.angular_velocity
        self.linear_accelration = nn.linear_acceleration
        self.pub1 = None # publish joint angles
        self.pub2 = None
        self.pub3 = None
        self.joint_position = [0,0,0]
        self.joint_velocity = [0,0,0]
        self.joint_position_desired = [0,0,0]
        self.joint_effort_desired = [0,0,0]

        # setup joint parameters
        self.k1 = 100
        self.d1 = 10
        self.k2 = 100
        self.d2 = 10
        self.d3 = 10

        # set up SLIP parameters
        self.f = 1.0 # frequency
        self.m = 3 + 0.3 + 0.3
        self.k3 = (2*math.pi*self.f)**2 * self.m # needed spring constant
        print("k is " , self.k3 )
        self.Ts = math.pi*math.sqrt(self.m/self.k3)
        self.g = 9.81
        self.z_apex = 2.0 # apex height
        self.z_bottom = 1.0 # bottom height
        self.deltab = math.sqrt( (2*self.m*self.g * (self.z_apex - self.z_bottom))/self.k3 )
        print("deltab is ", self.deltab )
        self.r0 = self.z_bottom + self.deltab
        self.leg_rest_length = 1.2

    def calTouchdownAngle(self):
        gamma  = math.asin(   math.sqrt( self.vel[0]**2 + self.vel[1]**2)   * self.Ts / 2 / self.r0)
        lam = math.atan2(self.vel[1], self.vel[0])
        return (lam, gamma)
        # gamma_y  = math.asin(self.vel(1) * self.Ts / 2 / self.r0)

    def updateState(self, states):
        self.orientation = states.orientation
        self.angular_velocity = states.angular_velocity
        self.linear_accelration = states.linear_acceleration

    def subIMU(self, data):
        self.orientation = data.orientation
        self.angular_velocity = data.angular_velocity
        self.linear_accelration = data.linear_acceleration
        self.performControl()
        # print(self.phase)

    def subState(self, data):
        for i in range(len(data.name)):
            if data.name[i] == 'singlebot::base_link':
                self.pos[0] = data.pose[i].position.x
                self.pos[1] = data.pose[i].position.y
                self.pos[2] = data.pose[i].position.z
                self.vel[0] = data.twist[i].linear.x
                self.vel[1] = data.twist[i].linear.y
                self.vel[2] = data.twist[i].linear.z

    def subContact(self, data):
        if len(data.states) == 0:
            self.phase = 'flight'
        else:
            self.phase = 'stance'

    def pubJointPos(self,val, pubber):
        pubber.publish(val)

    def calJointAngle(self):
        # since desired angles are only applicable for flight 
        if self.phase == 'flight':
            jda = self.calTouchdownAngle()
            jfl = self.r0 - self.leg_rest_length
            self.joint_position_desired[0] = jda[0]
            self.joint_position_desired[1] = jda[1]
            self.joint_position_desired[2] = jfl
            # print(self.joint_position_desired)
        return

    def calJointEffort(self):
        if self.phase == 'flight':
            self.joint_effort_desired[0] = self.k1 * (self.joint_position_desired[0] - self.joint_position[0]) - self.d1 *self.joint_velocity[0]
            self.joint_effort_desired[1] = self.k2 * (self.joint_position_desired[1] - self.joint_position[1]) - self.d2 *self.joint_velocity[1]
            self.joint_effort_desired[2] = self.k3 * (self.joint_position_desired[2] - self.joint_position[2]) - self.d3 *self.joint_velocity[2]
        else:
            self.joint_effort_desired[0] = 0
            self.joint_effort_desired[1] = 0
            self.joint_effort_desired[2] = self.k3 * (self.joint_position_desired[2] - self.joint_position[2])
        return

    def subJointState(self, data):
        self.joint_position[2] = data.position[0]
        self.joint_position[0] = data.position[1]
        self.joint_position[1] = data.position[2]
        self.joint_velocity[2] = data.velocity[0]
        self.joint_velocity[0] = data.velocity[1]
        self.joint_velocity[1] = data.velocity[2]
        # print(self.joint_position)
        # print(data.position)


    def performControl(self):
        self.calJointAngle()
        self.calJointEffort()
        self.pubJointPos(self.joint_effort_desired[0], self.pub1)
        self.pubJointPos(self.joint_effort_desired[1], self.pub2)
        self.pubJointPos(self.joint_effort_desired[2], self.pub3)

if __name__ == '__main__':

    rospy.init_node('singlebot_controller', anonymous=True)

    SLIP = SpringLoadedInvertedPendulum()
    SLIP.pub1 = rospy.Publisher('/singlebot/joint1_effort_controller/command', Float64, queue_size=10)
    SLIP.pub2 = rospy.Publisher('/singlebot/joint2_effort_controller/command', Float64, queue_size=10)
    SLIP.pub3 = rospy.Publisher('/singlebot/joint3_effort_controller/command', Float64, queue_size=10)
    
    # SLIP.pub2.publish(0.5)


    # subscribe for all needed information
    rospy.Subscriber("/imu", Imu, SLIP.subIMU)
    rospy.Subscriber("/gazebo/link_states", ModelStates, SLIP.subState)
    rospy.Subscriber("/foot_back_contact", ContactsState, SLIP.subContact)
    rospy.Subscriber("/singlebot/joint_states", JointState, SLIP.subJointState)

    # since IMU is "real time" use imu signal for control

    
    rospy.spin()