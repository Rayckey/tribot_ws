#!/usr/bin/env python
import sys
import yaml
import numpy as np
import math
import tf2_ros
import tf.transformations
from rospy import numpy_msg
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState, Imu
from gazebo_msgs.msg import ModelStates, ContactsState
import rospy
import roslib
from copy import deepcopy
import json
roslib.load_manifest('singlebot_control')


class SpringLoadedInvertedPendulum():
    def __init__(self):
        nn = Imu()
        self.phase = 'flight'
        self.leg = 'back'
        self.leg_contact = [0,0,0]
        self.pos = [0, 0, 0]
        self.vel = Vector3()
        self.vel_desired = Vector3()
        self.vel_desired.y = 2.51327412287
        self.orientation = nn.orientation
        self.orientation_desired = [0, 0, 0]
        self.angular_velocity = nn.angular_velocity
        self.linear_accelration = nn.linear_acceleration
        self.leg_offset = 2.09439510239
        self.angular_velocity_desired = [-self.leg_offset*0.8, 0, 0]

        self.pub1 = None
        self.pub2 = None
        self.pub3 = None
        self.pub4 = None
        self.pub5 = None
        self.pub6 = None
        self.pub7 = None
        self.pub8 = None
        self.pub9 = None
        
        self.joint_position = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.joint_velocity = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.joint_position_desired = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.joint_effort_desired = [0, 0, 0, 0, 0, 0, 0, 0, 0]

        self.active_leg_position_desired = [0,0,0]
        self.active_leg_effort_desired = [0,0,0]

        # setup joint parameters
        self.k1 = 50
        self.d1 = 5
        self.k2 = 50
        self.d2 = 5
        self.d3 = 10

        # setup control parameters
        self.kyaw = 25
        self.kyawd = 3
        self.kyd = 0.05

        # set up SLIP parameters
        self.f = 1  # frequency
        self.m = 3 + 0.3 * 9 + 0.2
        self.k3 = (2.0*math.pi*self.f)**2 * self.m  # needed spring constant
        print("k is ", self.k3)
        self.Ts = math.pi*math.sqrt(self.m/self.k3)
        print("TS is ", self.Ts)
        self.g = 9.81
        self.z_apex = 2.0  # apex height
        self.z_bottom = 1.0  # bottom height
        self.deltab = math.sqrt(
            (2*self.m*self.g * (self.z_apex - self.z_bottom))/self.k3)
        self.r0 = self.z_bottom + self.deltab
        print("r0 is ", self.r0)
        self.leg_rest_length = 1.2

        self.colllect_data = {'body_pos':[] ,'body_vel':[], 'body_ori':[] , 'body_ori_vel':[] , 'joint_pos':[] , 'joint_vel':[], 'joint_eff':[]}


    def calTransformedAngle(self, jda):
        print("jda is ", jda)
        
        eu = tf.transformations.euler_from_quaternion(
            [self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w])
        eu = list(eu)
        eu[0] = rad2up(eu[0])

        print("current body angle is ", eu[0])
        if self.leg == 'left':
            theta = jda - (eu[0] - self.leg_offset*2.0)
        elif self.leg == 'right':
            theta = jda - (eu[0] - self.leg_offset)
        else:
            theta = jda - eu[0]
        theta = rad2mid(theta)
        print("new jda is ", theta)
        return theta

    def calTouchdownAngle(self):
        yfd = self.vel.y * self.Ts / 2.0 + self.kyd * \
            (self.vel.y - self.vel_desired.y)
        theta = math.asin(yfd/self.r0)
        return theta

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

        self.collectState()

        if len(self.colllect_data['body_pos']) > 400:
            with open('single_bot_data.json', 'w') as outfile:
                json.dump(self.colllect_data, outfile)
        print(self.phase)

    def subState(self, data):
        for i in range(len(data.name)):
            if data.name[i] == 'singlebot::robot_base':
                self.pos[0] = data.pose[i].position.x
                self.pos[1] = data.pose[i].position.y
                self.pos[2] = data.pose[i].position.z
                self.vel.x = data.twist[i].linear.x
                self.vel.y = data.twist[i].linear.y
                self.vel.z = data.twist[i].linear.z

    def subContact(self):
        if sum(self.leg_contact) == 0:
            self.phase = 'flight'
        else:
            self.phase = 'stance'

    def subContactback(self, data):
        if len(data.states) == 0:
            self.leg_contact[0] = 0
        else:
            self.leg_contact[0] = 1
        self.subContact()

    def subContactleft(self, data):
        if len(data.states) == 0:
            self.leg_contact[1] = 0
        else:
            self.leg_contact[1] = 1
        self.subContact()

    def subContactright(self, data):
        if len(data.states) == 0:
            self.leg_contact[2] = 0
        else:
            self.leg_contact[2] = 1
        self.subContact()

    def pubJointPos(self, val, pubber):
        pubber.publish(val)

    def calJointAngle(self):
        # since desired angles are only applicable for flight

        if self.phase == 'flight':
            print(' world vel is ', self.vel)
            # print(' body vel is ', self.body_vel)
            jda = self.calTouchdownAngle()
            jda = self.calTransformedAngle(jda)
            jfl = self.r0 - self.leg_rest_length
            self.active_leg_position_desired = [0,jda,jfl]
            # print(self.joint_position_desired)
        else:
            jfl = self.r0 - self.leg_rest_length
            
            self.active_leg_position_desired[0] = 0

            self.active_leg_position_desired[2] = jfl
        return

    def calJointEffort(self , leg = 'back'):

        eu = tf.transformations.euler_from_quaternion(
            [self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w])

        joint_position = [0,0,0]
        joint_velocity = [0,0,0] 
        joint_effort_desired = [0,0,0]

        if leg == 'back':
            joint_position = self.joint_position[:3]
            joint_velocity = self.joint_velocity[:3]
        elif leg == 'left' :
            joint_position = self.joint_position[3:6]
            joint_velocity = self.joint_velocity[3:6]
        else:
            joint_position = self.joint_position[6:]
            joint_velocity = self.joint_velocity[6:]

        if self.leg == leg:
            print('target points are ' ,self.active_leg_position_desired)
            if self.phase == 'flight':
                joint_effort_desired[0] = self.k1 * (
                    self.active_leg_position_desired[0] - joint_position[0]) - self.d1 * joint_velocity[0]
                joint_effort_desired[1] = self.k2 * (
                    self.active_leg_position_desired[1] - joint_position[1]) - self.d2 * joint_velocity[1]
                joint_effort_desired[2] = self.k3 * (
                    self.active_leg_position_desired[2] - joint_position[2]) - self.d3 * joint_velocity[2]
            else:
                joint_effort_desired[0] = self.k1 * (
                    self.active_leg_position_desired[0] - joint_position[0]) - self.d1 * joint_velocity[0]
                joint_effort_desired[1] = -(-self.kyaw * (
                    eu[0] - self.orientation_desired[0]) -self.kyawd * (self.angular_velocity.x - self.angular_velocity_desired[0]))
                joint_effort_desired[2] = self.k3 * \
                    (self.active_leg_position_desired[2] - joint_position[2])
                print(' body pitch is ', eu[0])
                print(' body vel error is ', self.angular_velocity.x)
            print('target efforts are ' ,joint_effort_desired)
        else:
            joint_effort_desired[0] = self.k1 * (0 - joint_position[0]) - self.d1 * joint_velocity[0]
            joint_effort_desired[1] = self.k2 * (0 - joint_position[1]) - self.d2 * joint_velocity[1]
            joint_effort_desired[2] = self.k3 * (0 - joint_position[2]) - self.d3 * joint_velocity[2]
            print('target points are ' ,self.active_leg_position_desired)
            print('target efforts are ' ,joint_effort_desired)


        if leg == 'back':
            self.joint_effort_desired[:3]  = joint_effort_desired
        elif leg == 'left' :
            self.joint_effort_desired[3:6] = joint_effort_desired
        else: 
            self.joint_effort_desired[6:]  = joint_effort_desired


        return 


    def subJointState(self, data):
        self.joint_position[2] = data.position[0]
        self.joint_position[5] = data.position[1]
        self.joint_position[8] = data.position[2]
        self.joint_position[0] = data.position[3]
        self.joint_position[3] = data.position[4]
        self.joint_position[6] = data.position[5]
        self.joint_position[1] = data.position[6]
        self.joint_position[4] = data.position[7]
        self.joint_position[7] = data.position[8]
        
        self.joint_velocity[2] = data.velocity[0]
        self.joint_velocity[5] = data.velocity[1]
        self.joint_velocity[8] = data.velocity[2]
        self.joint_velocity[0] = data.velocity[3]
        self.joint_velocity[3] = data.velocity[4]
        self.joint_velocity[6] = data.velocity[5]
        self.joint_velocity[1] = data.velocity[6]
        self.joint_velocity[4] = data.velocity[7]
        self.joint_velocity[7] = data.velocity[8]


    def performControl(self):
        self.orientation_desired[0] += 0.01*self.angular_velocity_desired[0]
        self.distinguishLeg()
        self.calJointAngle()
        self.calJointEffort('back')
        self.calJointEffort('left')
        self.calJointEffort('right')
        self.pubJointPos(self.joint_effort_desired[0], self.pub1)
        self.pubJointPos(self.joint_effort_desired[1], self.pub2)
        self.pubJointPos(self.joint_effort_desired[2], self.pub3)
        self.pubJointPos(self.joint_effort_desired[3], self.pub4)
        self.pubJointPos(self.joint_effort_desired[4], self.pub5)
        self.pubJointPos(self.joint_effort_desired[5], self.pub6)
        self.pubJointPos(self.joint_effort_desired[6], self.pub7)
        self.pubJointPos(self.joint_effort_desired[7], self.pub8)
        self.pubJointPos(self.joint_effort_desired[8], self.pub9)


    def distinguishLeg(self):
        eu = tf.transformations.euler_from_quaternion([self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w])
        eu = list(eu)
        eu[0] = rad2up(eu[0])

        if self.phase == 'flight':
            if self.leg_offset/2.0 < eu[0] <  self.leg_offset/2.0*3.0 : 
                self.leg = 'right'
            elif self.leg_offset/2.0*3.0 < eu[0] <  self.leg_offset/2.0*5.0 : 
                self.leg = 'left'
            else:
                self.leg = 'back'


        print('current leg is ' ,self.leg)



    def collectState(self):
        self.colllect_data['body_pos'].append(deepcopy(self.pos))
        self.colllect_data['body_vel'].append([self.vel.x,self.vel.y,self.vel.z])
        self.colllect_data['body_ori'].append( [self.orientation.x,self.orientation.y,self.orientation.z,self.orientation.w ])
        self.colllect_data['body_ori_vel'].append([self.angular_velocity.x, self.angular_velocity.y,self.angular_velocity.z])
        self.colllect_data['joint_pos'].append(deepcopy(self.joint_position) )
        self.colllect_data['joint_vel'].append(deepcopy(self.joint_velocity) )
        self.colllect_data['joint_eff'].append(deepcopy(self.joint_effort_desired))

def rad2up(ang):
    if ang < 0: 
        return ang+np.pi*2
    return ang


def rad2mid(ang):
    if ang > np.pi: 
        return rad2mid(ang-np.pi*2)
    elif ang < -np.pi:
        return rad2mid(ang+np.pi*2)
    return ang

if __name__ == '__main__':

    rospy.init_node('singlebot_controller', anonymous=True)

    SLIP = SpringLoadedInvertedPendulum()
    SLIP.pub1 = rospy.Publisher('/singlebot/joint1_back_effort_controller/command', Float64, queue_size=10)
    SLIP.pub2 = rospy.Publisher('/singlebot/joint2_back_effort_controller/command', Float64, queue_size=10)
    SLIP.pub3 = rospy.Publisher('/singlebot/joint3_back_effort_controller/command', Float64, queue_size=10)
    SLIP.pub4 = rospy.Publisher('/singlebot/joint1_left_effort_controller/command', Float64, queue_size=10)
    SLIP.pub5 = rospy.Publisher('/singlebot/joint2_left_effort_controller/command', Float64, queue_size=10)
    SLIP.pub6 = rospy.Publisher('/singlebot/joint3_left_effort_controller/command', Float64, queue_size=10)
    SLIP.pub7 = rospy.Publisher('/singlebot/joint1_right_effort_controller/command', Float64, queue_size=10)
    SLIP.pub8 = rospy.Publisher('/singlebot/joint2_right_effort_controller/command', Float64, queue_size=10)
    SLIP.pub9 = rospy.Publisher('/singlebot/joint3_right_effort_controller/command', Float64, queue_size=10)





    # SLIP.pub2.publish(0.5)

    # subscribe for all needed information
    rospy.Subscriber("/imu", Imu, SLIP.subIMU)
    rospy.Subscriber("/gazebo/link_states", ModelStates, SLIP.subState)
    rospy.Subscriber("/foot_back_contact", ContactsState, SLIP.subContactback)
    rospy.Subscriber("/foot_left_contact", ContactsState, SLIP.subContactleft)
    rospy.Subscriber("/foot_right_contact", ContactsState, SLIP.subContactright)
    rospy.Subscriber("/singlebot/joint_states", JointState, SLIP.subJointState)

    # since IMU is "real time" use imu signal for control

    rospy.spin()
