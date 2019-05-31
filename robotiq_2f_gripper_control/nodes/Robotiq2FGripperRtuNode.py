#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Robotiq, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Robotiq, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2012, Robotiq, Inc.
# Revision $Id$

"""@package docstring
ROS node for controling a Robotiq 2F gripper using the Modbus RTU protocol.

The script takes as an argument the IP address of the gripper. It initializes a baseRobotiq2FGripper object and adds a comModbusTcp client to it. It then loops forever, reading the gripper status and updating its command. The gripper status is published on the 'Robotiq2FGripperRobotInput' topic using the 'Robotiq2FGripper_robot_input' msg type. The node subscribes to the 'Robotiq2FGripperRobotOutput' topic for new commands using the 'Robotiq2FGripper_robot_output' msg type. Examples are provided to control the gripper (Robotiq2FGripperSimpleController.py) and interpreting its status (Robotiq2FGripperStatusListener.py).
"""

import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
roslib.load_manifest('robotiq_modbus_rtu')
import rospy
import robotiq_2f_gripper_control.baseRobotiq2FGripper
import robotiq_modbus_rtu.comModbusRtu
import os, sys
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input  as inputMsg
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg
from sensor_msgs.msg import JointState

def mainLoop():
    #Gripper is a 2F with a TCP connection
    gripper = robotiq_2f_gripper_control.baseRobotiq2FGripper.robotiqbaseRobotiq2FGripper()
    gripper.client = robotiq_modbus_rtu.comModbusRtu.communication()

    rospy.init_node('robotiq2FGripper')
    device = rospy.get_param('~port', '/dev/ttyUSB0')

    #We connect to the address received as an argument
    gripper.client.connectToDevice(device)

    #The Gripper status is published on the topic named 'Robotiq2FGripperRobotInput'
    pub = rospy.Publisher('Robotiq2FGripperRobotInput', inputMsg.Robotiq2FGripper_robot_input, queue_size=5)

    #The Gripper command is received from the topic named 'Robotiq2FGripperRobotOutput'
    rospy.Subscriber('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, gripper.refreshCommand)

    # Publish joint state
    joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=2)
    joint_states = JointState()
    joint_names = ['robotiq_85_left_knuckle_joint', 'robotiq_85_right_knuckle_joint',
                   'robotiq_85_left_inner_knuckle_joint', 'robotiq_85_right_inner_knuckle_joint',
                   'robotiq_85_left_finger_tip_joint', 'robotiq_85_right_finger_tip_joint']
    joint_pos_sign = [1., 1., 1., 1., -1., -1.]
    joint_states.name = joint_names

    #We loop
    joint_idx = 0
    while not rospy.is_shutdown():
        #Get and publish the Gripper status
        status = gripper.getStatus()
        pub.publish(status)

        # publish joint states
        active = status.gSTA
        pos = status.gPO if active else 0
        pos = (pos / 255.) * 0.8
        joint_pos = [pos * sign for sign in joint_pos_sign]
        joint_states.header.stamp = rospy.get_rostime()
        joint_states.header.seq = joint_idx
        joint_states.position = joint_pos
        joint_pub.publish(joint_states)

        #Wait a little
        # rospy.sleep(0.05)

        #Send the most recent command
        gripper.sendCommand()

        #Wait a little
        #rospy.sleep(0.05)



if __name__ == '__main__':
    try:
        mainLoop()
    except rospy.ROSInterruptException: pass
