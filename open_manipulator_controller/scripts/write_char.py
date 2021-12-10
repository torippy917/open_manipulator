#!/usr/bin/env python
# -*- coding: utf-8 -*-    #日本語のコメントを入れるためのおまじない

from re import T
import sys, math, copy
import rospy, tf, geometry_msgs.msg
import numpy as np
import quaternion

from moveit_commander import MoveGroupCommander, RobotCommander
from geometry_msgs.msg import Pose, PoseStamped

pi = math.pi

# 回転ベクトルからクオータニオンを生成する.
def rotationVector2quaternion(vec, rad):
    v = vec.copy()
    norm = np.linalg.norm(v)
    v = v/norm
    w = math.cos(rad/2.0)
    v = v*math.sin(rad/2.0)
    return np.quaternion(w, v[0], v[1], v[2])
# エイリアス
rv2q = rotationVector2quaternion


class Arm:
    def __init__(self, write_origin, resolution = 0.05, log = True):
        self.group = MoveGroupCommander("arm")
        self.gripper_group = MoveGroupCommander("gripper")
        self.group.set_planning_time(600.0)
        self.gripper_joint_angle = self.gripper_group.get_current_joint_values()
        self.write_origin = np.array(write_origin)
        self.resolution = resolution
        self.z_write = 0.13
        self.z_move = 0.15
        self.pitch_write = 0.0
        self.log = log

    def moveGripper(self, angle):
        self.gripper_joint_angle[0] = angle
        self.gripper_group.set_joint_value_target(self.gripper_joint_angle)
        self.gripper_group.go()
        self.gripper_group.stop()
        self.gripper_group.clear_pose_targets()

    def holdBrush(self):
        rospy.loginfo("hold a brush.")
        self.moveGripper(-0.01)
        self.move(0.2, 0.0, 0.03, pi/2.0)
        self.moveGripper(0.01)
        self.move(0.2, 0.0, 0.25, pi/2.0)
        self.move(0.2, 0.0, 0.25, 0.0)
            
    def move(self, x, y, z, pitch):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        quat = rv2q(np.array([0,1,0]), pitch)
        quat = rv2q(np.array([0,0,1]), math.atan2(y,x)) * quat
        pose.orientation.w = quat.w
        pose.orientation.x = quat.x
        pose.orientation.y = quat.y
        pose.orientation.z = quat.z
        self.group.set_joint_value_target(pose, True)
        self.group.go()
        if self.log:
            self.logPose()

    def moveDefault(self):
        self.move(0.2, 0.0, 0.25, 0.0)

    def write(self, line):
        line2 = np.array(line) + self.write_origin
        self.move(line2[0,0], line2[0,1], self.z_move, self.pitch_write)
        for i in range(len(line2)-1):
            n = math.ceil(math.hypot(line2[i+1,0]-line2[i,0], line2[i+1,1]-line2[i,1])/self.resolution)
            for t in [i/n for i in range(n+1)]:
                self.move(t*line2[i+1,0]+(1-t)*line2[i,0], t*line2[i+1,1]+(1-t)*line2[i,1], self.z_write, self.pitch_write)
        self.move(line2[-1,0], line2[-1,1], self.z_move, self.pitch_write)

    def logPose(self):
        pose_current = self.group.get_current_pose()
        rospy.loginfo( "Get Current Pose:\n{}\n".format( pose_current ) )
        

def main():
    node_name = "arm_demo2"
    rospy.init_node(node_name, anonymous=True ) # ノードの初期化

    arm = Arm([0,0])
    arm.moveDefault()
    # arm.holdBrush()
    arm.move(0.2, 0.0, 0.13, 0.0)
    # line0 = [[61.0, 60.0], [22.0, 66.0]]
    # line1 = [[61.0, 60.0], [61.0, 43.0], [22.0, 43.0]]
    # line2 = [[44.0, 60.0], [44,0, 43.0]]
    # line3 = [[22.0, 60.0], [22.0, 43.0]]
    # line4 = [[63.0, 30.0], [31.0, 31.0], [23.0, 33.0], [13.0, 38.0], [5.0, 47.0]]
    # line5 = [[63.0, 30.0], [63.0, 8.0], [8.0, 8.0], [10.0, 19.0]]
    # line6 = [[42.0, 30.0], [42.0, 8.0]]
    # line7 = [[31.0, 31.0], [31.0, 8.0]]
    # arm.write(line0)
    # arm.write(line1)
    # arm.write(line2)
    # arm.write(line3)
    # arm.write(line4)
    # arm.write(line5)
    # arm.write(line6)
    # arm.write(line7)

    arm.moveDefault()


if __name__ == '__main__':
    main()