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
    def __init__(self):
        self.group = MoveGroupCommander("arm")
        self.gripper_group = MoveGroupCommander("gripper")
        self.group.set_planning_time(600.0)
        self.gripper_joint_angle = self.gripper_group.get_current_joint_values()

    def moveGripper(self, angle):
        self.gripper_joint_angle[0] = angle
        self.gripper_group.set_joint_value_target(self.gripper_joint_angle)
        self.gripper_group.go()
        self.gripper_group.stop()
        self.gripper_group.clear_pose_targets()

    def holdBrush(self):
        rospy.loginfo("hold a brush.")
        self.moveGripper(0.01)
        self.move(0.2, 0.0, 0.03, pi/2.0)
        self.moveGripper(-0.01)
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

    def moveDefault(self):
        self.move(0.2, 0.0, 0.25, 0.0)

    def logPose(self):
        pose_current = self.group.get_current_pose()
        rospy.loginfo( "Get Current Pose:\n{}\n".format( pose_current ) )
        

def main():
    node_name = "arm_demo2"
    rospy.init_node(node_name, anonymous=True ) # ノードの初期化

    arm = Arm()
    arm.moveDefault()
    arm.logPose()
    arm.holdBrush()
    arm.logPose()


# グリッパを開く 
def open_gripper():
    print("Opening Gripper...")

    # gripper_joint_angle[0] = -0.01 # 実機：-0.01, シミュレータ： 0.01
    gripper_joint_angle[0] = 0.01 # 実機：-0.01, シミュレータ： 0.01

    gripper_group.set_joint_value_target(gripper_joint_angle)
    plan2 = gripper_group.go()
    gripper_group.stop()
    gripper_group.clear_pose_targets()
    # rospy.sleep(1)


# グリッパを閉じる
def close_gripper():

    print("Closing Gripper...")
    # gripper_joint_angle[0] = 0.0 # 実機：0.0，シミュレータ：-0.01
    gripper_joint_angle[0] = -0.01 # 実機：0.0，シミュレータ：-0.01
    gripper_group.set_joint_value_target(gripper_joint_angle)
    plan2 = gripper_group.go()
    gripper_group.stop()
    gripper_group.clear_pose_targets()
    # rospy.sleep(1)


if __name__ == '__main__':
    
    node_name = "arm_demo2"
    rospy.init_node(node_name, anonymous=True ) # ノードの初期化

    main()

    exit()

    ## MoveGroupCommanderオブジェクトのインスタンス生成。これがジョイントへのインタフェースになる。
    ## このインタフェースは運動計画と動作実行に使われる。
    group = MoveGroupCommander("arm")
    gripper_group = MoveGroupCommander("gripper") # 追加
    
    group.set_planning_time(600.0) # 動作計画に使う時間[s]の設定
    
    # 初期姿勢の取得
    pose_init = group.get_current_pose()  # エンドエフェクタの位置(x, y, z)を取得
    rospy.loginfo( "Get Initial Pose\n{}".format( pose_init ) )
    rpy_init  = group.get_current_rpy()   # エンドエフェクタの姿勢(roll, pitch, yaw)を取得
    rospy.loginfo( "Get Initial RPY:{}".format( rpy_init ) )
    
    # グリッパの初期値を取得
    gripper_joint_angle = gripper_group.get_current_joint_values()
    print("Get Current Gripper angle:\n{}\n".format(gripper_joint_angle))


    # 姿勢の定義
    # 初期姿勢
    rospy.loginfo( "Starting Pose Init")    
    pose_init = Pose()
    pose_init.position.x =  0.2
    pose_init.position.y =  0.0
    pose_init.position.z =  0.1
    theta = math.pi/2
    v = [0, 1, 0]
    pose_init.orientation.x =  v[0]*math.sin(theta/2)  # 姿勢(クオータニオン)
    pose_init.orientation.y =  v[1]*math.sin(theta/2)  # 姿勢(クオータニオン)
    pose_init.orientation.z =  v[2]*math.sin(theta/2)  # 姿勢(クオータニオン)
    pose_init.orientation.w =  math.cos(theta/2) # 姿勢(クオータニオン)  
    
    # 姿勢1
    rospy.loginfo( "Starting Pose 1")    
    pose_target_1 = Pose()
    pose_target_1.position.x =  0.15
    pose_target_1.position.y =  0.10
    pose_target_1.position.z =  0.048
    pose_target_1.orientation.x = -0.2706  # 姿勢(クオータニオン)
    pose_target_1.orientation.y =  0.6533  # 姿勢(クオータニオン)
    pose_target_1.orientation.z =  0.2706  # 姿勢(クオータニオン)
    pose_target_1.orientation.w =  0.6533  # 姿勢(クオータニオン)  


    # 動作開始
    group.set_joint_value_target(pose_init, True)
    group.go()
    rospy.sleep(5.0)
    pose_current = group.get_current_pose()
    rospy.loginfo( "Get Current Pose:\n{}\n".format( pose_current ) )

    open_gripper()
    rospy.sleep(2.0)
    close_gripper()
    
    # 初期姿勢に戻る
    rospy.loginfo( "Back to Initial Pose")
    group.set_joint_value_target(pose_init, True)
    group.go()
    rospy.sleep(5.0)
    pose_current = group.get_current_pose()
    rospy.loginfo( "Get Current Pose:\n{}\n".format( pose_current ) )