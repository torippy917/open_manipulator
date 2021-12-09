#!/usr/bin/env python
# -*- coding: utf-8 -*-    #日本語のコメントを入れるためのおまじない

import sys, math, copy
import rospy, tf, geometry_msgs.msg

from moveit_commander import MoveGroupCommander, RobotCommander
from geometry_msgs.msg import Pose, PoseStamped

# グリッパを開く 
def open_gripper():
    print("Opening Gripper...")

    gripper_joint_angle[0] = -0.01 # 実機：-0.01, シミュレータ： 0.01
    # gripper_joint_angle[0] = 0.01 # 実機：-0.01, シミュレータ： 0.01

    gripper_group.set_joint_value_target(gripper_joint_angle)
    plan2 = gripper_group.go()
    gripper_group.stop()
    gripper_group.clear_pose_targets()
    # rospy.sleep(1)


# グリッパを閉じる
def close_gripper():

    print("Closing Gripper...")
    gripper_joint_angle[0] = -0.0 # 実機：0.0，シミュレータ：-0.01
    # gripper_joint_angle[0] = -0.01 # 実機：0.0，シミュレータ：-0.01
    gripper_group.set_joint_value_target(gripper_joint_angle)
    plan2 = gripper_group.go()
    gripper_group.stop()
    gripper_group.clear_pose_targets()
    # rospy.sleep(1)


if __name__ == '__main__':
    
    node_name = "arm_demo2"
    rospy.init_node(node_name, anonymous=True ) # ノードの初期化

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
    pose_init.position.x =  0.20
    pose_init.position.y =  0.10
    pose_init.position.z =  0.25
    pose_init.orientation.x = -0.2706  # 姿勢(クオータニオン)
    pose_init.orientation.y =  0.6533  # 姿勢(クオータニオン)
    pose_init.orientation.z =  0.2706  # 姿勢(クオータニオン)
    pose_init.orientation.w =  0.6533  # 姿勢(クオータニオン)  
    
    # 姿勢1
    rospy.loginfo( "Starting Pose 1")    
    pose_target_1 = Pose()
    pose_target_1.position.x =  0.20
    pose_target_1.position.y =  0.10
    pose_target_1.position.z =  0.04
    pose_target_1.orientation.x = -0.2706  # 姿勢(クオータニオン)
    pose_target_1.orientation.y =  0.6533  # 姿勢(クオータニオン)
    pose_target_1.orientation.z =  0.2706  # 姿勢(クオータニオン)
    pose_target_1.orientation.w =  0.6533  # 姿勢(クオータニオン)  


    # 動作開始
    close_gripper()

    group.set_joint_value_target(pose_init, True)
    group.go()
    rospy.sleep(1.0)
    pose_current = group.get_current_pose()
    rospy.loginfo( "Get Current Pose:\n{}\n".format( pose_current ) )

    group.set_joint_value_target(pose_target_1, True)
    group.go()    
    rospy.sleep(1.0)
    pose_current = group.get_current_pose()
    rospy.loginfo( "Get Current Pose:\n{}\n".format( pose_current ) )

    close_gripper()
    
    # 初期姿勢に戻る
    rospy.loginfo( "Back to Initial Pose")
    group.set_joint_value_target( pose_init, True )
    group.go()
    rospy.sleep(1.0)
    pose_current = group.get_current_pose()
    rospy.loginfo( "Get Current Pose:\n{}\n".format( pose_current ) )