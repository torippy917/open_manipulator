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
    rospy.sleep(1)


# グリッパを閉じる
def close_gripper():

    print("Closing Gripper...")
    gripper_joint_angle[0] = -0.0 # 実機：0.0，シミュレータ：-0.01
    # gripper_joint_angle[0] = -0.01 # 実機：0.0，シミュレータ：-0.01
    gripper_group.set_joint_value_target(gripper_joint_angle)
    plan2 = gripper_group.go()
    gripper_group.stop()
    gripper_group.clear_pose_targets()
    rospy.sleep(1)


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
    
    # 姿勢1
    rospy.loginfo( "Starting Pose 1")
    pose_target_1 =  [ 0.12, 0.0, 0.1, 0.0, math.pi/2.0, 0.0 ] # [ x, y, z, r, p, y ]
    group.set_pose_target( pose_target_1 ) # エンドエフェクタの姿勢設定
    group.go()    
    rospy.sleep(1.0) # 5秒のスリープ
    pose_current = group.get_current_pose()
    rospy.loginfo( "Get Current Pose:\n{}\n".format( pose_current ) )
    pose_init = pose_current
    
    open_gripper()
    
    # 姿勢３
    rospy.loginfo( "Starting Pose 3")    
    pose_target_3 = Pose()
    pose_target_3.position.x =  0.20
    pose_target_3.position.y =  0.10
    pose_target_3.position.z =  0.04
    pose_target_3.orientation.x = -0.2706  # 姿勢(クオータニオン)
    pose_target_3.orientation.y =  0.6533  # 姿勢(クオータニオン)
    pose_target_3.orientation.z =  0.2706  # 姿勢(クオータニオン)
    pose_target_3.orientation.w =  0.6533  # 姿勢(クオータニオン)  

    # 逆運動学で近似解を計算するためのエンドエフェクタの姿勢設定．2番目の引数がTrueだと近似解．
    # 2番目の引数がFalseまたは，無い場合は厳密解となる．
    group.set_joint_value_target(pose_target_3, True)
    group.go()    
    rospy.sleep(5.0)
    pose_current = group.get_current_pose()
    rospy.loginfo( "Get Current Pose:\n{}\n".format( pose_current ) )

    close_gripper()
    
    # # Pose 3 Z:-0.05[m]
    # rospy.loginfo( "Starting Pose 3 Z:-0.05[m]")
    # pose_target_3.position.z -= 0.05    
    # group.set_joint_value_target(pose_target_3, True )
    # group.go()    
    # rospy.sleep(5.0)
    # pose_current = group.get_current_pose()
    # rospy.loginfo( "Get Current Pose:\n{}\n".format( pose_current ) )
    
    # # 姿勢４
    # rospy.loginfo( "Starting Pose 4")    
    # pose_target_4 = Pose()
    # pose_target_4.position.x =  0.10
    # pose_target_4.position.y = -0.10
    # pose_target_4.position.z =  0.05
    # pose_target_4.orientation.x =  0.2706
    # pose_target_4.orientation.y =  0.6533
    # pose_target_4.orientation.z = -0.2706
    # pose_target_4.orientation.w =  0.6533    
    # group.set_joint_value_target( pose_target_4, True )
    # group.go()    
    # rospy.sleep(5.0)
    # pose_current = group.get_current_pose()
    # rospy.loginfo( "Get Current Pose:\n{}\n".format(pose_current ) )
    
    # # 姿勢４ Z:+0.05[m]
    # rospy.loginfo( "Starting Pose 4 Z:+0.05[m]")
    # pose_target_4.position.z += 0.05    
    # group.set_joint_value_target( pose_target_4, True )
    # group.go()    
    # rospy.sleep(5.0)
    # pose_current = group.get_current_pose()
    # rospy.loginfo( "Get Current Pose:\n{}\n".format( pose_current ) )
    
    # 初期姿勢に戻る
    rospy.loginfo( "Back to Initial Pose")
    group.set_joint_value_target( pose_init, True )
    group.go()
    
    rospy.sleep(5.0)
    pose_current = group.get_current_pose()
    rospy.loginfo( "Get Current Pose:\n{}\n".format( pose_current ) )