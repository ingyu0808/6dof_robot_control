#!/usr/bin/env python3

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

rospy.init_node('moveit_python_example', anonymous=True)

# MoveIt commander를 초기화합니다
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "ikfk"
group = moveit_commander.MoveGroupCommander(group_name)

# 로봇 팔의 현재 상태를 가져옵니다
start_state = moveit_msgs.msg.RobotState()
start_state.joint_state.name = group.get_active_joints()
start_state.joint_state.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 현재 상태를 변경할 값으로 설정합니다

robot_state = robot.get_current_state()
robot_state.joint_state.position = start_state.joint_state.position
group.set_start_state(robot_state)

