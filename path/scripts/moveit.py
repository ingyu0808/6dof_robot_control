import sys
import rospy
import moveit_commander
from moveit_commander import RobotCommander, MoveGroupCommander
from geometry_msgs.msg import PoseStamped

# ROS 노드 초기화
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('moveit_demo', anonymous=True)

# 로봇 모델 초기화
robot = RobotCommander()

# 이동 그룹 초기화
group_name = "manipulator"
move_group = MoveGroupCommander(group_name)

# 로봇 팔 초기 위치 확인
print("Initial Pose:")
print(move_group.get_current_pose().pose)

# 로봇 팔을 이동시키기 위한 목표 포즈 설정
target_pose = PoseStamped()
target_pose.header.frame_id = robot.get_planning_frame()
target_pose.pose.position.x = 0.5
target_pose.pose.position.y = 0.0
target_pose.pose.position.z = 0.5
target_pose.pose.orientation.x = 0.0
target_pose.pose.orientation.y = 0.0
target_pose.pose.orientation.z = 0.0
target_pose.pose.orientation.w = 1.0

# 로봇 팔 이동 계획 생성
move_group.set_pose_target(target_pose)
plan = move_group.plan()

# 계획된 이동 경로 확인
print("Plan:")
print(plan)

# 계획된 이동 경로 실행
move_group.execute(plan)

# MoveIt 종료
moveit_commander.roscpp_shutdown()
