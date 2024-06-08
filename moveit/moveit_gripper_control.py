#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from moveit_msgs.msg import MoveItErrorCodes
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
import geometry_msgs.msg
import time

def move_to_pose():
    # ROS 노드 초기화
    rospy.init_node('move_to_pose_node', anonymous=True)
    state_check_pub = rospy.Publisher("state_check", Int32, queue_size=10)
    gripper_pub = rospy.Publisher("gripper_check", Int32, queue_size=10)
    
    # MoveIt! 초기화
    moveit_commander.roscpp_initialize(sys.argv)
    
    # 로봇 모델 로딩
    robot = moveit_commander.RobotCommander()

    # 로봇 팔 그룹 초기화
    arm_group = moveit_commander.MoveGroupCommander("arm_group")
    
    current_joint_values = arm_group.get_current_joint_values()
    print("Current joint values:", current_joint_values)
    
    # 목표 포즈 생성
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = 0.20
    pose_goal.position.y = -0.1
    pose_goal.position.z = 0.33
    pose_goal.orientation.x = -1
    pose_goal.orientation.y = 0
    pose_goal.orientation.z = 0
    pose_goal.orientation.w = 0
    
    arm_group.set_pose_target(pose_goal)
    plan = arm_group.go(wait=True)  # 이동이 완료될 때까지 대기
    
    # 동작이 완료되었는지 확인
    if plan:
        print("Movement successful!")
        time.sleep(2)
        state_check_pub.publish(2)
        time.sleep(2)
        gripper_pub.publish(1)
        time.sleep(20)
        state_check_pub.publish(1)  # state_check 토픽에 1 발행
        # 여기에 동작 완료 후 수행할 작업 추가
    else:
        print("Movement failed!")
    
    arm_group.stop()
    arm_group.clear_pose_targets()


if __name__ == '__main__':
    try:
        move_to_pose()
    except rospy.ROSInterruptException:
        pass