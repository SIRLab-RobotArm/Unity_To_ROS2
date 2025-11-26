#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotState


class IKSolverNode(Node):
    def __init__(self):
        super().__init__('ik_solver_node')

        # Unity → ROS (목표 Pose)
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/unity_target_pose',
            self.pose_callback,
            10
        )

        # ROS → Unity (IK 결과 JointState)
        self.joint_pub = self.create_publisher(
            JointState,
            '/ik_joint_commands',
            10
        )

        # MoveIt2 IK 서비스
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')

        self.get_logger().info("Waiting for /compute_ik service...")
        self.ik_client.wait_for_service()
        self.get_logger().info("IK Solver Node Ready.")

    def pose_callback(self, pose_msg: PoseStamped):
        """Unity에서 PoseStamped를 받으면 IK 계산 요청"""

        self.get_logger().info("[1] Pose callback triggered! Unity → ROS message received.")
        self.get_logger().info(
            f"[1] Pose: ({pose_msg.pose.position.x:.3f}, "
            f"{pose_msg.pose.position.y:.3f}, "
            f"{pose_msg.pose.position.z:.3f})"
        )

        # IK 요청 만들기
        req = GetPositionIK.Request()
        req.ik_request.group_name = "arm"  # SRDF에서 확인한 group 이름
        req.ik_request.pose_stamped = pose_msg
        req.ik_request.robot_state = RobotState()

        self.get_logger().info("[2] Sending IK request to MoveIt (/compute_ik)...")

        # 비동기 호출 + 완료 콜백 등록
        future = self.ik_client.call_async(req)
        future.add_done_callback(self.ik_response_callback)

    def ik_response_callback(self, future):
        """IK 서비스 응답을 받았을 때 호출되는 콜백"""
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f"[3] IK service call raised exception: {repr(e)}")
            return

        if res is None:
            self.get_logger().error("[3] IK service returned NONE! (MoveIt 내부 에러 가능)")
            return

        self.get_logger().info("[3] IK service responded.")

        sol = res.solution.joint_state

        if len(sol.name) == 0:
            self.get_logger().warn("[4] IK solution NOT FOUND! (unreachable pose)")
            return

        self.get_logger().info(f"[4] IK solution FOUND: {sol.name}")
        self.get_logger().info(f"[4] Joint values: {sol.position}")

        self.get_logger().info("[5] Publishing IK joint commands to /ik_joint_commands")
        self.joint_pub.publish(sol)


def main(args=None):
    rclpy.init(args=args)
    node = IKSolverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

