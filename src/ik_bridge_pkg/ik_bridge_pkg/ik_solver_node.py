#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotState


class IKSolverNode(Node):
    def __init__(self):
        super().__init__("dual_ik_solver")

        # ===== 공통 JointState 퍼블리셔 (MoveIt용) =====
        # MoveIt의 /move_group 이 듣는 토픽: /joint_states
        self.joint_state_pub = self.create_publisher(
            JointState,
            "/joint_states",
            10
        )

        # ===== LEFT arm =====
        self.left_pose_sub = self.create_subscription(
            PoseStamped,
            "/left_arm_target_pose",
            self.left_pose_callback,
            10
        )
        self.left_joint_pub = self.create_publisher(
            JointState,
            "/ik_left_joint_commands",
            10
        )

        # ===== RIGHT arm =====
        self.right_pose_sub = self.create_subscription(
            PoseStamped,
            "/right_arm_target_pose",
            self.right_pose_callback,
            10
        )
        self.right_joint_pub = self.create_publisher(
            JointState,
            "/ik_right_joint_commands",
            10
        )

        # ===== MoveIt IK client =====
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self.get_logger().info("Waiting for /compute_ik...")
        self.ik_client.wait_for_service()
        self.get_logger().info("Dual IK Solver Ready.")

    # ==========================
    # Left arm pose callback
    # ==========================
    def left_pose_callback(self, pose_msg):
        self.get_logger().info("[LEFT] Received pose")
        self.send_ik_request(pose_msg, arm="left")

    # ==========================
    # Right arm pose callback
    # ==========================
    def right_pose_callback(self, pose_msg):
        self.get_logger().info("[RIGHT] Received pose")
        self.send_ik_request(pose_msg, arm="right")

    # ==========================
    # Common IK request
    # ==========================
    def send_ik_request(self, pose_msg, arm: str):
        # frame_id 없으면 world로 강제
        if not pose_msg.header.frame_id:
            pose_msg.header.frame_id = "world"

        req = GetPositionIK.Request()
        req.ik_request.group_name = "arm"
        req.ik_request.pose_stamped = pose_msg
        req.ik_request.robot_state = RobotState()

        future = self.ik_client.call_async(req)
        future.add_done_callback(lambda f: self.process_ik_result(f, arm))

    # ==========================
    # IK result callback
    # ==========================
    def process_ik_result(self, future, arm: str):
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f"[{arm.upper()}] IK error: {e}")
            return

        sol: JointState = res.solution.joint_state
        if len(sol.name) == 0:
            self.get_logger().warn(f"[{arm.upper()}] IK solution NOT found")
            return

        self.get_logger().info(
            f"[{arm.upper()}] IK returned names={list(sol.name)} "
            f"pos={[round(p, 3) for p in sol.position]}"
        )

        # 1) Unity 각각 팔로 퍼블리시
        if arm == "left":
            self.left_joint_pub.publish(sol)
        else:
            self.right_joint_pub.publish(sol)

        # 2) MoveIt이 듣는 /joint_states 로도 퍼블리시
        sol.header.stamp = self.get_clock().now().to_msg()
        self.joint_state_pub.publish(sol)


def main(args=None):
    rclpy.init(args=args)
    node = IKSolverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

