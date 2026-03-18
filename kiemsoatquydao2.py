#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time
import threading

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, OrientationConstraint, PositionConstraint, JointConstraint, BoundingVolume
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive

class RobotWallFinal(Node):
    def __init__(self):
        super().__init__('robot_wall_final')
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        
        # 1. Tọa độ chốt (Dùng bộ Q mặc định nhìn ngang cho gắp chai)
        self.SAFE_WAYPOINT = [0.328, 0.0, 0.3]
        self.GRIP_Q = [0.0, 0.0, 0.0, 1.0] 
        self.HOME_JOINTS = [0.009, 1.465, -1.474, 0.012] 
        self.JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4"]
        self.GRIPPER_JOINTS = ["gripper_left_joint", "gripper_right_joint"]

        print("Đang đợi MoveIt server...")
        self._action_client.wait_for_server()
        print("-> HE THONG DA SAN SANG!")

    def send_goal_sync(self, goal_msg):
        future = self._action_client.send_goal_async(goal_msg)
        while rclpy.ok() and not future.done():
            time.sleep(0.01)
        handle = future.result()
        if not handle or not handle.accepted:
            return False
        result_future = handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            time.sleep(0.01)
        return True

    def control_gripper(self, action, custom_val=None):
        """Điều khiển kẹp bảo vệ motor cho chai cứng 6cm"""
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "gripper"
        
        if action == 'mo':
            target_pos = 0.019  # Mở rộng tối đa
        else:
            # CHÚ Ý: Với chai 6cm cứng, dùng giá trị DƯ (0.005 đến 0.008)
            # Không được dùng số âm để tránh cháy motor.
            target_pos = float(custom_val) if custom_val is not None else 0.006 
        
        constraints = Constraints()
        for joint in self.GRIPPER_JOINTS:
            jc = JointConstraint()
            jc.joint_name = joint
            jc.position = target_pos
            # Nới lỏng sai số: Nếu lệch 2mm vẫn coi là hoàn thành, không bắt motor gồng
            jc.tolerance_above = 0.002
            jc.tolerance_below = 0.002
            jc.weight = 0.3 # Giảm độ ưu tiên lệnh để motor chạy 'mềm' hơn
            constraints.joint_constraints.append(jc)
        
        goal_msg.request.goal_constraints.append(constraints)
        print(f"-> {action.upper()} kẹp (Target: {target_pos})...")
        return self.send_goal_sync(goal_msg)

    def move_joint(self, joint_values):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "arm"
        constraints = Constraints()
        for name, val in zip(self.JOINT_NAMES, joint_values):
            jc = JointConstraint(joint_name=name, position=float(val), weight=1.0)
            constraints.joint_constraints.append(jc)
        goal_msg.request.goal_constraints.append(constraints)
        return self.send_goal_sync(goal_msg)

    def move_pose(self, x, y, z, q_list):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "arm"
        goal_msg.request.allowed_planning_time = 5.0
        
        constraints = Constraints()
        pcm = PositionConstraint()
        pcm.header.frame_id = "world"
        pcm.link_name = "end_effector_link"
        cbox = BoundingVolume()
        # Nới lỏng vùng đích lên 2cm để robot không bị khựng
        cbox.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.02, 0.02, 0.02])]
        target = Pose()
        target.position.x, target.position.y, target.position.z = float(x), float(y), float(z)
        cbox.primitive_poses = [target]
        pcm.constraint_region = cbox
        constraints.position_constraints.append(pcm)

        ocm = OrientationConstraint()
        ocm.header.frame_id = "world"
        ocm.link_name = "end_effector_link"
        ocm.orientation.x, ocm.orientation.y, ocm.orientation.z, ocm.orientation.w = q_list
        ocm.absolute_x_axis_tolerance = 0.2
        ocm.absolute_y_axis_tolerance = 0.2
        ocm.absolute_z_axis_tolerance = 0.2
        constraints.orientation_constraints.append(ocm)

        goal_msg.request.goal_constraints.append(constraints)
        return self.send_goal_sync(goal_msg)

def run_loop(node):
    while rclpy.ok():
        try:
            line = input("\n>> Nhập XYZ, 'h', 'mo' hoặc 'dong [giá trị]': ").strip().lower().split()
            if not line or line[0] == 'q': break
            
            cmd = line[0]
            if cmd == 'mo':
                node.control_gripper('mo')
            elif cmd == 'dong':
                val = line[1] if len(line) > 1 else None
                node.control_gripper('dong', val)
            elif cmd == 'h':
                print("-> Đang về Home...")
                node.move_pose(node.SAFE_WAYPOINT[0], node.SAFE_WAYPOINT[1], node.SAFE_WAYPOINT[2], node.GRIP_Q)
                node.move_joint(node.HOME_JOINTS)
            elif len(line) == 3:
                x, y, z = map(float, line)
                node.move_pose(node.SAFE_WAYPOINT[0], node.SAFE_WAYPOINT[1], node.SAFE_WAYPOINT[2], node.GRIP_Q)
                node.move_pose(x, y, z, node.GRIP_Q)
                print("--- DA DEN DICH ---")
        except Exception as e:
            print(f"Lỗi: {e}")

def main():
    rclpy.init()
    node = RobotWallFinal()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    run_loop(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()