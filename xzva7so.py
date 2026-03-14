#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import math
import time

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, OrientationConstraint, PositionConstraint, JointConstraint, BoundingVolume
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive

class RobotOptimization(Node):
    def __init__(self):
        super().__init__('robot_optimized_commander')
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self._action_client.wait_for_server()
        print("-> HE THONG TOI UU DA SAN SANG!")

    def move_robot(self, x, y, z, qx, qy, qz, qw, is_bottle_mode=False):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "arm"
        goal_msg.request.allowed_planning_time = 5.0
        
        constraints = Constraints()

        # 1. Position Constraint
        pcm = PositionConstraint()
        pcm.header.frame_id = "world"
        pcm.link_name = "end_effector_link"
        cbox = BoundingVolume()
        cbox.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.005, 0.005, 0.005])]
        target = Pose()
        target.position.x, target.position.y, target.position.z = x, y, z
        cbox.primitive_poses = [target]
        pcm.constraint_region = cbox
        constraints.position_constraints.append(pcm)

        # 2. Orientation Constraint
        ocm = OrientationConstraint()
        ocm.header.frame_id = "world"
        ocm.link_name = "end_effector_link"
        ocm.orientation.x, ocm.orientation.y, ocm.orientation.z, ocm.orientation.w = qx, qy, qz, qw
        
        if is_bottle_mode:
            # KHI GAP CHAI: Ep robot giu huong song song mat dat cuc ky chat che
            ocm.absolute_x_axis_tolerance = 0.1 # Roll chat (khong nghiêng chai)
            ocm.absolute_y_axis_tolerance = 0.1 # Pitch chat (song song mat dat)
            ocm.absolute_z_axis_tolerance = 0.1 # Yaw chat (khong xoay ngang)
        else:
            # KHI DIEU KHIEN TU DO: Tha long de de tim nghiem IK
            ocm.absolute_x_axis_tolerance = 0.5
            ocm.absolute_y_axis_tolerance = 0.5
            ocm.absolute_z_axis_tolerance = 0.5
            
        constraints.orientation_constraints.append(ocm)
        goal_msg.request.goal_constraints.append(constraints)
        
        return self.send_goal(goal_msg)

    def control_gripper(self, action):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "gripper"
        target_pos = 0.019 if action == 'mo' else -0.01
        
        constraints = Constraints()
        for joint in ["gripper_left_joint", "gripper_right_joint"]:
            jc = JointConstraint()
            jc.joint_name = joint
            jc.position = target_pos
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        
        goal_msg.request.goal_constraints.append(constraints)
        return self.send_goal(goal_msg)

    def send_goal(self, goal_msg):
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if not handle.accepted: return False
        res = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res)
        return res.result().result.error_code.val == 1

def main():
    rclpy.init()
    bot = RobotOptimization()

    print("\n--- CHE DO DIEU KHIEN TOI UU ---")
    print("1. gap x z   -> Chi toi-lui, len-xuong (Mac dinh Y=0, Kep ngang)")
    print("2. 7so x y z qx qy qz qw -> Toan quyen kiem soat")
    print("3. mo/dong   -> Kẹp/Nhả")

    while True:
        try:
            inp = input(">> ").strip().lower().split()
            if not inp or inp[0] == 'q': break
            
            cmd = inp[0]

            if cmd == 'mo': bot.control_gripper('mo')
            elif cmd == 'dong': bot.control_gripper('dong')
            
            elif cmd == 'gap' and len(inp) == 3:
                # TOI UU THEO Y BAN: Chi nhap X va Z
                x, z = float(inp[1]), float(inp[2])
                print(f"-> Gap chai tai: X={x}, Z={z} (Khóa Y=0, Giữ kẹp ngang)")
                # Su dung Q=[0,0,0,1] de kep song song mat dat
                bot.move_robot(x, 0.0, z, 0.0, 0.0, 0.0, 1.0, is_bottle_mode=True)

            elif cmd == '7so' and len(inp) == 8:
                val = [float(i) for i in inp[1:]]
                bot.move_robot(*val, is_bottle_mode=False)

        except Exception as e: print(f"Loi: {e}")

    rclpy.shutdown()

if __name__ == '__main__':
    main()