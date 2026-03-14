#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import math
import time

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, OrientationConstraint, PositionConstraint, JointConstraint, BoundingVolume
from geometry_msgs.msg import PoseStamped, Pose
from shape_msgs.msg import SolidPrimitive

def euler_to_quaternion(roll, pitch, yaw):
    """Chuyển đổi Độ sang Quaternion"""
    r, p, y = map(math.radians, [roll, pitch, yaw])
    cy, sy = math.cos(y*0.5), math.sin(y*0.5)
    cp, sp = math.cos(p*0.5), math.sin(p*0.5)
    cr, sr = math.cos(r*0.5), math.sin(r*0.5)
    return [
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy,
        cr*cp*cy + sr*sp*cy
    ]

class RobotCommander(Node):
    def __init__(self):
        super().__init__('robot_bottle_grabber')
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        print("Đang kết nối MoveIt...")
        self._action_client.wait_for_server()
        print("-> KẾT NỐI THÀNH CÔNG!")

    def di_chuyen_pose(self, x, y, z, r, p, yaw):
        """Hàm di chuyển bằng tọa độ và góc xoay RPY"""
        q = euler_to_quaternion(r, p, yaw)
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "arm"
        goal_msg.request.allowed_planning_time = 10.0
        goal_msg.request.max_velocity_scaling_factor = 0.5

        # Ràng buộc đích đến
        constraints = Constraints()
        
        # Vị trí (Dung sai 1cm để linh hoạt)
        pcm = PositionConstraint()
        pcm.header.frame_id = "world"
        pcm.link_name = "end_effector_link"
        cbox = BoundingVolume()
        cbox.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.01, 0.01, 0.01])]
        target = Pose()
        target.position.x, target.position.y, target.position.z = x, y, z
        cbox.primitive_poses = [target]
        pcm.constraint_region = cbox
        
        # Hướng (Dùng sai số 0.3 rad ~ 17 độ để lách qua các điểm khó)
        ocm = OrientationConstraint()
        ocm.header.frame_id = "world"
        ocm.link_name = "end_effector_link"
        ocm.orientation.x, ocm.orientation.y, ocm.orientation.z, ocm.orientation.w = q
        ocm.absolute_x_axis_tolerance = 0.3
        ocm.absolute_y_axis_tolerance = 0.3
        ocm.absolute_z_axis_tolerance = 0.3
        
        constraints.position_constraints.append(pcm)
        constraints.orientation_constraints.append(ocm)
        goal_msg.request.goal_constraints.append(constraints)
        
        print(f"-> Di chuyển tới: {x}, {y}, {z} | Góc: {r},{p},{yaw}")
        return self.gui_lenh(goal_msg)

    def dieu_khien_kep(self, lenh):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "gripper"
        target = 0.019 if lenh == 'mo' else -0.009
        constraints = Constraints()
        for j in ["gripper_left_joint", "gripper_right_joint"]:
            c = JointConstraint()
            c.joint_name = j; c.position = target; c.weight = 1.0
            constraints.joint_constraints.append(c)
        goal_msg.request.goal_constraints.append(constraints)
        self.gui_lenh(goal_msg)

    def gui_lenh(self, goal_msg):
        f = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, f)
        h = f.result()
        if not h.accepted: return False
        r = h.get_result_async()
        rclpy.spin_until_future_complete(self, r)
        return r.result().result.error_code.val == 1

def main():
    rclpy.init()
    bot = RobotCommander()

    print("\n--- CHẾ ĐỘ GẮP CHAI NƯỚC TỰ ĐỘNG ---")
    print("Nhập tọa độ chai nước (X Y Z):")

    while True:
        try:
            inp = input(">> ").strip().lower()
            if inp in ['q', 'exit']: break
            parts = inp.split()
            
            if len(parts) == 3:
                x, y, z = map(float, parts)
                
                # BƯỚC 1: Mở kẹp trước
                bot.dieu_khien_kep('mo')
                
                # BƯỚC 2: Tiếp cận chai (cách 5cm) - Pitch = 0 (gắp ngang)
                print("1. Đang tiếp cận...")
                bot.di_chuyen_pose(x - 0.05, y, z, 0, 0, 0)
                
                # BƯỚC 3: Tiến vào ôm thân chai
                print("2. Tiến vào gắp...")
                bot.di_chuyen_pose(x, y, z, 0, 0, 0)
                
                # BƯỚC 4: Đóng kẹp
                bot.dieu_khien_kep('dong')
                time.sleep(1) # Chờ kẹp chặt
                
                # BƯỚC 5: Nhấc chai lên cao
                print("3. Đang nhấc chai lên...")
                bot.di_chuyen_pose(x, y, z + 0.1, 0, 0, 0)
                
                print("--- HOÀN THÀNH CHU TRÌNH ---")

        except Exception as e: print(f"Lỗi: {e}")

    bot.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()