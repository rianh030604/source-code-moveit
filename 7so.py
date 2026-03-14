#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import math

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, OrientationConstraint, PositionConstraint, JointConstraint, BoundingVolume
from geometry_msgs.msg import PoseStamped, Pose
from shape_msgs.msg import SolidPrimitive

class RobotCommander(Node):
    def __init__(self):
        super().__init__('robot_input_xyz')
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        
        print("dang ket noi MoveIt...")
        self._action_client.wait_for_server()
        print("-> KET NOI THANH CONG!")

    def di_chuyen_7_so(self, x, y, z, qx, qy, qz, qw):
        """Di chuyển linh hoạt bằng 7 thông số Pose"""
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "arm"
        
        # Cấu hình thời gian lập trình dài hơn để tăng độ linh hoạt
        goal_msg.request.allowed_planning_time = 10.0
        goal_msg.request.num_planning_attempts = 20
        goal_msg.request.max_velocity_scaling_factor = 0.5
        goal_msg.request.max_acceleration_scaling_factor = 0.5

        # Tạo ràng buộc đích đến (Goal Constraints)
        constraints = Constraints()

        # 1. Ràng buộc Vị Trí (Tăng lên 1cm để linh hoạt hơn)
        pcm = PositionConstraint()
        pcm.header.frame_id = "world"
        pcm.link_name = "end_effector_link"
        cbox = BoundingVolume()
        cbox.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.01, 0.01, 0.01])]
        
        target_pose = Pose()
        target_pose.position.x = float(x)
        target_pose.position.y = float(y)
        target_pose.position.z = float(z)
        cbox.primitive_poses = [target_pose]
        pcm.constraint_region = cbox
        pcm.weight = 1.0

        # 2. Ràng buộc Hướng (Nhập trực tiếp từ 4 số Q)
        ocm = OrientationConstraint()
        ocm.header.frame_id = "world"
        ocm.link_name = "end_effector_link"
        ocm.orientation.x = float(qx)
        ocm.orientation.y = float(qy)
        ocm.orientation.z = float(qz)
        ocm.orientation.w = float(qw)
        # Đặt sai số nhỏ (0.1 rad) để robot giữ tư thế nhưng vẫn đủ linh hoạt để tìm đường
        ocm.absolute_x_axis_tolerance = 0.1
        ocm.absolute_y_axis_tolerance = 0.1
        ocm.absolute_z_axis_tolerance = 0.1
        ocm.weight = 1.0

        constraints.position_constraints.append(pcm)
        constraints.orientation_constraints.append(ocm)
        goal_msg.request.goal_constraints.append(constraints)

        self.get_logger().info(f"-> Di chuyen linh hoat: XYZ({x},{y},{z}) Q({qx},{qy},{qz},{qw})")
        self.gui_lenh(goal_msg)

    # ... (Giữ nguyên các hàm dieu_khien_kep, ve_nha, gui_lenh của bạn) ...
    def dieu_khien_kep(self, val):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "gripper"
        MAX = 0.019; MIN = -0.01
        target = 0.0
        try:
            s = str(val).lower()
            if s in ['mo', 'open']: target = MAX
            elif s in ['dong', 'close']: target = MIN + 0.001
            else:
                p = float(val); p = max(0.0, min(100.0, p))
                target = MIN + (p/100.0)*(MAX-MIN)
            print(f"-> Lenh Kep: {s} ({target:.4f})")
        except: return
        constraints = Constraints()
        for j in ["gripper_left_joint", "gripper_right_joint"]:
            c = JointConstraint()
            c.joint_name = j; c.position = target
            c.tolerance_above = 0.005; c.tolerance_below = 0.005; c.weight = 1.0
            constraints.joint_constraints.append(c)
        goal_msg.request.goal_constraints.append(constraints)
        self.gui_lenh(goal_msg)

    def ve_nha(self, che_do="home"):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "arm"
        goal_msg.request.max_velocity_scaling_factor = 0.5
        vals = [0.0, 0.0, 0.0, 0.0] if che_do == "init" else [0.0, -1.05, 0.35, 0.7]
        print(f"-> Ve {che_do.upper()}...")
        constraints = Constraints()
        for i, v in enumerate(vals):
            c = JointConstraint()
            c.joint_name = f"joint{i+1}"; c.position = v
            c.tolerance_above = 0.01; c.tolerance_below = 0.01; c.weight = 1.0
            constraints.joint_constraints.append(c)
        goal_msg.request.goal_constraints.append(constraints)
        self.gui_lenh(goal_msg)

    def gui_lenh(self, goal_msg):
        f = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, f)
        h = f.result()
        if not h.accepted: print("XXX BI TU CHOI XXX"); return
        r = h.get_result_async()
        rclpy.spin_until_future_complete(self, r)
        res = r.result().result
        if res.error_code.val == 1: print("--- THANH CONG ---")
        else: print(f"XXX THAT BAI (Ma loi: {res.error_code.val}) XXX")

def main():
    rclpy.init()
    bot = RobotCommander()

    print("\n==========================================")
    print("CHE DO DIEU KHIEN 7 THONG SO LINH HOAT")
    print(" 1. Nhap 3 so: X Y Z (Dung Q gap mac dinh)")
    print(" 2. Nhap 7 so: X Y Z Qx Qy Qz Qw")
    print(" 3. Lenh chu:  home, init, mo, dong")
    print("==========================================\n")

    # Bo so Q gap vat ly tuong ban da tim thay
    Q_GAP = [-0.001, 0.030, 0.000, 1.000]

    while True:
        try:
            inp = input(">> ").strip().lower()
            if not inp or inp in ['q', 'exit']: break
            parts = inp.split()
            
            if len(parts) == 1:
                if parts[0] == 'home': bot.ve_nha('home')
                elif parts[0] == 'init': bot.ve_nha('init')
                elif parts[0] in ['mo', 'open']: bot.dieu_khien_kep('mo')
                elif parts[0] in ['dong', 'close']: bot.dieu_khien_kep('dong')
            
            elif len(parts) == 3:
                x, y, z = map(float, parts)
                bot.di_chuyen_7_so(x, y, z, *Q_GAP)
            
            elif len(parts) == 7:
                x, y, z, qx, qy, qz, qw = map(float, parts)
                bot.di_chuyen_7_so(x, y, z, qx, qy, qz, qw)
                
            elif len(parts) == 4: # X Y Z + Lenh kep
                x, y, z = map(float, parts[:3])
                bot.di_chuyen_7_so(x, y, z, *Q_GAP)
                bot.dieu_khien_kep(parts[3])

        except Exception as e: print(f"Loi: {e}")

    bot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()