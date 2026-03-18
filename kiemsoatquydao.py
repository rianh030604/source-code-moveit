#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import Constraints, JointConstraint, RobotState
from geometry_msgs.msg import PoseStamped
import time

class RobotWallSimple(Node):
    def __init__(self):
        super().__init__('robot_wall_simple')
        
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self.ik_client = self.create_client(GetPositionIK, 'compute_ik')
        
        # --- CẬP NHẬT WAYPOINTS CHUẨN (ĐÃ FIX LỖI TOÁN HỌC) ---
        # Vùng dưới: X=0.35 (rút ngắn lại cho đỡ rung), hướng ngang
        self.WP_DUOI = [0.328, 0.0, 0.3, 0.032, 0.027, 0.001, 0.999]
        
        # Vùng trên: X=0.35, Hướng chúi xuống 45 độ (Q chuẩn cho MoveIt)
        # Bộ Quaternion này [0.0, 0.383, 0.0, 0.924] đại diện cho việc chúi kẹp xuống
        self.WP_TREN = [0.328, 0.0, 0.3, 0.032, 0.027, 0.001, 0.999]
        
        self.FLOOR_LIMIT = -0.3 # Cách sàn (đế cao 31cm)
        self.DEAD_ZONE_X = -0.1 # Cách tường
        self.JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4"]

        print("Đang kết nối MoveIt...")
        self.ik_client.wait_for_service()
        self._action_client.wait_for_server()
        print("-> HỆ THỐNG GẮN TƯỜNG SẴN SÀNG. NHẬP X Y Z.")

    def tinh_toan_ik(self, x, y, z, q_list):
        if x < self.DEAD_ZONE_X or z < self.FLOOR_LIMIT:
            print(f"!!! CHẶN: Tọa độ ({x}, {z}) vi phạm vùng an toàn (tường/sàn).")
            return None

        req = GetPositionIK.Request()
        req.ik_request.group_name = "arm"
        req.ik_request.avoid_collisions = True
        
        # Seed state ép J1 không xoay ngược
        seed = RobotState()
        seed.joint_state.name = self.JOINT_NAMES
        seed.joint_state.position = [0.0, 0.0, 0.0, 0.0]
        req.ik_request.robot_state = seed

        target = PoseStamped()
        target.header.frame_id = "world"
        target.pose.position.x, target.pose.position.y, target.pose.position.z = x, y, z
        target.pose.orientation.x, target.pose.orientation.y = q_list[0], q_list[1]
        target.pose.orientation.z, target.pose.orientation.w = q_list[2], q_list[3]
        
        req.ik_request.pose_stamped = target
        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        
        if res.error_code.val == 1:
            return res.solution.joint_state.position[:4]
        return None

    def di_chuyen_khop(self, joints):
        if joints is None: return False
        goal = MoveGroup.Goal()
        goal.request.group_name = "arm"
        goal.request.max_velocity_scaling_factor = 0.4
        
        c = Constraints()
        for name, pos in zip(self.JOINT_NAMES, joints):
            jc = JointConstraint(joint_name=name, position=float(pos), weight=1.0)
            jc.tolerance_above = jc.tolerance_below = 0.01
            c.joint_constraints.append(jc)
        
        goal.request.goal_constraints.append(c)
        f = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, f)
        res_f = f.result().get_result_async()
        rclpy.spin_until_future_complete(self, res_f)
        return True

    def thuc_thi(self, x, y, z):
        # Chọn Waypoint dựa trên Z của điểm đến
        if z > 0.31:
            print(f"-> Vùng TRÊN. Điểm trung gian: {self.WP_TREN[:3]}")
            wp_data = self.WP_TREN
        else:
            print(f"-> Vùng DƯỚI. Điểm trung gian: {self.WP_DUOI[:3]}")
            wp_data = self.WP_DUOI

        # Bước 1: Tới Waypoint
        joints_wp = self.tinh_toan_ik(wp_data[0], wp_data[1], wp_data[2], wp_data[3:])
        
        # Bước 2: Tới Đích (Mượn hướng của Waypoint)
        joints_target = self.tinh_toan_ik(x, y, z, wp_data[3:])

        if joints_wp and joints_target:
            print(">> Đang chạy Waypoint...")
            self.di_chuyen_khop(joints_wp)
            time.sleep(0.5)
            print(">> Đang vươn tới đích...")
            self.di_chuyen_khop(joints_target)
            print("--- HOÀN THÀNH ---")
        else:
            print("XXX Lỗi: MoveIt không tìm được đường đi cho lộ trình này!")

def main():
    rclpy.init()
    bot = RobotWallSimple()
    while True:
        try:
            inp = input("\nNhập XYZ: ").split()
            if not inp or inp[0] == 'q': break
            if len(inp) == 3:
                bot.thuc_thi(float(inp[0]), float(inp[1]), float(inp[2]))
        except Exception as e:
            print(f"Lỗi: {e}")
    rclpy.shutdown()

if __name__ == '__main__':
    main()