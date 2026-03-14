#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import math

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, OrientationConstraint, PositionConstraint, JointConstraint, BoundingVolume
from geometry_msgs.msg import PoseStamped, Pose
from shape_msgs.msg import SolidPrimitive

def euler_to_quaternion(roll, pitch, yaw):
    """Chuyển đổi Độ (Degree) sang Quaternion (x, y, z, w)"""
    r = math.radians(roll)
    p = math.radians(pitch)
    y = math.radians(yaw)
    
    cy = math.cos(y * 0.5); sy = math.sin(y * 0.5)
    cp = math.cos(p * 0.5); sp = math.sin(p * 0.5)
    cr = math.cos(r * 0.5); sr = math.sin(r * 0.5)

    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return [qx, qy, qz, qw]

class RobotCommander(Node):
    def __init__(self):
        super().__init__('robot_input_xyz')
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        print("dang ket noi MoveIt...")
        self._action_client.wait_for_server()
        print("-> KET NOI THANH CONG!")

    def di_chuyen_pose(self, x, y, z, qx, qy, qz, qw):
        """Hàm lõi: Điều khiển robot bằng 7 thông số Pose"""
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "arm"
        
        # Tối ưu hóa thời gian tính toán
        goal_msg.request.allowed_planning_time = 10.0
        goal_msg.request.num_planning_attempts = 20
        goal_msg.request.max_velocity_scaling_factor = 0.5

        constraints = Constraints()
        # 1. Position Constraint (Dung sai 1cm)
        pcm = PositionConstraint()
        pcm.header.frame_id = "world"
        pcm.link_name = "end_effector_link"
        cbox = BoundingVolume()
        cbox.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.01, 0.01, 0.01])]
        target_pose = Pose()
        target_pose.position.x = float(x); target_pose.position.y = float(y); target_pose.position.z = float(z)
        cbox.primitive_poses = [target_pose]
        pcm.constraint_region = cbox
        pcm.weight = 1.0

        # 2. Orientation Constraint
        ocm = OrientationConstraint()
        ocm.header.frame_id = "world"
        ocm.link_name = "end_effector_link"
        ocm.orientation.x = float(qx); ocm.orientation.y = float(qy); ocm.orientation.z = float(qz); ocm.orientation.w = float(qw)
        ocm.absolute_x_axis_tolerance = 0.1
        ocm.absolute_y_axis_tolerance = 0.1
        ocm.absolute_z_axis_tolerance = 0.1
        ocm.weight = 1.0

        constraints.position_constraints.append(pcm)
        constraints.orientation_constraints.append(ocm)
        goal_msg.request.goal_constraints.append(constraints)
        
        self.get_logger().info(f"-> Di chuyen: XYZ({x},{y},{z})")
        self.gui_lenh(goal_msg)

    def dieu_khien_kep(self, val):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "gripper"
        MAX = 0.019; MIN = -0.01
        try:
            s = str(val).lower()
            target = MAX if s in ['mo', 'open'] else (MIN + 0.001 if s in ['dong', 'close'] else MIN + (float(val)/100.0)*(MAX-MIN))
        except: return
        constraints = Constraints()
        for j in ["gripper_left_joint", "gripper_right_joint"]:
            c = JointConstraint()
            c.joint_name = j; c.position = target; c.weight = 1.0
            constraints.joint_constraints.append(c)
        goal_msg.request.goal_constraints.append(constraints)
        self.gui_lenh(goal_msg)

    def ve_nha(self, che_do="home"):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "arm"
        vals = [0.0, 0.0, 0.0, 0.0] if che_do == "init" else [0.0, -1.05, 0.35, 0.7]
        constraints = Constraints()
        for i, v in enumerate(vals):
            c = JointConstraint()
            c.joint_name = f"joint{i+1}"; c.position = v; c.weight = 1.0
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
        if r.result().result.error_code.val == 1: print("--- THANH CONG ---")
        else: print(f"XXX THAT BAI (Loi: {r.result().result.error_code.val})")

def main():
    rclpy.init()
    bot = RobotCommander()
    print("\n==========================================")
    print("HE THONG DIEU KHIEN CHUYEN NGHIEP (RPY)")
    print(" 1. Nhap 6 so:  X Y Z Roll Pitch Yaw")
    print("    (Vi du: 0.3 0.0 0.15 0 90 0)")
    print(" 2. Nhap 3 so:  X Y Z (Dung huong mac dinh)")
    print(" 3. Tien ich:   home, init, mo, dong")
    print("==========================================\n")

    # Huong gap mac dinh (Roll=0, Pitch=0, Yaw=0 - ban co the sua theo GUI)
    Q_DEFAULT = [0.0, 0.0, 0.0, 1.0] 

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
            
            elif len(parts) == 3: # X Y Z
                bot.di_chuyen_pose(float(parts[0]), float(parts[1]), float(parts[2]), *Q_DEFAULT)
            
            elif len(parts) == 6: # X Y Z R P Y
                x, y, z, r, p, yaw = map(float, parts)
                q = euler_to_quaternion(r, p, yaw)
                bot.di_chuyen_pose(x, y, z, *q)

        except Exception as e: print(f"Loi: {e}")

    bot.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()