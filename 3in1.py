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

class RobotMaster(Node):
    def __init__(self):
        super().__init__('robot_optimized_commander')
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self._action_client.wait_for_server()
        
        # 1. ĐỊNH NGHĨA CÁC TƯ THẾ CỐ ĐỊNH (TEMPLATES) - AN TOÀN TUYỆT ĐỐI
        self.TEMPLATES = {
            "gap": [0.0, 0.0, 0.0, 1.0],         # Kẹp ngang song song mặt đất
            "ban": [-0.001, 0.030, 0.0, 1.0],    # Kẹp chúi 90 độ xuống bàn
            "soi": [0.0, 0.38, 0.0, 0.92],       # Kẹp nghiêng 45 độ
            "trai": [0.0, 0.0, 0.707, 0.707],    # Xoay đế 90 độ trái
            "phai": [0.0, 0.0, -0.707, 0.707]    # Xoay đế 90 độ phải
        }
        print("-> HE THONG TOI UU DA SAN SANG!")

    def move_robot(self, x, y, z, q_list, strict=False):
        """Hàm di chuyển lõi với tùy chọn siết chặt hướng (strict)"""
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "arm"
        goal_msg.request.allowed_planning_time = 5.0
        
        # Cấu hình sai số (Tolerance)
        # Nếu strict=True (dùng cho gắp chai), robot sẽ cố giữ kẹp cực ngang
        tol = 0.1 if strict else 0.5 

        constraints = Constraints()

        # Ràng buộc vị trí (XYZ)
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

        # Ràng buộc hướng (Quaternion)
        ocm = OrientationConstraint()
        ocm.header.frame_id = "world"
        ocm.link_name = "end_effector_link"
        ocm.orientation.x, ocm.orientation.y, ocm.orientation.z, ocm.orientation.w = q_list
        ocm.absolute_x_axis_tolerance = tol
        ocm.absolute_y_axis_tolerance = tol
        ocm.absolute_z_axis_tolerance = tol
        constraints.orientation_constraints.append(ocm)

        goal_msg.request.goal_constraints.append(constraints)
        return self.send_goal(goal_msg)

    def control_gripper(self, action):
        """Điều khiển kẹp Dynamixel"""
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
        print(f"-> Dang {action.upper()} kep...")
        return self.send_goal(goal_msg)

    def send_goal(self, goal_msg):
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if not handle.accepted: return False
        res_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)

def main():
    rclpy.init()
    bot = RobotMaster()

    print("\n" + "="*50)
    print("       DIEU KHIEN ROBOT TOI UU (Hybrid Mode)")
    print(" 1. gap x z       : Gắp chai (Khóa Y=0, kẹp ngang)")
    print(" 2. ban x y z     : Gắp dọc trên bàn")
    print(" 3. [tu_the] x y z: Sử dụng template (soi, trai, phai)")
    print(" 4. 7so x y z q...: Kiểm soát toàn phần")
    print(" 5. mo / dong     : Đóng mở kẹp nhanh")
    print("="*50 + "\n")
    
    while True:
        try:
            line = input(">> ").strip().lower().split()
            if not line or line[0] == 'q': break
            
            cmd = line[0]

            # Xử lý lệnh đóng/mở kẹp
            if cmd in ['mo', 'dong']:
                bot.control_gripper(cmd)
                continue

            # Xử lý các lệnh tọa độ
            val = [float(i) for i in line[1:]]

            if cmd == 'gap' and len(val) == 2:
                # Tối ưu: Chỉ cần nhập X và Z, tự động mở kẹp và khóa Y=0
                bot.control_gripper('mo')
                time.sleep(0.5)
                bot.move_robot(val[0], 0.0, val[1], bot.TEMPLATES["gap"], strict=True)
                time.sleep(0.5)
                bot.control_gripper('dong')

            elif cmd in bot.TEMPLATES and len(val) == 3:
                # Sử dụng các tư thế an toàn định sẵn
                bot.move_robot(val[0], val[1], val[2], bot.TEMPLATES[cmd])

            elif cmd == '7so' and len(val) == 7:
                # Toàn quyền kiểm soát thủ công
                bot.move_robot(val[0], val[1], val[2], val[3:], strict=False)

            else:
                print("Sai cu phap! Vi du: gap 0.25 0.2")

        except Exception as e:
            print(f"Loi: {e}")

    rclpy.shutdown()

if __name__ == '__main__':
    main()