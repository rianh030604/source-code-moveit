#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import math

# Import MoveIt
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, BoundingVolume
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive

class PolarController(Node):
    def __init__(self):
        super().__init__('test_polar_fixed_z')
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        print("Dang ket noi MoveIt...")
        self._action_client.wait_for_server()
        print("-> SAN SANG!")

    def move_robot(self, distance, angle_deg):
        """
        Input: 
          - distance: Khoảng cách từ chân tường ra vật (mét)
          - angle_deg: Góc lệch (0=Thẳng, +=Trái, -=Phải)
        Output: Robot chạy đến X, Y tương ứng, Z cố định 0.1
        """
        
        # 1. CỐ ĐỊNH ĐỘ CAO
        FIXED_Z = 0.3 # 10cm
        
        # 2. TÍNH TOÁN X, Y (Hệ tọa độ cực)
        # X = d * cos(anlge)
        # Y = d * sin(angle)
        angle_rad = math.radians(angle_deg)
        target_x = distance * math.cos(angle_rad)
        target_y = distance * math.sin(angle_rad)

        print(f"-> Lenh: Cach {distance}m, Goc {angle_deg} do")
        print(f"-> Toa do World: X={target_x:.3f}, Y={target_y:.3f}, Z={FIXED_Z}")

        # 3. GỬI LỆNH CHO MOVEIT
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "arm"
        goal_msg.request.workspace_parameters.header.frame_id = "world"
        goal_msg.request.workspace_parameters.min_corner.z = -1.0
        goal_msg.request.workspace_parameters.max_corner.z = 1.0
        
        # Tốc độ test (50%)
        goal_msg.request.max_velocity_scaling_factor = 0.5
        goal_msg.request.max_acceleration_scaling_factor = 0.5

        # Ràng buộc vị trí
        constraints = Constraints()
        pcm = PositionConstraint()
        pcm.header.frame_id = "world"
        pcm.link_name = "end_effector_link"
        pcm.weight = 1.0
        
        # Hộp đích đến
        cbox = BoundingVolume()
        cbox.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.01, 0.01, 0.01])]
        cbox.primitive_poses = [PoseStamped().pose]
        cbox.primitive_poses[0].position.x = float(target_x)
        cbox.primitive_poses[0].position.y = float(target_y)
        cbox.primitive_poses[0].position.z = float(FIXED_Z)
        pcm.constraint_region = cbox
        
        constraints.position_constraints.append(pcm)
        goal_msg.request.goal_constraints.append(constraints)

        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            print("-> LOI: MoveIt tu choi!")
            return False

        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        result = get_result_future.result().result
        
        if result.error_code.val == 1:
            print("-> THANH CONG!\n")
        else:
            print(f"-> THAT BAI! Error Code: {result.error_code.val}\n")

def main(args=None):
    rclpy.init(args=args)
    bot = PolarController()

    print("\n-------------------------------------------------")
    print("NHAP: KHOANG_CACH (m) va GOC (do)")
    print("  * Goc 0   = Thang truoc mat")
    print("  * Goc > 0 = Lech TRAI")
    print("  * Goc < 0 = Lech PHAI")
    print("-------------------------------------------------")

    try:
        while True:
            user_input = input("Nhap (dist angle) >> ") # Ví dụ: 0.3 30
            if user_input.lower() in ['q', 'exit']:
                break
            
            try:
                parts = user_input.split()
                if len(parts) != 2:
                    print("Nhap sai! Hay nhap 2 so. Vi du: 0.35 0")
                    continue
                
                d = float(parts[0])
                deg = float(parts[1])
                
                # Cảnh báo nếu khoảng cách quá ngắn (sát tường)
                if d < 0.15:
                    print("(!) Canh bao: Qua gan tuong, co the khong voi toi!")

                bot.move_robot(d, deg)
                
            except ValueError:
                print("Loi: Chi nhap so!")
                
    except KeyboardInterrupt:
        print("\nStop.")
    
    bot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()