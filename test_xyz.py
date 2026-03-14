#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import sys

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, BoundingVolume
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive

class SimpleMover(Node):
    def __init__(self):
        super().__init__('test_xyz_manual')
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        print("dang ket noi voi robot...")
        self._action_client.wait_for_server()
        print("-> DA KET NOI! San sang nhan lenh.")

    def move_to_xyz(self, x, y, z):
        """Hàm gửi lệnh di chuyển đến tọa độ X, Y, Z trong hệ World"""
        
        # Goal
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "arm"
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        
        # Toc do 
        goal_msg.request.max_velocity_scaling_factor = 0.4
        goal_msg.request.max_acceleration_scaling_factor = 0.4

        # 2. "world"
        goal_msg.request.workspace_parameters.header.frame_id = "world"
        goal_msg.request.workspace_parameters.min_corner.z = -1.0 # Cho phép đi xuống thấp
        goal_msg.request.workspace_parameters.max_corner.z = 1.0

        constraints = Constraints()
        
        pcm = PositionConstraint()
        pcm.header.frame_id = "world"
        pcm.link_name = "end_effector_link" # gripper
        pcm.weight = 1.0
        
        # tao mot khoi hop 
        cbox = BoundingVolume()
        cbox.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.005, 0.005, 0.005])]
        cbox.primitive_poses = [PoseStamped().pose]
        cbox.primitive_poses[0].position.x = float(x)
        cbox.primitive_poses[0].position.y = float(y)
        cbox.primitive_poses[0].position.z = float(z)
        pcm.constraint_region = cbox
        
        constraints.position_constraints.append(pcm)
        
        
        goal_msg.request.goal_constraints.append(constraints)

        # 4. Gửi lệnh
        print(f"Dang di chuyen den: X={x}, Y={y}, Z={z} ...")
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            print("-> LOI: MoveIt tu choi lap ke hoach!")
            return False

        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        result = get_result_future.result().result
        
        if result.error_code.val == 1:
            print("-> THANH CONG!")
            return True
        else:
            print(f"-> THAT BAI! Ma loi: {result.error_code.val}")
            return False

def main(args=None):
    rclpy.init(args=args)
    mover = SimpleMover()

    print("\n-------------------------------------------------")
    print("NHAP TOA DO (X Y Z) DE DIEU KHIEN ROBOT")
    print("Vi du: 0.3 0.0 0.1")
    print("Nhap 'q' hoac 'exit' de thoat.")
    print("-------------------------------------------------\n")

    try:
        while True:
            user_input = input("Nhap X Y Z >> ")
            
            if user_input.lower() in ['q', 'exit', 'quit']:
                break
            
            try:
                # Tách chuỗi nhập vào thành 3 số
                parts = user_input.split()
                if len(parts) != 3:
                    print("Vui long nhap du 3 so! (Vi du: 0.25 0 0.1)")
                    continue
                
                x = float(parts[0])
                y = float(parts[1])
                z = float(parts[2])
                
                mover.move_to_xyz(x, y, z)
                
            except ValueError:
                print("Loi: Vui long chi nhap so!")
                
    except KeyboardInterrupt:
        print("\nStop.")

    mover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()