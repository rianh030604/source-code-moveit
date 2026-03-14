#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, JointConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller_node')
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        
        self.get_logger().info('Dang ket noi MoveIt Server...')
        self._action_client.wait_for_server()
        self.get_logger().info('-> KET NOI THANH CONG! San sang.')

    def dieu_khien_kep(self, gia_tri_dau_vao):
        """
        Điều khiển kẹp theo % hoặc lệnh mo/dong
        """
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "gripper"
        
        MAX_OPEN = 0.019  # Mở hết cỡ
        MIN_CLOSE = -0.01 # Đóng chặt
        
        target_pos = 0.0

        try:
            # Xử lý logic đầu vào
            val_str = str(gia_tri_dau_vao).lower()
            
            if val_str in ['mo', 'open']:
                target_pos = MAX_OPEN
                self.get_logger().info('-> Lenh: MO kep (100%)')
            
            elif val_str in ['dong', 'close']:
                target_pos = MIN_CLOSE + 0.001 
                self.get_logger().info('-> Lenh: DONG kep (0%)')
            
            else:
                percent = float(gia_tri_dau_vao)
                percent = max(0.0, min(100.0, percent))
                target_pos = MIN_CLOSE + (percent / 100.0) * (MAX_OPEN - MIN_CLOSE)
                self.get_logger().info(f'-> Lenh: Mo kep {percent}%')

        except ValueError:
            self.get_logger().error('Loi: Gia tri kep khong hop le!')
            return

        # Tạo ràng buộc cho CẢ 2 NGÓN
        constraints = Constraints()

        # Ngón Trái
        jcm_left = JointConstraint()
        jcm_left.joint_name = "gripper_left_joint"
        jcm_left.position = target_pos
        jcm_left.tolerance_above = 0.005
        jcm_left.tolerance_below = 0.005
        jcm_left.weight = 1.0
        constraints.joint_constraints.append(jcm_left)

        # Ngón Phải
        jcm_right = JointConstraint()
        jcm_right.joint_name = "gripper_right_joint"
        jcm_right.position = target_pos
        jcm_right.tolerance_above = 0.005
        jcm_right.tolerance_below = 0.005
        jcm_right.weight = 1.0
        constraints.joint_constraints.append(jcm_right)

        goal_msg.request.goal_constraints.append(constraints)
        self.gui_lenh(goal_msg)

    def di_chuyen_arm(self, x, y, z):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "arm"
        
        goal_msg.request.workspace_parameters.header.frame_id = "world"
        goal_msg.request.workspace_parameters.min_corner.x = -1.0
        goal_msg.request.workspace_parameters.min_corner.y = -1.0
        goal_msg.request.workspace_parameters.min_corner.z = -1.0
        goal_msg.request.workspace_parameters.max_corner.x = 1.0
        goal_msg.request.workspace_parameters.max_corner.y = 1.0
        goal_msg.request.workspace_parameters.max_corner.z = 1.0

        goal_msg.request.max_velocity_scaling_factor = 0.5
        goal_msg.request.max_acceleration_scaling_factor = 0.5
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0

        # Ràng buộc Vị Trí
        pcm = PositionConstraint()
        pcm.header.frame_id = "world"
        pcm.link_name = "end_effector_link"
        pcm.weight = 1.0
        cbox = BoundingVolume()
        cbox.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.005, 0.005, 0.005])]
        cbox.primitive_poses = [PoseStamped().pose]
        cbox.primitive_poses[0].position.x = float(x)
        cbox.primitive_poses[0].position.y = float(y)
        cbox.primitive_poses[0].position.z = float(z)
        pcm.constraint_region = cbox

        # Ràng buộc Hướng (Fix lỗi 4 trục)
        ocm = OrientationConstraint()
        ocm.header.frame_id = "world"
        ocm.link_name = "end_effector_link"
        ocm.weight = 1.0
        ocm.orientation.w = 1.0
        ocm.absolute_x_axis_tolerance = 0.1
        ocm.absolute_y_axis_tolerance = 3.14
        ocm.absolute_z_axis_tolerance = 3.14

        constraints = Constraints()
        constraints.position_constraints.append(pcm)
        constraints.orientation_constraints.append(ocm)
        goal_msg.request.goal_constraints.append(constraints)
        
        self.get_logger().info(f'Dang di chuyen den: {x}, {y}, {z} ...')
        self.gui_lenh(goal_msg)

    def gui_lenh(self, goal_msg):
        # 1. Gửi lệnh đi và chờ nhận "Vé" (Goal Handle)
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('LOI: MoveIt tu choi nhan lenh (Goal Rejected)')
            return

        # 2. Có "Vé" rồi, giờ ngồi chờ Robot chạy xong để lấy kết quả
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        
        # 3. Lấy kết quả cuối cùng
        result = get_result_future.result().result
        
        if result.error_code.val == 1:
            self.get_logger().info('-> THANH CONG!')
        else:
            self.get_logger().error(f'-> THAT BAI! Ma loi: {result.error_code.val}')

def main(args=None):
    rclpy.init(args=args)
    bot = RobotController()

    print("\n-------------------------------------------------")
    print("CU PHAP: X Y Z [Do_Mo_%]")
    print("VI DU:")
    print(" >> 0.25 0.0 0.1 100   (Di chuyen + MO HET)")
    print(" >> 0.25 0.0 0.1 0     (Di chuyen + DONG CHAT)")
    print(" >> 0.25 0.0 0.1 50    (Di chuyen + MO 50%)")
    print("-------------------------------------------------\n")

    try:
        while True:
            inp = input("LENH >> ")
            if inp in ['q', 'exit']: break
            
            parts = inp.split()
            try:
                x, y, z = map(float, parts[:3])
                bot.di_chuyen_arm(x, y, z)
                
                if len(parts) > 3:
                    bot.dieu_khien_kep(parts[3])
                        
            except ValueError:
                print("Loi: Nhap sai dinh dang!")
            except Exception as e:
                print(f"Loi: {e}")
                
    except KeyboardInterrupt:
        pass
    
    bot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()