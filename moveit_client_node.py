#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint

class MoveItSequencer(Node):
    def __init__(self):
        super().__init__('moveit_sequencer_node')
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self._action_client.wait_for_server()
        
        #  Tên khớp tay
        self.joint_names = ["joint1", "joint2", "joint3", "joint4"] 
        
        #  Tên khớp kẹp 
        
        self.gripper_joint_name = "gripper_left_joint"

    def send_goal_and_wait(self, goal_msg):
       
        goal_msg.planning_options.plan_only = False 
        
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal bị từ chối!")
            return False

        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        
        result = get_result_future.result().result
        
        if result.error_code.val == 1:
            return True
        else:
            self.get_logger().error(f"Thất bại mã lỗi: {result.error_code.val}")
            return False

    def move_to_joint_target(self, target_joints_rad):
        """Điều khiển CÁNH TAY (ARM)"""
        if len(target_joints_rad) != len(self.joint_names):
            self.get_logger().error(f"Lỗi: Cần {len(self.joint_names)} góc, nhập {len(target_joints_rad)}")
            return False

        goal_msg = MoveGroup.Goal()
        # Setup Workspace
        goal_msg.request.workspace_parameters.header.frame_id = "world"
        goal_msg.request.workspace_parameters.min_corner.x = -1.0
        goal_msg.request.workspace_parameters.min_corner.y = -1.0
        goal_msg.request.workspace_parameters.min_corner.z = -1.0
        goal_msg.request.workspace_parameters.max_corner.x = 1.0
        goal_msg.request.workspace_parameters.max_corner.y = 1.0
        goal_msg.request.workspace_parameters.max_corner.z = 1.0
        
        # group là "arm"
        goal_msg.request.group_name = "arm"
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.5 
        goal_msg.request.num_planning_attempts = 10
        
        constraints = Constraints()
        for i, angle_rad in enumerate(target_joints_rad):
            jc = JointConstraint()
            jc.joint_name = self.joint_names[i]
            jc.position = float(angle_rad) 
            jc.tolerance_above = 0.01 
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        goal_msg.request.goal_constraints.append(constraints)
        
        self.get_logger().info(f"Move Arm: {target_joints_rad}")
        # Gọi hàm phụ trợ đã tách ra ở trên
        return self.send_goal_and_wait(goal_msg)

    def move_gripper(self, open_value):
        """
        [MỚI] Điều khiển KẸP (GRIPPER)
        open_value (mét): 0.019 (Mở max), -0.01 (Đóng chặt)
        """
        goal_msg = MoveGroup.Goal()
        
        # [ group name "gripper"
        goal_msg.request.group_name = "gripper"
        goal_msg.request.allowed_planning_time = 5.0
        
        constraints = Constraints()
        jc = JointConstraint()
        
        # Chỉ định khớp kẹp
        jc.joint_name = self.gripper_joint_name
        jc.position = float(open_value)
        jc.tolerance_above = 0.01
        jc.tolerance_below = 0.01
        jc.weight = 1.0
        
        constraints.joint_constraints.append(jc)
        goal_msg.request.goal_constraints.append(constraints)

        self.get_logger().info(f"Move Gripper: {open_value}")
        return self.send_goal_and_wait(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    bot = MoveItSequencer()

    try:
        
        print("1. Về home")
        bot.move_to_joint_target([0.0, -1.0, 0.7, 0.3])
        time.sleep(1.0)

        print("2. Mở kẹp ra (0.019)")
        bot.move_gripper(0.019)
        time.sleep(1.0)

        print("3. Di chuyển đến vị trí gắp init")
        bot.move_to_joint_target([0.0, 0.0, 0.0, 0.0])
        time.sleep(1.0)

        print("4. Đóng kẹp lại (-0.005)")
        bot.move_gripper(-0.005) 
        time.sleep(1.0)

        print("5. Nhấc lên (Về home)")
        bot.move_to_joint_target([0.0, -1.0, 0.7, 0.3])
        time.sleep(1.0)
            
    except KeyboardInterrupt:
        print("Done")
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()