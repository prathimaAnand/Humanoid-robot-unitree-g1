import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit2 import MoveIt2

class FingertipIKDemo(Node):
    def __init__(self):
        super().__init__('fingertip_ik_demo')

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=[
                'shoulder_joint', 'elbow_joint', 'wrist_joint', 'left_finger_joint'
            ],
            base_link_name='g1_base_link',
            end_effector_name='left_index_fingertip_frame',
            group_name='arm_with_fingers'
        )

        self.timer = self.create_timer(2.0, self.send_target_pose)

    def send_target_pose(self):
        pose = PoseStamped()
        pose.header.frame_id = 'g1_base_link'
        pose.pose.position.x = 0.35
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.4
        pose.pose.orientation.w = 1.0

        self.moveit2.move_to_pose(pose)

def main():
    rclpy.init()
    node = FingertipIKDemo()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
