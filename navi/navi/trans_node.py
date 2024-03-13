import rclpy
from rclpy.node import Node
import tf2_ros
import geometry_msgs.msg 
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Odometry
import transforms3d 
import numpy as np
from transforms3d.quaternions import quat2mat, qmult
import os
class TransNode(Node):
    def __init__(self):
        super().__init__(node_name='odom2mapTransNode')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pose_subscriber = self.create_subscription(
            Odometry,
            "odom",
            self.odom_callback,
            10
        )
        self.pose_pub = self.create_publisher(PoseStamped, '/pose_at_map', 1)


    def odom_callback(self, msg):
        try:
            transform = self.tf_buffer.lookup_transform(
                msg.header.frame_id,
                "map",                
                rclpy.time.Time().to_msg()
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"Error occurred: {e}")
            return
        # print(transform)

        pose_out = PoseStamped()
        pose_out.header.frame_id = "map"
        pose_out.header.stamp = self.get_clock().now().to_msg()

        pose_out.pose = self.transform_pose(msg.pose.pose, transform)
        # print("pose_out", pose_out.pose)
        self.pose_pub.publish(pose_out)

    def transform_pose(self, origin_pose, transformRelation):
        # rot_a = transforms3d.quaternions.quat2mat([origin_pose.orientation.x, origin_pose.orientation.y, origin_pose.orientation.z, origin_pose.orientation.w])
        # rot_b = transforms3d.quaternions.quat2mat([transformRelation.transform.rotation.x, transformRelation.transform.rotation.y, transformRelation.transform.rotation.z, transformRelation.transform.rotation.w])

        # # 計算變換矩陣
        # trans_mat = np.eye(4)
        # trans_mat[:3, :3] = rot_b.dot(rot_a)
        # trans_mat[:3, 3] = np.array([transformRelation.transform.translation.x, transformRelation.transform.translation.y, transformRelation.transform.translation.z])

        # pose_b = Pose()
        # pose_b.position.x, pose_b.position.y, pose_b.position.z = np.dot(trans_mat, [origin_pose.position.x, origin_pose.position.y, origin_pose.position.z, 1.0])[:3]
        # pose_b.orientation.x, pose_b.orientation.y, pose_b.orientation.z, pose_b.orientation.w = transforms3d.quaternions.mat2quat(trans_mat[:3, :3])

        # return pose_b

        translation_diff = np.array([transformRelation.transform.translation.x, transformRelation.transform.translation.y, transformRelation.transform.translation.z])
        rotation_diff = np.array([transformRelation.transform.rotation.x, transformRelation.transform.rotation.y, transformRelation.transform.rotation.z, transformRelation.transform.rotation.w])
        
        translated_point = np.array([origin_pose.position.x, origin_pose.position.y, origin_pose.position.z]) - translation_diff
        
        res = Pose()
        res.position.x = translated_point[0]
        res.position.y = translated_point[1]
        res.position.z = translated_point[2]
        res.orientation= origin_pose.orientation
        return res


def main(args=None):
    rclpy.init(args=args)
    node= TransNode()
    # cwd = os.getcwd()
    # print("Current working directory: ", cwd)

    # # 獲取系統路徑
    # sys_path = os.path.dirname(os.path.realpath(__file__))
    # print("System path: ", sys_path)
    print("trans node running...")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
