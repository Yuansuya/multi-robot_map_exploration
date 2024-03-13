import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseArray
from std_msgs.msg import ColorRGBA 
class PointMarkerNode(Node):
    def __init__(self):
        super().__init__('point_marker_node')

        self.froniter_points = PoseArray()
        self.waypoint = PoseArray()
        self.waypoint_subscription = self.create_subscription(
            PoseArray,
            'nav_waypoint',
            self.waypoint_callback,
            10)
        self.frontiers_subscription = self.create_subscription(
            PoseArray,
            'all_frontier_visualization',
            self.frontiers_callback,
            10)
        
        self.publisher_ = self.create_publisher(Marker, 'point_marker', 10)
        self.timer_ = self.create_timer(0.1, self.publish_marker)
    def frontiers_callback(self, msg):
        self.froniter_points = msg
        print("got froniter_points")
    def waypoint_callback(self, msg):
        self.waypoint = msg
        print("got waypoint")


    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.pose.orientation.w = 1.0
        marker.pose.position.z = 0.1

        points = []
        colors = []
        # labels = []  # 存放编号的列表
        for idx, fPts in enumerate(self.froniter_points.poses):
            point = Point()
            point.x = fPts.position.x
            point.y = fPts.position.y
            point.z = 0.0
            points.append(point)

            color1 = ColorRGBA()
            color1.r = 1.0
            color1.g = 0.0
            color1.b = 0.0
            color1.a = 1.0
            colors.append(color1)  # 紅色點
            
            # labels.append(str(idx))  # 添加编号

        for wPts in self.waypoint.poses:
            point = Point()
            point.x = wPts.position.x
            point.y = wPts.position.y
            point.z = 0.0
            points.append(point)
            
            color2 = ColorRGBA()
            color2.r = 0.0
            color2.g = 1.0
            color2.b = 0.0
            color2.a = 1.0
            colors.append(color2) 
            
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.points = points
        marker.colors = colors


        # for idx, label in enumerate(labels):
        #     marker_text = Marker()
        #     marker_text.header.frame_id = 'map'
        #     marker_text.id = idx
        #     marker_text.type = Marker.TEXT_VIEW_FACING
        #     marker_text.action = Marker.ADD
        #     marker_text.pose.position.x = points[idx].x
        #     marker_text.pose.position.y = points[idx].y
        #     marker_text.pose.position.z = points[idx].z + 0.2
        #     marker_text.pose.orientation.w = 1.0
        #     marker_text.scale.z = 0.1
        #     marker_text.color.r = 0.0
        #     marker_text.color.g = 0.0
        #     marker_text.color.b = 1.0
        #     marker_text.color.a = 1.0
        #     marker_text.text = label

        self.publisher_.publish(marker)
            # self.publisher_.publish(marker_text)  



def main(args=None):
    rclpy.init(args=args)
    node = PointMarkerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
