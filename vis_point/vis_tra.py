import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA
import math

class TrajectoryColorizer(Node):
    def __init__(self):
        super().__init__('trajectory_colorizer')
        self.subscription = self.create_subscription(
            MarkerArray,
            '/trajectory_node_list',
            self.marker_array_callback,
            10
        )


        self.publisher = self.create_publisher(MarkerArray, '/colored_trajectory_node_list', 10)

    def marker_array_callback(self, msg):
        colored_msg = self.colorize_marker_array(msg)
        self.publisher.publish(colored_msg)

        total_length = 0.0
        for marker in colored_msg.markers:
            if marker.type == Marker.LINE_STRIP:
                for i in range(1, len(marker.points)):
                    p1 = marker.points[i - 1]
                    p2 = marker.points[i]
                    segment_length = self.calculate_distance(p1, p2)
                    total_length += segment_length
        print(total_length)
    def calculate_distance(self, p1, p2):
        dx = p2.x - p1.x
        dy = p2.y - p1.y
        dz = p2.z - p1.z
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)
        return distance

    def colorize_marker_array(self, marker_array):
        # Iterate through the markers in the MarkerArray
        for marker in marker_array.markers:
            # Set the desired color for the marker
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # Red color

        return marker_array
    


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryColorizer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

