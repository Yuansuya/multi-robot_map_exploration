import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA, Bool
import math

class TMShow(Node):
    def __init__(self):
        super().__init__('tm_show')
        
        self.publisher = self.create_publisher(Bool, '/get_TM', 10)
        self.timer_ = self.create_timer(5, self.publish_tm)
    def publish_tm(self):
        print("get_tm")
        msg = Bool()
        msg.data = True
        self.publisher.publish(msg)
   
   
    


def main(args=None):
    rclpy.init(args=args)
    node = TMShow()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

