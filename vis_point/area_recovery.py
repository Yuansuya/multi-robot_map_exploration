import rclpy
from nav_msgs.msg import OccupancyGrid

def map_callback(msg):
    # 取得佔據網格地圖
    grid = msg.data
    width = msg.info.width
    height = msg.info.height

    # 計算佔據的單元格數量
    count = sum([1 for cell in grid if cell != -1 and cell <= 50])

    # 輸出結果
    # print("free Cells:", count)
    cave_like_total = 38617
    office_like_total = 28209
    print("area coverage rate : {}".format(count / cave_like_total* 100))


def main():
    rclpy.init()

    node = rclpy.create_node("map_subscriber")

    # 訂閱 /map 主題
    subscriber = node.create_subscription(OccupancyGrid, "/map", map_callback, 10)

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
