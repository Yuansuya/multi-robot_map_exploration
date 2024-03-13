import rclpy
from rclpy.node import Node
import tf2_ros
import geometry_msgs.msg 
from geometry_msgs.msg import PoseStamped, Pose, PoseArray, Point, Quaternion
from nav_msgs.msg import Odometry, OccupancyGrid
from my_interfaces.srv import MoveGoal, FindFrontier
from std_srvs.srv import Empty
import time
from std_msgs.msg import Bool, String, Int8, UInt32
import numpy as np
import math
from action_msgs.msg import GoalStatusArray
class testNode(Node):
    def __init__(self):
        super().__init__(node_name='testgoalpose_node')
        self.declare_parameter('robotID', 0)
        self.robotID = self.get_parameter('robotID').value 

        self.current_pose= PoseStamped()
        self.current_map= OccupancyGrid()
        self.pose_receive= False
        self.map_receive= False
        self.busy_status= False
        self.bc_busy_status= False
        self.init_pose = PoseStamped() 
        self.waypointReceived= False
        self.waypoint= PoseArray()
        self.nav_status= False
        #The PointID where the robot is currently located in the TM
        self.robot_stand_node= -1
        self.cur_robot_located_node_received= False
        # self.req = MoveGoal.Request()
        self.nav_created_new_nodes= PoseArray()
        self.nav_new_nodes_received= False
        self.get_all_frontier_publisher = self.create_publisher(Bool, 'get_all_frontier', 10)
        self.get_TM_publisher = self.create_publisher(Bool, 'get_TM', 10)

        self.waypoint_pub = self.create_publisher(PoseArray, 'nav_waypoint', 10)
        self.nav_pose_pub = self.create_publisher(PoseArray, 'my_nav_poses', 10)
        self.get_tm_pub = self.create_publisher(Bool, 'get_TM', 10)

        self.task_allocation_publisher = self.create_publisher(UInt32, 'ta_service', 10)
        self.TA_subscription = self.create_subscription(
            PoseArray,
            'ta_output_waypoint',
            self.TA_callback,
            10)       

        self.moveRobot_publisher= self.create_publisher(UInt32, 'move_robot', 10)
        self.initRobot_publisher = self.create_publisher(Int8, 'init_robot', 10)
        self.create_node_subscription = self.create_subscription(
            PoseArray,
            'nav_created_node',
            self.nav_create_node_callback,
            20)

        self.updateTM_publisher = self.create_publisher(PoseArray, 'new_node_at_TM', 10)

        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            "pose_at_map",
            self.pose_callback,
            1
        )
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            "map",
            self.map_callback,
            1
        )
        self.busy_status_subscriber = self.create_subscription(
            Bool,
            "busy_status",
            self.busy_status_callback,
            1
        )


        self.BC_busy_status_subscriber = self.create_subscription(
            Bool,
            "bc_busy_status",
            self.bc_status_callback,
            20
        )

        self.cur_node_id_subscriber = self.create_subscription(
            UInt32,
            "cur_robot_located_node",
            self.cur_robot_located_node_callback,
            1
        )

        self.detector_client = self.create_client(FindFrontier, 'find_frontier_srv')
        # self.mapSaver_client = self.create_client(Empty, 'map_save_srv')

    def nav_create_node_callback(self, msg):
        self.nav_new_nodes_received= True
        self.nav_created_new_nodes = msg
        print("get nav new nodes")
    def cur_robot_located_node_callback(self, msg):
        self.cur_robot_located_node_received= True
        self.robot_stand_node = msg.data
        print("robot_stand_node:", self.robot_stand_node)
    def bc_status_callback(self, msg):
        self.bc_busy_status = msg.data
    def busy_status_callback(self, msg):
        self.busy_status = msg.data
    def pose_callback(self, msg):
        self.pose_receive= True
        self.current_pose= msg
    def map_callback(self, msg):
        self.map_receive= True
        self.current_map= msg
        
    def TA_callback(self, msg):
        self.waypointReceived=True
        self.waypoint = msg
        

    def initRobot_callback(self, msg):
        self.initPose=msg

    def check_curpose_and_map_received(self):
        if self.pose_receive and self.map_receive:
            print('Received messages from map and curPose.')
            self.pose_receive = False
            self.map_receive = False
            return True

        return False

    # def send_request_for_mapSaver(self):
    #     req= Empty()
    #     self.future = self.mapSaver_client.call_async(req)
    #     return self.future.result()
    def send_request_for_detector(self, curPose, curPointID):
        req = FindFrontier.Request()
        req.cur_pos = curPose
        req.map_data= self.current_map
        req.pre_point_id= curPointID
        self.future = self.detector_client.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


# pose1: pose ,pos2 : point
def calOrientation(pose1, pos2):
    # 計算pos1朝向pos2的角度

    # 計算两个点的差值
    delta_x = pos2.x - pose1.position.x
    delta_y = pos2.y - pose1.position.y

    # 計算朝向角度（弧度）
    angle_rad = math.atan2(delta_y, delta_x)

    # 将弧度主換為四元数的朝向表示
    orientation = Quaternion()
    orientation.x = pose1.orientation.x
    orientation.y = pose1.orientation.y
    orientation.z = math.sin(angle_rad / 2.0)
    orientation.w = math.cos(angle_rad / 2.0)

    return orientation




def main(args=None):
    rclpy.init(args=args)

    node = testNode()
    
    print("init robot ")
    
    #Init robot to join blockchain
    print("robotID: {}".format(node.robotID))
    init_robot_msg = Int8()
    init_robot_msg.data = node.robotID
    node.initRobot_publisher.publish(init_robot_msg)
    node.bc_busy_status= True
    init_count = 0
    while rclpy.ok() and node.bc_busy_status == True:
        rclpy.spin_once(node)
        print("wating for bc complete")
        time.sleep(0.5)
        init_count += 1

        if init_count% 6 == 0:
            node.initRobot_publisher.publish(init_robot_msg)
    # Set the initial node of the robot in the topological map
    node.robot_stand_node= 0
    print("init done ")

    while rclpy.ok():
        rclpy.spin_once(node)
        print("get map and cur pose")
        if node.check_curpose_and_map_received():
            #get and show all frontier points
            # print("get all frontier points")
            # get_all_frontier_msg = Bool()
            # get_all_frontier_msg.data = True
            # node.get_all_frontier_publisher.publish(get_all_frontier_msg)

            #Receive Task (Task Allocation)
            print("Get the goal pose by task allocation from bc")

            #前16bit放robotID, 後16bit放robot_stand_node
            combined = (node.robotID << 16) | node.robot_stand_node
            task_allocation_msg = UInt32()            
            task_allocation_msg.data= combined
            # print("robot {} stand on {} node and retrive task".format(node.robotID, node.curPointID))
            node.task_allocation_publisher.publish(task_allocation_msg)

            node.waypointReceived= False
            while(not node.waypointReceived):
                print("Wait for Task Allocation....")
                rclpy.spin_once(node)
                time.sleep(0.5)
            if len(node.waypoint.poses)== 0:
                print("no more frontier point")
                #Find New Task
                print("Find New Task")
                node.send_request_for_detector(node.current_pose, node.robot_stand_node)
                print("Find New Task done")

                #waiting New Task added
                print("waiting New Task added")
                node.bc_busy_status = True
                while node.bc_busy_status== True:
                    rclpy.spin_once(node)
                    print("wating for bc complete")
                    time.sleep(1.0)

                #show tasks
                bool_msg = Bool()
                bool_msg.data = True
                node.get_all_frontier_publisher.publish(bool_msg)
                continue

            
            
            #show waypoint
            print("show waypoint")
            node.waypoint_pub.publish(node.waypoint)

            
            # adjust naving pose 
            for i in range(len(node.waypoint.poses)):
                if i == 0:
                    node.waypoint.poses[i].orientation = calOrientation(node.current_pose.pose, node.waypoint.poses[i].position)
                else:
                    node.waypoint.poses[i].orientation = calOrientation(node.waypoint.poses[i-1], node.waypoint.poses[i].position)


            #Perform waypoint
            print("naving")
            node.nav_pose_pub.publish(node.waypoint)
            node.nav_new_nodes_received = False
            while node.nav_new_nodes_received == False:
                print("wait for nav_new_nodes_reveived")
                rclpy.spin_once(node)
                time.sleep(1.0)
                
            print("navi complete")
            print("got num of {} new nodes from nav".format(len(node.nav_created_new_nodes.poses)))
            # node.nav_created_new_nodes : PoseArray



            #add node and edge to TM 
            print("add node and edge to TM ")
            str_robotID= "{}".format(node.robotID)
            print("my robotID: "+ str_robotID)
            node.nav_created_new_nodes.header.frame_id= str_robotID
            node.updateTM_publisher.publish(node.nav_created_new_nodes)
            
            node.cur_robot_located_node_received = False
            while node.cur_robot_located_node_received == False:
                rclpy.spin_once(node)
                print("wating for cur_robot_located_node obtained")
                time.sleep(1.0) 
            
            print("Robot stand on the Node: {}".format(node.robot_stand_node))

            node.pose_receive = False
            while node.pose_receive == False:
                print("get cur pose")
                rclpy.spin_once(node)
                time.sleep(1.0)
            
            #Find New Task
            print("Find New Task")
            node.send_request_for_detector(node.current_pose, node.robot_stand_node)
            print("Find New Task done")

            #waiting New Task added
            print("waiting New Task added")
            node.bc_busy_status = True
            while node.bc_busy_status== True:
                rclpy.spin_once(node)
                print("wating for bc complete")
                time.sleep(1.0)
            #show tasks
            bool_msg = Bool()
            bool_msg.data = True
            node.get_all_frontier_publisher.publish(bool_msg)
            #end bc

            #show TM
            # node.get_TM_publisher.publish(bool_msg)
            
            
        
    node.destroy_node()
    rclpy.shutdown()
    
    

if __name__ == "__main__":
    main()
