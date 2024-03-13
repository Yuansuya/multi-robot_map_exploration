#! /usr/bin/env python3


import time
import nav_msgs
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose

import rclpy

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.duration import Duration

import tf2_ros
from my_interfaces.srv import MoveGoal
import os
from nav2_msgs.srv import SaveMap, LoadMap
import math
from std_msgs.msg import Bool

class BasicNavigator(Node):
    def __init__(self):
        super().__init__(node_name='basic_navigator')
        # 999999 disable max_step 
        self.declare_parameter('max_step', 5)
        self.max_step = self.get_parameter('max_step').value
        self.my_nav_poses = PoseArray()
        self.new_nav_task_received = False
        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.status = None

        self.create_node_list= PoseArray()
        self.create_node_list.header.frame_id= 'map'
        
        # self.cur_pose= PoseStamped()
        # self.cur_pose_received= False
 

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.create_node_pub = self.create_publisher(PoseArray,
                                                'nav_created_node',
                                                1)

        self.my_nav_pose_subscriber = self.create_subscription(
            PoseArray,
            "my_nav_poses",
            self.my_nav_poses_callback,
            1
        )

        # self.cur_pose_subscriber = self.create_subscription(
        #     PoseStamped,
        #     "pose_at_map",
        #     self.cur_pose_callback,
        #     1
        # )
    
    # def cur_pose_callback(self, msg):
    #     self.cur_pose_received= True
    #     self.cur_pose = msg
        
    def my_nav_poses_callback(self, msg):
        self.new_nav_task_received= True 
        self.my_nav_poses= msg

    def goToPose(self, pose):
        # Sends a `NavToPose` action request and waits for completion
        self.debug("Waiting for 'NavigateToPose' action server")
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateToPose' action server not available, waiting...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.info('Navigating to goal: ' + str(pose.pose.position.x) + ' ' +
                      str(pose.pose.position.y) + '...')
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg,
                                                                   self._feedbackCallback)
        
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()
        
        if not self.goal_handle.accepted:
            self.error('Goal to ' + str(pose.pose.position.x) + ' ' +
                           str(pose.pose.position.y) + ' was rejected!')
            return False
        self.result_future = self.goal_handle.get_result_async()
        return True

    def cancelNav(self):
        self.info('Canceling current goal.')
        if self.result_future:
            future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future)
        return

    def isNavComplete(self):
        if not self.result_future:
            # task was cancelled or completed
            return True
        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.10)
        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                self.info('Goal with failed with status code: {0}'.format(self.status))
                return True
        else:
            # Timed out, still processing, not complete yet
            return False

        self.info('Goal succeeded!')
        return True
    def getFeedback(self):
        return self.feedback

    def getResult(self):
        return self.status

   
    def _feedbackCallback(self, msg):
        self.feedback = msg.feedback
        return

    def info(self, msg):
        self.get_logger().info(msg)
        return

    def warn(self, msg):
        self.get_logger().warn(msg)
        return

    def error(self, msg):
        self.get_logger().error(msg)
        return

    def debug(self, msg):
        self.get_logger().debug(msg)
        return
    def filter(self, pose_list):
        if len(pose_list.poses) == 0:
            return pose_list
        dis_threshold= 0.3
        res= PoseArray()
        res.header.frame_id= 'map'
        res.poses.append(pose_list.poses[-1])
        for i in range(len(pose_list.poses)-2, -1, -1):

            dx= (pose_list.poses[i].position.x - res.poses[-1].position.x)**2
            dy= (pose_list.poses[i].position.y - res.poses[-1].position.y)**2    
            dis= math.sqrt(dx+ dy)
            # print(dis)
            if dis > dis_threshold:
                res.poses.append(pose_list.poses[i])
        
        l= 0
        r= len(res.poses)-1
        while l<= r:
            res.poses[l], res.poses[r] = res.poses[r], res.poses[l]
            l += 1
            r -= 1

        print("add {} new node".format(len(res.poses)))
        return res


        



def main(args=None):
    rclpy.init()
    navigator = BasicNavigator()
    
    time_threshold= 2_500_000_000
    while rclpy.ok():
        #wait for service client to get the goal point
        rclpy.spin_once(navigator)
        if navigator.new_nav_task_received == True:
            # update the map, saving and loading the new one
            print("{} steps need to navi".format(len(navigator.my_nav_poses.poses)))
            
            pre_nav_pose = None
            for i in range(len(navigator.my_nav_poses.poses)):
                if i > navigator.max_step:
                    print("reassign frontier point")
                    break
                cur_nav_pose = PoseStamped()
                cur_nav_pose.header.frame_id= 'map'
                cur_nav_pose.pose= navigator.my_nav_poses.poses[i]
                navigator.goToPose(cur_nav_pose)
                last_execution_time= Duration()

                feedback_pose = Pose()
                while not navigator.isNavComplete():

                    # Do something with the feedback
                    feedback = navigator.getFeedback()  
                    
                    # print(feedback)
                    if(feedback == None):
                        print("idk why feedback is None==")
                        break
                    
                    if Duration.from_msg(feedback.navigation_time) > Duration(seconds=25.0):
                        navigator.cancelNav()

                    
                    current_time = Duration.from_msg(feedback.navigation_time)
                    time_diff = current_time.nanoseconds - last_execution_time.nanoseconds
                    # 3 sec
                    if time_diff >= time_threshold:
                        feedback_pose= feedback.current_pose.pose
                        if pre_nav_pose != None and ((feedback_pose.position.x - pre_nav_pose.position.x)**2 + (feedback_pose.position.y - pre_nav_pose.position.y)**2) < 0.1:
                            print("robot is not moving ")
                            navigator.cancelNav()

                        print("feedback.current_pose.pose.x : {}, feedback.current_pose.pose.y : {}".format(feedback.current_pose.pose.position.x, feedback.current_pose.pose.position.y))     
                        navigator.create_node_list.poses.append(feedback.current_pose.pose)
                        # navigator.cur_pose_received= False
                        last_execution_time = current_time
                    
                    pre_nav_pose = feedback_pose
                result = navigator.getResult()
                if result == GoalStatus.STATUS_SUCCEEDED:
                    print('Goal succeeded!')
                elif result == GoalStatus.STATUS_CANCELED:
                    print('Goal was canceled!')
                elif result == GoalStatus.STATUS_ABORTED:
                    print('Goal failed!')
                else:
                    print('Goal has an invalid return status!')
                print("next point ")

            
            navigator.create_node_list = navigator.filter(navigator.create_node_list)
            print("num of new nodes: {}".format(len(navigator.create_node_list.poses)))
            navigator.create_node_pub.publish(navigator.create_node_list)
            navigator.create_node_list.poses.clear()
            navigator.new_nav_task_received= False  
           

    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()