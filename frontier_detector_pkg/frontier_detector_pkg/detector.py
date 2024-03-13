#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Point
from nav_msgs.msg import OccupancyGrid
import numpy as np
import cv2
from rclpy.node import Node
from rclpy.time import Time
from rclpy.clock import Clock
from frontier_detector_pkg.functions import clustering, find_centroid, find_close_point
from my_interfaces.srv import FindFrontier
from copy import copy
import math



class Detector(Node):
    def __init__(self):
        super().__init__(node_name='frontier_detector')
        self.declare_parameter('NUMBER_OF_CLUSTER', 5)
        self.NUMBER_OF_CLUSTER = self.get_parameter('NUMBER_OF_CLUSTER').value

        self.frontier_pub = self.create_publisher(PoseArray, 'frontier_points', 10)
        self.FindFrontierSrv = self.create_service(FindFrontier, 'find_frontier_srv', self.find_frontier_callback)
        print("NUMBER_OF_CLUSTER: {}".format(self.NUMBER_OF_CLUSTER) )
    def find_frontier_callback(self, request, response):
        print("FF staring")
        response.frontiers = self.getFrontier(request.cur_pos, request.map_data, request.pre_point_id)
        return response

    def getFrontier(self, cur_pose, mapData, prePointID):
        rawData = mapData.data
        w = mapData.info.width
        h = mapData.info.height
        resolution = mapData.info.resolution
        Xstartx = mapData.info.origin.position.x
        Xstarty = mapData.info.origin.position.y
        # print("W ", w, "H:", h, "res:", resolution, "sx: ", Xstartx, "sy: ", Xstarty)

        img = np.zeros((h, w, 1), np.uint8)

        for i in range(0,h):
            for j in range(0,w):
                if rawData[i*w+j]==100: # occupied
                    img[i, j]= 0
                elif rawData[i*w+j]==-1:#unknown
                    img[i,j]=205
                elif rawData[i*w+j]<=50:##freee
                    img[i,j]=255
                
      
        # cv2.imshow('im', img)
        #free黑，未知灰，占用白
        #0 for black 255 for white
        #0~1變白(255)，其他變黑(0)
        o=cv2.inRange(img,0,1)
        edges = cv2.Canny(img,0,255)
        # cv2.imshow('ed', edges)
        # cv2.imshow('o', o)
        
        contours, _ = cv2.findContours(o,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)

        cv2.drawContours(o, contours, -1, (255,255,255), 3)
        
        o= cv2.bitwise_not(o)
        # cv2.imshow('no', o)
        res= cv2.bitwise_and(o, edges)
        # test the contours img 
        # test = cv2.cvtColor(o, cv2.COLOR_GRAY2RGB)
        # cv2.drawContours(test, contours, -1, (0,255,0), 1)
        # cv2.imshow('an', res)
        # frontier=copy(res)
        # contours, hierarchy = cv2.findContours(frontier,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        # cv2.drawContours(frontier, contours, -1, (255,255,255), 2)

        # contours, hierarchy = cv2.findContours(frontier,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        frontier=copy(res)
        contours, _ = cv2.findContours(frontier,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(frontier, contours, -1, (255,255,255), 2)
        # cv2.imshow('fr1', frontier)

        contours, _ = cv2.findContours(frontier,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(frontier, contours, -1, (255,0,0), 1)
        # cv2.imshow('fr2', frontier)
        

        all_contour_point = []
        for i in range(0,len(contours)):
            for j in range(0, len(contours[i])):
                for k in range(0, len(contours[i][j])):
                    all_contour_point.append(contours[i][j][k])

        # clustering
        if len(all_contour_point) > 0:
            # clustering_contours= clustering(all_contour_point, NUMBER_OF_CLUSTER) advised

            cluster_centroid= clustering(all_contour_point, self.NUMBER_OF_CLUSTER)
            # find the centroid for each clustering
            # cluster_centroid = []  advised
            # for i in range(0, NUMBER_OF_CLUSTER): advised 
            #     cluster_centroid.append(find_centroid(clustering_contours[i])) advised
            # print("CC: ", cluster_centroid)
            

            # pub the original frontier point to rviz
            # ori_frontiers= PoseArray()
            # for i in range(0, len(cluster_centroid)):
            #     pose= Pose()
            #     pose.position.x= cluster_centroid[i][0]* resolution + Xstartx
            #     pose.position.y= cluster_centroid[i][1]* resolution + Xstarty
            #     ori_frontiers.poses.append(pose)
            # print("pub original forniter point")
            # self.ori_frontier_pub.publish(ori_frontiers)

            #find the grid between the cur_pos and goal_point by distance d
            # distance = 30 #單位 格(grid)
            # cur_x = int((cur_pose.pose.position.x - Xstartx) / resolution)
            # cur_y = int((cur_pose.pose.position.y - Xstarty) / resolution)
            # for i in range(0, NUMBER_OF_CLUSTER):
            #     cluster_centroid[i][0], cluster_centroid[i][1] = find_close_point(cur_x, cur_y, cluster_centroid[i][0], cluster_centroid[i][1], distance)
            # grid


            # world position
            res_centroid = []
            for i in range(0, len(cluster_centroid)):
                pt = Point()
                pt.x = cluster_centroid[i][0]* resolution + Xstartx
                pt.y= cluster_centroid[i][1]* resolution + Xstarty
                res_centroid.append(pt)
            # print("world postion:", res_centroid)

            # pub frontiers
            #prePointID轉成string塞在frame_id裡面
            print("frontier prevois node iD:{}".format(prePointID))
            exploration_goals= PoseArray()
            str_prePointID = "{}".format(prePointID)
            exploration_goals.header.frame_id= str_prePointID
            for i in range(0, len(cluster_centroid)):
                pose= Pose()
                pose.position.x= res_centroid[i].x
                pose.position.y= res_centroid[i].y
                #dis from pre node
                pose.orientation.x = math.sqrt((cur_pose.pose.position.x - res_centroid[i].x)**2 + (cur_pose.pose.position.y- res_centroid[i].y)**2)
                exploration_goals.poses.append(pose)
            
            print("pub frontiers")
            self.frontier_pub.publish(exploration_goals)

            return res_centroid

        print("no frontiers found")
        # exploration_goals= PoseArray()
        # str_prePointID = "{}".format(prePointID)
        # exploration_goals.header.frame_id= str_prePointID
        self.frontier_pub.publish(PoseArray())
        
        return []
        



def main(args=None):
    rclpy.init()
    node = Detector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()