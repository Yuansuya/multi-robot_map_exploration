#!/usr/bin/env python3
import numpy
from numpy import array
from numpy.linalg import norm
import math
from sklearn.cluster import KMeans
import cv2
from geometry_msgs.msg import Point
class KDTree:
    def __init__(self, data):
        def build_kd_tree(points, depth):
            if not points:
                return None
            axis = depth % k
            sorted_points = sorted(points, key=lambda point: point[axis])
            mid = len(points) // 2
            return {
                'point': sorted_points[mid],
                'left': build_kd_tree(sorted_points[:mid], depth + 1),
                'right': build_kd_tree(sorted_points[mid+1:], depth + 1)
            }

        k = len(data[0])
        self.root = build_kd_tree(data, 0)

    def points_within_radius(self, x, y, radius):
        def search(node, x, y, radius, depth):
            def distance(p1, p2):
                """兩點距離"""
                return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
            if not node:
                return []
            point = node['point']
            if point[0] >= x - radius and point[0] <= x + radius and point[1] >= y - radius and point[1] <= y + radius:
                if distance(point, (x, y)) <= radius:
                    result.append(point)
            axis = depth % k
            if x - radius < point[axis]:
                search(node['left'], x, y, radius, depth + 1)
            if x + radius > point[axis]:
                search(node['right'], x, y, radius, depth + 1)

        result = []
        k = 2
        search(self.root, x, y, radius, 0)
        return result
def find_neighboring_contours(mapData, x, y, r, contours):
    
    #contours location in map not real
    #x,y is in real , need to transfer
    tree = KDTree(contours)
    px = math.floor(x/ mapData.info.resolution)
    py = math.floor(y/ mapData.info.resolution)
    pr = math.floor(r/ mapData.info.resolution)
    #find the contours at the area by radius,pr 
    contours_within_radius = tree.points_within_radius(px, py, pr)
    return contours_within_radius

def clustering(contours, n):


    # KMeans
    np_contours = numpy.array(contours)
    kmeans = KMeans(n_clusters= n)
    kmeans.fit(np_contours)

    # clustering result
    labels = kmeans.labels_
    centroids = kmeans.cluster_centers_

    # 每個點到每個cluster中心的距離
    distances = kmeans.transform(np_contours)
    threshold = 100  #距離threshold

    # 重新調整cluster中心
    for i in range(len(np_contours)):
        if numpy.min(distances[i]) > threshold:
            # 將距離最遠的點作為新的cluster中心
            new_centroid = np_contours[i]
            centroids = numpy.vstack([centroids, new_centroid])
            labels[i] = len(centroids) - 1

    res = []
    for centroid in centroids:
        res.append([int(centroid[0]), int(centroid[1])])

    return res


def find_centroid(points):
    n = len(points)
    sum_x = sum(point[0] for point in points)
    sum_y = sum(point[1] for point in points)
    #格子
    centroid_x = int(sum_x / n)
    centroid_y = int(sum_y / n)
    return [centroid_x, centroid_y]

def find_close_point(cur_x, cur_y, fr_x, fr_y, dis):
    #朝向邊界點走距離d的位置點

    # 計算rx,ry到px,py的向量
    px_rx = fr_x - cur_x
    py_ry = fr_y - cur_y
    
    # 計算gx,gy跟px,py的向量的比率
    ratio = dis / math.sqrt(px_rx ** 2 + py_ry ** 2)
    
    # 計算gx,gy的座標
    gx = cur_x + px_rx * ratio
    gy = cur_y + py_ry * ratio
    
    return int(gx), int(gy)

