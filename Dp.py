# -*- coding: utf-8 -*-
"""
Created on Sat Apr  1 21:08:45 2023

@author: gaohaotian
"""

import math
import numpy as np

def remove_triangles(path):
    new_path = [path[0]]
    i = 0
    while i < len(path) - 2:
        if not np.all(path[i] == path[i+2]):
            new_path.append(path[i+1])
            i += 1
        i += 1
    new_path.append(path[-1])
    return np.array(new_path)


def simplify_path(points, tolerance):
    """
    使用Douglas-Peucker算法对路径进行简化
    """
    if len(points) < 3:
        return points

    # 找到路径上距离起点和终点最远的点
    d_max = 0
    index = 0
    end = len(points) - 1
    for i in range(1, end):
        d = perpendicular_distance(points[i], points[0], points[end])
        if d > d_max:
            index = i
            d_max = d

    # 如果最远点的距离小于阈值，则直接连接起点和终点
    if d_max < tolerance:
        return [points[0], points[end]]

    # 递归地对路径的左右两部分进行简化
    left = simplify_path(points[:index+1], tolerance)
    right = simplify_path(points[index:], tolerance)

    # 返回简化后的路径
    return left[:-1] + right


def perpendicular_distance(point, line_start, line_end):
    """
    计算点到直线的垂直距离
    """
    x1, y1 = line_start
    x2, y2 = line_end
    x0, y0 = point
    if x1 == x2:
        # 如果直线垂直于x轴，则直接计算距离
        return abs(x0 - x1)
    else:
        # 否则计算点到直线的距离
        k = (y2 - y1) / (x2 - x1)
        b = y1 - k * x1
        return abs(k * x0 - y0 + b) / math.sqrt(k**2 + 1)




def optimize_path(path):
    """
    对机器人的路径进行优化，保留拐点
    """
    # 将路径转换为点列表
    # path=remove_triangles(path) 
    # path=remove_triangles(path) 
    # path=remove_triangles(path) 
    points = [(path[i][0], path[i][1]) for i in range(len(path))]

    # 使用Douglas-Peucker算法对路径进行简化
    tolerance = 0.11# 阈值可以根据需要进行调整
    simplified_points = simplify_path(points, tolerance)

    # 将简化后的路径转换回原来的格式
    optimized_path = []
    for i in range(len(simplified_points)):
        x, y = simplified_points[i]
        optimized_path.append([x, y])
        # if i < len(simplified_points) - 1 and simplified_points[i+1][0] != x and simplified_points[i+1][1] != y:
        #     # 如果下一个点和当前点的x或y坐标不相等，则插入一个拐点
        #     optimized_path.append([x, y])
     
            
    # best_path=np.full((len(optimized_path), 2), 0.0) 
    # i=0
    # for node in optimized_path:
    #     # print(node.x, node.y)
    #     best_path[i][0]=node[0]
    #     best_path[i][1]=node[1]
    #     i+=1
    return optimized_path
