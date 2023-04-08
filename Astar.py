# -*- coding: utf-8 -*-
"""
Created on Wed Mar 29 16:17:43 2023

@author: gaohaotian
"""

import math
import numpy as np
from model import *
from Dp import *
import time

# 地图大小50×50米
MAP_SIZE = (50, 50)


# Node类
class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.g = 0
        self.h = 0
        self.parent = None
        self.obstacle = False

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __hash__(self):
        return hash((self.x, self.y))


# A*算法
def astar(start, goal, obstacles):
    open_set = set()
    closed_set = set()
    current = start
    open_set.add(current)
    counter = 1
    while open_set:
        counter += 1
        # if counter >= 10000:
        #     return None
        current = min(open_set, key=lambda x: x.g + x.h)
        open_set.remove(current)
        closed_set.add(current)

        if get_distance(current, goal) < 0.4:
            path = []
            while current:
                path.append(current)
                current = current.parent
            return path[::-1]

        for neighbor in get_neighbors(current, obstacles):
            if neighbor in closed_set:
                continue
            # tentative_g = current.g+ 1/3*get_distance(current, neighbor)
            tentative_g = 1 / 3 * get_distance(current, neighbor)
            if neighbor not in open_set:
                open_set.add(neighbor)
                neighbor.h = get_distance(neighbor, goal)
            elif tentative_g >= neighbor.g:
                continue

            neighbor.parent = current
            neighbor.g = tentative_g

    return None


# 获取当前点的邻居点
def get_neighbors(node, obstacles):
    neighbors = []
    for x in range(-1, 2):
        for y in range(-1, 2):
            if x == 0 and y == 0:
                continue
            if node.x + x / 4 < 0.25 or node.x + x / 4 > 49.75 or node.y + y / 4 < 0.25 or node.y + y / 4 > 49.75 or Node(
                    node.x + x / 4, node.y + y / 4) in obstacles:
                continue
            neighbor = Node(node.x + x / 4, node.y + y / 4)
            neighbors.append(neighbor)
    return neighbors


def get_distance(node1, node2):
    return math.sqrt((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2)


def get_manhadundis(node1, node2):
    return abs(node1.x - node2.x) + abs(node1.y - node2.y)


# 计算点P到线段AB的距离
def point_to_line_distance(x, y, A, B):
    ax = A[0]
    ay = A[1]
    bx = B[0]
    by = B[1]
    # 计算线段AB的长度
    L = math.sqrt((bx - ax) ** 2 + (by - ay) ** 2)

    # 将线段AB向量标准化
    ABx = (bx - ax) / L
    ABy = (by - ay) / L

    # 计算点P到线段起点A的向量AP
    APx = x - ax
    APy = y - ay

    # 计算AP在AB'上的投影
    d = APx * ABx + APy * ABy

    if d < 0:
        # 最短距离为点P到线段起点A的距离
        return math.sqrt((x - ax) ** 2 + (y - ay) ** 2)
    elif d > L:
        # 最短距离为点P到线段终点B的距离
        return math.sqrt((x - bx) ** 2 + (y - by) ** 2)
    else:
        # 计算点P到线段AB上距离点P最近的点C到点P的距离
        Cx = ax + ABx * d
        Cy = ay + ABy * d
        return math.sqrt((x - Cx) ** 2 + (y - Cy) ** 2)


# 找到线段两侧到线段距离小于d的点
def find_points_on_both_sides_of_line(A, B, d, obs_of_wall):
    points = []
    # 如果线段AB垂直与X轴
    if A[0] == B[0]:
        for xx in range(round(4 * (A[0] - d)), round(4 * (A[0] + d + 1))):
            for yy in range(round(4 * (min(A[1], B[1]) - d)), round(4 * (max(A[1], B[1]) - d))):
                distance = point_to_line_distance(xx / 4, yy / 4, A, B)
                if distance < d:
                    if Node(xx / 4, yy / 4) in obs_of_wall:
                        return False
    elif A[1] == B[1]:
        for xx in range(round(4 * (min(A[0], B[0]) - d - 1)), round(4 * (max(A[0], B[0]) + d + 1))):
            for yy in range(round(4 * (A[1] - d - 1)), round(4 * (A[1] + d + 1))):
                distance = point_to_line_distance(xx / 4, yy / 4, A, B)
                if distance < d:
                    if Node(xx / 4, yy / 4) in obs_of_wall:
                        return False
    else:
        # AB
        k = (B[1] - A[1]) / (B[0] - A[0])
        b = A[1] - k * A[0]
        # y1
        k1 = k
        b1 = d / math.cos(math.atan(k1)) + b

        # y2
        k2 = k
        b2 = -d / math.cos(math.atan(k2)) + b

        # y3
        k3 = -1 / k
        b3 = max(A[1], B[1]) - k3 * ((max(A[1], B[1]) - b) / k)

        # y4
        k4 = -1 / k
        b4 = min(A[1], B[1]) - k4 * ((min(A[1], B[1]) - b) / k)

        # y1*y4
        x14 = (b4 - b1) / (k1 - k4)
        y14 = k1 * x14 + b1

        # y1*y3
        x13 = (b3 - b1) / (k1 - k3)
        y13 = k1 * x13 + b1

        # y2*y3
        x23 = (b3 - b2) / (k2 - k3)
        y23 = k2 * x23 + b2

        # y2*y4
        x24 = (b4 - b2) / (k2 - k4)
        y24 = k2 * x24 + b2

        # 区域1
        for xx in range(round(4 * min(x14, x13, x23, x24) - 4), round(4 * max(x14, x13, x23, x24) + 4)):
            for yy in range(round(4 * min(y14, y13, y23, y24) - 4), round(4 * max(y14, y13, y23, y24) + 4)):
                x = xx / 4
                y = yy / 4

                distance = point_to_line_distance(x, y, A, B)
                if distance < d:
                    if Node(xx / 4, yy / 4) in obs_of_wall:
                        return False
    return True


def optimize_path1(path, obs_of_wall, iscarry):
    new_path = []
    if len(path) == 0:
        return new_path
    new_path.append(path[0])
    i = 0
    time_start = time.time()
    while i < len(path) - 1:
        time_end = time.time()
        if time_end - time_start > 0.6:
            print("time:", time_end - time_start, file=sys.stderr)
            return path, 1
        for j in range(i + 1, len(path)):
            A = (path[i][0], path[i][1])
            B = (path[j][0], path[j][1])
            if not find_points_on_both_sides_of_line(A, B, 0.45 + iscarry * 0.08, obs_of_wall):
                new_path.append(path[j - 1])
                i = j - 1
                break
        else:
            new_path.append(path[-1])
            break
    # if (new_path[0][0]-new_path[1][0])**2+(new_path[0][1]-new_path[1][1])**2<0.3**2:
    #     new_path.pop(0)
    return np.array(new_path), 0


# 获取起点到目标点的路径参数：地图、起点、终点、机器人是否携带商品     #main函数只调用get_astar_path函数 其他不用
def get_astar_path(game_map_array, rob_startpoint, goal, workstations, bot, iscarry):
    # print("rob_startpoint:", rob_startpoint,file=sys.stderr)
    # print("goal:",goal,file=sys.stderr)
    # game_map_array,rob_startpoint,goal,workstations,bots[i]
    obstacles = set()
    obs_of_wall = set()
    for i in range(len(game_map_array)):
        for j in range(len(game_map_array)):
            if game_map_array[i][j] == '#':  # 如果为‘#’，则为障碍物
                x = 0.25 + j * 0.5
                y = (99.5 - i) * 0.5
                obstacle = Node(x, y)

                obstacles.add(obstacle)
                if iscarry == 0:
                    for xx in range(-2, 3):
                        for yy in range(-2, 3):
                            if (xx == 0 and yy == 0):
                                continue
                            obstacles.add(Node(x + xx / 4, y + yy / 4))

                if iscarry == 1:
                    for xx in range(-3, 4):
                        for yy in range(-3, 4):
                            if (xx == 0 and yy == 0 or abs(xx * yy) >= 6):
                                continue
                            obstacles.add(Node(x + xx / 4, y + yy / 4))

                for xx in range(-1, 2):
                    for yy in range(-1, 2):
                        if xx == 0 and yy == 0:
                            continue
                        obs_of_wall.add(Node(x + xx / 4, y + yy / 4))

    start = Node(rob_startpoint[0], rob_startpoint[1])  # 起点
    goal = Node(workstations[goal].x, workstations[goal].y)  # 终点
    path1 = []
    path1 = astar(start, goal, obstacles)  # 查找路径
    path = []
    # path.append([bot.x,bot.y])
    for node in path1:
        path.append([node.x, node.y])

    path = optimize_path(path)
    path, a = optimize_path1(path, obs_of_wall, 0)
    if a == 0:
        path, b = optimize_path1(path, obs_of_wall, 0)
    # print("path:", path,file=sys.stderr)
    final_path = []
    for node in path:
        final_path.append(Node(node[0], node[1]))

    if final_path:  # 找到路径则返回
        return final_path
    else:
        return None  # 未找到路径范围None


# def is_valid(x, y, obstacles,R):

#     if x < 0.25 or x > 49.75 or y <0.25 or y > 49.75:
#         return False
#     for obstacle in obstacles:
#         if (x > obstacle.x - R-0.04 and x < obstacle.x + R+0.04 and 
#             y > obstacle.y - R-0.04 and y < obstacle.y + R+0.04):
#             return False
#     return True


def get_next(current, obstacles):
    neighbors = []
    for x in range(-1, 2):
        for y in range(-1, 2):

            if x * y != 0:
                continue
            if current.x + x < 0.25 or current.x + x > 49.75 or current.y + y < 0.25 or current.y + y > 49.75 or Node(
                    current.x + x / 4, current.y + y / 4) in obstacles:
                continue
            neighbor = Node(current.x + x / 2, current.y + y / 2)
            neighbors.append(neighbor)
    return neighbors


def find_live_points(start, obstacles):
    open_set = set()
    closed_set = set()
    current = start
    open_set.add(current)
    count = 0
    while open_set:
        count += 1
        current = min(open_set, key=lambda x: x.g + x.h)

        open_set.remove(current)
        closed_set.add(current)

        for neighbor in get_next(current, obstacles):

            if neighbor in closed_set:
                continue

            if neighbor not in open_set:
                open_set.add(neighbor)
    return closed_set


def is_live(live_points, pos_x, pos_y):
    pos = Node(pos_x, pos_y)
    if pos in live_points:
        return True
    else:
        return False


def set_oflivepoints(map_str):
    obstacles = set()
    workstations = []
    for i in range(len(map_str)):
        for j in range(len(map_str)):
            if map_str[i][j] == '#':  # 如果为‘#’，则为障碍物
                x = 0.25 + j * 0.5
                y = (99.5 - i) * 0.5
                obstacle = Node(x, y)
                iscarry = 1
                obstacles.add(obstacle)
                if iscarry == 0:
                    for xx in range(-2, 3):
                        for yy in range(-2, 3):
                            if xx == 0 and yy == 0:
                                continue
                            obstacles.add(Node(x + xx / 4, y + yy / 4))

                if iscarry == 1:
                    for xx in range(-3, 4):
                        for yy in range(-3, 4):
                            if xx == 0 and yy == 0 or abs(xx * yy) >= 6:
                                continue
                            obstacles.add(Node(x + xx / 4, y + yy / 4))

            elif map_str[i][j] == '.':
                start_point = Node(0.25 + j * 0.5, (99.5 - i) * 0.5)

            elif map_str[i][j] != '#' and map_str[i][j] != '.' and map_str[i][j] != 'A' and map_str[i][j] != '\n':

                workstation = Node(0.25 + (j) * 0.5, (99.5 - i) * 0.5)
                workstations.append(workstation)

    start = start_point

    # start=Node(start_x,start_y)
    islive_workstations = np.full((len(workstations), 1), 1)
    live_points = find_live_points(start, obstacles)
    for i in range(len(workstations)):
        if not is_live(live_points, workstations[i].x, workstations[i].y):
            islive_workstations[i][0] = 0

    return islive_workstations, live_points
