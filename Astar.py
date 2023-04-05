# -*- coding: utf-8 -*-
"""
Created on Wed Mar 29 16:17:43 2023

@author: gaohaotian
"""

import math
import numpy as np
from model import *

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
            tentative_g = 1/3*get_distance(current, neighbor)
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
            if x ==0 and y==0:
                continue
            if node.x+x/4 < 0.25 or node.x+x/4 >49.75 or node.y+y/4 < 0.25 or node.y+y/4 >49.75 or Node(node.x+x/4,node.y+y/4) in obstacles:
                continue
            neighbor = Node(node.x + x / 4, node.y + y / 4)
            neighbors.append(neighbor)
    return neighbors


def get_distance(node1, node2):
    return math.sqrt((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2)


def get_manhadundis(node1, node2):
    return abs(node1.x - node2.x) + abs(node1.y - node2.y)


# 获取起点到目标点的路径参数：地图、起点、终点、机器人是否携带商品     #main函数只调用get_astar_path函数 其他不用
def get_astar_path(game_map_array, rob_startpoint, goal, workstations, bot, iscarry):
    print("rob_startpoint:", rob_startpoint,file=sys.stderr)
    print("goal:",goal,file=sys.stderr)
    # game_map_array,rob_startpoint,goal,workstations,bots[i]
    obstacles = set()
    for i in range(len(game_map_array)):
        for j in range(len(game_map_array)):
            if game_map_array[i][j] == '#':  # 如果为‘#’，则为障碍物
                x=0.25 + j * 0.5
                y=(99.5 - i) * 0.5
                obstacle = Node(x, y)
    
                obstacles.add(obstacle)
                if iscarry==0:
                    for xx in range(-2,3):
                        for yy in range(-2,3):
                            if (xx==0 and yy==0 ):
                                continue
                            obstacles.add(Node(x+xx/4,y+yy/4))
                
                if iscarry==1:
                    for xx in range(-3,4):
                        for yy in range(-3,4):
                            if (xx==0 and yy==0 or abs(xx*yy)>=6 ):
                                continue
                            obstacles.add(Node(x+xx/4,y+yy/4))

    start = Node(rob_startpoint[0], rob_startpoint[1])  # 起点
    goal = Node(workstations[goal].x, workstations[goal].y)  # 终点
    path = astar(start, goal, obstacles)  # 查找路径
    if path:  # 找到路径则返回
        return path
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


def get_next(current,obstacles):
    neighbors=[]
    for x in range(-1, 2):
        for y in range(-1, 2):
            
            if x *y!=0:
                continue
            if current.x+x < 0.25 or current.x+x >49.75 or current.y+y < 0.25 or current.y+y >49.75 or Node(current.x+x/4,current.y+y/4) in obstacles:
                continue
            neighbor = Node(current.x+x/2,current.y+y/2)
            neighbors.append(neighbor)
    return neighbors




def find_live_points(start, obstacles):
    open_set = set()
    closed_set = set()
    current = start
    open_set.add(current)
    count=0
    while open_set:
        count+=1
        current = min(open_set, key=lambda x: x.g + x.h)
    
        open_set.remove(current)
        closed_set.add(current)
        
        for neighbor in get_next(current, obstacles):
               
            if neighbor in closed_set:
                continue
            
            if neighbor not in open_set:
                open_set.add(neighbor)
    return closed_set

def is_live(live_points,pos_x,pos_y):
    pos=Node(pos_x,pos_y)
    if pos in live_points:
        return True
    else:
        return False


def set_oflivepoints(map_str):
    
    
    obstacles =set()
    workstations=[]
    for i in range(len(map_str)):
        for j in range(len(map_str)):
            if map_str[i][j] == '#':  # 如果为‘#’，则为障碍物
                x=0.25 + j * 0.5
                y=(99.5 - i) * 0.5
                obstacle = Node(x, y)
                iscarry=1
                obstacles.add(obstacle)
                if iscarry==0:
                    for xx in range(-2,3):
                        for yy in range(-2,3):
                            if (xx==0 and yy==0 ):
                                continue
                            obstacles.add(Node(x+xx/4,y+yy/4))
                
                if iscarry==1:
                    for xx in range(-3,4):
                        for yy in range(-3,4):
                            if (xx==0 and yy==0 or abs(xx*yy)>=6 ):
                                continue
                            obstacles.add(Node(x+xx/4,y+yy/4))
                            
            elif map_str[i][j]=='.':
                start_point=Node(0.25 + j * 0.5, (99.5 - i) * 0.5) 
                
            elif map_str[i][j]!='#' and map_str[i][j]!='.' and map_str[i][j]!='A' and map_str[i][j]!='\n':
               
                workstation=Node(0.25+(j)*0.5,(99.5-i)*0.5)
                workstations.append(workstation)

    
    
    start=start_point
       
    # start=Node(start_x,start_y)
    islive_worksations=np.full((len(workstations), 1),1)
    live_poins=find_live_points(start,obstacles)
    for i in range(len(workstations)):
        if  not is_live(live_poins,workstations[i].x,workstations[i].y):
            islive_worksations[i][0]=0
            
    return islive_worksations,live_poins




    









