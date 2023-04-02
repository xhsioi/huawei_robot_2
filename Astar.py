# -*- coding: utf-8 -*-
"""
Created on Wed Mar 29 16:17:43 2023

@author: gaohaotian
"""

import math
import numpy as np
from model import *
#地图大小50×50米
MAP_SIZE = (50, 50)


#Node类
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


#A*算法
def astar(start, goal, obstacles,R):
    open_set = set()
    closed_set = set()
    current = start
    open_set.add(current)
    counter=1
    while open_set:
        counter+=1
        if counter>=10000:
            return None
        current = min(open_set, key=lambda x: x.g + x.h)
        open_set.remove(current)
        closed_set.add(current)

        if get_distance(current, goal)<0.4:
            path = []
            while current:
                path.append(current)
                current = current.parent
            return path[::-1]

        for neighbor in get_neighbors(current, obstacles,R):
            if neighbor in closed_set:
                continue

            tentative_g = current.g + get_distance(current, neighbor)
            if neighbor not in open_set:
                open_set.add(neighbor)
                neighbor.h = get_distance(neighbor, goal)
            elif tentative_g >= neighbor.g:
                continue

            neighbor.parent = current
            neighbor.g = tentative_g
    
    return None

#判断点是否违法
def is_valid_location(x, y, obstacles,R):

    if x <= 0.25 or x >= 49.75 or y <= 0.25 or y >= 49.75: #超出边界，违法
        return False
    for obstacle in obstacles:
        if (x > obstacle.x - R-0.04 and x < obstacle.x + R+0.04 and  #进入障碍物，违法
           y > obstacle.y - R-0.04 and y < obstacle.y + R+0.04):
            return False
    return True

#获取当前点的邻居点
def get_neighbors(node, obstacles,R):
    neighbors = []
    if R==0.45+0.25:
        K=4
    else:
        K=2
    for x in range(-1, 2):
        for y in range(-1, 2):
            if x==0 and y==0:
                continue
            
            if not is_valid_location(node.x + x/K, node.y + y/K, obstacles,R):
                continue
            neighbor = Node(node.x + x/K, node.y + y/K)
            neighbors.append(neighbor)
    return neighbors

def get_distance(node1, node2):
    return math.sqrt((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2)
def get_manhadundis(node1,node2):
    return abs(node1.x - node2.x) + abs(node1.y - node2.y)


#获取起点到目标点的路径参数：地图、起点、终点、机器人是否携带商品     #main函数只调用get_astar_path函数 其他不用
def get_astar_path(game_map_array,rob_startpoint,goal,workstations,bot,iscarry):
    print("牛牛牛\n", file=sys.stderr)
    # game_map_array,rob_startpoint,goal,workstations,bots[i]
    obstacles=[]
    if iscarry==0:
        
        R=0.25+0.45  #未携带产品，半径为机器人半径＋障碍物半径
    else:
        R=0.45+0.25+0.08 #携带产品，半径增加
    for i in range(len(game_map_array)):
        for j in range(len(game_map_array)):
            if game_map_array[i][j]=='#':  #如果为‘#’，则为障碍物
                obstacle=Node(0.25+(j)*0.5,(99.5-i)*0.5)
                obstacles.append(obstacle)
                
    start=Node(rob_startpoint[0],rob_startpoint[1])   #起点
    goal=Node(workstations[goal].x,workstations[goal].y)   #终点
    path = astar(start, goal, obstacles,R)    #查找路径
    if path:   #找到路径则返回
        return path
    else:
        return None  #未找到路径范围None
    
# 示例运行

# with open('4.txt', 'r') as file:
#     data = file.readlines()
#     game_map = []
#     for line in data:
#         game_map.append(list(line))
# obstacles=[]        
# mapp=np.full((len(game_map), len(game_map)), 0)
# for i in range(len(game_map)):
#     for j in range(len(game_map)):
#         if game_map[i][j]=='#':
#             mapp[i][j]=1
#             obstacle=Node(0.25+(j-1)*0.5,0.25+(100-i-1)*0.5)
#             obstacles.append(obstacle)
    
# start = Node(0.75, 48.75)
# goal = Node(49.25,0.75)
# # obstacles = [Node(10.25, 10.25),Node(10.25, 15.25),Node(10.25, 20.25),Node(10.25, 25.25),Node(10.25, 5.25),Node(10.25, 4.25),
#              # Node(22.25, 47.25),Node(22.25, 43.25),Node(22.25, 39.25),Node(22.25, 35.25),Node(22.25, 31.25),Node(22.25, 27.25)]
# path = astar(start, goal, obstacles)
# if path:
#     path_finall=np.full((len(path), 2), 0.0)
    
#     i=0
#     for node in path:
#         print(node.x, node.y)
#         path_finall[i][0]=node.x
#         path_finall[i][1]=node.y
#         i+=1
            
            
    
       
    
#     fig, ax = plt.subplots()
#     ax.set_xlim(0, MAP_SIZE[0])
#     ax.set_ylim(0, MAP_SIZE[1])   
    

#     plt.plot(path_finall[:, 0], path_finall[:, 1],linewidth=4)
    
    
#     for obstacle in obstacles:
#         # plt.scatter(obstacle.x, obstacle.y, color='red')
#         plt.gca().add_patch(Rectangle((obstacle.x-0.25, obstacle.y-0.25), 0.5, 0.5, color='red'))
    

#     plt.show()

# else:
#     print("No path found.")