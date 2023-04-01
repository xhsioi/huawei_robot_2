import math
import sys

import numpy as np

from util import *
from control import *

#判断机器人或者工作站是否被锁
bot_status = np.zeros(4)
bot_fenced = np.zeros(4)
workstation_fenced = np.zeros(50)


#函数功能：对已有的路径进行筛点操作，得到最终的路径
#path为nx2的列表结构,path_type代表是否带有货物（0,1），radius代表筛除范围的半径
def load_path_information(path,path_type):
    length = len(path)
    if path_type == 0:
        radius = 5
    else:
        radius = 7
    last = [path[0][0],path[0][1]]
    result = [last]
    for i in range(1,length):
        dis = path[i] - last
        if np.sqrt(dis[0]**2 + dis[1]**2) < radius:
            continue
        else:
            last = path[i]
            result.append(last)
    return result


def check_xy(x,y):
    return 0 <= x < 100 and 0 <= y < 100


def dfs(grid, i, j):
    if i < 0 or i >= len(grid) or j < 0 or j >= len(grid[0]):
        return False
    if grid[i][j] == '#':
        return True
    grid[i][j] = '#'
    res = dfs(grid, i + 1, j) and dfs(grid, i - 1, j) and dfs(grid, i, j + 1) and dfs(grid, i, j - 1)
    grid[i][j] = '.'
    return res


#功能：实现获取当前工作站是否处于被锁状态，代表当前工作站的坐标，game_map代表二维地图,返回值为全新的地图
def get_locked_workstation(game_map):
    #先排除所有能进但是不能出的门，将这些门完全封死：
    direction = [[0,1],[1,0],[-1,0],[0,-1]]
    m, n = len(game_map), len(game_map[0])
    for i in range(m):
        for j in range(n):
            if game_map[i] == '#':
                for k in range(4):
                    xx = i + direction[k][0]
                    yy = j + direction[k][1]
                    if check_xy(xx,yy) and game_map[xx][yy] == '.' and check_xy(xx+direction[k][0] , yy + direction[k][1]) and game_map[xx + direction[k][0]][yy + direction[k][1]] == '#':
                        game_map[xx][yy] = '#'

    bot_index = 0
    workstation_index = 0
    for i in range(m):
        for j in range(n):
            if game_map[i][j] == 'A':
                if dfs(game_map,i,j):
                    bot_fenced[bot_index] = 1
                bot_index = bot_index + 1
            elif '0' <= game_map[i][j] <= '9':
                if dfs(game_map,i,j):
                    workstation_fenced[workstation_index] = 1
                workstation_index = workstation_index + 1
    return game_map


if __name__ == '__main__':
    # 获取地图(100x100数组)
    game_map_array = init()
    #print(np.size(game_map_array), file=sys.stderr)
    ws_config = get_ws_config(game_map_array)
    wall_number, wall_config = get_wall_config(game_map_array)

    #加载地图信息
    fid, money, workstations, bots = get_input_var()
    load_path_information(workstations)
    print(type(workstations[0].buy[1]),file = sys.stderr)

    workstations_lock = np.full((1, 1), 0)
    rob_path_information = np.full((1, 1), 0)  # 记录进货、出货的两个工作站的状态（-1：已完成操作；其他：未完成此操作）
    rob_startpoint = np.full((1, 1), 0)
    while True:
        # 获取信息
        fid, money, workstations, bots = get_input_var()
        #[0.75, 48.75], [4.75, 48.25]
        # 输出 fid
        sys.stdout.write('%d\n' % fid)

        # 初始化相关全局变量
        if fid == 1:
            # 初始化工作台锁
            workstations_lock = np.full((len(workstations), 8), 0)
            # 初始化机器人路径信息
            rob_path_information = np.full((len(bots), 5), -1)
            for i in range(len(bots)):
                rob_path_information[i][3] = bots[i].x
                rob_path_information[i][4] = bots[i].y

            # 初始化机器人起始位置
            rob_startpoint = np.full((len(bots), 2), 0.0)
            for i in range(len(bots)):
                rob_startpoint[i][0] = bots[i].x
                rob_startpoint[i][1] = bots[i].y
        #print(bot0_status,file=sys.stderr)
        #print(bots[0].pos,file=sys.stderr)

        #对机器人进行控制
        for i in range(4):
            bot_status[i] = control_to_goal(bots[i], test_0, bot_status[i]) #这里test0传递的是路径信息，当前机器人到某个点的路径信息
        # bot0_status = control_to_goal(bots[0], test_0, bot0_status)
        # 结束
        sys.stdout.write('OK\n')
        sys.stdout.flush()

        if fid == 15000:
            exit()
