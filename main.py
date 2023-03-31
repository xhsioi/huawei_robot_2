import math
import sys

import numpy as np

from util import *
from control import *

bot_status = np.zeros(4)


def load_path_information(workstations):#计算sell序列和buy序列
    data = np.load('map2.npz')
    path_data = data['map2_path']
    length = len(workstations)
    for i in range(length):
        for j in range(length):
            for k in range(2):
                if k == 0:
                    flag2 = False
                    flag1 = False
                    test_0 = []
                    last_x = path_data[i][j][0][0][0]
                    last_y = path_data[i][j][0][0][1]
                    test_0.append([last_x, last_y])
                    for i in range(1, len(path_data[i][j][0])):
                        if path_data[i][j][0][i][0] != path_data[i][j][0][i-1][0]:
                            if flag2:
                                flag2 = False
                                test_0.append([path_data[i][j][0][i][0], path_data[i][j][0][i][1]])
                            flag1 = True
                        if path_data[i][j][0][i][0][1] != path_data[i][j][0][i-1][1]:
                            if flag1:
                                flag1 = False
                                test_0.append([path_data[i][j][0][i][0], path_data[i][j][0][i][1]])
                            flag2 = True
                    test_0.remove(test_0[0])
                    workstations[i].buy.append(test_0)
                else:
                    flag2 = False
                    flag1 = False
                    test_0 = []
                    last_x = path_data[i][j][0][1][0]
                    last_y = path_data[i][j][0][1][1]
                    test_0.append([last_x, last_y])
                    for i in range(1, len(path_data[i][j][1])):
                        if path_data[i][j][1][i][0] != path_data[i][j][1][i - 1][0]:
                            if flag2:
                                flag2 = False
                                test_0.append([path_data[i][j][1][i][0], path_data[i][j][1][i][1]])
                            flag1 = True
                        if path_data[i][j][1][i][0][1] != path_data[i][j][1][i - 1][1]:
                            if flag1:
                                flag1 = False
                                test_0.append([path_data[i][j][1][i][0], path_data[i][j][1][i][1]])
                            flag2 = True
                    test_0.remove(test_0[0])
                    workstations[i].sell.append(test_0)



if __name__ == '__main__':
    # 获取地图(100x100数组)
    game_map_array = init()
    #print(np.size(game_map_array), file=sys.stderr)
    ws_config = get_ws_config(game_map_array)
    wall_number, wall_config = get_wall_config(game_map_array)

    # # test
    # path_0 = np.loadtxt('matrix.txt', dtype=float, delimiter=',')
    # # print(path_0,file=sys.stderr)
    #
    # flag2 = False
    # flag1 = False
    # test_0 = []
    # last_x = path_0[0][0]
    # last_y = path_0[0][1]
    # test_0.append([last_x, last_y])
    # for i in range(1, len(path_0)):
    #     if path_0[i][0] != path_0[i-1][0]:
    #         if flag2:
    #             flag2 = False
    #             test_0.append([path_0[i][0],path_0[i][1]])
    #         flag1 = True
    #     if path_0[i][1] != path_0[i-1][1]:
    #         if flag1:
    #             flag1 = False
    #             test_0.append([path_0[i][0],path_0[i][1]])
    #         flag2 = True
    # test_0.remove(test_0[0])
    # #print(test_0,file=sys.stderr)

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
