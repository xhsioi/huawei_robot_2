# vision 1.0 不考虑工作台不能达到
# vision 1.1 ,考虑路径可能找不到
"""
vision 1.2 ,路径一般找到了，开始优化
"""
import math
import sys
import numpy as np
from util import *
from control import *
from Astar import *
from Dp import *


# 如果给地图就初始化地图所有路径
def load_path_information(workstations):
    None


def compute_workstation1_value1(workstation1, workstation2):
    value1 = 3000
    if workstation2.type == 4 or workstation2.type == 5 or workstation2.type == 6:
        if workstation2.material_status != 0:  # 如果是345号工作台，且材料栏差一个就可以加工物品，则价值等于workstation1生产的物品价格加上workstation2的价格一半
            value1 = workstation1.value + workstation2.value / 2
        if workstation2.material_status == 0:
            value1 = workstation1.value + workstation2.value / 4
    elif workstation2.type == 7:
        if workstation2.material_status == 0:
            value1 = workstation1.value + workstation2.value / 6
        if workstation2.material_status // 16 == 6 or workstation2.material_status // 16 == 5 or workstation2.material_status // 16 == 3:
            value1 = workstation1.value + workstation2.value / 2
        if workstation2.material_status // 16 == 1 or workstation2.material_status // 16 == 2 or workstation2.material_status // 16 == 4:
            value1 = workstation1.value + workstation2.value / 4
    elif workstation2.type == 8 or workstation2.type == 9:
        value1 = workstation1.value
    return value1


def nextmin_dis_compute(work_type, workstation2):
    flag = 0
    nextmin_dis = math.inf
    best_dis = 0
    value3 = 0
    best_j = 0
    if work_type == 4:
        for j in range(len(workstations)):
            # if workstations_lock[j][4]!=0:
            #     continue
            if ((workstations[j].type == 7 and workstations[j].material_status % 32 == 0) or
                    (workstations[j].type == 9)):
                flag = 1
                next_dis2 = np.linalg.norm(workstation2.pos - workstations[j].pos)
                if next_dis2 < nextmin_dis:
                    nextmin_dis = next_dis2
                    best_dis = next_dis2
                    best_j = j
        value3 = flag * compute_workstation1_value1(workstation2, workstations[best_j])

    if work_type == 5:
        for j in range(len(workstations)):
            # if workstations_lock[j][5]!=0:
            #     continue
            if ((workstations[j].type == 7 and workstations[j].material_status // 32 % 2 == 0) or
                    (workstations[j].type == 9)):
                flag = 1
                next_dis2 = np.linalg.norm(workstation2.pos - workstations[j].pos)
                if next_dis2 < nextmin_dis:
                    nextmin_dis = next_dis2
                    best_dis = next_dis2
                    best_j = j
        value3 = flag * compute_workstation1_value1(workstation2, workstations[best_j])

    if work_type == 6:
        for j in range(len(workstations)):
            # if workstations_lock[j][6]!=0:
            #     continue
            if ((workstations[j].type == 7 and workstations[j].material_status < 64) or
                    (workstations[j].type == 9)):
                flag = 1
                next_dis2 = np.linalg.norm(workstation2.pos - workstations[j].pos)
                if next_dis2 < nextmin_dis:
                    nextmin_dis = next_dis2
                    best_dis = next_dis2
                    best_j = j
        value3 = flag * compute_workstation1_value1(workstation2, workstations[best_j])
    if work_type == 7:
        for j in range(len(workstations)):
            # if fid>9000-700:
            #     continue
            if workstations[j].type == 8 or workstations[j].type == 9:
                flag = 1
                next_dis2 = np.linalg.norm(workstation2.pos - workstations[j].pos)
                if next_dis2 < nextmin_dis:
                    nextmin_dis = next_dis2
                    best_dis = next_dis2
                    best_j = j
        value3 = flag * compute_workstation1_value1(workstation2, workstations[best_j])
    return best_dis, flag, value3


def workstations_value(workstation1, workstation2, dis1, dis2):
    value1 = 3000
    # value2=0
    flag1 = 0
    # flag2=0
    nextmin_dis = 0
    nextmin_dis2 = 0
    work_type = workstation2.type
    value2 = 0
    if ((workstation2.product_status == 1) or (workstation2.proc_time < 150 and workstation2.proc_time >= 0)):
        value2 = workstation2.value
        nextmin_dis, flag1, _ = nextmin_dis_compute(work_type, workstation2)
    nextmin_dis2, flag2, value3 = nextmin_dis_compute(work_type, workstation2)

    value1 = compute_workstation1_value1(workstation1, workstation2)

    all_value = (
                    value1 * Vaule_weight1 if flag1 == 0 else value1 * Vaule_weight1 + value2 * flag1 * Vaule_weight1) + value3 * flag2 * Vaule_weight3
    all_dis = dis1 + dis2 + nextmin_dis * flag1 + nextmin_dis2 * flag2 * Vaule_weight3
    return all_dis / all_value


def get_path_distence(path):
    distence = 0
    if path == None:
        distence = math.inf
    else:
        for i in range(1, len(path)):
            distence += math.sqrt((path[i].x - path[i - 1].x) ** 2 + (path[i].y - path[i - 1].y) ** 2)
    return distence


def find_haveproduct_stations(Single_robot):  # 如果机器人没有携带产品，查找机器人可以购买产品的工作台编号
    """
    找到机器人可以购买商品的工作台集合
    return:
        bots_to_product:np数组，bots_to_product[0][i]==1表示机器人可以去i工作台
    """

    bots_to_product = np.full((1, len(workstations)),
                              0)  # bots_to_product 1行k列矩阵。 bots_to_product[0][i]代表机器人可以到达编号为i的工作台
    if Single_robot.carried_item_type == 0:
        for j in range(len(workstations)):
            dis = np.linalg.norm(workstations[j].pos - Single_robot.pos)
            # dis=get_path_distence(Single_robot,j,map_all_workstation_path,"path_of_noproduct")
            if workstations_lock[j][0] != 0 or islive_worksations[j][0] == 0:
                continue
            if ((workstations[j].product_status == 1) or (
                    workstations[j].proc_time <= dis / 5 * 50 and workstations[j].proc_time >= 0)):
                bots_to_product[0][j] = 1
    return bots_to_product


def find_needmaterial_stations(bots_to_product, Single_robot):  # 如果机器人购买了编号为i的工作台产品，查找可以收购编号i工作台产品的j号工作台

    """
    找到携带i工作台产品的机器人可以售卖产品的工作台集合
    return:
        product_to_material:np数组,product_to_material[i][j]==1表示机器人可以从i购买商品然后卖到j
    """
    product_to_material = np.full((len(workstations), len(workstations)),
                                  0)  # k行k列。product_to_material[i][j]代表机器人可以到i工作台购买物品，然后买到j号工作台

    for i in range(len(workstations)):  # 查找编号为i的工作台可以将产品卖到那些工作台
        if bots_to_product[0][i] == 1:
            work_type = workstations[i].type  # 编号i工作台的产品编号
            if work_type == 1:  # 如果是1号产品，则型号为4,5,9的工作台可以收购该产品
                for j in range(len(workstations)):
                    if workstations_lock[j][1] != 0 or islive_worksations[j][0] == 0:  # 如果j号工作台已经成为另一个机器人的目标，则不能成为目标
                        continue
                    if ((workstations[j].type == 5 and workstations[j].material_status % 8 == 0) or
                            (workstations[j].type == 4 and workstations[
                                j].material_status % 4 == 0) or  # 4号工作台可以收购12号产品。如果材料格没有1号产品，则可以成为目标
                            (workstations[j].type == 9)):
                        product_to_material[i][j] = 1
            if work_type == 2:
                for j in range(len(workstations)):
                    if workstations_lock[j][2] != 0 or islive_worksations[j][0] == 0:
                        continue
                    if ((workstations[j].type == 4 and workstations[j].material_status < 4) or
                            (workstations[j].type == 6 and workstations[j].material_status % 8 == 0) or
                            (workstations[j].type == 9)):
                        product_to_material[i][j] = 1
            if work_type == 3:
                for j in range(len(workstations)):
                    if workstations_lock[j][3] != 0 or islive_worksations[j][0] == 0:
                        continue
                    if ((workstations[j].type == 5 and workstations[j].material_status < 8) or
                            (workstations[j].type == 6 and workstations[j].material_status < 8) or
                            (workstations[j].type == 9)):
                        product_to_material[i][j] = 1
            if work_type == 4:
                for j in range(len(workstations)):
                    if workstations_lock[j][4] != 0 or islive_worksations[j][0] == 0:
                        continue
                    if ((workstations[j].type == 7 and workstations[j].material_status % 32 == 0) or
                            (workstations[j].type == 9)):
                        product_to_material[i][j] = 1
            if work_type == 5:
                for j in range(len(workstations)):
                    if workstations_lock[j][5] != 0 or islive_worksations[j][0] == 0:
                        continue
                    if ((workstations[j].type == 7 and workstations[j].material_status // 32 % 2 == 0) or
                            (workstations[j].type == 9)):
                        product_to_material[i][j] = 1

            if work_type == 6:
                for j in range(len(workstations)):
                    if workstations_lock[j][6] != 0 or islive_worksations[j][0] == 0:
                        continue
                    if ((workstations[j].type == 7 and workstations[j].material_status < 64) or
                            (workstations[j].type == 9)):
                        product_to_material[i][j] = 1
            if work_type == 7:
                for j in range(len(workstations)):
                    if islive_worksations[j][0] == 0:
                        continue
                    if fid > 50 * 60 * 5 - (np.linalg.norm(workstations[i].pos - Single_robot.pos) + np.linalg.norm(
                            workstations[i].pos - workstations[j].pos)) / 5 * 50:
                        continue
                    if workstations[j].type == 8 or workstations[j].type == 9:
                        product_to_material[i][j] = 1
    return product_to_material


def compute_shortest_paths(Single_robot, product_to_material, map_all_workstation_path,
                           start_paths):  # 查找机器人bot从当前坐标先到i工作台购买物品再卖到j工作台的最短距离
    """
    计算开销最小的方案，并查找其路径
    """
    bot_path = np.full((1, 2), 1)  # 最优路径
    dis = np.full((len(workstations), len(workstations)), 1.0)
    min_dis = math.inf
    for j in range(len(workstations)):
        for k in range(len(workstations)):
            if product_to_material[j][k] == 1:
                dis1 = np.linalg.norm(workstations[j].pos - Single_robot.pos)
                dis2 = np.linalg.norm(workstations[j].pos - workstations[k].pos)
                if start_paths[Single_robot.id - 1]["path_of_noproduct"] is None or \
                        map_all_workstation_path[rob_path_information[Single_robot.id - 1][4] - 1][j][
                            "path_of_noproduct"] == None:
                    real_dis1 = dis1 * 1.5
                else:
                    real_dis1 = distance_of_every_workstation[rob_path_information[Single_robot.id - 1][4] - 1][j][0]

                if map_all_workstation_path[j][k]["path_of_product"] is None:
                    real_dis2 = dis2 * 1.5
                else:
                    real_dis2 = distance_of_every_workstation[j][k][1]

                real_dis = (real_dis1 + real_dis2) / workstations[
                    j].value  # workstations_value(workstations[j],workstations[k],dis1,dis2)   #(dis1+dis2)/workstations[i].value  # （最终距离=机器人到i的距离+机器人到j的距离）/i工作台到j工作台的价值
                dis[j][k] = real_dis
                if real_dis < min_dis:
                    min_dis = real_dis
                    bot_path[0][0] = j
                    bot_path[0][1] = k

    # 没有可以去的工作台，直接返回等待
    if bot_path[0][1] == 1 and bot_path[0][0] == 1:
        return bot_path, map_all_workstation_path, start_paths

    if start_paths[Single_robot.id - 1]["path_of_noproduct"] is None:  # 如果机器人i初始位置到第一个工作台的路径不可知，即初始化路径
        """
        机器人起点到目标点没有路径
        """
        bots_stop()
        sys.stdout.write('OK\n')
        sys.stdout.flush()
        pa = get_astar_path(game_map_array, rob_startpoint[Single_robot.id - 1], bot_path[0][0], workstations,
                            Single_robot, 0)
        van = get_input_var()
        sys.stdout.write('%d\n' % fid)

        if pa is None:
            print(Single_robot.id - 1, ": No path from start location to ", bot_path[0][0], "workstation\n",
                  file=sys.stderr)
            bot_path[0][1] = 1
            bot_path[0][0] = 1
            return bot_path, map_all_workstation_path, start_paths

        # 如果找到路径
        start_paths[Single_robot.id - 1]["path_of_noproduct"] = pa

        start_point = np.zeros(2)
        start_point[0] = workstations[bot_path[0][0]].x
        start_point[1] = workstations[bot_path[0][0]].y

        bots_stop()
        sys.stdout.write('OK\n')
        sys.stdout.flush()
        pa = get_astar_path(game_map_array, start_point, bot_path[0][1], workstations, Single_robot, 1)
        van = get_input_var()
        sys.stdout.write('%d\n' % fid)

        if pa is None:
            print(Single_robot.id - 1, ": No path from", bot_path[0][0], "workstation to ", bot_path[0][1],
                  "workstation\n", file=sys.stderr)
            bot_path[0][1] = 1
            bot_path[0][0] = 1
            start_paths[Single_robot.id - 1]["path_of_noproduct"] = None
            return bot_path, map_all_workstation_path, start_paths
        map_all_workstation_path[bot_path[0][0]][bot_path[0][1]]["path_of_product"] = pa
        distance_of_every_workstation[bot_path[0][0]][bot_path[0][1]][1] = get_path_distence(pa)


    else:  # 机器人已经不是出生位置了，开始在各个工作台之间往返
        # 先找保存的路径，如果没有就A*找路径
        if map_all_workstation_path[rob_path_information[Single_robot.id - 1][4] - 1][bot_path[0][0]][
            "path_of_noproduct"] == None:

            start_point = np.zeros(2)
            start_point[0] = workstations[rob_path_information[Single_robot.id - 1][4] - 1].x
            start_point[1] = workstations[rob_path_information[Single_robot.id - 1][4] - 1].y

            bots_stop()
            sys.stdout.write('OK\n')
            sys.stdout.flush()
            pa = get_astar_path(game_map_array, start_point, bot_path[0][0], workstations, Single_robot, 0)
            van = get_input_var()
            sys.stdout.write('%d\n' % fid)

            if pa is None:
                print(Single_robot.id - 1, ": No path from", rob_path_information[Single_robot.id - 1][4] - 1,
                      "workstation to ", bot_path[0][0], "workstation\n", file=sys.stderr)
                bot_path[0][1] = 1
                bot_path[0][0] = 1
                return bot_path, map_all_workstation_path, start_paths
            map_all_workstation_path[rob_path_information[Single_robot.id - 1][4] - 1][bot_path[0][0]][
                "path_of_noproduct"] = pa
            distance_of_every_workstation[rob_path_information[Single_robot.id - 1][4] - 1][bot_path[0][0]][
                0] = get_path_distence(pa)

        if map_all_workstation_path[bot_path[0][0]][bot_path[0][1]]["path_of_product"] is None:
            start_point = np.zeros(2)

            dao1x = map_all_workstation_path[rob_path_information[Single_robot.id - 1][4] - 1][bot_path[0][0]][
                "path_of_noproduct"][-1].x
            dao1y = map_all_workstation_path[rob_path_information[Single_robot.id - 1][4] - 1][bot_path[0][0]][
                "path_of_noproduct"][-1].y
            if len(map_all_workstation_path[rob_path_information[Single_robot.id - 1][4] - 1][bot_path[0][0]][
                       "path_of_noproduct"]) >= 2:
                dao2x = map_all_workstation_path[rob_path_information[Single_robot.id - 1][4] - 1][bot_path[0][0]][
                    "path_of_noproduct"][-2].x
                dao2y = map_all_workstation_path[rob_path_information[Single_robot.id - 1][4] - 1][bot_path[0][0]][
                    "path_of_noproduct"][-2].y
                if (dao1x - dao2x) ** 2 + (dao1y - dao2y) ** 2 > 1 ** 2:
                    start_point[0] = dao1x
                    start_point[1] = dao1y
                else:
                    start_point[0] = dao2x
                    start_point[1] = dao2y
            else:
                start_point[0] = dao1x
                start_point[1] = dao1y
            # start_point[0]=workstations[bot_path[0][0]].x
            # start_point[1]=workstations[bot_path[0][0]].y

            bots_stop()
            sys.stdout.write('OK\n')
            sys.stdout.flush()
            pa = get_astar_path(game_map_array, start_point, bot_path[0][1], workstations, Single_robot, 1)

            van = get_input_var()
            sys.stdout.write('%d\n' % fid)

            if pa is None:
                print(Single_robot.id - 1, ": No path from", bot_path[0][0], "workstation to ", bot_path[0][1],
                      "workstation\n", file=sys.stderr)
                bot_path[0][1] = 1
                bot_path[0][0] = 1
                return bot_path, map_all_workstation_path, start_paths
            map_all_workstation_path[bot_path[0][0]][bot_path[0][1]]["path_of_product"] = pa
            distance_of_every_workstation[bot_path[0][0]][bot_path[0][1]][1] = get_path_distence(pa)

    return bot_path, map_all_workstation_path, start_paths  # 确定最优路径  机器人先到bot_path【0】 再到bot_path【1】


def update_path(Single_robot, map_all_workstation_path, start_paths):  # 更新路径，如果一个机器人完成了运送，确定他的下一个运送路径
    """
    更新机器人路径
    """
    bots_to_product = find_haveproduct_stations(Single_robot)
    # print("bots_to_product:\n",bots_to_product, file=sys.stderr)
    product_to_material = find_needmaterial_stations(bots_to_product, Single_robot)
    # print("product_to_material:\n",product_to_material, file=sys.stderr)
    best_path, map_all_workstation_path, start_paths = compute_shortest_paths(Single_robot, product_to_material,
                                                                              map_all_workstation_path, start_paths)
    # print("best_path:\n",best_path, file=sys.stderr)
    return best_path, map_all_workstation_path, start_paths


def is_have_map(map_type):
    if map_type == 1:
        return True
    elif map_type == 2:
        return True
    elif map_type == 3:
        return False
    elif map_type == 4:
        return False


def init_workstations_path(ishavemap):
    # if ishavemap==True:
    #     load_path_information(workstations)
    None


# 从map_all_workstation_path获取path
def from_allpath_getpath(path_Node, bot, ca):
    """
    从Node类的路径转换到np数组类的路径

    Parameters
    ----------
    path_Node : Node类列表
    bot : bot类
    
    Returns
    -------
    path : np数组，
    """
    path1 = []
    if ca == 0:
        path1 = path_Node["path_of_noproduct"]
    else:
        path1 = path_Node["path_of_product"]
    if path1 == None:
        # print("rob_path_informationz:\n",rob_path_information,file=sys.stderr)
        print("cuowu:\n", bot.id, file=sys.stderr)
        # print("cuowu:\n",path_Node["path_of_noproduct"],file=sys.stderr)
        # print("cuowu:\n",map_all_workstation_path[0][5]["path_of_product"],file=sys.stderr)
        path = np.full((1, 2), 0.0)
    else:

        path = np.full((len(path1), 2), 0.0)
        i = 0
        for node in path1:
            # print(node.x, node.y)
            path[i][0] = node.x
            path[i][1] = node.y
            i += 1
    return path


def bots_stop():
    for i in range(len(bots)):
        robot_control("forward", i, 0)
        robot_control("rotate", i, 0)


def bots_coordinate_motion(best_path, bot_status,map_all_workstation_path, start_paths):
    # def bots_coordinate_motion(map_all_workstation_path,start_paths):
    """
    控制机器人找目标、运动、更新路径等

    Parameters
    ----------
    map_all_workstation_path : 列表
        保存工作台i到工作台j之间的路径
    start_paths : 列表
        保存机器人出生点到第一个目标的路径
        
    Returns
    -------
    map_all_workstation_path : 列表
        该函数对此列表进行更新
    start_paths : 列表
        更新

    """
    for i in range(len(bots)):  # 分别控制每一个机器人运动
        if islive_robots[i][0] == 0:
            continue
        if rob_path_information[i][0] >= 0:  # 如果机器人目标工作台未完成，则继续走
            goal = rob_path_information[i][0] - 1
            rob_path_information[i][2] = goal + 1
            # 这里先给目标工作台加锁，因为路径可能太长了，加锁没必要。看情况
            workstations_lock[goal][0] = bots[i].id  # 这里的id从1到4
        elif rob_path_information[i][0] < 0 and rob_path_information[i][1] > 0:  # 如果机器人已经购买了商品，则前往计划好的工作台进行销售
            goal = rob_path_information[i][1] - 1  # 此时目标是滴二个工作台
            rob_path_information[i][2] = goal + 1
        else:  # 两个目标都完成了，机器人更新路径
            goal_station, map_all_workstation_path, start_paths = update_path(bots[i], map_all_workstation_path,
                                                                              start_paths)
            # print("goal_station:\n",goal_station,bots[i].id, file=sys.stderr)
            # print("ispathstation2station:\n",ispathstation2station, file=sys.stderr)

            # np.savetxt('output.txt', ispathstation2station,fmt='%d')
            if goal_station[0][0] == 1 and goal_station[0][1] == 1:  # 工作台个数太少，可能有机器人闲置，则原地转圈
                rob_path_information[i][0] = -99
                rob_path_information[i][1] = -99
                robot_control("forward", i, 0)
                robot_control("rotate", i, 0)
                continue
            goal = goal_station[0][0]  # goal为机器人当前的目标，如果是刚更新的路径，则当前目标一定是第一个工作台
            if workstations[goal].type < 7:
                #  如果机器人将type产品送到对应工作台销售，则其他机器人不能前往该工作台销售type
                workstations_lock[goal_station[0][1]][workstations[goal].type] = bots[i].id
            rob_path_information[i][2] = goal + 1  # rob_path_information第三个位置记录机器人当前目标
            workstations_lock[goal][0] = bots[i].id  # 如果机器人正在前往goal工作台购买商品，则其他机器人不能前往该工作台购买
            rob_path_information[i][0] = goal_station[0][0] + 1
            rob_path_information[i][1] = goal_station[0][1] + 1

        """
        开始控制机器人前往目标点
        """
        if rob_path_information[i][3] != -100 and rob_path_information[i][4] != -100:
            if rob_path_information[i][0] > 0:  # 如果第一个目标没完成，路径为上个目标到这个目标
                path = from_allpath_getpath(map_all_workstation_path[abs(rob_path_information[i][4]) - 1][goal],
                                            bots[i], 0)
            elif rob_path_information[i][0] < 0 and rob_path_information[i][1] > 0:
                path = from_allpath_getpath(map_all_workstation_path[abs(rob_path_information[i][0]) - 1][goal],
                                            bots[i], 1)  # path N行2列矩阵
            # best_path= optimize_path(path)
            # bot_status[i]=control_to_goal(bots[i],best_path,bot_status[i])   #加返回值

            best_path[i] = optimize_path(path)
            # bot_status,bot_control = control_to_goal(bot_control , bots, best_path, bot_status, i)
            bot_status = control_to_goal(game_map_array, bots, best_path, bot_status, i)

        elif rob_path_information[i][3] != -100 and rob_path_information[i][4] == -100:
            path = from_allpath_getpath(map_all_workstation_path[rob_path_information[i][3] - 1][goal], bots[i],
                                        bots[i].carried_item_type)
            # best_path= optimize_path(path)
            # bot_status[i]=control_to_goal(bots[i],best_path,bot_status[i])   #加返回值

            best_path[i] = optimize_path(path)
            # bot_status,bot_control = control_to_goal(bot_control , bots, best_path, bot_status, i)
            bot_status = control_to_goal(game_map_array, bots, best_path, bot_status, i)

        else:
            path1 = start_paths[i]["path_of_noproduct"]
            path = np.full((len(path1), 2), 0.0)
            co = 0
            for node in path1:
                path[co][0] = node.x
                path[co][1] = node.y
                co += 1
            # best_path= optimize_path(path)
            # bot_status[i]=control_to_goal(bots[i],best_path,bot_status[i])

            best_path[i] = optimize_path(path)
            # bot_status,bot_control = control_to_goal(bot_control , bots, best_path, bot_status, i)
            bot_status = control_to_goal(game_map_array, bots, best_path, bot_status, i)

    return map_all_workstation_path, start_paths


def bots_operator():  # 进行购买 出售操作 
    """
    操作机器人进行购买销售销毁操作，更新机器人一些状态
    """
    for robot in bots:
        if islive_robots[robot.id - 1][0] == 0:
            continue
        if ((robot.time_value_coe < 0.92 and robot.time_value_coe != 0) or  # 如果时间太久，直接去卖
                (robot.carried_item_type < 0.92 and robot.carried_item_type != 0)):
            rob_path_information[robot.id - 1][0] = -abs(rob_path_information[robot.id - 1][0])

        if robot.at_ws_id == rob_path_information[robot.id - 1][2] - 1:  # 当前位置是目标位置进行卖买操作
            if rob_path_information[robot.id - 1][0] == rob_path_information[robot.id - 1][2] and workstations[
                robot.at_ws_id].product_status == 1:
                robot_control("buy", robot.id - 1)
                bot_status[robot.id - 1] = 1
                rob_path_information[robot.id - 1][3] = abs(rob_path_information[robot.id - 1][0])
                rob_path_information[robot.id - 1][0] = -1 * abs(rob_path_information[robot.id - 1][0])

                rob_startpoint[robot.id - 1][0] = robot.x
                rob_startpoint[robot.id - 1][1] = robot.y
                workstations_lock[rob_path_information[robot.id - 1][2] - 1][0] = 0  # 买完之后，其他机器人可以选择本工作台
            if rob_path_information[robot.id - 1][2] == rob_path_information[robot.id - 1][1]:
                robot_control("sell", robot.id - 1)
                bot_status[robot.id - 1] = 1
                rob_path_information[robot.id - 1][1] = -1 * abs(rob_path_information[robot.id - 1][1])
                workstations_lock[rob_path_information[robot.id - 1][2] - 1][robot.carried_item_type] = 0

                rob_path_information[robot.id - 1][4] = abs(rob_path_information[robot.id - 1][1])
                rob_startpoint[robot.id - 1][0] = robot.x
                rob_startpoint[robot.id - 1][1] = robot.y


def update_workstation_value():
    for i in range(len(workstations)):
        if workstations[i].type == 7:
            if workstations[i].material_status == 2 ** 4 + 2 ** 5:
                for j in range(len(workstations)):
                    if workstations[i].type == 6:
                        workstations[i].value = (27500 - 19200) * 2
            elif workstations[i].material_status == 2 ** 4 + 2 ** 6:
                for j in range(len(workstations)):
                    if workstations[i].type == 5:
                        workstations[i].value = (25000 - 17200) * 2
            elif workstations[i].material_status == 2 ** 5 + 2 ** 6:
                for j in range(len(workstations)):
                    if workstations[i].type == 4:
                        workstations[i].value = (22500 - 15400) * 2

            elif workstations[i].material_status == 2 ** 4:
                for j in range(len(workstations)):
                    if workstations[i].type == 5:
                        workstations[i].value = (25000 - 17200) * 1.5
                    if workstations[i].type == 4:
                        workstations[i].value = (27500 - 19200) * 1.5

            elif workstations[i].material_status == 2 ** 5:
                for j in range(len(workstations)):
                    if workstations[i].type == 6:
                        workstations[i].value = (27500 - 19200) * 1.5
                    if workstations[i].type == 4:
                        workstations[i].value = (22500 - 15400) * 1.5
            elif workstations[i].material_status == 2 ** 6:
                for j in range(len(workstations)):
                    if workstations[i].type == 5:
                        workstations[i].value = (25000 - 17200) * 1.5
                    if workstations[i].type == 4:
                        workstations[i].value = (22500 - 15400) * 1.5


def init_map_path():
    """
    初始化工作台i到工作台j的路径，初始值为None
    """
    map_all_workstation_path = []
    for i in range(len(workstations)):
        every_workstations_to_other = []
        for j in range(len(workstations)):
            every_workstation_to_other = {
                "path_of_noproduct": None,
                "path_of_product": None
            }
            every_workstations_to_other.append(every_workstation_to_other)

        map_all_workstation_path.append(every_workstations_to_other)
    return map_all_workstation_path


def init_start_path():
    """
    初始化机器人出生位置到第一个目标工作台的路径为None
    return:start_path一个路径列表，初始值为None
    """
    start_paths = []
    for i in range(len(bots)):
        start_path = {
            "path_of_noproduct": None,
            "path_of_product": None
        }
        start_paths.append(start_path)
    return start_paths


if __name__ == '__main__':

    # 获取地图(100x100数组)
    game_map_array = init()

    # 获取机器人状态
    bot_status = np.zeros(4) #获取机器人的状态，表示遍历到路径上的哪一个点
    best_path = np.full(4, list) #对于每一个机器人，构建一个最优路径列表

    # 获取工作台和机器人的数目
    workstations_number, bots_number = get_numbers(game_map_array)

    # 获取没有被围死的点
    islive_worksations, live_points = set_oflivepoints(game_map_array)

    # 初始化价值系数
    Vaule_weight1 = 0.5
    Vaule_weight2 = 0.1
    Vaule_weight3 = 0.1

    # 判断是否公示地图 这里目前都是没有
    ishavemap = is_have_map(4)

    # 初始化路径信息
    init_workstations_path(ishavemap)

    # 初始化工作台锁
    workstations_lock = np.full((workstations_number, 8), 0)

    # 初始化机器人路径信息  
    rob_path_information = np.full((bots_number, 5), -100)

    # 初始化工作台距离信息  
    distance_of_every_workstation = np.full((workstations_number, workstations_number, 2), 100)

    # 结束初始化
    sys.stdout.write('OK\n')
    sys.stdout.flush()
    while True:

        # 每循环开始获取robot输入信息
        fid, money, workstations, bots = get_input_var()

        # 输出帧数，开始操作
        sys.stdout.write('%d\n' % fid)

        # 初始化相关全局变量
        if fid == 1:

            # 初始化起点路径信息
            start_paths = init_start_path()

            # 初始化工作台之间路径信息
            map_all_workstation_path = init_map_path()

            # 初始化机器人起始位置
            rob_startpoint = np.full((len(bots), 2), 0.0)
            for i in range(len(bots)):
                rob_startpoint[i][0] = bots[i].x
                rob_startpoint[i][1] = bots[i].y

            # 获取没有被围死的机器人
            islive_robots = np.full((bots_number, 1), 1)
            for i in range(bots_number):
                if not is_live(live_points, bots[i].x, bots[i].y):
                    islive_robots[i][0] = 0

            # 初始化工作台距离信息
            for i in range(len(workstations)):
                for j in range(len(workstations)):
                    distance_of_every_workstation[i][j][0] = ((workstations[i].x - workstations[j].x) ** 2 + (
                            workstations[i].y - workstations[j].y) ** 2) ** (1 / 2)
                    distance_of_every_workstation[i][j][1] = ((workstations[i].x - workstations[j].x) ** 2 + (
                            workstations[i].y - workstations[j].y) ** 2) ** (1 / 2)

            # print("map_all_workstation_path:\n",map_all_workstation_path[1][5]["path_of_noproduct"], file=sys.stderr)

        # 每一帧开始运动
        update_workstation_value()  # 更新价值，每一个种路径都有价值，此价值是动态的，需要更新

        map_all_workstation_path, start_paths = bots_coordinate_motion(best_path,
                                                                       bot_status,
                                                                       map_all_workstation_path, start_paths)
        # map_all_workstation_path,start_paths=bots_coordinate_motion(map_all_workstation_path,start_paths)  # 控制多个机器人运动，包括目标选取、路径选择
        bots_operator()  # 控制单个机器人进行购买、销售、销毁操作，并更新一些全局变量

        # 结束

        sys.stdout.write('OK\n')
        sys.stdout.flush()

        if fid == 15000:
            exit()
