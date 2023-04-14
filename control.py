# -*- coding: utf-8 -*-
"""
Created on Thu Apr  6 20:47:37 2023

@author: xhsioi
"""

import numpy as np
import sys
import math

coun=25
bot_location = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
bot_locked = np.full(4, False)
location_count = np.full(4, coun)


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __sub__(self, other):
        return Point(self.x - other.x, self.y - other.y)


def cross(v1, v2):
    return v1.x * v2.y - v1.y * v2.x


def dot(v1, v2):
    return v1.x * v2.x - v1.y * v2.y


def distance(p1, p2):
    return np.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)


def robot_control(order, _id, value=None):  # 定义了一个名为robot_control的函数，输入指令，机器人id ，参数即可操控机器人
    if value is not None:
        sys.stdout.write("{} {} {}\n".format(order, _id, value))
    else:
        sys.stdout.write("{} {}\n".format(order, _id))


def intersection(p1, p2, p3, p4):
    # 进行快速排斥实验和跨立实验
    if max(p1.x, p3.x) < min(p2.x, p4.x) or max(p2.x, p4.x) < min(p1.x, p3.x) or max(p1.y, p3.y) < min(p2.y,
                                                                                                       p4.y) or max(
        p2.y, p4.y) < min(p1.y, p3.y):
        return False
    if cross(p2 - p3, p1 - p3) * cross(p1 - p3, p1 - p3) > 0 or cross(p3 - p4, p2 - p4) * cross(p1 - p4, p2 - p4) > 0:
        return False
    return True


#实现对机器人的控制，包括到达目标点、防止撞墙撞死、防止对撞撞死
def control_to_goal(game_map_array, bots, path, bot0_status, index):
    Single_robot = bots[index]
    connect_set = [-1, 1]
    # 防止撞死：
    # if len(path[index])==1:
        
    #     return bot0_status
    for j in range(4):
        if j != bots[index].id - 1 and type(path[j]) is list:
            if j > index:
                connect_set = [-1, 1]
            else:
                connect_set = [1, -1]
            # 判断两个向量是否相交：
            ii = int(bot0_status[index])
            jj = int(bot0_status[j])
            p1 = Point(bots[index].x, bots[index].y)
            p3 = Point(bots[j].x, bots[j].y)
            p2 = Point(path[index][min(len(path[index]) - 1, ii)][0], path[index][min(len(path[index]) - 1, ii)][1])
            p4 = Point(path[bots[j].id - 1][min(len(path[j]) - 1, jj)][0],
                       path[bots[j].id - 1][min(len(path[j]) - 1, jj)][1])
            # 判断两个机器人之间的距离,出现对撞后进行避让
            if distance(p1, p3) <= 1.10 and index < j:
                robot_control("forward" , bots[index].id-1 , -2)
                robot_control("rotate", bots[index].id - 1, np.pi/6)
                return bot0_status
            if intersection(p1, p3, p2, p4):
                # 求交点
                px = ((p1.x * p2.y - p1.y * p2.x) * (p3.x - p4.x) - (p1.x - p2.x) * (p3.x * p4.y - p3.y * p4.x)) / (
                        (p1.x - p2.x) * (p3.y - p4.y) - (p1.y - p2.y) * (p3.x - p4.x))
                py = ((p1.x * p2.y - p1.y * p2.x) * (p3.y - p4.y) - (p1.y - p2.y) * (p3.x * p4.y - p3.y * p4.x)) / (
                        (p1.x - p2.x) * (p3.y - p4.y) - (p1.y - p2.y) * (p3.x - p4.x))
                intersect = Point(px, py)
                # 判断交点是否在路径上
                if (math.fabs(dot(p2-p1,p4-p3)) > 0.8*distance(p2-p1 , Point(0,0))*distance(p4-p3 , Point(0,0))) and distance(p1, p3) < 2 and index < j:
                    robot_control("forward", bots[index].id - 1, -2)
                    robot_control("rotate", bots[index].id - 1, connect_set[0] * np.pi)
                    return bot0_status
                if distance(intersect, p1) < 4 and distance(intersect, p3) < 2 and index < j:
                    robot_control("forward", bots[index].id - 1, -2)
                    robot_control("rotate", bots[index].id - 1, connect_set[0] * np.pi)
                    return bot0_status

    # 如果不存在路径，保持静止状态
    if path[index][0][0] == 0:
        robot_control("rotate", Single_robot.id - 1, 0)
        robot_control("forward", Single_robot.id - 1, 0)
        return bot0_status
    # 当路径上存在目标，进行直行、旋转、防撞死等操作
    else:
        bot0_status_i = int(bot0_status[index])
        target1 = np.array(
            [path[index][min(len(path[index]) - 1, bot0_status_i)][0],
             path[index][min(len(path[index]) - 1, bot0_status_i)][1]])
        # target0 = np.array(
        #     [path[index][max(0, bot0_status_i-1)][0],
        #      path[index][max(0, bot0_status_i-1)][1]])
        direction1 = (target1 - Single_robot.pos) / np.linalg.norm(target1 - Single_robot.pos)  # 计算机器人到目标点的方向向量
        direction_right1 = math.atan2(direction1[1], direction1[0])  # 机器人到目标点的夹角
        ori = Single_robot.toward  # 机器人当前朝向
        dis = np.linalg.norm(Single_robot.pos - target1)  # 计算机器人到目标点的距离
        err = direction_right1 - ori  # 计算机器人到目标点的夹角和机器人当前朝向的误差
        while err > math.pi:
            err -= 2.0 * math.pi
        while err < -math.pi:
            err += 2.0 * math.pi
        r_sei = math.pi / 2 - err
        r = dis / 2 / math.cos(r_sei)

        # 防止撞在墙上不能动
        if location_count[index] == coun:
            bot_location[index][0] = bots[index].x
            bot_location[index][1] = bots[index].y
            location_count[index] = 0
        location_count[index] = location_count[index] + 1
        if bot_location[index][0] != 0 and bot_location[index][1] != 0 and distance(Point(bots[index].x, bots[index].y),
                                                                                    Point(bot_location[index][0],
                                                                                          bot_location[index][
                                                                                              1])) < 0.01 and \
                location_count[index] == coun:
            bot0_status[index] = max(bot0_status[index] - 1, 0)
            robot_control("forward",bots[index].id-1 , -2)
            return bot0_status

        # #如果已经标记为撞死，或者接近撞死状态（与之前的位置信息进行比较）
        # if bot_locked[index] or (bot_location[index][0] != 0 and bot_location[index][1] != 0 and distance(
        #         Point(bots[index].x, bots[index].y),
        #         Point(bot_location[index][0], bot_location[index][1])) < 0.001):
        #     bot_locked[index] = True
        #     #如果一个机器人进入撞死的处理状态，那么其他机器人不进行此过程，保证互斥
        #     for i in range(4):
        #         if bot_locked[i] is True:
        #             bot_locked[index] = False
        #             break
        #     if bot_locked[index]:
        #         #构建随机的旋转方向和直行反向，方便机器人更快地解除锁死状态
        #         robot_control("forward", Single_robot.id - 1, np.power(-1, random_index_1) * 6)
        #         robot_control("rotate", Single_robot.id - 1, np.power(-1, random_index) * Single_robot.toward * 100)
        #         bot_locked[index] = True
        #         if distance(Point(bots[index].x, bots[index].y),
        #                     Point(bot_location[index][0], bot_location[index][1])) > 0.25:
        #             bot_locked[index] = False
        #         return bot0_status
        # random.seed(time.time())
        # random_number = random.randint(1, 100)
        # #构建随机数随机更新前一时刻的位置，防止每一帧都保存
        # if random_number % 4 == 0:
        #     bot_location[index][0] = bots[index].x
        #     bot_location[index][1] = bots[index].y

        # 到达预定拐点，重新锁定目标地点
        # RR=dis/2/math.cos(math.pi/10)
        
        if dis < 0.1:
            # robot_control("rotate", Single_robot.id - 1, 0)
            # robot_control("forward", Single_robot.id - 1, 0)
            bot0_status[index] = bot0_status_i + 1

        # if 6/RR>math.pi:
        #     if abs(err) > math.pi / 18:
        #             robot_control("forward", Single_robot.id - 1, -0.0)
        #             robot_control("rotate", Single_robot.id - 1, err*5)
        #     else:                    
        #         #                 return bot0_status
        #         w=math.pi/2
        #         v=w*RR
        #         robot_control("forward", Single_robot.id - 1, v)
        #         robot_control("rotate", Single_robot.id - 1, w)
        # else:
        #     if abs(err) > math.pi / 18:
        #             robot_control("forward", Single_robot.id - 1, -0.0)
        #             robot_control("rotate", Single_robot.id - 1, err*5)
        #     else:                    
        #         #                 return bot0_status
        #         w=6/RR
        #         v=6
        #         robot_control("forward", Single_robot.id - 1, v)
        #         robot_control("rotate", Single_robot.id - 1, w)
        # if index==2:
        print("target:",target1,bots[index].id,file=sys.stderr)

        if dis<1:
            if abs(err) > math.pi / 4 :
                robot_control("forward", Single_robot.id - 1, -1)
                robot_control("rotate", Single_robot.id - 1, err*10)
            elif abs(err) > math.pi / 18:
                robot_control("forward", Single_robot.id - 1, -0.1)
                robot_control("rotate", Single_robot.id - 1, err*10)
            else:
                r_sei = math.pi / 2 - err
                r = dis / 2 / math.cos(r_sei)
                if 2/r>math.pi:
                    w=math.pi/4
                    v=w*r
                    robot_control("forward", Single_robot.id - 1, v)
                    robot_control("rotate", Single_robot.id - 1, w)
                else:                    
                    #                 return bot0_status
                    v=2
                    w=v/r
                    
                    robot_control("forward", Single_robot.id - 1, v)
                    robot_control("rotate", Single_robot.id - 1, w)
        else:
            if abs(err) > math.pi / 4:
                robot_control("forward", Single_robot.id - 1, -1)
                robot_control("rotate", Single_robot.id - 1, err*10)
            elif abs(err) > math.pi / 18:
                robot_control("forward", Single_robot.id - 1, -0.1)
                robot_control("rotate", Single_robot.id - 1, err*10)
            else:
                r_sei = math.pi / 2 - err
                r = dis / 2 / math.cos(r_sei)
                if 6/r>math.pi:
                    w=math.pi/2
                    v=w*r
                    robot_control("forward", Single_robot.id - 1, v)
                    robot_control("rotate", Single_robot.id - 1, w)
                else:                    
                    #                 return bot0_status
                    w=6/r
                    v=6
                    robot_control("forward", Single_robot.id - 1, v)
                    robot_control("rotate", Single_robot.id - 1, w)

    return bot0_status
