import numpy as np
import sys
import math
import random
import time

bot_location = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
bot_locked = np.full(4, False)


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __sub__(self, other):
        return Point(self.x - other.x, self.y - other.y)


def cross(v1, v2):
    return v1.x * v2.y - v1.y * v2.x


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


def control_to_goal(game_map_array, bot_control, bots, path, bot0_status, index):
    Single_robot = bots[index]
    connect_set = [-1, 1]
    # 防止撞死：
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
            p2 = Point(path[index][ii][0], path[index][ii][1])
            p4 = Point(path[bots[j].id - 1][jj][0], path[bots[j].id - 1][jj][1])
            if math.fabs(cross(p1 - p2, p3 - p4)) < 1 or distance(p1, p3) < 1:
                robot_control("forward", bots[index].id - 1, connect_set[0] * 6)
                robot_control("rotate", bots[index].id - 1, connect_set[0] * np.pi * 10)
                robot_control("forward", bots[j].id - 1, connect_set[1] * 6)
                robot_control("rotate", bots[j].id - 1, connect_set[1] * np.pi * 10)
                return bot0_status, bot_control
            if intersection(p1, p3, p2, p4):
                # 求交点
                px = ((p1.x * p2.y - p1.y * p2.x) * (p3.x - p4.x) - (p1.x - p2.x) * (p3.x * p4.y - p3.y * p4.x)) / (
                        (p1.x - p2.x) * (p3.y - p4.y) - (p1.y - p2.y) * (p3.x - p4.x))
                py = ((p1.x * p2.y - p1.y * p2.x) * (p3.y - p4.y) - (p1.y - p2.y) * (p3.x * p4.y - p3.y * p4.x)) / (
                        (p1.x - p2.x) * (p3.y - p4.y) - (p1.y - p2.y) * (p3.x - p4.x))
                intersect = Point(px, py)
                # 判断交点是否在路径上
                if cross(p2 - p1, intersect - p1) >= 0 and cross(p4 - p3, intersect - p3) >= 0:
                    robot_control("forward", bots[index].id - 1, connect_set[0] * 6)
                    robot_control("rotate", bots[index].id - 1, connect_set[0] * np.pi * 10)
                    robot_control("forward", bots[j].id - 1, connect_set[1] * 6)
                    robot_control("rotate", bots[j].id - 1, connect_set[1] * np.pi * 10)
                    return bot0_status, bot_control

    # 如果不存在路径，保持静止状态
    if path[index][0][0] == 0:
        robot_control("rotate", Single_robot.id - 1, 0)
        robot_control("forward", Single_robot.id - 1, 0)
        return bot0_status, bot_control
    # 当路径上存在目标，进行直行、旋转、防撞死等操作
    else:
        bot0_status_i = int(bot0_status[index])
        target1 = np.array(
            [path[index][min(len(path[index]) - 1, bot0_status_i)][0],
             path[index][min(len(path[index]) - 1, bot0_status_i)][1]])
        direction1 = (target1 - Single_robot.pos) / np.linalg.norm(target1 - Single_robot.pos)  # 计算机器人到目标点的方向向量
        direction_right1 = math.atan2(direction1[1], direction1[0])  # 机器人到目标点的夹角
        ori = Single_robot.toward  # 机器人当前朝向
        dis = np.linalg.norm(Single_robot.pos - target1)  # 计算机器人到目标点的距离
        err = direction_right1 - ori  # 计算机器人到目标点的夹角和机器人当前朝向的误差
        while err > math.pi:
            err -= 2.0 * math.pi
        while err < -math.pi:
            err += 2.0 * math.pi

        #这里取两个随机数，用于解决撞死后的后退、旋转的问题
        random.seed(time.time())
        random_index = random.randint(1, 10)
        random_index_1 = random.randint(1, 20)
        #如果已经标记为撞死，或者接近撞死状态（与之前的位置信息进行比较）
        if bot_locked[index] or (bot_location[index][0] != 0 and bot_location[index][1] != 0 and distance(
                Point(bots[index].x, bots[index].y),
                Point(bot_location[index][0], bot_location[index][1])) < 0.001):
            bot_locked[index] = True
            #如果一个机器人进入撞死的处理状态，那么其他机器人不进行此过程，保证互斥
            for i in range(4):
                if bot_locked[i] is True:
                    bot_locked[index] = False
                    break
            if bot_locked[index]:
                #构建随机的旋转方向和直行反向，方便机器人更快地解除锁死状态
                robot_control("forward", Single_robot.id - 1, np.power(-1, random_index_1) * 6)
                robot_control("rotate", Single_robot.id - 1, np.power(-1, random_index) * Single_robot.toward * 100)
                bot_locked[index] = True
                if distance(Point(bots[index].x, bots[index].y),
                            Point(bot_location[index][0], bot_location[index][1])) > 0.25:
                    bot_locked[index] = False
                return bot0_status, bot_control
        random.seed(time.time())
        random_number = random.randint(1, 100)
        #构建随机数随机更新前一时刻的位置，防止每一帧都保存
        if random_number % 4 == 0:
            bot_location[index][0] = bots[index].x
            bot_location[index][1] = bots[index].y

        #到达预定拐点，重新锁定目标地点
        if dis < 1:
            robot_control("rotate", Single_robot.id - 1, 0)
            robot_control("forward", Single_robot.id - 1, 0)
            bot0_status[index] = bot0_status_i + 1

        if abs(err) > math.pi / 18:
            robot_control("forward", Single_robot.id - 1, 1)
            robot_control("rotate", Single_robot.id - 1, err * 2)
        else:
            r_sei = math.pi / 2 - err
            r = dis / 2 / math.cos(r_sei)
            # 防止撞在墙上不能动
            if bot0_status[index] % 3 == 0:
                xx = int(bots[index].x)
                yy = int(bots[index].y)
                direction_near = [[0, 0], [1, 0], [-1, 0], [0, -1], [1, 1], [-1, -1], [1, -1], [-1, 1], [0, 1]]
                for i in range(9):
                    x = xx + direction_near[i][0]
                    y = yy + direction_near[i][1]
                    if game_map_array[x][y] is '#':
                        if x == xx and y == yy:
                            robot_control("forward", Single_robot.id - 1, 6)
                            robot_control("rotate", Single_robot.id - 1, -6 / r)
                            return bot0_status, bot_control
                        v1 = np.array(direction_near[i])
                        v2 = np.array([path[index][min(len(path[index]) - 1, bot0_status_i)][0] - bots[index].x,
                                       path[index][min(len(path[index]) - 1, bot0_status_i)][1] - bots[index].y])
                        v1_u = v1 / np.linalg.norm(v1)
                        v2_u = v2 / np.linalg.norm(v2)
                        theta = np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))
                        if theta < np.pi / 2:
                            robot_control("forward", Single_robot.id - 1, 5)
                            robot_control("rotate", Single_robot.id - 1, -3 / r)
                            return bot0_status, bot_control
            robot_control("forward", Single_robot.id - 1, 5)
            robot_control("rotate", Single_robot.id - 1, 6 / r)

    return bot0_status, bot_control
