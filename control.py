import numpy as np
import sys
import math

bot_location = np.zeros(shape=(4, 2))


def robot_control(order, _id, value=None):  # 定义了一个名为robot_control的函数，输入指令，机器人id ，参数即可操控机器人
    if value is not None:
        sys.stdout.write("{} {} {}\n".format(order, _id, value))
    else:
        sys.stdout.write("{} {}\n".format(order, _id))


def control_to_goal(Single_robot, path, bot0_status: object):
    # 防止撞死：
    # if np.linalg.norm(bot_location[Single_robot.id - 1] - np.array([Single_robot.x, Single_robot.y])) < 0.01:
    #     robot_control("forward",Single_robot.id-1,-2)
    #     return bot0_status
    bot_location[Single_robot.id - 1] = np.array([Single_robot.x, Single_robot.y])
    
    if path[0][0] == 0:
        robot_control("rotate", Single_robot.id - 1, 0)
        robot_control("forward", Single_robot.id - 1, 0)
    else:

        bot0_status = int(bot0_status)
        target1 = np.array([path[min(len(path) - 1, bot0_status)][0], path[min(len(path) - 1, bot0_status)][1]])
        direction1 = (target1 - Single_robot.pos) / np.linalg.norm(target1 - Single_robot.pos)  # 计算机器人到目标点的方向向量
        direction_right1 = math.atan2(direction1[1], direction1[0])  # 机器人到目标点的夹角
        ori = Single_robot.toward  # 机器人当前朝向
        dis = np.linalg.norm(Single_robot.pos - target1)  # 计算机器人到目标点的距离
        err = direction_right1 - ori  # 计算机器人到目标点的夹角和机器人当前朝向的误差
        while err > math.pi:
            err -= 2.0 * math.pi
        while err < -math.pi:
            err += 2.0 * math.pi

        if dis < 0.25:
            robot_control("rotate", Single_robot.id - 1, 0)
            robot_control("forward", Single_robot.id - 1, 0)
            bot0_status = bot0_status + 1

        if abs(err) > math.pi / 18:
            robot_control("forward", Single_robot.id - 1, 0.2)
            robot_control("rotate", Single_robot.id - 1, err * 2)
        else:
            r_sei = math.pi / 2 - err
            r = dis / 2 / math.cos(r_sei)
            robot_control("forward", Single_robot.id - 1, 4)

            robot_control("rotate", Single_robot.id - 1, 6 / r)

    return bot0_status