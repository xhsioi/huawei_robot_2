import numpy as np
import sys
import math


def robot_control(order, _id, value=None):  # 定义了一个名为robot_control的函数，输入指令，机器人id ，参数即可操控机器人
    if value is not None:
        sys.stdout.write("{} {} {}\n".format(order, _id, value))
    else:
        sys.stdout.write("{} {}\n".format(order, _id))


def control_to_goal(Single_robot, path, bot0_status):
    target1 = np.array([path[bot0_status][0], path[bot0_status][1]])
    # print(target1, file=sys.stderr)
    # print(Single_robot.pos, file=sys.stderr)
    direction1 = (target1 - Single_robot.pos) / np.linalg.norm(target1 - Single_robot.pos)  # 计算机器人到目标点的方向向量
    direction_right1 = math.atan2(direction1[1], direction1[0])  # 机器人到目标点的夹角
    ori = Single_robot.toward  # 机器人当前朝向
    dis = np.linalg.norm(Single_robot.pos - target1)  # 计算机器人到目标点的距离
    # print(Single_robot.pos, file=sys.stderr)
    # print(target1, file=sys.stderr)
    err = direction_right1 - ori  # 计算机器人到目标点的夹角和机器人当前朝向的误差
    while err > math.pi:
        err -= 2.0 * math.pi
    while err < -math.pi:
        err += 2.0 * math.pi
    print(bot0_status, file=sys.stderr)
    print(err, file=sys.stderr)

    if dis < 0.5:
        robot_control("rotate", Single_robot.id - 1, 0)
        robot_control("forward", Single_robot.id - 1, 0)
        bot0_status = bot0_status + 1
    else:
        if np.fabs(err) > np.pi / 32:
            robot_control("rotate", Single_robot.id - 1, err * 4)
            robot_control("forward", Single_robot.id - 1, 1.5)
        else:
            robot_control("forward", Single_robot.id - 1, 4)
            robot_control("rotate", Single_robot.id - 1, 0)
    return bot0_status
