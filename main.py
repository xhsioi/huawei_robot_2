from util import *

if __name__ == '__main__':
    # 获取地图(100x100数组)
    game_map_array = init()
    print(np.size(game_map_array), file=sys.stderr)
    ws_config = get_ws_config(game_map_array)
    wall_number, wall_config = get_wall_config(game_map_array)

    workstations_lock = np.full((1, 1), 0)
    rob_path_information = np.full((1, 1), 0)  # 记录进货、出货的两个工作站的状态（-1：已完成操作；其他：未完成此操作）
    rob_startpoint = np.full((1, 1), 0)
    while True:
        # 获取信息
        fid, money, workstations, bots = get_input_var()
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

        # 结束
        sys.stdout.write('OK\n')
        sys.stdout.flush()

        if fid == 9000:
            exit()
