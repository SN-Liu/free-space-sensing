import path5
import numpy as np
import matplotlib.pyplot as plt

def plan_path(ego_v_x, max_lane_width = 3.5, plan_range = 50, num_paths = 5):
    
    #根据规划距离与自车速度计算规划时间
    t = plan_range/ego_v_x
    #每次变道的横向偏移距离
    lane_width = np.linspace(-1*max_lane_width, max_lane_width, num_paths)
    
    #单条路径输出信息
    path_info_single = {
            'ID': '',
            'x' : [],
            'y' : [],
            'curvature' : []
        }
    
    #所有路径信息
    path_info_all = [] 
    for i in range(num_paths):
        path_info_all.append(path_info_single.copy())


    for i in range(num_paths):
        # 规划车辆变道轨迹的函数
        coeffs, max_curvature = path5.plan_trajectory(t,ego_v_x,lane_width=lane_width[i])
        #生成车辆轨迹
        x,y,curvature = path5.generate_path(coeffs,t,ego_v_x)
        #判断该轨迹是否符合车辆动力学性能
        is_success = True
        for curv_item in curvature:
            if max_curvature < curv_item:
                is_success = False
        if is_success == False:
            print("Lat_ACC Violated")
            continue
        #写入信息
        path_info_all[i]['ID'] = str(i)
        path_info_all[i]['x'] = x
        path_info_all[i]['y'] = y
        path_info_all[i]['curvature'] = curvature
    
    return path_info_all

def main():
    ego_v_x = 20
    path_info_all = plan_path(ego_v_x)

    plt.figure()  # 创建一个新的图形
    for i in range(len(path_info_all)):
        plt.plot(path_info_all[i]['x'],path_info_all[i]['y'], label='path'+str(i))
    plt.xlabel('x (m)')  # 设置x轴标签
    plt.ylabel('y (m)')  # 设置y轴标签
    plt.legend()  # 添加图例
    plt.grid()  # 添加网格线
    plt.show()  # 显示图形

if __name__ == "__main__":
    main()


