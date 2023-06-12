import numpy as np
import matplotlib.pyplot as plt
#import pycarmaker

# 规划车辆变道轨迹的函数
def plan_trajectory(t, v, lane_width=3.5, a_lat_max=4):
    # 设置初始条件（x0, y0, yaw0, v0）和终止条件（xf, yf, yawf, vf）
    x0, y0, yaw0, v0 = 0, 0, 0, v
    xf, yf, yawf, vf = v*t, lane_width, 0, v

    # 计算侧向曲率的最大值，通过侧向加速度公式 a_lat = v^2 / r
    max_curvature = a_lat_max / (v**2)

    # 设定五次多项式路径的边界条件
    # 对于五次多项式 y(t) = a*t^5 + b*t^4 + c*t^3 + d*t^2 + e*t + f
    # 我们有以下边界条件：
    # 1. 起始时刻，y(0) = y0
    # 2. 终止时刻，y(t) = yf
    # 3. 起始时刻，轨迹的一阶导数 y'(0) = 0，表示横向速度为0
    # 4. 终止时刻，轨迹的一阶导数 y'(t) = 0，表示横向速度为0
    # 5. 起始时刻，轨迹的二阶导数 y''(0) = 0，表示起点曲率为0
    # 6. 终止时刻，轨迹的二阶导数 y''(t) = 0，表示终点曲率为0
    A = np.array([
        [0, 0, 0, 0, 0, 1],           # 边界条件1
        [t**5, t**4, t**3, t**2, t, 1],     # 边界条件2
        [0, 0, 0, 0, 1, 0],           # 边界条件3
        [5*t**4, 4*t**3, 3*t**2, 2*t, 1, 0],     # 边界条件4
        [0, 0, 0, 2, 0, 0],           # 边界条件5
        [20*t**3, 12*t**2, 6*t, 2, 0, 0]     # 边界条件6
    ])
    B = np.array([y0, yf, 0, 0, 0, 0])

    # 求解线性方程组，得到五次多项式系数
    coeffs = np.linalg.solve(A, B)

    return coeffs, max_curvature


def generate_path(coeffs, t, v, num_points=100):
    """
    time_steps = np.linspace(0, t, num_points)  # 生成时间步长
    x = v * time_steps  # x坐标是时间乘以速度，因为我们假设纵向速度恒定
    y = np.polyval(coeffs, time_steps)  # 计算y坐标，通过将时间值代入五次多项式

    # 计算曲率
    dy = np.polyval(np.polyder(coeffs, 1), time_steps)  # 计算一阶导数
    ddy = np.polyval(np.polyder(coeffs, 2), time_steps)  # 计算二阶导数
    #curvature = np.abs(ddy) / (1 + dy ** 2) ** (3 / 2)  # 计算曲率
    curvature = ddy / (1 + dy ** 2) ** (3 / 2)  # 计算曲率

    return x, y, curvature
    """
    time_steps = np.linspace(0, t, num_points)  # 生成时间步长
    x = v * time_steps  # x坐标是时间乘以速度，因为我们假设纵向速度恒定
    y = np.polyval(coeffs, time_steps)  # 计算y坐标，通过将时间值代入五次多项式

    # 计算y相对于t的一阶导数和二阶导数
    dy_dt = np.polyval(np.polyder(coeffs, 1), time_steps)  # 计算一阶导数
    ddy_dt = np.polyval(np.polyder(coeffs, 2), time_steps)  # 计算二阶导数

    # 计算y相对于x的一阶导数和二阶导数
    dx_dt = v
    dy_dx = dy_dt / dx_dt
    ddy_dx = ddy_dt / (dx_dt ** 2)

    # 计算曲率
    curvature = ddy_dx / (1 + dy_dx ** 2) ** (3 / 2)  # 计算曲率
    #curvature = v * ddy_dt / (v**2 + dy_dt) ** (3 / 2) # 参数方程求导法计算曲率

    return x, y, curvature


def visualize_path(x, y):
    plt.figure()  # 创建一个新的图形
    plt.plot(x, y, label='Planned trajectory')  # 绘制轨迹
    plt.xlabel('x (m)')  # 设置x轴标签
    plt.ylabel('y (m)')  # 设置y轴标签
    plt.legend()  # 添加图例
    plt.grid()  # 添加网格线
    plt.show()  # 显示图形

def main():
    v = 22.22  # 起点和终点的速度 (m/s)
    # v = 13.89  # 起点和终点的速度 (m/s)
    t = 4  # 变道完成所用的时间 (s)
    lane_width = 1  # 车道宽度 (m)
    a_lat_max = 1

    # 调用轨迹规划函数，输入时间、速度和车道宽度，返回五次多项式系数
    coeffs, max_curvature = plan_trajectory(t, v, lane_width, a_lat_max)
    print(coeffs)

    # 调用轨迹生成函数，输入五次多项式系数、时间和速度，返回轨迹点和曲率
    x, y, curvature = generate_path(coeffs, t, v)
    for curv_item in curvature:
        if max_curvature < curv_item:
            print("Lat_ACC Violated")

    # 调用可视化函数，输入轨迹点，绘制并显示轨迹
    visualize_path(x, y)

    # 输出曲率
    print("Curvatures: ", curvature)
    print(max_curvature)
    print("x:\n", x)
    print("y:\n", y)
    #print(x)


if __name__ == "__main__":
    main()

