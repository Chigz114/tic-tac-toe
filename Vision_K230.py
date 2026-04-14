# 导入必要的模块：时间、数学、操作系统、垃圾回收、系统
import time, math, os, gc, sys

# 导入媒体相关模块：传感器、显示、媒体管理
from media.sensor import *
from media.display import *
from media.media import *

# 导入串口通信库
from ybUtils.YbUart import YbUart

# ROI : X0 = 20, X1 = 450, Y0 = 160, Y1 = 400


# ------------------- 辅助函数 -------------------
def calculate_distance(p1, p2):
    """计算两个点字典之间的距离"""
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def get_line_angle(line):
    """计算直线的角度，并归一化到 [0, 180) 度。"""
    angle = math.degrees(math.atan2(line.y2() - line.y1(), line.x2() - line.x1()))
    return angle % 180

def find_intersection(line1, line2):
    """
    计算两条直线的交点。
    使用线性方程组 Ax + By = C 的形式求解。
    返回交点坐标 (x, y)，如果两线平行则返回 None。
    """
    x1, y1, x2, y2 = line1.line()
    x3, y3, x4, y4 = line2.line()

    # Line1 的方程: a1*x + b1*y = c1
    a1 = y2 - y1
    b1 = x1 - x2
    c1 = a1 * x1 + b1 * y1

    # Line2 的方程: a2*x + b2*y = c2
    a2 = y4 - y3
    b2 = x3 - x4
    c2 = a2 * x3 + b2 * y3

    determinant = a1 * b2 - a2 * b1

    if determinant == 0:
        # 两条线平行或共线
        return None
    else:
        # 使用克莱姆法则求解
        intersect_x = (b2 * c1 - b1 * c2) / determinant
        intersect_y = (a1 * c2 - a2 * c1) / determinant
        return (intersect_x, intersect_y)

def combinations(iterable, r):
    """
    一个迭代式的 combinations 实现，功能与 itertools.combinations 相同。
    在内存受限的 MicroPython 环境中表现良好。

    用法:
    >>> list(combinations('ABC', 2))
    [('A', 'B'), ('A', 'C'), ('B', 'C')]
    >>> list(combinations(range(4), 3))
    [(0, 1, 2), (0, 1, 3), (0, 2, 3), (1, 2, 3)]
    """
    # 将可迭代对象转换为元组，方便索引访问
    pool = tuple(iterable)
    n = len(pool)

    # 如果 r 大于可迭代对象的长度，则不可能有任何组合
    if r > n:
        return

    # 初始化索引列表，例如 r=3 时为 [0, 1, 2]
    indices = list(range(r))

    # 第一次立即生成由初始索引构成的元组
    # 使用 tuple() 和生成器表达式来创建结果，非常高效
    yield tuple(pool[i] for i in indices)

    # 循环以查找下一个组合
    while True:
        # 从右向左查找第一个可以增加的索引
        # 使用 for...else 结构，如果循环正常结束（未 break），则执行 else
        for i in reversed(range(r)):
            # 检查当前索引是否已达到其最大可能值
            # 第 i 个索引的最大值是 (n - r + i)
            if indices[i] != i + n - r:
                break  # 找到了可以增加的索引，跳出循环
        else:
            # 如果 for 循环完成而没有 break，意味着所有索引都达到了最大值
            # 例如，对于 combinations(range(5), 3)，最后一个是 [2, 3, 4]
            # 这时，所有组合都已生成，结束生成器
            return

        # 将找到的索引加 1
        indices[i] += 1

        # 将该索引右边的所有索引重置为其可能的最小值
        # 例如，如果 indices 从 [0, 2, 4] 变为 [0, 3, ?]
        # 那么问号处的值需要重置为 3 + 1 = 4，所以 indices 变为 [0, 3, 4]
        for j in range(i + 1, r):
            indices[j] = indices[j - 1] + 1

        # 生成新的组合
        yield tuple(pool[i] for i in indices)

# --- 主算法 ---

def find_largest_rectangle(lines):
    """
    从一个包含两组近似正交直线的列表中，找到由这些直线构成的最大矩形。

    Args:
        lines: 一个包含 line 对象的列表。

    Returns:
        一个包含最大矩形四个顶点坐标 (x, y) 的列表。如果无法构成矩形，则返回空列表。
    """
    if len(lines) < 4:
        return []

    # 1. 根据角度将直线分为两组
    group1 = []
    group2 = []
    angle_tolerance_deg = 15  # 角度容差，可根据实际情况调整

    # 以第一条线为基准
    ref_angle = get_line_angle(lines[0])
    perp_angle = (ref_angle + 90) % 180

    for line in lines:
        angle = get_line_angle(line)
        # 计算与基准角度的最小夹角（处理 0/180 度环绕问题）
        diff_ref = abs(angle - ref_angle)
        diff_ref = min(diff_ref, 180 - diff_ref)

        # 计算与垂直角度的最小夹角
        diff_perp = abs(angle - perp_angle)
        diff_perp = min(diff_perp, 180 - diff_perp)

        if diff_ref < diff_perp and diff_ref <= angle_tolerance_deg:
            group1.append(line)
        elif diff_perp <= angle_tolerance_deg:
            group2.append(line)

    # 2. 检查是否能构成矩形
    if len(group1) < 2 or len(group2) < 2:
        # print("无法找到两组正交的直线来构成矩形。")
        return []

    max_area = 0
    largest_rectangle_vertices = []

    # 3. 遍历所有可能的四条线组合
    for h_line1, h_line2 in combinations(group1, 2):
        for v_line1, v_line2 in combinations(group2, 2):

            # 4. 计算四个交点
            p1 = find_intersection(h_line1, v_line1)
            p2 = find_intersection(h_line1, v_line2)
            p3 = find_intersection(h_line2, v_line1)
            p4 = find_intersection(h_line2, v_line2)

            # 确保所有交点都存在
            if None in [p1, p2, p3, p4]:
                continue

            # 5. 计算四边形面积
            # 使用向量叉乘计算平行四边形面积
            # A = |(P2 - P1) x (P3 - P1)|
            vec_a = (p2[0] - p1[0], p2[1] - p1[1])  # p1 -> p2
            vec_b = (p3[0] - p1[0], p3[1] - p1[1])  # p1 -> p3

            area = abs(vec_a[0] * vec_b[1] - vec_a[1] * vec_b[0])

            # 6. 更新最大面积和顶点
            if area > max_area:
                max_area = area
                vertices = [p1, p2, p3, p4]
                # 对顶点进行排序，以确保输出顺序一致
                largest_rectangle_vertices = sorted(vertices)

    return largest_rectangle_vertices

try:
    # ------------------- 初始化 -------------------
    WIDTH = 640
    HEIGHT = 480
    sensor = None
    sensor = Sensor(width = WIDTH, height = HEIGHT, fps=30)
    sensor.reset()
    sensor.set_framesize(width = WIDTH, height = HEIGHT)
    sensor.set_pixformat(sensor.RGB565)
    Display.init(Display.ST7701, width = WIDTH, height = HEIGHT, to_ide = True)
    MediaManager.init()
    sensor.run()
    uart = YbUart(baudrate=115200)

    # ------------------- 参数 -------------------
    C_PARAMS = {
        "threshold": 2400,
        "x_margin": 30, "y_margin": 30, "r_margin": 30,
        "r_min": 5, "r_max": 20, "r_step": 2,
        "roi": (20, 150, 600, 300)
    }

    # 平均灰度峰值偏移，用于构造低/高阈值
    THRESH_DELTA_L = 15
    THRESH_DELTA_H = 30

    # 标注颜色
    COLOR_BLACK = (0,   0, 255)  # “黑”圆→蓝圈
    COLOR_WHITE = (255,255,   0) # “白”圆→黄圈


    # ------------------- 主循环 -------------------
    while True:
        gc.collect()
        os.exitpoint()

        while not uart.read():
            gc.collect()
            os.exitpoint()

        loop_cnt = 10
        all_circles = []
        while loop_cnt > 0:
            img = sensor.snapshot()

            # ---- 预处理：灰度 + 中值滤波 ----
            gray = img.to_grayscale()  # 新图
            # gray.median(1)
            gray.laplacian(1, sharpen=True)

            # ---- 计算全图平均灰度，并构造双阈值 ----
            stats = gray.get_statistics()
            mean_l = stats.l_mean()        # 全图平均亮度
            lowT   = mean_l - THRESH_DELTA_L
            # lowT   = mean_l
            highT  = mean_l + THRESH_DELTA_H
            # highT  = mean_l

            # ---- Hough 圆检测 & 分类 ----
            circles = gray.find_circles(**C_PARAMS)

            for c in circles:
                x, y, r = c.x(), c.y(), c.r()
                sum_l, cnt = 0, 0

                # 遍历圆内像素，累加灰度
                for dx in range(-r, r+1):
                    xs = x + dx
                    if xs < 0 or xs >= gray.width(): continue
                    half_h = int(math.sqrt(r*r - dx*dx))
                    for dy in range(-half_h, half_h+1):
                        ys = y + dy
                        if ys < 0 or ys >= gray.height(): continue
                        sum_l += gray.get_pixel(xs, ys)
                        cnt   += 1

                if cnt == 0:
                    continue
                avg_l = sum_l // cnt

                # 双阈值分类：低于 lowT → 黑, 高于 highT → 白, 否则忽略
                if   avg_l < lowT:
                    # color = COLOR_BLACK
                    all_circles.append([c, 'black'])
                elif avg_l > highT:
                    # color = COLOR_WHITE
                    all_circles.append([c, 'white'])
                else:
                    continue

                # 在原 RGB 图上画彩色圆
                # img.draw_circle(x, y, r, color=color, thickness=2)

            # 显示到 LCD
            # Display.show_image(img)

            loop_cnt -= 1

        # print(all_circles)

        # 步骤一：聚类（分组）
        # groups是一个列表，其中每个元素是另一个列表，代表一个合并后的圆形组
        groups = []
        for my_circle in all_circles:
            point, color = my_circle[0], my_circle[1]

            # 查找当前圆形属于哪些已有的组
            matching_groups_indices = []
            for i, group in enumerate(groups):
                # 规则1: 颜色必须相同
                if group[0][1] != color:
                    continue

                # 规则2: 与组内任一成员的距离小于20
                for member in group:
                    if calculate_distance(point, member[0]) < 20:
                        matching_groups_indices.append(i)
                        break  # 找到一个匹配就够了，继续检查下一个组

            # 根据找到的匹配组数量，进行处理
            if len(matching_groups_indices) == 0:
                # 如果没有找到任何匹配的组，则创建一个新组
                groups.append([my_circle])
            elif len(matching_groups_indices) == 1:
                # 如果只匹配一个组，则将当前圆形加入该组
                groups[matching_groups_indices[0]].append(my_circle)
            else:
                # 如果匹配多个组（说明此圆形是连接它们的“桥梁”），则将这些组和当前圆形合并
                # 从后往前遍历索引，以避免在删除时破坏索引
                merged_group = [my_circle]
                for i in sorted(matching_groups_indices, reverse=True):
                    merged_group.extend(groups.pop(i))
                groups.append(merged_group)

        # 步骤二：计算平均值并生成最终结果
        final_circles = []
        for group in groups:
            # 一个组内的所有圆形颜色都相同
            group_color = group[0][1]

            # 计算坐标总和
            sum_x = sum(member[0][0] for member in group)
            sum_y = sum(member[0][1] for member in group)
            sum_r = sum(member[0][2] for member in group)
            count = len(group)

            # 计算平均坐标并四舍五入为整数
            avg_x = round(sum_x / count)
            avg_y = round(sum_y / count)
            avg_r = round(sum_r / count)

            # 构建最终输出格式的圆形
            final_circles.append([{"x": avg_x, "y": avg_y}, group_color])
            uart.send(str(avg_x)+' '+str(avg_y)+' '+group_color+'\n')

            # 绘制圆形
            if group_color == 'white':
                disp_color = COLOR_WHITE
            else:
                disp_color = COLOR_BLACK
            # img.draw_circle(avg_x, avg_y, avg_r, color=disp_color, thickness=2)

        # Display.show_image(img)
        print(final_circles)
        # uart.send('\n')

        # 矩形检测
        new_img = sensor.snapshot()
        input_lines = new_img.find_lines(roi = (20, 150, 600, 220), threshold = 1000, theta_margin = 25, rho_margin = 25)
        # for l in input_lines:
        #     new_img.draw_line(l.x1(), l.y1(), l.x2(), l.y2())
        largest_rect = find_largest_rectangle(input_lines)
        if largest_rect:
            # print("\n找到的最大矩形的四个顶点坐标为:")
            for vertex in largest_rect:
                # 格式化输出，保留两位小数
                print(f"  ({vertex[0]:.2f}, {vertex[1]:.2f})")
                uart.send(str(int(vertex[0]))+' '+str(int(vertex[1]))+'\n')
                new_img.draw_cross(int(vertex[0]), int(vertex[1]), color=(255,0,0),thickness=2)

        uart.send('\n')
        Display.show_image(new_img)


except KeyboardInterrupt as e:
    # 捕获键盘中断异常（用户手动停止）
    print(f"user stop")
except BaseException as e:
    # 捕获所有其他异常
    print(f"Exception '{e}'")
finally:
    # 无论如何都执行清理工作
    # 停止传感器运行（如果传感器对象存在）
    if isinstance(sensor, Sensor):
        sensor.stop()
    # 反初始化显示
    Display.deinit()
    # 设置退出点，允许进入睡眠模式
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    # 短暂延时100毫秒
    time.sleep_ms(100)
    # 释放媒体缓冲区
    MediaManager.deinit()
