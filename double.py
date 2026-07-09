# =====================================================================
# 🏎️ OpenMV 智能赛道·多目标自适应区域探针数字识别系统（完美容错版）
# =====================================================================
import sensor, image, time, pyb
from pyb import UART

# ---------------------------------------------------------------------
# 1. 初始化摄像头与通信
# ---------------------------------------------------------------------
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)     # 160x120
sensor.skip_frames(time = 2000)

uart = UART(3, 115200)
black_threshold = 95  # 黑色灰度阈值

print("====== 🏎️ 区域探针容错系统已就绪！ ======")

last_result = None
confirm_count = 0

# ---------------------------------------------------------------------
# 🛡️ 安全区域黑色探测函数（区域联防：告别单像素误差）
# ---------------------------------------------------------------------
def check_area_black(img, cx, cy, radius=1):
    """在以(cx,cy)为中心，radius为半径的方块区域内，只要有黑点就返回True"""
    for ox in range(-radius, radius + 1):
        for oy in range(-radius, radius + 1):
            px = max(0, min(159, cx + ox))
            py = max(0, min(119, cy + oy))
            if img.get_pixel(px, py) <= black_threshold:
                return True
    return False

# ---------------------------------------------------------------------
# 2. 核心算法：单目标数字 DNA 识别
# ---------------------------------------------------------------------
def analyze_digit_frame(img, frame_blob):
    fx, fy, fw, fh = frame_blob.rect()

    # ✂️ 黄金缩进 18% 彻底剥离外框线条
    margin_w = int(fw * 0.18)
    margin_h = int(fh * 0.18)
    inner_x = max(0, fx + margin_w)
    inner_y = max(0, fy + margin_h)
    inner_w = min(160 - inner_x, fw - (margin_w * 2))
    inner_h = min(120 - inner_y, fh - (margin_h * 2))

    # 寻找纯数字黑色块
    digit_blobs = img.find_blobs([(0, black_threshold)], roi=(inner_x, inner_y, inner_w, inner_h), pixels_threshold=12, area_threshold=12, merge=True)

    if not digit_blobs:
        return None

    digit_blob = max(digit_blobs, key=lambda b: b.pixels())
    dx, dy, dw, dh = digit_blob.rect()
    img.draw_rectangle(digit_blob.rect(), color=255) # 白色框标记数字核心

    # -----------------------------------------------------------------
    # 探针坐标计算
    # -----------------------------------------------------------------
    x_mid = dx + int(dw * 0.50)
    y_top = dy + int(dh * 0.26)
    y_bot = dy + int(dh * 0.74)

    # 特征 1：中心垂直骨架密度（专杀 1）
    v_mid_pixels = 0
    for py in range(dy, dy + dh):
        if img.get_pixel(x_mid, py) <= black_threshold:
            v_mid_pixels += 1

    # 特征 2：横向切线断数扫描
    def count_h_segments(y_pos):
        segments, in_black = 0, False
        for px in range(dx, dx + dw, 1):
            if img.get_pixel(px, y_pos) <= black_threshold:
                if not in_black: segments += 1; in_black = True
            else: in_black = False
        return segments

    seg_top = count_h_segments(y_top)
    seg_bot = count_h_segments(y_bot)

    # 特征 3：大斜杠横向拦截网（2的特征）
    has_2_diagonal = False
    y_intercept = dy + int(dh * 0.72)
    for px in range(dx + int(dw * 0.15), dx + int(dw * 0.42)):
        if img.get_pixel(px, y_intercept) <= black_threshold:
            has_2_diagonal = True
            break

    # 绘制可视化阵列中心（调车用）
    img.draw_line(dx + int(dw * 0.15), y_intercept, dx + int(dw * 0.42), y_intercept, color=120, thickness=1)
    img.draw_circle(dx + int(dw * 0.22), dy + int(dh * 0.25), 2, color=150)
    img.draw_circle(dx + int(dw * 0.78), dy + int(dh * 0.86), 2, color=150)

    # -----------------------------------------------------------------
    # 升级：区域阵列决策树（radius=2 表示扫描 5x5 的小区域，容错极高）
    # -----------------------------------------------------------------
    if v_mid_pixels > int(dh * 0.72): return "1"
    elif seg_top == 2 and seg_bot == 2: return "8"
    elif seg_top == 2 and seg_bot == 1: return "4"
    elif seg_top == 1 and seg_bot == 2: return "6"
    else:
        # 使用全新的区域联防探测
        is_tl_black = check_area_black(img, dx + int(dw * 0.22), dy + int(dh * 0.25), radius=2)
        is_br_black = check_area_black(img, dx + int(dw * 0.78), dy + int(dh * 0.86), radius=2)

        if not is_br_black: return "7"   # 右下无墨，铁证为 7
        elif is_tl_black: return "5"     # 左上右下都有，为 5
        elif has_2_diagonal: return "2"  # 有斜向底盘，为 2
        else: return "3"                 # 排除后为 3

# ---------------------------------------------------------------------
# 3. 主循环：多目标过滤、反套娃与排序
# ---------------------------------------------------------------------
while(True):
    img = sensor.snapshot()
    all_blobs = img.find_blobs([(0, black_threshold)], pixels_threshold=40, area_threshold=40, merge=True)

    potential_frames = []

    # 🛡️ 粗筛
    for b in all_blobs:
        fx, fy, fw, fh = b.rect()
        if 22 <= fw <= 130 and 25 <= fh <= 130:
            aspect_ratio = fw / fh
            if 0.40 <= aspect_ratio <= 1.80:
                potential_frames.append(b)

    # 🛡️ 反套娃算法
    valid_frames = []
    for i, b1 in enumerate(potential_frames):
        x1, y1, w1, h1 = b1.rect()
        is_inside_another = False
        for j, b2 in enumerate(potential_frames):
            if i != j:
                x2, y2, w2, h2 = b2.rect()
                if x1 >= x2 and y1 >= y2 and (x1 + w1) <= (x2 + w2) and (y1 + h1) <= (y2 + h2):
                    if (w1 * h1) < (w2 * h2):
                        is_inside_another = True
                        break
        if not is_inside_another:
            valid_frames.append(b1)

    # 🔍 空间排序
    valid_frames = sorted(valid_frames, key=lambda b: b.x())

    current_frame_result = None

    # -----------------------------------------------------------------
    # 4. 多目标动态匹配
    # -----------------------------------------------------------------
    if len(valid_frames) == 1:
        frame = valid_frames[0]
        img.draw_rectangle(frame.rect(), color=180)
        res = analyze_digit_frame(img, frame)
        if res:
            current_frame_result = res
            img.draw_string(frame.x(), frame.y()-22, res, color=255, scale=2)

    elif len(valid_frames) >= 2:
        left_frame = valid_frames[0]
        right_frame = valid_frames[1]

        img.draw_rectangle(left_frame.rect(), color=180)
        img.draw_rectangle(right_frame.rect(), color=180)

        res_l = analyze_digit_frame(img, left_frame)
        res_r = analyze_digit_frame(img, right_frame)

        if res_l and res_r:
            current_frame_result = "L:" + res_l + ",R:" + res_r
            img.draw_string(left_frame.x(), left_frame.y()-22, "L:"+res_l, color=255, scale=1.5)
            img.draw_string(right_frame.x(), right_frame.y()-22, "R:"+res_r, color=255, scale=1.5)
        elif res_l: current_frame_result = res_l
        elif res_r: current_frame_result = res_r

    # 🛑 终端实时看内心世界
    print("【当前检测】:", current_frame_result, " | 独立有效外框数:", len(valid_frames))

    # -----------------------------------------------------------------
    # 5. 稳定度校验与串口发送（连续 3 帧相同触发）
    # -----------------------------------------------------------------
    if current_frame_result and current_frame_result == last_result:
        confirm_count += 1
    else:
        confirm_count = 1
        last_result = current_frame_result

    if confirm_count >= 3 and current_frame_result:
        str_output = current_frame_result + "\r\n"
        uart.write(str_output)
        pyb.LED(3).on()
        time.sleep_ms(60)
        pyb.LED(3).off()
        confirm_count = 0
        last_result = None
