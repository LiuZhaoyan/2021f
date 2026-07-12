import sensor, image, time
from pyb import UART, LED

# =====================================================================
# 1. 硬件初始化
# =====================================================================
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE) # 灰度模式
sensor.set_framesize(sensor.QQVGA)     # 160x120 分辨率
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(False)            # 必须关闭自动增益
sensor.set_auto_whitebal(False)        # 必须关闭自动白平衡

# 初始化状态指示灯
led = LED(2)  # 绿灯

# 核心修改：采用刚才 100% 测通的官方串口 3 标准初始化方法 (P4=TX, P5=RX)
uart = UART(3, 115200)
uart.init(115200, bits=8, parity=None, stop=1)
print('UART3 初始化成功 (P4->TX, P5->RX)')

# =====================================================================
# 2. 核心参数配置
# =====================================================================
MEASURE_MODE = False
BLACK_THRESHOLD = (0, 80)

MIN_DIGIT_HEIGHT = 22
MAX_DIGIT_HEIGHT = 75
MIN_DIGIT_PIXELS = 120
MAX_DIGIT_PIXELS = 650

# 你的黄金特征数据库
DIGIT_TEMPLATES = {
    "1": [186, 75, 59, 146, 195, 196, 78, 187, 61, 136, 195, 196, 192, 193, 61, 122, 196, 196, 192, 193, 64, 108, 195, 196, 193, 193, 67, 93, 195, 196, 192, 192, 71, 78, 195, 195, 190, 192, 80, 65, 193, 193, 60, 57, 51, 53, 54, 57],
    "2": [174, 91, 51, 70, 178, 179, 82, 173, 75, 72, 179, 180, 179, 179, 71, 75, 180, 180, 180, 180, 66, 77, 180, 180, 181, 181, 63, 80, 180, 180, 182, 181, 60, 83, 181, 180, 181, 181, 59, 84, 180, 179, 56, 55, 49, 50, 52, 53],
    "3": [66, 65, 78, 57, 122, 196, 194, 193, 191, 184, 60, 195, 199, 195, 192, 182, 58, 197, 199, 120, 78, 60, 186, 199, 198, 192, 190, 188, 61, 66, 199, 195, 194, 194, 173, 61, 197, 197, 195, 196, 107, 64, 63, 58, 64, 61, 64, 186],
    "4": [184, 181, 57, 49, 165, 167, 184, 172, 72, 52, 145, 169, 184, 70, 169, 63, 98, 169, 157, 61, 179, 95, 58, 170, 62, 144, 180, 131, 49, 171, 61, 148, 117, 80, 50, 51, 114, 133, 153, 169, 53, 171, 193, 190, 186, 181, 56, 166],
    "5": [81, 62, 62, 64, 67, 179, 75, 156, 207, 206, 206, 205, 72, 157, 208, 206, 205, 203, 71, 62, 63, 61, 59, 162, 207, 208, 208, 204, 139, 61, 205, 208, 207, 203, 198, 60, 205, 206, 205, 204, 78, 60, 56, 60, 60, 58, 84, 199],
    "6": [176, 60, 66, 96, 67, 152, 55, 134, 181, 182, 181, 179, 54, 181, 182, 182, 182, 179, 53, 58, 58, 58, 52, 144, 53, 180, 182, 181, 165, 51, 53, 178, 184, 183, 181, 52, 56, 102, 186, 185, 164, 52, 177, 56, 54, 55, 53, 156],
    "7": [78, 69, 63, 58, 55, 55, 188, 189, 189, 189, 146, 56, 188, 190, 191, 192, 55, 118, 190, 190, 190, 123, 53, 186, 190, 189, 187, 55, 159, 185, 188, 188, 72, 59, 190, 186, 190, 179, 54, 162, 191, 187, 193, 62, 61, 192, 193, 188],
    "8": [62, 79, 138, 87, 91, 186, 56, 189, 188, 187, 56, 180, 60, 104, 190, 184, 58, 185, 193, 135, 56, 57, 184, 182, 114, 63, 185, 174, 61, 119, 60, 184, 191, 193, 186, 57, 61, 155, 194, 194, 185, 57, 166, 62, 63, 67, 59, 146]
}

# =====================================================================
# 4. 辅助函数与三维立体特征拦截器
# =====================================================================
def send_to_stm32(left, right):
    packet = bytes([0xAA, 0xBB, int(left), int(right), 0xCC])
    uart.write(packet)
    led.toggle()  # 每次成功送出数据，翻转一次绿灯状态，用于肉眼观测

def extract_features(img, blob):
    w_step = blob.w() / 6
    h_step = blob.h() / 8
    features = []
    for r in range(8):
        for c in range(6):
            px = int(blob.x() + c * w_step + w_step / 2)
            py = int(blob.y() + r * h_step + h_step / 2)
            px = min(max(px, 0), img.width() - 1)
            py = min(max(py, 0), img.height() - 1)
            features.append(img.get_pixel(px, py))
    return features

def verify_is_really_2(img, blob):
    tr_start_x = int(blob.x() + blob.w() * 0.5)
    tr_end_x = int(blob.x() + blob.w())
    tr_start_y = int(blob.y())
    tr_end_y = int(blob.y() + blob.h() * 0.3)

    tr_black = 0
    tr_total = 0
    for y in range(tr_start_y, tr_end_y):
        for x in range(tr_start_x, tr_end_x):
            if x < img.width() and y < img.height():
                tr_total += 1
                if img.get_pixel(x, y) < 110:
                    tr_black += 1
    tr_ratio = (tr_black / tr_total * 100) if tr_total > 0 else 0

    mid_start_y = int(blob.y() + blob.h() * 0.3)
    mid_end_y = int(blob.y() + blob.h() * 0.6)
    mid_black = 0
    mid_total = 0
    for y in range(mid_start_y, mid_end_y):
        for x in range(blob.x(), blob.x() + blob.w()):
            if x < img.width() and y < img.height():
                mid_total += 1
                if img.get_pixel(x, y) < 110:
                    mid_black += 1
    mid_ratio = (mid_black / mid_total * 100) if mid_total > 0 else 0

    bl_start_x = int(blob.x())
    bl_end_x = int(blob.x() + blob.w() * 0.4)
    bl_start_y = int(blob.y() + blob.h() * 0.6)
    bl_end_y = int(blob.y() + blob.h() * 0.95)
    bl_black = 0
    bl_total = 0
    for y in range(bl_start_y, bl_end_y):
        for x in range(bl_start_x, bl_end_x):
            if x < img.width() and y < img.height():
                bl_total += 1
                if img.get_pixel(x, y) < 110:
                    bl_black += 1
    bl_ratio = (bl_black / bl_total * 100) if bl_total > 0 else 0

    if tr_ratio > 22 and bl_ratio < 20:
        return "IS_7"
    if bl_ratio > 8 and mid_ratio > 18:
        return "IS_2"
    return "IS_1"

# =====================================================================
# 5. 主循环
# =====================================================================
clock = time.clock()

# 新增计时器变量，防止串口发得太密冲爆单片机
last_send_time = time.ticks_ms()

while(True):
    clock.tick()
    img = sensor.snapshot()

    blobs = img.find_blobs([BLACK_THRESHOLD], pixels_threshold=80, area_threshold=80, merge=True)
    valid_targets = []

    for blob in blobs:
        aspect_ratio = blob.w() / blob.h()
        if (0.12 < aspect_ratio < 1.5) and \
           (MIN_DIGIT_HEIGHT <= blob.h() <= MAX_DIGIT_HEIGHT) and \
           (MIN_DIGIT_PIXELS <= blob.pixels() <= MAX_DIGIT_PIXELS):
            img.draw_rectangle(blob.rect(), color=127)
            valid_targets.append(blob)

    valid_targets.sort(key=lambda b: b.x())

    left_result = 0
    right_result = 0

    if len(valid_targets) > 0:
        b_left = valid_targets[0]
        features_left = extract_features(img, b_left)

        min_dist = 999999
        match_digit = 0
        for digit, template in DIGIT_TEMPLATES.items():
            dist = sum(abs(f - t) for f, t in zip(features_left, template))
            if dist < min_dist:
                min_dist = dist
                match_digit = int(digit)

        if match_digit in [1, 2, 7]:
            judge = verify_is_really_2(img, b_left)
            if judge == "IS_7":
                match_digit = 7
            elif judge == "IS_2":
                match_digit = 2
            else:
                match_digit = 1

        MAX_ALLOW_DIST = 3500
        if min_dist <= MAX_ALLOW_DIST:
            left_result = match_digit
            img.draw_rectangle(b_left.rect(), color=255, thickness=2)
            img.draw_string(b_left.x(), b_left.y()-14, "L:"+str(left_result), color=255, scale=1.2)

    if len(valid_targets) > 1:
        b_right = valid_targets[1]
        features_right = extract_features(img, b_right)

        min_dist = 999999
        match_digit = 0
        for digit, template in DIGIT_TEMPLATES.items():
            dist = sum(abs(f - t) for f, t in zip(features_right, template))
            if dist < min_dist:
                min_dist = dist
                match_digit = int(digit)

        if match_digit in [1, 2, 7]:
            judge = verify_is_really_2(img, b_right)
            if judge == "IS_7":
                match_digit = 7
            elif judge == "IS_2":
                match_digit = 2
            else:
                match_digit = 1

        MAX_ALLOW_DIST = 3500
        if min_dist <= MAX_ALLOW_DIST:
            right_result = match_digit
            img.draw_rectangle(b_right.rect(), color=255, thickness=2)
            img.draw_string(b_right.x(), b_right.y()-14, "R:"+str(right_result), color=255, scale=1.2)

    # 🌟 串口限速发送逻辑优化：
    # 图像识别频率通常高达每秒几十帧，但 STM32 没必要接收得这么密。
    # 这里限制每隔 100 毫秒（每秒 10 次）向 STM32 或者 DAPLink 发送一次当前识别到的最新结果。
    if time.ticks_diff(time.ticks_ms(), last_send_time) > 100:
        send_to_stm32(left_result, right_result)
        last_send_time = time.ticks_ms()
