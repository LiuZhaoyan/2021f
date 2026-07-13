import sensor, image, time
from pyb import UART, LED

# =====================================================================
# 1. 硬件初始化
# =====================================================================
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)     # 160x120
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

led = LED(2)
uart = UART(3, 115200)
uart.init(115200, bits=8, parity=None, stop=1)
print('UART3 5/6/7左侧脊梁骨校准版初始化成功')

# =====================================================================
# 2. 核心参数配置
# =====================================================================
MEASURE_MODE = True
COLOR_BLACK_THRESHOLD = [(0, 48, -20, 20, -20, 20)]

MIN_DIGIT_HEIGHT = 18
MAX_DIGIT_HEIGHT = 85
MIN_DIGIT_PIXELS = 60
MAX_DIGIT_PIXELS = 900

DIGIT_TEMPLATES = {
    "1": [140, 139, 103, 32, 28, 121, 79, 29, 51, 58, 31, 109, 84, 142, 143, 88, 28, 92, 150, 150, 150, 109, 32, 72, 150, 150, 150, 125, 32, 46, 150, 150, 150, 135, 29, 36, 150, 150, 150, 142, 29, 29, 150, 150, 150, 146, 29, 29],
    "2": [174, 91, 51, 70, 178, 179, 82, 173, 75, 72, 179, 180, 179, 179, 71, 75, 180, 180, 180, 180, 66, 77, 180, 180, 181, 181, 63, 80, 180, 180, 182, 181, 60, 83, 181, 180, 181, 181, 59, 84, 180, 179, 56, 55, 49, 50, 52, 53],
    "3": [66, 65, 78, 57, 122, 196, 194, 193, 191, 184, 60, 195, 199, 195, 192, 182, 58, 197, 199, 120, 78, 60, 186, 199, 198, 192, 190, 188, 61, 66, 199, 195, 194, 194, 173, 61, 197, 197, 195, 196, 107, 64, 63, 58, 64, 61, 64, 186],
    "4": [184, 181, 57, 49, 165, 167, 184, 172, 72, 52, 145, 169, 184, 70, 169, 63, 98, 169, 157, 61, 179, 95, 58, 170, 62, 144, 180, 131, 49, 171, 61, 148, 117, 80, 50, 51, 114, 133, 153, 169, 53, 171, 193, 190, 186, 181, 56, 166],
    "5": [81, 62, 62, 64, 67, 179, 75, 156, 207, 206, 206, 205, 72, 157, 208, 206, 205, 203, 71, 62, 63, 61, 59, 162, 207, 208, 208, 204, 139, 61, 205, 208, 207, 203, 198, 60, 205, 206, 205, 204, 78, 60, 56, 60, 60, 58, 84, 199],
    "6": [176, 60, 66, 96, 67, 152, 55, 134, 181, 182, 181, 179, 54, 181, 182, 182, 182, 179, 53, 58, 58, 58, 52, 144, 53, 180, 182, 181, 165, 51, 53, 178, 184, 183, 181, 52, 56, 102, 186, 185, 164, 52, 177, 56, 54, 55, 53, 156],
    "7": [78, 69, 63, 58, 55, 55, 188, 189, 189, 189, 146, 56, 188, 190, 191, 192, 55, 118, 190, 190, 190, 123, 53, 186, 190, 189, 187, 55, 159, 185, 188, 188, 72, 59, 190, 186, 190, 179, 54, 162, 191, 187, 193, 62, 61, 192, 193, 188],
    "8": [62, 79, 138, 87, 91, 186, 56, 189, 188, 187, 56, 180, 60, 104, 190, 184, 58, 185, 193, 135, 56, 57, 184, 182, 114, 63, 185, 174, 61, 119, 60, 184, 191, 193, 186, 57, 61, 155, 194, 194, 185, 57, 166, 62, 63, 67, 59, 146]
}

left_history = []
right_history = []

# =====================================================================
# 3. 核心辅助算法与二次拓扑拦截器
# =====================================================================
def send_to_stm32(left, right):
    packet = bytes([0xAA, 0xBB, int(left), int(right), 0xCC])
    uart.write(packet)
    led.toggle()

def extract_features(img, blob):
    margin_w = blob.w() * 0.05
    margin_h = blob.h() * 0.05
    inner_w = blob.w() - 2 * margin_w
    inner_h = blob.h() - 2 * margin_h

    w_step = inner_w / 6
    h_step = inner_h / 8
    features = []
    for r in range(8):
        for c in range(6):
            px = int(blob.x() + margin_w + c * w_step + w_step / 2)
            py = int(blob.y() + margin_h + r * h_step + h_step / 2)
            px = min(max(px, 0), img.width() - 1)
            py = min(max(py, 0), img.height() - 1)

            rgb = img.get_pixel(px, py)
            gray_val = int((rgb[0] + rgb[1] + rgb[2]) / 3)
            features.append(gray_val)
    return features

def verify_1_2_7(img, blob, thresh):
    ratio = blob.w() / blob.h()
    if ratio < 0.32:
        return 1

    def get_gray(x, y):
        if 0 <= x < img.width() and 0 <= y < img.height():
            c = img.get_pixel(x, y)
            return int((c[0] + c[1] + c[2]) / 3)
        return 255

    # 区分 2 和 7：精准采样右下底角
    br_start_x = int(blob.x() + blob.w() * 0.65)
    br_end_x = int(blob.x() + blob.w() * 0.95)
    br_start_y = int(blob.y() + blob.h() * 0.75)
    br_end_y = int(blob.y() + blob.h() * 0.95)

    br_black = 0
    br_total = 0
    for y in range(br_start_y, br_end_y):
        for x in range(br_start_x, br_end_x):
            br_total += 1
            if get_gray(x, y) < thresh:
                br_black += 1

    br_ratio = (br_black / br_total * 100) if br_total > 0 else 0
    if br_ratio < 12:
        return 7
    return 2

def verify_5_6(img, blob, thresh):
    """ 🌟 终极重构：利用【左侧中部】是否有连续脊梁骨来绝对切分 5 和 6 """
    def get_gray(x, y):
        if 0 <= x < img.width() and 0 <= y < img.height():
            c = img.get_pixel(x, y)
            return int((c[0] + c[1] + c[2]) / 3)
        return 255

    # 精准探测范围：左侧中部偏下区域（宽度的15%~38%，高度的45%~70%）
    ml_start_x = int(blob.x() + blob.w() * 0.15)
    ml_end_x = int(blob.x() + blob.w() * 0.38)
    ml_start_y = int(blob.y() + blob.h() * 0.45)
    ml_end_y = int(blob.y() + blob.h() * 0.70)

    ml_black = 0
    ml_total = 0
    for y in range(ml_start_y, ml_end_y):
        for x in range(ml_start_x, ml_end_x):
            ml_total += 1
            if get_gray(x, y) < thresh:
                ml_black += 1

    ml_ratio = (ml_black / ml_total * 100) if ml_total > 0 else 0

    # 6 拥有连贯的主干黑线，这个区域黑点比例极高（通常 > 30%）
    # 5 在这里是个完全敞开的白色缺口，黑点比例接近 0
    if ml_ratio > 25:
        return 6
    return 5

def update_and_vote(history_list, current_val, max_size=4):
    history_list.append(current_val)
    if len(history_list) > max_size:
        history_list.pop(0)
    counts = {}
    for val in history_list:
        if val != 0:
            counts[val] = counts.get(val, 0) + 1
    max_count = 0
    best_target = 0
    for val, count in counts.items():
        if count > max_count:
            max_count = count
            best_target = val
    if max_count >= 2:
        return best_target
    return 0

# =====================================================================
# 4. 主循环
# =====================================================================
clock = time.clock()
last_send_time = time.ticks_ms()
last_print_time = time.ticks_ms()

while(True):
    clock.tick()
    img = sensor.snapshot()

    blobs = img.find_blobs(COLOR_BLACK_THRESHOLD, pixels_threshold=35, area_threshold=35, merge=False)
    valid_targets = []

    for blob in blobs:
        if blob.y() < 8 or blob.y() > 112:
            continue
        if blob.w() < 5:
            continue
        if (blob.pixels() / blob.h()) < 1.6:
            continue

        aspect_ratio = blob.w() / blob.h()

        if (0.22 <= aspect_ratio <= 0.95) and \
           (MIN_DIGIT_HEIGHT <= blob.h() <= MAX_DIGIT_HEIGHT) and \
           (MIN_DIGIT_PIXELS <= blob.pixels() <= MAX_DIGIT_PIXELS):

            density = blob.pixels() / (blob.w() * blob.h())
            if density < 0.18 and blob.w() > 15:
                continue

            img.draw_rectangle(blob.rect(), color=(127, 127, 127))
            valid_targets.append(blob)

    # 从左到右排序
    valid_targets.sort(key=lambda b: b.x())

    raw_left = 0
    raw_right = 0
    should_print = MEASURE_MODE and (time.ticks_diff(time.ticks_ms(), last_print_time) > 1000)

    # 识别左侧数字
    if len(valid_targets) > 0:
        b_left = valid_targets[0]
        features_left = extract_features(img, b_left)

        thresh_left = int((min(features_left) + max(features_left)) / 2)

        if should_print:
            print("Left Digit Features: ", features_left)
            last_print_time = time.ticks_ms()

        min_dist = 999999
        match_digit = 0

        for digit, template in DIGIT_TEMPLATES.items():
            brightness_bias = (sum(features_left) - sum(template)) / 48
            dist = sum(abs((f - brightness_bias) - t) for f, t in zip(features_left, template))
            if dist < min_dist:
                min_dist = dist
                match_digit = int(digit)

        if match_digit in [1, 2, 7]:
            match_digit = verify_1_2_7(img, b_left, thresh_left)
        elif match_digit in [5, 6]:
            match_digit = verify_5_6(img, b_left, thresh_left)

        if min_dist <= 4200:
            raw_left = match_digit
            img.draw_rectangle(b_left.rect(), color=(255, 0, 0), thickness=2)
            img.draw_string(b_left.x(), b_left.y()-14, "L:"+str(raw_left), color=(255, 0, 0), scale=1.2)

    # 识别右侧数字
    if len(valid_targets) > 1:
        b_right = valid_targets[1]
        features_right = extract_features(img, b_right)

        thresh_right = int((min(features_right) + max(features_right)) / 2)

        min_dist = 999999
        match_digit = 0

        for digit, template in DIGIT_TEMPLATES.items():
            brightness_bias = (sum(features_right) - sum(template)) / 48
            dist = sum(abs((f - brightness_bias) - t) for f, t in zip(features_right, template))
            if dist < min_dist:
                min_dist = dist
                match_digit = int(digit)

        if match_digit in [1, 2, 7]:
            match_digit = verify_1_2_7(img, b_right, thresh_right)
        elif match_digit in [5, 6]:
            match_digit = verify_5_6(img, b_right, thresh_right)

        if min_dist <= 4200:
            raw_right = match_digit
            img.draw_rectangle(b_right.rect(), color=(0, 255, 0), thickness=2)
            img.draw_string(b_right.x(), b_right.y()-14, "R:"+str(raw_right), color=(0, 255, 0), scale=1.2)

    # 只有一个数字时，根据它在画面中的实际位置决定左右方向。
    if len(valid_targets) == 1 and raw_left != 0:
        if valid_targets[0].cx() >= (img.width() // 2):
            raw_right = raw_left
            raw_left = 0

    # 岔路接近过程中数字停留时间较短，直接发送当前帧结果；
    # STM32 会在停车后读取并使用最近一次有效识别结果。
    final_left = raw_left
    final_right = raw_right

    # 发送串口数据
    if time.ticks_diff(time.ticks_ms(), last_send_time) > 50:
        send_to_stm32(final_left, final_right)
        last_send_time = time.ticks_ms()
