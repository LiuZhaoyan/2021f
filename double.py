import sensor, image, time
from pyb import UART

# =====================================================================
# 1. 硬件初始化
# =====================================================================
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE) # 灰度模式
sensor.set_framesize(sensor.QQVGA)     # 160x120 分辨率
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(False)            # 必须关闭自动增益
sensor.set_auto_whitebal(False)        # 必须关闭自动白平衡

uart = UART(3, 115200, timeout_char=10)

# =====================================================================
# 2. 核心模式与参数配置（根据 image_b1ccfb.jpg 现场数据完美微调）
# =====================================================================
# 🌟【测量开关】由于你改小了数字，需要设为 True 重新录入 48 维数组模板。录完请改回 False
MEASURE_MODE = True

# 黑色数字的灰度阈值
BLACK_THRESHOLD = (0, 80)

# 🌟【物理双向大闸：高度与像素数双向夹击】
# 根据你的控制台日志：
# 缩小后的数字：高度约 39，像素数约 264
# 卡片外黑框：高度约 80，像素数约 989
# 左侧跑道黑线：高度 > 120，像素数 > 1200
# 极小杂噪：高度 < 15
MIN_DIGIT_HEIGHT = 22   # 调低高度下限，完美容纳缩小后的数字
MAX_DIGIT_HEIGHT = 75   # 🌟 设高度上限，直接拦截 80 的卡片框与 120 的跑道线！

MIN_DIGIT_PIXELS = 120  # 调低像素下限，放行缩小的数字
MAX_DIGIT_PIXELS = 650  # 🌟 设像素上限，直接拦截大块黑线和外框！

# =====================================================================
# 3. 1-8 数字特征数据库 (重新测量后请将控制台打印的新数组替换到这里)
# =====================================================================
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
# 4. 辅助函数
# =====================================================================
def send_to_stm32(left, right):
    packet = bytes([0xAA, 0xBB, int(left), int(right), 0xCC])
    uart.write(packet)

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

# =====================================================================
# 5. 主循环
# =====================================================================
clock = time.clock()

while(True):
    clock.tick()
    img = sensor.snapshot()

    # 🔍 步骤一：寻找黑色色块
    blobs = img.find_blobs([BLACK_THRESHOLD], pixels_threshold=80, area_threshold=80, merge=True)
    valid_targets = []

    for blob in blobs:
        # 📐 步骤二：增加物理大闸过滤（长宽比 + 高度双向 + 像素数双向）
        aspect_ratio = blob.w() / blob.h()

        # 🌟【拦截核心】：
        # 1. 长宽比放宽到 0.12 ~ 1.5 (数字 1 较瘦窄也能完美通过)
        # 2. 高度必须在 MIN_DIGIT_HEIGHT(22) 到 MAX_DIGIT_HEIGHT(75) 之间
        # 3. 像素数必须在 MIN_DIGIT_PIXELS(120) 到 MAX_DIGIT_PIXELS(650) 之间
        if (0.12 < aspect_ratio < 1.5) and \
           (MIN_DIGIT_HEIGHT <= blob.h() <= MAX_DIGIT_HEIGHT) and \
           (MIN_DIGIT_PIXELS <= blob.pixels() <= MAX_DIGIT_PIXELS):
            img.draw_rectangle(blob.rect(), color=127) # 绘制候选框
            valid_targets.append(blob)
        else:
            # 如果开启测量模式，可以顺便打印一下被淘汰的色块尺寸，方便你微调阈值
            if MEASURE_MODE and blob.h() > 5:
                print("【被过滤杂噪】高:%d, 像素数:%d" % (blob.h(), blob.pixels()))

    # 按从左到右(X坐标)排序
    valid_targets.sort(key=lambda b: b.x())

    left_result = 0
    right_result = 0

    # 🎯 步骤三：双通道匹配与实时测量输出
    if len(valid_targets) > 0:
        b_left = valid_targets[0]
        features_left = extract_features(img, b_left)

        if MEASURE_MODE:
            print("【测量模式】左框新特征数组 = %s" % features_left)

        min_dist = 999999
        match_digit = 0
        for digit, template in DIGIT_TEMPLATES.items():
            dist = sum(abs(f - t) for f, t in zip(features_left, template))
            if dist < min_dist:
                min_dist = dist
                match_digit = int(digit)

        if MEASURE_MODE:
            print("   -> (左) 匹配数字: %d, SAD距离: %d" % (match_digit, min_dist))

        # 录入模式下无限距离，以便观察；正常模式下卡死在合适范围
        MAX_ALLOW_DIST = 999999 if MEASURE_MODE else 3500
        if min_dist <= MAX_ALLOW_DIST:
            left_result = match_digit
            img.draw_rectangle(b_left.rect(), color=255, thickness=2)
            img.draw_string(b_left.x(), b_left.y()-14, "L:"+str(left_result), color=255, scale=1.2)

    if len(valid_targets) > 1:
        b_right = valid_targets[1]
        features_right = extract_features(img, b_right)

        if MEASURE_MODE:
            print("【测量模式】右框新特征数组 = %s" % features_right)

        min_dist = 999999
        match_digit = 0
        for digit, template in DIGIT_TEMPLATES.items():
            dist = sum(abs(f - t) for f, t in zip(features_right, template))
            if dist < min_dist:
                min_dist = dist
                match_digit = int(digit)

        if MEASURE_MODE:
            print("   -> (右) 匹配数字: %d, SAD距离: %d" % (match_digit, min_dist))

        MAX_ALLOW_DIST = 999999 if MEASURE_MODE else 3500
        if min_dist <= MAX_ALLOW_DIST:
            right_result = match_digit
            img.draw_rectangle(b_right.rect(), color=255, thickness=2)
            img.draw_string(b_right.x(), b_right.y()-14, "R:"+str(right_result), color=255, scale=1.2)

    # 6. 数据外发
    send_to_stm32(left_result, right_result)
    time.sleep_ms(30)
