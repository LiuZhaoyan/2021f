#include "medicine_car_app.h"

#include <stddef.h>

#include "medicine_car_config.h"
#include "medicine_car_platform.h"
#include "medicine_car_return.h"
#include "medicine_car_vision_ring.h"

static uint8_t route_running;

typedef enum {
    SEG_CROSS,
    SEG_FORK,
    SEG_RP2,
    SEG_FORK_FIXED,
    SEG_DOOR,
    SEG_END
} SegmentType;

typedef uint8_t (*RecognizeFn)(void);

typedef struct {
    SegmentType type;
    uint16_t distance;
    RecognizeFn recognize;
    uint8_t auto_door;
    uint8_t advance_if_straight; /* Escape the cross line when going straight. */
    uint8_t stable_precache;
} RouteSegment;

typedef struct {
    int nums[MED_CAR_RP2_MAX_COLLECTED];
    uint8_t sides[MED_CAR_RP2_MAX_COLLECTED];
    uint8_t count;
    uint8_t valid;
} Rp2RecognitionResult;

static Rp2RecognitionResult rp2_result;
static uint8_t rp2_test_logging;
static uint8_t rp2_test_turn_ok;

static void beep_matched_vision_once(void);
static void rp2_clear_result(void);

#define RP2_TEST_LOG(...)                      \
    do {                                       \
        if (rp2_test_logging != 0U) {          \
            u2_printf(__VA_ARGS__);            \
        }                                      \
    } while (0)

static uint32_t rp2_encoder_abs_sum(void)
{
    uint32_t left = (Encoder_Left < 0) ?
                    (uint32_t)(-Encoder_Left) : (uint32_t)Encoder_Left;
    uint32_t right = (Encoder_Right < 0) ?
                     (uint32_t)(-Encoder_Right) : (uint32_t)Encoder_Right;

    return left + right;
}

static void rp2_wiggle_with_log(const char *stage,
                                int left_pwm, int right_pwm)
{
    uint32_t total_ticks;

    if (rp2_test_logging == 0U) {
        wiggle_by_ticks(left_pwm, right_pwm, MED_CAR_RP2_WIGGLE_TICKS);
        return;
    }

    RP2_TEST_LOG("[RP2][WIGGLE] %s start pwm=(%d,%d) target=%u timeout=%ums\r\n",
                 stage,
                 left_pwm,
                 right_pwm,
                 (unsigned int)MED_CAR_RP2_WIGGLE_TICKS,
                 (unsigned int)MED_CAR_RP2_WIGGLE_TIMEOUT_MS);
    wiggle_by_ticks(left_pwm, right_pwm, MED_CAR_RP2_WIGGLE_TICKS);
    Read_Speed();
    total_ticks = rp2_encoder_abs_sum();
    RP2_TEST_LOG("[RP2][WIGGLE] %s %s enc=(%d,%d) total=%lu\r\n",
                 stage,
                 (total_ticks >= (uint32_t)MED_CAR_RP2_WIGGLE_TICKS) ?
                 "OK" : "TIMEOUT",
                 Encoder_Left,
                 Encoder_Right,
                 (unsigned long)total_ticks);
}

static uint8_t find_matched_side(void)
{
    if ((NumBuff[0] != 0) && (aim == NumBuff[0])) {
        return (XBuff[0] == 1) ? 1U : 2U;
    }
    if ((NumBuff[1] != 0) && (aim == NumBuff[1])) {
        return (XBuff[1] == 1) ? 1U : 2U;
    }
    return 0U;
}

static void wait_delivery_done(void)
{
    uint32_t waited = 0U;

    MedicineCar_SetRedLed(1U);
    Load(0, 0);
    while ((MedicineCar_ReadDrugPresent() != 0U) &&
           (waited < MED_CAR_DELIVERY_WAIT_TIMEOUT_MS)) {
        Load(0, 0);
        delay_ms(20U);
        waited += 20U;
    }

    MedicineCar_SetRedLed(0U);
}

static void finish_at_home(uint16_t distance, int pwm)
{
    zhao_bai(distance, pwm);
    Load(0, 0);
    MedicineCar_SetGreenLed(1U);
}

static uint8_t check_match_and_turn(void)
{
    uint8_t side = find_matched_side();
    uint8_t turn_ok;

    RP2_TEST_LOG("[RP2][MATCH] aim=%d Num=[%d,%d] X=[%d,%d] side=%u\r\n",
                 aim, NumBuff[0], NumBuff[1], XBuff[0], XBuff[1], side);

    if (side == 0U) {
        RP2_TEST_LOG("[RP2][MATCH] MISS: target was not collected\r\n");
        return 0U;
    }

    VisionRing_StableRelease();
    rp2_clear_result();
    beep_matched_vision_once();
    if (side == 1U) {
        RP2_TEST_LOG("[RP2][TURN] LEFT command\r\n");
        Return_Push(RETURN_DIR_LEFT);
        turn_ok = sensor_turn_left();
    } else {
        RP2_TEST_LOG("[RP2][TURN] RIGHT command\r\n");
        Return_Push(RETURN_DIR_RIGHT);
        turn_ok = sensor_turn_right();
    }
    RP2_TEST_LOG("[RP2][TURN] %s enc=(%d,%d)\r\n",
                 (turn_ok != 0U) ? "OK" : "TIMEOUT",
                 Encoder_Left,
                 Encoder_Right);
    rp2_test_turn_ok = turn_ok;
    return 1U;
}

#if MED_CAR_USE_SENSOR_GUIDED_TURNS
static void sensor_deliver_left(uint16_t home_distance, int pwm)
{
    sensor_turn_left();
    xunxian_until_door(MED_CAR_DISTANCE_MID, pwm);
    wait_delivery_done();
    sensor_diaotou();
    xunxian(MED_CAR_DISTANCE_MID, pwm);
    sensor_turn_right();
    finish_at_home(home_distance, 5000);
}

static void sensor_deliver_right(uint16_t home_distance, int pwm)
{
    sensor_turn_right();
    xunxian_until_door(MED_CAR_DISTANCE_MID, pwm);
    wait_delivery_done();
    sensor_diaotou();
    xunxian(MED_CAR_DISTANCE_MID, pwm);
    sensor_turn_left();
    finish_at_home(home_distance, 5000);
}
#endif

static void deliver_left(uint16_t return_cross_distance, uint16_t home_distance, int pwm)
{
#if MED_CAR_USE_SENSOR_GUIDED_TURNS
    (void)return_cross_distance;
    sensor_deliver_left(home_distance, pwm);
#else
    turn_left();
    xunxian(MED_CAR_DISTANCE_MID, pwm);
    wait_delivery_done();
    diaotou();
    xunxian(return_cross_distance, pwm);
    turn_right();
    finish_at_home(home_distance, 5000);
#endif
}

static void deliver_right(uint16_t return_cross_distance, uint16_t home_distance, int pwm)
{
#if MED_CAR_USE_SENSOR_GUIDED_TURNS
    (void)return_cross_distance;
    sensor_deliver_right(home_distance, pwm);
#else
    turn_right();
    xunxian(MED_CAR_DISTANCE_MID, pwm);
    wait_delivery_done();
    diaotou();
    xunxian(return_cross_distance, pwm);
    turn_left();
    finish_at_home(home_distance, 5000);
#endif
}

static void clear_recognition_buffers(void)
{
    NumBuff[0] = 0;
    NumBuff[1] = 0;
    XBuff[0] = 0;
    XBuff[1] = 0;
}

static void beep_matched_vision_once(void)
{
    MedicineCar_SetYellowLed(1U);
    delay_ms(MED_CAR_MATCH_BEEP_MS);
    MedicineCar_SetYellowLed(0U);
}

static uint8_t request_recognition_pair(void)
{
    VisionRingEntry entry;

    clear_recognition_buffers();
    if (VisionRing_StableRead(&entry) == 0U) {
        return 0U;
    }

    if (entry.left != 0U) {
        NumBuff[0] = (int)entry.left;
        XBuff[0] = 1;
    }
    if (entry.right != 0U) {
        if (entry.left != 0U) {
            NumBuff[1] = (int)entry.right;
            XBuff[1] = 2;
        } else {
            NumBuff[0] = (int)entry.right;
            XBuff[0] = 2;
        }
    }

    return 1U;
}

static uint8_t shibie(void)
{
    return request_recognition_pair();
}

#define FORK_LEFT   1U
#define FORK_RIGHT  2U

static void rp2_clear_result(void)
{
    uint8_t i;

    for (i = 0U; i < MED_CAR_RP2_MAX_COLLECTED; i++) {
        rp2_result.nums[i] = 0;
        rp2_result.sides[i] = 0U;
    }
    rp2_result.count = 0U;
    rp2_result.valid = 0U;
}

static void rp2_add_unique(uint8_t num, uint8_t side)
{
    uint8_t i;

    if (num == 0U) {
        return;
    }
    for (i = 0U; i < rp2_result.count; i++) {
        if (rp2_result.nums[i] == (int)num) {
            RP2_TEST_LOG("[RP2][COLLECT] duplicate num=%u ignored\r\n", num);
            return;
        }
    }
    if (rp2_result.count < MED_CAR_RP2_MAX_COLLECTED) {
        rp2_result.nums[rp2_result.count] = (int)num;
        rp2_result.sides[rp2_result.count] = side;
        rp2_result.count++;
        RP2_TEST_LOG("[RP2][COLLECT] num=%u side=%s count=%u\r\n",
                     num,
                     (side == FORK_LEFT) ? "LEFT" : "RIGHT",
                     rp2_result.count);
    }
}

static uint8_t rp2_read_fresh_entry(VisionRingEntry *entry, uint8_t flush_first)
{
    if (entry == NULL) {
        return 0U;
    }
    if (flush_first != 0U) {
        RP2_TEST_LOG("[RP2][VISION] flush and wait fresh valid frame, timeout=%ums\r\n",
                     (unsigned int)MED_CAR_RP2_SCAN_TIMEOUT_MS);
        VisionRing_Flush();
        if (VisionRing_WaitForNewEntry(MED_CAR_RP2_SCAN_TIMEOUT_MS) == 0U) {
            RP2_TEST_LOG("[RP2][VISION] TIMEOUT: no fresh non-zero frame\r\n");
            return 0U;
        }
        if (VisionRing_ReadLatest(entry) == 0U) {
            RP2_TEST_LOG("[RP2][VISION] read latest failed after wakeup\r\n");
            return 0U;
        }
        RP2_TEST_LOG("[RP2][VISION] fresh left=%u right=%u timestamp=%lums\r\n",
                     entry->left,
                     entry->right,
                     (unsigned long)entry->timestamp_ms);
        return 1U;
    }
    if (VisionRing_ReadLatest(entry) == 0U) {
        RP2_TEST_LOG("[RP2][VISION] no buffered frame\r\n");
        return 0U;
    }
    RP2_TEST_LOG("[RP2][VISION] buffered left=%u right=%u timestamp=%lums\r\n",
                 entry->left,
                 entry->right,
                 (unsigned long)entry->timestamp_ms);
    return 1U;
}

static uint8_t shibie_rp2(void)
{
    //VisionRingEntry center;  /* 已废弃：不再读中心帧 */
    VisionRingEntry outer;

    RP2_TEST_LOG("[RP2][SCAN] begin settle=%ums\r\n",
                 (unsigned int)MED_CAR_RP2_SCAN_SETTLE_MS);
    clear_recognition_buffers();
    delay_ms(MED_CAR_RP2_SCAN_SETTLE_MS);
    VisionRing_StableRelease();
    RP2_TEST_LOG("[RP2][SCAN] center stable cache released; scan outer views\r\n");

    /* 已废弃：StableRead 方式读取中心帧
    if (VisionRing_StableRead(&center) == 0U) {
        VisionRing_StableRelease();
        return 0U;
    }
    VisionRing_StableRelease();
    rp2_add_unique(center.left, FORK_LEFT);
    rp2_add_unique(center.right, FORK_RIGHT);
    */

    /* Left wiggle -> scan both pixels -> reverse back to center */
    rp2_wiggle_with_log("LEFT_SCAN",
                        MED_CAR_RP2_WIGGLE_LEFT_PWM_LEFT,
                        MED_CAR_RP2_WIGGLE_LEFT_PWM_RIGHT);
    delay_ms(MED_CAR_RP2_SCAN_SETTLE_MS);
    if (rp2_read_fresh_entry(&outer, 1U) != 0U) {
        rp2_add_unique(outer.left, FORK_LEFT);
        rp2_add_unique(outer.right, FORK_LEFT);
    }
    rp2_wiggle_with_log("LEFT_RETURN",
                        MED_CAR_RP2_WIGGLE_RIGHT_PWM_LEFT,
                        MED_CAR_RP2_WIGGLE_RIGHT_PWM_RIGHT);

    /* Right wiggle -> scan both pixels -> reverse back to center */
    rp2_wiggle_with_log("RIGHT_SCAN",
                        MED_CAR_RP2_WIGGLE_RIGHT_PWM_LEFT,
                        MED_CAR_RP2_WIGGLE_RIGHT_PWM_RIGHT);
    delay_ms(MED_CAR_RP2_SCAN_SETTLE_MS);
    if (rp2_read_fresh_entry(&outer, 1U) != 0U) {
        rp2_add_unique(outer.left, FORK_RIGHT);
        rp2_add_unique(outer.right, FORK_RIGHT);
    }
    rp2_wiggle_with_log("RIGHT_RETURN",
                        MED_CAR_RP2_WIGGLE_LEFT_PWM_LEFT,
                        MED_CAR_RP2_WIGGLE_LEFT_PWM_RIGHT);

    rp2_result.valid = (rp2_result.count > 0U) ? 1U : 0U;
    RP2_TEST_LOG("[RP2][SCAN] complete valid=%u count=%u\r\n",
                 rp2_result.valid,
                 rp2_result.count);
    return rp2_result.valid;
}

static void rp2_populate_recognition_buffers(void)
{
    uint8_t i;

    clear_recognition_buffers();
    for (i = 0U; i < rp2_result.count; i++) {
        if (rp2_result.nums[i] == aim) {
            if (rp2_result.sides[i] == FORK_LEFT) {
                NumBuff[0] = aim;
                XBuff[0] = 1;
            } else {
                NumBuff[1] = aim;
                XBuff[1] = 2;
            }
            break;
        }
    }
    for (i = 0U; i < rp2_result.count; i++) {
        if (rp2_result.nums[i] == aim) {
            continue;
        }
        if (NumBuff[0] == 0) {
            NumBuff[0] = rp2_result.nums[i];
            XBuff[0] = (rp2_result.sides[i] == FORK_LEFT) ? 1 : 2;
        } else if (NumBuff[1] == 0) {
            NumBuff[1] = rp2_result.nums[i];
            XBuff[1] = (rp2_result.sides[i] == FORK_LEFT) ? 1 : 2;
        }
    }
}

static uint8_t rp2_scan_before_fork(uint16_t max_distance, int pwm)
{
    MedicineCarTraceStopReason reason;
    uint8_t scan_ok;

    rp2_clear_result();
    clear_recognition_buffers();
    VisionRing_StableArm();
    RP2_TEST_LOG("[RP2][APPROACH] trace start max=%u pwm=%d; stop on first digit\r\n",
                 max_distance,
                 pwm);
    reason = xunxian_until_fork_or_condition(
        max_distance, pwm, VisionRing_AnyDigitSeen);
    RP2_TEST_LOG("[RP2][APPROACH] stopped reason=%s enc=(%d,%d)\r\n",
                 (reason == MED_CAR_TRACE_STOP_CONDITION) ? "DIGIT" :
                 ((reason == MED_CAR_TRACE_STOP_FORK) ? "FORK" : "LIMIT"),
                 Encoder_Left,
                 Encoder_Right);

    if (reason != MED_CAR_TRACE_STOP_CONDITION) {
        VisionRing_StableRelease();
        RP2_TEST_LOG("[RP2][APPROACH] FAIL: no digit-triggered stop\r\n");
        return 0U;
    }
    scan_ok = shibie_rp2();
    if (scan_ok == 0U) {
        rp2_clear_result();
    }
    RP2_TEST_LOG("[RP2][ADVANCE] forward %ums pwm=%d\r\n",
                 (unsigned int)MED_CAR_RP2_FORK_ADVANCE_MS,
                 pwm);
    move_forward_timed(MED_CAR_RP2_FORK_ADVANCE_MS, pwm);
    if (scan_ok == 0U) {
        clear_recognition_buffers();
        rp2_clear_result();
        return 0U;
    }

    rp2_populate_recognition_buffers();
    RP2_TEST_LOG("[RP2][BUFFER] aim=%d Num=[%d,%d] X=[%d,%d]\r\n",
                 aim, NumBuff[0], NumBuff[1], XBuff[0], XBuff[1]);
    return 1U;
}

static const RouteSegment route12_segments[] = {
    {SEG_FORK_FIXED, MED_CAR_DISTANCE_FIRST_CHECK, NULL,  1, 0, 0},
    {SEG_DOOR,       MED_CAR_DISTANCE_MID,         NULL,  0, 0, 0},
    {SEG_END,        0,                            NULL,  0, 0, 0},
};

static const RouteSegment route38_segments[] = {
    {SEG_CROSS, MED_CAR_DISTANCE_SECOND_CHECK,  NULL,       0, 1, 0},
    {SEG_FORK,  MED_CAR_DISTANCE_R3_8_SHORT,     shibie,     1, 1, 1},
    {SEG_RP2,   MED_CAR_DISTANCE_R3_8_SHORT,     NULL,       0, 1, 0},
    {SEG_FORK,  MED_CAR_DISTANCE_R3_8_SHORT,     shibie,     1, 0, 1},
    {SEG_DOOR,  MED_CAR_DISTANCE_MID,            NULL,       0, 0, 0},
    {SEG_END,   0,                               NULL,       0, 0, 0},
};

static void find_1(void)
{
#if MED_CAR_USE_SENSOR_GUIDED_TURNS
    xunxian(MED_CAR_DISTANCE_FIRST_CHECK, 4000);
    sensor_deliver_left(MED_CAR_DISTANCE_HOME_WHITE, 4000);
#else
    xunxian(MED_CAR_DISTANCE_FIRST_CHECK, 4000);
    deliver_left(MED_CAR_DISTANCE_MID, MED_CAR_DISTANCE_HOME_WHITE, 4000);
#endif
}

static void find_2(void)
{
#if MED_CAR_USE_SENSOR_GUIDED_TURNS
    xunxian(MED_CAR_DISTANCE_FIRST_CHECK, 4000);
    sensor_deliver_right(MED_CAR_DISTANCE_HOME_WHITE, 4000);
#else
    xunxian(MED_CAR_DISTANCE_FIRST_CHECK, 4000);
    deliver_right(MED_CAR_DISTANCE_MID, MED_CAR_DISTANCE_HOME_WHITE, 4000);
#endif
}

static uint8_t deliver_if_matched(uint16_t return_cross_distance,
                                  uint16_t home_distance,
                                  int pwm)
{
    uint8_t side = find_matched_side();

    if (side == 0U) {
        return 0U;
    }

    VisionRing_StableRelease();
    rp2_clear_result();
    beep_matched_vision_once();
    if (side == 1U) {
        deliver_left(return_cross_distance, home_distance, pwm);
    } else {
        deliver_right(return_cross_distance, home_distance, pwm);
    }
    return 1U;
}

static void run3_8(void)
{
    xunxian(MED_CAR_DISTANCE_SECOND_CHECK, MED_CAR_R3_8_PWM_MAIN);

    VisionRing_StableArm();
    xunxian(MED_CAR_DISTANCE_R3_8_APPROACH, MED_CAR_R3_8_PWM_MAIN);
    xunxian(MED_CAR_DISTANCE_R3_8_SHORT, MED_CAR_R3_8_PWM_MAIN);
    shibie();

    if (deliver_if_matched(MED_CAR_DISTANCE_RETURN_CROSS3,
                           MED_CAR_DISTANCE_THIRD_CHECK,
                           MED_CAR_R3_8_PWM_MAIN) != 0U) {
        return;
    }
    VisionRing_StableRelease();

    move_forward_timed(MED_CAR_CROSS_ADVANCE_MS,
                       MED_CAR_R3_8_PWM_MAIN);
    if (rp2_scan_before_fork(MED_CAR_DISTANCE_THIRD_CHECK,
                             MED_CAR_R3_8_PWM_MAIN) != 0U) {
        if (deliver_if_matched(MED_CAR_DISTANCE_RETURN_CROSS4,
                               MED_CAR_DISTANCE_THIRD_CHECK,
                               MED_CAR_R3_8_PWM_MAIN) != 0U) {
            return;
        }
    }
    rp2_clear_result();
    VisionRing_StableRelease();
    move_forward_timed(MED_CAR_CROSS_ADVANCE_MS,
                       MED_CAR_R3_8_PWM_MAIN);

    VisionRing_StableArm();
    xunxian(MED_CAR_DISTANCE_R3_8_SHORT, MED_CAR_R3_8_PWM_MAIN);
    shibie();
    if (deliver_if_matched(MED_CAR_DISTANCE_RETURN_CROSS4,
                           MED_CAR_DISTANCE_THIRD_CHECK,
                           MED_CAR_R3_8_PWM_MAIN) == 0U) {
        VisionRing_StableRelease();
    }
}

static void fahui(void)
{
    VisionRing_StableArm();
    xunxian(MED_CAR_DISTANCE_SECOND_CHECK, MED_CAR_R3_8_PWM_FAHUI);
    shibie();

    if ((NumBuff[0] == 0) && (NumBuff[1] == 0)) {
        VisionRing_StableRelease();
        Load(0, 0);
        return;
    }

    if (deliver_if_matched(MED_CAR_DISTANCE_MID,
                           MED_CAR_DISTANCE_SECOND_CHECK,
                           MED_CAR_R3_8_PWM_FAHUI) != 0U) {
        return;
    }
    VisionRing_StableRelease();

    VisionRing_StableArm();
    zhao_bai(MED_CAR_DISTANCE_R3_8_ZHAO_BAI, MED_CAR_R3_8_PWM_MAIN);
    xunxian(MED_CAR_DISTANCE_R3_8_SHORT, MED_CAR_R3_8_PWM_MAIN);
    shibie();

    if (deliver_if_matched(MED_CAR_DISTANCE_RETURN_CROSS3,
                           MED_CAR_DISTANCE_THIRD_CHECK,
                           MED_CAR_R3_8_PWM_MAIN) != 0U) {
        return;
    }
    VisionRing_StableRelease();

    run3_8();
}

static uint8_t route_run(const RouteSegment *segments,
                         uint16_t return_distance, int pwm)
{
    uint8_t i;
    uint8_t door_mode = 0U;

    Return_Init();
    VisionRing_Flush();

    for (i = 0; segments[i].type != SEG_END; i++) {
        const RouteSegment *seg = &segments[i];

        if (door_mode != 0U) {
            xunxian_until_door(seg->distance, pwm);
            wait_delivery_done();
            Return_Execute(return_distance, pwm);
            return 1U;
        }

        switch (seg->type) {
        case SEG_CROSS:
            xunxian(seg->distance, pwm);
            if (seg->advance_if_straight != 0U) {
                move_forward_timed(MED_CAR_CROSS_ADVANCE_MS,
                                   MED_CAR_CROSS_ADVANCE_PWM);
            }
            Return_Push(RETURN_DIR_STRAIGHT);
            break;

        case SEG_FORK:
            if (seg->stable_precache != 0U) {
                VisionRing_StableArm();
            } else {
                VisionRing_Flush();
            }
            xunxian(seg->distance, pwm);
            Load(0, 0);
            delay_ms(MED_CAR_FORK_STOP_SETTLE_MS);
            if (seg->recognize != NULL) {
                seg->recognize();
            }
            if (check_match_and_turn() != 0U) {
                if (seg->auto_door != 0U) {
                    door_mode = 1U;
                }
            } else {
                VisionRing_StableRelease();
                Return_Push(RETURN_DIR_STRAIGHT);
                if (seg->advance_if_straight != 0U) {
                    move_forward_timed(MED_CAR_CROSS_ADVANCE_MS, pwm);
                }
            }
            break;

        case SEG_RP2:
            if (rp2_scan_before_fork(seg->distance, pwm) != 0U) {
                if (check_match_and_turn() != 0U) {
                    if (seg->auto_door != 0U) {
                        door_mode = 1U;
                    }
                    break;
                }
            }
            rp2_clear_result();
            VisionRing_StableRelease();
            Return_Push(RETURN_DIR_STRAIGHT);
            if (seg->advance_if_straight != 0U) {
                move_forward_timed(MED_CAR_CROSS_ADVANCE_MS, pwm);
            }
            break;

        case SEG_FORK_FIXED:
            xunxian(seg->distance, pwm);
            if (aim == 1) {
                Return_Push(RETURN_DIR_LEFT);
                sensor_turn_left();
            } else {
                Return_Push(RETURN_DIR_RIGHT);
                sensor_turn_right();
            }
            if (seg->auto_door != 0U) {
                door_mode = 1U;
            }
            break;

        case SEG_DOOR:
        case SEG_END:
            break;
        }
    }

    return 0U;
}

uint8_t MedicineCar_RunRoute12Test(uint8_t target_room)
{
    uint8_t ok;

    if ((target_room < 1U) || (target_room > 2U)) {
        u2_printf("ROUTE12 invalid target=%u\r\n", target_room);
        Load(0, 0);
        return 0U;
    }

    aim = (int)target_room;
    Run_Flag = 0;
    route_running = 1U;

    u2_printf("\r\n=== ROUTE12 TEST START target=%u ===\r\n", target_room);
    ok = route_run(route12_segments,
                   MED_CAR_DISTANCE_FIRST_CHECK,
                   MED_CAR_TRACE_PWM);

    route_running = 0U;
    if (ok == 0U) {
        Load(0, 0);
        MedicineCar_SetRedLed(1U);
        u2_printf("=== ROUTE12 FAIL target=%u ===\r\n", target_room);
    } else {
        u2_printf("=== ROUTE12 OK target=%u ===\r\n", target_room);
    }
    return ok;
}

uint8_t MedicineCar_RunRoute3To8Test(uint8_t target_room)
{
    uint8_t ok;

    if ((target_room < 3U) || (target_room > 8U)) {
        u2_printf("ROUTE3_8 invalid target=%u\r\n", target_room);
        Load(0, 0);
        return 0U;
    }

    aim = (int)target_room;
    Run_Flag = 0;
    route_running = 1U;

    u2_printf("\r\n=== ROUTE3_8 TEST START target=%u ===\r\n", target_room);
    ok = route_run(route38_segments,
                   MED_CAR_DISTANCE_FIRST_CHECK,
                   MED_CAR_TRACE_PWM);

    route_running = 0U;
    if (ok == 0U) {
        Load(0, 0);
        MedicineCar_SetRedLed(1U);
        u2_printf("=== ROUTE3_8 FAIL target=%u ===\r\n", target_room);
    } else {
        u2_printf("=== ROUTE3_8 OK target=%u ===\r\n", target_room);
    }
    return ok;
}

uint8_t MedicineCar_RunRp2Test(uint8_t target_room)
{
    uint8_t scan_ok;
    uint8_t matched = 0U;
    uint8_t success;

    if ((target_room < 3U) || (target_room > 8U)) {
        u2_printf("RP2 TEST invalid target=%u\r\n", target_room);
        Load(0, 0);
        return 0U;
    }

    aim = (int)target_room;
    Run_Flag = 0;
    route_running = 1U;
    rp2_test_logging = 1U;
    rp2_test_turn_ok = 0U;
    Return_Init();
    VisionRing_Flush();

    u2_printf("\r\n=== RP2 PROCESS TEST START target=%u ===\r\n",
              target_room);
    u2_printf("Production params: distance=%u pwm=%d advance=%ums\r\n",
              (unsigned int)MED_CAR_DISTANCE_R3_8_SHORT,
              MED_CAR_TRACE_PWM,
              (unsigned int)MED_CAR_RP2_FORK_ADVANCE_MS);

    scan_ok = rp2_scan_before_fork(MED_CAR_DISTANCE_R3_8_SHORT,
                                   MED_CAR_TRACE_PWM);
    if (scan_ok != 0U) {
        matched = check_match_and_turn();
    } else {
        RP2_TEST_LOG("[RP2][RESULT] scan failed; turn skipped\r\n");
    }

    if (matched == 0U) {
        rp2_clear_result();
        clear_recognition_buffers();
        VisionRing_StableRelease();
        Load(0, 0);
    }

    success = ((matched != 0U) && (rp2_test_turn_ok != 0U)) ? 1U : 0U;
    RP2_TEST_LOG("[RP2][RESULT] %s target=%u matched=%u turn_ok=%u\r\n",
                 (success != 0U) ? "PASS" : "FAILED",
                 target_room,
                 matched,
                 rp2_test_turn_ok);
    rp2_test_logging = 0U;
    route_running = 0U;
    return success;
}

void MedicineCar_Init(void)
{
    MedicineCarPlatform_Init();
    VisionRing_Init();
    route_running = 0U;
    rp2_test_logging = 0U;
    rp2_test_turn_ok = 0U;
    Run_Flag = 0;
    aim = 0;
    Number = 0;
    MedicineCar_SetGreenLed(0U);
}

void MedicineCar_Step(void)
{
    MedicineCarPlatform_Service();

    if ((MedicineCar_ReadDrugPresent() != 0U) && (N != 0)) {
        aim = N;
        Run_Flag = 1;
    }

    if ((Run_Flag == 0) || (route_running != 0U)) {
        Read_Speed();
        delay_ms(MED_CAR_STEP_IDLE_DELAY_MS);
        return;
    }

    route_running = 1U;
    Run_Flag = 0;

    if (aim == 0) {
        Load(0, 0);
    } else if (aim == 1) {
        find_1();
    } else if (aim == 2) {
        find_2();
    } else {
        fahui();
    }

    route_running = 0U;
}

void MedicineCar_RequestRun(uint8_t target_room)
{
    aim = (int)target_room;
    Run_Flag = 1;
}

void MedicineCar_SetRecognizedNumber(uint8_t number)
{
    N = (int)number;
}
