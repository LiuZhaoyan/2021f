#include "medicine_car_app.h"

#include <stddef.h>

#include "medicine_car_config.h"
#include "medicine_car_platform.h"
#include "medicine_car_return.h"
#include "medicine_car_vision_cache.h"

#define FORK_LEFT  1U
#define FORK_RIGHT 2U

#define APP_LOG(stage, ...)                   \
    do {                                      \
        u2_printf("[%s][%s] ", __func__, stage); \
        u2_printf(__VA_ARGS__);               \
        u2_printf("\r\n");                  \
    } while (0)

typedef enum {
    SEG_CROSS,
    SEG_FORK,
    SEG_RP2,
    SEG_FORK_FIXED,
    SEG_DOOR,
    SEG_END
} SegmentType;

typedef struct {
    SegmentType type;
    uint16_t distance;
    uint8_t auto_door;
    uint8_t advance_if_straight;
} RouteSegment;

typedef struct {
    int nums[MED_CAR_RP2_MAX_COLLECTED];
    uint8_t sides[MED_CAR_RP2_MAX_COLLECTED];
    uint8_t count;
    uint8_t valid;
} Rp2RecognitionResult;

typedef enum {
    MATCH_TURN_MISS = 0,
    MATCH_TURN_OK,
    MATCH_TURN_FAILED
} MatchTurnResult;

static Rp2RecognitionResult rp2_result;

static const char *segment_name(SegmentType type)
{
    switch (type) {
    case SEG_CROSS:
        return "CROSS";
    case SEG_FORK:
        return "FORK";
    case SEG_RP2:
        return "RP2";
    case SEG_FORK_FIXED:
        return "FORK_FIXED";
    case SEG_DOOR:
        return "DOOR";
    case SEG_END:
        return "END";
    default:
        return "UNKNOWN";
    }
}

static void clear_recognition_buffers(void)
{
    NumBuff[0] = 0;
    NumBuff[1] = 0;
    XBuff[0] = 0;
    XBuff[1] = 0;
}

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

static void beep_matched_vision_once(void)
{
    MedicineCar_SetYellowLed(1U);
    delay_ms(MED_CAR_MATCH_BEEP_MS);
    MedicineCar_SetYellowLed(0U);
}

static void wait_delivery_done(void)
{
    uint32_t waited = 0U;

    APP_LOG("DELIVERY", "waiting for drug removal timeout=%lums",
            (unsigned long)MED_CAR_DELIVERY_WAIT_TIMEOUT_MS);
    MedicineCar_SetRedLed(1U);
    Load(0, 0);
    while ((MedicineCar_ReadDrugPresent() != 0U) &&
           (waited < MED_CAR_DELIVERY_WAIT_TIMEOUT_MS)) {
        Load(0, 0);
        delay_ms(20U);
        waited += 20U;
    }
    MedicineCar_SetRedLed(0U);
    APP_LOG("DELIVERY", "drug_present=%u waited=%lums",
            MedicineCar_ReadDrugPresent(), (unsigned long)waited);
}

static uint8_t find_matched_side(void)
{
    if ((NumBuff[0] != 0) && (aim == NumBuff[0])) {
        return (XBuff[0] == 1) ? FORK_LEFT : FORK_RIGHT;
    }
    if ((NumBuff[1] != 0) && (aim == NumBuff[1])) {
        return (XBuff[1] == 1) ? FORK_LEFT : FORK_RIGHT;
    }
    return 0U;
}

static MatchTurnResult check_match_and_turn(void)
{
    uint8_t side = find_matched_side();
    uint8_t turn_ok;

    APP_LOG("MATCH", "aim=%d Num=[%d,%d] X=[%d,%d] side=%u",
            aim, NumBuff[0], NumBuff[1], XBuff[0], XBuff[1], side);

    if (side == 0U) {
        APP_LOG("MISS", "target was not collected");
        return MATCH_TURN_MISS;
    }

    VisionCache_EndWindow();
    rp2_clear_result();
    beep_matched_vision_once();

    if (side == FORK_LEFT) {
        APP_LOG("TURN", "LEFT command");
        Return_Push(RETURN_DIR_LEFT);
        turn_ok = sensor_turn_left();
    } else {
        APP_LOG("TURN", "RIGHT command");
        Return_Push(RETURN_DIR_RIGHT);
        turn_ok = sensor_turn_right();
    }

    APP_LOG("TURN", "%s enc=(%d,%d)",
            (turn_ok != 0U) ? "OK" : "TIMEOUT",
            Encoder_Left, Encoder_Right);
    return (turn_ok != 0U) ? MATCH_TURN_OK : MATCH_TURN_FAILED;
}

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

    APP_LOG(stage, "start pwm=(%d,%d) target=%u timeout=%lums",
            left_pwm, right_pwm,
            (unsigned int)MED_CAR_RP2_WIGGLE_TICKS,
            (unsigned long)MED_CAR_RP2_WIGGLE_TIMEOUT_MS);
    wiggle_by_ticks(left_pwm, right_pwm, MED_CAR_RP2_WIGGLE_TICKS);
    Read_Speed();
    total_ticks = rp2_encoder_abs_sum();
    APP_LOG(stage, "%s enc=(%d,%d) total=%lu",
            (total_ticks >= (uint32_t)MED_CAR_RP2_WIGGLE_TICKS) ?
            "OK" : "TIMEOUT",
            Encoder_Left, Encoder_Right, (unsigned long)total_ticks);
}

static uint8_t populate_pair_from_frame(const MedicineCarVisionFrame *frame)
{
    if (frame == NULL) {
        return 0U;
    }

    clear_recognition_buffers();
    if (frame->left != 0U) {
        NumBuff[0] = (int)frame->left;
        XBuff[0] = FORK_LEFT;
    }
    if (frame->right != 0U) {
        if (frame->left != 0U) {
            NumBuff[1] = (int)frame->right;
            XBuff[1] = FORK_RIGHT;
        } else {
            NumBuff[0] = (int)frame->right;
            XBuff[0] = FORK_RIGHT;
        }
    }
    return 1U;
}

static void rp2_add_unique(uint8_t num, uint8_t side)
{
    uint8_t i;

    if (num == 0U) {
        return;
    }
    for (i = 0U; i < rp2_result.count; i++) {
        if (rp2_result.nums[i] == (int)num) {
            APP_LOG("RP2_COLLECT", "duplicate num=%u ignored", num);
            return;
        }
    }
    if (rp2_result.count < MED_CAR_RP2_MAX_COLLECTED) {
        rp2_result.nums[rp2_result.count] = (int)num;
        rp2_result.sides[rp2_result.count] = side;
        rp2_result.count++;
        APP_LOG("RP2_COLLECT", "num=%u side=%s count=%u", num,
                (side == FORK_LEFT) ? "LEFT" : "RIGHT",
                rp2_result.count);
    }
}

static uint8_t rp2_wait_view(const char *stage,
                             MedicineCarVisionFrame *frame)
{
    VisionCache_BeginWindow();
    APP_LOG(stage, "vision window opened timeout=%lums",
            (unsigned long)MED_CAR_RP2_SCAN_TIMEOUT_MS);

    if (VisionCache_Wait(frame, MED_CAR_RP2_SCAN_TIMEOUT_MS) == 0U) {
        APP_LOG(stage, "TIMEOUT: no complete valid frame");
        VisionCache_EndWindow();
        return 0U;
    }

    APP_LOG(stage, "frame left=%u right=%u timestamp=%lums",
            frame->left, frame->right,
            (unsigned long)frame->timestamp_ms);
    VisionCache_EndWindow();
    return 1U;
}

static uint8_t shibie_rp2(void)
{
    MedicineCarVisionFrame outer;

    APP_LOG("RP2_SCAN", "begin settle=%lums",
            (unsigned long)MED_CAR_RP2_SCAN_SETTLE_MS);
    clear_recognition_buffers();
    rp2_clear_result();
    VisionCache_EndWindow();
    delay_ms(MED_CAR_RP2_SCAN_SETTLE_MS);

    rp2_wiggle_with_log("LEFT_SCAN",
                        MED_CAR_RP2_WIGGLE_LEFT_PWM_LEFT,
                        MED_CAR_RP2_WIGGLE_LEFT_PWM_RIGHT);
    delay_ms(MED_CAR_RP2_SCAN_SETTLE_MS);
    if (rp2_wait_view("LEFT_VISION", &outer) != 0U) {
        rp2_add_unique(outer.left, FORK_LEFT);
        rp2_add_unique(outer.right, FORK_LEFT);
    }
    rp2_wiggle_with_log("LEFT_RETURN",
                        MED_CAR_RP2_WIGGLE_RIGHT_PWM_LEFT,
                        MED_CAR_RP2_WIGGLE_RIGHT_PWM_RIGHT);

    rp2_wiggle_with_log("RIGHT_SCAN",
                        MED_CAR_RP2_WIGGLE_RIGHT_PWM_LEFT,
                        MED_CAR_RP2_WIGGLE_RIGHT_PWM_RIGHT);
    delay_ms(MED_CAR_RP2_SCAN_SETTLE_MS);
    if (rp2_wait_view("RIGHT_VISION", &outer) != 0U) {
        rp2_add_unique(outer.left, FORK_RIGHT);
        rp2_add_unique(outer.right, FORK_RIGHT);
    }
    rp2_wiggle_with_log("RIGHT_RETURN",
                        MED_CAR_RP2_WIGGLE_LEFT_PWM_LEFT,
                        MED_CAR_RP2_WIGGLE_LEFT_PWM_RIGHT);

    rp2_result.valid = (rp2_result.count > 0U) ? 1U : 0U;
    APP_LOG("RP2_SCAN", "complete valid=%u count=%u",
            rp2_result.valid, rp2_result.count);
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
                XBuff[0] = FORK_LEFT;
            } else {
                NumBuff[1] = aim;
                XBuff[1] = FORK_RIGHT;
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
            XBuff[0] = (rp2_result.sides[i] == FORK_LEFT) ?
                       FORK_LEFT : FORK_RIGHT;
        } else if (NumBuff[1] == 0) {
            NumBuff[1] = rp2_result.nums[i];
            XBuff[1] = (rp2_result.sides[i] == FORK_LEFT) ?
                       FORK_LEFT : FORK_RIGHT;
        }
    }
}

static uint8_t rp2_scan_before_fork(uint16_t approach_distance,
                                    uint16_t to_fork_distance,
                                    int pwm)
{
    MedicineCarTraceStopReason reason;
    MedicineCarVisionFrame trigger;
    uint8_t scan_ok = 0U;
    uint8_t fork_found;

    rp2_clear_result();
    clear_recognition_buffers();
    VisionCache_BeginWindow();
    APP_LOG("APPROACH", "window opened max=%u pwm=%d; stop on first frame",
            approach_distance, pwm);

    reason = xunxian_until_fork_or_condition(
        approach_distance, pwm, VisionCache_HasFrame);
    APP_LOG("APPROACH", "stopped reason=%s enc=(%d,%d)",
            (reason == MED_CAR_TRACE_STOP_CONDITION) ? "VISION" :
            ((reason == MED_CAR_TRACE_STOP_FORK) ? "FORK" : "LIMIT"),
            Encoder_Left, Encoder_Right);

    if (reason == MED_CAR_TRACE_STOP_CONDITION) {
        if (VisionCache_Read(&trigger) != 0U) {
            APP_LOG("TRIGGER", "left=%u right=%u timestamp=%lums",
                    trigger.left, trigger.right,
                    (unsigned long)trigger.timestamp_ms);
        }
        VisionCache_EndWindow();
        scan_ok = shibie_rp2();
    } else {
        VisionCache_EndWindow();
        if (reason == MED_CAR_TRACE_STOP_LIMIT) {
            APP_LOG("MOTION_FAIL", "vision and fork were not found");
            return 0U;
        }
        APP_LOG("NO_TRIGGER", "fork reached before a valid vision frame");
    }

    if (reason == MED_CAR_TRACE_STOP_FORK) {
        fork_found = 1U;
    } else {
        APP_LOG("TO_FORK", "trace start max=%u pwm=%d",
                to_fork_distance, pwm);
        fork_found = xunxian_until_fork(to_fork_distance, pwm);
        APP_LOG("TO_FORK", "%s enc=(%d,%d)",
                (fork_found != 0U) ? "FORK" : "LIMIT",
                Encoder_Left, Encoder_Right);
    }

    if (fork_found == 0U) {
        clear_recognition_buffers();
        rp2_clear_result();
        APP_LOG("MOTION_FAIL", "fork was not detected after RP2 scan");
        return 0U;
    }

    if (scan_ok != 0U) {
        rp2_populate_recognition_buffers();
    } else {
        clear_recognition_buffers();
        rp2_clear_result();
    }

    APP_LOG("BUFFER", "aim=%d Num=[%d,%d] X=[%d,%d]",
            aim, NumBuff[0], NumBuff[1], XBuff[0], XBuff[1]);
    return 1U;
}

static const RouteSegment route12_segments[] = {
    {SEG_FORK_FIXED, MED_CAR_DISTANCE_FIRST_CHECK, 1U, 0U},
    {SEG_DOOR,       MED_CAR_DISTANCE_MID,         0U, 0U},
    {SEG_END,        0U,                           0U, 0U},
};

static const RouteSegment route38_segments[] = {
    {SEG_CROSS, MED_CAR_DISTANCE_SECOND_CHECK, 0U, 1U},
    {SEG_FORK,  MED_CAR_DISTANCE_R3_8_SHORT,    1U, 1U},
    {SEG_RP2,   MED_CAR_DISTANCE_R3_8_SHORT,    0U, 1U},
    {SEG_FORK,  MED_CAR_DISTANCE_R3_8_SHORT,    1U, 1U},
    {SEG_DOOR,  MED_CAR_DISTANCE_MID,           0U, 0U},
    {SEG_END,   0U,                             0U, 0U},
};

static void route_continue_straight(int pwm)
{
    VisionCache_EndWindow();
    Return_Push(RETURN_DIR_STRAIGHT);
    APP_LOG("FALLBACK", "straight escape %lums pwm=%d",
            (unsigned long)MED_CAR_CROSS_ADVANCE_MS, pwm);
    move_forward_timed(MED_CAR_CROSS_ADVANCE_MS, pwm);
}

static uint8_t route_run(const RouteSegment *segments,
                         uint16_t return_distance, int pwm)
{
    uint8_t i;
    uint8_t door_mode = 0U;

    Return_Init();
    VisionCache_EndWindow();
    clear_recognition_buffers();
    rp2_clear_result();

    for (i = 0U; segments[i].type != SEG_END; i++) {
        const RouteSegment *seg = &segments[i];
        MatchTurnResult match_result;
        uint8_t motion_ok;

        VisionCache_EndWindow();
        clear_recognition_buffers();
        rp2_clear_result();
        APP_LOG("SEG_BEGIN", "index=%u type=%s distance=%u cache=cleared",
                i, segment_name(seg->type), seg->distance);

        if (door_mode != 0U) {
            motion_ok = xunxian_until_door(seg->distance, pwm);
            APP_LOG("DOOR", "detected=%u enc=(%d,%d)",
                    motion_ok, Encoder_Left, Encoder_Right);
            if (motion_ok == 0U) {
                Load(0, 0);
                return 0U;
            }
            wait_delivery_done();
            Return_Execute(return_distance, pwm);
            APP_LOG("ROUTE_OK", "delivery and return completed");
            return 1U;
        }

        switch (seg->type) {
        case SEG_CROSS:
            motion_ok = xunxian(seg->distance, pwm);
            APP_LOG("CROSS", "detected=%u enc=(%d,%d)",
                    motion_ok, Encoder_Left, Encoder_Right);
            if (motion_ok == 0U) {
                return 0U;
            }
            if (seg->advance_if_straight != 0U) {
                move_forward_timed(MED_CAR_CROSS_ADVANCE_MS,
                                   MED_CAR_CROSS_ADVANCE_PWM);
            }
            Return_Push(RETURN_DIR_STRAIGHT);
            break;

        case SEG_FORK: {
            MedicineCarVisionFrame frame;
            uint8_t frame_valid;

            VisionCache_BeginWindow();
            APP_LOG("FORK", "vision window opened; tracing without vision stop");
            motion_ok = xunxian(seg->distance, pwm);
            APP_LOG("FORK", "detected=%u enc=(%d,%d)",
                    motion_ok, Encoder_Left, Encoder_Right);
            if (motion_ok == 0U) {
                VisionCache_EndWindow();
                return 0U;
            }

            frame_valid = VisionCache_Read(&frame);
            VisionCache_EndWindow();
            Load(0, 0);
            delay_ms(MED_CAR_FORK_STOP_SETTLE_MS);
            if (frame_valid != 0U) {
                APP_LOG("FORK_FRAME", "left=%u right=%u timestamp=%lums",
                        frame.left, frame.right,
                        (unsigned long)frame.timestamp_ms);
                populate_pair_from_frame(&frame);
            } else {
                APP_LOG("FORK_FRAME", "no complete valid frame cached");
            }

            match_result = check_match_and_turn();
            if (match_result == MATCH_TURN_OK) {
                if (seg->auto_door != 0U) {
                    door_mode = 1U;
                }
            } else if (match_result == MATCH_TURN_FAILED) {
                return 0U;
            } else {
                route_continue_straight(pwm);
            }
            break;
        }

        case SEG_RP2:
            motion_ok = rp2_scan_before_fork(
                seg->distance,
                MED_CAR_RP2_TO_FORK_MAX_DISTANCE,
                pwm);
            if (motion_ok == 0U) {
                return 0U;
            }

            match_result = check_match_and_turn();
            if (match_result == MATCH_TURN_OK) {
                if (seg->auto_door != 0U) {
                    door_mode = 1U;
                }
            } else if (match_result == MATCH_TURN_FAILED) {
                return 0U;
            } else {
                route_continue_straight(pwm);
            }
            break;

        case SEG_FORK_FIXED:
            motion_ok = xunxian(seg->distance, pwm);
            APP_LOG("FORK_FIXED", "detected=%u target=%d enc=(%d,%d)",
                    motion_ok, aim, Encoder_Left, Encoder_Right);
            if (motion_ok == 0U) {
                return 0U;
            }
            if (aim == 1) {
                Return_Push(RETURN_DIR_LEFT);
                motion_ok = sensor_turn_left();
            } else {
                Return_Push(RETURN_DIR_RIGHT);
                motion_ok = sensor_turn_right();
            }
            APP_LOG("FORK_FIXED", "turn=%s",
                    (motion_ok != 0U) ? "OK" : "TIMEOUT");
            if (motion_ok == 0U) {
                return 0U;
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

    Load(0, 0);
    APP_LOG("ROUTE_FAIL", "route ended without delivery");
    return 0U;
}

uint8_t MedicineCar_RunRoute12Test(uint8_t target_room)
{
    uint8_t ok;

    if ((target_room < 1U) || (target_room > 2U)) {
        APP_LOG("INVALID", "target=%u", target_room);
        Load(0, 0);
        return 0U;
    }

    aim = (int)target_room;
    APP_LOG("START", "target=%u", target_room);
    ok = route_run(route12_segments,
                   MED_CAR_DISTANCE_FIRST_CHECK,
                   MED_CAR_TRACE_PWM);
    if (ok == 0U) {
        Load(0, 0);
        MedicineCar_SetRedLed(1U);
    }
    APP_LOG("RESULT", "%s target=%u", (ok != 0U) ? "OK" : "FAIL",
            target_room);
    return ok;
}

uint8_t MedicineCar_RunRoute3To8Test(uint8_t target_room)
{
    uint8_t ok;

    if ((target_room < 3U) || (target_room > 8U)) {
        APP_LOG("INVALID", "target=%u", target_room);
        Load(0, 0);
        return 0U;
    }

    aim = (int)target_room;
    APP_LOG("START", "target=%u", target_room);
    ok = route_run(route38_segments,
                   MED_CAR_DISTANCE_FIRST_CHECK,
                   MED_CAR_TRACE_PWM);
    if (ok == 0U) {
        Load(0, 0);
        MedicineCar_SetRedLed(1U);
    }
    APP_LOG("RESULT", "%s target=%u", (ok != 0U) ? "OK" : "FAIL",
            target_room);
    return ok;
}

uint8_t MedicineCar_RunRp2Test(uint8_t target_room)
{
    uint8_t motion_ok;
    MatchTurnResult match_result = MATCH_TURN_MISS;
    uint8_t success;

    if ((target_room < 3U) || (target_room > 8U)) {
        APP_LOG("INVALID", "target=%u", target_room);
        Load(0, 0);
        return 0U;
    }

    aim = (int)target_room;
    Return_Init();
    VisionCache_EndWindow();
    APP_LOG("START", "target=%u approach=%u to_fork=%u pwm=%d",
            target_room,
            (unsigned int)MED_CAR_DISTANCE_R3_8_SHORT,
            (unsigned int)MED_CAR_RP2_TO_FORK_MAX_DISTANCE,
            MED_CAR_TRACE_PWM);

    motion_ok = rp2_scan_before_fork(
        MED_CAR_DISTANCE_R3_8_SHORT,
        MED_CAR_RP2_TO_FORK_MAX_DISTANCE,
        MED_CAR_TRACE_PWM);
    if (motion_ok != 0U) {
        match_result = check_match_and_turn();
    }

    success = ((motion_ok != 0U) &&
               (match_result == MATCH_TURN_OK)) ? 1U : 0U;
    if (success == 0U) {
        VisionCache_EndWindow();
        rp2_clear_result();
        clear_recognition_buffers();
        Load(0, 0);
    }

    APP_LOG("RESULT", "%s target=%u motion_ok=%u match_result=%u",
            (success != 0U) ? "PASS" : "FAIL",
            target_room, motion_ok, (unsigned int)match_result);
    return success;
}

void MedicineCar_Init(void)
{
    MedicineCarPlatform_Init();
    VisionCache_Init();
    aim = 0;
    VisionCache_EndWindow();
    clear_recognition_buffers();
    rp2_clear_result();
    MedicineCar_SetGreenLed(0U);
}

void MedicineCar_Step(void)
{
    MedicineCarPlatform_Service();
    Load(0, 0);
    Read_Speed();
    delay_ms(MED_CAR_STEP_IDLE_DELAY_MS);
}
