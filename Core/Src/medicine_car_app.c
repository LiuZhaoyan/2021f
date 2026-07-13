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

static void beep_matched_vision_once(void);

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

    if (side == 0U) {
        return 0U;
    }

    VisionRing_StableRelease();
    beep_matched_vision_once();
    if (side == 1U) {
        Return_Push(RETURN_DIR_LEFT);
        sensor_turn_left();
    } else {
        Return_Push(RETURN_DIR_RIGHT);
        sensor_turn_right();
    }
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

static uint8_t shibie_rp2(void)
{
    int    nums[4];
    uint8_t sides[4];
    uint8_t count = 0U;
    uint8_t i;

#define ADD_UNIQUE(num, side)                                          \
    do {                                                               \
        uint8_t __u;                                                   \
        uint8_t __found = 0U;                                          \
        if ((num) == 0) break;                                         \
        for (__u = 0U; __u < count; __u++) {                           \
            if (nums[__u] == (int)(num)) { __found = 1U; break; }      \
        }                                                              \
        if (!__found && count < MED_CAR_RP2_MAX_COLLECTED) {           \
            nums[count]  = (int)(num);                                 \
            sides[count] = (side);                                     \
            count++;                                                   \
        }                                                              \
    } while(0)

#define READ_FRESH_ENTRY(out)                                          \
    do {                                                               \
        VisionRing_Flush();                                            \
        delay_ms(MED_CAR_RP2_SCAN_SETTLE_MS);                         \
        if (VisionRing_WaitForNewEntry(                                \
                MED_CAR_RP2_SCAN_TIMEOUT_MS)) {                       \
            (void)VisionRing_ReadNext(&(out));                         \
        } else {                                                       \
            (out).left  = 0U;                                          \
            (out).right = 0U;                                          \
        }                                                              \
    } while(0)

    clear_recognition_buffers();

    /* 1. Center: left pixel -> left fork, right pixel -> right fork */
    {
        VisionRingEntry entry;
        READ_FRESH_ENTRY(entry);
        ADD_UNIQUE(entry.left,  FORK_LEFT);
        ADD_UNIQUE(entry.right, FORK_RIGHT);
    }

    /* 2. Wiggle left */
    wiggle_by_ticks(MED_CAR_RP2_WIGGLE_LEFT_PWM_LEFT,
                    MED_CAR_RP2_WIGGLE_LEFT_PWM_RIGHT,
                    MED_CAR_RP2_WIGGLE_TICKS);
    {
        VisionRingEntry entry;
        READ_FRESH_ENTRY(entry);
        ADD_UNIQUE(entry.left,  FORK_LEFT);
        ADD_UNIQUE(entry.right, FORK_LEFT);
    }

    /* 3. Wiggle right (2x ticks, left -> center -> right) */
    wiggle_by_ticks(MED_CAR_RP2_WIGGLE_RIGHT_PWM_LEFT,
                    MED_CAR_RP2_WIGGLE_RIGHT_PWM_RIGHT,
                    (uint16_t)(MED_CAR_RP2_WIGGLE_TICKS * 2U));
    {
        VisionRingEntry entry;
        READ_FRESH_ENTRY(entry);
        ADD_UNIQUE(entry.left,  FORK_RIGHT);
        ADD_UNIQUE(entry.right, FORK_RIGHT);
    }

    /* 4. Return to center */
    wiggle_by_ticks(MED_CAR_RP2_WIGGLE_LEFT_PWM_LEFT,
                    MED_CAR_RP2_WIGGLE_LEFT_PWM_RIGHT,
                    MED_CAR_RP2_WIGGLE_TICKS);

    /* 5. Populate NumBuff/XBuff — aim-priority strategy */
    clear_recognition_buffers();
    for (i = 0U; i < count; i++) {
        if (nums[i] == aim) {
            if (sides[i] == FORK_LEFT) {
                NumBuff[0] = aim;
                XBuff[0]  = 1;
            } else {
                NumBuff[1] = aim;
                XBuff[1]  = 2;
            }
            break;
        }
    }
    for (i = 0U; i < count; i++) {
        if (nums[i] == aim) {
            continue;
        }
        if (NumBuff[0] == 0) {
            NumBuff[0] = nums[i];
            XBuff[0]  = (sides[i] == FORK_LEFT) ? 1 : 2;
        } else if (NumBuff[1] == 0) {
            NumBuff[1] = nums[i];
            XBuff[1]  = (sides[i] == FORK_LEFT) ? 1 : 2;
        }
    }

#undef ADD_UNIQUE
#undef READ_FRESH_ENTRY

    return (count > 0U) ? 1U : 0U;
}

static const RouteSegment route12_segments[] = {
    {SEG_FORK_FIXED, MED_CAR_DISTANCE_FIRST_CHECK, NULL,  1, 0, 0},
    {SEG_DOOR,       MED_CAR_DISTANCE_MID,         NULL,  0, 0, 0},
    {SEG_END,        0,                            NULL,  0, 0, 0},
};

static const RouteSegment route38_segments[] = {
    {SEG_CROSS, MED_CAR_DISTANCE_SECOND_CHECK,  NULL,       0, 1, 0},
    {SEG_FORK,  MED_CAR_DISTANCE_R3_8_SHORT,     shibie,     1, 1, 1},
    {SEG_FORK,  MED_CAR_DISTANCE_R3_8_SHORT,     shibie_rp2, 0, 0, 0},
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

    xunxian(MED_CAR_DISTANCE_THIRD_CHECK, MED_CAR_R3_8_PWM_MAIN);
    shibie_rp2();

    if (deliver_if_matched(MED_CAR_DISTANCE_RETURN_CROSS4,
                           MED_CAR_DISTANCE_THIRD_CHECK,
                           MED_CAR_R3_8_PWM_MAIN) != 0U) {
        return;
    }
    VisionRing_StableRelease();

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
                   MED_CAR_TEST_GRAY_TRACE_PWM);

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
                   MED_CAR_TEST_GRAY_TRACE_PWM);

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

void MedicineCar_Init(void)
{
    MedicineCarPlatform_Init();
    VisionRing_Init();
    route_running = 0U;
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
