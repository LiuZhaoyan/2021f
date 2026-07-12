#include "medicine_car_app.h"

#include "medicine_car_config.h"
#include "medicine_car_platform.h"
#include "medicine_car_return.h"
#include "medicine_car_vision.h"

static uint8_t route_running;

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

static void deliver_branch(uint8_t direction, uint16_t return_distance, int pwm)
{
    Return_Push(direction);
    if (direction == RETURN_DIR_LEFT) {
        sensor_turn_left();
    } else {
        sensor_turn_right();
    }
    xunxian_until_door(MED_CAR_DISTANCE_MID, pwm);
    wait_delivery_done();
    Return_Execute(return_distance, pwm);
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

static uint8_t request_recognition_pair(uint8_t reverse_positions)
{
    uint8_t attempt;

    clear_recognition_buffers();
    for (attempt = 0U; attempt < MED_CAR_VISION_RETRY_COUNT; attempt++) {
        MedicineCarVisionResult result;

        if (MedicineCarVision_Request(&result,
                                      MED_CAR_VISION_UART_TIMEOUT_MS) != 0U) {
            uint8_t index;
            uint8_t usable_count = result.count;

            if (usable_count > 2U) {
                usable_count = 2U;
            }

            for (index = 0U; index < usable_count; index++) {
                NumBuff[index] = (int)result.digits[index];
                if (reverse_positions != 0U) {
                    XBuff[index] = (index == 0U) ? 2 : 1;
                } else {
                    XBuff[index] = (int)(index + 1U);
                }
            }
            return (usable_count > 0U) ? 1U : 0U;
        }
    }

    clear_recognition_buffers();
    return 0U;
}

static uint8_t shibie(void)
{
    return request_recognition_pair(0U);
}

static uint8_t shibie_1(void)
{
    return request_recognition_pair(1U);
}

static uint8_t shibie_34(void)
{
    return request_recognition_pair(1U);
}

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
    if (aim == NumBuff[0]) {
        if (XBuff[0] == 1) {
            deliver_left(return_cross_distance, home_distance, pwm);
        } else {
            deliver_right(return_cross_distance, home_distance, pwm);
        }
        return 1U;
    }

    if (aim == NumBuff[1]) {
        if (XBuff[1] == 1) {
            deliver_left(return_cross_distance, home_distance, pwm);
        } else {
            deliver_right(return_cross_distance, home_distance, pwm);
        }
        return 1U;
    }

    return 0U;
}

static uint8_t deliver_if_matched_logged(const char *stage,
                                         uint16_t return_cross_distance,
                                         uint16_t home_distance,
                                         int pwm)
{
    if (aim == NumBuff[0]) {
        if (XBuff[0] == 1) {
            u2_printf("[%s] match idx=0 room=%d dir=LEFT\r\n",
                      stage, NumBuff[0]);
            deliver_left(return_cross_distance, home_distance, pwm);
        } else {
            u2_printf("[%s] match idx=0 room=%d dir=RIGHT\r\n",
                      stage, NumBuff[0]);
            deliver_right(return_cross_distance, home_distance, pwm);
        }
        return 1U;
    }

    if (aim == NumBuff[1]) {
        if (XBuff[1] == 1) {
            u2_printf("[%s] match idx=1 room=%d dir=LEFT\r\n",
                      stage, NumBuff[1]);
            deliver_left(return_cross_distance, home_distance, pwm);
        } else {
            u2_printf("[%s] match idx=1 room=%d dir=RIGHT\r\n",
                      stage, NumBuff[1]);
            deliver_right(return_cross_distance, home_distance, pwm);
        }
        return 1U;
    }

    u2_printf("[%s] no match target=%d\r\n", stage, aim);
    return 0U;
}

static void route3_8_print_buffers(const char *stage, uint8_t recog_ok)
{
    u2_printf("[%s] vision ok=%u status=%s\r\n",
              stage,
              recog_ok,
              MedicineCarVision_LastStatusText());
    u2_printf("[%s] vision raw='%s'\r\n",
              stage,
              MedicineCarVision_LastLine());
    u2_printf("[%s] Num=[%d,%d] X=[%d,%d] target=%d\r\n",
              stage,
              NumBuff[0],
              NumBuff[1],
              XBuff[0],
              XBuff[1],
              aim);
}

static void run3_8(void)
{
    xunxian(MED_CAR_DISTANCE_SECOND_CHECK, MED_CAR_R3_8_PWM_MAIN);
    shibie_1();

    if (deliver_if_matched(MED_CAR_DISTANCE_MID,
                           MED_CAR_DISTANCE_SECOND_CHECK,
                           MED_CAR_R3_8_PWM_MAIN) != 0U) {
        return;
    }

    xunxian(MED_CAR_DISTANCE_R3_8_APPROACH, MED_CAR_R3_8_PWM_MAIN);
    shibie();
    xunxian(MED_CAR_DISTANCE_R3_8_SHORT, MED_CAR_R3_8_PWM_MAIN);

    if (deliver_if_matched(MED_CAR_DISTANCE_RETURN_CROSS3,
                           MED_CAR_DISTANCE_THIRD_CHECK,
                           MED_CAR_R3_8_PWM_MAIN) != 0U) {
        return;
    }

    xunxian(MED_CAR_DISTANCE_THIRD_CHECK, MED_CAR_R3_8_PWM_MAIN);
    shibie_1();
    (void)deliver_if_matched(MED_CAR_DISTANCE_RETURN_CROSS4,
                             MED_CAR_DISTANCE_THIRD_CHECK,
                             MED_CAR_R3_8_PWM_MAIN);
}

static void fahui(void)
{
    uint8_t recognize_attempts = 0U;

    xunxian(MED_CAR_DISTANCE_SECOND_CHECK, MED_CAR_R3_8_PWM_FAHUI);

    do {
        recognize_attempts++;
        if (shibie_34() == 0U) {
            break;
        }
    } while ((NumBuff[0] == NumBuff[1]) &&
             (recognize_attempts < MED_CAR_VISION_RETRY_COUNT));

    if ((NumBuff[0] == 0) && (NumBuff[1] == 0)) {
        Load(0, 0);
        return;
    }

    if (deliver_if_matched(MED_CAR_DISTANCE_MID,
                           MED_CAR_DISTANCE_SECOND_CHECK,
                           MED_CAR_R3_8_PWM_FAHUI) != 0U) {
        return;
    }

    shibie();
    zhao_bai(MED_CAR_DISTANCE_R3_8_ZHAO_BAI, MED_CAR_R3_8_PWM_MAIN);
    xunxian(MED_CAR_DISTANCE_R3_8_SHORT, MED_CAR_R3_8_PWM_MAIN);

    if (deliver_if_matched(MED_CAR_DISTANCE_RETURN_CROSS3,
                           MED_CAR_DISTANCE_THIRD_CHECK,
                           MED_CAR_R3_8_PWM_MAIN) != 0U) {
        return;
    }

    run3_8();
}

static void deliver_branch_logged(const char *stage, int num_val, int x_val,
                                   uint16_t return_distance, int pwm)
{
    uint8_t dir = (x_val == 1) ? RETURN_DIR_LEFT : RETURN_DIR_RIGHT;

    u2_printf("[%s] match room=%d dir=%s, push + deliver + return\r\n",
              stage, num_val, (dir == RETURN_DIR_LEFT) ? "LEFT" : "RIGHT");
    deliver_branch(dir, return_distance, pwm);
}

static uint8_t deliver_if_matched_return(const char *stage,
                                          uint16_t return_distance, int pwm)
{
    if (aim == NumBuff[0]) {
        deliver_branch_logged(stage, NumBuff[0], XBuff[0], return_distance, pwm);
        return 1U;
    }

    if (aim == NumBuff[1]) {
        deliver_branch_logged(stage, NumBuff[1], XBuff[1], return_distance, pwm);
        return 1U;
    }

    Return_Push(RETURN_DIR_STRAIGHT);
    u2_printf("[%s] no match, pushed STRAIGHT\r\n", stage);
    return 0U;
}

uint8_t MedicineCar_RunRoute3To8Test(uint8_t target_room)
{
    uint8_t recog_ok;

    if ((target_room < 3U) || (target_room > 8U)) {
        u2_printf("ROUTE3_8 invalid target=%u\r\n", target_room);
        Load(0, 0);
        return 0U;
    }

    Return_Init();
    aim = (int)target_room;
    Run_Flag = 0;
    route_running = 1U;

    u2_printf("\r\n=== ROUTE3_8 TEST START target=%u ===\r\n", target_room);

    u2_printf("[RP1] xunxian to cross1 (no digit), dist=%u pwm=%d\r\n",
              MED_CAR_DISTANCE_SECOND_CHECK, MED_CAR_TEST_GRAY_TRACE_PWM);
    xunxian(MED_CAR_DISTANCE_SECOND_CHECK, MED_CAR_TEST_GRAY_TRACE_PWM);
    Return_Push(RETURN_DIR_STRAIGHT);
    u2_printf("[RP1] cross1 passed, pushed STRAIGHT\r\n");

    recog_ok = shibie_34();
    route3_8_print_buffers("RP1", recog_ok);
    if (deliver_if_matched_return("RP1",
                                  MED_CAR_DISTANCE_FIRST_CHECK,
                                  MED_CAR_TEST_GRAY_TRACE_PWM) != 0U) {
        u2_printf("=== ROUTE3_8 OK target=%u stage=RP1 ===\r\n", target_room);
        route_running = 0U;
        return 1U;
    }

    u2_printf("[RP2] zhao_bai dist=%u pwm=%d\r\n",
              MED_CAR_DISTANCE_R3_8_ZHAO_BAI, MED_CAR_TEST_GRAY_TRACE_PWM);
    zhao_bai(MED_CAR_DISTANCE_R3_8_ZHAO_BAI, MED_CAR_TEST_GRAY_TRACE_PWM);
    recog_ok = shibie();
    route3_8_print_buffers("RP2", recog_ok);
    u2_printf("[RP2] xunxian dist=%u pwm=%d\r\n",
              MED_CAR_DISTANCE_R3_8_SHORT, MED_CAR_TEST_GRAY_TRACE_PWM);
    xunxian(MED_CAR_DISTANCE_R3_8_SHORT, MED_CAR_TEST_GRAY_TRACE_PWM);
    if (deliver_if_matched_return("RP2",
                                  MED_CAR_DISTANCE_FIRST_CHECK,
                                  MED_CAR_TEST_GRAY_TRACE_PWM) != 0U) {
        u2_printf("=== ROUTE3_8 OK target=%u stage=RP2 ===\r\n", target_room);
        route_running = 0U;
        return 1U;
    }

    u2_printf("[RP3] xunxian dist=%u pwm=%d\r\n",
              MED_CAR_DISTANCE_SECOND_CHECK, MED_CAR_TEST_GRAY_TRACE_PWM);
    xunxian(MED_CAR_DISTANCE_SECOND_CHECK, MED_CAR_TEST_GRAY_TRACE_PWM);
    recog_ok = shibie_1();
    route3_8_print_buffers("RP3", recog_ok);
    if (deliver_if_matched_return("RP3",
                                  MED_CAR_DISTANCE_FIRST_CHECK,
                                  MED_CAR_TEST_GRAY_TRACE_PWM) != 0U) {
        u2_printf("=== ROUTE3_8 OK target=%u stage=RP3 ===\r\n", target_room);
        route_running = 0U;
        return 1U;
    }

    u2_printf("[RP4] xunxian dist=%u pwm=%d\r\n",
              MED_CAR_DISTANCE_R3_8_APPROACH, MED_CAR_TEST_GRAY_TRACE_PWM);
    xunxian(MED_CAR_DISTANCE_R3_8_APPROACH, MED_CAR_TEST_GRAY_TRACE_PWM);
    recog_ok = shibie();
    route3_8_print_buffers("RP4", recog_ok);
    u2_printf("[RP4] xunxian dist=%u pwm=%d\r\n",
              MED_CAR_DISTANCE_R3_8_SHORT, MED_CAR_TEST_GRAY_TRACE_PWM);
    xunxian(MED_CAR_DISTANCE_R3_8_SHORT, MED_CAR_TEST_GRAY_TRACE_PWM);
    if (deliver_if_matched_return("RP4",
                                  MED_CAR_DISTANCE_FIRST_CHECK,
                                  MED_CAR_TEST_GRAY_TRACE_PWM) != 0U) {
        u2_printf("=== ROUTE3_8 OK target=%u stage=RP4 ===\r\n", target_room);
        route_running = 0U;
        return 1U;
    }

    u2_printf("[RP5] xunxian dist=%u pwm=%d\r\n",
              MED_CAR_DISTANCE_THIRD_CHECK, MED_CAR_TEST_GRAY_TRACE_PWM);
    xunxian(MED_CAR_DISTANCE_THIRD_CHECK, MED_CAR_TEST_GRAY_TRACE_PWM);
    recog_ok = shibie_1();
    route3_8_print_buffers("RP5", recog_ok);
    if (deliver_if_matched_return("RP5",
                                  MED_CAR_DISTANCE_FIRST_CHECK,
                                  MED_CAR_TEST_GRAY_TRACE_PWM) != 0U) {
        u2_printf("=== ROUTE3_8 OK target=%u stage=RP5 ===\r\n", target_room);
        route_running = 0U;
        return 1U;
    }

    Load(0, 0);
    MedicineCar_SetRedLed(1U);
    u2_printf("=== ROUTE3_8 FAIL no target matched target=%u ===\r\n", target_room);
    route_running = 0U;
    return 0U;
}

void MedicineCar_Init(void)
{
    MedicineCarPlatform_Init();
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
