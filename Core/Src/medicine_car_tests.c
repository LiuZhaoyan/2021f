#include "medicine_car_tests.h"

#include <stdint.h>

#include "medicine_car_app.h"
#include "medicine_car_config.h"
#include "medicine_car_test_config.h"
#include "medicine_car_platform.h"
#include "medicine_car_return.h"
#include "medicine_car_vision_cache.h"

#define TEST_LOG(stage, ...)                  \
    do {                                      \
        u2_printf("[%s][%s] ", __func__, stage); \
        u2_printf(__VA_ARGS__);               \
        u2_printf("\r\n");                  \
    } while (0)

static uint8_t wait_for_test_target(uint8_t min_target,
                                    uint8_t max_target);

static int abs_int(int value)
{
    return (value < 0) ? -value : value;
}

static void debug_motor_test_loop(void)
{
    const int pwm = MED_CAR_TEST_MOTOR_PWM;

    u2_printf("\r\nMotor test start, pwm=%d\r\n", pwm);
    Load(0, 0);
    delay_ms(1000U);

    while (1) {
        u2_printf("Left forward\r\n");
        MedicineCar_SetGreenLed(1U);
        Load(pwm, 0);
        delay_ms(MED_CAR_TEST_MOTOR_RUN_MS);
        Load(0, 0);
        delay_ms(MED_CAR_TEST_MOTOR_STOP_MS);

        u2_printf("Left reverse\r\n");
        Load(-pwm, 0);
        delay_ms(MED_CAR_TEST_MOTOR_RUN_MS);
        Load(0, 0);
        delay_ms(MED_CAR_TEST_MOTOR_STOP_MS);

        u2_printf("Right forward\r\n");
        Load(0, pwm);
        delay_ms(MED_CAR_TEST_MOTOR_RUN_MS);
        Load(0, 0);
        delay_ms(MED_CAR_TEST_MOTOR_STOP_MS);

        u2_printf("Right reverse\r\n");
        Load(0, -pwm);
        delay_ms(MED_CAR_TEST_MOTOR_RUN_MS);
        Load(0, 0);
        delay_ms(MED_CAR_TEST_MOTOR_STOP_MS);

        u2_printf("Both forward\r\n");
        Load(pwm, pwm);
        delay_ms(MED_CAR_TEST_MOTOR_RUN_MS);
        Load(0, 0);
        delay_ms(MED_CAR_TEST_MOTOR_STOP_MS);

        u2_printf("Both reverse\r\n");
        Load(-pwm, -pwm);
        delay_ms(MED_CAR_TEST_MOTOR_RUN_MS);
        Load(0, 0);
        MedicineCar_SetGreenLed(0U);
        delay_ms(2000U);
    }
}

static void print_gray_snapshot(const char *prefix)
{
    uint8_t gray[8];

    MedicineCar_ReadLineSensors(gray);
    u2_printf("%s: %u %u %u %u %u %u %u %u\r\n",
              prefix,
              gray[0], gray[1], gray[2], gray[3],
              gray[4], gray[5], gray[6], gray[7]);
}

static void debug_gray_test_loop(void)
{
    u2_printf("\r\nGray sensor test start\r\n");
    while (1) {
        print_gray_snapshot("GRAY");
        delay_ms(MED_CAR_TEST_GRAY_PRINT_MS);
    }
}

static void debug_gray_trace_test_loop(void)
{
    u2_printf("\r\nGray trace test start, distance=%u pwm=%d\r\n",
              MED_CAR_TEST_GRAY_TRACE_DISTANCE,
              MED_CAR_TEST_GRAY_TRACE_PWM);
    Load(0, 0);
    delay_ms(1000U);

    while (1) {
        print_gray_snapshot("GRAY BEFORE TRACE");
        Read_Speed();
        u2_printf("TRACE BEFORE\r\n");
        u2_printf("Speed L=%d R=%d\r\n", Speed_L, Speed_R);
        u2_printf("Enc L=%d R=%d\r\n", Encoder_Left, Encoder_Right);
        xunxian(MED_CAR_TEST_GRAY_TRACE_DISTANCE, MED_CAR_TEST_GRAY_TRACE_PWM);
        Load(0, 0);
        print_gray_snapshot("GRAY AFTER TRACE");
        Read_Speed();
        u2_printf("TRACE AFTER\r\n");
        u2_printf("Speed L=%d R=%d\r\n", Speed_L, Speed_R);
        u2_printf("Enc L=%d R=%d\r\n", Encoder_Left, Encoder_Right);
        u2_printf("Gray trace segment done\r\n");
        delay_ms(MED_CAR_TEST_GRAY_TRACE_PAUSE_MS);
    }
}

static void debug_led_beep_test_loop(void)
{
    u2_printf("\r\nLED and beep test start\r\n");

    while (1) {
        u2_printf("LED on, beep off\r\n");
        MedicineCar_SetRedLed(1U);
        MedicineCar_SetYellowLed(0U);
        delay_ms(MED_CAR_TEST_LED_BEEP_STEP_MS);

        u2_printf("LED off, beep on\r\n");
        MedicineCar_SetRedLed(0U);
        MedicineCar_SetYellowLed(1U);
        delay_ms(MED_CAR_TEST_LED_BEEP_STEP_MS);

        u2_printf("LED on, beep on\r\n");
        MedicineCar_SetRedLed(1U);
        MedicineCar_SetYellowLed(1U);
        delay_ms(MED_CAR_TEST_LED_BEEP_STEP_MS);

        u2_printf("LED off, beep off\r\n");
        MedicineCar_SetRedLed(0U);
        MedicineCar_SetYellowLed(0U);
        delay_ms(MED_CAR_TEST_LED_BEEP_STEP_MS);
    }
}

static void debug_route_test_loop(void)
{
    uint8_t target;

    Load(0, 0);
    MedicineCar_SetGreenLed(0U);

    TEST_LOG("START", "ROUTE 1/2 TEST (route table)");
    target = wait_for_test_target(1U, 2U);

    TEST_LOG("WAIT_DRUG", "target=%u waiting for PRESENT", target);
    while (MedicineCar_ReadDrugPresent() == 0U) {
        MedicineCarPlatform_Service();
        delay_ms(100U);
    }
    TEST_LOG("DRUG_READY", "target=%u drug=PRESENT", target);

    TEST_LOG("ROUTE_START", "target=%u", target);
    (void)MedicineCar_RunRoute12Test(target);

    while (1) {
        MedicineCarPlatform_Service();
        delay_ms(1000U);
    }
}

static void debug_route3_8_test_loop(void)
{
    uint8_t cached_target;
    uint8_t success;
    uint32_t drug_wait_ms = 0U;

    Load(0, 0);
    TEST_LOG("START", "ROUTE3_8 DYNAMIC TEST");
    cached_target = wait_for_test_target(3U, 8U);

    TEST_LOG("WAIT_DRUG", "target=%u raw=%s interpreted=%s",
             cached_target,
             (MedicineCar_ReadDrugSensorRaw() != 0U) ? "HIGH" : "LOW",
             (MedicineCar_ReadDrugPresent() != 0U) ? "PRESENT" : "ABSENT");

    while (MedicineCar_ReadDrugPresent() == 0U) {
        MedicineCarPlatform_Service();
        delay_ms(MED_CAR_TEST_R3_8_DRUG_POLL_MS);
        drug_wait_ms += MED_CAR_TEST_R3_8_DRUG_POLL_MS;
        if (drug_wait_ms >= MED_CAR_TEST_R3_8_IDLE_PRINT_MS) {
            drug_wait_ms = 0U;
            TEST_LOG("WAIT_DRUG", "raw=%s interpreted=%s",
                     (MedicineCar_ReadDrugSensorRaw() != 0U) ? "HIGH" : "LOW",
                     (MedicineCar_ReadDrugPresent() != 0U) ? "PRESENT" : "ABSENT");
        }
    }

    TEST_LOG("ROUTE_START", "drug=PRESENT target=%u", cached_target);
    success = MedicineCar_RunRoute3To8Test(cached_target);
    Load(0, 0);
    if (success != 0U) {
        MedicineCar_SetGreenLed(1U);
    }

    TEST_LOG("RESULT", "success=%u target=%u; idle loop",
             success, cached_target);
    while (1) {
        MedicineCarPlatform_Service();
        TEST_LOG("IDLE", "success=%u target=%u", success, cached_target);
        delay_ms(MED_CAR_TEST_R3_8_IDLE_PRINT_MS);
    }
}

static void print_vision_frame(const MedicineCarVisionFrame *frame)
{
    if (frame == 0) {
        return;
    }

    TEST_LOG("VISION_FRAME", "left=%u right=%u timestamp=%lums",
             frame->left, frame->right,
             (unsigned long)frame->timestamp_ms);
}

static uint8_t find_target_in_frame(const MedicineCarVisionFrame *frame,
                                    uint8_t min_target,
                                    uint8_t max_target)
{
    if (frame == 0) {
        return 0U;
    }
    if ((frame->left >= min_target) && (frame->left <= max_target)) {
        return frame->left;
    }
    if ((frame->right >= min_target) && (frame->right <= max_target)) {
        return frame->right;
    }
    return 0U;
}

static uint8_t wait_for_test_target(uint8_t min_target,
                                    uint8_t max_target)
{
    uint8_t target = 0U;

    VisionCache_EndWindow();
    TEST_LOG("WAIT_TARGET", "waiting indefinitely for target %u..%u",
             min_target, max_target);

    while (target == 0U) {
        MedicineCarVisionFrame frame;

        VisionCache_BeginWindow();
        if (VisionCache_Wait(&frame,
                             MED_CAR_VISION_UART_TIMEOUT_MS) != 0U) {
            print_vision_frame(&frame);
            target = find_target_in_frame(&frame, min_target, max_target);
            if (target != 0U) {
                TEST_LOG("TARGET_CACHED", "target=%u", target);
            } else {
                TEST_LOG("TARGET_IGNORED",
                         "frame contains no target in %u..%u",
                         min_target, max_target);
            }
        } else {
            TEST_LOG("WAIT_TARGET", "no complete valid frame in %lums",
                     (unsigned long)MED_CAR_VISION_UART_TIMEOUT_MS);
        }
        VisionCache_EndWindow();

        if (target == 0U) {
            MedicineCarPlatform_Service();
            delay_ms(MED_CAR_TEST_R3_8_VISION_INTERVAL_MS);
        }
    }

    return target;
}

static void debug_rp2_test_loop(void)
{
    uint8_t target;
    uint8_t success;
    uint32_t wait_ms = 0U;

    Load(0, 0);
    MedicineCar_SetRedLed(0U);
    MedicineCar_SetGreenLed(0U);
    TEST_LOG("START", "RP2 PROCESS TEST");
    target = wait_for_test_target(3U, 8U);

    TEST_LOG("WAIT_DRUG_ABSENT", "target=%u remove drug before arming", target);
    while (MedicineCar_ReadDrugPresent() != 0U) {
        MedicineCarPlatform_Service();
        delay_ms(MED_CAR_TEST_R3_8_DRUG_POLL_MS);
        wait_ms += MED_CAR_TEST_R3_8_DRUG_POLL_MS;
        if (wait_ms >= MED_CAR_TEST_R3_8_IDLE_PRINT_MS) {
            wait_ms = 0U;
            TEST_LOG("WAIT_DRUG_ABSENT", "target=%u", target);
        }
    }

    wait_ms = 0U;
    TEST_LOG("WAIT_DRUG_PRESENT",
             "place car before RP2, then load drug; target=%u", target);
    while (MedicineCar_ReadDrugPresent() == 0U) {
        MedicineCarPlatform_Service();
        delay_ms(MED_CAR_TEST_R3_8_DRUG_POLL_MS);
        wait_ms += MED_CAR_TEST_R3_8_DRUG_POLL_MS;
        if (wait_ms >= MED_CAR_TEST_R3_8_IDLE_PRINT_MS) {
            wait_ms = 0U;
            TEST_LOG("WAIT_DRUG_PRESENT", "armed target=%u", target);
        }
    }

    TEST_LOG("ROUTE_START", "target=%u", target);
    success = MedicineCar_RunRp2Test(target);
    Load(0, 0);
    if (success != 0U) {
        MedicineCar_SetGreenLed(1U);
    } else {
        MedicineCar_SetRedLed(1U);
    }

    TEST_LOG("RESULT", "%s target=%u",
             (success != 0U) ? "PASS" : "FAIL", target);
    while (1) {
        MedicineCarPlatform_Service();
        delay_ms(MED_CAR_TEST_R3_8_IDLE_PRINT_MS);
    }
}

static void debug_encoder_test_loop(void)
{
    const int pwm = MED_CAR_TEST_ENCODER_PWM;

    u2_printf("\r\nEncoder test start, pwm=%d\r\n", pwm);

    while (1) {
        uint32_t elapsed = 0U;

        MedicineCar_ResetEncoders();
        u2_printf("Encoder reset: L=%d R=%d\r\n", Encoder_Left, Encoder_Right);

        Load(pwm, pwm);
        while (elapsed < MED_CAR_TEST_ENCODER_RUN_MS) {
            delay_ms(MED_CAR_TEST_ENCODER_SAMPLE_MS);
            elapsed += MED_CAR_TEST_ENCODER_SAMPLE_MS;
            Read_Speed();
            u2_printf("Encoder %lu ms: L=%d R=%d SpeedL=%d SpeedR=%d\r\n",
                      (unsigned long)elapsed,
                      Encoder_Left,
                      Encoder_Right,
                      Speed_L,
                      Speed_R);
        }

        Load(0, 0);
        u2_printf("Encoder test cycle done\r\n");
        delay_ms(1000U);
    }
}

static void debug_wheel_match_test_loop(void)
{
    const int left_pwm = MED_CAR_TEST_WHEEL_MATCH_LEFT_PWM;
    const int right_pwm = MED_CAR_TEST_WHEEL_MATCH_RIGHT_PWM;

    u2_printf("\r\nWheel match test start, pwm L=%d R=%d\r\n",
              left_pwm,
              right_pwm);
    Load(0, 0);
    delay_ms(1000U);

    while (1) {
        uint32_t elapsed = 0U;
        int last_left;
        int last_right;

        MedicineCar_ResetEncoders();
        last_left = Encoder_Left;
        last_right = Encoder_Right;

        Load(left_pwm, right_pwm);
        while (elapsed < MED_CAR_TEST_WHEEL_MATCH_RUN_MS) {
            int left_delta;
            int right_delta;
            int speed_diff;

            delay_ms(MED_CAR_TEST_WHEEL_MATCH_SAMPLE_MS);
            elapsed += MED_CAR_TEST_WHEEL_MATCH_SAMPLE_MS;
            Read_Speed();

            left_delta = Encoder_Left - last_left;
            right_delta = Encoder_Right - last_right;
            speed_diff = abs_int(left_delta) - abs_int(right_delta);

            u2_printf("Wheel match %lu ms: PWM L=%d R=%d "
                      "Tick L=%d R=%d Diff=%d Enc L=%d R=%d\r\n",
                      (unsigned long)elapsed,
                      left_pwm,
                      right_pwm,
                      left_delta,
                      right_delta,
                      speed_diff,
                      Encoder_Left,
                      Encoder_Right);

            last_left = Encoder_Left;
            last_right = Encoder_Right;
        }

        Load(0, 0);
        u2_printf("Wheel match cycle done, diff>0 means left faster\r\n");
        delay_ms(MED_CAR_TEST_WHEEL_MATCH_PAUSE_MS);
    }
}

static void debug_vision_uart_test_loop(void)
{
    TEST_LOG("START", "USART3 115200 8N1");
    TEST_LOG("PROTOCOL", "OpenMV streams AA BB left right CC every 50 ms");

    while (1) {
        MedicineCarVisionFrame frame;

        VisionCache_BeginWindow();
        if (VisionCache_Wait(&frame,
                             MED_CAR_VISION_UART_TIMEOUT_MS) != 0U) {
            print_vision_frame(&frame);
        } else {
            TEST_LOG("TIMEOUT", "no complete valid frame in %lums",
                     (unsigned long)MED_CAR_VISION_UART_TIMEOUT_MS);
        }
        VisionCache_EndWindow();

        delay_ms(MED_CAR_TEST_VISION_PRINT_MS);
    }
}

static void debug_drug_sensor_test_loop(void)
{
    u2_printf("\r\nDrug sensor test start, PC1 input pull-up, low means present\r\n");

    while (1) {
        uint8_t raw = MedicineCar_ReadDrugSensorRaw();
        uint8_t present = MedicineCar_ReadDrugPresent();

        u2_printf("DRUG raw=%s interpreted=%s\r\n",
                  (raw != 0U) ? "HIGH" : "LOW",
                  (present != 0U) ? "PRESENT" : "ABSENT");
        delay_ms(MED_CAR_TEST_DRUG_PRINT_MS);
    }
}

static void debug_cross_turn_once(uint32_t cycle,
                                  const char *turn_name,
                                  uint8_t (*turn_fn)(void))
{
    uint8_t cross_found;
    uint8_t turn_result;
    uint32_t left_abs;
    uint32_t right_abs;
    uint32_t total_abs_ticks;
    uint32_t wheel_gap;

    Load(0, 0);
    u2_printf("Cycle %lu %s: put the car on the line before a cross, start in 3s.\r\n",
              (unsigned long)cycle,
              turn_name);
    delay_ms(3000U);

    cross_found = xunxian(MED_CAR_TEST_CROSS_TURN_MAX_TICKS,
                          MED_CAR_TEST_CROSS_TURN_PWM);
    Read_Speed();
    Load(0, 0);

    left_abs = (uint32_t)abs_int(Encoder_Left);
    right_abs = (uint32_t)abs_int(Encoder_Right);
    total_abs_ticks = left_abs + right_abs;
    wheel_gap = (left_abs > right_abs) ? (left_abs - right_abs) : (right_abs - left_abs);

    u2_printf("Cycle %lu %s approach %s: L=%d R=%d sum=%d total_abs=%lu gap=%lu\r\n",
              (unsigned long)cycle,
              turn_name,
              (cross_found != 0U) ? "CROSS" : "LIMIT",
              Encoder_Left,
              Encoder_Right,
              Encoder_Left + Encoder_Right,
              (unsigned long)total_abs_ticks,
              (unsigned long)wheel_gap);

    if (cross_found == 0U) {
        u2_printf("Cycle %lu %s skip turn: cross not found before max ticks.\r\n",
                  (unsigned long)cycle,
                  turn_name);
        return;
    }

    u2_printf("Cycle %lu turn %s...\r\n",
              (unsigned long)cycle,
              turn_name);
    turn_result = turn_fn();
    u2_printf("Cycle %lu turn %s result: %s\r\n",
              (unsigned long)cycle,
              turn_name,
              (turn_result != 0U) ? "OK" : "TIMEOUT");
}

static void debug_cross_turn_test_loop(void)
{
    uint32_t cycle;

    u2_printf("\r\nCross turn test start\r\n");
    u2_printf("Right turn -> door -> drug removal -> diaotou -> cross -> left turn -> door\r\n");
    u2_printf("PWM=%d, approach=%u, door=%u, return=%u, left_door=%u\r\n",
              MED_CAR_TEST_CROSS_TURN_PWM,
              MED_CAR_TEST_CROSS_TURN_MAX_TICKS,
              MED_CAR_TEST_CROSS_TURN_DOOR_DISTANCE,
              MED_CAR_TEST_CROSS_TURN_RETURN_DISTANCE,
              MED_CAR_TEST_CROSS_TURN_LEFT_DOOR_DISTANCE);

    while (1) {
        for (cycle = 1U; cycle <= MED_CAR_TEST_CROSS_TURN_CYCLES; cycle++) {
            uint8_t cross_found;
            uint8_t turn_result;
            uint8_t door_found;
            uint8_t diaotou_result;

            Load(0, 0);
            u2_printf("Cycle %lu: put car on line before cross, start in 3s.\r\n",
                      (unsigned long)cycle);
            delay_ms(3000U);

            cross_found = xunxian(MED_CAR_TEST_CROSS_TURN_MAX_TICKS,
                                  MED_CAR_TEST_CROSS_TURN_PWM);
            Read_Speed();
            Load(0, 0);

            u2_printf("Cycle %lu approach %s: L=%d R=%d sum=%d\r\n",
                      (unsigned long)cycle,
                      (cross_found != 0U) ? "CROSS" : "LIMIT",
                      Encoder_Left, Encoder_Right,
                      Encoder_Left + Encoder_Right);

            if (cross_found == 0U) {
                u2_printf("Cycle %lu skip: cross not found.\r\n",
                          (unsigned long)cycle);
                delay_ms(MED_CAR_TEST_CROSS_TURN_PAUSE_MS);
                continue;
            }

            u2_printf("Cycle %lu turn RIGHT...\r\n", (unsigned long)cycle);
            turn_result = sensor_turn_right();
            u2_printf("Cycle %lu turn RIGHT result: %s\r\n",
                      (unsigned long)cycle,
                      (turn_result != 0U) ? "OK" : "TIMEOUT");

            u2_printf("Cycle %lu follow line to door, max=%u pwm=%d...\r\n",
                      (unsigned long)cycle,
                      MED_CAR_TEST_CROSS_TURN_DOOR_DISTANCE,
                      MED_CAR_TEST_CROSS_TURN_DOOR_PWM);
            door_found = xunxian_until_door(MED_CAR_TEST_CROSS_TURN_DOOR_DISTANCE,
                                            MED_CAR_TEST_CROSS_TURN_DOOR_PWM);
            u2_printf("Cycle %lu door: %s\r\n",
                      (unsigned long)cycle,
                      (door_found != 0U) ? "DETECTED" : "MAX_DISTANCE");

            stop(1);
            u2_printf("Cycle %lu waiting for drug removal...\r\n",
                      (unsigned long)cycle);
            while (MedicineCar_ReadDrugPresent() != 0U) {
                delay_ms(100U);
            }
            u2_printf("Cycle %lu drug removed.\r\n",
                      (unsigned long)cycle);

            u2_printf("Cycle %lu diaotou...\r\n", (unsigned long)cycle);
            diaotou_result = sensor_diaotou();
            u2_printf("Cycle %lu diaotou result: %s\r\n",
                      (unsigned long)cycle,
                      (diaotou_result != 0U) ? "OK" : "TIMEOUT");

            u2_printf("Cycle %lu return to cross, distance=%u...\r\n",
                      (unsigned long)cycle,
                      MED_CAR_TEST_CROSS_TURN_RETURN_DISTANCE);
            {
                uint8_t cross_on_return;
                uint8_t left_turn_result;
                uint8_t left_door_found;

                cross_on_return = xunxian(MED_CAR_TEST_CROSS_TURN_RETURN_DISTANCE,
                                          MED_CAR_TEST_CROSS_TURN_PWM);
                u2_printf("Cycle %lu return cross: %s\r\n",
                          (unsigned long)cycle,
                          (cross_on_return != 0U) ? "FOUND" : "LIMIT");

                if (cross_on_return == 0U) {
                    u2_printf("Cycle %lu skip left turn: cross not found.\r\n",
                              (unsigned long)cycle);
                    delay_ms(MED_CAR_TEST_CROSS_TURN_PAUSE_MS);
                    continue;
                }

                u2_printf("Cycle %lu turn LEFT...\r\n", (unsigned long)cycle);
                left_turn_result = sensor_turn_left();
                u2_printf("Cycle %lu turn LEFT result: %s\r\n",
                          (unsigned long)cycle,
                          (left_turn_result != 0U) ? "OK" : "TIMEOUT");

                u2_printf("Cycle %lu follow to left door, max=%u...\r\n",
                          (unsigned long)cycle,
                          MED_CAR_TEST_CROSS_TURN_LEFT_DOOR_DISTANCE);
                left_door_found = xunxian_until_door(
                    MED_CAR_TEST_CROSS_TURN_LEFT_DOOR_DISTANCE,
                    MED_CAR_TEST_CROSS_TURN_DOOR_PWM);
                u2_printf("Cycle %lu left door: %s\r\n",
                          (unsigned long)cycle,
                          (left_door_found != 0U) ? "DETECTED" : "MAX_DISTANCE");
                stop(1);
            }

            delay_ms(MED_CAR_TEST_CROSS_TURN_PAUSE_MS);
        }

        u2_printf("Cross turn round complete. Next round after pause.\r\n");
        delay_ms(MED_CAR_TEST_CROSS_TURN_PAUSE_MS);
    }
}

static void debug_sensor_door_test_loop(void)
{
    u2_printf("\r\nSensor door detection test start\r\n");
    u2_printf("Place car on branch line before dashed door zone\r\n");
    Load(0, 0);
    delay_ms(3000U);

    while (1) {
        uint8_t result;

        print_gray_snapshot("DOOR BEFORE");
        u2_printf("Driving until door dashed line, max=%u pwm=%d\r\n",
                  MED_CAR_TEST_SENSOR_DOOR_DISTANCE,
                  MED_CAR_TEST_SENSOR_DOOR_PWM);

        result = xunxian_until_door(MED_CAR_TEST_SENSOR_DOOR_DISTANCE,
                                    MED_CAR_TEST_SENSOR_DOOR_PWM);
        print_gray_snapshot("DOOR AFTER");
        u2_printf("Door detection result: %s\r\n",
                  (result != 0U) ? "DETECTED" : "MAX_DISTANCE");
        delay_ms(MED_CAR_TEST_SENSOR_DOOR_PAUSE_MS);
    }
}

static void debug_wiggle_calibration_test_loop(void)
{
    u2_printf("\r\n=== WIGGLE CALIBRATION TEST ===\r\n");
    u2_printf("PWM: L=(%d,%d) R=(%d,%d)\r\n",
              MED_CAR_RP2_WIGGLE_LEFT_PWM_LEFT,
              MED_CAR_RP2_WIGGLE_LEFT_PWM_RIGHT,
              MED_CAR_RP2_WIGGLE_RIGHT_PWM_LEFT,
              MED_CAR_RP2_WIGGLE_RIGHT_PWM_RIGHT);
    u2_printf("Target ticks: %u\r\n",
              (unsigned int)MED_CAR_RP2_WIGGLE_TICKS);
    u2_printf("Cycle: L -> print -> R(back) -> print -> pause\r\n");
    delay_ms(3000U);

    while (1) {
        int before_L, before_R;
        int delta_L, delta_R;

        Read_Speed();
        before_L = Encoder_Left;
        before_R = Encoder_Right;
        u2_printf("L: before enc L=%d R=%d\r\n", before_L, before_R);

        wiggle_by_ticks(MED_CAR_RP2_WIGGLE_LEFT_PWM_LEFT,
                        MED_CAR_RP2_WIGGLE_LEFT_PWM_RIGHT,
                        MED_CAR_RP2_WIGGLE_TICKS);

        Read_Speed();
        delta_L = Encoder_Left - before_L;
        delta_R = Encoder_Right - before_R;
        u2_printf("L: after enc L=%d R=%d | delta L=%d R=%d sum=%d\r\n",
                  Encoder_Left, Encoder_Right,
                  delta_L, delta_R,
                  (delta_L < 0 ? -delta_L : delta_L) +
                  (delta_R < 0 ? -delta_R : delta_R));

        Read_Speed();
        before_L = Encoder_Left;
        before_R = Encoder_Right;
        u2_printf("R: before enc L=%d R=%d\r\n", before_L, before_R);

        wiggle_by_ticks(MED_CAR_RP2_WIGGLE_RIGHT_PWM_LEFT,
                        MED_CAR_RP2_WIGGLE_RIGHT_PWM_RIGHT,
                        MED_CAR_RP2_WIGGLE_TICKS);

        Read_Speed();
        delta_L = Encoder_Left - before_L;
        delta_R = Encoder_Right - before_R;
        u2_printf("R: after enc L=%d R=%d | delta L=%d R=%d sum=%d\r\n",
                  Encoder_Left, Encoder_Right,
                  delta_L, delta_R,
                  (delta_L < 0 ? -delta_L : delta_L) +
                  (delta_R < 0 ? -delta_R : delta_R));

        u2_printf("--- pause %ums ---\r\n",
                  (unsigned int)MED_CAR_TEST_WIGGLE_PAUSE_MS);
        delay_ms(MED_CAR_TEST_WIGGLE_PAUSE_MS);
    }
}

void MedicineCar_RunFirmwareTestLoop(void)
{
#if MED_CAR_TEST_MODE == MED_CAR_TEST_MODE_MOTOR
    debug_motor_test_loop();
#elif MED_CAR_TEST_MODE == MED_CAR_TEST_MODE_GRAY
    debug_gray_test_loop();
#elif MED_CAR_TEST_MODE == MED_CAR_TEST_MODE_GRAY_TRACE
    debug_gray_trace_test_loop();
#elif MED_CAR_TEST_MODE == MED_CAR_TEST_MODE_LED_BEEP
    debug_led_beep_test_loop();
#elif MED_CAR_TEST_MODE == MED_CAR_TEST_MODE_ROUTE1
    debug_route_test_loop();
#elif MED_CAR_TEST_MODE == MED_CAR_TEST_MODE_ROUTE2
    debug_route_test_loop();
#elif MED_CAR_TEST_MODE == MED_CAR_TEST_MODE_ENCODER
    debug_encoder_test_loop();
#elif MED_CAR_TEST_MODE == MED_CAR_TEST_MODE_WHEEL_MATCH
    debug_wheel_match_test_loop();
#elif MED_CAR_TEST_MODE == MED_CAR_TEST_MODE_VISION_UART
    debug_vision_uart_test_loop();
#elif MED_CAR_TEST_MODE == MED_CAR_TEST_MODE_DRUG_SENSOR
    debug_drug_sensor_test_loop();
#elif MED_CAR_TEST_MODE == MED_CAR_TEST_MODE_SENSOR_DOOR
    debug_sensor_door_test_loop();
#elif MED_CAR_TEST_MODE == MED_CAR_TEST_MODE_CROSS_TURN
    debug_cross_turn_test_loop();
#elif MED_CAR_TEST_MODE == MED_CAR_TEST_MODE_ROUTE3_8
    debug_route3_8_test_loop();
#elif MED_CAR_TEST_MODE == MED_CAR_TEST_MODE_WIGGLE_CALIB
    debug_wiggle_calibration_test_loop();
#elif MED_CAR_TEST_MODE == MED_CAR_TEST_MODE_RP2
    debug_rp2_test_loop();
#else
    return;
#endif
}
