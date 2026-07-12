#include "medicine_car_tests.h"

#include <stdint.h>

#include "medicine_car_config.h"
#include "medicine_car_platform.h"
#include "medicine_car_return.h"
#include "medicine_car_vision.h"

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
    uint8_t target = 0U;

    Return_Init();
    N = 0;
    Load(0, 0);
    MedicineCar_SetGreenLed(0U);

    u2_printf("\r\n=== ROUTE 1/2 TEST (return module) ===\r\n");

    u2_printf("[Phase 1] waiting for vision (room 1 or 2)...\r\n");
    while (target == 0U) {
        MedicineCarVisionResult result;

        if (MedicineCarVision_Request(&result,
                                      MED_CAR_VISION_UART_TIMEOUT_MS) != 0U) {
            uint8_t i;

            u2_printf("[Phase 1] vision digits:");
            for (i = 0U; i < result.count; i++) {
                u2_printf(" %u", result.digits[i]);
                if ((result.digits[i] == 1U) || (result.digits[i] == 2U)) {
                    target = result.digits[i];
                }
            }
            u2_printf("\r\n");
        } else {
            u2_printf("[Phase 1] vision err: %s\r\n",
                      MedicineCarVision_LastStatusText());
        }

        if (target == 0U) {
            MedicineCarPlatform_Service();
            delay_ms(MED_CAR_TEST_VISION_PRINT_MS);
        }
    }
    u2_printf("[Phase 1] done: target = %u\r\n", target);

    u2_printf("[Phase 2] waiting for drug load...\r\n");
    while (MedicineCar_ReadDrugPresent() == 0U) {
        MedicineCarPlatform_Service();
        delay_ms(100U);
    }
    u2_printf("[Phase 2] done: drug loaded\r\n");

    u2_printf("[Phase 3] xunxian to intersection, distance=%u\r\n",
              MED_CAR_DISTANCE_FIRST_CHECK);
    xunxian(MED_CAR_DISTANCE_FIRST_CHECK, 4000);

    if (target == 1U) {
        u2_printf("[Phase 3] push LEFT, turn into left branch\r\n");
        Return_Push(RETURN_DIR_LEFT);
        sensor_turn_left();
    } else {
        u2_printf("[Phase 3] push RIGHT, turn into right branch\r\n");
        Return_Push(RETURN_DIR_RIGHT);
        sensor_turn_right();
    }

    u2_printf("[Phase 4] xunxian to door, max=%u\r\n",
              MED_CAR_DISTANCE_MID);
    xunxian_until_door(MED_CAR_DISTANCE_MID, 4000);

    u2_printf("[Phase 5] waiting for drug removal...\r\n");
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
    u2_printf("[Phase 5] done: drug removed\r\n");

    u2_printf("[Phase 6] Return_Execute: diaotou + reverse stack + head home\r\n");
    Return_Execute(MED_CAR_DISTANCE_FIRST_CHECK, MED_CAR_TEST_GRAY_TRACE_PWM);

    u2_printf("=== TEST COMPLETE: returned to start ===\r\n");
    MedicineCar_SetGreenLed(1U);
    Load(0, 0);

    while (1) {
        MedicineCarPlatform_Service();
        delay_ms(1000U);
    }
}

static void print_vision_digits(const MedicineCarVisionResult *result)
{
    uint8_t index;

    if (result == 0) {
        return;
    }

    u2_printf("VISION OK count=%u digits=", result->count);
    for (index = 0U; index < result->count; index++) {
        u2_printf("%u", result->digits[index]);
        if (index != (uint8_t)(result->count - 1U)) {
            u2_printf(",");
        }
    }
    u2_printf("\r\n");
}

static uint8_t find_route3_8_target(const MedicineCarVisionResult *result)
{
    uint8_t index;

    if (result == 0) {
        return 0U;
    }

    for (index = 0U; index < result->count; index++) {
        if ((result->digits[index] >= 3U) && (result->digits[index] <= 8U)) {
            return result->digits[index];
        }
    }

    return 0U;
}

static void debug_route3_8_test_loop(void)
{
    uint8_t cached_target = 0U;
    uint8_t success;
    uint32_t drug_wait_ms = 0U;

    Load(0, 0);
    MedicineCar_SetRecognizedNumber(0U);
    u2_printf("\r\n=== ROUTE3_8 DYNAMIC TEST ===\r\n");
    u2_printf("Waiting for vision target 3..8\r\n");

    while (cached_target == 0U) {
        MedicineCarVisionResult result;

        if (MedicineCarVision_Request(&result,
                                      MED_CAR_VISION_UART_TIMEOUT_MS) != 0U) {
            print_vision_digits(&result);
            cached_target = find_route3_8_target(&result);
            if (cached_target != 0U) {
                u2_printf("CACHED target=%u\r\n", cached_target);
            } else {
                u2_printf("No 3..8 target in vision result\r\n");
            }
        } else {
            u2_printf("VISION ERR %s raw='%s'\r\n",
                      MedicineCarVision_LastStatusText(),
                      MedicineCarVision_LastLine());
        }

        if (cached_target == 0U) {
            MedicineCarPlatform_Service();
            delay_ms(MED_CAR_TEST_R3_8_VISION_INTERVAL_MS);
        }
    }

    u2_printf("Target cached=%u, waiting for drug PRESENT\r\n",
              cached_target);
    u2_printf("DRUG raw=%s interpreted=%s\r\n",
              (MedicineCar_ReadDrugSensorRaw() != 0U) ? "HIGH" : "LOW",
              (MedicineCar_ReadDrugPresent() != 0U) ? "PRESENT" : "ABSENT");

    while (MedicineCar_ReadDrugPresent() == 0U) {
        MedicineCarPlatform_Service();
        delay_ms(MED_CAR_TEST_R3_8_DRUG_POLL_MS);
        drug_wait_ms += MED_CAR_TEST_R3_8_DRUG_POLL_MS;
        if (drug_wait_ms >= MED_CAR_TEST_R3_8_IDLE_PRINT_MS) {
            drug_wait_ms = 0U;
            u2_printf("Waiting drug, raw=%s interpreted=%s\r\n",
                      (MedicineCar_ReadDrugSensorRaw() != 0U) ? "HIGH" : "LOW",
                      (MedicineCar_ReadDrugPresent() != 0U) ? "PRESENT" : "ABSENT");
        }
    }

    u2_printf("Drug PRESENT, start route3_8 target=%u\r\n", cached_target);
    success = MedicineCar_RunRoute3To8Test(cached_target);
    Load(0, 0);
    if (success != 0U) {
        MedicineCar_SetGreenLed(1U);
    }

    u2_printf("ROUTE3_8 test complete success=%u, idle loop\r\n", success);
    while (1) {
        MedicineCarPlatform_Service();
        u2_printf("ROUTE3_8 idle success=%u target=%u\r\n",
                  success,
                  cached_target);
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
    u2_printf("\r\nVision UART test start, USART3 115200 8N1\r\n");
    u2_printf("Protocol: OpenMV streams AA BB left right CC every 100 ms\r\n");

    while (1) {
        MedicineCarVisionResult result;

        if (MedicineCarVision_Request(&result,
                                      MED_CAR_VISION_UART_TIMEOUT_MS) != 0U) {
            uint8_t index;

            u2_printf("VISION OK raw='%s' count=%u digits=",
                      MedicineCarVision_LastLine(),
                      result.count);
            for (index = 0U; index < result.count; index++) {
                u2_printf("%u", result.digits[index]);
                if (index != (uint8_t)(result.count - 1U)) {
                    u2_printf(",");
                }
            }
            u2_printf("\r\n");
        } else {
            u2_printf("VISION ERR %s raw='%s'\r\n",
                      MedicineCarVision_LastStatusText(),
                      MedicineCarVision_LastLine());
        }

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
#else
    return;
#endif
}
