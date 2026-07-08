#include "medicine_car_tests.h"

#include <stdint.h>

#include "medicine_car_app.h"
#include "medicine_car_config.h"
#include "medicine_car_platform.h"

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
    uint8_t io1 = MedicineCar_ReadLineSensor(MED_CAR_SENSOR_IO1);
    uint8_t io2 = MedicineCar_ReadLineSensor(MED_CAR_SENSOR_IO2);
    uint8_t io3 = MedicineCar_ReadLineSensor(MED_CAR_SENSOR_IO3);
    uint8_t io4 = MedicineCar_ReadLineSensor(MED_CAR_SENSOR_IO4);
    uint8_t io5 = MedicineCar_ReadLineSensor(MED_CAR_SENSOR_IO5);
    uint8_t io6 = MedicineCar_ReadLineSensor(MED_CAR_SENSOR_IO6);
    uint8_t io7 = MedicineCar_ReadLineSensor(MED_CAR_SENSOR_IO7);
    uint8_t io8 = MedicineCar_ReadLineSensor(MED_CAR_SENSOR_IO8);

    u2_printf("%s: %u %u %u %u %u %u %u %u\r\n",
              prefix, io1, io2, io3, io4, io5, io6, io7, io8);
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
        xunxian(MED_CAR_TEST_GRAY_TRACE_DISTANCE, MED_CAR_TEST_GRAY_TRACE_PWM);
        Load(0, 0);
        print_gray_snapshot("GRAY AFTER TRACE");
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

static void debug_route_test_loop(uint8_t target_room)
{
    u2_printf("\r\nRoute %u test start\r\n", target_room);
    MedicineCar_RequestRun(target_room);
    MedicineCar_Step();
    Load(0, 0);
    u2_printf("Route %u test done, idle now\r\n", target_room);

    while (1) {
        MedicineCarPlatform_Service();
        delay_ms(1000U);
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
    debug_route_test_loop(1U);
#elif MED_CAR_TEST_MODE == MED_CAR_TEST_MODE_ROUTE2
    debug_route_test_loop(2U);
#elif MED_CAR_TEST_MODE == MED_CAR_TEST_MODE_ENCODER
    debug_encoder_test_loop();
#elif MED_CAR_TEST_MODE == MED_CAR_TEST_MODE_WHEEL_MATCH
    debug_wheel_match_test_loop();
#else
    return;
#endif
}
