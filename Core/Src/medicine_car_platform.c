#include "medicine_car_platform.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "gpio.h"
#include "medicine_car_config.h"
#include "medicine_car_detect.h"
#include "tim.h"
#include "usart.h"

volatile int N = 0;
volatile int Speed_L = 0;
volatile int Speed_R = 0;
volatile int Pwm_L = 0;
volatile int Pwm_R = 0;
volatile int Time_s = 0;
volatile int Run_Flag = 0;
volatile int Number = 0;
volatile int aim = 0;
volatile int BluetoohAim = 0;
volatile int Encoder_Left = 0;
volatile int Encoder_Right = 0;
int NumBuff[2] = {0, 0};
int XBuff[2] = {0, 0};
char num[50];

static uint32_t service_tick_ms;
static uint8_t gray_sensor_values[8] = {
    MED_CAR_GRAY_WHITE_LEVEL, MED_CAR_GRAY_WHITE_LEVEL,
    MED_CAR_GRAY_WHITE_LEVEL, MED_CAR_GRAY_WHITE_LEVEL,
    MED_CAR_GRAY_WHITE_LEVEL, MED_CAR_GRAY_WHITE_LEVEL,
    MED_CAR_GRAY_WHITE_LEVEL, MED_CAR_GRAY_WHITE_LEVEL
};
static uint16_t last_left_encoder_count;
static uint16_t last_right_encoder_count;

void move_forward_timed(uint32_t duration_ms, int pwm);
static int abs_int_local(int value);

static uint8_t read_pin(GPIO_TypeDef *port, uint16_t pin)
{
    return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET) ? 1U : 0U;
}

static void write_pin(GPIO_TypeDef *port, uint16_t pin, uint8_t on)
{
    HAL_GPIO_WritePin(port, pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static int abs_int_local(int value)
{
    return (value < 0) ? -value : value;
}

static void gray_clock_high_delay(void)
{
    volatile uint32_t cycles = MED_CAR_GRAY_SERIAL_CLK_HIGH_CYCLES;

    while (cycles-- > 0U) {
        __NOP();
    }
}

static uint8_t gray_read_serial_status(void)
{
    uint8_t bit;
    uint8_t status = 0U;

    write_pin(GRAY_CLK_GPIO_Port, GRAY_CLK_Pin, 0U);
    HAL_Delay(MED_CAR_GRAY_SERIAL_FRAME_SYNC_MS);

    for (bit = 0U; bit < 8U; bit++) {
        write_pin(GRAY_CLK_GPIO_Port, GRAY_CLK_Pin, 0U);
        if (read_pin(GRAY_DAT_GPIO_Port, GRAY_DAT_Pin) == MED_CAR_GRAY_WHITE_LEVEL) {
            status |= (uint8_t)(1U << bit);
        }
        write_pin(GRAY_CLK_GPIO_Port, GRAY_CLK_Pin, 1U);
        gray_clock_high_delay();
    }

    write_pin(GRAY_CLK_GPIO_Port, GRAY_CLK_Pin, 0U);
    return status;
}

static void gray_update_all(void)
{
    uint8_t channel;
    uint8_t status = gray_read_serial_status();

    for (channel = 0U; channel < 8U; channel++) {
        gray_sensor_values[channel] = (uint8_t)((status >> channel) & 0x01U);
    }
}

static uint32_t scale_code_pwm(int pwm)
{
    uint32_t value = (pwm < 0) ? (uint32_t)(-pwm) : (uint32_t)pwm;

    if (value > (uint32_t)MED_CAR_CODE_PWM_MAX) {
        value = (uint32_t)MED_CAR_CODE_PWM_MAX;
    }

    return (value * MED_CAR_H7_PWM_PERIOD) / (uint32_t)MED_CAR_CODE_PWM_MAX;
}

static int trim_run_pwm(int pwm, int numerator)
{
    return (pwm * numerator) / MED_CAR_PWM_TRIM_DEN;
}

static int get_virtual_encoder_delta(int pwm)
{
    return (pwm < 0) ? -80 : ((pwm > 0) ? 80 : 0);
}

static int get_encoder_delta(TIM_HandleTypeDef *htim, uint16_t *last_count)
{
    uint16_t now = (uint16_t)__HAL_TIM_GET_COUNTER(htim);
    int delta = (int)((int16_t)(now - *last_count));

    *last_count = now;
    return delta;
}

static void uart_vprintf(UART_HandleTypeDef *huart, const char *fmt, va_list args)
{
    char buffer[96];
    int len = vsnprintf(buffer, sizeof(buffer), fmt, args);

    if (len < 0) {
        return;
    }
    if (len > (int)sizeof(buffer)) {
        len = (int)sizeof(buffer);
    }

    (void)HAL_UART_Transmit(huart, (uint8_t *)buffer, (uint16_t)len, 50U);
}

void MedicineCarPlatform_Init(void)
{
#if MED_CAR_ENABLE_ENCODER
    (void)HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    (void)HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
    MedicineCar_ResetEncoders();
#endif
    (void)HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    Load(0, 0);
    MedicineCar_SetRedLed(0U);
    MedicineCar_SetYellowLed(0U);
    MedicineCar_SetGreenLed(0U);
}

void MedicineCarPlatform_Service(void)
{
    uint32_t now = HAL_GetTick();

    if ((now - service_tick_ms) >= 1000U) {
        service_tick_ms = now;
        if (Time_s < 60) {
            Time_s++;
        } else {
            Time_s = 0;
        }
    }
}

uint8_t MedicineCar_ReadLineSensor(uint8_t index)
{
    gray_update_all();

    switch (index) {
    case MED_CAR_SENSOR_IO1:
        return gray_sensor_values[0];
    case MED_CAR_SENSOR_IO2:
        return gray_sensor_values[1];
    case MED_CAR_SENSOR_IO3:
        return gray_sensor_values[2];
    case MED_CAR_SENSOR_IO4:
        return gray_sensor_values[3];
    case MED_CAR_SENSOR_IO5:
        return gray_sensor_values[4];
    case MED_CAR_SENSOR_IO6:
        return gray_sensor_values[5];
    case MED_CAR_SENSOR_IO7:
        return gray_sensor_values[6];
    case MED_CAR_SENSOR_IO8:
        return gray_sensor_values[7];
    default:
        return MED_CAR_DEFAULT_MISSING_SENSOR;
    }
}

void MedicineCar_ReadLineSensors(uint8_t values[8])
{
    uint8_t channel;

    if (values == NULL) {
        return;
    }

    gray_update_all();
    for (channel = 0U; channel < 8U; channel++) {
        values[channel] = gray_sensor_values[channel];
    }
}

uint8_t MedicineCar_ReadDrugPresent(void)
{
#if MED_CAR_ENABLE_DRUG_SENSOR
    return (MedicineCar_ReadDrugSensorRaw() == MED_CAR_DRUG_PRESENT_LEVEL) ? 1U : 0U;
#else
    return MED_CAR_DEFAULT_DRUG_PRESENT;
#endif
}

uint8_t MedicineCar_ReadDrugSensorRaw(void)
{
#if MED_CAR_ENABLE_DRUG_SENSOR
    return read_pin(DRUG_SENSOR_GPIO_Port, DRUG_SENSOR_Pin);
#else
    return MED_CAR_DEFAULT_DRUG_PRESENT;
#endif
}

void MedicineCar_SetRedLed(uint8_t on)
{
    write_pin(CAR_LED_GPIO_Port, CAR_LED_Pin, on);
}

void MedicineCar_SetYellowLed(uint8_t on)
{
    HAL_GPIO_WritePin(CAR_BEEP_GPIO_Port,
                      CAR_BEEP_Pin,
                      on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void MedicineCar_SetGreenLed(uint8_t on)
{
    write_pin(CAR_LED_GPIO_Port, CAR_LED_Pin, on);
}

void delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}

void MedicineCar_ResetEncoders(void)
{
#if MED_CAR_ENABLE_ENCODER
    __HAL_TIM_SET_COUNTER(&htim1, 0U);
    __HAL_TIM_SET_COUNTER(&htim8, 0U);
    last_left_encoder_count = 0U;
    last_right_encoder_count = 0U;
#endif
    Encoder_Left = 0;
    Encoder_Right = 0;
    Speed_L = 0;
    Speed_R = 0;
}

void Read_Speed(void)
{
#if MED_CAR_ENABLE_ENCODER
    Speed_L = Read_Encoder(1U);
    Speed_R = Read_Encoder(8U);
    Encoder_Left += Speed_L;
    Encoder_Right += Speed_R;
#else
    Speed_L = get_virtual_encoder_delta(Pwm_L);
    Speed_R = get_virtual_encoder_delta(Pwm_R);
    Encoder_Left += Speed_L;
    Encoder_Right += Speed_R;
#endif
}

int Read_Encoder(uint8_t timx)
{
#if MED_CAR_ENABLE_ENCODER
    int delta;

    if (timx == 1U) {
        delta = get_encoder_delta(&htim1, &last_left_encoder_count);
#if MED_CAR_LEFT_ENCODER_INVERT
        delta = -delta;
#endif
        return delta;
    }

    if (timx == 8U) {
        delta = get_encoder_delta(&htim8, &last_right_encoder_count);
#if MED_CAR_RIGHT_ENCODER_INVERT
        delta = -delta;
#endif
        return delta;
    }
#else
    if (timx == 1U) {
        return get_virtual_encoder_delta(Pwm_L);
    }
    if (timx == 8U) {
        return get_virtual_encoder_delta(Pwm_R);
    }
#endif

    return 0;
}

void Load(int moto1, int moto2)
{
    uint32_t left_pwm;
    uint32_t right_pwm;

    Limit(&moto1, &moto2);
#if MED_CAR_LEFT_MOTOR_INVERT
    moto1 = -moto1;
#endif
#if MED_CAR_RIGHT_MOTOR_INVERT
    moto2 = -moto2;
#endif
    Pwm_L = moto1;
    Pwm_R = moto2;

    if (moto1 > 0) {
        write_pin(CAR_LEFT_AIN1_GPIO_Port, CAR_LEFT_AIN1_Pin, 1U);
        write_pin(CAR_LEFT_AIN2_GPIO_Port, CAR_LEFT_AIN2_Pin, 0U);
    } else if (moto1 < 0) {
        write_pin(CAR_LEFT_AIN1_GPIO_Port, CAR_LEFT_AIN1_Pin, 0U);
        write_pin(CAR_LEFT_AIN2_GPIO_Port, CAR_LEFT_AIN2_Pin, 1U);
    } else {
        write_pin(CAR_LEFT_AIN1_GPIO_Port, CAR_LEFT_AIN1_Pin, 0U);
        write_pin(CAR_LEFT_AIN2_GPIO_Port, CAR_LEFT_AIN2_Pin, 0U);
    }

    if (moto2 > 0) {
        write_pin(CAR_RIGHT_BIN1_GPIO_Port, CAR_RIGHT_BIN1_Pin, 1U);
        write_pin(CAR_RIGHT_BIN2_GPIO_Port, CAR_RIGHT_BIN2_Pin, 0U);
    } else if (moto2 < 0) {
        write_pin(CAR_RIGHT_BIN1_GPIO_Port, CAR_RIGHT_BIN1_Pin, 0U);
        write_pin(CAR_RIGHT_BIN2_GPIO_Port, CAR_RIGHT_BIN2_Pin, 1U);
    } else {
        write_pin(CAR_RIGHT_BIN1_GPIO_Port, CAR_RIGHT_BIN1_Pin, 0U);
        write_pin(CAR_RIGHT_BIN2_GPIO_Port, CAR_RIGHT_BIN2_Pin, 0U);
    }

    left_pwm = scale_code_pwm(moto1);
    right_pwm = scale_code_pwm(moto2);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, left_pwm);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, right_pwm);
}

void Limit(int *motoA, int *motoB)
{
    if (*motoA > MED_CAR_CODE_PWM_MAX) {
        *motoA = MED_CAR_CODE_PWM_MAX;
    }
    if (*motoA < -MED_CAR_CODE_PWM_MAX) {
        *motoA = -MED_CAR_CODE_PWM_MAX;
    }
    if (*motoB > MED_CAR_CODE_PWM_MAX) {
        *motoB = MED_CAR_CODE_PWM_MAX;
    }
    if (*motoB < -MED_CAR_CODE_PWM_MAX) {
        *motoB = -MED_CAR_CODE_PWM_MAX;
    }
}

void stop(int stoptime)
{
    Load(0, 0);
    HAL_Delay((uint32_t)stoptime * MED_CAR_STOP_TICK_MS);
}

void turn_left(void)
{
    Load(-2000, 2000);
    HAL_Delay(MED_CAR_TURN_LEFT_MS);
    stop(1);
}

void turn_right(void)
{
    Load(3000, -3000);
    HAL_Delay(MED_CAR_TURN_RIGHT_MS);
    stop(1);
}

void diaotou(void)
{
    Load(-3000, 3000);
    HAL_Delay(MED_CAR_TURN_AROUND_MS);
    stop(1);
}

uint8_t xunxian(uint16_t roadsum, int pwm)
{
    static const int8_t line_weights[8] = {3, 2, 1, 0, 0, -1, -2, -3};
    uint32_t guard = 0U;
    int last_direction = 0;
    int correction;

    MedicineCar_ResetEncoders();

    {
        uint8_t warmup_gray[8];
        int warmup_channel;
        int warmup_sum = 0;

        MedicineCar_ReadLineSensors(warmup_gray);
        for (warmup_channel = 0; warmup_channel < 8; warmup_channel++) {
            if (warmup_gray[warmup_channel] == MED_CAR_GRAY_BLACK_LEVEL) {
                warmup_sum += line_weights[warmup_channel];
            }
        }
        if (warmup_sum > 0) {
            last_direction = 1;
        } else if (warmup_sum < 0) {
            last_direction = -1;
        }
    }

    while (((Encoder_Left + Encoder_Right) < (int)roadsum) && (guard < 800U)) {
        uint8_t gray[8];
        uint8_t io1;
        uint8_t io2;
        uint8_t io3;
        uint8_t io4;
        uint8_t io5;
        uint8_t io6;
        uint8_t io7;
        uint8_t io8;
        uint8_t line_seen;
        uint8_t channel;
        uint8_t black_count;
        int weighted_sum;

        MedicineCar_ReadLineSensors(gray);

        if ((gray[1] == MED_CAR_GRAY_BLACK_LEVEL) &&
            (gray[2] == MED_CAR_GRAY_BLACK_LEVEL) &&
            (gray[5] == MED_CAR_GRAY_BLACK_LEVEL) &&
            (gray[6] == MED_CAR_GRAY_BLACK_LEVEL)) {
            stop(1);
            return 1U;
        }

        io1 = gray[0];
        io2 = gray[1];
        io3 = gray[2];
        io4 = gray[3];
        io5 = gray[4];
        io6 = gray[5];
        io7 = gray[6];
        io8 = gray[7];
        line_seen = ((io1 == MED_CAR_GRAY_BLACK_LEVEL) ||
                     (io2 == MED_CAR_GRAY_BLACK_LEVEL) ||
                     (io3 == MED_CAR_GRAY_BLACK_LEVEL) ||
                     (io4 == MED_CAR_GRAY_BLACK_LEVEL) ||
                     (io5 == MED_CAR_GRAY_BLACK_LEVEL) ||
                     (io6 == MED_CAR_GRAY_BLACK_LEVEL) ||
                     (io7 == MED_CAR_GRAY_BLACK_LEVEL) ||
                     (io8 == MED_CAR_GRAY_BLACK_LEVEL)) ? 1U : 0U;

        correction = 0;
        if (line_seen != 0U) {
            black_count = 0U;
            weighted_sum = 0;
            for (channel = 0U; channel < 8U; channel++) {
                if (gray[channel] == MED_CAR_GRAY_BLACK_LEVEL) {
                    weighted_sum += line_weights[channel];
                    black_count++;
                }
            }

            if (black_count > 0U) {
                correction = (weighted_sum * MED_CAR_LINE_PWM_ADJUST_1) /
                             (int)black_count;
                if (correction > 0) {
                    last_direction = 1;
                } else if (correction < 0) {
                    last_direction = -1;
                } else {
                    last_direction = 0;
                }
            }
        } else if (last_direction > 0) {
            correction += MED_CAR_LINE_PWM_ADJUST_3;
        } else if (last_direction < 0) {
            correction -= MED_CAR_LINE_PWM_ADJUST_3;
        }

        Load(trim_run_pwm(pwm - correction, MED_CAR_LEFT_PWM_TRIM_NUM),
             trim_run_pwm(pwm + correction, MED_CAR_RIGHT_PWM_TRIM_NUM));
        HAL_Delay(10U);
        Read_Speed();
        guard++;
    }

    stop(1);
    return 0U;
}

void zhao_bai(uint16_t roadsum, int pwm)
{
    xunxian(roadsum, pwm);
}

static uint8_t search_line_rotating_with_min_ticks(int left_pwm,
                                                   int right_pwm,
                                                   uint16_t min_turn_ticks,
                                                   uint16_t min_delay_ms,
                                                   uint16_t timeout_ms,
                                                   uint8_t (*aligned_fn)(void))
{
    uint32_t elapsed;
    uint8_t aligned_confirm = 0U;
    uint8_t min_angle_reached = 0U;

    MedicineCar_ResetEncoders();
    Load(left_pwm, right_pwm);
    HAL_Delay(min_delay_ms);
    elapsed = (uint32_t)min_delay_ms;

    while (elapsed < (uint32_t)timeout_ms) {
        Read_Speed();

        if (min_angle_reached == 0U) {
            if ((uint32_t)(abs_int_local(Encoder_Left) +
                           abs_int_local(Encoder_Right)) >=
                (uint32_t)min_turn_ticks) {
                min_angle_reached = 1U;
            }
        } else {
            if (aligned_fn() != 0U) {
                aligned_confirm++;
                if (aligned_confirm >= MED_CAR_SENSOR_TURN_CONFIRM_CNT) {
                    stop(1);
                    return 1U;
                }
            } else {
                aligned_confirm = 0U;
            }
        }

        HAL_Delay(MED_CAR_TURN_POLL_MS);
        elapsed += (uint32_t)MED_CAR_TURN_POLL_MS;
    }

    stop(1);
    return 0U;
}

uint8_t search_line_rotating(int left_pwm, int right_pwm,
                             uint16_t min_delay_ms, uint16_t timeout_ms,
                             uint8_t (*aligned_fn)(void))
{
    return search_line_rotating_with_min_ticks(left_pwm,
                                               right_pwm,
                                               MED_CAR_SENSOR_TURN_LEFT_MIN_TICKS,
                                               min_delay_ms,
                                               timeout_ms,
                                               aligned_fn);
}

uint8_t sensor_turn_left(void)
{
    // move_forward_timed(MED_CAR_CROSS_ADVANCE_MS,
    //                    MED_CAR_CROSS_ADVANCE_PWM);
    return search_line_rotating_with_min_ticks(MED_CAR_TURN_LEFT_LEFT_PWM,
                                               MED_CAR_TURN_LEFT_RIGHT_PWM,
                                               MED_CAR_SENSOR_TURN_LEFT_MIN_TICKS,
                                               MED_CAR_TURN_MIN_MS,
                                               MED_CAR_TURN_TIMEOUT_MS,
                                               is_line_left);
}

uint8_t sensor_turn_right(void)
{
    // move_forward_timed(MED_CAR_CROSS_ADVANCE_MS,
    //                    MED_CAR_CROSS_ADVANCE_PWM);
    return search_line_rotating_with_min_ticks(MED_CAR_TURN_RIGHT_LEFT_PWM,
                                               MED_CAR_TURN_RIGHT_RIGHT_PWM,
                                               MED_CAR_SENSOR_TURN_RIGHT_MIN_TICKS,
                                               MED_CAR_TURN_MIN_MS,
                                               MED_CAR_TURN_TIMEOUT_MS,
                                               is_line_right);
}

uint8_t sensor_diaotou(void)
{
    return search_line_rotating_with_min_ticks(MED_CAR_DIAOTOU_LEFT_PWM,
                                               MED_CAR_DIAOTOU_RIGHT_PWM,
                                               MED_CAR_SENSOR_DIAOTOU_MIN_TICKS,
                                               MED_CAR_DIAOTOU_MIN_MS,
                                               MED_CAR_DIAOTOU_TIMEOUT_MS,
                                               is_line_center);
}

uint8_t xunxian_until_door(uint16_t max_distance, int pwm)
{
    static const int8_t line_weights[8] = {3, 2, 1, 0, 0, -1, -2, -3};
    uint32_t guard = 0U;
    int last_direction = 0;
    uint8_t confirm_count = 0U;
    uint8_t miss_count = 0U;
    uint8_t door_region = 0U;
    int correction;

    MedicineCar_ResetEncoders();

    {
        uint8_t warmup_gray[8];
        int warmup_channel;
        int warmup_sum = 0;

        MedicineCar_ReadLineSensors(warmup_gray);
        for (warmup_channel = 0; warmup_channel < 8; warmup_channel++) {
            if (warmup_gray[warmup_channel] == MED_CAR_GRAY_BLACK_LEVEL) {
                warmup_sum += line_weights[warmup_channel];
            }
        }
        if (warmup_sum > 0) {
            last_direction = 1;
        } else if (warmup_sum < 0) {
            last_direction = -1;
        }
    }

    while (((Encoder_Left + Encoder_Right) < (int)max_distance) &&
           (guard < MED_CAR_DOOR_GUARD_MAX)) {
        uint8_t gray[8];
        uint8_t line_seen;
        uint8_t channel;
        uint8_t black_count;
        int weighted_sum;

        MedicineCar_ReadLineSensors(gray);

        black_count = 0U;
        for (channel = 0U; channel < 8U; channel++) {
            if (gray[channel] == MED_CAR_GRAY_BLACK_LEVEL) {
                black_count++;
            }
        }

        if (black_count >= MED_CAR_DOOR_BLACK_MIN) {
            door_region = 1U;
            miss_count = 0U;
            confirm_count++;
            if (confirm_count >= MED_CAR_DOOR_CONFIRM_CNT) {
                stop(1);
                return 1U;
            }
        } else if (door_region != 0U) {
            uint8_t mid_black = 0U;
            uint8_t ch;

            for (ch = 2U; ch <= 5U; ch++) {
                if (gray[ch] == MED_CAR_GRAY_BLACK_LEVEL) {
                    mid_black++;
                }
            }

            if (mid_black >= 2U &&
                gray[0] == MED_CAR_GRAY_WHITE_LEVEL &&
                gray[1] == MED_CAR_GRAY_WHITE_LEVEL &&
                gray[6] == MED_CAR_GRAY_WHITE_LEVEL &&
                gray[7] == MED_CAR_GRAY_WHITE_LEVEL) {
                miss_count = 0U;
                confirm_count++;
                if (confirm_count >= MED_CAR_DOOR_CONFIRM_CNT) {
                    stop(1);
                    return 1U;
                }
            } else {
                miss_count++;
                if (miss_count >= MED_CAR_DOOR_MISS_MAX) {
                    door_region = 0U;
                    confirm_count = 0U;
                    miss_count = 0U;
                }
            }
        }

        line_seen = ((gray[0] == MED_CAR_GRAY_BLACK_LEVEL) ||
                     (gray[1] == MED_CAR_GRAY_BLACK_LEVEL) ||
                     (gray[2] == MED_CAR_GRAY_BLACK_LEVEL) ||
                     (gray[3] == MED_CAR_GRAY_BLACK_LEVEL) ||
                     (gray[4] == MED_CAR_GRAY_BLACK_LEVEL) ||
                     (gray[5] == MED_CAR_GRAY_BLACK_LEVEL) ||
                     (gray[6] == MED_CAR_GRAY_BLACK_LEVEL) ||
                     (gray[7] == MED_CAR_GRAY_BLACK_LEVEL)) ? 1U : 0U;

        correction = 0;
        if (line_seen != 0U) {
            black_count = 0U;
            weighted_sum = 0;
            for (channel = 0U; channel < 8U; channel++) {
                if (gray[channel] == MED_CAR_GRAY_BLACK_LEVEL) {
                    weighted_sum += line_weights[channel];
                    black_count++;
                }
            }

            if (black_count > 0U) {
                correction = (weighted_sum * MED_CAR_LINE_PWM_ADJUST_1) /
                             (int)black_count;
                if (correction > 0) {
                    last_direction = 1;
                } else if (correction < 0) {
                    last_direction = -1;
                } else {
                    last_direction = 0;
                }
            }
        } else if (last_direction > 0) {
            correction += MED_CAR_LINE_PWM_ADJUST_3;
        } else if (last_direction < 0) {
            correction -= MED_CAR_LINE_PWM_ADJUST_3;
        }

        Load(trim_run_pwm(pwm - correction, MED_CAR_LEFT_PWM_TRIM_NUM),
             trim_run_pwm(pwm + correction, MED_CAR_RIGHT_PWM_TRIM_NUM));
        HAL_Delay(10U);
        Read_Speed();
        guard++;
    }

    stop(1);
    return 0U;
}

static uint8_t fork_detected(const uint8_t gray[8])
{
    uint8_t left  = (gray[0] == MED_CAR_GRAY_BLACK_LEVEL ||
                     gray[1] == MED_CAR_GRAY_BLACK_LEVEL);
    uint8_t right = (gray[6] == MED_CAR_GRAY_BLACK_LEVEL ||
                     gray[7] == MED_CAR_GRAY_BLACK_LEVEL);
    return (left || right) ? 1U : 0U;
}

uint8_t xunxian_until_fork(uint16_t max_distance, int pwm)
{
    static const int8_t line_weights[8] = {3, 2, 1, 0, 0, -1, -2, -3};
    uint32_t guard = 0U;
    int last_direction = 0;
    int correction;

    MedicineCar_ResetEncoders();

    {
        uint8_t warmup_gray[8];
        int warmup_channel;
        int warmup_sum = 0;

        MedicineCar_ReadLineSensors(warmup_gray);
        for (warmup_channel = 0; warmup_channel < 8; warmup_channel++) {
            if (warmup_gray[warmup_channel] == MED_CAR_GRAY_BLACK_LEVEL) {
                warmup_sum += line_weights[warmup_channel];
            }
        }
        if (warmup_sum > 0) {
            last_direction = 1;
        } else if (warmup_sum < 0) {
            last_direction = -1;
        }
    }

    while (((Encoder_Left + Encoder_Right) < (int)max_distance) &&
           (guard < MED_CAR_DOOR_GUARD_MAX)) {
        uint8_t gray[8];
        uint8_t line_seen;
        uint8_t channel;
        uint8_t black_count;
        int weighted_sum;

        MedicineCar_ReadLineSensors(gray);

        if (fork_detected(gray) != 0U) {
            stop(1);
            return 1U;
        }

        line_seen = ((gray[0] == MED_CAR_GRAY_BLACK_LEVEL) ||
                     (gray[1] == MED_CAR_GRAY_BLACK_LEVEL) ||
                     (gray[2] == MED_CAR_GRAY_BLACK_LEVEL) ||
                     (gray[3] == MED_CAR_GRAY_BLACK_LEVEL) ||
                     (gray[4] == MED_CAR_GRAY_BLACK_LEVEL) ||
                     (gray[5] == MED_CAR_GRAY_BLACK_LEVEL) ||
                     (gray[6] == MED_CAR_GRAY_BLACK_LEVEL) ||
                     (gray[7] == MED_CAR_GRAY_BLACK_LEVEL)) ? 1U : 0U;

        correction = 0;
        if (line_seen != 0U) {
            black_count = 0U;
            weighted_sum = 0;
            for (channel = 0U; channel < 8U; channel++) {
                if (gray[channel] == MED_CAR_GRAY_BLACK_LEVEL) {
                    weighted_sum += line_weights[channel];
                    black_count++;
                }
            }

            if (black_count > 0U) {
                correction = (weighted_sum * MED_CAR_LINE_PWM_ADJUST_1) /
                             (int)black_count;
                if (correction > 0) {
                    last_direction = 1;
                } else if (correction < 0) {
                    last_direction = -1;
                } else {
                    last_direction = 0;
                }
            }
        } else if (last_direction > 0) {
            correction += MED_CAR_LINE_PWM_ADJUST_3;
        } else if (last_direction < 0) {
            correction -= MED_CAR_LINE_PWM_ADJUST_3;
        }

        Load(trim_run_pwm(pwm - correction, MED_CAR_LEFT_PWM_TRIM_NUM),
             trim_run_pwm(pwm + correction, MED_CAR_RIGHT_PWM_TRIM_NUM));
        HAL_Delay(10U);
        Read_Speed();
        guard++;
    }

    stop(1);
    return 0U;
}

void move_forward_timed(uint32_t duration_ms, int pwm)
{
    MedicineCar_ResetEncoders();
    Load(trim_run_pwm(pwm, MED_CAR_LEFT_PWM_TRIM_NUM),
         trim_run_pwm(pwm, MED_CAR_RIGHT_PWM_TRIM_NUM));
    HAL_Delay(duration_ms);
    stop(1);
}

int getnum(void)
{
    return N;
}

void u2_printf(const char *fmt, ...)
{
    va_list args;

    va_start(args, fmt);
    uart_vprintf(&huart1, fmt, args);
    va_end(args);
}

void u3_printf(const char *fmt, ...)
{
    va_list args;

    va_start(args, fmt);
    uart_vprintf(&huart1, fmt, args);
    va_end(args);
}
