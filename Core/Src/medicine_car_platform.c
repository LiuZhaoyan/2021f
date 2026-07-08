#include "medicine_car_platform.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "gpio.h"
#include "medicine_car_config.h"
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
static uint8_t gray_sensor_values[8];
static uint16_t last_left_encoder_count;
static uint16_t last_right_encoder_count;

static uint8_t read_pin(GPIO_TypeDef *port, uint16_t pin)
{
    return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET) ? 1U : 0U;
}

static void write_pin(GPIO_TypeDef *port, uint16_t pin, uint8_t on)
{
    HAL_GPIO_WritePin(port, pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void gray_select_channel(uint8_t channel)
{
    write_pin(GRAY_AD0_GPIO_Port, GRAY_AD0_Pin, (channel & 0x01U) ? 1U : 0U);
    write_pin(GRAY_AD1_GPIO_Port, GRAY_AD1_Pin, (channel & 0x02U) ? 1U : 0U);
    write_pin(GRAY_AD2_GPIO_Port, GRAY_AD2_Pin, (channel & 0x04U) ? 1U : 0U);
}

static void gray_settle_delay(void)
{
    volatile uint32_t cycles = MED_CAR_GRAY_SETTLE_CYCLES;

    while (cycles-- > 0U) {
        __NOP();
    }
}

static void gray_update_all(void)
{
    uint8_t channel;

    for (channel = 0U; channel < 8U; channel++) {
        gray_select_channel(channel);
        gray_settle_delay();
        gray_sensor_values[channel] =
            (read_pin(GRAY_OUT_GPIO_Port, GRAY_OUT_Pin) == MED_CAR_GRAY_BLACK_LEVEL)
                ? 1U
                : 0U;
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

uint8_t MedicineCar_ReadDrugPresent(void)
{
#if MED_CAR_ENABLE_DRUG_SENSOR
    return 0U;
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

void xunxian(uint16_t roadsum, int pwm)
{
    uint32_t guard = 0U;
    int correction;

    MedicineCar_ResetEncoders();

    while (((Encoder_Left + Encoder_Right) < (int)roadsum) && (guard < 800U)) {
        uint8_t io2 = MedicineCar_ReadLineSensor(MED_CAR_SENSOR_IO2);
        uint8_t io3 = MedicineCar_ReadLineSensor(MED_CAR_SENSOR_IO3);
        uint8_t io5 = MedicineCar_ReadLineSensor(MED_CAR_SENSOR_IO5);
        uint8_t io6 = MedicineCar_ReadLineSensor(MED_CAR_SENSOR_IO6);

        correction = 0;
        if (io3 != 0U) {
            correction -= MED_CAR_LINE_PWM_ADJUST_1;
        }
        if (io2 != 0U) {
            correction -= MED_CAR_LINE_PWM_ADJUST_2;
        }
        if (io5 != 0U) {
            correction += MED_CAR_LINE_PWM_ADJUST_1;
        }
        if (io6 != 0U) {
            correction += MED_CAR_LINE_PWM_ADJUST_2;
        }

        Load(pwm - correction, pwm + correction);
        HAL_Delay(10U);
        Read_Speed();
        guard++;
    }

    stop(1);
}

void zhao_bai(uint16_t roadsum, int pwm)
{
    uint32_t guard = 0U;

    MedicineCar_ResetEncoders();
    while (((Encoder_Left + Encoder_Right) < (int)roadsum) && (guard < 800U)) {
        xunxian(300U, pwm);
        if ((MedicineCar_ReadLineSensor(MED_CAR_SENSOR_IO2) == 0U) &&
            (MedicineCar_ReadLineSensor(MED_CAR_SENSOR_IO3) == 0U) &&
            (MedicineCar_ReadLineSensor(MED_CAR_SENSOR_IO5) == 0U) &&
            (MedicineCar_ReadLineSensor(MED_CAR_SENSOR_IO6) == 0U)) {
            break;
        }
        guard++;
    }
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
