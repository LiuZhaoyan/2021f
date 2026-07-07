#ifndef MEDICINE_CAR_CONFIG_H
#define MEDICINE_CAR_CONFIG_H

#include <stdint.h>

/*
 * Hardware that has not been built yet stays disabled by default. Enable one
 * flag at a time after the matching pins/peripheral are configured in CubeMX.
 */
#define MED_CAR_ENABLE_ENCODER          0U
#define MED_CAR_ENABLE_BLUETOOTH        0U
#define MED_CAR_ENABLE_RECOGNITION_UART 0U
#define MED_CAR_ENABLE_TFT              0U
#define MED_CAR_ENABLE_KEYS             0U
#define MED_CAR_ENABLE_DRUG_SENSOR      0U

#define MED_CAR_DEFAULT_MISSING_SENSOR  0U
#define MED_CAR_DEFAULT_DRUG_PRESENT    1U
#define MED_CAR_GRAY_BLACK_LEVEL        1U
#define MED_CAR_GRAY_SETTLE_CYCLES      160U

#define MED_CAR_CODE_PWM_MAX            8000
#define MED_CAR_H7_PWM_PERIOD           1999U

#define MED_CAR_STOP_TICK_MS            20U
#define MED_CAR_STEP_IDLE_DELAY_MS      10U
#define MED_CAR_DELIVERY_WAIT_TIMEOUT_MS 30000U

#define MED_CAR_TURN_LEFT_MS            380U
#define MED_CAR_TURN_RIGHT_MS           380U
#define MED_CAR_TURN_AROUND_MS          760U

#define MED_CAR_LINE_PWM_ADJUST_1       1000
#define MED_CAR_LINE_PWM_ADJUST_2       2000
#define MED_CAR_LINE_PWM_ADJUST_3       3000

#define MED_CAR_DISTANCE_MID            6000U
#define MED_CAR_DISTANCE_FIRST_CHECK    11800U
#define MED_CAR_DISTANCE_SECOND_CHECK   19500U
#define MED_CAR_DISTANCE_THIRD_CHECK    33000U
#define MED_CAR_DISTANCE_RETURN_MID     13000U
#define MED_CAR_DISTANCE_RETURN_CROSS4  5900U
#define MED_CAR_DISTANCE_RETURN_CROSS3  8000U
#define MED_CAR_DISTANCE_HOME_WHITE     9000U

typedef enum {
    MED_CAR_SENSOR_IO1 = 1,
    MED_CAR_SENSOR_IO2 = 2,
    MED_CAR_SENSOR_IO3 = 3,
    MED_CAR_SENSOR_IO4 = 4,
    MED_CAR_SENSOR_IO5 = 5,
    MED_CAR_SENSOR_IO6 = 6,
    MED_CAR_SENSOR_IO7 = 7,
    MED_CAR_SENSOR_IO8 = 8
} MedicineCarSensorId;

#endif
