#include "medicine_car_detect.h"

#include "medicine_car_config.h"
#include "medicine_car_platform.h"

uint8_t is_at_cross(void)
{
    uint8_t gray[8];

    MedicineCar_ReadLineSensors(gray);
    if ((gray[1] == MED_CAR_GRAY_BLACK_LEVEL) &&
        (gray[2] == MED_CAR_GRAY_BLACK_LEVEL) &&
        (gray[4] == MED_CAR_GRAY_BLACK_LEVEL) &&
        (gray[5] == MED_CAR_GRAY_BLACK_LEVEL)) {
        return 1U;
    }
    return 0U;
}

uint8_t is_wide_black(uint8_t threshold)
{
    uint8_t gray[8];
    uint8_t channel;
    uint8_t count = 0U;

    MedicineCar_ReadLineSensors(gray);
    for (channel = 0U; channel < 8U; channel++) {
        if (gray[channel] == MED_CAR_GRAY_BLACK_LEVEL) {
            count++;
        }
    }
    return (count >= threshold) ? 1U : 0U;
}

uint8_t is_line_left(void)
{
    uint8_t gray[8];

    MedicineCar_ReadLineSensors(gray);
    if ((gray[2] == MED_CAR_GRAY_BLACK_LEVEL) &&
        (gray[1] == MED_CAR_GRAY_BLACK_LEVEL)) {
        return 1U;
    }
    return 0U;
}

uint8_t is_line_right(void)
{
    uint8_t gray[8];

    MedicineCar_ReadLineSensors(gray);
    if ((gray[4] == MED_CAR_GRAY_BLACK_LEVEL) &&
        (gray[5] == MED_CAR_GRAY_BLACK_LEVEL)) {
        return 1U;
    }
    return 0U;
}

uint8_t is_line_center(void)
{
    uint8_t gray[8];

    MedicineCar_ReadLineSensors(gray);
    if ((gray[2] == MED_CAR_GRAY_BLACK_LEVEL) ||
        (gray[3] == MED_CAR_GRAY_BLACK_LEVEL) ||
        (gray[4] == MED_CAR_GRAY_BLACK_LEVEL) ||
        (gray[5] == MED_CAR_GRAY_BLACK_LEVEL)) {
        return 1U;
    }
    return 0U;
}

uint8_t is_fork(void)
{
    uint8_t gray[8];

    MedicineCar_ReadLineSensors(gray);

    if ((gray[1] == MED_CAR_GRAY_BLACK_LEVEL) &&
        (gray[2] == MED_CAR_GRAY_BLACK_LEVEL) &&
        (gray[4] == MED_CAR_GRAY_BLACK_LEVEL) &&
        (gray[5] == MED_CAR_GRAY_BLACK_LEVEL)) {
        return 0U;
    }

    {
        uint8_t left_black = (gray[0] == MED_CAR_GRAY_BLACK_LEVEL ||
                              gray[1] == MED_CAR_GRAY_BLACK_LEVEL ||
                              gray[2] == MED_CAR_GRAY_BLACK_LEVEL);
        uint8_t right_black = (gray[5] == MED_CAR_GRAY_BLACK_LEVEL ||
                               gray[6] == MED_CAR_GRAY_BLACK_LEVEL ||
                               gray[7] == MED_CAR_GRAY_BLACK_LEVEL);
        return (left_black && right_black) ? 1U : 0U;
    }
}
