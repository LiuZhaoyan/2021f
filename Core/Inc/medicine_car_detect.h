#ifndef MEDICINE_CAR_DETECT_H
#define MEDICINE_CAR_DETECT_H

#include <stdint.h>

uint8_t is_at_cross(void);
uint8_t is_wide_black(const uint8_t gray[8], uint8_t threshold);
uint8_t is_line_left(void);
uint8_t is_line_right(void);
uint8_t is_line_center(void);
uint8_t is_fork(void);

#endif
