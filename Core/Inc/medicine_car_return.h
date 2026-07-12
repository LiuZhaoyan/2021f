#ifndef MEDICINE_CAR_RETURN_H
#define MEDICINE_CAR_RETURN_H

#include <stdint.h>

#define RETURN_DIR_LEFT   0U
#define RETURN_DIR_RIGHT  1U

void Return_Init(void);
void Return_Push(uint8_t direction);
uint8_t Return_IsEmpty(void);
void Return_Execute(uint16_t home_distance, int pwm);

#endif
