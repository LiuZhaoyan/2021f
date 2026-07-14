#ifndef MEDICINE_CAR_APP_H
#define MEDICINE_CAR_APP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void MedicineCar_Init(void);
void MedicineCar_Step(void);
void MedicineCar_RequestRun(uint8_t target_room);
void MedicineCar_SetRecognizedNumber(uint8_t number);
uint8_t MedicineCar_RunRoute12Test(uint8_t target_room);
uint8_t MedicineCar_RunRoute3To8Test(uint8_t target_room);
uint8_t MedicineCar_RunRp2Test(uint8_t target_room);

#ifdef __cplusplus
}
#endif

#endif
