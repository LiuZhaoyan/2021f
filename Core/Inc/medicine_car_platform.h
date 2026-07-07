#ifndef MEDICINE_CAR_PLATFORM_H
#define MEDICINE_CAR_PLATFORM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

extern volatile int N;
extern volatile int Speed_L;
extern volatile int Speed_R;
extern volatile int Pwm_L;
extern volatile int Pwm_R;
extern volatile int Time_s;
extern volatile int Run_Flag;
extern volatile int Number;
extern volatile int aim;
extern volatile int BluetoohAim;
extern volatile int Encoder_Left;
extern volatile int Encoder_Right;
extern int NumBuff[2];
extern int XBuff[2];
extern char num[50];

void MedicineCarPlatform_Init(void);
void MedicineCarPlatform_Service(void);

uint8_t MedicineCar_ReadLineSensor(uint8_t index);
uint8_t MedicineCar_ReadDrugPresent(void);
void MedicineCar_SetRedLed(uint8_t on);
void MedicineCar_SetYellowLed(uint8_t on);
void MedicineCar_SetGreenLed(uint8_t on);

void delay_ms(uint32_t ms);
void Read_Speed(void);
int Read_Encoder(uint8_t timx);
void Load(int moto1, int moto2);
void Limit(int *motoA, int *motoB);
void stop(int stoptime);
void turn_left(void);
void turn_right(void);
void diaotou(void);
void xunxian(uint16_t roadsum, int pwm);
void zhao_bai(uint16_t roadsum, int pwm);
int getnum(void);

void u2_printf(const char *fmt, ...);
void u3_printf(const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif
