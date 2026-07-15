#ifndef MEDICINE_CAR_VISION_CACHE_H
#define MEDICINE_CAR_VISION_CACHE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t left;
    uint8_t right;
    uint32_t timestamp_ms;
} MedicineCarVisionFrame;

void VisionCache_Init(void);
void VisionCache_BeginWindow(void);
void VisionCache_EndWindow(void);
uint8_t VisionCache_HasFrame(void);
uint8_t VisionCache_Read(MedicineCarVisionFrame *out);
uint8_t VisionCache_Wait(MedicineCarVisionFrame *out, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* MEDICINE_CAR_VISION_CACHE_H */
