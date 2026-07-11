#ifndef MEDICINE_CAR_VISION_H
#define MEDICINE_CAR_VISION_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MED_CAR_VISION_MAX_DIGITS 4U

typedef struct {
    uint8_t count;
    uint8_t digits[MED_CAR_VISION_MAX_DIGITS];
} MedicineCarVisionResult;

uint8_t MedicineCarVision_Request(MedicineCarVisionResult *out, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif
