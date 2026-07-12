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

typedef enum {
    MED_CAR_VISION_STATUS_OK = 0,
    MED_CAR_VISION_STATUS_BAD_ARG,
    MED_CAR_VISION_STATUS_DISABLED,
    MED_CAR_VISION_STATUS_TX_ERROR,
    MED_CAR_VISION_STATUS_RX_ERROR,
    MED_CAR_VISION_STATUS_TIMEOUT,
    MED_CAR_VISION_STATUS_REMOTE_ERR,
    MED_CAR_VISION_STATUS_INVALID_RESPONSE
} MedicineCarVisionStatus;

uint8_t MedicineCarVision_Request(MedicineCarVisionResult *out, uint32_t timeout_ms);
MedicineCarVisionStatus MedicineCarVision_LastStatus(void);
const char *MedicineCarVision_LastStatusText(void);
const char *MedicineCarVision_LastLine(void);

#ifdef __cplusplus
}
#endif

#endif
