#ifndef MEDICINE_CAR_VISION_RING_H
#define MEDICINE_CAR_VISION_RING_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define VISION_RING_SIZE 16U

typedef struct {
    uint8_t  left;
    uint8_t  right;
    uint32_t timestamp_ms;
} VisionRingEntry;

void     VisionRing_Init(void);
uint8_t  VisionRing_WaitForNewEntry(uint32_t timeout_ms);
uint8_t  VisionRing_ReadNext(VisionRingEntry *out);
uint8_t  VisionRing_ReadLatest(VisionRingEntry *out);
uint8_t  VisionRing_PeekRecent(VisionRingEntry *out_entries,
                               uint32_t max_count,
                               uint32_t *actual_count);
uint32_t VisionRing_GetCount(void);
void     VisionRing_Flush(void);
void     VisionRing_StableArm(void);
uint8_t  VisionRing_StableRead(VisionRingEntry *out);
uint8_t  VisionRing_StableIsLocked(void);
uint8_t  VisionRing_StableWait(uint32_t timeout_ms);
void     VisionRing_StableRelease(void);

#ifdef __cplusplus
}
#endif

#endif /* MEDICINE_CAR_VISION_RING_H */
