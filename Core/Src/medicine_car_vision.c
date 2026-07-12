#include "medicine_car_vision.h"

#include <stddef.h>
#include <string.h>

#include "medicine_car_config.h"
#include "medicine_car_vision_ring.h"

#define VISION_RX_LINE_MAX  32U
#define VISION_FRAME_HEAD_0 0xAAU
#define VISION_FRAME_HEAD_1 0xBBU
#define VISION_FRAME_TAIL   0xCCU
#define VISION_PACKET_SIZE  5U

static MedicineCarVisionStatus last_status = MED_CAR_VISION_STATUS_OK;
static char last_line[VISION_RX_LINE_MAX];

static void vision_set_status(MedicineCarVisionStatus status)
{
    last_status = status;
}

static void vision_set_last_line(const char *line)
{
    if (line == NULL) {
        last_line[0] = '\0';
        return;
    }

    (void)strncpy(last_line, line, sizeof(last_line) - 1U);
    last_line[sizeof(last_line) - 1U] = '\0';
}

static void vision_clear_result(MedicineCarVisionResult *out)
{
    if (out != NULL) {
        memset(out, 0, sizeof(*out));
    }
}

static void vision_format_packet(const uint8_t packet[VISION_PACKET_SIZE])
{
    static const char hex[] = "0123456789ABCDEF";
    uint32_t i;
    uint32_t pos = 0U;

    if (packet == NULL) {
        vision_set_last_line(NULL);
        return;
    }

    for (i = 0U; i < VISION_PACKET_SIZE; i++) {
        if ((pos + 3U) >= sizeof(last_line)) {
            break;
        }
        last_line[pos++] = hex[(packet[i] >> 4U) & 0x0FU];
        last_line[pos++] = hex[packet[i] & 0x0FU];
        if (i != (VISION_PACKET_SIZE - 1U)) {
            last_line[pos++] = ' ';
        }
    }
    last_line[pos] = '\0';
}

uint8_t MedicineCarVision_Request(MedicineCarVisionResult *out, uint32_t timeout_ms)
{
#if MED_CAR_ENABLE_RECOGNITION_UART
    VisionRingEntry entry;
    uint8_t packet[VISION_PACKET_SIZE];

    vision_set_last_line(NULL);
    vision_clear_result(out);
    if (out == NULL) {
        vision_set_status(MED_CAR_VISION_STATUS_BAD_ARG);
        return 0U;
    }

    if (VisionRing_WaitForNewEntry(timeout_ms) == 0U) {
        vision_set_status(MED_CAR_VISION_STATUS_TIMEOUT);
        return 0U;
    }

    if (VisionRing_ReadNext(&entry) == 0U) {
        vision_set_status(MED_CAR_VISION_STATUS_TIMEOUT);
        return 0U;
    }

    packet[0] = VISION_FRAME_HEAD_0;
    packet[1] = VISION_FRAME_HEAD_1;
    packet[2] = entry.left;
    packet[3] = entry.right;
    packet[4] = VISION_FRAME_TAIL;
    vision_format_packet(packet);

    memset(out, 0, sizeof(*out));
    if (entry.left != 0U) {
        out->digits[out->count++] = entry.left;
    }
    if (entry.right != 0U) {
        out->digits[out->count++] = entry.right;
    }

    vision_set_status(MED_CAR_VISION_STATUS_OK);
    return (out->count > 0U) ? 1U : 0U;
#else
    (void)timeout_ms;
    vision_set_last_line(NULL);
    vision_set_status(MED_CAR_VISION_STATUS_DISABLED);
    vision_clear_result(out);
    return 0U;
#endif
}

MedicineCarVisionStatus MedicineCarVision_LastStatus(void)
{
    return last_status;
}

const char *MedicineCarVision_LastStatusText(void)
{
    switch (last_status) {
    case MED_CAR_VISION_STATUS_OK:
        return "ok";
    case MED_CAR_VISION_STATUS_BAD_ARG:
        return "bad arg";
    case MED_CAR_VISION_STATUS_DISABLED:
        return "uart disabled";
    case MED_CAR_VISION_STATUS_TX_ERROR:
        return "tx error";
    case MED_CAR_VISION_STATUS_RX_ERROR:
        return "rx error";
    case MED_CAR_VISION_STATUS_TIMEOUT:
        return "timeout";
    case MED_CAR_VISION_STATUS_REMOTE_ERR:
        return "remote err";
    case MED_CAR_VISION_STATUS_INVALID_RESPONSE:
        return "invalid response";
    default:
        return "unknown";
    }
}

const char *MedicineCarVision_LastLine(void)
{
    return last_line;
}
