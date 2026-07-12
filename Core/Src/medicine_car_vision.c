#include "medicine_car_vision.h"

#include <stddef.h>
#include <string.h>

#include "medicine_car_config.h"
#include "usart.h"

#define VISION_RX_LINE_MAX  32U
#define VISION_FRAME_HEAD_0 0xAAU
#define VISION_FRAME_HEAD_1 0xBBU
#define VISION_FRAME_TAIL   0xCCU
#define VISION_PACKET_SIZE  5U
#define VISION_SNIFF_MAX    10U

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

static void vision_prepare_rx(void)
{
    __HAL_UART_CLEAR_PEFLAG(&huart3);
    __HAL_UART_CLEAR_FEFLAG(&huart3);
    __HAL_UART_CLEAR_NEFLAG(&huart3);
    __HAL_UART_CLEAR_OREFLAG(&huart3);
    __HAL_UART_FLUSH_DRREGISTER(&huart3);
    huart3.ErrorCode = HAL_UART_ERROR_NONE;
}

static uint32_t remaining_timeout(uint32_t start, uint32_t timeout_ms)
{
    uint32_t elapsed = HAL_GetTick() - start;

    if (elapsed >= timeout_ms) {
        return 0U;
    }
    return timeout_ms - elapsed;
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

static void vision_format_bytes(const uint8_t *bytes, uint32_t size)
{
    static const char hex[] = "0123456789ABCDEF";
    uint32_t i;
    uint32_t pos = 0U;

    if ((bytes == NULL) || (size == 0U)) {
        vision_set_last_line(NULL);
        return;
    }

    for (i = 0U; i < size; i++) {
        if ((pos + 3U) >= sizeof(last_line)) {
            break;
        }
        last_line[pos++] = hex[(bytes[i] >> 4U) & 0x0FU];
        last_line[pos++] = hex[bytes[i] & 0x0FU];
        if (i != (size - 1U)) {
            last_line[pos++] = ' ';
        }
    }
    last_line[pos] = '\0';
}

static uint8_t vision_read_packet(uint8_t packet[VISION_PACKET_SIZE],
                                  uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    uint8_t sniff[VISION_SNIFF_MAX];
    uint32_t sniff_count = 0U;
    uint8_t state = 0U;
    uint8_t byte;

    if (packet == NULL) {
        return 0U;
    }

    while (remaining_timeout(start, timeout_ms) > 0U) {
        uint32_t wait_ms = remaining_timeout(start, timeout_ms);

        if (HAL_UART_Receive(&huart3, &byte, 1U, wait_ms) != HAL_OK) {
            if (sniff_count > 0U) {
                vision_format_bytes(sniff, sniff_count);
            }
            break;
        }

        if (sniff_count < VISION_SNIFF_MAX) {
            sniff[sniff_count++] = byte;
        }

        switch (state) {
        case 0U:
            if (byte == VISION_FRAME_HEAD_0) {
                packet[0] = byte;
                state = 1U;
            }
            break;
        case 1U:
            if (byte == VISION_FRAME_HEAD_1) {
                packet[1] = byte;
                state = 2U;
            } else {
                if (byte == VISION_FRAME_HEAD_0) {
                    packet[0] = byte;
                    state = 1U;
                } else {
                    state = 0U;
                }
            }
            break;
        case 2U:
            packet[2] = byte;
            state = 3U;
            break;
        case 3U:
            packet[3] = byte;
            state = 4U;
            break;
        default:
            if (byte == VISION_FRAME_TAIL) {
                packet[4] = byte;
                vision_format_packet(packet);
                return 1U;
            }
            if (byte == VISION_FRAME_HEAD_0) {
                packet[0] = byte;
                state = 1U;
            } else {
                state = 0U;
            }
            break;
        }
    }

    if (sniff_count > 0U) {
        vision_format_bytes(sniff, sniff_count);
    }
    return 0U;
}

static uint8_t vision_parse_packet(const uint8_t packet[VISION_PACKET_SIZE],
                                   MedicineCarVisionResult *out)
{
    MedicineCarVisionResult parsed;
    uint8_t left;
    uint8_t right;

    if ((packet == NULL) || (out == NULL)) {
        return 0U;
    }

    if ((packet[0] != VISION_FRAME_HEAD_0) ||
        (packet[1] != VISION_FRAME_HEAD_1) ||
        (packet[4] != VISION_FRAME_TAIL)) {
        return 0U;
    }

    left = packet[2];
    right = packet[3];
    if ((left > 8U) || (right > 8U)) {
        return 0U;
    }

    memset(&parsed, 0, sizeof(parsed));
    if (left != 0U) {
        parsed.digits[parsed.count++] = left;
    }
    if (right != 0U) {
        parsed.digits[parsed.count++] = right;
    }

    *out = parsed;
    return 1U;
}

uint8_t MedicineCarVision_Request(MedicineCarVisionResult *out, uint32_t timeout_ms)
{
#if MED_CAR_ENABLE_RECOGNITION_UART
    uint8_t packet[VISION_PACKET_SIZE];

    vision_set_last_line(NULL);
    vision_clear_result(out);
    if (out == NULL) {
        vision_set_status(MED_CAR_VISION_STATUS_BAD_ARG);
        return 0U;
    }

    vision_prepare_rx();
    if (vision_read_packet(packet, timeout_ms) == 0U) {
        if (huart3.ErrorCode != HAL_UART_ERROR_NONE) {
            vision_set_status(MED_CAR_VISION_STATUS_RX_ERROR);
        } else {
            vision_set_status(MED_CAR_VISION_STATUS_TIMEOUT);
        }
        return 0U;
    }

    if (vision_parse_packet(packet, out) != 0U) {
        vision_set_status(MED_CAR_VISION_STATUS_OK);
        return 1U;
    }

    vision_set_status(MED_CAR_VISION_STATUS_INVALID_RESPONSE);
    return 0U;
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
