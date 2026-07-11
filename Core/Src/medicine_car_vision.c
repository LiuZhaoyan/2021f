#include "medicine_car_vision.h"

#include <stddef.h>
#include <string.h>

#include "medicine_car_config.h"
#include "usart.h"

#define VISION_REQUEST_TEXT "REQ\r\n"
#define VISION_RX_LINE_MAX  32U

static void vision_clear_result(MedicineCarVisionResult *out)
{
    if (out != NULL) {
        memset(out, 0, sizeof(*out));
    }
}

static void vision_flush_rx(void)
{
#if MED_CAR_ENABLE_RECOGNITION_UART
    uint8_t byte;

    while (HAL_UART_Receive(&huart3, &byte, 1U, 0U) == HAL_OK) {
    }
#endif
}

static uint32_t remaining_timeout(uint32_t start, uint32_t timeout_ms)
{
    uint32_t elapsed = HAL_GetTick() - start;

    if (elapsed >= timeout_ms) {
        return 0U;
    }
    return timeout_ms - elapsed;
}

static uint8_t vision_read_line(char *line, uint32_t line_size, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    uint32_t pos = 0U;

    if ((line == NULL) || (line_size == 0U)) {
        return 0U;
    }

    while (remaining_timeout(start, timeout_ms) > 0U) {
        uint8_t byte;
        uint32_t wait_ms = remaining_timeout(start, timeout_ms);

        if (HAL_UART_Receive(&huart3, &byte, 1U, wait_ms) != HAL_OK) {
            break;
        }

        if (byte == '\r') {
            continue;
        }
        if (byte == '\n') {
            line[pos] = '\0';
            return (pos > 0U) ? 1U : 0U;
        }

        if (pos >= (line_size - 1U)) {
            line[line_size - 1U] = '\0';
            return 0U;
        }
        line[pos++] = (char)byte;
    }

    line[pos] = '\0';
    return 0U;
}

static uint8_t parse_digit(const char **cursor, uint8_t *digit)
{
    const char *p;

    if ((cursor == NULL) || (*cursor == NULL) || (digit == NULL)) {
        return 0U;
    }

    p = *cursor;
    if ((*p < '0') || (*p > '9')) {
        return 0U;
    }

    *digit = (uint8_t)(*p - '0');
    p++;
    *cursor = p;
    return 1U;
}

static uint8_t vision_parse_ok_line(const char *line, MedicineCarVisionResult *out)
{
    const char *p;
    uint8_t count;
    uint8_t index;
    MedicineCarVisionResult parsed;

    if ((line == NULL) || (out == NULL)) {
        return 0U;
    }

    if (strncmp(line, "OK,", 3U) != 0) {
        return 0U;
    }

    p = line + 3U;
    if ((*p < '1') || (*p > '4')) {
        return 0U;
    }
    count = (uint8_t)(*p - '0');
    p++;
    if (*p != ',') {
        return 0U;
    }
    p++;

    memset(&parsed, 0, sizeof(parsed));
    parsed.count = count;
    for (index = 0U; index < count; index++) {
        if (parse_digit(&p, &parsed.digits[index]) == 0U) {
            return 0U;
        }

        if (index == (uint8_t)(count - 1U)) {
            if (*p != '\0') {
                return 0U;
            }
        } else {
            if (*p != ',') {
                return 0U;
            }
            p++;
        }
    }

    *out = parsed;
    return 1U;
}

uint8_t MedicineCarVision_Request(MedicineCarVisionResult *out, uint32_t timeout_ms)
{
#if MED_CAR_ENABLE_RECOGNITION_UART
    static const uint8_t request[] = VISION_REQUEST_TEXT;
    char line[VISION_RX_LINE_MAX];

    vision_clear_result(out);
    if (out == NULL) {
        return 0U;
    }

    vision_flush_rx();
    if (HAL_UART_Transmit(&huart3,
                          (uint8_t *)request,
                          (uint16_t)(sizeof(request) - 1U),
                          50U) != HAL_OK) {
        return 0U;
    }

    if (vision_read_line(line, sizeof(line), timeout_ms) == 0U) {
        return 0U;
    }

    return vision_parse_ok_line(line, out);
#else
    (void)timeout_ms;
    vision_clear_result(out);
    return 0U;
#endif
}
