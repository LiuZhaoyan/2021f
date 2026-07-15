#include "medicine_car_vision_cache.h"

#include <stddef.h>

#include "medicine_car_config.h"
#include "usart.h"

#define VISION_FRAME_HEAD_0 0xAAU
#define VISION_FRAME_HEAD_1 0xBBU
#define VISION_FRAME_TAIL   0xCCU

static volatile uint8_t g_window_open;
static volatile uint8_t g_frame_valid;
static volatile uint8_t g_parser_state;
static uint8_t g_packet[5];
static volatile MedicineCarVisionFrame g_cached_frame;

static void cache_clear_locked(void)
{
    g_frame_valid = 0U;
    g_cached_frame.left = 0U;
    g_cached_frame.right = 0U;
    g_cached_frame.timestamp_ms = 0U;
}

static void cache_complete_packet(void)
{
    uint8_t left = g_packet[2];
    uint8_t right = g_packet[3];

    g_parser_state = 0U;

    if ((left > 8U) || (right > 8U)) {
        return;
    }
    if ((left == 0U) && (right == 0U)) {
        return;
    }
    if ((g_window_open == 0U) || (g_frame_valid != 0U)) {
        return;
    }

    g_cached_frame.left = left;
    g_cached_frame.right = right;
    g_cached_frame.timestamp_ms = HAL_GetTick();
    g_frame_valid = 1U;
}

static void parser_feed_byte(uint8_t byte)
{
    switch (g_parser_state) {
    case 0U:
        if (byte == VISION_FRAME_HEAD_0) {
            g_packet[0] = byte;
            g_parser_state = 1U;
        }
        break;

    case 1U:
        if (byte == VISION_FRAME_HEAD_1) {
            g_packet[1] = byte;
            g_parser_state = 2U;
        } else if (byte == VISION_FRAME_HEAD_0) {
            g_packet[0] = byte;
        } else {
            g_parser_state = 0U;
        }
        break;

    case 2U:
        g_packet[2] = byte;
        g_parser_state = 3U;
        break;

    case 3U:
        g_packet[3] = byte;
        g_parser_state = 4U;
        break;

    default:
        if (byte == VISION_FRAME_TAIL) {
            g_packet[4] = byte;
            cache_complete_packet();
        } else if (byte == VISION_FRAME_HEAD_0) {
            g_packet[0] = byte;
            g_parser_state = 1U;
        } else {
            g_parser_state = 0U;
        }
        break;
    }
}

void USART3_IRQHandler(void)
{
    uint32_t isr_flags = READ_REG(huart3.Instance->ISR);

    if ((isr_flags & (USART_ISR_ORE | USART_ISR_FE |
                      USART_ISR_NE | USART_ISR_PE)) != 0U) {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
        __HAL_UART_CLEAR_FEFLAG(&huart3);
        __HAL_UART_CLEAR_NEFLAG(&huart3);
        __HAL_UART_CLEAR_OREFLAG(&huart3);
        (void)READ_REG(huart3.Instance->RDR);
        g_parser_state = 0U;
        return;
    }

    if ((isr_flags & USART_ISR_RXNE_RXFNE) != 0U) {
        uint8_t byte = (uint8_t)(READ_REG(huart3.Instance->RDR) & 0xFFU);
        parser_feed_byte(byte);
    }
}

void VisionCache_Init(void)
{
#if MED_CAR_ENABLE_RECOGNITION_UART
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    g_window_open = 0U;
    g_parser_state = 0U;
    cache_clear_locked();
    __set_PRIMASK(primask);

    __HAL_UART_CLEAR_PEFLAG(&huart3);
    __HAL_UART_CLEAR_FEFLAG(&huart3);
    __HAL_UART_CLEAR_NEFLAG(&huart3);
    __HAL_UART_CLEAR_OREFLAG(&huart3);
    (void)READ_REG(huart3.Instance->RDR);

    __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
    HAL_NVIC_SetPriority(USART3_IRQn, 14U, 0U);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
#endif
}

void VisionCache_BeginWindow(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    g_window_open = 0U;
    g_parser_state = 0U;
    cache_clear_locked();
    g_window_open = 1U;
    __set_PRIMASK(primask);
}

void VisionCache_EndWindow(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    g_window_open = 0U;
    g_parser_state = 0U;
    cache_clear_locked();
    __set_PRIMASK(primask);
}

uint8_t VisionCache_HasFrame(void)
{
    return g_frame_valid;
}

uint8_t VisionCache_Read(MedicineCarVisionFrame *out)
{
    uint32_t primask;
    uint8_t valid;

    if (out == NULL) {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    valid = g_frame_valid;
    if (valid != 0U) {
        out->left = g_cached_frame.left;
        out->right = g_cached_frame.right;
        out->timestamp_ms = g_cached_frame.timestamp_ms;
    }
    __set_PRIMASK(primask);

    return valid;
}

uint8_t VisionCache_Wait(MedicineCarVisionFrame *out, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    while (VisionCache_HasFrame() == 0U) {
        if ((HAL_GetTick() - start) >= timeout_ms) {
            return 0U;
        }
    }

    return VisionCache_Read(out);
}
