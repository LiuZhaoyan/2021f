#include "medicine_car_vision_ring.h"

#include <stddef.h>

#include "medicine_car_config.h"
#include "usart.h"

#define VISION_FRAME_HEAD_0  0xAAU
#define VISION_FRAME_HEAD_1  0xBBU
#define VISION_FRAME_TAIL    0xCCU
#define VISION_BUF_MASK      (VISION_RING_SIZE - 1U)

static VisionRingEntry g_ring_buf[VISION_RING_SIZE];
static volatile uint32_t g_ring_write;
static volatile uint32_t g_ring_read;
static volatile uint8_t  g_isr_state;
static uint8_t           g_isr_packet[5];

static void ring_push(const VisionRingEntry *entry)
{
    uint32_t next = (g_ring_write + 1U) & VISION_BUF_MASK;

    if (next == g_ring_read) {
        return;
    }
    g_ring_buf[g_ring_write] = *entry;
    g_ring_write = next;
}

static void isr_process_complete_packet(void)
{
    uint8_t left  = g_isr_packet[2];
    uint8_t right = g_isr_packet[3];
    VisionRingEntry entry;

    g_isr_state = 0U;

    if ((left > 8U) || (right > 8U)) {
        return;
    }
    if ((left == 0U) && (right == 0U)) {
        return;
    }

    entry.left         = left;
    entry.right        = right;
    entry.timestamp_ms = HAL_GetTick();
    ring_push(&entry);
}

static void isr_feed_byte(uint8_t byte)
{
    switch (g_isr_state) {
    case 0U:
        if (byte == VISION_FRAME_HEAD_0) {
            g_isr_packet[0] = byte;
            g_isr_state = 1U;
        }
        break;
    case 1U:
        if (byte == VISION_FRAME_HEAD_1) {
            g_isr_packet[1] = byte;
            g_isr_state = 2U;
        } else if (byte == VISION_FRAME_HEAD_0) {
            g_isr_packet[0] = byte;
        } else {
            g_isr_state = 0U;
        }
        break;
    case 2U:
        g_isr_packet[2] = byte;
        g_isr_state = 3U;
        break;
    case 3U:
        g_isr_packet[3] = byte;
        g_isr_state = 4U;
        break;
    default:
        if (byte == VISION_FRAME_TAIL) {
            g_isr_packet[4] = byte;
            isr_process_complete_packet();
        } else if (byte == VISION_FRAME_HEAD_0) {
            g_isr_packet[0] = byte;
            g_isr_state = 1U;
        } else {
            g_isr_state = 0U;
        }
        break;
    }
}

void USART3_IRQHandler(void)
{
    uint32_t isr_flags = READ_REG(huart3.Instance->ISR);

    if ((isr_flags & (USART_ISR_ORE | USART_ISR_FE |
                      USART_ISR_NE  | USART_ISR_PE)) != 0U) {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
        __HAL_UART_CLEAR_FEFLAG(&huart3);
        __HAL_UART_CLEAR_NEFLAG(&huart3);
        __HAL_UART_CLEAR_OREFLAG(&huart3);
        (void)READ_REG(huart3.Instance->RDR);
        return;
    }

    if ((isr_flags & USART_ISR_RXNE_RXFNE) != 0U) {
        uint8_t byte = (uint8_t)(READ_REG(huart3.Instance->RDR) & 0xFFU);
        isr_feed_byte(byte);
    }
}

void VisionRing_Init(void)
{
#if MED_CAR_ENABLE_RECOGNITION_UART
    g_ring_write = 0U;
    g_ring_read  = 0U;
    g_isr_state  = 0U;

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

uint8_t VisionRing_WaitForNewEntry(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    uint32_t write_snap = g_ring_write;

    while (1) {
        if (g_ring_write != write_snap) {
            return 1U;
        }
        if ((HAL_GetTick() - start) >= timeout_ms) {
            return 0U;
        }
    }
}

uint8_t VisionRing_ReadNext(VisionRingEntry *out)
{
    if (g_ring_read == g_ring_write) {
        return 0U;
    }
    if (out != NULL) {
        *out = g_ring_buf[g_ring_read];
    }
    g_ring_read = (g_ring_read + 1U) & VISION_BUF_MASK;
    return 1U;
}

uint8_t VisionRing_PeekRecent(VisionRingEntry *out_entries,
                              uint32_t max_count,
                              uint32_t *actual_count)
{
    uint32_t total;
    uint32_t count;
    uint32_t i;
    uint32_t idx;

    if ((out_entries == NULL) || (max_count == 0U)) {
        if (actual_count != NULL) {
            *actual_count = 0U;
        }
        return 0U;
    }

    total = VisionRing_GetCount();
    if (total == 0U) {
        if (actual_count != NULL) {
            *actual_count = 0U;
        }
        return 0U;
    }

    count = (total < max_count) ? total : max_count;

    for (i = 0U; i < count; i++) {
        idx = (g_ring_write - count + i) & VISION_BUF_MASK;
        out_entries[i] = g_ring_buf[idx];
    }

    if (actual_count != NULL) {
        *actual_count = count;
    }
    return 1U;
}

uint32_t VisionRing_GetCount(void)
{
    return (g_ring_write - g_ring_read) & VISION_BUF_MASK;
}

void VisionRing_Flush(void)
{
    g_ring_read = g_ring_write;
}
