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
static volatile uint8_t  g_stable_armed;
static volatile uint8_t  g_stable_locked;
static volatile uint8_t  g_stable_candidate_count;
static volatile uint8_t  g_complete_pair_seen;
static volatile VisionRingEntry g_stable_candidate;
static volatile VisionRingEntry g_stable_entry;
static volatile VisionRingEntry g_complete_pair_entry;

static void stable_clear_state(void)
{
    g_stable_armed = 0U;
    g_stable_locked = 0U;
    g_stable_candidate_count = 0U;
    g_complete_pair_seen = 0U;
    g_stable_candidate.left = 0U;
    g_stable_candidate.right = 0U;
    g_stable_candidate.timestamp_ms = 0U;
    g_stable_entry.left = 0U;
    g_stable_entry.right = 0U;
    g_stable_entry.timestamp_ms = 0U;
    g_complete_pair_entry.left = 0U;
    g_complete_pair_entry.right = 0U;
    g_complete_pair_entry.timestamp_ms = 0U;
}

static void stable_feed_entry(const VisionRingEntry *entry)
{
    if ((g_stable_armed == 0U) || (entry == NULL)) {
        return;
    }

    if ((g_complete_pair_seen == 0U) &&
        (entry->left != 0U) && (entry->right != 0U)) {
        g_complete_pair_entry.left = entry->left;
        g_complete_pair_entry.right = entry->right;
        g_complete_pair_entry.timestamp_ms = entry->timestamp_ms;
        g_complete_pair_seen = 1U;
    }
    if (g_stable_locked != 0U) {
        return;
    }
    if ((g_stable_candidate_count != 0U) &&
        (g_stable_candidate.left == entry->left) &&
        (g_stable_candidate.right == entry->right)) {
        if (g_stable_candidate_count < 0xFFU) {
            g_stable_candidate_count++;
        }
    } else {
        g_stable_candidate.left = entry->left;
        g_stable_candidate.right = entry->right;
        g_stable_candidate.timestamp_ms = entry->timestamp_ms;
        g_stable_candidate_count = 1U;
    }

    if (g_stable_candidate_count >= MED_CAR_VISION_STABLE_FRAME_COUNT) {
        g_stable_entry.left = entry->left;
        g_stable_entry.right = entry->right;
        g_stable_entry.timestamp_ms = entry->timestamp_ms;
        g_stable_locked = 1U;
    }
}

static void ring_push(const VisionRingEntry *entry)
{
    uint32_t next = (g_ring_write + 1U) & VISION_BUF_MASK;

    if (next == g_ring_read) {
        g_ring_read = (g_ring_read + 1U) & VISION_BUF_MASK;
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
    stable_feed_entry(&entry);
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
    stable_clear_state();

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

uint8_t VisionRing_ReadLatest(VisionRingEntry *out)
{
    uint32_t write_snap = g_ring_write;
    uint32_t latest;

    if (g_ring_read == write_snap) {
        return 0U;
    }

    latest = (write_snap - 1U) & VISION_BUF_MASK;
    if (out != NULL) {
        *out = g_ring_buf[latest];
    }
    g_ring_read = write_snap;
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

void VisionRing_StableArm(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    stable_clear_state();
    g_ring_read = g_ring_write;
    g_stable_armed = 1U;
    __set_PRIMASK(primask);
}

uint8_t VisionRing_StableRead(VisionRingEntry *out)
{
    uint32_t primask;
    uint8_t locked;

    if (out == NULL) {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    locked = g_stable_locked;
    if (locked != 0U) {
        out->left = g_stable_entry.left;
        out->right = g_stable_entry.right;
        out->timestamp_ms = g_stable_entry.timestamp_ms;
    }
    __set_PRIMASK(primask);

    return locked;
}

uint8_t VisionRing_StableIsLocked(void)
{
    return g_stable_locked;
}

uint8_t VisionRing_CompletePairSeen(void)
{
    return g_complete_pair_seen;
}

uint8_t VisionRing_CompletePairRead(VisionRingEntry *out)
{
    uint32_t primask;
    uint8_t seen;

    if (out == NULL) {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    seen = g_complete_pair_seen;
    if (seen != 0U) {
        out->left = g_complete_pair_entry.left;
        out->right = g_complete_pair_entry.right;
        out->timestamp_ms = g_complete_pair_entry.timestamp_ms;
    }
    __set_PRIMASK(primask);
    return seen;
}

uint8_t VisionRing_StableWait(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    while (VisionRing_StableIsLocked() == 0U) {
        if ((HAL_GetTick() - start) >= timeout_ms) {
            return 0U;
        }
    }
    return 1U;
}

void VisionRing_StableRelease(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    stable_clear_state();
    __set_PRIMASK(primask);
}
