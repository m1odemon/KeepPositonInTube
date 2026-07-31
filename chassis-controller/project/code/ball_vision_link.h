#ifndef BALL_VISION_LINK_H
#define BALL_VISION_LINK_H

#include <stdbool.h>
#include <stdint.h>

#include "ball_control.h"

/*
 * Raspberry Pi -> MSPM0 fixed-length visual measurement frame, little-endian:
 *
 *   offset  size  field
 *   0       1     0xA5
 *   1       1     0x5A
 *   2       1     protocol_version (currently 1)
 *   3       4     sequence (uint32)
 *   7       4     capture_timestamp_us (uint32, Raspberry Pi clock domain)
 *   11      4     position_mm (IEEE-754 float32, relative to the tube)
 *   15      4     confidence (IEEE-754 float32, range 0.0 to 1.0)
 *   19      2     CRC-16/CCITT-FALSE over bytes [2, 18], little-endian CRC
 *
 * The protocol is 21 bytes.  The MSPM0 must use receive_timestamp_us only for
 * visual timeout; capture_timestamp_us is used only between Pi frames for dt.
 */
#define BALL_VISION_FRAME_HEADER_0       (0xA5U)
#define BALL_VISION_FRAME_HEADER_1       (0x5AU)
#define BALL_VISION_PROTOCOL_VERSION     (1U)
#define BALL_VISION_FRAME_SIZE            (21U)
#define BALL_VISION_CRC_OFFSET            (19U)
#define BALL_VISION_CRC_LENGTH            (17U)

typedef struct
{
    ball_control_measurement_t measurement;
    uint32_t receive_timestamp_us;
} ball_vision_frame_t;

typedef struct
{
    volatile uint32_t publish_generation;
    uint32_t consumed_generation;
    volatile ball_vision_frame_t latest_frame;
    uint8_t rx_buffer[BALL_VISION_FRAME_SIZE];
    uint8_t rx_index;
    volatile uint32_t valid_frame_count;
    volatile uint32_t crc_error_count;
    volatile uint32_t format_error_count;
} ball_vision_link_t;

void ball_vision_link_init(ball_vision_link_t *link);

/* Call from UART2 RX interrupt for every received byte. */
void ball_vision_link_rx_byte_from_isr(ball_vision_link_t *link,
                                       uint8_t byte,
                                       uint32_t receive_timestamp_us);

/*
 * Copy the newest complete frame once.  This is safe with one UART ISR writer
 * and one control-task reader; no individual position/timestamp reads occur.
 */
bool ball_vision_link_take_latest(ball_vision_link_t *link,
                                  ball_vision_frame_t *frame);

#endif
