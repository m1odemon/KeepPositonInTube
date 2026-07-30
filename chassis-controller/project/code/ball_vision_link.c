#include "ball_vision_link.h"

#include <math.h>
#include <string.h>

typedef struct __attribute__((packed))
{
    uint8_t header_0;
    uint8_t header_1;
    uint8_t version;
    uint32_t sequence;
    uint32_t capture_timestamp_us;
    float position_mm;
    float confidence;
    uint16_t crc16;
} ball_vision_wire_frame_t;

typedef char ball_vision_wire_frame_size_must_be_21_bytes[
    (sizeof(ball_vision_wire_frame_t) == BALL_VISION_FRAME_SIZE) ? 1 : -1];

static uint16_t ball_vision_crc16_ccitt_false(const uint8_t *data, uint8_t length)
{
    uint16_t crc = 0xFFFFU;

    while (length-- > 0U)
    {
        crc ^= (uint16_t)(*data++) << 8;
        for (uint8_t bit = 0U; bit < 8U; bit++)
        {
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U)
                                  : (uint16_t)(crc << 1);
        }
    }

    return crc;
}

static void ball_vision_link_publish_from_isr(ball_vision_link_t *link,
                                               const ball_vision_wire_frame_t *wire_frame,
                                               uint32_t receive_timestamp_us)
{
    /* A sequence lock makes the multi-field frame atomic to the reader. */
    link->publish_generation++;
    link->latest_frame.measurement.sequence = wire_frame->sequence;
    link->latest_frame.measurement.capture_timestamp_us = wire_frame->capture_timestamp_us;
    link->latest_frame.measurement.position_mm = wire_frame->position_mm;
    link->latest_frame.measurement.confidence = wire_frame->confidence;
    link->latest_frame.receive_timestamp_us = receive_timestamp_us;
    link->publish_generation++;
    link->valid_frame_count++;
}

void ball_vision_link_init(ball_vision_link_t *link)
{
    if (link != NULL)
    {
        memset(link, 0, sizeof(*link));
    }
}

void ball_vision_link_rx_byte_from_isr(ball_vision_link_t *link,
                                       uint8_t byte,
                                       uint32_t receive_timestamp_us)
{
    ball_vision_wire_frame_t wire_frame;

    if (link == NULL)
    {
        return;
    }

    if (link->rx_index == 0U)
    {
        if (byte == BALL_VISION_FRAME_HEADER_0)
        {
            link->rx_buffer[0] = byte;
            link->rx_index = 1U;
        }
        return;
    }

    if (link->rx_index == 1U)
    {
        if (byte == BALL_VISION_FRAME_HEADER_1)
        {
            link->rx_buffer[1] = byte;
            link->rx_index = 2U;
        }
        else if (byte == BALL_VISION_FRAME_HEADER_0)
        {
            link->rx_buffer[0] = byte;
            link->rx_index = 1U;
        }
        else
        {
            link->rx_index = 0U;
        }
        return;
    }

    link->rx_buffer[link->rx_index++] = byte;
    if (link->rx_index < BALL_VISION_FRAME_SIZE)
    {
        return;
    }

    link->rx_index = 0U;
    memcpy(&wire_frame, link->rx_buffer, sizeof(wire_frame));

    uint16_t expected_crc = ball_vision_crc16_ccitt_false(&link->rx_buffer[2],
                                                            BALL_VISION_CRC_LENGTH);
    if (wire_frame.version != BALL_VISION_PROTOCOL_VERSION ||
        !isfinite(wire_frame.position_mm) ||
        !isfinite(wire_frame.confidence) ||
        wire_frame.confidence < 0.0f || wire_frame.confidence > 1.0f)
    {
        link->format_error_count++;
        return;
    }

    if (wire_frame.crc16 != expected_crc)
    {
        link->crc_error_count++;
        return;
    }

    ball_vision_link_publish_from_isr(link, &wire_frame, receive_timestamp_us);
}

bool ball_vision_link_take_latest(ball_vision_link_t *link,
                                  ball_vision_frame_t *frame)
{
    if (link == NULL || frame == NULL)
    {
        return false;
    }

    for (uint8_t attempt = 0U; attempt < 3U; attempt++)
    {
        uint32_t before_generation = link->publish_generation;
        if ((before_generation & 1U) != 0U ||
            before_generation == link->consumed_generation)
        {
            return false;
        }

        ball_vision_frame_t local_frame = link->latest_frame;
        uint32_t after_generation = link->publish_generation;
        if (before_generation == after_generation && (after_generation & 1U) == 0U)
        {
            *frame = local_frame;
            link->consumed_generation = after_generation;
            return true;
        }
    }

    return false;
}
