#include "serial.h"

#include <string.h>

enum {
    S_HEAD_L = 0U,
    S_HEAD_H,
    S_CHANNEL,
    S_FLAGS,
    S_LENGTH_L,
    S_LENGTH_H,
    S_PAYLOAD,
    S_CHECKSUM_L,
    S_CHECKSUM_H,
    S_TAIL_L,
    S_TAIL_H
};

uint16_t package_remain_size(package_t *package)
{
    return (NULL != package) ? (uint16_t)(package->length - package->ridx) : 0U;
}

void package_read(package_t *package, void *data, size_t size)
{
    uint16_t remain_size;
    size_t bytes;

    if ((NULL == package) || (NULL == data)) {
        return;
    }

    remain_size = package_remain_size(package);
    bytes = (size > remain_size) ? remain_size : size;
    (void)memcpy(data, package->payload + package->ridx, bytes);
    package_skip(package, bytes);
}

uint8_t package_read_u8(package_t *package)
{
    uint8_t value = 0U;

    package_read(package, &value, 1U);
    return value;
}

uint16_t package_read_u16(package_t *package)
{
    uint16_t value = 0U;

    package_read(package, &value, 2U);
    return value;
}

void package_skip(package_t *package, size_t size)
{
    if (NULL == package) {
        return;
    }

    package->ridx = (uint16_t)(package->ridx + size);
    if (package->ridx > package->length) {
        package->ridx = package->length;
    }
}

uint16_t pack(uint8_t *buffer, uint8_t ch, uint8_t flags, const void *payload, uint16_t len)
{
    uint16_t checksum = 0U;

    if ((NULL == buffer) || ((NULL == payload) && (0U != len))) {
        return 0U;
    }

    buffer[0] = '<';
    buffer[1] = '<';
    buffer[2] = ch;
    buffer[3] = flags;
    buffer[4] = (uint8_t)(len & 0xFFU);
    buffer[5] = (uint8_t)(len >> 8U);
    if (0U != len) {
        (void)memcpy(buffer + 6, payload, len);
        checksum = crc_16((const uint8_t *)payload, len);
    }
    buffer[6 + len] = (uint8_t)(checksum & 0xFFU);
    buffer[7 + len] = (uint8_t)(checksum >> 8U);
    buffer[8 + len] = '>';
    buffer[9 + len] = '>';

    return (uint16_t)(len + 10U);
}

void unpack(serial_t *serial)
{
    uint16_t i;

    if (NULL == serial) {
        return;
    }

    for (i = 0U; i < serial->rxsize; ++i) {
        uint8_t ch = serial->rxbuf[i];
        serial->dbg_last_byte = ch;

        switch (serial->package.state) {
        case S_HEAD_L:
            serial->dbg_unpack_stage = S_HEAD_L;
            serial->package.state = (ch == '<') ? S_HEAD_H : S_HEAD_L;
            break;
        case S_HEAD_H:
            serial->dbg_unpack_stage = S_HEAD_H;
            serial->package.state = (ch == '<') ? S_CHANNEL : S_HEAD_L;
            break;
        case S_CHANNEL:
            serial->dbg_unpack_stage = S_CHANNEL;
            serial->package.channel = ch;
            serial->package.state = S_FLAGS;
            break;
        case S_FLAGS:
            serial->dbg_unpack_stage = S_FLAGS;
            serial->package.flags = ch;
            serial->package.state = S_LENGTH_L;
            break;
        case S_LENGTH_L:
            serial->dbg_unpack_stage = S_LENGTH_L;
            serial->package.length = ch;
            serial->package.state = S_LENGTH_H;
            break;
        case S_LENGTH_H:
            serial->dbg_unpack_stage = S_LENGTH_H;
            serial->package.length = (uint16_t)((ch << 8U) | serial->package.length);
            serial->dbg_last_length = serial->package.length;
            serial->package.state = S_PAYLOAD;
            serial->package.widx = 0U;
            serial->package.ridx = 0U;
            break;
        case S_PAYLOAD:
            serial->dbg_unpack_stage = S_PAYLOAD;
            if (serial->package.widx < serial->package.length) {
                serial->package.payload[serial->package.widx++] = ch;
            }
            if (serial->package.widx == serial->package.length) {
                serial->package.state = S_CHECKSUM_L;
            }
            break;
        case S_CHECKSUM_L:
            serial->dbg_unpack_stage = S_CHECKSUM_L;
            serial->package.checksum = ch;
            serial->package.state = S_CHECKSUM_H;
            break;
        case S_CHECKSUM_H:
            serial->dbg_unpack_stage = S_CHECKSUM_H;
            serial->package.checksum = (uint16_t)((ch << 8U) | serial->package.checksum);
            serial->dbg_crc_recv = serial->package.checksum;
            serial->package.state = S_TAIL_L;
            break;
        case S_TAIL_L:
            serial->dbg_unpack_stage = S_TAIL_L;
            serial->package.state = (ch == '>') ? S_TAIL_H : S_HEAD_L;
            break;
        case S_TAIL_H:
            serial->dbg_unpack_stage = S_TAIL_H;
            serial->package.state = S_HEAD_L;
            serial->dbg_tail_ok = (uint8_t)(ch == '>');
            serial->dbg_crc_calc = crc_16(serial->package.payload, serial->package.length);
            if ((ch == '>') && (serial->package.checksum == serial->dbg_crc_calc)) {
                serial->dbg_frame_ok = 1U;
                if (NULL != serial->onready) {
                    serial->onready(&serial->package);
                }
            } else {
                serial->dbg_frame_ok = 0U;
            }
            break;
        default:
            serial->dbg_unpack_stage = 0xFFU;
            serial->package.state = S_HEAD_L;
            break;
        }
    }

    serial->rxsize = 0U;
    serial->package.state = S_HEAD_L;
}

uint16_t crc_16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0U;
    size_t i;
    uint8_t bit;

    while (length-- > 0U) {
        crc ^= *data++;
        for (i = 0U; i < 8U; ++i) {
            bit = (uint8_t)(crc & 0x0001U);
            crc >>= 1U;
            if (0U != bit) {
                crc ^= 0xA001U;
            }
        }
    }

    return crc;
}
