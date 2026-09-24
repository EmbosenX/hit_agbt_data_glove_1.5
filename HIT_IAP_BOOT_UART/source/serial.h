#ifndef IAP_BOOT_UART_SERIAL_H
#define IAP_BOOT_UART_SERIAL_H

#include "main.h"

#include <stddef.h>
#include <stdint.h>

#define SERIAL_RX_BUFFER_SIZE          (2304U)
#define SERIAL_TX_BUFFER_SIZE          (20U)

typedef struct {
    uint8_t  channel;
    uint8_t  flags;
    uint16_t length;
    uint8_t  payload[SERIAL_RX_BUFFER_SIZE];
    uint16_t checksum;
    uint8_t  state;
    uint16_t widx;
    uint16_t ridx;
} package_t;

typedef void (SerialOnReadyHandler)(package_t *package);

typedef struct {
    uint16_t txsize;
    uint8_t txbuf[SERIAL_TX_BUFFER_SIZE];
    uint16_t rxsize;
    uint8_t rxbuf[SERIAL_RX_BUFFER_SIZE];
    SerialOnReadyHandler *onready;
    package_t package;
    uint8_t dbg_unpack_stage;
    uint8_t dbg_last_byte;
    uint8_t dbg_frame_ok;
    uint8_t dbg_tail_ok;
    uint16_t dbg_last_length;
    uint16_t dbg_crc_recv;
    uint16_t dbg_crc_calc;
} serial_t;

uint16_t package_remain_size(package_t *package);
void package_read(package_t *package, void *data, size_t size);
uint8_t package_read_u8(package_t *package);
uint16_t package_read_u16(package_t *package);
void package_skip(package_t *package, size_t size);
uint16_t pack(uint8_t *buffer, uint8_t ch, uint8_t flags, const void *payload, uint16_t len);
void unpack(serial_t *serial);
uint16_t crc_16(const uint8_t *data, size_t length);

#endif
