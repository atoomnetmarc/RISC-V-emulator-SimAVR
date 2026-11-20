/*
Copyright Marc Ketel
SPDX-License-Identifier: Apache-2.0
*/

#ifndef VPI_H_
#define VPI_H_

#include <stdint.h>

#include <simavr/sim_avr.h>

#define VPI_ADDR_LO  0x31
#define VPI_ADDR_MID 0x32
#define VPI_ADDR_HI  0x33
#define VPI_DATA     0x34
#define VPI_ADDR_X   0x36
#define VPI_CALL     0x38
#define VPI_ARGRES   0x3A
#define VPI_STATUS   0x3C
#define VPI_SPACE    0x3D
#define VPI_RX_LEN   0x3E
#define VPI_RX_DATA  0x3F
#define VPI_EXIT     0x39

#define VPI_EXIT_PASS   0x00
#define VPI_EXIT_FAIL   0x01
#define VPI_EXIT_REBOOT 0x02

#define VPI_STATUS_IDLE  0
#define VPI_STATUS_DONE  1
#define VPI_STATUS_ERROR 2

#define VPI_SPACE_RAM     0
#define VPI_SPACE_SCRATCH 1

#define VPI_CALL_UART_TX       1
#define VPI_CALL_CONSOLE_TX    2
#define VPI_CALL_DISK_READ     3
#define VPI_CALL_DISK_WRITE    4
#define VPI_CALL_DISK_FLUSH    5
#define VPI_CALL_NET_TX        6
#define VPI_CALL_RNG_FILL      7
#define VPI_CALL_RTC_REALTIME  8
#define VPI_CALL_RTC_MONOTONIC 9
#define VPI_CALL_TIME_SOURCE   10
#define VPI_CALL_DISK_CAPACITY 11

#define VPI_SCRATCH_SIZE 4096
#define VPI_RX_SIZE      1024
#define VPI_ARGS_SIZE    16
#define VPI_RESULT_SIZE  16

typedef struct {
    avr_io_t io;
    uint8_t *memory;
    uint32_t memory_size;
    uint32_t address;
    uint8_t space;
    uint8_t scratch[VPI_SCRATCH_SIZE];
    uint32_t scratch_address;
    uint8_t args[VPI_ARGS_SIZE];
    uint8_t arg_index;
    uint8_t result[VPI_RESULT_SIZE];
    uint8_t result_index;
    uint8_t status;
    uint8_t exit_status;
    uint8_t halted;
    uint8_t rx[VPI_RX_SIZE];
    uint16_t rx_head;
    uint16_t rx_tail;
    int disk_fd;
    uint64_t disk_capacity;
    int tap_fd;
} vpi_t;

void vpi_init(avr_t *avr, vpi_t *vpi);
void vpi_set_memory(vpi_t *vpi, uint8_t *memory, uint32_t size);
void vpi_set_disk(vpi_t *vpi, int fd, uint64_t capacity);
void vpi_set_tap(vpi_t *vpi, int fd);
void vpi_rx_push(vpi_t *vpi, uint8_t byte);
uint8_t vpi_get_exit_status(const vpi_t *vpi);

#endif /* VPI_H_ */
