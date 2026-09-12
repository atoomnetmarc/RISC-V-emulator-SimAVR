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
#define VPI_EXIT     0x39

#define VPI_EXIT_PASS 0x00
#define VPI_EXIT_FAIL 0x01

typedef struct {
    avr_io_t io;
    uint8_t *memory;
    uint32_t address;
    uint8_t exit_status;
    uint8_t halted;
} vpi_t;

void vpi_init(avr_t *avr, vpi_t *vpi);
void vpi_set_memory(vpi_t *vpi, uint8_t *memory);
uint8_t vpi_get_exit_status(const vpi_t *vpi);

#endif /* VPI_H_ */
