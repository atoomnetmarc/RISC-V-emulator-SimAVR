/*
Copyright Marc Ketel
SPDX-License-Identifier: Apache-2.0
*/

#include "vpi.h"

#include <stdarg.h>

#include <stdio.h>

extern int verbose;

static void msgf(const char *fmt, ...) {
    fputc('\n', stderr);
    fputs("[simavr-host] ", stderr);
    va_list ap;
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
}

#include <simavr/sim_io.h>

static avr_io_addr_t vpi_addr_lo = VPI_ADDR_LO;
static avr_io_addr_t vpi_addr_mid = VPI_ADDR_MID;
static avr_io_addr_t vpi_addr_hi = VPI_ADDR_HI;
static avr_io_addr_t vpi_addr_data = VPI_DATA;
static avr_io_addr_t vpi_addr_exit = VPI_EXIT;

static uint8_t vpi_read(struct avr_t *avr, avr_io_addr_t addr, void *param) {
    vpi_t *vpi = (vpi_t *)param;
    (void)avr;

    if (addr == vpi_addr_data) {
        return vpi->memory[vpi->address++ & 0xffffff];
    }

    if (addr == vpi_addr_lo) {
        return vpi->address & 0xff;
    }
    if (addr == vpi_addr_mid) {
        return (vpi->address >> 8) & 0xff;
    }
    if (addr == vpi_addr_hi) {
        return (vpi->address >> 16) & 0xff;
    }
    return 0;
}

static void vpi_write(struct avr_t *avr, avr_io_addr_t addr, uint8_t v, void *param) {
    vpi_t *vpi = (vpi_t *)param;
    (void)avr;

    if (addr == vpi_addr_lo) {
        vpi->address = (vpi->address & 0xffff00) | v;
        return;
    }
    if (addr == vpi_addr_mid) {
        vpi->address = (vpi->address & 0xff00ff) | ((uint32_t)v << 8);
        return;
    }
    if (addr == vpi_addr_hi) {
        vpi->address = (vpi->address & 0x00ffff) | ((uint32_t)v << 16);
        return;
    }
    if (addr == vpi_addr_data) {
        vpi->memory[vpi->address++ & 0xffffff] = v;
        return;
    }
    if (addr == vpi_addr_exit) {
        vpi->exit_status = v;
        vpi->halted = 1;
        if (verbose) {
            msgf("VPI: exit status 0x%02x\n", v);
        }
        return;
    }
}

static const char *irq_names[] = {NULL};

void vpi_init(avr_t *avr, vpi_t *vpi) {
    vpi->io.avr = avr;
    vpi->io.kind = "vpi";
    vpi->io.irq_names = irq_names;
    vpi->io.reset = NULL;
    vpi->io.ioctl = NULL;

    avr_register_io(avr, &vpi->io);
    avr_register_io_read(avr, vpi_addr_lo, vpi_read, vpi);
    avr_register_io_read(avr, vpi_addr_mid, vpi_read, vpi);
    avr_register_io_read(avr, vpi_addr_hi, vpi_read, vpi);
    avr_register_io_read(avr, vpi_addr_data, vpi_read, vpi);
    avr_register_io_write(avr, vpi_addr_lo, vpi_write, vpi);
    avr_register_io_write(avr, vpi_addr_mid, vpi_write, vpi);
    avr_register_io_write(avr, vpi_addr_hi, vpi_write, vpi);
    avr_register_io_write(avr, vpi_addr_data, vpi_write, vpi);
    avr_register_io_write(avr, vpi_addr_exit, vpi_write, vpi);
}

void vpi_set_memory(vpi_t *vpi, uint8_t *memory) {
    vpi->memory = memory;
}

uint8_t vpi_get_exit_status(const vpi_t *vpi) {
    return vpi->exit_status;
}
