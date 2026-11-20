/*
Copyright Marc Ketel
SPDX-License-Identifier: Apache-2.0
*/

#define _GNU_SOURCE

#include "vpi.h"

#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <sys/random.h>
#include <time.h>
#include <unistd.h>

#include <simavr/sim_io.h>

extern int verbose;

static void msgf(const char *fmt, ...) {
    fputc('\n', stderr);
    fputs("[simavr-host] ", stderr);
    va_list ap;
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
}

static avr_io_addr_t vpi_addr_lo = VPI_ADDR_LO;
static avr_io_addr_t vpi_addr_mid = VPI_ADDR_MID;
static avr_io_addr_t vpi_addr_hi = VPI_ADDR_HI;
static avr_io_addr_t vpi_addr_x = VPI_ADDR_X;
static avr_io_addr_t vpi_addr_data = VPI_DATA;
static avr_io_addr_t vpi_addr_call = VPI_CALL;
static avr_io_addr_t vpi_addr_argres = VPI_ARGRES;
static avr_io_addr_t vpi_addr_status = VPI_STATUS;
static avr_io_addr_t vpi_addr_space = VPI_SPACE;
static avr_io_addr_t vpi_addr_rx_len = VPI_RX_LEN;
static avr_io_addr_t vpi_addr_rx_data = VPI_RX_DATA;
static avr_io_addr_t vpi_addr_exit = VPI_EXIT;
static avr_io_addr_t vpi_addr_snapshot = VPI_SNAPSHOT;

static void store_u64_le(uint8_t *destination, uint64_t value) {
    for (uint8_t i = 0; i < 8; i++) {
        destination[i] = (uint8_t)(value >> (8u * i));
    }
}

static uint64_t load_u64_le(const uint8_t *source) {
    uint64_t value = 0;
    for (uint8_t i = 0; i < 8; i++) {
        value |= (uint64_t)source[i] << (8u * i);
    }
    return value;
}

static uint16_t load_u16_le(const uint8_t *source) {
    return (uint16_t)source[0] | (uint16_t)((uint16_t)source[1] << 8u);
}

static uint64_t monotonic_us(void) {
    struct timespec value;
    if (clock_gettime(CLOCK_MONOTONIC, &value) != 0) {
        return 0;
    }
    return (uint64_t)value.tv_sec * 1000000u + (uint64_t)value.tv_nsec / 1000u;
}

static uint64_t clock_ns(int clockid) {
    struct timespec value;
    if (clock_gettime(clockid, &value) != 0) {
        return 0;
    }
    return (uint64_t)value.tv_sec * 1000000000u + (uint64_t)value.tv_nsec;
}

static uint16_t rx_length(const vpi_t *vpi) {
    return (uint16_t)((vpi->rx_tail + VPI_RX_SIZE - vpi->rx_head) % VPI_RX_SIZE);
}

static void execute_call(vpi_t *vpi, uint8_t code) {
    vpi->result_index = 0;
    vpi->status = VPI_STATUS_DONE;

    switch (code) {
        case VPI_CALL_UART_TX:
            fputc(vpi->args[0], stdout);
            fflush(stdout);
            break;

        case VPI_CALL_CONSOLE_TX: {
            uint16_t length = load_u16_le(vpi->args);
            if (length > VPI_SCRATCH_SIZE) {
                vpi->status = VPI_STATUS_ERROR;
                break;
            }
            fwrite(vpi->scratch, 1, length, stdout);
            fflush(stdout);
            break;
        }

        case VPI_CALL_DISK_READ: {
            uint64_t sector = load_u64_le(vpi->args);
            uint16_t count = vpi->args[8];
            size_t length = (size_t)count * 512u;
            if (length > VPI_SCRATCH_SIZE || vpi->disk_fd < 0) {
                vpi->status = VPI_STATUS_ERROR;
                vpi->result[0] = 0;
                break;
            }
            ssize_t got = pread(vpi->disk_fd, vpi->scratch, length, (off_t)(sector * 512u));
            vpi->result[0] = (got == (ssize_t)length) ? 1u : 0u;
            break;
        }

        case VPI_CALL_DISK_WRITE: {
            uint64_t sector = load_u64_le(vpi->args);
            uint16_t count = vpi->args[8];
            size_t length = (size_t)count * 512u;
            if (length > VPI_SCRATCH_SIZE || vpi->disk_fd < 0) {
                vpi->status = VPI_STATUS_ERROR;
                vpi->result[0] = 0;
                break;
            }
            ssize_t put = pwrite(vpi->disk_fd, vpi->scratch, length, (off_t)(sector * 512u));
            vpi->result[0] = (put == (ssize_t)length) ? 1u : 0u;
            break;
        }

        case VPI_CALL_DISK_FLUSH:
            if (vpi->disk_fd < 0 || fsync(vpi->disk_fd) != 0) {
                vpi->status = VPI_STATUS_ERROR;
                vpi->result[0] = 0;
            } else {
                vpi->result[0] = 1;
            }
            break;

        case VPI_CALL_NET_TX: {
            uint16_t length = load_u16_le(vpi->args);
            if (length > VPI_SCRATCH_SIZE || vpi->tap_fd < 0) {
                vpi->status = VPI_STATUS_ERROR;
                break;
            }
            (void)write(vpi->tap_fd, vpi->scratch, length);
            break;
        }

        case VPI_CALL_RNG_FILL: {
            uint16_t length = load_u16_le(vpi->args);
            if (length > VPI_SCRATCH_SIZE) {
                vpi->status = VPI_STATUS_ERROR;
                break;
            }
            uint8_t *bytes = vpi->scratch;
            while (length > 0) {
                ssize_t count = getrandom(bytes, length, 0);
                if (count > 0) {
                    bytes += count;
                    length = (uint16_t)(length - (uint16_t)count);
                    continue;
                }
                if (count < 0 && errno == EINTR) {
                    continue;
                }
                memset(bytes, 0, length);
                break;
            }
            break;
        }

        case VPI_CALL_RTC_REALTIME:
            store_u64_le(vpi->result, clock_ns(CLOCK_REALTIME));
            break;

        case VPI_CALL_RTC_MONOTONIC:
            store_u64_le(vpi->result, clock_ns(CLOCK_MONOTONIC));
            break;

        case VPI_CALL_TIME_SOURCE:
            store_u64_le(vpi->result, monotonic_us());
            break;

        case VPI_CALL_DISK_CAPACITY:
            store_u64_le(vpi->result, vpi->disk_capacity);
            break;

        case VPI_CALL_STATE_SAVE:
            vpi->snapshot_ready = 1;
            break;

        default:
            vpi->status = VPI_STATUS_ERROR;
            if (verbose) {
                msgf("VPI: unknown call 0x%02x\n", code);
            }
            break;
    }
    vpi->arg_index = 0;
}

static uint8_t vpi_read(struct avr_t *avr, avr_io_addr_t addr, void *param) {
    vpi_t *vpi = (vpi_t *)param;
    (void)avr;

    if (addr == vpi_addr_data) {
        uint8_t value;
        if (vpi->space == VPI_SPACE_SCRATCH) {
            value = vpi->scratch[vpi->address % VPI_SCRATCH_SIZE];
        } else {
            value = vpi->memory[vpi->address % vpi->memory_size];
        }
        vpi->address++;
        return value;
    }

    if (addr == vpi_addr_lo) {
        return (uint8_t)vpi->address;
    }
    if (addr == vpi_addr_mid) {
        return (uint8_t)(vpi->address >> 8u);
    }
    if (addr == vpi_addr_hi) {
        return (uint8_t)(vpi->address >> 16u);
    }
    if (addr == vpi_addr_x) {
        return (uint8_t)(vpi->address >> 24u);
    }
    if (addr == vpi_addr_argres) {
        uint8_t value = vpi->result[vpi->result_index % VPI_RESULT_SIZE];
        vpi->result_index++;
        return value;
    }
    if (addr == vpi_addr_status) {
        return vpi->status;
    }
    if (addr == vpi_addr_rx_len) {
        return (uint8_t)rx_length(vpi);
    }
    if (addr == vpi_addr_rx_data) {
        uint8_t value = vpi->rx[vpi->rx_head];
        vpi->rx_head = (uint16_t)((vpi->rx_head + 1u) % VPI_RX_SIZE);
        return value;
    }
    if (addr == vpi_addr_snapshot) {
        return vpi->snapshot_request;
    }
    return 0;
}

static void vpi_write(struct avr_t *avr, avr_io_addr_t addr, uint8_t value, void *param) {
    vpi_t *vpi = (vpi_t *)param;
    (void)avr;

    if (addr == vpi_addr_lo) {
        vpi->address = (vpi->address & UINT32_C(0xffffff00)) | value;
        return;
    }
    if (addr == vpi_addr_mid) {
        vpi->address = (vpi->address & UINT32_C(0xffff00ff)) | ((uint32_t)value << 8u);
        return;
    }
    if (addr == vpi_addr_hi) {
        vpi->address = (vpi->address & UINT32_C(0xff00ffff)) | ((uint32_t)value << 16u);
        return;
    }
    if (addr == vpi_addr_x) {
        vpi->address = (vpi->address & UINT32_C(0x00ffffff)) | ((uint32_t)value << 24u);
        return;
    }
    if (addr == vpi_addr_data) {
        if (vpi->space == VPI_SPACE_SCRATCH) {
            vpi->scratch[vpi->address % VPI_SCRATCH_SIZE] = value;
        } else {
            vpi->memory[vpi->address % vpi->memory_size] = value;
        }
        vpi->address++;
        return;
    }
    if (addr == vpi_addr_argres) {
        vpi->args[vpi->arg_index % VPI_ARGS_SIZE] = value;
        vpi->arg_index++;
        return;
    }
    if (addr == vpi_addr_call) {
        execute_call(vpi, value);
        return;
    }
    if (addr == vpi_addr_space) {
        vpi->space = value;
        vpi->address = 0;
        return;
    }
    if (addr == vpi_addr_exit) {
        vpi->exit_status = value;
        vpi->halted = 1;
        if (verbose) {
            msgf("VPI: exit status 0x%02x\n", value);
        }
        return;
    }
    if (addr == vpi_addr_snapshot) {
        vpi->snapshot_request = value;
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

    vpi->address = 0;
    vpi->space = VPI_SPACE_RAM;
    vpi->arg_index = 0;
    vpi->result_index = 0;
    vpi->status = VPI_STATUS_IDLE;
    vpi->exit_status = 0xff;
    vpi->halted = 0;
    vpi->snapshot_request = 0;
    vpi->snapshot_ready = 0;
    vpi->rx_head = 0;
    vpi->rx_tail = 0;
    vpi->disk_fd = -1;
    vpi->disk_capacity = 0;
    vpi->tap_fd = -1;

    avr_register_io(avr, &vpi->io);
    avr_register_io_read(avr, vpi_addr_lo, vpi_read, vpi);
    avr_register_io_read(avr, vpi_addr_mid, vpi_read, vpi);
    avr_register_io_read(avr, vpi_addr_hi, vpi_read, vpi);
    avr_register_io_read(avr, vpi_addr_x, vpi_read, vpi);
    avr_register_io_read(avr, vpi_addr_data, vpi_read, vpi);
    avr_register_io_read(avr, vpi_addr_argres, vpi_read, vpi);
    avr_register_io_read(avr, vpi_addr_status, vpi_read, vpi);
    avr_register_io_read(avr, vpi_addr_rx_len, vpi_read, vpi);
    avr_register_io_read(avr, vpi_addr_rx_data, vpi_read, vpi);
    avr_register_io_write(avr, vpi_addr_lo, vpi_write, vpi);
    avr_register_io_write(avr, vpi_addr_mid, vpi_write, vpi);
    avr_register_io_write(avr, vpi_addr_hi, vpi_write, vpi);
    avr_register_io_write(avr, vpi_addr_x, vpi_write, vpi);
    avr_register_io_write(avr, vpi_addr_data, vpi_write, vpi);
    avr_register_io_write(avr, vpi_addr_argres, vpi_write, vpi);
    avr_register_io_write(avr, vpi_addr_call, vpi_write, vpi);
    avr_register_io_write(avr, vpi_addr_space, vpi_write, vpi);
    avr_register_io_write(avr, vpi_addr_exit, vpi_write, vpi);
    avr_register_io_read(avr, vpi_addr_snapshot, vpi_read, vpi);
    avr_register_io_write(avr, vpi_addr_snapshot, vpi_write, vpi);
}

void vpi_set_memory(vpi_t *vpi, uint8_t *memory, uint32_t size) {
    vpi->memory = memory;
    vpi->memory_size = size;
}

void vpi_set_disk(vpi_t *vpi, int fd, uint64_t capacity) {
    vpi->disk_fd = fd;
    vpi->disk_capacity = capacity;
}

void vpi_set_tap(vpi_t *vpi, int fd) {
    vpi->tap_fd = fd;
}

void vpi_rx_push(vpi_t *vpi, uint8_t byte) {
    uint16_t next = (uint16_t)((vpi->rx_tail + 1u) % VPI_RX_SIZE);
    if (next == vpi->rx_head) {
        return;
    }
    vpi->rx[vpi->rx_tail] = byte;
    vpi->rx_tail = next;
}

uint8_t vpi_get_exit_status(const vpi_t *vpi) {
    return vpi->exit_status;
}

void vpi_set_snapshot_request(vpi_t *vpi, uint8_t request) {
    vpi->snapshot_request = request;
}

uint8_t vpi_take_snapshot_ready(vpi_t *vpi) {
    uint8_t ready = vpi->snapshot_ready;
    vpi->snapshot_ready = 0;
    return ready;
}
