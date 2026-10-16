/*
Copyright Marc Ketel
SPDX-License-Identifier: Apache-2.0
*/

#define _POSIX_C_SOURCE 199309L

#include <fcntl.h>
#include <signal.h>
#include <stdarg.h>
#include <termios.h>
#include <unistd.h>

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <string.h>

#include <simavr/avr_uart.h>
#include <simavr/sim_avr.h>
#include <simavr/sim_elf.h>
#include <simavr/sim_hex.h>

#include "vpi.h"

#define RAM_LENGTH 0x1000000

static avr_t *avr = NULL;
static vpi_t vpi;
int verbose = 0; // non-static: also read by vpi.c

#define RUN_TIMEOUT_SECONDS  300
#define BREAK_LOOP_THRESHOLD 1000

// How often (in avr_run() iterations) the run loop polls stdin. A syscall
// per iteration slows the simulation drastically.
#define UART_STDIN_POLL_INTERVAL 4096

static void uart_tx_hook(struct avr_irq_t *irq, uint32_t value, void *param) {
    (void)irq;
    (void)param;
    fputc((int)value, stdout);
    fflush(stdout);
}

// Set by the AVR_CORE_BAD_OPCODE IRQ hook the first time the AVR executes an
// unknown instruction. simavr itself only steps over the bad opcode and keeps
// running, so without this the host would spin until the run timeout.
static int bad_opcode = 0;

static void bad_opcode_hook(struct avr_irq_t *irq, uint32_t value, void *param) {
    (void)irq;
    (void)value;
    (void)param;
    bad_opcode = 1;
}

static void msgf(const char *fmt, ...) {
    fputc('\n', stderr);
    fputs("[simavr-host] ", stderr);
    va_list ap;
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
}

// Put stdin in raw, non-blocking mode so typed characters reach the
// emulated UART immediately. Only applies when stdin is a terminal;
// restored by stdin_restore() on every exit path, including signals.
static struct termios orig_termios;
static int stdin_is_tty = 0;

static void stdin_restore(void) {
    if (stdin_is_tty) {
        tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios);
    }
}

static void stdin_raw_enable(void) {
    stdin_is_tty = isatty(STDIN_FILENO);
    if (!stdin_is_tty) {
        return;
    }
    tcgetattr(STDIN_FILENO, &orig_termios);
    struct termios term = orig_termios;
    term.c_lflag &= ~(tcflag_t)(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &term);
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    atexit(stdin_restore);
    signal(SIGINT, exit);
    signal(SIGTERM, exit);
}

int main(int argc, char *argv[]) {
    int argi = 1;
    while (argi < argc && argv[argi][0] == '-' && argv[argi][1] != '\0') {
        if (strcmp(argv[argi], "-v") == 0) {
            verbose = 1;
            argi++;
        } else {
            msgf("Error: unknown option: %s\n", argv[argi]);
            return EXIT_FAILURE;
        }
    }
    if (argc - argi < 1 || argc - argi > 2) {
        msgf("Usage: %s [-v] <firmware.hex> [<test.bin>]\n", argv[0]);
        return EXIT_FAILURE;
    }
    const char *firmware_hex = argv[argi];
    const char *testbin = (argc - argi == 2) ? argv[argi + 1] : NULL;

    avr = avr_make_mcu_by_name("atmega1284p");
    if (avr == NULL) {
        msgf("Error: failed to create atmega1284p core\n");
        return EXIT_FAILURE;
    }

    avr_init(avr);

    uint8_t *memory = malloc(RAM_LENGTH);
    if (memory == NULL) {
        msgf("Error: failed to allocate %d bytes of RISC-V RAM\n", RAM_LENGTH);
        return EXIT_FAILURE;
    }
    memset(memory, 0, RAM_LENGTH);
    vpi_init(avr, &vpi);
    vpi_set_memory(&vpi, memory);

    if (testbin != NULL) {
        FILE *fbin = fopen(testbin, "rb");
        if (fbin == NULL) {
            msgf("Error: file not found: %s\n", testbin);
            return EXIT_FAILURE;
        }
        size_t binsize = fread(memory, 1, RAM_LENGTH, fbin);
        fclose(fbin);
        msgf("Loaded %zu bytes of test binary into VPI RAM\n", binsize);
    }

    elf_firmware_t firmware;
    memset(&firmware, 0, sizeof(firmware));
    sim_setup_firmware(firmware_hex, 0, &firmware, argv[0]);
    avr_load_firmware(avr, &firmware);
    if (verbose) {
        msgf("Verbose: firmware %s, test binary %s\n", firmware_hex,
             testbin != NULL ? testbin : "(none, smoke test)");
    }

    avr_irq_t *uart0_tx =
        avr_io_getirq(avr, AVR_IOCTL_UART_GETIRQ('0'), UART_IRQ_OUTPUT);
    if (uart0_tx == NULL) {
        msgf("Error: UART0 TX IRQ not found\n");
        return EXIT_FAILURE;
    }
    avr_irq_register_notify(uart0_tx, uart_tx_hook, NULL);

    avr_irq_t *uart0_rx =
        avr_io_getirq(avr, AVR_IOCTL_UART_GETIRQ('0'), UART_IRQ_INPUT);
    if (uart0_rx == NULL) {
        msgf("Error: UART0 RX IRQ not found\n");
        return EXIT_FAILURE;
    }

    avr_irq_t *core_bad_opcode = avr_get_core_irq(avr, AVR_CORE_BAD_OPCODE);
    if (core_bad_opcode != NULL) {
        avr_irq_register_notify(core_bad_opcode, bad_opcode_hook, avr);
    }

    uint32_t uart_flags = AVR_UART_FLAG_POLL_SLEEP;
    avr_ioctl(avr, AVR_IOCTL_UART_SET_FLAGS('0'), &uart_flags);

    struct timespec start;
    clock_gettime(CLOCK_MONOTONIC, &start);
    int timed_out = 0;
    int break_loop = 0;
    unsigned break_count = 0;
    uint16_t prev_pc = 0xffff;
    uint16_t prev2_pc = 0xffff;
    unsigned long run_iterations = 0;
    stdin_raw_enable();
    for (;;) {
        int state = avr_run(avr);
        if (state == cpu_Sleeping || state == cpu_Crashed || state == cpu_Done) {
            if (verbose) {
                msgf("Verbose: run loop ended with CPU state %d at pc 0x%04x\n",
                     state, avr->pc);
            }
            break;
        }
        // Feed any typed characters into the emulated UART. Polling stdin on
        // every avr_run() call slows the simulation drastically, so only do
        // it once every UART_STDIN_POLL_INTERVAL iterations.
        if (++run_iterations % UART_STDIN_POLL_INTERVAL == 0) {
            uint8_t ch;
            while (read(STDIN_FILENO, &ch, 1) == 1) {
                avr_raise_irq(uart0_rx, ch);
            }
        }
        if (bad_opcode) {
            if (verbose) {
                msgf("Verbose: bad AVR opcode at 0x%04x\n", avr->pc);
            }
            break;
        }
        uint16_t pc = avr->pc;
        int pc_is_break =
            ((uint32_t)pc + 1 < avr->flashend && avr->flash[pc] == 0x98 &&
             avr->flash[pc + 1] == 0x95);
        int prev_is_break =
            ((uint32_t)prev_pc + 1 < avr->flashend &&
             avr->flash[prev_pc] == 0x98 && avr->flash[prev_pc + 1] == 0x95);
        if (pc == prev2_pc && (pc_is_break || prev_is_break)) {
            if (++break_count >= BREAK_LOOP_THRESHOLD) {
                if (verbose) {
                    msgf("Verbose: BREAK loop detected at 0x%04x\n", pc);
                }
                break_loop = 1;
                break;
            }
        } else {
            break_count = 0;
        }
        prev2_pc = prev_pc;
        prev_pc = pc;
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        double elapsed = (double)(now.tv_sec - start.tv_sec) +
                         (double)(now.tv_nsec - start.tv_nsec) / 1e9;
        if (elapsed >= RUN_TIMEOUT_SECONDS) {
            msgf("Error: run loop timed out after %d s\n", RUN_TIMEOUT_SECONDS);
            timed_out = 1;
            break;
        }
    }

    stdin_restore();

    fputc('\n', stdout);

    msgf("Statistics: %llu cycles (~%.6f s simulated at 20 MHz)\n",
         (unsigned long long)avr->cycle, (double)avr->cycle / 20000000.0);

    free(memory);

    if (timed_out) {
        return EXIT_FAILURE;
    }
    if (testbin == NULL) {
        return EXIT_SUCCESS;
    }
    if (!vpi.halted) {
        msgf("Error: firmware finished without writing VPI EXIT\n");
        return EXIT_FAILURE;
    }
    if (verbose) {
        msgf("Verbose: VPI exit status=0x%02x\n", vpi_get_exit_status(&vpi));
    }
    if (break_loop && !vpi.halted) {
        msgf("Error: firmware halted without writing VPI EXIT\n");
        return EXIT_FAILURE;
    }
    if (vpi_get_exit_status(&vpi) == VPI_EXIT_PASS) {
        return EXIT_SUCCESS;
    }
    msgf("Test failed. exit status=0x%02x\n", vpi_get_exit_status(&vpi));
    return EXIT_FAILURE;
}
