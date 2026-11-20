/*
Copyright Marc Ketel
SPDX-License-Identifier: Apache-2.0
*/

#define _POSIX_C_SOURCE 200809L

#include <fcntl.h>
#include <signal.h>
#include <stdarg.h>
#include <sys/stat.h>
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

#include <RiscvEmulatorSnapshot.h>

#include "vpi.h"

#define RAM_ORIGIN  0x80000000u
#define RAM_LENGTH  0x4000000u
#define DTB_ADDRESS 0x82200000u
#define MAX_LOADS   8

static avr_t *avr = NULL;
static vpi_t vpi;
int verbose = 0; // non-static: also read by vpi.c

#define BREAK_LOOP_THRESHOLD 1000

static unsigned run_timeout_seconds = 0;
static const char *snapshot_prefix = NULL;
static const char *restore_prefix = NULL;
static volatile sig_atomic_t snapshot_requested = 0;

// How often (in avr_run() iterations) the run loop polls stdin. A syscall
// per iteration slows the simulation drastically.
#define UART_STDIN_POLL_INTERVAL 4096

static struct {
    uint32_t address;
    const char *path;
} loads[MAX_LOADS];
static size_t load_count = 0;
static const char *dtbpath = NULL;
static const char *diskpath = NULL;
static const char *tappath = NULL;
static int disk_fd = -1;
static int tap_fd = -1;

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

static void msgf(const char *fmt, ...);

/**
 * Requests a snapshot from the SIGUSR1 handler.
 */
static void snapshot_signal_handler(int signal_number) {
    (void)signal_number;
    snapshot_requested = 1;
}

/**
 * Writes the snapshot blob and guest RAM to disk.
 */
static void snapshot_save(const uint8_t *memory) {
    char json_path[1024];
    char ram_path[1024];
    char json_tmp[1024];
    char ram_tmp[1024];
    if (snprintf(json_path, sizeof(json_path), "%s.json", snapshot_prefix) >= (int)sizeof(json_path) ||
        snprintf(ram_path, sizeof(ram_path), "%s.bin", snapshot_prefix) >= (int)sizeof(ram_path) ||
        snprintf(json_tmp, sizeof(json_tmp), "%s.json.tmp", snapshot_prefix) >= (int)sizeof(json_tmp) ||
        snprintf(ram_tmp, sizeof(ram_tmp), "%s.bin.tmp", snapshot_prefix) >= (int)sizeof(ram_tmp)) {
        msgf("Snapshot path is too long: %s\n", snapshot_prefix);
        return;
    }

    uint32_t offset = RVE_SNAPSHOT_META_ADDRESS - RAM_ORIGIN;
    uint32_t magic = 0;
    uint32_t length = 0;
    memcpy(&magic, &memory[offset], sizeof(magic));
    memcpy(&length, &memory[offset + 4u], sizeof(length));
    if (magic != 0x5256534Eu || length != sizeof(rve_snapshot_t)) {
        msgf("Snapshot blob is not ready in guest RAM\n");
        return;
    }
    rve_snapshot_t blob;
    memcpy(&blob, &memory[offset + 8u], sizeof(blob));
    if (blob.magic != RVE_SNAPSHOT_MAGIC || blob.version != RVE_SNAPSHOT_VERSION) {
        msgf("Snapshot blob has an incompatible header\n");
        return;
    }

    FILE *json = fopen(json_tmp, "w");
    if (json == NULL) {
        msgf("Cannot write %s\n", json_tmp);
        return;
    }
    FILE *ram = fopen(ram_tmp, "wb");
    if (ram == NULL) {
        msgf("Cannot write %s\n", ram_tmp);
        fclose(json);
        remove(json_tmp);
        return;
    }
    rve_snapshot_json_write(json, &blob);
    if (fclose(json) != 0 || fwrite(memory, 1, RAM_LENGTH, ram) != RAM_LENGTH || fclose(ram) != 0) {
        msgf("Cannot write snapshot files for %s\n", snapshot_prefix);
        remove(json_tmp);
        remove(ram_tmp);
        return;
    }
    if (rename(json_tmp, json_path) != 0 || rename(ram_tmp, ram_path) != 0) {
        msgf("Cannot install snapshot files for %s\n", snapshot_prefix);
        remove(json_tmp);
        remove(ram_tmp);
        return;
    }
    msgf("Snapshot written to %s and %s after %llu instructions\n", json_path, ram_path,
         (unsigned long long)blob.instructions);
}

/**
 * Loads a snapshot into guest RAM and asks the firmware to restore its state.
 */
static void snapshot_load(uint8_t *memory) {
    char json_path[1024];
    char ram_path[1024];
    if (snprintf(json_path, sizeof(json_path), "%s.json", restore_prefix) >= (int)sizeof(json_path) ||
        snprintf(ram_path, sizeof(ram_path), "%s.bin", restore_prefix) >= (int)sizeof(ram_path)) {
        msgf("Snapshot path is too long: %s\n", restore_prefix);
        return;
    }
    FILE *json = fopen(json_path, "r");
    if (json == NULL) {
        msgf("Cannot read %s\n", json_path);
        return;
    }
    if (fseek(json, 0, SEEK_END) != 0) {
        fclose(json);
        return;
    }
    long size = ftell(json);
    if (size <= 0 || size > 1024 * 1024 || fseek(json, 0, SEEK_SET) != 0) {
        fclose(json);
        return;
    }
    char *text = malloc((size_t)size + 1u);
    if (text == NULL || fread(text, 1, (size_t)size, json) != (size_t)size) {
        free(text);
        fclose(json);
        return;
    }
    text[size] = '\0';
    fclose(json);

    rve_snapshot_t blob;
    memset(&blob, 0, sizeof(blob));
    bool ok = rve_snapshot_json_read(text, &blob);
    free(text);
    if (!ok || blob.magic != RVE_SNAPSHOT_MAGIC || blob.version != RVE_SNAPSHOT_VERSION ||
        blob.ram_origin != RAM_ORIGIN || blob.ram_size != RAM_LENGTH) {
        msgf("Snapshot %s has an incompatible header\n", json_path);
        return;
    }

    FILE *ram = fopen(ram_path, "rb");
    if (ram == NULL) {
        msgf("Cannot read %s\n", ram_path);
        return;
    }
    size_t got = fread(memory, 1, RAM_LENGTH, ram);
    fclose(ram);
    if (got != RAM_LENGTH) {
        msgf("Snapshot RAM %s has %zu bytes, expected %u\n", ram_path, got, RAM_LENGTH);
        return;
    }

    uint32_t offset = RVE_SNAPSHOT_META_ADDRESS - RAM_ORIGIN;
    uint32_t magic = 0x5256534Eu;
    uint32_t length = sizeof(blob);
    memcpy(&memory[offset], &magic, sizeof(magic));
    memcpy(&memory[offset + 4u], &length, sizeof(length));
    memcpy(&memory[offset + 8u], &blob, sizeof(blob));
    vpi_set_snapshot_request(&vpi, 2);
    msgf("Restored snapshot %s at %llu instructions, pc=0x%08X\n", json_path,
         (unsigned long long)blob.instructions, blob.cpu_pc);
}

/**
 * Prints host diagnostics.
 */
static void msgf(const char *fmt, ...) {
    fputc('\n', stderr);
    fputs("[simavr-host] ", stderr);
    va_list ap;
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
}

// Put stdin in raw, non-blocking mode so typed characters reach the emulated
// UART immediately. Only applies when stdin is a terminal; restored by
// stdin_restore() on every exit path, including signals.
static struct termios orig_termios;
static int stdin_is_tty = 0;

static void stdin_restore(void) {
    if (stdin_is_tty) {
        tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios);
    }
}

static void stdin_raw_enable(void) {
    // The run loop polls stdin with read() every UART_STDIN_POLL_INTERVAL
    // iterations. Piped stdin must be non-blocking too, otherwise a read()
    // with no data available stalls the whole simulation.
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (flags >= 0) {
        (void)fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    }

    stdin_is_tty = isatty(STDIN_FILENO);
    if (!stdin_is_tty) {
        return;
    }
    tcgetattr(STDIN_FILENO, &orig_termios);
    struct termios term = orig_termios;
    term.c_lflag &= ~(tcflag_t)(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &term);
    atexit(stdin_restore);
    signal(SIGINT, exit);
    signal(SIGTERM, exit);
}

static int load_image(uint8_t *memory, uint32_t address, const char *path) {
    if (address < RAM_ORIGIN || address >= RAM_ORIGIN + RAM_LENGTH) {
        msgf("Error: load address 0x%08X is outside RAM\n", address);
        return 0;
    }
    FILE *file = fopen(path, "rb");
    if (file == NULL) {
        msgf("Error: file not found: %s\n", path);
        return 0;
    }
    size_t offset = (size_t)(address - RAM_ORIGIN);
    size_t size = fread(&memory[offset], 1, (size_t)RAM_LENGTH - offset, file);
    if (ferror(file)) {
        fclose(file);
        msgf("Error reading %s\n", path);
        return 0;
    }
    fclose(file);
    if (verbose) {
        msgf("Loaded %zu bytes at 0x%08X from %s\n", size, address, path);
    }
    return 1;
}

int main(int argc, char *argv[]) {
    int argi = 1;
    while (argi < argc && argv[argi][0] == '-' && argv[argi][1] != '\0') {
        if (strcmp(argv[argi], "-v") == 0) {
            verbose = 1;
            argi++;
        } else if (strcmp(argv[argi], "-l") == 0 && argi + 1 < argc) {
            argi++;
            char *end;
            unsigned long address = strtoul(argv[argi], &end, 0);
            if (*end != ',' || end[1] == '\0') {
                msgf("Error: -l requires <address>,<file>, got %s\n", argv[argi]);
                return EXIT_FAILURE;
            }
            if (load_count == MAX_LOADS) {
                msgf("Error: at most %d -l options are supported\n", MAX_LOADS);
                return EXIT_FAILURE;
            }
            loads[load_count].address = (uint32_t)address;
            loads[load_count].path = end + 1;
            load_count++;
            argi++;
        } else if (strcmp(argv[argi], "-b") == 0 && argi + 1 < argc) {
            argi++;
            char *end;
            unsigned long address = strtoul(argv[argi], &end, 0);
            if (*end == ',') {
                if (address != DTB_ADDRESS) {
                    msgf("Error: the firmware expects the DTB at 0x%08X\n", DTB_ADDRESS);
                    return EXIT_FAILURE;
                }
                dtbpath = end + 1;
            } else {
                dtbpath = argv[argi];
            }
            argi++;
        } else if (strcmp(argv[argi], "-D") == 0 && argi + 1 < argc) {
            diskpath = argv[argi + 1];
            argi += 2;
        } else if (strcmp(argv[argi], "-N") == 0 && argi + 1 < argc) {
            tappath = argv[argi + 1];
            argi += 2;
        } else if (strcmp(argv[argi], "-S") == 0 && argi + 1 < argc) {
            snapshot_prefix = argv[argi + 1];
            argi += 2;
        } else if (strcmp(argv[argi], "-R") == 0 && argi + 1 < argc) {
            restore_prefix = argv[argi + 1];
            argi += 2;
        } else if (strcmp(argv[argi], "-T") == 0 && argi + 1 < argc) {
            char *end;
            unsigned long seconds = strtoul(argv[argi + 1], &end, 10);
            if (*end != '\0' || seconds == 0) {
                msgf("Error: -T requires a positive number of seconds\n");
                return EXIT_FAILURE;
            }
            run_timeout_seconds = (unsigned)seconds;
            argi += 2;
        } else {
            msgf("Error: unknown option: %s\n", argv[argi]);
            return EXIT_FAILURE;
        }
    }
    if (argc - argi < 1 || argc - argi > 2) {
        msgf("Usage: %s [-v] [-l <address>,<file>] [-b <file>] [-D <disk>] [-N <ifname>] [-S <prefix>] [-R <prefix>] [-T <seconds>] <firmware.hex> [<test.bin>]\n", argv[0]);
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
        msgf("Error: failed to allocate %u bytes of RISC-V RAM\n", RAM_LENGTH);
        return EXIT_FAILURE;
    }
    memset(memory, 0, RAM_LENGTH);
    vpi_init(avr, &vpi);
    vpi_set_memory(&vpi, memory, RAM_LENGTH);

    if (testbin != NULL && !load_image(memory, RAM_ORIGIN, testbin)) {
        free(memory);
        return EXIT_FAILURE;
    }
    for (size_t i = 0; i < load_count; i++) {
        if (!load_image(memory, loads[i].address, loads[i].path)) {
            free(memory);
            return EXIT_FAILURE;
        }
    }
    if (dtbpath != NULL && !load_image(memory, DTB_ADDRESS, dtbpath)) {
        free(memory);
        return EXIT_FAILURE;
    }
    if (restore_prefix != NULL) {
        snapshot_load(memory);
    }
    if (snapshot_prefix != NULL) {
        struct sigaction action = {0};
        action.sa_handler = snapshot_signal_handler;
        sigemptyset(&action.sa_mask);
        if (sigaction(SIGUSR1, &action, NULL) != 0) {
            msgf("Cannot install the SIGUSR1 handler\n");
            free(memory);
            return EXIT_FAILURE;
        }
    }

    if (diskpath != NULL) {
        disk_fd = open(diskpath, O_RDWR);
        if (disk_fd < 0) {
            msgf("Error: cannot open disk %s\n", diskpath);
            free(memory);
            return EXIT_FAILURE;
        }
        struct stat st;
        if (fstat(disk_fd, &st) != 0 || st.st_size <= 0 || (st.st_size % 512) != 0) {
            msgf("Error: disk size must be a positive multiple of 512 bytes\n");
            close(disk_fd);
            free(memory);
            return EXIT_FAILURE;
        }
        vpi_set_disk(&vpi, disk_fd, (uint64_t)st.st_size / 512u);
    }

    if (tappath != NULL) {
        tap_fd = open("/dev/net/tun", O_RDWR | O_NONBLOCK);
        if (tap_fd < 0) {
            msgf("Error: cannot open /dev/net/tun\n");
            if (disk_fd >= 0) {
                close(disk_fd);
            }
            free(memory);
            return EXIT_FAILURE;
        }
        vpi_set_tap(&vpi, tap_fd);
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
        if (vpi_take_snapshot_ready(&vpi) != 0 && snapshot_prefix != NULL) {
            snapshot_save(memory);
        }
        if (snapshot_requested != 0) {
            snapshot_requested = 0;
            if (snapshot_prefix != NULL) {
                vpi_set_snapshot_request(&vpi, 1);
            }
        }
        int state = avr_run(avr);
        if (state == cpu_Sleeping || state == cpu_Crashed || state == cpu_Done) {
            if (verbose) {
                msgf("Verbose: run loop ended with CPU state %d at pc 0x%04x\n",
                     state, avr->pc);
            }
            break;
        }
        // Feed any typed characters into the emulated UART and the VPI
        // receive buffer. Polling stdin on every avr_run() call slows the
        // simulation drastically, so only do it once every interval.
        if (++run_iterations % UART_STDIN_POLL_INTERVAL == 0) {
            uint8_t ch;
            while (read(STDIN_FILENO, &ch, 1) == 1) {
                avr_raise_irq(uart0_rx, ch);
                vpi_rx_push(&vpi, ch);
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
        if (run_timeout_seconds != 0 && elapsed >= (double)run_timeout_seconds) {
            msgf("Error: run loop timed out after %u s\n", run_timeout_seconds);
            timed_out = 1;
            break;
        }
    }

    stdin_restore();

    fputc('\n', stdout);

    msgf("Statistics: %llu cycles (~%.6f s simulated at 20 MHz)\n",
         (unsigned long long)avr->cycle, (double)avr->cycle / 20000000.0);

    free(memory);
    if (disk_fd >= 0) {
        close(disk_fd);
    }
    if (tap_fd >= 0) {
        close(tap_fd);
    }

    if (timed_out) {
        return EXIT_FAILURE;
    }
    if (testbin == NULL && dtbpath == NULL && load_count == 0) {
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
