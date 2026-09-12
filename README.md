# RISC-V-emulator-SimAVR

simavr-host: a simavr wrapper that runs AVR firmware (Intel HEX) on an atmega1284p core and executes the RISC-V ACT tests via a VPI peripheral.

## How it works

- **[`src/main.c`](src/main.c)** — host wrapper. Loads the firmware, taps the UART0 TX IRQ (every transmitted byte goes to stdout) and runs the simulation. With a `<test.bin>` the RISC-V test binary is loaded into the 16 MiB host RAM of the VPI peripheral (byte 0 = RISC-V address 0x80000000); the exit code is then the test result (0 = pass, 1 = fail). Without a test binary: smoke test. The simulation stops on power-down sleep, a BREAK halt loop (opcode 0x9598, detected via a repeating 2-instruction PC cycle) or a 25 s wall-clock timeout. `-v` enables verbose output on stderr.
- **[`src/vpi.c`](src/vpi.c) / [`src/vpi.h`](src/vpi.h)** — VPI peripheral: a pure memory device that backs the RISC-V RAM with host memory and carries the test exit status. Register contract (IO addresses, free in iom1284p.h and the simavr atmega1284p core, < 0x60 for 1-cycle `in`/`out`):

  | Addr | Register | Function                                       |
  | ---- | -------- | ---------------------------------------------- |
  | 0x31 | ADDR_LO  | bits 0–7 of the 24-bit RAM offset              |
  | 0x32 | ADDR_MID | bits 8–15                                      |
  | 0x33 | ADDR_HI  | bits 16–23                                     |
  | 0x34 | DATA     | byte read/write; every access auto-increments  |
  | 0x39 | EXIT     | exit status: 0x00 = pass, else fail; stops sim |

  24-bit addressing because the largest ACT test binary (139392 bytes) exceeds a 16-bit window. The firmware computes offset = address − 0x80000000.

## Usage

```
pio run -e simavr-host && .pio/build/simavr-host/simavr-host firmware.hex [test.bin]
```

`firmware.hex` is the AVR firmware (Intel HEX) that runs on the simulated atmega1284p and drives the RISC-V emulator. `test.bin` is the optional RISC-V test binary loaded into the VPI RAM; its exit code is the test result.

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
