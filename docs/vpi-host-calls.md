# VPI host-call contract

The AVR firmware runs the RISC-V device models. Those models call host hooks for disk I/O, console output, network output, entropy and time. Under simavr the hooks become synchronous VPI host calls: the AVR writes arguments and a call code to VPI IO registers, the host callback runs inside `avr_run()` and writes the result back. No threads and no polling are needed for a call.

## IO registers

The addresses are IO addresses as used by simavr's `avr_register_io_read()` and `avr_register_io_write()`. The AVR side uses `_SFR_IO8(address - 0x20)`.

| Address | Name | Access | Function |
| --- | --- | --- | --- |
| 0x31 | ADDR_LO | R/W | bits 0-7 of the 32-bit offset |
| 0x32 | ADDR_MID | R/W | bits 8-15 |
| 0x33 | ADDR_HI | R/W | bits 16-23 |
| 0x34 | DATA | R/W | byte at the selected offset, auto-increments the offset |
| 0x36 | ADDR_X | R/W | bits 24-31 of the 32-bit offset |
| 0x38 | CALL | W | request code; the host executes the call and resets the result index |
| 0x3A | ARGRES | R/W | write appends an argument byte, read returns a result byte; both auto-increment |
| 0x3C | STATUS | R | 0 = idle, 1 = done, 2 = error |
| 0x3D | SPACE | W | 0 = guest RAM, 1 = host scratch buffer |
| 0x3E | RX_LEN | R | number of bytes available in the receive buffer |
| 0x3F | RX_DATA | R | reads one receive byte and decrements RX_LEN |
| 0x39 | EXIT | W | exit status; stops simavr |

`SPACE` selects the target of `ADDR`/`DATA`. Guest RAM is 64 MiB at RISC-V address `0x80000000`; the offset is the RISC-V address minus `0x80000000`. The scratch buffer is 4096 bytes on the host and is used for bulk data that must not consume guest RAM.

## Call protocol

1. Set `SPACE` to the buffer that holds any payload.
2. Write the payload through `ADDR`/`DATA`.
3. Write each argument byte to `ARGRES` in order.
4. Write the call code to `CALL`. The host runs the call synchronously.
5. Read `STATUS`. On success, read the result bytes from `ARGRES` in order and read any returned payload through `ADDR`/`DATA`.

Writing `CALL` resets the result index to zero. Writing `ARGRES` after the call does not change the result.

## Call codes

Multi-byte values are little-endian.

| Code | Name | Arguments | Result | Payload |
| --- | --- | --- | --- | --- |
| 1 | UART_TX | byte | - | - |
| 2 | CONSOLE_TX | length: u16 | - | scratch[0..length] to host stdout |
| 3 | DISK_READ | sector: u64, count: u8 | ok: u8 | scratch[0..count*512] from host disk |
| 4 | DISK_WRITE | sector: u64, count: u8 | ok: u8 | scratch[0..count*512] to host disk |
| 5 | DISK_FLUSH | - | ok: u8 | - |
| 6 | NET_TX | length: u16 | - | scratch[0..length] to host TAP |
| 7 | RNG_FILL | length: u16 | - | scratch[0..length] from host entropy |
| 8 | RTC_REALTIME | - | ns: u64 | - |
| 9 | RTC_MONOTONIC | - | ns: u64 | - |
| 10 | TIME_SOURCE | - | us: u64 | - |
| 11 | DISK_CAPACITY | - | sectors: u64 | - |

`TIME_SOURCE` returns the CLINT `mtime` in microseconds. `DISK_CAPACITY` returns the number of 512-byte sectors of the attached disk. The SYSCON device writes `EXIT` on guest poweroff and reboot: `0x00` for a successful poweroff, `0x01` for a failed poweroff and `0x02` for a reboot. The host treats only `0x00` as a passing exit.

## Receive path

The host main loop polls stdin and appends bytes to a 1024-byte receive buffer. The AVR polls `RX_LEN` and drains `RX_DATA`. Network receive uses the same buffer with a two-byte little-endian frame length followed by the frame bytes.
