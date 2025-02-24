# Programming environment

## Register map

| Addr | Width  | Name | Description  |
|------|-|-----|----|
| `0xE0` |2 | SPI_CNT | SPI control register |
| `0xE2`|1 | POW_CNT | Power control |
| `0xE3` |1| NILE_IRQ | Controls nileswan IRQ |
| `0xE4`|2 | BANK_MASK | Mask for bank index |

## Banks

**ROM**

| Bank(s) | Description |
|------------|-------------|
| 0-127 | PSRAM (8 MB) |
| 128-255 | Reserved for PSRAM expansion to 16 MB |
| 256-509 | Unused (open bus) |
| 510 | SPI RX buffer (read only, 512 bytes mirrored) |
| 511 | Bootrom |

**RAM**

| Bank(s) | Description |
|------------|-------------|
| 0-7 | SRAM (512 KB) |
| 8-14 | Unused (open bus) |
| 15 | SPI TX buffer (write only, 512 bytes mirrored) |

SPI RX and TX buffer are double buffered with only one currently visible. This is contrable via `SPI_CNT`.

## Registers

### SPI

**`0xE0` - `SPI_CNT` (16 bit)**
| Bit(s) | Description |
|------|------|
|0-8|SPI transfer length in bytes minus one|
|9-10|Mode (0 = write, 1 = read, 2 = exchange, 3 = wait and read) |
|11|Transfer speed (0 = 25 MHz, 1 = 390.625 kHz)|
|12|Controls chip select line of the current device (0 = /CS is high, 1 = /CS is low)|
|13|SPI device (0 = SPI flash, 1 = TF)|
|14|Memory mapped RX and TX buffer index (0-1)|
|15|Start/busy, when written (0=abort transfer, 1=start transfer), when read (0=idle, 1=transfering)|

Rationale for separating device select from chip select: The TF is hooked up on a separate SPI bus so that it does not interfere while the FPGA initialise. Additionally it is necessary to be eable to send data while the TF card is deselected. Having both on a separate bus also allows safely cutting power to the TF card.

* In read mode all output serial bits are 1. The incoming bits are stored in the RX buffer.
* In write mode bits from the TX buffer are output. The incoming serial bits are discarded.
* In exchange mode the TX buffer is output, while simultanously the RX buffer is populated with incoming data.
* Wait and read mode behaves like read mode except bytes are only stored with the first byte received which is not 0xFF.

During transfer the data in the TX buffer currently not mapped into the address space is sent out. The received data is stored in the RX buffer currently not memory mapped. The transfer always starts from the beginning of the buffers.

All values besides the transfer abort are read only while a transfer is in progress.

### Power

**`0xE2` - `POW_CNT` (8 bit)**
| Bit(s) | Description |
|------|------|
|0|Enable 25 MHz high frequency clock. If disabled SPI will stop working. (0=off, 1=on)|
|1|Enable TF power (0=off, 1=on)|
|2-7|Unused/0|

By default the HF oscillator is enabled and TF power is disabled.

### Interrupts

**`0xE2` - `NILE_IRQ` (8 bit)**
| Bit(s) | Description |
|------|------|
|0|Enable SPI IRQ generation|
|1|SPI IRQ status, when read (0 = no IRQ, 1 = IRQ), when written (0 = nothing, 1 = acknowledge IRQ)|
|2-7|Unused/0|

If SPI IRQ generation is enabled an IRQ will be generated whenever the the busy bit of `SPI_CNT` changes from 1 to 0. It is signalled and acknowledgeable via the SPI IRQ status bit.

### Banking control

**`0xE4` - `BANK_MASK` (16 bit)**
| Bit(s) | Description |
|------|------|
|0-8|Mask to be applied to ROM bank index|
|9-11|Unused/0|
|12-15|Mask to be applied to RAM bank index|

To allow booting from bootrom the masks are initialised with all bits set.
