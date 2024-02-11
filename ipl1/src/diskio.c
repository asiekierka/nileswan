/**
 * Copyright (c) 2024 Adrian Siekierka
 *
 * Nileswan IPL1 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Nileswan IPL1 is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with Nileswan IPL1. If not, see <https://www.gnu.org/licenses/>.
 */

#include <string.h>
#include <ws.h>
#include <ws/display.h>
#include <ws/util.h>
#include "nileswan/nileswan.h"
#include "fatfs/ff.h"
#include "fatfs/diskio.h"

// #define USE_DEVFS
#define USE_MULTI_TRANSFER_READS
#define USE_MULTI_TRANSFER_WRITES

#ifdef USE_DEVFS

DSTATUS disk_status(BYTE pdrv) {
	return 0;
}

DSTATUS disk_initialize(BYTE pdrv) {
	return 0;
}

DRESULT disk_read (BYTE pdrv, BYTE __far* buff, LBA_t sector, UINT count) {
	sector <<= 9;

	while (count) {
		cpu_irq_disable();
		outportw(IO_BANK_ROM1, sector >> 16);
		memcpy(buff, MK_FP(0x3000, (uint16_t) sector), 512);
		cpu_irq_enable();

		buff += 512;
		sector += 512;
		count--;
	}

	return RES_OK;
}

#if FF_FS_READONLY == 0

DRESULT disk_write (BYTE pdrv, const BYTE __far* buff, LBA_t sector, UINT count) {
	return RES_PARERR;
}

#endif

DRESULT disk_ioctl (BYTE pdrv, BYTE cmd, void *buff) {
	return RES_PARERR;
}

#else

// TF card-related defines

#define TFC_CMD(n) (0x40 | (n))
#define TFC_ACMD(n) (0xC0 | (n))
#define TFC_GO_IDLE_STATE        TFC_CMD(0)
#define TFC_SEND_OP_COND         TFC_CMD(1)
#define TFC_SEND_IF_COND         TFC_CMD(8)
#define TFC_SEND_CSD             TFC_CMD(9)
#define TFC_SEND_CID             TFC_CMD(10)
#define TFC_STOP_TRANSMISSION    TFC_CMD(12)
#define TFC_SET_BLOCKLEN         TFC_CMD(16)
#define TFC_READ_SINGLE_BLOCK    TFC_CMD(17)
#define TFC_READ_MULTIPLE_BLOCK  TFC_CMD(18)
#define TFC_SET_BLOCK_COUNT      TFC_CMD(23)
#define TFC_WRITE_BLOCK          TFC_CMD(24)
#define TFC_WRITE_MULTIPLE_BLOCK TFC_CMD(25)
#define TFC_APP_SEND_OP_COND     TFC_ACMD(41)
#define TFC_APP_PREFIX           TFC_CMD(55)
#define TFC_READ_OCR             TFC_CMD(58)
#define TFC_R1_IDLE        0x01
#define TFC_R1_ERASE_RESET 0x02
#define TFC_R1_ILLEGAL_CMD 0x04
#define TFC_R1_CRC_ERROR   0x08
#define TFC_R1_ERASE_ERROR 0x10
#define TFC_R1_ADDR_ERROR  0x20
#define TFC_R1_PARAM_ERROR 0x40

// FatFS API implementation

#ifdef NILESWAN_IPL1
uint8_t diskio_detail_code;
#define set_detail_code(v) diskio_detail_code = v
#else
#define set_detail_code(v)
#endif

/* Wait until the TF card is finished (responds 0xFF...) */
static uint8_t tfc_wait_until_ready(uint8_t resp) {
	uint16_t timeout = 0;
#ifdef __OPTIMIZE_SIZE__
	// smaller but slower code variant
	uint8_t resp_busy[1];
	while (--timeout) {
		// wait for 0xFF to signify end of busy time
		if (!nile_spi_rx_copy(&resp_busy, 1, NILE_SPI_MODE_READ))
			return 0xFF;
		if (resp_busy[0] == 0xFF)
			break;
	}

	if (!timeout)
		return 0xFF;
	return resp;
#else
	if (!nile_spi_wait_busy())
		return 0xFF;

	uint16_t cnt = inportw(IO_NILE_SPI_CNT);
	uint16_t new_cnt = (48 - 1) | NILE_SPI_MODE_READ | (cnt & 0x7800);
	volatile uint16_t prev_bank = inportw(IO_BANK_2003_ROM1);
	outportw(IO_BANK_2003_ROM1, NILE_SEG_ROM_RX);

	// send first read request
	outportw(IO_NILE_SPI_CNT, new_cnt | NILE_SPI_START);

	while (--timeout) {
		if (!nile_spi_wait_busy()) {
			resp = 0xFF;
			break;
		}

		// send second read request
		new_cnt ^= NILE_SPI_BUFFER_IDX;
		outportw(IO_NILE_SPI_CNT, new_cnt | NILE_SPI_START);

		// read first read request
		if (*((uint16_t __far*) MK_FP(0x3000, 46)) == 0xFFFF)
			break;
	}

	outportw(IO_BANK_2003_ROM1, prev_bank);
	return resp;
#endif
}

static bool tfc_cs_high(void) {
	if (!nile_spi_wait_busy())
		return false;
	outportw(IO_NILE_SPI_CNT, inportw(IO_NILE_SPI_CNT) & ~NILE_SPI_CS);
	if (!nile_spi_rx(1, NILE_SPI_MODE_READ))
		return false;
	return true;
}

static bool tfc_cs_low(void) {
	if (!nile_spi_wait_busy())
		return false;
	outportw(IO_NILE_SPI_CNT, inportw(IO_NILE_SPI_CNT) | NILE_SPI_CS);
	/* if (!nile_spi_rx(1, NILE_SPI_MODE_READ))
		return false; */
	if (tfc_wait_until_ready(0x00))
		return false;
	return true;
}

static uint8_t tfc_read_response_r1b(void) {
	uint8_t resp = 0xFF;

	if (!nile_spi_rx_copy(&resp, 1, NILE_SPI_MODE_WAIT_READ) || resp)
		return resp;

	return tfc_wait_until_ready(resp);
}

static uint8_t tfc_read_response(uint8_t *buffer, uint16_t size) {
	buffer[0] = 0xFF;
	nile_spi_rx_copy(buffer, size + 1, NILE_SPI_MODE_WAIT_READ);
	return buffer[0];
}

static bool tfc_send_cmd(uint8_t cmd, uint8_t crc, uint32_t arg) {
	uint8_t buffer[6];

	if (cmd & 0x80) {
		if (!tfc_send_cmd(TFC_APP_PREFIX, 0x95, 0)) {
			return false;
		}
		if (tfc_read_response(buffer, 1) & ~TFC_R1_IDLE) {
			return false;
		}
	}

	if (!tfc_cs_high())
		return false;
	if (!tfc_cs_low())
		return false;

	buffer[0] = cmd & 0x7F;
	buffer[1] = arg >> 24;
	buffer[2] = arg >> 16;
	buffer[3] = arg >> 8;
	buffer[4] = arg;
	buffer[5] = crc;
	return nile_spi_tx(buffer, sizeof(buffer));
}

DSTATUS disk_status(BYTE pdrv) {
	return nile_ipl_data->card_state == NILE_CARD_NOT_INITIALIZED ? STA_NOINIT : 0;
}

#define MAX_RETRIES 200

#define SCREEN ((uint16_t*) (0x3800 + (13 * 32 * 2)))
#define SCV(c) (((uint8_t) (c)) | 0x100)

DSTATUS disk_initialize(BYTE pdrv) {
	uint8_t retries;
	uint8_t buffer[16];
	bool cond1, cond2;

	nile_ipl_data->card_state = NILE_CARD_NOT_INITIALIZED;
	nile_spi_timeout_ms = 1000;

	set_detail_code(0);
	outportw(IO_NILE_SPI_CNT, NILE_SPI_DEV_TF | NILE_SPI_390KHZ | NILE_SPI_CS_HIGH);
	tfc_cs_high();
	SCREEN[32] = SCV('@');

	uint8_t powcnt = inportb(IO_NILE_POW_CNT);
	if (!(powcnt & NILE_POW_TF)) {
		SCREEN[33] = SCV('p');
		// Power card on
		powcnt |= NILE_POW_TF;
		outportb(IO_NILE_POW_CNT, powcnt);
		// Wait 250 milliseconds
		for (uint8_t i = 0; i < 5; i++)
			ws_busywait(50000);
	}

	nile_spi_rx(10, NILE_SPI_MODE_READ);

	// Reset card
	if (!(cond1 = tfc_send_cmd(TFC_GO_IDLE_STATE, 0x95, 0)) || (cond2 = (tfc_read_response(buffer, 1) & ~TFC_R1_IDLE))) {
		SCREEN[34] = cond1 ? 'Y' : 'N';
		SCREEN[35] = cond2 ? 'Y' : 'N';
		// Error/No response
		set_detail_code(1);
		goto card_init_failed;
	}

	// Query interface configuration
	if (tfc_send_cmd(TFC_SEND_IF_COND, 0x87, 0x000001AA) && !(tfc_read_response(buffer, 5) & ~TFC_R1_IDLE)) {
		SCREEN[36] = SCV('I');
		// Check voltage/pattern value match
		if ((buffer[3] & 0xF) == 0x1 && buffer[4] == 0xAA) {
			// Attempt high-capacity card init
			retries = MAX_RETRIES;
			nile_spi_timeout_ms = 10;
			while (--retries) {
				SCREEN[36]++;
				if (tfc_send_cmd(TFC_APP_SEND_OP_COND, 0x95, 1UL << 30)) {
					SCREEN[37] = SCV('O');
					uint8_t init_response = tfc_read_response(buffer, 1);
					if (init_response & ~TFC_R1_IDLE) {
						SCREEN[38] = SCV('r');
						// Initialization error
						retries = 0;
						break;
					} else if (!init_response) {
						// Initialization success
						SCREEN[39] = SCV('R');
						break;
					}
				}
				// Card still idle, try again
			}

			// Card init successful?
			if (retries) {
				SCREEN[40] = SCV('0');
				// Read OCR to check for HC card
				if (tfc_send_cmd(TFC_READ_OCR, 0x95, 0)) {
					SCREEN[41] = SCV('1');
					if (!tfc_read_response(buffer, 5)) {
						SCREEN[42] = SCV('2');
						if (buffer[1] & 0x40) {
							nile_ipl_data->card_state = NILE_CARD_BLOCK_ADDRESSING;
						}
					}
				}
				goto card_init_complete_hc;
			}
		} else {
			SCREEN[43] = SCV('3');
			// Voltage/pattern value mismatch
			set_detail_code(2);
			goto card_init_failed;
		}
	}

	// Attempt card init
	retries = MAX_RETRIES;
	nile_spi_timeout_ms = 10;
	SCREEN[44] = SCV('@');
	while (--retries) {
		SCREEN[44]++;
		if (tfc_send_cmd(TFC_APP_SEND_OP_COND, 0x95, 0)) {
			uint8_t init_response = tfc_read_response(buffer, 1);
			if (init_response & ~TFC_R1_IDLE) {
				SCREEN[45] = SCV('4');
				// Initialization error
				retries = 0;
				break;
			} else if (!init_response) {
				// Initialization success
				goto card_init_complete;
			}
		}
	}

	// Attempt legacy card init
	retries = MAX_RETRIES;
	SCREEN[46] = SCV('@');
	while (--retries) {
		SCREEN[46]++;
		if (tfc_send_cmd(TFC_SEND_OP_COND, 0x95, 0)) {
			uint8_t init_response = tfc_read_response(buffer, 1);
			if (init_response & ~TFC_R1_IDLE) {
				SCREEN[47] = SCV('5');
				// Initialization error
				retries = 0;
				break;
			} else if (!init_response) {
				// Initialization success
				goto card_init_complete;
			}
		}
	}

	set_detail_code(3);
card_init_failed:
	// Power off card
	outportb(IO_NILE_POW_CNT, 0);
	outportw(IO_NILE_SPI_CNT, 0);
	return STA_NOINIT;

card_init_complete:
	nile_spi_timeout_ms = 250;
	if (nile_ipl_data->card_state == NILE_CARD_NOT_INITIALIZED) {
		// set block size to 512
		if (tfc_send_cmd(TFC_SET_BLOCKLEN, 0x95, 512)) {
			if (tfc_read_response(buffer, 1)) {
				set_detail_code(4);
				goto card_init_failed;
			}
		}
	}

card_init_complete_hc:
	outportb(IO_NILE_POW_CNT, powcnt | NILE_POW_CLOCK);
	outportw(IO_NILE_SPI_CNT, NILE_SPI_DEV_TF | NILE_SPI_25MHZ | NILE_SPI_CS_HIGH);
	tfc_cs_high();
	nile_ipl_data->card_state = NILE_CARD_BYTE_ADDRESSING;
	return 0;
}

DRESULT disk_read (BYTE pdrv, BYTE __far* buff, LBA_t sector, UINT count) {
	uint8_t result = RES_ERROR;
	uint8_t resp[16];

	if (nile_ipl_data->card_state == NILE_CARD_BYTE_ADDRESSING)
		sector <<= 9;

	tfc_cs_low();

#ifdef USE_MULTI_TRANSFER_READS
	bool multi_transfer = count > 1;
	if (!tfc_send_cmd(multi_transfer ? TFC_READ_MULTIPLE_BLOCK : TFC_READ_SINGLE_BLOCK, 0x95, sector))
		goto disk_read_end;
	if (!nile_spi_rx_copy(resp, 1, NILE_SPI_MODE_WAIT_READ))
		goto disk_read_end;
	if (resp[0])
		goto disk_read_end;

	while (count) {
		if (!nile_spi_rx_copy(resp, 1, NILE_SPI_MODE_WAIT_READ))
			goto disk_read_stop;
		if (resp[0] != 0xFE)
			goto disk_read_stop;
		if (!nile_spi_rx_copy(buff, 512, NILE_SPI_MODE_READ))
			goto disk_read_stop;
		if (!nile_spi_rx(2, NILE_SPI_MODE_READ))
			goto disk_read_stop;
		buff += 512;
		count--;
	}

disk_read_stop:
	if (multi_transfer) {
		resp[0] = TFC_STOP_TRANSMISSION;
		resp[5] = 0x95;
		resp[6] = 0xFF; // skip one byte
		if (!nile_spi_tx(resp, 7))
			goto disk_read_end;
		if (tfc_read_response_r1b())
			goto disk_read_end;
	}
#else
	while (count) {
		if (!tfc_send_cmd(TFC_READ_SINGLE_BLOCK, 0x95, sector))
			goto disk_read_end;
		if (!nile_spi_rx_copy(resp, 1, NILE_SPI_MODE_WAIT_READ))
			goto disk_read_end;
		if (resp[0])
			goto disk_read_end;

		if (!nile_spi_rx_copy(resp, 1, NILE_SPI_MODE_WAIT_READ))
			goto disk_read_end;
		if (resp[0] != 0xFE)
			goto disk_read_end;
		if (!nile_spi_rx_copy(buff, 512, NILE_SPI_MODE_READ))
			goto disk_read_end;
		if (!nile_spi_rx(2, NILE_SPI_MODE_READ))
			goto disk_read_end;
		buff += 512;
		sector += card_hc ? 1 : 512;
		count--;
	}
#endif

	result = RES_OK;
disk_read_end:
	nile_spi_wait_busy();
	tfc_cs_high();
	return result;
}

#if FF_FS_READONLY == 0

DRESULT disk_write (BYTE pdrv, const BYTE __far* buff, LBA_t sector, UINT count) {
	uint8_t result = RES_ERROR;
	uint8_t resp[16];

	if (nile_ipl_data->card_state == NILE_CARD_BYTE_ADDRESSING)
		sector <<= 9;

	tfc_cs_low();

#ifdef USE_MULTI_TRANSFER_WRITES
	bool multi_transfer = count > 1;
	if (!tfc_send_cmd(multi_transfer ? TFC_WRITE_MULTIPLE_BLOCK : TFC_WRITE_BLOCK, 0x95, sector))
		goto disk_read_end;
	if (!nile_spi_rx_copy(resp, 1, NILE_SPI_MODE_WAIT_READ))
		goto disk_read_end;
	if (resp[0])
		goto disk_read_end;

	while (count) {
		resp[0] = 0xFF;
		resp[1] = multi_transfer ? 0xFC : 0xFE;
		if (!nile_spi_tx(resp, 2))
			goto disk_read_stop;
		if (!nile_spi_tx(buff, 512))
			goto disk_read_stop;
		if (!nile_spi_tx(resp, 2))
			goto disk_read_stop;
		if (!nile_spi_rx_copy(resp, 1, NILE_SPI_MODE_WAIT_READ))
			goto disk_read_stop;
		// TODO: error handling?
		buff += 512;
		count--;
	}
disk_read_stop:

	if (multi_transfer) {
		resp[0] = 0xFF;
		resp[1] = 0xFD;
		resp[2] = 0xFF;
		nile_spi_tx(resp, 3);
		if (tfc_wait_until_ready(0x00))
			goto disk_read_end;
	}
#else
	while (count) {
		if (!tfc_send_cmd(TFC_WRITE_BLOCK, 0x95, sector))
			goto disk_read_end;
		if (!nile_spi_rx_copy(resp, 1, NILE_SPI_MODE_WAIT_READ))
			goto disk_read_end;
		if (resp[0])
			goto disk_read_end;

		resp[0] = 0xFF;
		resp[1] = 0xFE;
		if (!nile_spi_tx(resp, 2))
			goto disk_read_end;
		if (!nile_spi_tx(buff, 512))
			goto disk_read_end;
		if (!nile_spi_tx(resp, 2))
			goto disk_read_end;
		if (!nile_spi_rx_copy(resp, 1, NILE_SPI_MODE_WAIT_READ))
			goto disk_read_end;
		// TODO: error handling?
		buff += 512;
		sector += 512;
		count--;
	}
#endif

	result = RES_OK;
disk_read_end:
	nile_spi_wait_busy();
	tfc_cs_high();
	return result;
}

#endif

DRESULT disk_ioctl (BYTE pdrv, BYTE cmd, void *buff) {
	if (cmd == CTRL_SYNC)
		return RES_OK;
	return RES_PARERR;
}

#endif
