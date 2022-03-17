/***************************************************************************
 *   Copyright (C) 2008 by Kevin McGuire                                   *
 *   Copyright (C) 2008 by Marcel Wijlaars                                 *
 *   Copyright (C) 2009 by Michael Ashton                                  *
 *   Copyright (C) 2022 by Jeroen Domburg                                  *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/binarybuffer.h>
#include <helper/time_support.h>
#include <target/algorithm.h>
#include <target/arm.h>

static int hc32l110_check_flash_completion(struct target *target, unsigned int timeout_ms);

#define HC32L110_FLASH_CR			0x40020020 //Flash control register
#define HC32L110_FLASH_CR_BUSY		(1<<4)     //If this bit is 1, the flash is busy
#define HC32L110_FLASH_BYPASS		0x4002002C //Bypass register, write magic here to enable other regs.
#define HC32L110_FLASH_SLOCK		0x40020030 //Sector erase lock, 0=locked
#define HC32L110_FLASH_SIZE			0x00100C70 //Indicates how much flash the chip has.

//Flash erase size is 512 bytes.
#define FLASH_SECTOR_SIZE 512
//A bit in SPROT covers 4Kbytes.
#define SPROT_SEC_SIZE 4096

//CR register  op definitions
#define FLASH_OP_PROGRAM 1
#define FLASH_OP_ERASE_SECTOR 2
#define FLASH_OP_ERASE_CHIP 3


/* flash bank hc32l110 0 0 0 0 <target#>
 * The hc32l110 devices all have the same flash layout, but varying amounts of it. */
FLASH_BANK_COMMAND_HANDLER(hc32l110_flash_bank_command)
{
	bank->base = 0x0000;
	bank->size = 0x8000; //assume the max of 32K for now
	return ERROR_OK;
}

/* This writes the magic words to the bypass register to enable writing to the SLOCK/CR register. */
static void hc32l110_bypass(struct target *target) 
{
	target_write_u32(target, HC32L110_FLASH_BYPASS, 0x5a5a);
	target_write_u32(target, HC32L110_FLASH_BYPASS, 0xa5a5);
}

/* Unlock a certain region for erasing/writing */
static void hc32l110_sunlock(struct target *target, int start_adr, int end_adr)
{
	int start_sec=start_adr/SPROT_SEC_SIZE;
	int end_sec=(end_adr+SPROT_SEC_SIZE-1)/SPROT_SEC_SIZE;
	int b=0;
	for (int i=start_sec; i<end_sec; i++) {
		b|=(1<<i);
	}
	hc32l110_bypass(target);
	target_write_u32(target, HC32L110_FLASH_SLOCK, b);
}

/* Lock all regions to protect against programming/reading. Note that the locks set here
 * are volatile. */
static void hc32l110_slock_all(struct target *target)
{
	hc32l110_bypass(target);
	target_write_u32(target, HC32L110_FLASH_SLOCK, 0);
}


static int hc32l110_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	unsigned int x;
	struct target *target = bank->target;

	/* mass erase */
	if (((first | last) == 0) || ((first == 0) && (last >= bank->num_sectors))) {
		LOG_DEBUG("performing mass erase.");
		hc32l110_bypass(target);
		target_write_u32(target, HC32L110_FLASH_CR, FLASH_OP_ERASE_CHIP);
		hc32l110_sunlock(target, 0, 32*1024);
		target_write_u32(target, 0, 0); //trigger erase

		if (hc32l110_check_flash_completion(target, 3500) != ERROR_OK) {
			LOG_ERROR("mass erase failed");
			return ERROR_FLASH_OPERATION_FAILED;
		}
		hc32l110_slock_all(target);

		LOG_DEBUG("mass erase successful.");
		return ERROR_OK;
	} else {
		unsigned long adr;

		for (x = first; x < last; x++) {
			adr = bank->base + (x * FLASH_SECTOR_SIZE);

			hc32l110_bypass(target);
			target_write_u32(target, HC32L110_FLASH_CR, FLASH_OP_ERASE_SECTOR);
			hc32l110_bypass(target);
			hc32l110_sunlock(target, adr, adr+FLASH_SECTOR_SIZE);
			target_write_u32(target, HC32L110_FLASH_SLOCK, 1<<(x/4));
			target_write_u32(target, adr, 0); //trigger erase

			if (hc32l110_check_flash_completion(target, 50) != ERROR_OK) {
				LOG_ERROR("failed to erase sector at address 0x%08lX", adr);
				return ERROR_FLASH_SECTOR_NOT_ERASED;
			}

			LOG_DEBUG("erased sector at address 0x%08lX", adr);
		}
		hc32l110_slock_all(target);
	}
	return ERROR_OK;
}

/* All-JTAG, single-access method. Could be accelerated by having a method that does the
 * programming on the ARM itself. */
static int hc32l110_write_single(struct flash_bank *bank,
	const uint8_t *buffer,
	uint32_t offset,
	uint32_t count)
{
	struct target *target = bank->target;

	hc32l110_bypass(target);
	target_write_u32(target, HC32L110_FLASH_CR, FLASH_OP_PROGRAM);
	hc32l110_sunlock(target, offset&~3, (offset+count+3)&~3);
	
	//If we start at an address that is not aligned to 4, we need to
	//also write the bytes before it to 0xff; we need to start earlier.
	uint32_t neg_start = (offset % 4);
	
	for (int x = -neg_start; x < (int32_t)count; x += 4) {
		uint32_t v = 0;
		//gather a word worth of data
		for (int b=0; b<4; b++) {
			v>>=8;
			if ( (x+b) < (int32_t)count && (x+b) >= 0) {
				v |= buffer[x+b]<<24;
			} else {
				v |= 0xFF000000;
			}
		}
		target_write_u32(target, offset+x, v); //program

		if (hc32l110_check_flash_completion(target, 1) != ERROR_OK) {
			LOG_ERROR("single write failed for address 0x%08lX", (unsigned long)(offset + x));
			return ERROR_FLASH_OPERATION_FAILED;
		}
	}
	hc32l110_slock_all(target);
	LOG_DEBUG("wrote %d bytes at address 0x%08lX", (int)count, (unsigned long)(offset));

	return ERROR_OK;
}

static int hc32l110_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	int retval;

	retval = hc32l110_write_single(bank, buffer, offset, count);
	if (retval != ERROR_OK) {
		LOG_ERROR("write failed");
		return ERROR_FLASH_OPERATION_FAILED;
	}
	return retval;
}

static int hc32l110_probe(struct flash_bank *bank)
{
	struct target *target = bank->target;
	uint32_t flash_size=0;
	target_read_u32(target, HC32L110_FLASH_SIZE, &flash_size); 
	if (flash_size>32768 || flash_size<4096) return ERROR_FLASH_OPERATION_FAILED;
	LOG_INFO("%dKiB of flash detected.", flash_size/1024);
	bank->size = flash_size;

	uint32_t offset = 0;
	bank->num_sectors = bank->size / FLASH_SECTOR_SIZE;
	bank->sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);
	for (unsigned int i = 0; i < bank->num_sectors; ++i) {
		bank->sectors[i].offset = offset;
		bank->sectors[i].size = FLASH_SECTOR_SIZE;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = 0;
		offset += bank->sectors[i].size;
	}
	return ERROR_OK;
}

/* wait up to timeout_ms for controller to not be busy,
 * then check whether the command passed or failed.
 *
 * this function sleeps 1ms between checks (after the first one),
 * so in some cases may slow things down without a usleep after the first read */
static int hc32l110_check_flash_completion(struct target *target, unsigned int timeout_ms)
{
	uint32_t v = HC32L110_FLASH_CR_BUSY;

	int64_t endtime = timeval_ms() + timeout_ms;
	while (1) {
		target_read_u32(target, HC32L110_FLASH_CR, &v);
		if ((v & HC32L110_FLASH_CR_BUSY) == 0) break;
		alive_sleep(1);
		if (timeval_ms() >= endtime) break;
	}

	return ERROR_OK;
}

const struct flash_driver hc32l110_flash = {
	.name = "hc32l110",
	.flash_bank_command = hc32l110_flash_bank_command,
	.erase = hc32l110_erase,
	.write = hc32l110_write,
	.read = default_flash_read,
	.probe = hc32l110_probe,
	.auto_probe = hc32l110_probe,
	.erase_check = default_flash_blank_check,
	.free_driver_priv = default_flash_free_driver_priv,
};
