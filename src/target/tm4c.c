/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2017  newbrain.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* This file implements tm4c target specific functions for detecting
 * the device, providing the XML memory map and Flash memory programming.
 *
 * Refereces:
 * TI doc - SLAU356G
 *   MSP423P4xx Technical Reference Manual
 * TI doc - SLAS826G
 *   tm4cP401R, tm4cP401M SimpleLink Mixed-Signal Microcontrollers
 * TI doc - SLAA704
 *   Flash Operations on tm4c MCUs
 * TI doc -
 *   tm4c® Peripheral Driver Library User's Guide
 *
 */

#include "general.h"
#include "target.h"
#include "target_internal.h"
#include "cortexm.h"

/* TLV: Device info tag, address and expected value */

#define TM4C_IDCODE_ADDR  0x400FE004

#define TM4C_BASE 0x400FD000

#define  TM4C_FLASH_FMA 	(TM4C_BASE+0x00)
#define  TM4C_FLASH_FMD 	(TM4C_BASE+0x04)
#define  TM4C_FLASH_FMC 	(TM4C_BASE+0x08)

#define  TM4C_FLASH_FMPRE0 	(TM4C_BASE+0x200)
#define  TM4C_FLASH_FMPRE1 	(TM4C_BASE+0x204)
#define  TM4C_FLASH_FMPRE2 	(TM4C_BASE+0x208)
#define  TM4C_FLASH_FMPRE3 	(TM4C_BASE+0x20C)
#define  TM4C_FLASH_FMPRE4 	(TM4C_BASE+0x210)
#define  TM4C_FLASH_FMPRE5 	(TM4C_BASE+0x214)
#define  TM4C_FLASH_FMPRE6 	(TM4C_BASE+0x218)
#define  TM4C_FLASH_FMPRE7 	(TM4C_BASE+0x21C)

#define  TM4C_FLASH_FMPPE0 	(TM4C_BASE+0x400)
#define  TM4C_FLASH_FMPPE1 	(TM4C_BASE+0x404)
#define  TM4C_FLASH_FMPPE2 	(TM4C_BASE+0x408)
#define  TM4C_FLASH_FMPPE3 	(TM4C_BASE+0x40C)
#define  TM4C_FLASH_FMPPE4 	(TM4C_BASE+0x410)
#define  TM4C_FLASH_FMPPE5 	(TM4C_BASE+0x414)
#define  TM4C_FLASH_FMPPE6 	(TM4C_BASE+0x418)
#define  TM4C_FLASH_FMPPE7 	(TM4C_BASE+0x41C)

#define FLASH_FMC_WRITE		(1<<0)
#define FLASH_FMC_ERASE		(1<<1)
#define FLASH_FMC_MERASE	(1<<2)
#define FLASH_FMC_COMT		(1<<3)
#define FLASH_FMC_WRKEY		0xA4420000

#define DEVINFO_TAG_ADDR 0x00201004u
#define DEVINFO_TAG_VALUE 0x0000000Bu

/* TLV: Device info length, address and expected value */
#define DEVINFO_LEN_ADDR 0x00201008u
#define DEVINFO_LEN_VALUE 0x00000004u

/* TLV: Device ID, address and expected values */
#define DEVID_ADDR 0x0020100Cu
#define DEVID_tm4cP401RIPZ 0x0000A000u
#define DEVID_tm4cP401MIPZ 0x0000A001u
#define DEVID_tm4cP401RIZXH 0x0000A002u
#define DEVID_tm4cP401MIZXH 0x0000A003u
#define DEVID_tm4cP401RIRGC 0x0000A004u
#define DEVID_tm4cP401MIRGC 0x0000A005u

/* TLV: Hardware revision, address and minimum expected value */
#define HWREV_ADDR 0x00201010u
#define HWREV_MIN_VALUE 0x00000043u

/* ROM Device Driver Table pointer addresses */
#define ROM_APITABLE 0x02000800u

#define OFS_FLASHCTLTABLE 28             /* ROM_APITABLE[7] */
#define OFS_FlashCtl_performMassErase 32 /* ROM_FLASHCTLTABLE[8] */
#define OFS_FlashCtl_eraseSector 36      /* ROM_FLASHCTLTABLE[9] */
#define OFS_FlashCtl_programMemory 40    /* ROM_FLASHCTLTABLE[10] */

/* Memory sizes and base addresses */
#define MAIN_FLASH_BASE 0x00000000u /* Beginning of Main Flash */
#define INFO_FLASH_BASE 0x00200000u /* Beginning of Info Flash */
#define INFO_BANK_SIZE 0x00002000u  /* Size of 1 bank of Info Flash */
#define SECTOR_SIZE 0x2000u         /* Size of erase page: 4KB */

/* Flash protection registers */
#define INFO_BANK0_WEPROT 0x400110B0u /* Write/Erase protection Bank 0 Info */
#define MAIN_BANK0_WEPROT 0x400110B4u /* Write/Erase protection Bank 0 Main */
#define INFO_BANK1_WEPROT 0x400110C0u /* Write/Erase protection Bank 1 Info */
#define MAIN_BANK1_WEPROT 0x400110C4u /* Write/Erase protection Bank 1 Main */

/* Main Flash and SRAM size registers */
#define SYS_SRAM_SIZE 0x400FD000  /* Size of SRAM in SYSCTL */
#define SYS_FLASH_SIZE 0xE0043020u /* Size of main flash in SYSCTL */

/* RAM info */
#define SRAM_BASE 0x20000000u       /* Beginning of SRAM */
#define SRAM_BASE_SIZE 0x20000000u       /* Beginning of SRAM */
#define SRAM_CODE_BASE 0x01000000u  /* Beginning of SRAM, Code zone alias */
#define P401M_SRAM_SIZE 0x00008000u /* Size of SRAM, M: 32KB */
#define P401R_SRAM_SIZE 0x00010000u /* Size of SRAM, R: 64KB */

/* Flash write buffer and stack */
#define SRAM_STACK_OFFSET 0x00000200u /* A bit less than 512 stack room */
#define SRAM_STACK_PTR (SRAM_BASE + SRAM_STACK_OFFSET)
#define SRAM_WRITE_BUFFER SRAM_STACK_PTR /* Buffer right above stack */
#define SRAM_WRITE_BUF_SIZE 0x00000400u  /* Write 1024 bytes at a tima */

/* Watchdog */
#define WDT_A_WTDCTL 0x4000480Cu /* Control register for watchdog */
#define WDT_A_HOLD 0x5A88u       /* Clears and halts the watchdog */

/* Support variables to call code in ROM */
struct tm4c_flash {
	struct target_flash f;
	target_addr flash_protect_register; /* Address of the WEPROT register*/
	target_addr FlashCtl_eraseSector; /* Erase flash sector routine in ROM*/
	target_addr FlashCtl_programMemory; /* Flash programming routine in ROM */
};

/* Flash operations */
static bool tm4c_sector_erase(struct target_flash *f, target_addr addr);
static int tm4c_flash_erase(struct target_flash *f, target_addr addr,
		size_t len);
static int tm4c_flash_write(struct target_flash *f, target_addr dest,
		const void *src, size_t len);


//static void tm4c_call_ROM(target *t, uint32_t address, uint32_t regs[]);
enum IDS_TM4C {

	ID_TM4C1294KCPDT = 0x34
};
/* Utility functions */
/* Find the the target flash that conatins a specific address */
static struct target_flash* get_target_flash(target *t, target_addr addr);

/* Call a subroutine in the tm4c ROM (or anywhere else...)*/
//static void tm4c_call_ROM(target *t, uint32_t address, uint32_t regs[]);

/* Protect or unprotect the sector containing address */
static inline uint32_t tm4c_sector_unprotect(struct tm4c_flash *mf,
		target_addr addr) {
	/* Read the old protection register */
	uint32_t old_mask = target_mem_read32(mf->f.t, mf->flash_protect_register);
	/* Find the bit representing the sector and set it to 0  */
	uint32_t sec_mask = ~(1u << ((addr - mf->f.start) / SECTOR_SIZE));
	/* Clear the potection bit */
	sec_mask &= old_mask;
	target_mem_write32(mf->f.t, mf->flash_protect_register, sec_mask);
	return old_mask;
}

/* Optional commands handlers */
/* Erase all of main flash */
static bool tm4c_cmd_erase_main(target *t, int argc, const char **argv);
/* Erase a single (4KB) sector */
static bool tm4c_cmd_sector_erase(target *t, int argc, const char **argv);

static bool tm4c_cmd_mass_erase(target *t, int argc, const char **argv);
/* Optional commands structure*/
const struct command_s tm4c_cmd_list[] = { { "erase_mass",
		(cmd_handler) tm4c_cmd_mass_erase, "Mass erase flash" }, { "erase",
		(cmd_handler) tm4c_cmd_erase_main, "Erase main flash" }, {
		"sector_erase", (cmd_handler) tm4c_cmd_sector_erase,
		"Erase sector containing given address" }, { NULL, NULL, NULL } };

static void tm4c_add_flash(target *t, uint32_t addr, size_t length,
		target_addr prot_reg) {
	struct tm4c_flash *mf = calloc(1, sizeof(*mf));
	struct target_flash *f;
	if (!mf) { /* calloc failed: heap exhaustion */
		DEBUG_WARN("calloc: failed in %s\n", __func__);
		return;
	}

	f = &mf->f;
	f->start = addr;
	f->length = length;
	f->blocksize = SECTOR_SIZE;
	f->erase = tm4c_flash_erase;
	f->write = tm4c_flash_write;
	f->buf_size = SRAM_WRITE_BUF_SIZE;
	f->erased = 0xff;
	target_add_flash(t, f);
	/* Initialize ROM call pointers. Silicon rev B is not supported */
	uint32_t flashctltable = target_mem_read32(t,
	ROM_APITABLE + OFS_FLASHCTLTABLE);
	mf->FlashCtl_eraseSector = target_mem_read32(t,
			flashctltable + OFS_FlashCtl_eraseSector);
	mf->FlashCtl_programMemory = target_mem_read32(t,
			flashctltable + OFS_FlashCtl_programMemory);
	mf->flash_protect_register = prot_reg;
}

bool tm4c_probe(target *t) {
//	/* Check for the right device info tag in the TLV ROM structure */
//	if (target_mem_read32(t, DEVINFO_TAG_ADDR) != DEVINFO_TAG_VALUE)
//		return false;
//
//	/* Check for the right device info length tag in the TLV ROM structure */
//	if (target_mem_read32(t, DEVINFO_LEN_ADDR) != DEVINFO_LEN_VALUE)
//		return false;
//
//	/* Check for the right HW revision: at least C, as no flash support for B */
//	if (target_mem_read32(t, HWREV_ADDR) < HWREV_MIN_VALUE) {
//		DEBUG_INFO("tm4c Version not handled\n");
//		return false;
//	}
	uint32_t id_code = (target_mem_read32(t, TM4C_IDCODE_ADDR) & 0x00ff0000)
			>> 16;

	/* If we got till this point, we are most probably looking at a real TLV  */
	/* Device Information structure. Now check for the correct device         */
	switch (id_code) {
	case ID_TM4C1294KCPDT:
		t->driver = "TM4C1294KCPDT 512-kb Flash, 256-kb RAM";
		break;
	default:
		/* Unknown device, not an tm4c or not a real TLV */
		return false;
	}
	/* SRAM region, SRAM zone */
	target_add_ram(t, SRAM_BASE, 0x40000);
	/* Flash bank size */
	uint32_t banksize = 0x20000;
	/* Main Flash Bank 0 */
	tm4c_add_flash(t, MAIN_FLASH_BASE, banksize, MAIN_BANK0_WEPROT);
	/* Main Flash Bank 1 */
	tm4c_add_flash(t, MAIN_FLASH_BASE + banksize, banksize, MAIN_BANK1_WEPROT);
	/* Info Flash Bank 0 */
	tm4c_add_flash(t, INFO_FLASH_BASE, INFO_BANK_SIZE, INFO_BANK0_WEPROT);
	/* Info Flash Bank 1 */
	tm4c_add_flash(t, INFO_FLASH_BASE + INFO_BANK_SIZE, INFO_BANK_SIZE,
	INFO_BANK1_WEPROT);

	/* Connect the optional commands */
	target_add_commands(t, tm4c_cmd_list, "tm4c1294");

	/* All done */
	return true;
}

/* Flash operations */
/* Erase a single sector at addr calling the ROM routine*/
static bool tm4c_sector_erase(struct target_flash *f, target_addr addr) {
	target *t = f->t;

	tc_printf(t, "Erasing flash sector... This may take a few seconds.  ");

	target_mem_write32(t, TM4C_FLASH_FMA, addr);
	target_mem_write32(t, TM4C_FLASH_FMC, (FLASH_FMC_WRKEY | FLASH_FMC_ERASE));

	while (target_mem_read32(t, TM4C_FLASH_FMC) & FLASH_FMC_ERASE) {
		if (target_check_error(t)) {
			tc_printf(t, "Err\n");
			return false;
		}
	}
	return 0;
}

/* Erase from addr for len bytes */
static int tm4c_flash_erase(struct target_flash *f, target_addr addr,
		size_t len) {
	int ret = 0;
	uint32_t sector = addr;

	while (len) {
		ret |= tm4c_sector_erase(f, sector);

		/* update len and addr */
		len -= f->blocksize;
		if (len > f->blocksize)
			len -= f->blocksize;
		else
			len = 0;
		sector += SECTOR_SIZE;
	}

	return 0;
}

/* Program flash */
static int tm4c_flash_write(struct target_flash *f, target_addr dest,
		const void *src, size_t len) {

//	struct msp432_flash *mf = (struct msp432_flash *)f;
	target *t = f->t;

	/* Prepare RAM buffer in target */
	target_mem_write(t, SRAM_WRITE_BUFFER, src, len);

	/* Unprotect sector, len is always < SECTOR_SIZE */
//	uint32_t old_prot = msp432_sector_unprotect(mf, dest);

	DEBUG_WARN("Flash protect: 0x%08"PRIX32"\n",
			   target_mem_read32(t, mf->flash_protect_register));

	/* Prepare input data */
	uint32_t regs[t->regs_size / sizeof(uint32_t)]; // Use of VLA
	target_regs_read(t, regs);
	regs[0] = SRAM_WRITE_BUFFER; // Address of buffer to be flashed in R0
	regs[1] = dest;              // Flash address to be write to in R1
	regs[2] = len;               // Size of buffer to be flashed in R2

	DEBUG_INFO("Writing 0x%04" PRIX32 " bytes at 0x%08zu\n", dest, len);
	/* Call ROM */
//	tm4c_call_ROM(t, mf->FlashCtl_programMemory, regs);

	/* Restore original protection */
//	target_mem_write32(t, mf->flash_protect_register, old_prot);

	DEBUG_INFO("ROM return value: %"PRIu32"\n", regs[0]);
	// Result value in R0 is true for success
	return !regs[0];
}

/* Optional commands handlers */
static bool tm4c_cmd_erase_main(target *t, int argc, const char **argv) {
	(void) argc;
	(void) argv;
	/* The mass erase routine in ROM will also erase the Info Flash. */
	/* Usually, this is not wanted, so go sector by sector...        */

	uint32_t banksize = 0x40000;
	DEBUG_INFO("Bank Size: 0x%08"PRIX32"\n", banksize);

	/* Erase first bank */
	struct target_flash *f = get_target_flash(t, MAIN_FLASH_BASE);
	bool ret = tm4c_flash_erase(f, MAIN_FLASH_BASE, banksize);

	/* Erase second bank */
	f = get_target_flash(t, MAIN_FLASH_BASE + banksize);
	ret |= tm4c_flash_erase(f, MAIN_FLASH_BASE + banksize, banksize);

	return 0;
}

static bool tm4c_cmd_sector_erase(target *t, int argc, const char **argv) {
	if (argc < 2)
		tc_printf(t, "usage: monitor sector_erase <addr>\n");

	uint32_t addr = strtoul(argv[1], NULL, 0);

	/* Find the flash structure (for the rigth protect register) */
	struct target_flash *f = get_target_flash(t, addr);

	if (f)
		return tm4c_sector_erase(f, addr);
	tc_printf(t, "Invalid sector address\n");
	return false;
}

/* Returns flash bank containing addr, or NULL if not found */
static struct target_flash* get_target_flash(target *t, target_addr addr) {
	struct target_flash *f = t->flash;
	while (f) {
		if ((f->start <= addr) && (addr < f->start + f->length))
			break;
		f = f->next;
	}
	return f;
}

/* tm4c ROM routine invocation */
//static void tm4c_call_ROM(target *t, uint32_t address, uint32_t regs[]) {
//	/* Kill watchdog */
//	target_mem_write16(t, WDT_A_WTDCTL, WDT_A_HOLD);
//
//	/* Breakpoint at the beginning of CODE SRAM alias area */
//	target_mem_write16(t, SRAM_CODE_BASE, ARM_THUMB_BREAKPOINT);
//
//	/* Prepare registers */
//	regs[REG_MSP] = SRAM_STACK_PTR; /* Stack space */
//	regs[REG_LR] = SRAM_CODE_BASE | 1; /* Return to beginning of SRAM CODE alias */
//	regs[REG_PC] = address; /* Start at given address */
//	target_regs_write(t, regs);
//
//	/* Call ROM */
//	/* start the target and wait for it to halt again */
//	target_halt_resume(t, false);
//	while (!target_halt_poll(t, NULL))
//		;
//
//	// Read registers to get result
//	target_regs_read(t, regs);
//}

//static void tm4c_flash_unlock(target *t) {
//
//	target_mem_write32(t, TM4C_FLASH_FMPPE0, 0xFFFFFFFF);
//	target_mem_write32(t, TM4C_FLASH_FMPPE1, 0xFFFFFFFF);
//	target_mem_write32(t, TM4C_FLASH_FMPPE2, 0xFFFFFFFF);
//	target_mem_write32(t, TM4C_FLASH_FMPPE3, 0xFFFFFFFF);
//	target_mem_write32(t, TM4C_FLASH_FMPPE4, 0xFFFFFFFF);
//	target_mem_write32(t, TM4C_FLASH_FMPPE5, 0xFFFFFFFF);
//	target_mem_write32(t, TM4C_FLASH_FMPPE6, 0xFFFFFFFF);
//	target_mem_write32(t, TM4C_FLASH_FMPPE7, 0xFFFFFFFF);
//
//	target_mem_write32(t, TM4C_FLASH_FMPRE0, 0xFFFFFFFF);
//	target_mem_write32(t, TM4C_FLASH_FMPRE1, 0xFFFFFFFF);
//	target_mem_write32(t, TM4C_FLASH_FMPRE2, 0xFFFFFFFF);
//	target_mem_write32(t, TM4C_FLASH_FMPRE3, 0xFFFFFFFF);
//	target_mem_write32(t, TM4C_FLASH_FMPRE4, 0xFFFFFFFF);
//	target_mem_write32(t, TM4C_FLASH_FMPRE5, 0xFFFFFFFF);
//	target_mem_write32(t, TM4C_FLASH_FMPRE6, 0xFFFFFFFF);
//	target_mem_write32(t, TM4C_FLASH_FMPRE7, 0xFFFFFFFF);
//
//}
static bool tm4c_cmd_mass_erase(target *t, int argc, const char **argv) {
	(void) argc;
	(void) argv;
	const char spinner[] = "|/-\\";
	int spinindex = 0;

	tc_printf(t, "Erasing flash... This may take a few seconds.  ");
//	tm4c_flash_unlock(t);
//	uint32_t FMP[16];
//	FMP[0] = target_mem_read32(t, TM4C_FLASH_FMPPE0);
//	FMP[1] = target_mem_read32(t, TM4C_FLASH_FMPPE1);
//	FMP[2] = target_mem_read32(t, TM4C_FLASH_FMPPE2);
//	FMP[3] = target_mem_read32(t, TM4C_FLASH_FMPPE3);
//	FMP[4] = target_mem_read32(t, TM4C_FLASH_FMPPE4);
//	FMP[5] = target_mem_read32(t, TM4C_FLASH_FMPPE5);
//	FMP[6] = target_mem_read32(t, TM4C_FLASH_FMPPE6);
//	FMP[7] = target_mem_read32(t, TM4C_FLASH_FMPPE7);
//	FMP[8] = target_mem_read32(t, TM4C_FLASH_FMPRE0);
//	FMP[9] = target_mem_read32(t, TM4C_FLASH_FMPRE1);
//	FMP[10] = target_mem_read32(t, TM4C_FLASH_FMPRE2);
//	FMP[11] = target_mem_read32(t, TM4C_FLASH_FMPRE3);
//	FMP[12] = target_mem_read32(t, TM4C_FLASH_FMPRE4);
//	FMP[13] = target_mem_read32(t, TM4C_FLASH_FMPRE5);
//	FMP[14] = target_mem_read32(t, TM4C_FLASH_FMPRE6);
//	FMP[15] = target_mem_read32(t, TM4C_FLASH_FMPRE7);
//
//	int i;
//	for (i = 0; i < 16; i++) {
//		if (FMP[i] == 0) {
//			tc_printf(t, "Err\n");
//		} else {
//			tc_printf(t, "Err00\n");
//		}
//	}
	/* Flash mass erase start instruction */
	target_mem_write32(t, TM4C_FLASH_FMC, (FLASH_FMC_WRKEY | FLASH_FMC_MERASE));

	while (target_mem_read32(t, TM4C_FLASH_FMC) & FLASH_FMC_MERASE) {
		tc_printf(t, "\b%c", spinner[spinindex++ % 4]);
		if (target_check_error(t)) {
			tc_printf(t, "Err\n");
			return false;
		}
	}
	tc_printf(t, "\n");
	tc_printf(t, "done");
	tc_printf(t, "\n");
	/* Check for error */

	return 1;
}

