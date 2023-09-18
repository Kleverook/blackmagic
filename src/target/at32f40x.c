/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
 * Copyright (C) 2017, 2018  Uwe Bonnes
 *                           <bon@elektron.ikp.physik.tu-darmstadt.de>
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

/* This file implements at32f40x target specific functions for detecting
 * the device, providing the XML memory map and Flash memory programming.
 *
 * Refereces:
 * ST doc - RM0090
 *   Reference manual - at32f40x05xx, at32f40x07xx, at32f40x3x15xx and at32f40x3x17xx
 *   advanced ARM-based 32-bit MCUs
 * ST doc - PM0081
 *   Programming manual - at32f40x3x0xxx and at32f40x1xxx Flash programming
 *    manual
 */

#include "general.h"
#include "target.h"
#include "target_internal.h"
#include "cortexm.h"
#include "gdb_packet.h"

//static bool at32f40x_cmd_option(target *t, int argc, char **argv);
static bool at32f40x_cmd_psize(target *t, int argc, char **argv);
static bool at32f40x_cmd_erase_mass(target *t, int argc, const char **argv);

const struct command_s at32f40x_cmd_list[] = {
		{"erase_mass", (cmd_handler)at32f40x_cmd_erase_mass,
			 "Erase entire flash memory"},
		 { "psize",
		(cmd_handler) at32f40x_cmd_psize,
		"Configure flash write parallelism: (x8|x16|x32(default)|x64)" }, {
NULL, NULL, NULL } };


static bool at32f40x_attach(target *t);
static int at32f40x_flash_erase(struct target_flash *f, target_addr addr,
		size_t len);
static int at32f40x_flash_write(struct target_flash *f, target_addr dest,
		const void *src, size_t len);


/* Flash Program and Erase Controller Register Map */
#define FPEC_BASE	0x40022000

#define FLASH_ACR	(FPEC_BASE+0x00)
#define FLASH_UNLOCK	(FPEC_BASE+0x04)
#define FLASH_OPTKEYR	(FPEC_BASE+0x08)
#define FLASH_STS	(FPEC_BASE+0x0C)
#define FLASH_CTRL	(FPEC_BASE+0x10)
#define FLASH_ADDR	(FPEC_BASE+0x14)

#define FLASH_CTRL_FPRGM		(1 << 0)
#define FLASH_CTRL_BANKERS		(1 << 2)
#define FLASH_CTRL_BLKERS		(1 << 3)
#define FLASH_CTRL_SECERS		(1 << 1)
#define FLASH_CTRL_ERSTR		(1 << 6)
#define FLASH_CTRL_ODFIE		(1 << 12)
#define FLASH_CTRL_ERRIE		(1 << 10)
#define FLASH_CTRL_OPLK		(1 << 7)

#define FLASH_STS_OBF		(1 << 0)

#define FLASH_OPTCR_OPTLOCK	(1 << 0)
#define FLASH_OPTCR_OPTSTRT	(1 << 1)
#define FLASH_OPTCR_WDG_SW	(1 << 5)
#define FLASH_OPTCR_nDBANK	(1 << 29)
#define FLASH_OPTCR_DB1M	(1 << 30)

#define FLASH_OPTCR_PROT_MASK	0xff00
#define FLASH_OPTCR_PROT_L0  	0xaa00
#define FLASH_OPTCR_PROT_L1  	0xbb00

#define KEY1 0x45670123
#define KEY2 0xCDEF89AB

#define OPTKEY1 0x08192A3B
#define OPTKEY2 0x4C5D6E7F

#define SR_ERROR_MASK	0xF2
#define SR_EOP		0x01

#define F4_FLASHSIZE	0x1FFF7A22
#define F7_FLASHSIZE	0x1FF0F442
#define F72X_FLASHSIZE	0x1FF07A22
#define DBGMCU_IDCODE	0xE0042000
#define DBGMCU_CR		0xE0042004
#define DBG_SLEEP		(1 <<  0)

#define AXIM_BASE 0x8000000
#define ITCM_BASE 0x0200000

#define DBGMCU_CR_DBG_SLEEP		(0x1U << 0U)
#define DBGMCU_CR_DBG_STOP		(0x1U << 1U)
#define DBGMCU_CR_DBG_STANDBY	(0x1U << 2U)

struct at32f40x_flash {
	struct target_flash f;
	enum align psize;
	uint8_t base_sector;
	uint8_t bank_split;
};

struct at32f40x_priv_s {
	uint32_t dbgmcu_cr;
};

enum IDS_STM32F247 {

    ID_AT32F403 = 0x347, ID_AT32F403ARGT7 = 0x345
};

static void at32f40x_add_flash(target *t, uint32_t addr, size_t length,
		size_t blocksize, unsigned int base_sector, int split) {
	if (length == 0)
		return;
	struct at32f40x_flash *sf = calloc(1, sizeof(*sf));
	if (!sf) { /* calloc failed: heap exhaustion */
		DEBUG_WARN("calloc: failed in %s\n", __func__);
		return;
	}

	struct target_flash *f = &sf->f;
	f->start = addr;
	f->length = length;
	f->blocksize = blocksize;
	f->erase = at32f40x_flash_erase;
	f->write = at32f40x_flash_write;
	f->buf_size = 1024;
	f->erased = 0xff;
	sf->base_sector = base_sector;
	sf->bank_split = split;
	sf->psize = ALIGN_WORD;
	target_add_flash(t, f);
}

static char* at32f40x_get_chip_name(uint32_t idcode) {
	switch (idcode) {
        case ID_AT32F403: /* F40XxE/G */
            return "AT32F403";
        case ID_AT32F403ARGT7: /* F40XxE/G */
            return "AT32F403ARGT7";
        default:
            return NULL;
    }
}

static void at32f40x_detach(target *t) {
	struct at32f40x_priv_s *ps = (struct at32f40x_priv_s*) t->target_storage;

	/*reverse all changes to DBGMCU_CR*/
	target_mem_write32(t, DBGMCU_CR, ps->dbgmcu_cr);
	cortexm_detach(t);
}

bool at32f40x_probe(target *t) {
	t->idcode = target_mem_read32(t, DBGMCU_IDCODE) & 0xfff;

	switch (t->idcode) {

        case ID_AT32F403: /* F413     RM0430 Rev.2, 320 kB Ram, 1.5 MB flash. */
//		t->mass_erase = at32f40x_mass_erase;
            t->detach = at32f40x_detach;
            t->driver = at32f40x_get_chip_name(t->idcode);
            t->attach = at32f40x_attach;
            target_add_commands(t, at32f40x_cmd_list, t->driver);

            return true;

        case ID_AT32F403ARGT7: /* F413     RM0430 Rev.2, 320 kB Ram, 1.5 MB flash. */
//		t->mass_erase = at32f40x_mass_erase;
            t->detach = at32f40x_detach;
            t->driver = at32f40x_get_chip_name(t->idcode);
            t->attach = at32f40x_attach;
            target_add_commands(t, at32f40x_cmd_list, t->driver);

            return true;

        default:
            return false;
    }
}

static bool at32f40x_attach(target *t) {

	uint16_t max_flashsize;

	if (!cortexm_attach(t))
		return false;

	switch (t->idcode) {

        case ID_AT32F403: /* F74x RM0385 Rev.4 */
            max_flashsize = 1024;
            break;
        case ID_AT32F403ARGT7: /* F74x RM0385 Rev.4 */
            max_flashsize = 1024;
            break;

        default:
            return false;
    }

	/* Save DBGMCU_CR to restore it when detaching*/
	struct at32f40x_priv_s *priv_storage = calloc(1, sizeof(*priv_storage));
	if (!priv_storage) { /* calloc failed: heap exhaustion */
		DEBUG_WARN("calloc: failed in %s\n", __func__);
		return false;
	}
	priv_storage->dbgmcu_cr = target_mem_read32(t, DBGMCU_CR);
	t->target_storage = (void*) priv_storage;
	/* Enable debugging during all low power modes*/

	target_mem_write32(t, DBGMCU_CR, priv_storage->dbgmcu_cr |
	DBGMCU_CR_DBG_SLEEP | DBGMCU_CR_DBG_STANDBY | DBGMCU_CR_DBG_STOP);

	/* Free previously loaded memory map */

	target_mem_map_free(t);



	int split = 0;
	uint32_t banksize;

	banksize = max_flashsize << 10;



	switch (t->idcode) {

        case ID_AT32F403: /* F74x RM0385 Rev.4 */
            target_add_ram(t, 0x20000000, 0x80000); /* 512 k RAM */
            at32f40x_add_flash(t, 0x8000000, banksize, 0x800, 0, split);
            break;
        case ID_AT32F403ARGT7:
            target_add_ram(t, 0x20000000, 0x18000); /* 96 k RAM */
            at32f40x_add_flash(t, 0x8000000, banksize, 0x800, 0, split);

            break;

        default:
            break;
    }

	return (true);
}

static void at32f40x_flash_unlock(target *t) {

	target_mem_write32(t, FLASH_UNLOCK, KEY1);
	target_mem_write32(t, FLASH_UNLOCK, KEY2);

}

static int at32f40x_flash_erase(struct target_flash *f, target_addr addr,
		size_t len) {
	target *t = f->t;
//	struct at32f40x_flash *sf = (struct at32f40x_flash*) f;
//	uint32_t sr;
	/* No address translation is needed here, as we erase by sector number */
	uint32_t sector = addr;
	at32f40x_flash_unlock(t);

	while (len) {

		/* Flash page erase instruction */
		switch (t->idcode) {

            case ID_AT32F403: /* F74x RM0385 Rev.4 */
                target_mem_write32(t, FLASH_ADDR, sector);
                /* write address to FMA */
                target_mem_write32(t, FLASH_CTRL,
                                   (FLASH_CTRL_SECERS | FLASH_CTRL_ERSTR));
                break;
            case ID_AT32F403ARGT7: /* F74x RM0385 Rev.4 */
                target_mem_write32(t, FLASH_ADDR, sector);
                /* write address to FMA */
                target_mem_write32(t, FLASH_CTRL,
                                   (FLASH_CTRL_SECERS | FLASH_CTRL_ERSTR));
                break;

            default:
                break;
        }

		/* Read FLASH_STS to poll for BSY bit */
		while (target_mem_read32(t, FLASH_STS) & FLASH_STS_OBF)
			if (target_check_error(t)) {
				DEBUG_WARN("at32f40x flash erase: comm error\n");
				return -1;
			}
		if (len > f->blocksize)
			len -= f->blocksize;
		else
			len = 0;
		switch (t->idcode) {

            case ID_AT32F403: /* F74x RM0385 Rev.4 */
                sector += 0x800;
                break;
            case ID_AT32F403ARGT7: /* F74x RM0385 Rev.4 */
                sector += 0x800;
                break;

            default:
                break;
        }

	}

	/* Check for error */
//	uint32_t sr = target_mem_read32(t, FLASH_STS);
//	if (sr & SR_ERROR_MASK) {
////		DEBUG_WARN("at32f40x flash erase: sr error: 0x%" PRIx32 "\n", sr);
//		return -1;
//	}
	return 0;
}

static int at32f40x_flash_write(struct target_flash *f, target_addr dest,
		const void *src, size_t len) {
	/* Translate ITCM addresses to AXIM */
//	if ((dest >= ITCM_BASE) && (dest < AXIM_BASE)) {
//		dest = AXIM_BASE + (dest - ITCM_BASE);
//	}
	target *t = f->t;
	uint32_t sr;
	enum align psize = ((struct at32f40x_flash*) f)->psize;
	target_mem_write32(t, FLASH_CTRL, FLASH_CTRL_FPRGM);
	cortexm_mem_write_sized(t, dest, src, len, psize);
	/* Read FLASH_STS to poll for BSY bit */
	/* Wait for completion or an error */
	do {
		sr = target_mem_read32(t, FLASH_STS);
		if (target_check_error(t)) {
			DEBUG_WARN("at32f40x flash write: comm error\n");
			return -1;
		}
	} while (sr & FLASH_STS_OBF);

//	if (sr & SR_ERROR_MASK) {
//		DEBUG_WARN("at32f40x flash write error 0x%" PRIx32 "\n", sr);
//		return -1;
//	}
	return 0;
}

static bool at32f40x_cmd_erase_mass(target *t, int argc, const char **argv) {

	(void)argc;
	(void)argv;
	const char spinner[] = "|/-\\";
	int spinindex = 0;

	tc_printf(t, "Erasing flash... This may take a few seconds.  ");
	at32f40x_flash_unlock(t);

	/* Flash mass erase start instruction */
	target_mem_write32(t, FLASH_CTRL, (FLASH_CTRL_BANKERS | FLASH_CTRL_ERSTR));

	while (target_mem_read32(t, FLASH_STS) & FLASH_STS_OBF) {
		tc_printf(t, "\b%c", spinner[spinindex++ % 4]);
		if(target_check_error(t)) {
			tc_printf(t, "\n");
			return false;
		}
	}
	tc_printf(t, "\n");
	/* Check for error */

	const uint32_t result = target_mem_read32(t, FLASH_STS);
	return !(result & SR_ERROR_MASK);
}
/* Dev   | DOC  |Rev|ID |OPTCR    |OPTCR   |OPTCR1   |OPTCR1 | OPTCR2
 |hex|default  |reserved|default  |resvd  | default|resvd
 * F20x  |pm0059|5.1|411|0FFFAAED |F0000010|
 * F40x  |rm0090|11 |413|0FFFAAED |F0000010|
 * F42x  |rm0090|11 |419|0FFFAAED |30000000|0FFF0000 |F000FFFF
 * F446  |rm0390| 2 |421|0FFFAAED |7F000010|
 * F401BC|rm0368| 3 |423|0FFFAAED |7FC00010|
 * F411  |rm0383| 2 |431|0FFFAAED |7F000010|
 * F401DE|rm0368| 3 |433|0FFFAAED |7F000010|
 * F46x  |rm0386| 2 |434|0FFFAAED |30000000|0FFF0000 |F000FFFF
 * F412  |rm0402| 4 |441|0FFFAAED*|70000010|
 * F74x  |rm0385| 4 |449|C0FFAAFD |3F000000|00400080*|00000000
 * F76x  |rm0410| 2 |451|FFFFAAFD*|00000000|00400080*|00000000
 * F72x  |rm0431| 1 |452|C0FFAAFD |3F000000|00400080*|00000000|00000000|800000FF
 * F410  |rm0401| 2 |458|0FFFAAED*|7FE00010|
 * F413  |rm0430| 2 |463|7FFFAAED*|00000010|
 *
 * * Documentation for F7 with OPTCR1 default = 0fff7f0080 seems wrong!
 * * Documentation for F412 with OPTCR default = 0ffffffed seems wrong!
 * * Documentation for F413 with OPTCR default = 0ffffffed seems wrong!
 */

//static bool optcr_mask(target *t, uint32_t *val) {
//	switch (t->idcode) {
//	case ID_AT32F403:
//	case ID_AT32F437:
//		val[0] &= ~0x3F000000;
//		break;
//	default:
//		return false;
//	}
//	return true;
//}

//static bool at32f40x_option_write(target *t, uint32_t *val, int count) {
//	val[0] &= ~(FLASH_OPTCR_OPTSTRT | FLASH_OPTCR_OPTLOCK);
//	uint32_t optcr = target_mem_read32(t, FLASH_CTRL_ODFIE);
//	/* Check if watchdog and read protection is active.
//	 * When both are active, watchdog will trigger when erasing
//	 * to get back to level 0 protection and operation aborts!
//	 */
//	if (!(optcr & FLASH_OPTCR_WDG_SW)
//			&& ((optcr & FLASH_OPTCR_PROT_MASK) != FLASH_OPTCR_PROT_L0)
//			&& ((val[0] & FLASH_OPTCR_PROT_MASK) != FLASH_OPTCR_PROT_L1)) {
//		val[0] &= ~FLASH_OPTCR_PROT_MASK;
//		val[0] |= FLASH_OPTCR_PROT_L1;
//		tc_printf(t, "Keeping L1 protection while HW Watchdog fuse is set!\n");
//	}
//	target_mem_write32(t, FLASH_OPTKEYR, OPTKEY1);
//	target_mem_write32(t, FLASH_OPTKEYR, OPTKEY2);
//	while (target_mem_read32(t, FLASH_STS) & FLASH_STS_OBF)
//		if (target_check_error(t))
//			return -1;
//
//	/* WRITE option bytes instruction */
////	if (((t->idcode == ID_AT32F403) || (t->idcode == ID_AT32F437))
////			&& (count > 1))
//	/* Checkme: Do we need to read old value and then set it? */
////		target_mem_write32(t, FLASH_OPTCR + 4, val[1]);
////	if ((t->idcode == ID_AT32F403) && (count > 2))
////		target_mem_write32(t, FLASH_OPTCR + 8, val[2]);
////	target_mem_write32(t, FLASH_OPTCR, val[0]);
////	target_mem_write32(t, FLASH_OPTCR, val[0] | FLASH_OPTCR_OPTSTRT);
//	const char spinner[] = "|/-\\";
//	int spinindex = 0;
//	tc_printf(t, "Erasing flash... This may take a few seconds.  ");
//	/* Read FLASH_STS to poll for BSY bit */
//	while (target_mem_read32(t, FLASH_STS) & FLASH_STS_OBF) {
//		platform_delay(100);
//		tc_printf(t, "\b%c", spinner[spinindex++ % 4]);
//		if (target_check_error(t)) {
//			tc_printf(t, " failed\n");
//			return false;
//		}
//	}
//	tc_printf(t, "\n");
////	target_mem_write32(t, FLASH_OPTCR, FLASH_OPTCR_OPTLOCK);
//	/* Reset target to reload option bits.*/
//	target_reset(t);
//	return true;
//}

//static bool at32f40x_option_write_default(target *t) {
//	uint32_t val[3];
//	switch (t->idcode) {
////	case ID_AT32F42X:
////	case ID_AT32F46X:
////		val[0] = 0x0FFFAAED;
////		val[1] = 0x0FFF0000;
////		return at32f40x_option_write(t, val, 2);
////	case ID_STM32F72X:
////		val[0] = 0xC0FFAAFD;
////		val[1] = 0x00400080;
////		val[2] = 0;
////		return at32f40x_option_write(t, val, 3);
////	case ID_STM32F74X:
////		val[0] = 0xC0FFAAFD;
////		val[1] = 0x00400080;
////		return at32f40x_option_write(t, val, 2);
////	case ID_STM32F76X:
////		val[0] = 0xFFFFAAFD;
////		val[1] = 0x00400080;
////		return at32f40x_option_write(t, val, 2);
////	case ID_AT32F413:
////		val[0] = 0x7FFFAAFD;
////		return at32f40x_option_write(t, val, 1);
//	default:
//		val[0] = 0x0FFFAAED;
//		return at32f40x_option_write(t, val, 1);
//	}
//}

//static bool at32f40x_cmd_option(target *t, int argc, char *argv[]) {
//	uint32_t val[3];
//	int count = 0, readcount = 1;
//
////	switch (t->idcode) {
////	case ID_STM32F72X: /* STM32F72|3 */
////		readcount++;
////		/* fall through.*/
////	case ID_STM32F74X:
////	case ID_STM32F76X:
////		readcount++;
////		break;
////	case ID_AT32F42X:
////	case ID_AT32F46X:
////		readcount++;
////	}
//
//	if ((argc == 2) && !strncasecmp(argv[1], "erase", 1)) {
//		at32f40x_option_write_default(t);
//	} else if ((argc > 2) && !strncasecmp(argv[1], "write", 1)) {
//		val[0] = strtoul(argv[2], NULL, 0);
//		count++;
//		if (argc > 3) {
//			val[1] = strtoul(argv[3], NULL, 0);
//			count++;
//		}
//		if (argc > 4) {
//			val[2] = strtoul(argv[4], NULL, 0);
//			count++;
//		}
//		if (optcr_mask(t, val))
//			at32f40x_option_write(t, val, count);
//		else
//			tc_printf(t, "error\n");
//	} else {
//		tc_printf(t, "usage: monitor option erase\n");
//		tc_printf(t, "usage: monitor option write <OPTCR>");
//		if (readcount > 1)
//			tc_printf(t, " <OPTCR1>");
//		if (readcount > 2)
//			tc_printf(t, " <OPTCR2>");
//		tc_printf(t, "\n");
//	}
//
////	val[0] = target_mem_read32(t, FLASH_OPTCR);
//	if (readcount > 1)
////		val[1] = target_mem_read32(t, FLASH_OPTCR + 4);
//		if (readcount > 2)
////		val[2] = target_mem_read32(t, FLASH_OPTCR + 8);
//			optcr_mask(t, val);
//	tc_printf(t, "OPTCR: 0x%08X ", val[0]);
//	if (readcount > 1)
//		tc_printf(t, "OPTCR1: 0x%08lx ", val[1]);
//	if (readcount > 2)
//		tc_printf(t, "OPTCR2: 0x%08lx", val[2]);
//	tc_printf(t, "\n");
//	return true;
//}

static bool at32f40x_cmd_psize(target *t, int argc, char *argv[]) {
	if (argc == 1) {
		enum align psize = ALIGN_WORD;
		for (struct target_flash *f = t->flash; f; f = f->next) {
			if (f->write == at32f40x_flash_write) {
				psize = ((struct at32f40x_flash*) f)->psize;
			}
		}
		tc_printf(t, "Flash write parallelism: %s\n",
				psize == ALIGN_DWORD ? "x64" : psize == ALIGN_WORD ? "x32" :
				psize == ALIGN_HALFWORD ? "x16" : "x8");
	} else {
		enum align psize;
		if (!strcmp(argv[1], "x8")) {
			psize = ALIGN_BYTE;
		} else if (!strcmp(argv[1], "x16")) {
			psize = ALIGN_HALFWORD;
		} else if (!strcmp(argv[1], "x32")) {
			psize = ALIGN_WORD;
		} else if (!strcmp(argv[1], "x64")) {
			psize = ALIGN_DWORD;
		} else {
			tc_printf(t, "usage: monitor psize (x8|x16|x32|x32)\n");
			return false;
		}
		for (struct target_flash *f = t->flash; f; f = f->next) {
			if (f->write == at32f40x_flash_write) {
				((struct at32f40x_flash*) f)->psize = psize;
			}
		}
	}
	return true;
}
