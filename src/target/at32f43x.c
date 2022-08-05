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

/* This file implements at32f43x target specific functions for detecting
 * the device, providing the XML memory map and Flash memory programming.
 *
 * Refereces:
 * ST doc - RM0090
 *   Reference manual - at32f43x05xx, at32f43x07xx, at32f43x3x15xx and at32f43x3x17xx
 *   advanced ARM-based 32-bit MCUs
 * ST doc - PM0081
 *   Programming manual - at32f43x3x0xxx and at32f43x1xxx Flash programming
 *    manual
 */

#include "general.h"
#include "target.h"
#include "target_internal.h"
#include "cortexm.h"
#include "gdb_packet.h"

//static bool at32f43x_cmd_option(target *t, int argc, char **argv);
static bool at32f43x_cmd_psize(target *t, int argc, char **argv);
static bool at32f43x_cmd_erase_mass(target *t, int argc, const char **argv);

const struct command_s at32f43x_cmd_list[] = {
		{"erase_mass", (cmd_handler)at32f43x_cmd_erase_mass,
			 "Erase entire flash memory"},
		 { "psize",
		(cmd_handler) at32f43x_cmd_psize,
		"Configure flash write parallelism: (x8|x16|x32(default)|x64)" }, {
NULL, NULL, NULL } };


static bool at32f43x_attach(target *t);
static int at32f43x_flash_erase(struct target_flash *f, target_addr addr,
		size_t len);
static int at32f43x_flash_write(struct target_flash *f, target_addr dest,
		const void *src, size_t len);


/* Flash Program and Erase Controller Register Map */
#define AT_FPEC_BASE	0x40023C00

#define AT_FLASH_ACR	(AT_FPEC_BASE+0x00)
#define AT_FLASH_UNLOCK	(AT_FPEC_BASE+0x04)
#define AT_FLASH_OPTKEYR	(AT_FPEC_BASE+0x08)
#define AT_FLASH_STS	(AT_FPEC_BASE+0x0C)
#define AT_FLASH_CTRL	(AT_FPEC_BASE+0x10)
#define AT_FLASH_ADDR	(AT_FPEC_BASE+0x14)

#define AT_FLASH_CTRL_FPRGM		(1 << 0)
#define AT_FLASH_CTRL_BANKERS		(1 << 2)
#define AT_FLASH_CTRL_BLKERS		(1 << 3)
#define AT_FLASH_CTRL_SECERS		(1 << 1)
#define AT_FLASH_CTRL_ERSTR		(1 << 6)
//#define AT_FLASH_CTRL_ODFIE		(1 << 12)
//#define AT_FLASH_CTRL_ERRIE		(1 << 10)
//#define AT_FLASH_CTRL_OPLK		(1 << 7)

#define AT_FLASH_STS_OBF		(1 << 0)

#define AT_KEY1 0x45670123
#define AT_KEY2 0xCDEF89AB


#define AT_SR_ERROR_MASK	0xF2



#define AT_DBGMCU_IDCODE	0xE0042000
#define AT_DBGMCU_CR		0xE0042004
#define AT_DBG_SLEEP		(1 <<  0)

#define AT_DBGMCU_CR_DBG_SLEEP		(0x1U << 0U)
#define AT_DBGMCU_CR_DBG_STOP		(0x1U << 1U)
#define AT_DBGMCU_CR_DBG_STANDBY	(0x1U << 2U)

struct at32f43x_flash {
	struct target_flash f;
	enum align psize;
	uint8_t base_sector;
	uint8_t bank_split;
};

struct at32f43x_priv_s {
	uint32_t dbgmcu_cr;
};

enum IDS_AT32F43 {

	ID_AT32F437 = 0x54f
};

static void at32f43x_add_flash(target *t, uint32_t addr, size_t length,
		size_t blocksize, unsigned int base_sector, int split) {
	if (length == 0)
		return;
	struct at32f43x_flash *sf = calloc(1, sizeof(*sf));
	if (!sf) { /* calloc failed: heap exhaustion */
		DEBUG_WARN("calloc: failed in %s\n", __func__);
		return;
	}

	struct target_flash *f = &sf->f;
	f->start = addr;
	f->length = length;
	f->blocksize = blocksize;
	f->erase = at32f43x_flash_erase;
	f->write = at32f43x_flash_write;
	f->buf_size = 1024;
	f->erased = 0xff;
	sf->base_sector = base_sector;
	sf->bank_split = split;
	sf->psize = ALIGN_WORD;
	target_add_flash(t, f);
}

static char* at32f43x_get_chip_name(uint32_t idcode) {
	switch (idcode) {

	case ID_AT32F437: /* F42XxG/I */
		return "AT32F437";
	default:
		return NULL;
	}
}

static void at32f43x_detach(target *t) {
	struct at32f43x_priv_s *ps = (struct at32f43x_priv_s*) t->target_storage;

	/*reverse all changes to AT_DBGMCU_CR*/
	target_mem_write32(t, AT_DBGMCU_CR, ps->dbgmcu_cr);
	cortexm_detach(t);
}

bool at32f43x_probe(target *t) {
	t->idcode = target_mem_read32(t, AT_DBGMCU_IDCODE) & 0xfff;

	switch (t->idcode) {



	case ID_AT32F437: /* F413     RM0430 Rev.2, 320 kB Ram, 1.5 MB flash. */
//		t->mass_erase = at32f43x_mass_erase;
		t->detach = at32f43x_detach;
		t->driver = at32f43x_get_chip_name(t->idcode);
		t->attach = at32f43x_attach;
		target_add_commands(t, at32f43x_cmd_list, t->driver);

		return true;
	default:
		return false;
	}
}

static bool at32f43x_attach(target *t) {

	uint16_t max_flashsize;

	if (!cortexm_attach(t))
		return false;

	switch (t->idcode) {


	case ID_AT32F437: /* F76x F77x RM0410 */
		max_flashsize = 4032;
		break;
	default:
		return false;
	}

	/* Save AT_DBGMCU_CR to restore it when detaching*/
	struct at32f43x_priv_s *priv_storage = calloc(1, sizeof(*priv_storage));
	if (!priv_storage) { /* calloc failed: heap exhaustion */
		DEBUG_WARN("calloc: failed in %s\n", __func__);
		return false;
	}
	priv_storage->dbgmcu_cr = target_mem_read32(t, AT_DBGMCU_CR);
	t->target_storage = (void*) priv_storage;
	/* Enable debugging during all low power modes*/

	target_mem_write32(t, AT_DBGMCU_CR, priv_storage->dbgmcu_cr |
	AT_DBGMCU_CR_DBG_SLEEP | AT_DBGMCU_CR_DBG_STANDBY | AT_DBGMCU_CR_DBG_STOP);

	/* Free previously loaded memory map */

	target_mem_map_free(t);

	target_add_ram(t, 0x20000000, 0x80000); /* 320 k RAM */

	int split = 0;
	uint32_t banksize;

	banksize = max_flashsize << 10;



	switch (t->idcode) {


	case ID_AT32F437: /* F76x F77x RM0410 */
		at32f43x_add_flash(t, 0x8000000, banksize, 0x10000, 0, split);
		break;
	default:
		break;
	}

	return (true);
}

static void at32f43x_AT_FLASH_UNLOCK(target *t) {

	target_mem_write32(t, AT_FLASH_UNLOCK, AT_KEY1);
	target_mem_write32(t, AT_FLASH_UNLOCK, AT_KEY2);

}

static int at32f43x_flash_erase(struct target_flash *f, target_addr addr,
		size_t len) {
	target *t = f->t;
//	struct at32f43x_flash *sf = (struct at32f43x_flash*) f;
//	uint32_t sr;
	/* No address translation is needed here, as we erase by sector number */
	uint32_t sector = addr;
	at32f43x_AT_FLASH_UNLOCK(t);

	while (len) {

		/* Flash page erase instruction */
		switch (t->idcode) {

		case ID_AT32F437: /* F76x F77x RM0410 */
			target_mem_write32(t, AT_FLASH_ADDR, sector);
			/* write address to FMA */
			target_mem_write32(t, AT_FLASH_CTRL,
					(AT_FLASH_CTRL_BLKERS | AT_FLASH_CTRL_ERSTR));
			break;
		default:
			break;
		}

		/* Read AT_FLASH_STS to poll for BSY bit */
		while (target_mem_read32(t, AT_FLASH_STS) & AT_FLASH_STS_OBF)
			if (target_check_error(t)) {
				DEBUG_WARN("at32f43x flash erase: comm error\n");
				return -1;
			}
		if (len > f->blocksize)
			len -= f->blocksize;
		else
			len = 0;
		switch (t->idcode) {


		case ID_AT32F437: /* F76x F77x RM0410 */
			sector += 0x10000;
			break;
		default:
			break;
		}

	}

	/* Check for error */

	return 0;
}

static int at32f43x_flash_write(struct target_flash *f, target_addr dest,
		const void *src, size_t len) {
	/* Translate ITCM addresses to AXIM */

	target *t = f->t;
	uint32_t sr;
	enum align psize = ((struct at32f43x_flash*) f)->psize;
	target_mem_write32(t, AT_FLASH_CTRL, AT_FLASH_CTRL_FPRGM);
	cortexm_mem_write_sized(t, dest, src, len, psize);
	/* Read AT_FLASH_STS to poll for BSY bit */
	/* Wait for completion or an error */
	do {
		sr = target_mem_read32(t, AT_FLASH_STS);
		if (target_check_error(t)) {
			DEBUG_WARN("at32f43x flash write: comm error\n");
			return -1;
		}
	} while (sr & AT_FLASH_STS_OBF);


	return 0;
}

static bool at32f43x_cmd_erase_mass(target *t, int argc, const char **argv) {

	(void)argc;
	(void)argv;
	const char spinner[] = "|/-\\";
	int spinindex = 0;

	tc_printf(t, "Erasing flash... This may take a few seconds.  ");
	at32f43x_AT_FLASH_UNLOCK(t);

	/* Flash mass erase start instruction */
	target_mem_write32(t, AT_FLASH_CTRL, (AT_FLASH_CTRL_BANKERS | AT_FLASH_CTRL_ERSTR));

	while (target_mem_read32(t, AT_FLASH_STS) & AT_FLASH_STS_OBF) {
		tc_printf(t, "\b%c", spinner[spinindex++ % 4]);
		if(target_check_error(t)) {
			tc_printf(t, "\n");
			return false;
		}
	}
	tc_printf(t, "\n");
	/* Check for error */

	const uint32_t result = target_mem_read32(t, AT_FLASH_STS);
	return !(result & AT_SR_ERROR_MASK);
}


static bool at32f43x_cmd_psize(target *t, int argc, char *argv[]) {
	if (argc == 1) {
		enum align psize = ALIGN_WORD;
		for (struct target_flash *f = t->flash; f; f = f->next) {
			if (f->write == at32f43x_flash_write) {
				psize = ((struct at32f43x_flash*) f)->psize;
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
			if (f->write == at32f43x_flash_write) {
				((struct at32f43x_flash*) f)->psize = psize;
			}
		}
	}
	return true;
}
