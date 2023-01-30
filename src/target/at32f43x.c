//Test2


#include "general.h"
#include "target.h"
#include "target_internal.h"
#include "cortexm.h"

#define AT32F4x_IDCODE_SERIES_MASK 0xfffff000U
#define AT32F4x_IDCODE_PART_MASK   0x00000fffU
#define AT32F41_SERIES             0x70030000U
#define AT32F40_SERIES             0x70050000U
#define AT32F43_SERIES             0x70084000U

#define AT_FPEC_BASE	0x40023C00

#define AT_FLASH_ACR	(AT_FPEC_BASE+0x00)
#define AT_FLASH_UNLOCK	(AT_FPEC_BASE+0x04)
#define AT_FLASH_USD_UNLOCK	(AT_FPEC_BASE+0x08)
#define AT_FLASH_STS	(AT_FPEC_BASE+0x0C)
#define AT_FLASH_CTRL	(AT_FPEC_BASE+0x10)
#define AT_FLASH_ADDR	(AT_FPEC_BASE+0x14)
#define AT_FLASH_USD	(AT_FPEC_BASE+0x1C)

#define AT_FLASH_CTRL_FPRGM		(1 << 0)
#define AT_FLASH_CTRL_BANKERS		(1 << 2)
#define AT_FLASH_CTRL_USDERS		(1 << 5)
#define AT_FLASH_CTRL_BLKERS		(1 << 3)
#define AT_FLASH_CTRL_SECERS		(1 << 1)
#define AT_FLASH_CTRL_ERSTR		(1 << 6)
#define AT_FLASH_CTRL_USDPRGM	(1 << 4)
//#define AT_FLASH_CTRL_ODFIE		(1 << 12)
//#define AT_FLASH_CTRL_ERRIE		(1 << 10)
//#define AT_FLASH_CTRL_OPLK		(1 << 7)

#define AT_FLASH_STS_OBF		(1 << 0)
#define AT_FLASH_FAP      0x1FFFC000
#define AT_KEY1 0x45670123
#define AT_KEY2 0xCDEF89AB

#define FAP_RELIEVE_KEY   0x5aa5U

#define FLASH_OBP_RDP_KEY    0x5aa5U

#define AT_SR_ERROR_MASK	0xF2

#define AT_DBGMCU_IDCODE	0xE0042000
#define AT_DBGMCU_CR		0xE0042004
#define AT_DBG_SLEEP		(1 <<  0)

#define AT_DBGMCU_CR_DBG_SLEEP		(0x1U << 0U)
#define AT_DBGMCU_CR_DBG_STOP		(0x1U << 1U)
#define AT_DBGMCU_CR_DBG_STANDBY	(0x1U << 2U)

static bool at32f43x_option_write(target_s *t);
static bool at32f43x_cmd_option(target_s *t, int argc, const char **argv);
static bool at32f43x_cmd_erase_mass(target_s *t);
static bool at32f43x_option_erase(target_s *t);
static bool at32f43x_flash_write(target_flash_s *f, target_addr_t dest,
		const void *src, size_t len);
static bool at32f43x_flash_erase(target_flash_s *f, target_addr_t addr,
		size_t len);
static bool at32f43x_cmd_option(target_s *t, int argc, const char **argv);
const command_s at32f43x_cmd_list[] = {

{ "option", at32f43x_cmd_option, "Manipulate option bytes" } };

static void at32f43x_add_flash(target_s *t, uint32_t addr, size_t length,
		size_t blocksize) {

	target_flash_s *f = calloc(1, sizeof(*f));
	if (!f) { /* calloc failed: heap exhaustion */
		DEBUG_WARN("calloc: failed in %s\n", __func__);
		return;
	}

	f->start = addr;
	f->length = length;
	f->blocksize = blocksize;
	f->erase = at32f43x_flash_erase;
	f->write = at32f43x_flash_write;
	f->writesize = blocksize;
	f->erased = 0xff;

	target_add_flash(t, f);
}
static bool at32f40_detect(target_s *t, const uint16_t part_id) {
// Current driver supports only *default* memory layout (256 KB Flash / 96 KB SRAM)
// XXX: Support for external Flash for 512KB and 1024KB parts requires specific flash code (not implemented)
	switch (part_id) {
	case 0x0240U: // AT32F403AVCT7 256KB / LQFP100
	case 0x0241U: // AT32F403ARCT7 256KB / LQFP64
	case 0x0242U: // AT32F403ACCT7 256KB / LQFP48
	case 0x0243U: // AT32F403ACCU7 256KB / QFN48
	case 0x0249U: // AT32F407VCT7 256KB / LQFP100
	case 0x024aU: // AT32F407RCT7 256KB / LQFP64
	case 0x0254U: // AT32F407AVCT7 256KB / LQFP100
	case 0x02cdU: // AT32F403AVET7 512KB / LQFP100 (*)
	case 0x02ceU: // AT32F403ARET7 512KB / LQFP64 (*)
	case 0x02cfU: // AT32F403ACET7 512KB / LQFP48 (*)
	case 0x02d0U: // AT32F403ACEU7 512KB / QFN48 (*)
	case 0x02d1U: // AT32F407VET7 512KB / LQFP100 (*)
	case 0x02d2U: // AT32F407RET7 512KB / LQFP64 (*)
	case 0x0344U: // AT32F403AVGT7 1024KB / LQFP100 (*)
	case 0x0345U: // AT32F403ARGT7 1024KB / LQFP64 (*)
	case 0x0346U: // AT32F403ACGT7 1024KB / LQFP48 (*)
	case 0x0347U: // AT32F403ACGU7 1024KB / QFN48 (found on BlackPill+ WeAct Studio) (*)
	case 0x034bU: // AT32F407VGT7 1024KB / LQFP100 (*)
	case 0x034cU: // AT32F407VGT7 1024KB / LQFP64 (*)
	case 0x0353U: // AT32F407AVGT7 1024KB / LQFP100 (*)
		// Flash: 256 KB / 2KB per block
		at32f43x_add_flash(t, 0x08000000, 256U * 1024U, 2U * 1024U);
		break;
		// Unknown/undocumented
	default:
		return false;
	}
// All parts have 96KB SRAM
	target_add_ram(t, 0x20000000, 96U * 1024U);
	t->driver = "AT32F403A/407";
	t->mass_erase = at32f43x_cmd_erase_mass;
	return true;
}

static bool at32f41_detect(target_s *t, const uint16_t part_id) {
	switch (part_id) {
	case 0x0240U: // LQFP64_10x10
	case 0x0241U: // LQFP48_7x7
	case 0x0242U: // QFN32_4x4
	case 0x0243U: // LQFP64_7x7
	case 0x024cU: // QFN48_6x6
		// Flash: 256 KB / 2KB per block
		at32f43x_add_flash(t, 0x08000000, 256U * 1024U, 2U * 1024U);
		break;
	case 0x01c4U: // LQFP64_10x10
	case 0x01c5U: // LQFP48_7x7
	case 0x01c6U: // QFN32_4x4
	case 0x01c7U: // LQFP64_7x7
	case 0x01cdU: // QFN48_6x6
		// Flash: 128 KB / 2KB per block
		at32f43x_add_flash(t, 0x08000000, 128U * 1024U, 2U * 1024U);
		break;
	case 0x0108U: // LQFP64_10x10
	case 0x0109U: // LQFP48_7x7
	case 0x010aU: // QFN32_4x4
		// Flash: 64 KB / 2KB per block
		at32f43x_add_flash(t, 0x08000000, 64U * 1024U, 2U * 1024U);
		break;
		// Unknown/undocumented
	default:
		return false;
	}
// All parts have 32KB SRAM
	target_add_ram(t, 0x20000000, 32U * 1024U);
	t->driver = "AT32F415";
	t->mass_erase = at32f43x_cmd_erase_mass;
	return true;
}

static bool at32f43_detect(target_s *t, const uint16_t part_id) {
	switch (part_id) {

	case 0x54fU: // QFN32_4x4
		// Flash: 64 KB / 2KB per block

		t->driver = "AT32F437";
		uint32_t banksize = 4032 << 10;
		target_add_ram(t, 0x20000000, 0x80000); /* 512 k RAM */
		at32f43x_add_flash(t, 0x8000000, banksize, 0x1000);
		target_add_commands(t, at32f43x_cmd_list, t->driver);
		break;
		// Unknown/undocumented
	default:
		return false;
	}
// All parts have 32KB SRAM

	t->mass_erase = at32f43x_cmd_erase_mass;
	return true;
}

/* Identify AT32F4x devices (Cortex-M4) */
bool at32fxx_probe(target_s *t) {
// Artery clones use Cortex M4 cores
	if ((t->cpuid & CPUID_PARTNO_MASK) != CORTEX_M4)
		return false;

// Artery chips use the complete idcode word for identification
	const uint32_t idcode = target_mem_read32(t, AT_DBGMCU_IDCODE);
	const uint32_t series = idcode & AT32F4x_IDCODE_SERIES_MASK;
	const uint16_t part_id = idcode & AT32F4x_IDCODE_PART_MASK;
	if (series == AT32F40_SERIES)
		return at32f40_detect(t, part_id);
	if (series == AT32F41_SERIES)
		return at32f41_detect(t, part_id);
	if (series == AT32F43_SERIES)
		return at32f43_detect(t, part_id);
	return false;
}

static void at32f43x_AT_FLASH_UNLOCK(target_s *t) {

	target_mem_write32(t, AT_FLASH_UNLOCK, AT_KEY1);
	target_mem_write32(t, AT_FLASH_UNLOCK, AT_KEY2);

}

static bool at32f43x_flash_erase(target_flash_s *f, target_addr_t addr,
		size_t len) {
	target_s *t = f->t;
//	struct at32f43x_flash *sf = (struct at32f43x_flash*) f;
//	uint32_t sr;
	/* No address translation is needed here, as we erase by sector number */
	uint32_t sector = addr;
	at32f43x_AT_FLASH_UNLOCK(t);

	while (len) {

		/* Flash page erase instruction */

		target_mem_write32(t, AT_FLASH_ADDR, sector);
		/* write address to FMA */
		target_mem_write32(t, AT_FLASH_CTRL,
				(AT_FLASH_CTRL_BLKERS | AT_FLASH_CTRL_ERSTR));

		/* Read AT_FLASH_STS to poll for BSY bit */
		while (target_mem_read32(t, AT_FLASH_STS) & AT_FLASH_STS_OBF)
			if (target_check_error(t)) {
				DEBUG_WARN("at32f43x flash erase: comm error\n");
				return false;
			}
		if (len >= f->blocksize)
			len -= f->blocksize;
		else
			len = 0;

		sector += 0x10000;

	}

	/* Check for error */

	return true;
}

static bool at32f43x_flash_write(target_flash_s *f, target_addr_t dest,
		const void *src, size_t len) {
	/* Translate ITCM addresses to AXIM */

	target_s *t = f->t;
	uint32_t sr;
//	enum align psize = ((struct at32f43x_flash*) f)->psize;
	target_mem_write32(t, AT_FLASH_CTRL, AT_FLASH_CTRL_FPRGM);
	cortexm_mem_write_sized(t, dest, src, len, ALIGN_WORD);
	/* Read AT_FLASH_STS to poll for BSY bit */
	/* Wait for completion or an error */
	do {
		sr = target_mem_read32(t, AT_FLASH_STS);
		if (target_check_error(t)) {
			DEBUG_WARN("at32f43x flash write: comm error\n");
			return false;
		}
	} while (sr & AT_FLASH_STS_OBF);

	return true;
}

static bool at32f43x_cmd_erase_mass(target_s *t) {

	const char spinner[] = "|/-\\";
	int spinindex = 0;

	tc_printf(t, "Erasing flash... This may take a few seconds.  ");
	at32f43x_AT_FLASH_UNLOCK(t);

	/* Flash mass erase start instruction */
	target_mem_write32(t, AT_FLASH_CTRL,
			(AT_FLASH_CTRL_BANKERS | AT_FLASH_CTRL_ERSTR));

	while (target_mem_read32(t, AT_FLASH_STS) & AT_FLASH_STS_OBF) {
		tc_printf(t, "\b%c", spinner[spinindex++ % 4]);
		if (target_check_error(t)) {
			tc_printf(t, "\n");
			return false;
		}
	}
	tc_printf(t, "\n");
	/* Check for error */

	const uint32_t result = target_mem_read32(t, AT_FLASH_STS);
	return !(result & AT_SR_ERROR_MASK);
}

static bool at32f43x_option_erase(target_s *t) {

	/* Erase option bytes instruction */

	const char spinner[] = "|/-\\";
	int spinindex = 0;

	tc_printf(t, "Erasing flash... This may take a few seconds.  ");
	target_mem_write32(t, AT_FLASH_USD_UNLOCK, AT_KEY1);
	target_mem_write32(t, AT_FLASH_USD_UNLOCK, AT_KEY2);
	at32f43x_AT_FLASH_UNLOCK(t);

	/* Flash mass erase start instruction */
	target_mem_write32(t, AT_FLASH_CTRL,
			(AT_FLASH_CTRL_USDERS | AT_FLASH_CTRL_ERSTR));

	while (target_mem_read32(t, AT_FLASH_STS) & AT_FLASH_STS_OBF) {
		tc_printf(t, "\b%c", spinner[spinindex++ % 4]);
		if (target_check_error(t)) {
			tc_printf(t, "\n");
			return false;
		}
	}
	tc_printf(t, "\n");
	/* Check for error */
	const uint32_t result = target_mem_read32(t, AT_FLASH_USD);
	tc_printf(t, "result: %x\n", result);
	at32f43x_option_write(t);

//		const uint32_t result = target_mem_read32(t, AT_FLASH_STS);
	return !(result & AT_SR_ERROR_MASK);

	return true;
}

static bool at32f43x_option_write(target_s *t) {

	const char spinner[] = "|/-\\";
	int spinindex = 0;
	/* Flash mass erase start instruction */
	target_mem_write32(t, AT_FLASH_CTRL, AT_FLASH_CTRL_USDPRGM);
	target_mem_write16(t, AT_FLASH_FAP, FAP_RELIEVE_KEY);

	while (target_mem_read32(t, AT_FLASH_STS) & AT_FLASH_STS_OBF) {
		tc_printf(t, "\b%c", spinner[spinindex++ % 4]);
		if (target_check_error(t)) {
			tc_printf(t, "\n");
			return false;
		}
	}

	const uint32_t result = target_mem_read32(t, AT_FLASH_USD);

	tc_printf(t, "result: %x\n", result);
	return true;
}

static bool at32f43x_cmd_option(target_s *t, int argc, const char **argv) {

	const uint32_t rdprt = target_mem_read32(t, AT_FLASH_USD);

	at32f43x_AT_FLASH_UNLOCK(t);

	target_mem_write32(t, AT_FLASH_USD_UNLOCK, AT_KEY1);
	target_mem_write32(t, AT_FLASH_USD_UNLOCK, AT_KEY2);
	const uint32_t CTRL = target_mem_read32(t, AT_FLASH_CTRL);
	if (argc == 2 && strcmp(argv[1], "erase") == 0) {
		at32f43x_option_erase(t);

	} else if (rdprt) {
		tc_printf(t,
				"Device is Read Protected\nUse `monitor option erase` to unprotect and erase device\n");
		return true;

	} else if (CTRL) {
		tc_printf(t, "ce\n");
		return true;
	} else
		tc_printf(t,
				"usage: monitor option erase\nusage: monitor option <addr> <value>\n");

	for (size_t i = 0U; i < 16U; i += 4U) {
		const uint32_t addr = AT_FLASH_FAP + i;
		const uint32_t val = target_mem_read32(t, addr);
		tc_printf(t, "0x%08X: 0x%04X\n", addr, val & 0xffffU);
		tc_printf(t, "0x%08X: 0x%04X\n", addr + 2U, val >> 16U);
	}

	for (size_t i = 0U; i < 16U; i += 4U) {
		const uint32_t addr = AT_FLASH_USD + i;
		const uint32_t val = target_mem_read32(t, addr);
		tc_printf(t, "0x%08X: 0x%04X\n", addr, val & 0xffffU);
		tc_printf(t, "0x%08X: 0x%04X\n", addr + 2U, val >> 16U);
	}

	return true;
}

