// SPDX-License-Identifier: GPL-2.0+
/*
 * Raspberry Pi SMBIOS sysinfo driver.
 *
 * Supplies the SMBIOS (Type 1-4, 7) values that the generic tables cannot
 * derive on their own: board revision, serial number, a stable UUID and
 * the SoC's processor/cache topology.  Everything is queried live from the
 * VideoCore firmware (mailbox) and the control DT, so the tables are
 * correct no matter which DTB the firmware handed over — unlike a static
 * "u-boot,sysinfo-smbios" DT node, which is only present when U-Boot's own
 * bundled DTB is used.
 *
 * The device is bound via U_BOOT_DRVINFO (dm_scan_plat), which runs before
 * the DT scan, so this driver is the first UCLASS_SYSINFO device and takes
 * precedence over any DT smbios node.
 *
 * Note: lib/smbios.c lets the "serial#" environment variable override the
 * Type 1 serial number/UUID string.  This deployment relies on that to
 * carry the Talos NoCloud datasource string; the driver's UUID (via
 * SYSID_SM_SYSTEM_UUID) still replaces the UUID field afterwards, so the
 * reported UUID stays stable either way.
 */

#include <dm.h>
#include <memalign.h>
#include <smbios.h>
#include <sysinfo.h>
#include <dm/ofnode.h>
#include <asm/arch/mbox.h>
#include <asm/global_data.h>
#include <linux/bitops.h>

DECLARE_GLOBAL_DATA_PTR;

struct msg_get_board_rev {
	struct bcm2835_mbox_hdr hdr;
	struct bcm2835_mbox_tag_get_board_rev get_board_rev;
	u32 end_tag;
};

struct msg_get_board_serial {
	struct bcm2835_mbox_hdr hdr;
	struct bcm2835_mbox_tag_get_board_serial get_board_serial;
	u32 end_tag;
};

/**
 * struct rpi_soc_info - per-SoC processor and cache topology
 *
 * Cache sizes are totals per level across all cores, in KiB.
 */
struct rpi_soc_info {
	const char *part;    /* SoC part number */
	const char *core;    /* ARM core name */
	u16 max_mhz;         /* maximum core clock */
	u16 ext_clock_mhz;   /* external oscillator */
	u8 cores;
	u8 levels;           /* number of cache levels */
	u16 cache_kb[SYSINFO_CACHE_LVL_MAX];
	u8 cache_assoc[SYSINFO_CACHE_LVL_MAX];
	u8 cache_ecc[SYSINFO_CACHE_LVL_MAX];
	u8 mem_type;         /* SMBIOS Type 17 memory type */
	u16 mem_speed;       /* memory data rate in MT/s, 0 = unknown */
};

/* Indexed by the processor field of the new-scheme revision code */
static const struct rpi_soc_info rpi_socs[] = {
	[0] = {
		.part = "BCM2835", .core = "ARM1176JZF-S",
		.max_mhz = 700, .ext_clock_mhz = 19, .cores = 1, .levels = 1,
		.cache_kb = { 32 },
		.cache_assoc = { SMBIOS_CACHE_ASSOC_4WAY },
		.cache_ecc = { SMBIOS_CACHE_ERRCORR_NONE },
		.mem_type = SMBIOS_MD_TYPE_LPDDR2,
	},
	[1] = {
		.part = "BCM2836", .core = "Cortex-A7",
		.max_mhz = 900, .ext_clock_mhz = 19, .cores = 4, .levels = 2,
		.cache_kb = { 256, 512 },
		.cache_assoc = { SMBIOS_CACHE_ASSOC_OTHER,
				 SMBIOS_CACHE_ASSOC_8WAY },
		.cache_ecc = { SMBIOS_CACHE_ERRCORR_PARITY,
			       SMBIOS_CACHE_ERRCORR_NONE },
		.mem_type = SMBIOS_MD_TYPE_LPDDR2,
	},
	[2] = {
		.part = "BCM2837", .core = "Cortex-A53",
		.max_mhz = 1400, .ext_clock_mhz = 19, .cores = 4, .levels = 2,
		.cache_kb = { 256, 512 },
		.cache_assoc = { SMBIOS_CACHE_ASSOC_OTHER,
				 SMBIOS_CACHE_ASSOC_16WAY },
		.cache_ecc = { SMBIOS_CACHE_ERRCORR_PARITY,
			       SMBIOS_CACHE_ERRCORR_NONE },
		.mem_type = SMBIOS_MD_TYPE_LPDDR2,
	},
	[3] = {
		.part = "BCM2711", .core = "Cortex-A72",
		.max_mhz = 1800, .ext_clock_mhz = 54, .cores = 4, .levels = 2,
		.cache_kb = { 320, 1024 },
		.cache_assoc = { SMBIOS_CACHE_ASSOC_OTHER,
				 SMBIOS_CACHE_ASSOC_16WAY },
		.cache_ecc = { SMBIOS_CACHE_ERRCORR_PARITY,
			       SMBIOS_CACHE_ERRCORR_SBITECC },
		.mem_type = SMBIOS_MD_TYPE_LPDDR4, .mem_speed = 3200,
	},
	[4] = {
		.part = "BCM2712", .core = "Cortex-A76",
		.max_mhz = 2400, .ext_clock_mhz = 54, .cores = 4, .levels = 3,
		.cache_kb = { 512, 2048, 2048 },
		.cache_assoc = { SMBIOS_CACHE_ASSOC_4WAY,
				 SMBIOS_CACHE_ASSOC_8WAY,
				 SMBIOS_CACHE_ASSOC_16WAY },
		.cache_ecc = { SMBIOS_CACHE_ERRCORR_PARITY,
			       SMBIOS_CACHE_ERRCORR_SBITECC,
			       SMBIOS_CACHE_ERRCORR_SBITECC },
		/* BCM2712 is LPDDR4X; SMBIOS has no distinct 4X code */
		.mem_type = SMBIOS_MD_TYPE_LPDDR4, .mem_speed = 4267,
	},
};

struct rpi_sysinfo_priv {
	bool detected;
	u32 revision;
	const struct rpi_soc_info *soc;
	char serial_str[17];
	char version_str[8];
	char sku_str[12];
	char cpu_str[32];
	u32 processor_id[2];
	u8 uuid[16];
	u16 cache_handles[SYSINFO_CACHE_LVL_MAX];
	u16 marray_handles[SYSINFO_MEM_HANDLE_MAX];
	u64 mem_start;         /* lowest DRAM bank start, bytes */
	u64 mem_end;           /* highest DRAM bank end, bytes (inclusive) */
	u32 mem_max_cap_kb;    /* Type 16 maximum capacity, KB */
	u32 mem_ext_size_kb;   /* Type 17 extended size, KB (0 if size fits) */
	u32 mem_map_start_kb;  /* Type 19 starting address, KB */
	u32 mem_map_end_kb;    /* Type 19 ending address, KB */
	u16 mem_size_mb;       /* Type 17 size, MB (or SMBIOS_MD_SIZE_EXT) */
};

static int rpi_sysinfo_detect(struct udevice *dev)
{
	ALLOC_CACHE_ALIGN_BUFFER(struct msg_get_board_rev, msg_rev, 1);
	ALLOC_CACHE_ALIGN_BUFFER(struct msg_get_board_serial, msg_ser, 1);
	struct rpi_sysinfo_priv *priv = dev_get_priv(dev);
	unsigned int soc_idx = 0;
	u64 serial;
	int ret;

	if (priv->detected)
		return 0;

	BCM2835_MBOX_INIT_HDR(msg_rev);
	BCM2835_MBOX_INIT_TAG(&msg_rev->get_board_rev, GET_BOARD_REV);
	ret = bcm2835_mbox_call_prop(BCM2835_MBOX_PROP_CHAN, &msg_rev->hdr);
	if (ret)
		return ret;
	priv->revision = msg_rev->get_board_rev.body.resp.rev;

	BCM2835_MBOX_INIT_HDR(msg_ser);
	BCM2835_MBOX_INIT_TAG_NO_REQ(&msg_ser->get_board_serial,
				     GET_BOARD_SERIAL);
	ret = bcm2835_mbox_call_prop(BCM2835_MBOX_PROP_CHAN, &msg_ser->hdr);
	if (ret)
		return ret;
	serial = msg_ser->get_board_serial.body.resp.serial;

	/* New-scheme revision codes carry the SoC in bits [15:12] */
	if (priv->revision & BIT(23))
		soc_idx = (priv->revision >> 12) & 0xf;
	if (soc_idx >= ARRAY_SIZE(rpi_socs) || !rpi_socs[soc_idx].part)
		soc_idx = 0;
	priv->soc = &rpi_socs[soc_idx];

	snprintf(priv->serial_str, sizeof(priv->serial_str), "%016llx",
		 serial);
	snprintf(priv->sku_str, sizeof(priv->sku_str), "%X", priv->revision);
	if (priv->revision & BIT(23))
		snprintf(priv->version_str, sizeof(priv->version_str), "1.%u",
			 priv->revision & 0xf);
	else
		snprintf(priv->version_str, sizeof(priv->version_str), "%u",
			 priv->revision & 0xff);
	snprintf(priv->cpu_str, sizeof(priv->cpu_str), "%s %s",
		 priv->soc->part, priv->soc->core);

	/*
	 * Deterministic RFC 4122-shaped UUID from the board serial and
	 * revision: same board, same UUID, every boot.
	 */
	memcpy(&priv->uuid[0], &serial, 8);
	memcpy(&priv->uuid[8], &priv->revision, 4);
	priv->uuid[12] = 'R';
	priv->uuid[13] = 'P';
	priv->uuid[14] = 'i';
	priv->uuid[15] = 0;
	priv->uuid[6] = (priv->uuid[6] & 0x0f) | 0x40; /* version 4 */
	priv->uuid[8] = (priv->uuid[8] & 0x3f) | 0x80; /* variant 1 */

#ifdef CONFIG_ARM64
	{
		u64 midr;

		asm volatile("mrs %0, midr_el1" : "=r" (midr));
		priv->processor_id[0] = (u32)midr;
	}
#endif

	/*
	 * Memory topology for SMBIOS types 16/17/19. Sum the populated DRAM
	 * banks for the total (more reliable than the ARM-visible mailbox tag
	 * on >4 GiB boards) and span them for the mapped address range.
	 */
	{
		u64 total = 0;
		u64 start = ~0ULL, end = 0;
		int i;

		if (gd->bd) {
			for (i = 0; i < CONFIG_NR_DRAM_BANKS; i++) {
				u64 bstart = gd->bd->bi_dram[i].start;
				u64 bsize = gd->bd->bi_dram[i].size;

				if (!bsize)
					continue;
				total += bsize;
				if (bstart < start)
					start = bstart;
				if (bstart + bsize - 1 > end)
					end = bstart + bsize - 1;
			}
		}
		if (!total) {
			total = gd->ram_size;
			start = 0;
			end = total ? total - 1 : 0;
		}

		priv->mem_start = start;
		priv->mem_end = end;
		priv->mem_max_cap_kb = (u32)(total >> 10);

		/* Type 17 size is 16-bit MB; fall back to the extended field */
		if ((total >> 20) < SMBIOS_MD_SIZE_EXT) {
			priv->mem_size_mb = (u16)(total >> 20);
			priv->mem_ext_size_kb = 0;
		} else {
			priv->mem_size_mb = SMBIOS_MD_SIZE_EXT;
			priv->mem_ext_size_kb = (u32)(total >> 10);
		}

		priv->mem_map_start_kb = (u32)(start >> 10);
		priv->mem_map_end_kb = (u32)(end >> 10);
	}

	priv->detected = true;
	return 0;
}

static int rpi_sysinfo_get_str(struct udevice *dev, int id, size_t size,
			       char *val)
{
	struct rpi_sysinfo_priv *priv = dev_get_priv(dev);
	const char *str = NULL;

	switch (id) {
	case SYSID_SM_SYSTEM_MANUFACTURER:
	case SYSID_SM_BASEBOARD_MANUFACTURER:
	case SYSID_SM_ENCLOSURE_MANUFACTURER:
	case SYSID_SM_SYSTEM_FAMILY:
		str = "Raspberry Pi";
		break;
	case SYSID_SM_SYSTEM_PRODUCT:
	case SYSID_SM_BASEBOARD_PRODUCT:
	case SYSID_BOARD_MODEL:
		str = ofnode_read_string(ofnode_root(), "model");
		break;
	case SYSID_SM_SYSTEM_VERSION:
	case SYSID_SM_BASEBOARD_VERSION:
	case SYSID_SM_ENCLOSURE_VERSION:
		str = priv->version_str;
		break;
	case SYSID_SM_SYSTEM_SERIAL:
	case SYSID_SM_BASEBOARD_SERIAL:
	case SYSID_SM_ENCLOSURE_SERIAL:
		str = priv->serial_str;
		break;
	case SYSID_SM_SYSTEM_SKU:
	case SYSID_SM_ENCLOSURE_SKU:
		str = priv->sku_str;
		break;
	case SYSID_SM_PROCESSOR_SOCKET:
		str = "SoC";
		break;
	case SYSID_SM_PROCESSOR_MANUFACT:
		str = "Broadcom";
		break;
	case SYSID_SM_PROCESSOR_VERSION:
		str = priv->cpu_str;
		break;
	case SYSID_SM_PROCESSOR_PN:
		str = priv->soc->part;
		break;
	case SYSID_SM_CACHE_SOCKET + SMBIOS_CACHE_LEVEL_1:
		str = "L1 Cache";
		break;
	case SYSID_SM_CACHE_SOCKET + SMBIOS_CACHE_LEVEL_2:
		str = "L2 Cache";
		break;
	case SYSID_SM_CACHE_SOCKET + SMBIOS_CACHE_LEVEL_3:
		str = "L3 Cache";
		break;
	case SYSID_SM_MEMDEV_LOCATOR:
		str = "System Memory";
		break;
	default:
		return -ENOSYS;
	}

	if (!str)
		return -ENOSYS;
	strlcpy(val, str, size);
	return 0;
}

static int rpi_sysinfo_cache_int(struct rpi_sysinfo_priv *priv, int id,
				 int *val)
{
	const struct rpi_soc_info *soc = priv->soc;
	int level;

	if (id >= SYSID_SM_CACHE_CONFIG &&
	    id < SYSID_SM_CACHE_CONFIG + SYSINFO_CACHE_LVL_MAX) {
		level = id - SYSID_SM_CACHE_CONFIG;
		*val = SMBIOS_CACHE_ENABLED | SMBIOS_CACHE_OP_WB | level;
		return 0;
	}
	if ((id >= SYSID_SM_CACHE_MAX_SIZE &&
	     id < SYSID_SM_CACHE_MAX_SIZE + SYSINFO_CACHE_LVL_MAX) ||
	    (id >= SYSID_SM_CACHE_INST_SIZE &&
	     id < SYSID_SM_CACHE_INST_SIZE + SYSINFO_CACHE_LVL_MAX) ||
	    (id >= SYSID_SM_CACHE_MAX_SIZE2 &&
	     id < SYSID_SM_CACHE_MAX_SIZE2 + SYSINFO_CACHE_LVL_MAX) ||
	    (id >= SYSID_SM_CACHE_INST_SIZE2 &&
	     id < SYSID_SM_CACHE_INST_SIZE2 + SYSINFO_CACHE_LVL_MAX)) {
		level = (id - SYSID_SM_CACHE_INFO_START) %
			SYSINFO_CACHE_LVL_MAX;
		*val = soc->cache_kb[level]; /* 1K granularity */
		return 0;
	}
	if (id >= SYSID_SM_CACHE_SCACHE_TYPE &&
	    id < SYSID_SM_CACHE_SCACHE_TYPE + SYSINFO_CACHE_LVL_MAX) {
		/* L1 is split I/D; a single record can't express that */
		level = id - SYSID_SM_CACHE_SCACHE_TYPE;
		*val = level == SMBIOS_CACHE_LEVEL_1 ?
			SMBIOS_CACHE_SYSCACHE_TYPE_OTHER :
			SMBIOS_CACHE_SYSCACHE_TYPE_UNIFIED;
		return 0;
	}
	if (id >= SYSID_SM_CACHE_ASSOC &&
	    id < SYSID_SM_CACHE_ASSOC + SYSINFO_CACHE_LVL_MAX) {
		*val = soc->cache_assoc[id - SYSID_SM_CACHE_ASSOC];
		return 0;
	}
	if (id >= SYSID_SM_CACHE_ERRCOR_TYPE &&
	    id < SYSID_SM_CACHE_ERRCOR_TYPE + SYSINFO_CACHE_LVL_MAX) {
		*val = soc->cache_ecc[id - SYSID_SM_CACHE_ERRCOR_TYPE];
		return 0;
	}
	return -ENOSYS;
}

static int rpi_sysinfo_get_int(struct udevice *dev, int id, int *val)
{
	struct rpi_sysinfo_priv *priv = dev_get_priv(dev);
	const struct rpi_soc_info *soc = priv->soc;

	switch (id) {
	case SYSID_SM_SYSTEM_WAKEUP:
		*val = SMBIOS_WAKEUP_TYPE_POWER_SWITCH;
		return 0;
	case SYSID_SM_BASEBOARD_FEATURE:
		*val = SMBIOS_BOARD_FEAT_HOST_BOARD;
		return 0;
	case SYSID_SM_BASEBOARD_TYPE:
		*val = SMBIOS_BOARD_TYPE_MOTHERBOARD;
		return 0;
	case SYSID_SM_ENCLOSURE_TYPE:
		*val = 0x22; /* Embedded PC */
		return 0;
	case SYSID_SM_ENCLOSURE_BOOTUP:
	case SYSID_SM_ENCLOSURE_POW:
	case SYSID_SM_ENCLOSURE_THERMAL:
		*val = SMBIOS_STATE_SAFE;
		return 0;
	case SYSID_SM_ENCLOSURE_SECURITY:
		*val = SMBIOS_SECURITY_NONE;
		return 0;
	case SYSID_SM_PROCESSOR_TYPE:
		*val = SMBIOS_PROCESSOR_TYPE_CENTRAL;
		return 0;
	case SYSID_SM_PROCESSOR_FAMILY:
		*val = SMBIOS_PROCESSOR_FAMILY_EXT;
		return 0;
	case SYSID_SM_PROCESSOR_FAMILY2:
		*val = SMBIOS_PROCESSOR_FAMILY_ARMV8;
		return 0;
	case SYSID_SM_PROCESSOR_STATUS:
		*val = BIT(6) | SMBIOS_PROCESSOR_STATUS_ENABLED;
		return 0;
	case SYSID_SM_PROCESSOR_UPGRADE:
		*val = SMBIOS_PROCESSOR_UPGRADE_NONE;
		return 0;
	case SYSID_SM_PROCESSOR_CHARA:
		*val = SMBIOS_PROCESSOR_64BIT | SMBIOS_PROCESSOR_MULTICORE |
		       SMBIOS_PROCESSOR_EXEC_PROT | SMBIOS_PROCESSOR_ENH_VIRT;
		return 0;
	case SYSID_SM_PROCESSOR_EXT_CLOCK:
		*val = soc->ext_clock_mhz;
		return 0;
	case SYSID_SM_PROCESSOR_MAX_SPEED:
	case SYSID_SM_PROCESSOR_CUR_SPEED:
		*val = soc->max_mhz;
		return 0;
	case SYSID_SM_PROCESSOR_CORE_CNT:
	case SYSID_SM_PROCESSOR_CORE_EN:
	case SYSID_SM_PROCESSOR_THREAD_CNT:
	case SYSID_SM_PROCESSOR_CORE_CNT2:
	case SYSID_SM_PROCESSOR_CORE_EN2:
	case SYSID_SM_PROCESSOR_THREAD_CNT2:
	case SYSID_SM_PROCESSOR_THREAD_EN:
		*val = soc->cores;
		return 0;
	case SYSID_SM_CACHE_LEVEL:
		*val = soc->levels - 1;
		return 0;
	/* Physical Memory Array (Type 16) */
	case SYSID_SM_MEMARRAY_LOCATION:
		*val = SMBIOS_MA_LOCATION_MOTHERBOARD;
		return 0;
	case SYSID_SM_MEMARRAY_USE:
		*val = SMBIOS_MA_USE_SYSTEM;
		return 0;
	case SYSID_SM_MEMARRAY_ECC:
		*val = SMBIOS_MA_ERRCORR_NONE;
		return 0;
	case SYSID_SM_MEMARRAY_NUM_DEV:
		*val = 1;
		return 0;
	case SYSID_SM_MEMARRAY_MAXCAP:
		*val = priv->mem_max_cap_kb;
		return 0;
	/* Memory Device (Type 17) */
	case SYSID_SM_MEMDEV_TOTALWIDTH:
	case SYSID_SM_MEMDEV_DATAWIDTH:
		*val = 0xffff; /* Unknown */
		return 0;
	case SYSID_SM_MEMDEV_SIZE:
		*val = priv->mem_size_mb;
		return 0;
	case SYSID_SM_MEMDEV_EXTSIZE:
		*val = priv->mem_ext_size_kb;
		return 0;
	case SYSID_SM_MEMDEV_FORMFACTOR:
		*val = SMBIOS_MD_FF_ROC; /* soldered row of chips */
		return 0;
	case SYSID_SM_MEMDEV_TYPE:
		*val = soc->mem_type;
		return 0;
	case SYSID_SM_MEMDEV_TYPEDETAIL:
		*val = SMBIOS_MD_TD_SYNC;
		return 0;
	case SYSID_SM_MEMDEV_SPEED:
		*val = soc->mem_speed;
		return 0;
	/* Memory Array Mapped Address (Type 19) */
	case SYSID_SM_MEMMAP_START:
		*val = priv->mem_map_start_kb;
		return 0;
	case SYSID_SM_MEMMAP_END:
		*val = priv->mem_map_end_kb;
		return 0;
	case SYSID_SM_MEMMAP_PARTWIDTH:
		*val = 1;
		return 0;
	default:
		return rpi_sysinfo_cache_int(priv, id, val);
	}
}

static int rpi_sysinfo_get_data(struct udevice *dev, int id, void **datap,
				size_t *sizep)
{
	struct rpi_sysinfo_priv *priv = dev_get_priv(dev);

	switch (id) {
	case SYSID_SM_SYSTEM_UUID:
		*datap = priv->uuid;
		*sizep = sizeof(priv->uuid);
		return 0;
	case SYSID_SM_PROCESSOR_ID:
		*datap = priv->processor_id;
		*sizep = sizeof(priv->processor_id);
		return 0;
	case SYSID_SM_CACHE_HANDLE:
		/* lib/smbios.c stores the assigned Type 7 handles here */
		*datap = priv->cache_handles;
		*sizep = sizeof(priv->cache_handles);
		return 0;
	case SYSID_SM_MEMARRAY_HANDLE:
		/* lib/smbios.c stores the assigned Type 16 handles here */
		*datap = priv->marray_handles;
		*sizep = sizeof(priv->marray_handles);
		return 0;
	case SYSID_SM_MEMMAP_EXTSTART:
		*datap = &priv->mem_start;
		*sizep = sizeof(priv->mem_start);
		return 0;
	case SYSID_SM_MEMMAP_EXTEND:
		*datap = &priv->mem_end;
		*sizep = sizeof(priv->mem_end);
		return 0;
	default:
		return -ENOSYS;
	}
}

static const struct sysinfo_ops rpi_sysinfo_ops = {
	.detect = rpi_sysinfo_detect,
	.get_str = rpi_sysinfo_get_str,
	.get_int = rpi_sysinfo_get_int,
	.get_data = rpi_sysinfo_get_data,
};

U_BOOT_DRIVER(rpi_sysinfo) = {
	.name = "rpi_sysinfo",
	.id = UCLASS_SYSINFO,
	.ops = &rpi_sysinfo_ops,
	.priv_auto = sizeof(struct rpi_sysinfo_priv),
};

U_BOOT_DRVINFO(rpi_sysinfo) = {
	.name = "rpi_sysinfo",
};
