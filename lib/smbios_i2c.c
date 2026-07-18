// SPDX-License-Identifier: GPL-2.0+
/*
 * Copy the generated SMBIOS tables into a region of an I2C EEPROM.
 *
 * The BMC reads the region out-of-band to report host inventory (product,
 * version, serial, UUID, CPU, cache) over Redfish/IPMI while the host is off.
 * It carries strictly more than the U-Boot environment does - the type 1 UUID
 * in particular exists nowhere else.
 *
 * The blob written is exactly what write_smbios_table() produced: the _SM3_
 * entry point followed by the structure table. It is self-describing (the
 * anchor holds the table length), so standard SMBIOS tooling can parse it
 * straight out of the EEPROM. Note the anchor's struct_table_address field is
 * the DRAM address the tables were generated at and is meaningless to a
 * reader of the EEPROM; the tables always begin at
 * ALIGN(entry.length, 16) bytes into the region (see write_smbios_table()).
 *
 * This shares the EEPROM with the UEFI variable store and the environment:
 *
 *	0x0000..0x3fff  UEFI variable blob (CONFIG_EFI_VARIABLE_I2C_STORE)
 *	0x4000..0x5fff  U-Boot environment (CONFIG_ENV_IS_IN_EEPROM)
 *	0x6000..        SMBIOS tables      (CONFIG_SMBIOS_I2C_STORE_OFFSET)
 */

#define LOG_CATEGORY LOGC_BOARD

#include <dm.h>
#include <i2c_eeprom.h>
#include <log.h>
#include <malloc.h>
#include <mapmem.h>
#include <smbios.h>
#include <linux/errno.h>

int smbios_store_i2c(ulong addr, size_t len)
{
	struct udevice *dev;
	void *tables;
	u8 *stored;
	int ret;

	if (!len)
		return 0;

	if (len > CONFIG_SMBIOS_I2C_STORE_SIZE) {
		log_err("SMBIOS tables (%zu bytes) exceed the EEPROM region (%d bytes)\n",
			len, CONFIG_SMBIOS_I2C_STORE_SIZE);
		return -ENOSPC;
	}

	/*
	 * The same device as the UEFI variable store and the environment - one
	 * EEPROM, three regions - so this must not be a separate DT node.
	 */
	ret = uclass_first_device_err(UCLASS_I2C_EEPROM, &dev);
	if (ret) {
		log_debug("No EEPROM for the SMBIOS tables: %d\n", ret);
		return ret;
	}

	tables = map_sysmem(addr, len);

	/*
	 * Skip the write when the EEPROM already holds these exact bytes. The
	 * tables only change across a U-Boot update or a serial#/DT change,
	 * while i2c_eeprom_write() sleeps 10ms per page - so the common boot
	 * costs one read instead of a pointless rewrite.
	 */
	stored = malloc(len);
	if (stored) {
		bool same = !i2c_eeprom_read(dev, CONFIG_SMBIOS_I2C_STORE_OFFSET,
					     stored, len) &&
			    !memcmp(stored, tables, len);

		free(stored);
		if (same) {
			log_debug("SMBIOS tables in EEPROM are up to date\n");
			unmap_sysmem(tables);
			return 0;
		}
	}

	ret = i2c_eeprom_write(dev, CONFIG_SMBIOS_I2C_STORE_OFFSET, tables, len);
	unmap_sysmem(tables);
	if (ret) {
		log_err("Failed to write the SMBIOS tables to EEPROM: %d\n", ret);
		return ret;
	}

	log_debug("SMBIOS: wrote %zu bytes at EEPROM offset %#x\n", len,
		  (uint)CONFIG_SMBIOS_I2C_STORE_OFFSET);

	return 0;
}
