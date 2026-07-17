// SPDX-License-Identifier: GPL-2.0+
/*
 * I2C EEPROM interface for UEFI variables
 *
 * Non-volatile UEFI variables are persisted as a single serialized blob
 * (struct efi_var_file, the same format as the ubootefi.var file) at
 * offset 0 of the first UCLASS_I2C_EEPROM device.  Because the format is
 * self-describing (magic, length, CRC32), the same EEPROM can be read or
 * rewritten out-of-band - e.g. by a BMC acting as the I2C slave, or by the
 * OS through an at24-style driver - to inspect or modify BootOrder,
 * BootNext and friends.
 */

#define LOG_CATEGORY LOGC_EFI

#include <dm.h>
#include <efi_loader.h>
#include <efi_variable.h>
#include <i2c.h>
#include <i2c_eeprom.h>
#include <log.h>
#include <malloc.h>

static int efi_var_eeprom_get(struct udevice **devp)
{
	return uclass_first_device_err(UCLASS_I2C_EEPROM, devp);
}

/**
 * efi_var_to_file() - save non-volatile variables to the EEPROM
 *
 * The variable blob is written at offset 0 of the variable store EEPROM.
 * The write is skipped when the EEPROM already holds an identical blob so
 * that repeated SetVariable calls do not wear the device.
 *
 * Return:	status code
 */
efi_status_t efi_var_to_file(void)
{
	struct efi_var_file *buf = NULL;
	struct efi_var_file hdr;
	struct udevice *dev;
	efi_status_t ret;
	static bool once;
	loff_t len;
	int size;

	ret = efi_var_collect(&buf, &len, EFI_VARIABLE_NON_VOLATILE);
	if (ret != EFI_SUCCESS)
		goto error;

	if (efi_var_eeprom_get(&dev)) {
		if (!once) {
			log_warning("Cannot persist EFI variables without EEPROM\n");
			once = true;
		}
		ret = EFI_DEVICE_ERROR;
		goto out;
	}
	once = false;

	size = i2c_eeprom_size(dev);

	/*
	 * When the U-Boot environment shares this EEPROM (CONFIG_ENV_IS_IN_EEPROM
	 * at the same i2c address), it occupies [CONFIG_ENV_OFFSET, end) while the
	 * variable blob is written at offset 0.  Cap the usable size at
	 * CONFIG_ENV_OFFSET so the two regions can never overlap - the store is
	 * otherwise bounded only by the full chip size.
	 */
#if defined(CONFIG_ENV_IS_IN_EEPROM) && (CONFIG_ENV_OFFSET > 0)
	{
		struct dm_i2c_chip *chip = dev_get_parent_plat(dev);

		if (chip && chip->chip_addr == CONFIG_SYS_I2C_EEPROM_ADDR &&
		    (size <= 0 || size > CONFIG_ENV_OFFSET))
			size = CONFIG_ENV_OFFSET;
	}
#endif

	if (size > 0 && len > size) {
		log_err("EFI variables (%llu bytes) exceed store size (%d bytes)\n",
			(unsigned long long)len, size);
		ret = EFI_OUT_OF_RESOURCES;
		goto error;
	}

	/*
	 * Skip the write when the stored blob already matches; length and
	 * CRC32 cover the complete variable contents.
	 */
	if (!i2c_eeprom_read(dev, 0, (u8 *)&hdr, sizeof(hdr)) &&
	    hdr.magic == buf->magic && hdr.length == buf->length &&
	    hdr.crc32 == buf->crc32)
		goto out;

	if (i2c_eeprom_write(dev, 0, (u8 *)buf, len))
		ret = EFI_DEVICE_ERROR;

error:
	if (ret != EFI_SUCCESS)
		log_err("Failed to persist EFI variables\n");
out:
	free(buf);
	return ret;
}

/**
 * efi_var_from_file() - read variables from the EEPROM
 *
 * The variable blob is read from offset 0 of the variable store EEPROM and
 * the variables it contains are created.
 *
 * On first boot the EEPROM is blank.  This is why we must return
 * EFI_SUCCESS in this case.
 *
 * If the blob is corrupted, e.g. incorrect CRC32, we do not want to stop
 * the boot process.  We deliberately return EFI_SUCCESS in this case, too.
 *
 * Return:	status code
 */
efi_status_t efi_var_from_file(void)
{
	struct efi_var_file *buf;
	struct udevice *dev;

	buf = calloc(1, EFI_VAR_BUF_SIZE);
	if (!buf) {
		log_err("Out of memory\n");
		return EFI_OUT_OF_RESOURCES;
	}

	if (efi_var_eeprom_get(&dev)) {
		log_info("No EFI variable EEPROM, no variables loaded\n");
		goto out;
	}

	if (i2c_eeprom_read(dev, 0, (u8 *)buf, sizeof(*buf))) {
		log_info("No EFI variables loaded\n");
		goto out;
	}

	if (buf->magic != EFI_VAR_FILE_MAGIC ||
	    buf->length < sizeof(*buf) || buf->length > EFI_VAR_BUF_SIZE) {
		log_info("No EFI variables found in EEPROM\n");
		goto out;
	}

	if (buf->length > sizeof(*buf) &&
	    i2c_eeprom_read(dev, sizeof(*buf), (u8 *)buf + sizeof(*buf),
			    buf->length - sizeof(*buf))) {
		log_err("Reading EFI variables from EEPROM failed\n");
		goto out;
	}

	if (efi_var_restore(buf, false) != EFI_SUCCESS)
		log_err("Invalid EFI variables in EEPROM\n");
out:
	free(buf);
	return EFI_SUCCESS;
}
