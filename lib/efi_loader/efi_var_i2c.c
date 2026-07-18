// SPDX-License-Identifier: GPL-2.0+
/*
 * I2C EEPROM interface for UEFI variables
 *
 * Non-volatile UEFI variables are persisted as a single serialized blob
 * (struct efi_var_file, the same format as the ubootefi.var file) at offset 0
 * of the first UCLASS_I2C_EEPROM device.  Because the format is self-describing
 * (magic, length, CRC32), the same EEPROM can be read or rewritten out-of-band
 * - e.g. by a BMC acting as the I2C slave, or by the OS through an at24-style
 * driver - to inspect or modify BootOrder, BootNext and friends.
 *
 * This mirrors efi_var_sf.c (the SPI-flash backend); the differences are all
 * intrinsic to the medium:
 *   - the EEPROM is byte-writable, so there is no erase step;
 *   - the blob lives at offset 0 and the store may share the chip with the
 *     U-Boot environment, so its size is capped below CONFIG_ENV_OFFSET;
 *   - a matching blob is not rewritten, to spare EEPROM wear (and to avoid
 *     poking a BMC-emulated slave) on repeated SetVariable calls;
 *   - efi_var_from_storage() never fails the boot: efi_init_variables() treats
 *     a non-EFI_SUCCESS return as fatal, and the EEPROM - or the BMC behind it
 *     - may legitimately be absent or blank.  It also reads only the bytes the
 *     blob declares rather than the whole EFI_VAR_BUF_SIZE, as the SPI-flash
 *     backend does, because I2C is slow and the store is comparatively tiny.
 */

#define LOG_CATEGORY LOGC_EFI

#include <efi_loader.h>
#include <efi_variable.h>
#include <log.h>
#include <malloc.h>
#include <i2c.h>
#include <i2c_eeprom.h>
#include <dm.h>

/*
 * efi_var_i2c_store_size() - usable size of the variable store, in bytes
 *
 * When the U-Boot environment shares this EEPROM (CONFIG_ENV_IS_IN_EEPROM at
 * the same i2c address) it occupies [CONFIG_ENV_OFFSET, end), so cap the store
 * at CONFIG_ENV_OFFSET so the blob at offset 0 can never overlap it.  Returns
 * <= 0 if the size cannot be determined.
 */
static int efi_var_i2c_store_size(struct udevice *dev)
{
	int size = i2c_eeprom_size(dev);

#if defined(CONFIG_ENV_IS_IN_EEPROM) && (CONFIG_ENV_OFFSET > 0)
	struct dm_i2c_chip *chip = dev_get_parent_plat(dev);

	if (chip && chip->chip_addr == CONFIG_SYS_I2C_EEPROM_ADDR &&
	    (size <= 0 || size > CONFIG_ENV_OFFSET))
		size = CONFIG_ENV_OFFSET;
#endif
	return size;
}

efi_status_t efi_var_to_storage(void)
{
	struct efi_var_file *buf;
	struct efi_var_file hdr;
	struct udevice *eedev;
	efi_status_t ret;
	loff_t len;
	int size;
	int r;

	ret = efi_var_collect(&buf, &len, EFI_VARIABLE_NON_VOLATILE);
	if (ret != EFI_SUCCESS)
		goto error;

	r = uclass_first_device_err(UCLASS_I2C_EEPROM, &eedev);
	if (r) {
		log_debug("Failed to get I2C EEPROM device: %d\n", r);
		ret = EFI_DEVICE_ERROR;
		goto error;
	}

	size = efi_var_i2c_store_size(eedev);
	if (size > 0 && len > size) {
		log_debug("EFI var buffer length more than target EEPROM size\n");
		ret = EFI_OUT_OF_RESOURCES;
		goto error;
	}

	log_debug("Got buffer to write buf->len: %d\n", buf->length);

	/* Skip the write when the stored blob already matches (see header). */
	if (!i2c_eeprom_read(eedev, 0, (u8 *)&hdr, sizeof(hdr)) &&
	    hdr.magic == buf->magic && hdr.length == buf->length &&
	    hdr.crc32 == buf->crc32)
		goto error;

	r = i2c_eeprom_write(eedev, 0, (u8 *)buf, len);
	if (r) {
		log_debug("Failed to write to I2C EEPROM: %d\n", r);
		ret = EFI_DEVICE_ERROR;
	}

error:
	free(buf);
	return ret;
}

efi_status_t efi_var_from_storage(void)
{
	struct efi_var_file *buf;
	struct udevice *eedev;
	int r;

	buf = calloc(1, EFI_VAR_BUF_SIZE);
	if (!buf) {
		log_err("Unable to allocate buffer\n");
		return EFI_OUT_OF_RESOURCES;
	}

	r = uclass_first_device_err(UCLASS_I2C_EEPROM, &eedev);
	if (r) {
		log_debug("No I2C EEPROM device, no EFI variables loaded\n");
		goto out;
	}

	/* Read the fixed header, validate it, then read only the declared blob. */
	if (i2c_eeprom_read(eedev, 0, (u8 *)buf, sizeof(*buf))) {
		log_debug("Failed to read from I2C EEPROM\n");
		goto out;
	}

	if (buf->magic != EFI_VAR_FILE_MAGIC ||
	    buf->length < sizeof(*buf) || buf->length > EFI_VAR_BUF_SIZE) {
		log_debug("No EFI variables found in I2C EEPROM\n");
		goto out;
	}

	if (buf->length > sizeof(*buf) &&
	    i2c_eeprom_read(eedev, sizeof(*buf), (u8 *)buf + sizeof(*buf),
			    buf->length - sizeof(*buf))) {
		log_debug("Failed to read from I2C EEPROM\n");
		goto out;
	}

	if (efi_var_restore(buf, false) != EFI_SUCCESS)
		log_err("No valid EFI variables in I2C EEPROM\n");
out:
	free(buf);
	return EFI_SUCCESS;
}
