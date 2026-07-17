// SPDX-License-Identifier: GPL-2.0
/*
 * (C) Copyright 2012 Stephen Warren
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 */

#include <config.h>
#include <command.h>
#include <cpu_func.h>
#include <asm/io.h>
#include <asm/arch/base.h>
#include <asm/arch/wdog.h>
#include <asm/global_data.h>
#include <efi_loader.h>
#include <fdt_support.h>
#include <sysreset.h>
#include <linux/psci.h>

DECLARE_GLOBAL_DATA_PTR;

/*
 * BCM2712 (RPi 5 family) has no BCM2835-style PM/WDOG block; reset and
 * poweroff are serviced by PSCI (TF-A in the EEPROM). Older BCM283x SoCs
 * still use the watchdog/RSTS path below. Builds like rpi_arm64_defconfig
 * may target both SoC families in a single binary, so dispatch at runtime
 * based on the firmware-provided FDT.
 */
static bool rpi_is_bcm2712(void)
{
	return IS_ENABLED(CONFIG_ARM_PSCI_FW) &&
	       gd && gd->fdt_blob &&
	       !fdt_node_check_compatible(gd->fdt_blob, 0, "brcm,bcm2712");
}

#define RESET_TIMEOUT 10

/*
 * The Raspberry Pi firmware uses the RSTS register to know which partiton
 * to boot from. The partiton value is spread into bits 0, 2, 4, 6, 8, 10.
 * Partiton 63 is a special partition used by the firmware to indicate halt.
 */
#define BCM2835_WDOG_RSTS_RASPBERRYPI_HALT	0x555

/* max ticks timeout */
#define BCM2835_WDOG_MAX_TIMEOUT	0x000fffff

__efi_runtime_data struct bcm2835_wdog_regs *wdog_regs;
/* Latched at efi_reset_system_init() time so we don't need gd at runtime. */
static __efi_runtime_data bool efi_use_psci;

static void __efi_runtime
__reset_cpu(struct bcm2835_wdog_regs *wdog_regs, ulong ticks)
{
	uint32_t rstc, timeout;

	if (ticks == 0)
		timeout = RESET_TIMEOUT;
	else
		timeout = ticks & BCM2835_WDOG_MAX_TIMEOUT;

	rstc = readl(&wdog_regs->rstc);
	rstc &= ~BCM2835_WDOG_RSTC_WRCFG_MASK;
	rstc |= BCM2835_WDOG_RSTC_WRCFG_FULL_RESET;

	writel(BCM2835_WDOG_PASSWORD | timeout, &wdog_regs->wdog);
	writel(BCM2835_WDOG_PASSWORD | rstc, &wdog_regs->rstc);
}

void reset_cpu(void)
{
	struct bcm2835_wdog_regs *regs =
		(struct bcm2835_wdog_regs *)BCM2835_WDOG_PHYSADDR;

	if (rpi_is_bcm2712()) {
		psci_sys_reset(SYSRESET_COLD);
		/* PSCI should not return; fall through to wdog as a safety net. */
	}

	__reset_cpu(regs, 0);
}

/*
 * `do_poweroff` is provided by drivers/firmware/psci.c when ARM_PSCI_FW is
 * enabled: it calls PSCI SYSTEM_OFF, which TF-A on BCM2712 forwards to the
 * EEPROM/PMIC sequence (honoring POWER_OFF_ON_HALT). On non-BCM2712 SoCs in
 * a combined build PSCI isn't bound and the psci.c path returns failure;
 * legacy BCM2835 wdog-RSTS-partition-63 poweroff is no longer wired up for
 * those (RPi 3/4 standalone builds don't enable CMD_POWEROFF).
 */

#ifdef CONFIG_EFI_LOADER

void __efi_runtime EFIAPI efi_reset_system(
			enum efi_reset_type reset_type,
			efi_status_t reset_status,
			unsigned long data_size, void *reset_data)
{
	u32 val;

	if (reset_type == EFI_RESET_COLD ||
	    reset_type == EFI_RESET_WARM ||
	    reset_type == EFI_RESET_PLATFORM_SPECIFIC) {
		if (efi_use_psci)
			psci_sys_reset(SYSRESET_COLD);
		else
			__reset_cpu(wdog_regs, 0);
	} else if (reset_type == EFI_RESET_SHUTDOWN) {
		if (efi_use_psci) {
			psci_sys_poweroff();
		} else {
			/*
			 * We set the watchdog hard reset bit here to distinguish this reset
			 * from the normal (full) reset. bootcode.bin will not reboot after a
			 * hard reset.
			 */
			val = readl(&wdog_regs->rsts);
			val |= BCM2835_WDOG_PASSWORD;
			val |= BCM2835_WDOG_RSTS_RASPBERRYPI_HALT;
			writel(val, &wdog_regs->rsts);
			__reset_cpu(wdog_regs, 0);
		}
	}

	while (1) { }
}

efi_status_t efi_reset_system_init(void)
{
	efi_use_psci = rpi_is_bcm2712();
	wdog_regs = (struct bcm2835_wdog_regs *)BCM2835_WDOG_PHYSADDR;
	return efi_add_runtime_mmio(&wdog_regs, sizeof(*wdog_regs));
}

#endif
