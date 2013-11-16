/*
 * Copyright (C) 2013 Intel Corp
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/efi.h>
#include <linux/ucs2_string.h>
#include <linux/nls.h>

#include "reboot_target.h"

static const char TARGET_VARNAME[]     = "BootNext";
static const char LOAD_OPTION_FORMAT[] = "Boot%04X";

static bool uefi_load_option_is_present(const u16 id)
{
	char varname8[sizeof(LOAD_OPTION_FORMAT)];
	wchar_t varname16[sizeof(LOAD_OPTION_FORMAT)];
	bool found = false;

	sprintf(varname8, LOAD_OPTION_FORMAT, id);
	utf8s_to_utf16s(varname8, sizeof(varname8),
			UTF16_LITTLE_ENDIAN, varname16, sizeof(varname16));
	varname16[sizeof(LOAD_OPTION_FORMAT) - 1] = 0;

	efivar_entry_iter_begin();
	if (efivar_entry_find(varname16, EFI_GLOBAL_VARIABLE_GUID,
			      &efivar_sysfs_list, false))
		found = true;
	efivar_entry_iter_end();

	return found;
}

static int uefi_set_reboot_target(const char *name, const int id)
{
	wchar_t varname[sizeof(TARGET_VARNAME)];
	u32 attributes = EFI_VARIABLE_NON_VOLATILE
		| EFI_VARIABLE_BOOTSERVICE_ACCESS
		| EFI_VARIABLE_RUNTIME_ACCESS;
	u16 target_id = (0x1 << 8) | (id & 0xFF);

	if (!uefi_load_option_is_present(target_id)) {
		pr_err("%s: Load option %04X not found", __func__, target_id);
		return EINVAL;
	}

	utf8s_to_utf16s(TARGET_VARNAME, sizeof(TARGET_VARNAME),
			UTF16_LITTLE_ENDIAN, varname, sizeof(varname));
	varname[sizeof(TARGET_VARNAME) - 1] = 0;

	return efivar_entry_set_safe(varname, EFI_GLOBAL_VARIABLE_GUID,
				     attributes, true, sizeof(u16), &target_id);
}

struct reboot_target reboot_target_uefi = {
	.set_reboot_target = uefi_set_reboot_target,
};

static int reboot_target_uefi_probe(struct platform_device *pdev)
{
	return reboot_target_register(&reboot_target_uefi);
}

static int reboot_target_uefi_remove(struct platform_device *pdev)
{
	return reboot_target_unregister(&reboot_target_uefi);
}

struct platform_driver reboot_target_uefi_driver = {
	.probe = reboot_target_uefi_probe,
	.remove = reboot_target_uefi_remove,
	.driver.name = KBUILD_MODNAME,
	.driver.owner = THIS_MODULE,
};

static struct platform_device *uefi_pdev;

static int __init reboot_target_uefi_init(void)
{
	if (efi_enabled(EFI_BOOT) && efi_enabled(EFI_RUNTIME_SERVICES)) {
		 uefi_pdev = platform_device_register_simple(KBUILD_MODNAME, -1,
							     NULL, 0);
		 if (IS_ERR(uefi_pdev))
			 return PTR_ERR(uefi_pdev);
	} else
		return -ENODEV;

	return platform_driver_register(&reboot_target_uefi_driver);
}

static void __exit reboot_target_uefi_exit(void)
{
	platform_device_unregister(uefi_pdev);
	platform_driver_unregister(&reboot_target_uefi_driver);
}

module_init(reboot_target_uefi_init);
module_exit(reboot_target_uefi_exit);

MODULE_AUTHOR("Jeremy Compostella <jeremy.compostella@intel.com>");
MODULE_DESCRIPTION("Intel Reboot Target UEFI implementation");
MODULE_LICENSE("GPL v2");
