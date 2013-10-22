/*
 * ACPI support for Intel MID LPSS.
 *
 * Copyright (C) 2013, Intel Corporation
 * Authors: Jason Chen <jason.cj.chen@intel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/acpi.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/platform_device.h>

int acpi_create_platform_device(struct acpi_device *adev,
				const struct acpi_device_id *id);

ACPI_MODULE_NAME("acpi_mid_lpss");

static const struct acpi_device_id acpi_mid_lpss_device_ids[] = {
	/* MID LPSS I2C controller*/
	{ "80860F41" },
	{ }
};

static int acpi_mid_lpss_create_device(struct acpi_device *adev,
				   const struct acpi_device_id *id)
{
	return acpi_create_platform_device(adev, id);
}

static struct acpi_scan_handler mid_lpss_handler = {
	.ids = acpi_mid_lpss_device_ids,
	.attach = acpi_mid_lpss_create_device,
};

void __init acpi_mid_lpss_init(void)
{
	acpi_scan_add_handler(&mid_lpss_handler);
}