/**
 * linux/modules/drivers/modem_control/imc_6x6x.c
 *
 * Version 1.0
 *
 * This code includes power sequences for IMC 6260 modems and its derivatives.
 * That includes:
 * - XMM6268
 * There is no guarantee for other modems.
 *
 * Intel Mobile driver for modem powering.
 *
 * Copyright (C) 2013 Intel Corporation. All rights reserved.
 *
 * Contact: Faouaz Tenoutit <faouazx.tenoutit@intel.com>
 *          Frederic Berat <fredericx.berat@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */

#include "mdm_util.h"
#include "mdm_imc.h"

#include <linux/mdm_ctrl.h>

/*****************************************************************************
 *
 * Modem Power/Reset functions
 *
 ****************************************************************************/

/**
 *  mdm_ctrl_cold_boot_6x6x - Perform a modem cold boot sequence
 *  @drv: Reference to the driver structure
 *
 *  - Set to HIGH the PWRDWN_N to switch ON the modem
 *  - Set to HIGH the RESET_BB_N
 *  - Do a pulse on ON1
 *   usleep_range are used in order to avoid spinning the cpu
 */
int mdm_ctrl_cold_boot_6x6x(struct mdm_ctrl *drv)
{
	struct mdm_ctrl_pdata *pdata = drv->pdata;
	struct mdm_ctrl_device_info *mid_info =
		(struct mdm_ctrl_device_info *)pdata->device_data;

	int ret = 0;
	u16 addr = pdata->chipctrl;
	u8 data;
	u8 def_value = 0x00;
	unsigned long flags;

	pr_warn(DRVNAME": Cold boot requested");

	/* Set the current modem state */
	mdm_ctrl_launch_work(drv, MDM_CTRL_STATE_COLD_BOOT);
	flush_workqueue(drv->change_state_wq);

	/* Based on chipctrl_mask value we decide if we need to read
	 * the current register value.
	 */
	if (pdata->chipctrl_mask) {
		/* Save the current register value */
		ret = intel_scu_ipc_readv(&addr, &def_value, 1);
		if (ret) {
			pr_err(DRVNAME": ipc_readv() failed (ret: %d)", ret);
			goto out;
		}
	}

	/* Write the new register value (CHIPCNTRL_ON) */
	data = (def_value & pdata->chipctrl_mask) | pdata->chipctrlon;
	ret =  intel_scu_ipc_writev(&addr, &data, 1);
	if (ret) {
		pr_err(DRVNAME": ipc_writev(ON) failed (ret: %d)", ret);
		goto out;
	}

	/* Restore previously saved register value if any */
	if (pdata->chipctrl_mask) {
		ret =  intel_scu_ipc_writev(&addr, &def_value, 1);
		if (ret) {
			pr_err(DRVNAME": ipc_writev() failed (ret: %d)", ret);
			goto out;
		}
	}

	/* AP request => just ignore the modem reset */
	spin_lock_irqsave(&drv->state_lck, flags);
	mdm_ctrl_set_reset_ongoing(drv, 1);
	spin_unlock_irqrestore(&drv->state_lck, flags);

	/* Toggle the RESET_BB_N */
	gpio_set_value(drv->gpio_rst_bbn, 1);

	/* Wait before doing the pulse on ON1 */
	usleep_range(mid_info->pre_on_delay, mid_info->pre_on_delay);

	/* Do a pulse on ON1 */
	gpio_set_value(drv->gpio_pwr_on, 1);
	usleep_range(mid_info->on_duration, mid_info->on_duration);
	gpio_set_value(drv->gpio_pwr_on, 0);

	mdm_ctrl_launch_timer(&drv->flashing_timer,
			mid_info->pre_wflash_delay,
			MDM_TIMER_FLASH_ENABLE);
out:
	return ret;
}

/**
 *  mdm_ctrl_cold_reset_6x6x - Perform a modem cold reset
 *  @drv: Reference to the driver structure
 *
 *  - Set the RESET_BB_N to low (better SIM protection)
 *  - Set the EXT1P35VREN field to low  during 20ms (CHIPCNTRL PMIC register)
 *  - set the EXT1P35VREN field to high during 10ms (CHIPCNTRL PMIC register)
 *  usleep_range are used in order to avoid spinning the cpu
 */
int mdm_ctrl_cold_reset_6x6x(struct mdm_ctrl *drv)
{
	struct mdm_ctrl_pdata *pdata = drv->pdata;
	struct mdm_ctrl_device_info *mid_info =
		(struct mdm_ctrl_device_info *)pdata->device_data;

	u16 addr = pdata->chipctrl;
	u8 data;
	u8 def_value = 0x00;
	int ret = 0;
	unsigned long flags;

	pr_info(DRVNAME": Cold reset requested");

	/* Based on chipctrl_mask value we decide if we need to read
	 * the current register value.
	 */
	if (pdata->chipctrl_mask) {
		/* Save the current register value */
		ret = intel_scu_ipc_readv(&addr, &def_value, 1);
		if (ret) {
			pr_err(DRVNAME ": ipc_readv() failed (ret: %d)", ret);
			goto out;
		}
	}

	/* AP requested reset => ignore reset IRQ */
	spin_lock_irqsave(&drv->state_lck, flags);
	mdm_ctrl_set_reset_ongoing(drv, 1);
	spin_unlock_irqrestore(&drv->state_lck, flags);

	/* Set the current modem state */
	mdm_ctrl_launch_work(drv, MDM_CTRL_STATE_COLD_BOOT);
	flush_workqueue(drv->change_state_wq);

	/* Set the reset_bb to low */
	gpio_set_value(drv->gpio_rst_bbn, 0);
	usleep_range(pdata->pre_pwr_down_delay, pdata->pre_pwr_down_delay);

	/* Write the new register value (CHIPCNTRL_OFF) */
	data = (def_value & pdata->chipctrl_mask) | pdata->chipctrloff;
	ret =  intel_scu_ipc_writev(&addr, &data, 1);
	if (ret) {
		pr_err(DRVNAME ": ipc_writev(OFF) failed (ret: %d)", ret);
		goto out;
	}
	usleep_range(pdata->pwr_down_duration, pdata->pwr_down_duration);

	/* Write the new register value (CHIPCNTRL_ON) */
	data = (def_value & pdata->chipctrl_mask) | pdata->chipctrlon;
	ret =  intel_scu_ipc_writev(&addr, &data, 1);
	if (ret) {
		pr_err(DRVNAME ": ipc_writev(ON) failed (ret: %d)", ret);
		goto out;
	}

	if (pdata->chipctrl_mask) {
		/* Write back the saved register value */
		ret =  intel_scu_ipc_writev(&addr, &def_value, 1);

		if (ret) {
			pr_err(DRVNAME ": ipc_writev() failed (ret: %d)", ret);
			goto out;
		}
	}

	/* Set the RESET_BB_N to high */
	gpio_set_value(drv->gpio_rst_bbn, 1);

	usleep_range(mid_info->pre_on_delay, mid_info->pre_on_delay);

	/* Assert, wait & de-assert the ON1 */
	gpio_set_value(drv->gpio_pwr_on, 1);
	usleep_range(mid_info->on_duration, mid_info->on_duration);
	gpio_set_value(drv->gpio_pwr_on, 0);

	mdm_ctrl_launch_timer(&drv->flashing_timer,
			mid_info->pre_wflash_delay,
			MDM_TIMER_FLASH_ENABLE);
out:
	return ret;
}

/**
 *  mdm_ctrl_silent_warm_reset_6x6x - Perform a silent modem warm reset
 *				      sequence
 *  @drv: Reference to the driver structure
 *
 *  - Do a pulse on the RESET_BB_N
 *  - No struct modification
 *  - debug purpose only
 *   usleep_range are used in order to avoid spinning the cpu
 */
int mdm_ctrl_silent_warm_reset_6x6x(struct mdm_ctrl *drv)
{
	struct mdm_ctrl_device_info *mid_info = drv->pdata->device_data;

	gpio_set_value(drv->gpio_rst_bbn, 0);
	usleep_range(mid_info->warm_rst_duration, mid_info->warm_rst_duration);
	gpio_set_value(drv->gpio_rst_bbn, 1);

	return 0;
}

/**
 *  mdm_ctrl_normal_warm_reset_6x6x - Perform a normal modem warm reset
 *				      sequence
 *  @drv: Reference to the driver structure
 *
 *  - Do a pulse on the RESET_BB_N
 */
int mdm_ctrl_normal_warm_reset_6x6x(struct mdm_ctrl *drv)
{
	struct mdm_ctrl_device_info *mid_info = drv->pdata->device_data;

	unsigned long flags;

	pr_info(DRVNAME ": Normal warm reset requested\r\n");

	spin_lock_irqsave(&drv->state_lck, flags);
	/* AP requested reset => just ignore */
	mdm_ctrl_set_reset_ongoing(drv, 1);
	spin_unlock_irqrestore(&drv->state_lck, flags);

	/* Set the current modem state */
	mdm_ctrl_launch_work(drv, MDM_CTRL_STATE_WARM_BOOT);
	flush_workqueue(drv->change_state_wq);

	/* Now executing reset sequence */
	mdm_ctrl_silent_warm_reset_6x6x(drv);

	/* Launching timer to switch in FW_DOWNLOAD_READY state */
	mdm_ctrl_launch_timer(&drv->flashing_timer,
			mid_info->pre_wflash_delay,
			MDM_TIMER_FLASH_ENABLE);

	return 0;
}

/**
 *  mdm_ctrl_flashing_warm_reset_6x6x - Perform a normal modem warm reset
 *					sequence
 *  @drv: Reference to the driver structure
 *
 *  - Do a pulse on the RESET_BB_N
 *  - Wait before return
 *  - No switch to FW_DOWNLOAD_READY state
 */
int mdm_ctrl_flashing_warm_reset_6x6x(struct mdm_ctrl *drv)
{
	struct mdm_ctrl_device_info *mid_info = drv->pdata->device_data;

	unsigned long flags;

	pr_info(DRVNAME ": Flashing warm reset requested");

	spin_lock_irqsave(&drv->state_lck, flags);
	/* AP requested reset => just ignore */
	mdm_ctrl_set_reset_ongoing(drv, 1);
	spin_unlock_irqrestore(&drv->state_lck, flags);

	/* Set the current modem state */
	mdm_ctrl_launch_work(drv, MDM_CTRL_STATE_WARM_BOOT);
	flush_workqueue(drv->change_state_wq);

	/* Now executing reset sequence */
	mdm_ctrl_silent_warm_reset_6x6x(drv);

	/* Wait for wrm reset flash delay before return  */
	msleep(mid_info->pre_wflash_delay);

	return 0;
}

/**
 *  mdm_ctrl_power_off_6x6x - Perform the modem switch OFF sequence
 *  @drv: Reference to the driver structure
 *
 *  - Set to low the ON1
 *  - Write the PMIC reg
 */
int mdm_ctrl_power_off_6x6x(struct mdm_ctrl *drv)
{
	struct mdm_ctrl_pdata *pdata = drv->pdata;

	u16 addr = pdata->chipctrl;
	u8 data;
	u8 def_value = 0x00;
	int ret = 0;
	unsigned long flags;

	pr_info(DRVNAME ": Power OFF requested");

	spin_lock_irqsave(&drv->state_lck, flags);
	/* AP requested reset => just ignore */
	mdm_ctrl_set_reset_ongoing(drv, 1);
	spin_unlock_irqrestore(&drv->state_lck, flags);

	/* Set the modem state to OFF */
	mdm_ctrl_launch_work(drv, MDM_CTRL_STATE_OFF);
	flush_workqueue(drv->change_state_wq);

	/* Set to low the ON1 */
	gpio_set_value(drv->gpio_rst_bbn, 0);

	/* Based on chipctrl_mask value we decide if we need to read
	 * the current register value.
	 */
	if (pdata->chipctrl_mask) {
		/* Save the current register value */
		ret = intel_scu_ipc_readv(&addr, &def_value, 1);
		if (ret) {
			pr_err(DRVNAME ": ipc_readv() failed (ret: %d)", ret);
			goto out;
		}
	}

	/* Write the new register value (CHIPCNTRL_OFF) */
	data = (def_value & pdata->chipctrl_mask) | pdata->chipctrloff;
	ret =  intel_scu_ipc_writev(&addr, &data, 1);
	if (ret) {
		pr_err(DRVNAME ": ipc_writev(OFF)  failed (ret: %d)", ret);
		goto out;
	}

	/* Write back the saved register value */
	if (pdata->chipctrl_mask)
		ret =  intel_scu_ipc_writev(&addr, &def_value, 1);

	if (ret)
		pr_err(DRVNAME ": ipc_writev() failed (ret: %d)", ret);

out:
	return ret;
}

