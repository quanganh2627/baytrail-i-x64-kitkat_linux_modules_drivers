/*
 * linux/drivers/modem_control/mdm_util.c
 *
 * Version 1.0
 *
 * Utilities for modem control driver.
 *
 * Copyright (C) 2012 Intel Corporation. All rights reserved.
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

/* Modem control driver instance */
struct mdm_ctrl *mdm_drv;

/**
 *  mdm_ctrl_set_opened - Set the open device state
 *  @drv: Reference to the driver structure
 *  @value: Value to set
 *
 */
inline void mdm_ctrl_set_opened(struct mdm_ctrl *drv, int value)
{
	/* Set the open flag */
	drv->opened = value;
}

/**
 *  mdm_ctrl_get_opened - Return the device open state
 *  @drv: Reference to the driver structure
 *
 */
inline int mdm_ctrl_get_opened(struct mdm_ctrl *drv)
{
	int opened;

	/* Set the open flag */
	opened = drv->opened;
	return opened;
}

/**
 *  mdm_ctrl_launch_work - Launch work to change modem state
 *  @drv: Reference to the driver structure
 *  @state: New state
 *
 *  Defer and queue change of state in order to avoid race condition
 *  with IRQ and timer expiration management.
 */
inline void mdm_ctrl_launch_work(struct mdm_ctrl *drv, int state)
{
	struct next_state *new_state;
	unsigned long flags;

	new_state = kzalloc(sizeof(struct next_state), GFP_ATOMIC);
	if (!new_state) {
		pr_err(DRVNAME": Can't allocate new_state memory");
		return;
	}
	new_state->state = state;

	spin_lock_irqsave(&drv->state_lck, flags);
	list_add_tail(&new_state->link, &drv->next_state_link);
	queue_work(drv->change_state_wq, &drv->change_state_work);
	spin_unlock_irqrestore(&drv->state_lck, flags);


}

/**
 *  mdm_ctrl_set_state -  Effectively sets the modem state on work execution
 *  @work: Reference to the work structure
 *
 */
inline void mdm_ctrl_set_state(struct work_struct *work)
{
	struct mdm_ctrl *drv;
	struct next_state *new_state;
	unsigned long flags;

	drv = container_of(work, struct mdm_ctrl, change_state_work);

	/* List can have several elements */
	while (!list_empty_careful(&drv->next_state_link)) {
		spin_lock_irqsave(&drv->state_lck, flags);
		new_state = list_first_entry(&drv->next_state_link,
				struct next_state, link);
		list_del_init(&new_state->link);
		spin_unlock_irqrestore(&drv->state_lck, flags);

		/* Set the current modem state */
		drv->modem_state = new_state->state;
		if (likely(drv->modem_state != MDM_CTRL_STATE_UNKNOWN) &&
				(drv->modem_state & drv->polled_states)) {

			drv->polled_state_reached = true;
			/* Waking up the poll work queue */
			wake_up(&drv->wait_wq);
			pr_info(DRVNAME": Waking up polling 0x%x\r\n",
					drv->modem_state);

		}
		/* Waking up the wait_for_state work queue */
		wake_up(&drv->event);
		kfree(new_state);
	}
}

/**
 *  mdm_ctrl_get_state - Return the local current modem state
 *  @drv: Reference to the driver structure
 *
 *  Note: Real current state may be different in case of self-reset
 *	  or if user has manually changed the state.
 */
inline int mdm_ctrl_get_state(struct mdm_ctrl *drv)
{
	return drv->modem_state;
}

/**
 *  mdm_ctrl_enable_flashing - Set the modem state to FW_DOWNLOAD_READY
 *
 */
void mdm_ctrl_enable_flashing(unsigned long int param)
{
	struct mdm_ctrl *drv = (struct mdm_ctrl *) param;

	del_timer(&drv->flashing_timer);
	if (mdm_ctrl_get_state(drv) != MDM_CTRL_STATE_IPC_READY) {
		mdm_ctrl_launch_work(drv, MDM_CTRL_STATE_FW_DOWNLOAD_READY);
	}
}

/**
 *  mdm_ctrl_launch_timer - Timer launcher helper
 *  @timer: Timer to activate
 *  @delay: Timer duration
 *  @timer_type: Timer type
 *
 *  Type can be MDM_TIMER_FLASH_ENABLE.
 *  Note: Type MDM_TIMER_FLASH_DISABLE is not used anymore.
 */
void mdm_ctrl_launch_timer(struct timer_list *timer, int delay,
		unsigned int timer_type)
{
	timer->data = (unsigned long int) mdm_drv;
	switch (timer_type) {
	case MDM_TIMER_FLASH_ENABLE:
		timer->function = mdm_ctrl_enable_flashing;
		break;
	case MDM_TIMER_FLASH_DISABLE:
	default:
		pr_err(DRVNAME": Unrecognized timer type %d", timer_type);
		del_timer(timer);
		return;
		break;
	}
	mod_timer(timer, jiffies + msecs_to_jiffies(delay));
}

/**
 *  mdm_ctrl_set_reset_ongoing - Set the RESET ongoing flag value
 *  @drv: Reference to the driver structure
 *  @ongoing: Flag value to set
 *
 */
inline void mdm_ctrl_set_reset_ongoing(struct mdm_ctrl *drv, int ongoing)
{
	drv->rst_ongoing = ongoing;
}

/**
 *  mdm_ctrl_get_reset_ongoing - Return the RESET ongoing flag value
 *  @drv: Reference to the driver structure
 *
 */
inline int mdm_ctrl_get_reset_ongoing(struct mdm_ctrl *drv)
{

	return drv->rst_ongoing;
}

/**
 *  mdm_ctrl_set_gpio - Move the gpio value to simplify access
 *  @drv: Reference to the driver structure
 *
 */
void mdm_ctrl_set_gpio(struct mdm_ctrl *drv)
{
	struct mdm_ctrl_pdata *pdata = drv->pdata;

	drv->gpio_rst_out = pdata->cpu_data->gpio_rst_out;
	drv->gpio_pwr_on  = pdata->cpu_data->gpio_pwr_on;
	drv->gpio_rst_bbn = pdata->cpu_data->gpio_rst_bbn;
	drv->gpio_cdump   = pdata->cpu_data->gpio_cdump;
}

/**
 *  mdm_ctrl_set_func - Set modem sequences functions to use
 *  @drv: Reference to the driver structure
 *
 */
void mdm_ctrl_set_func(struct mdm_ctrl *drv)
{
	int modem_type = 0;

	if (drv->pdata)
		modem_type = drv->pdata->modem;
	else {
		pr_info(DRVNAME": No modem data available");
		return;
	}

	pr_info(DRVNAME": Taking %d modem sequences", modem_type);

	/* There should be on modem file per provider at least */
	switch (modem_type) {
	case MODEM_6260:
	case MODEM_6268:
		drv->mdm_ctrl_cold_boot = mdm_ctrl_cold_boot_6x6x;
		drv->mdm_ctrl_cold_reset = mdm_ctrl_cold_reset_6x6x;
		drv->mdm_ctrl_normal_warm_reset =
			mdm_ctrl_normal_warm_reset_6x6x;
		drv->mdm_ctrl_silent_warm_reset =
			mdm_ctrl_silent_warm_reset_6x6x;
		drv->mdm_ctrl_flashing_warm_reset =
			mdm_ctrl_flashing_warm_reset_6x6x;
		drv->mdm_ctrl_power_off = mdm_ctrl_power_off_6x6x;
		break;
	case MODEM_6360:
	case MODEM_7160:
	case MODEM_7260:
		drv->mdm_ctrl_cold_boot = mdm_ctrl_cold_boot_7x6x;
		drv->mdm_ctrl_cold_reset = mdm_ctrl_cold_reset_7x6x;
		drv->mdm_ctrl_normal_warm_reset =
			mdm_ctrl_normal_warm_reset_7x6x;
		drv->mdm_ctrl_silent_warm_reset =
			mdm_ctrl_silent_warm_reset_7x6x;
		drv->mdm_ctrl_flashing_warm_reset =
			mdm_ctrl_flashing_warm_reset_7x6x;
		drv->mdm_ctrl_power_off = mdm_ctrl_power_off_7x6x;
		break;
	default:
		pr_info(DRVNAME": Can't retrieve modem specific functions");
		drv->pdata->is_mdm_ctrl_disabled = true;
		break;
	}
}

/**
 *  modem_ctrl_create_pdata - Create platform data
 *
 *  pdata is created base on information given by platform.
 *  Data used is the modem type, the cpu type and the pmic type.
 */
struct mdm_ctrl_pdata *modem_ctrl_create_pdata(struct platform_device *pdev)
{
	struct mdm_ctrl_pdata *pdata = NULL;
	struct mdm_ctrl_device_info *mid_info = NULL;
	struct modem_base_info *mb_info = NULL ;
	struct mdm_ctrl_pmic_data *pmic_data = NULL;

	if (!pdev->dev.platform_data) {
		pr_info("%s: No platform data available, checking ACPI...",
			__func__);
		/* FOR ACPI HANDLING */
		if (retrieve_modem_platform_data(pdev)) {
		  pr_err("%s: No registered info found. Disabling driver.",
			 __func__);
		  return NULL;
		}
	}

	mb_info = pdev->dev.platform_data;

	pdata = kzalloc(sizeof(struct mdm_ctrl_pdata), GFP_ATOMIC);
	if (!pdata) {
		pr_err(DRVNAME": Can't allocate pdata memory");
		return NULL;
	};

	mid_info = kzalloc(sizeof(struct mdm_ctrl_device_info), GFP_ATOMIC);
	if (!mid_info) {
		pr_err(DRVNAME": Can't allocate mid_info memory");
		goto Error;
	};

	/* Check if the modem is supported.
	 * Then set its specific timing values.
	 */
	switch (mb_info->id) {
	case MODEM_6260:
	case MODEM_6268:
		mid_info->pre_on_delay = 200;
		mid_info->on_duration = 60;
		mid_info->pre_wflash_delay = 30;
		mid_info->pre_cflash_delay = 60;
		mid_info->flash_duration = 60;
		mid_info->warm_rst_duration = 60;
		break;
	case MODEM_6360:
	case MODEM_7160:
	case MODEM_7260:
		mid_info->pre_on_delay = 200;
		mid_info->on_duration = 60;
		mid_info->pre_wflash_delay = 30;
		mid_info->pre_cflash_delay = 60;
		mid_info->flash_duration = 60;
		mid_info->warm_rst_duration = 60;
		break;
	default:
		pr_info(DRVNAME": Modem not supported %d", mb_info->id);
		goto Free_mid_info;
	}

	/* Modem is supported, fill in the pdata structure with it */
	pdata->modem = mb_info->id;

	pmic_data = (struct mdm_ctrl_pmic_data *) mb_info->pmic;
	if (!pmic_data) {
		pr_err(DRVNAME": No available PMIC Data!\n");
		goto Free_mid_info;
	};

	/* Check if the PMIC is supported.
	 * Then provide register adresses and values.
	 */
	switch (pmic_data->id) {
	case MFLD_PMIC:
		pdata->chipctrl = 0x0E0;
		pdata->chipctrlon = 0x4;
		pdata->chipctrloff = 0x2;
		pdata->chipctrl_mask = 0xF8;
		pdata->pre_pwr_down_delay = 60;
		pdata->pwr_down_duration = 20000;
		break;
	case CLVT_PMIC:
		pdata->chipctrl = 0x100;
		pdata->chipctrlon = 0x10;
		pdata->chipctrloff = 0x10;
		pdata->chipctrl_mask = 0x00;
		pdata->pre_pwr_down_delay = 650;
		pdata->pwr_down_duration = 20000;
		break;
	case MRFL_PMIC:
		pdata->chipctrl = 0x31;
		pdata->chipctrlon = 0x2;
		pdata->chipctrloff = 0x0;
		pdata->chipctrl_mask = 0xFC;
		pdata->pre_pwr_down_delay = 650;
		pdata->pwr_down_duration = 20000;
		break;
	case BYT_PMIC:
		pdata->chipctrl = pmic_data->chipctrl;
		pdata->chipctrlon = pmic_data->chipctrlon;
		pdata->chipctrloff = pmic_data->chipctrloff;
		pdata->chipctrl_mask = pmic_data->chipctrl_mask;
		pdata->pre_pwr_down_delay = 650;
		pdata->pwr_down_duration = 20000;
		break;
	default:
		pr_err(DRVNAME": PMIC not supported %d", pmic_data->id);
		goto Free_mid_info;
	}

	/* Retrieve cpu data */
	pdata->cpu_data = mb_info->data;

	pdata->device_data = (void *)mid_info;
	goto Out;

Free_mid_info:
	kfree(mid_info);
Error:
	kfree(pdata);
	pdata = NULL;
Out:
	return pdata;
}

/**
 *  mdm_ctrl_get_device_info - Create platform and modem data.
 *  @drv: Reference to the driver structure
 *
 *  Platform are build from SFI table data.
 */
void mdm_ctrl_get_device_info(struct mdm_ctrl *drv,
		struct platform_device *pdev)
{
	drv->pdata = modem_ctrl_create_pdata(pdev);

	if (!drv->pdata || drv->pdata->is_mdm_ctrl_disabled) {
		drv->is_mdm_ctrl_disabled = true;
		kfree(drv->pdata->device_data);
		kfree(drv->pdata);
		pr_info(DRVNAME": Disabling driver. No known device.");
		goto out;
	}

	mdm_ctrl_set_func(drv);
out:
	return;
}
