/*
 * linux/drivers/modem_control/mdm_imc.h
 *
 * Version 1.0
 *
 * This code includes definitions for IMC modems.
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


#ifndef _MDM_IMC_H
#define _MDM_IMC_H

#define MDM_WARM_RST_FLASHING_OVER      90 /* Flashing window closed (msec) */

int mdm_ctrl_cold_boot_6x6x(struct mdm_ctrl *drv);
int mdm_ctrl_cold_reset_6x6x(struct mdm_ctrl *drv);
int mdm_ctrl_silent_warm_reset_6x6x(struct mdm_ctrl *drv);
int mdm_ctrl_normal_warm_reset_6x6x(struct mdm_ctrl *drv);
int mdm_ctrl_flashing_warm_reset_6x6x(struct mdm_ctrl *drv);
int mdm_ctrl_power_off_6x6x(struct mdm_ctrl *drv);

int mdm_ctrl_cold_boot_7x6x(struct mdm_ctrl *drv);
int mdm_ctrl_cold_reset_7x6x(struct mdm_ctrl *drv);
int mdm_ctrl_silent_warm_reset_7x6x(struct mdm_ctrl *drv);
int mdm_ctrl_normal_warm_reset_7x6x(struct mdm_ctrl *drv);
int mdm_ctrl_flashing_warm_reset_7x6x(struct mdm_ctrl *drv);
int mdm_ctrl_power_off_7x6x(struct mdm_ctrl *drv);

#endif /* _MDM_IMC_H */
