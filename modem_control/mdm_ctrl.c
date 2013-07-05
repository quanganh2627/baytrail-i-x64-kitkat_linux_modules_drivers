/**
 * linux/modules/drivers/modem_control/mdm_ctrl.c
 *
 * Version 1.0
 *
 * This code allows to power and reset IMC modems.
 * There is a list of commands available in include/linux/mdm_ctrl.h
 * Current version supports the following modems :
 * - IMC6260
 * - IMC6360
 * - IMC7160
 * - IMC7260
 * There is no guarantee for other modems
 *
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

#define MDM_BOOT_DEVNAME	CONFIG_MDM_CTRL_DEV_NAME

#define MDM_MODEM_READY_DELAY	60 /* Modem readiness wait duration (sec) */

/*****************************************************************************
 *
 * Local driver functions
 *
 ****************************************************************************/

/**
 *  mdm_ctrl_handle_hangup - This function handle the modem reset/coredump
 *  @work: a reference to work queue element
 *
 */
static void mdm_ctrl_handle_hangup(struct work_struct *work)
{
	struct mdm_ctrl *drv = mdm_drv;
	int modem_rst;

	/* Check the hangup reason */
	modem_rst = drv->hangup_causes;

	if (modem_rst & MDM_CTRL_HU_RESET)
		mdm_ctrl_launch_work(drv, MDM_CTRL_STATE_WARM_BOOT);

	if (modem_rst & MDM_CTRL_HU_COREDUMP)
		mdm_ctrl_launch_work(drv, MDM_CTRL_STATE_COREDUMP);

	flush_workqueue(drv->change_state_wq);

	pr_info(DRVNAME ": %s (reasons: 0x%X)\n", __func__, drv->hangup_causes);
}


/**
 *  mdm_ctrl_coredump_it - Modem has signaled a core dump
 *  @irq: IRQ number
 *  @data: mdm_ctrl driver reference
 *
 *  Schedule a work to handle CORE_DUMP depending on current modem state.
 */
static irqreturn_t mdm_ctrl_coredump_it(int irq, void *data)
{
	struct mdm_ctrl *drv = data;

	pr_err(DRVNAME": CORE_DUMP 0x%x", gpio_get_value(drv->gpio_cdump));

	/* Ignoring event if we are in OFF state. */
	if (mdm_ctrl_get_state(drv) == MDM_CTRL_STATE_OFF) {
		pr_err(DRVNAME": CORE_DUMP while OFF\r\n");
		goto out;
	}

	/* Ignoring if Modem reset is ongoing. */
	if (mdm_ctrl_get_reset_ongoing(drv) == 1) {
		pr_err(DRVNAME": CORE_DUMP while Modem Reset is ongoing\r\n");
		goto out;
	}

	/* Set the reason & launch the work to handle the hangup */
	drv->hangup_causes |= MDM_CTRL_HU_COREDUMP;
	queue_work(drv->hu_wq, &drv->hangup_work);

out:
	return IRQ_HANDLED;
}

/**
 *  mdm_ctrl_reset_it - Modem has changed reset state
 *  @irq: IRQ number
 *  @data: mdm_ctrl driver reference
 *
 *  Change current state and schedule work to handle unexpected resets.
 */
static irqreturn_t mdm_ctrl_reset_it(int irq, void *data)
{
	int value, reset_ongoing;
	struct mdm_ctrl *drv = data;
	unsigned long flags;

	value = gpio_get_value(drv->gpio_rst_out);

	/* Ignoring event if we are in OFF state. */
	if (mdm_ctrl_get_state(drv) == MDM_CTRL_STATE_OFF) {
		/* Logging event in order to minimise risk of hidding bug */
		pr_err(DRVNAME": RESET_OUT 0x%x while OFF\r\n", value);
		goto out;
	}

	/* If reset is ongoing we expect falling if applicable and rising
	 * edge.
	 */
	reset_ongoing = mdm_ctrl_get_reset_ongoing(drv);
	if (reset_ongoing) {
		pr_err(DRVNAME": RESET_OUT 0x%x\r\n", value);

		/* Rising EDGE (IPC ready) */
		if (value) {
			spin_lock_irqsave(&drv->state_lck, flags);
			/* Reset the reset ongoing flag */
			mdm_ctrl_set_reset_ongoing(drv, 0);
			spin_unlock_irqrestore(&drv->state_lck, flags);

			pr_err(DRVNAME": IPC READY !\r\n");
			mdm_ctrl_launch_work(drv, MDM_CTRL_STATE_IPC_READY);
		}

		goto out;
	}

	pr_err(DRVNAME": Unexpected RESET_OUT 0x%x\r\n", value);

	/* Unexpected reset received */
	spin_lock_irqsave(&drv->state_lck, flags);
	mdm_ctrl_set_reset_ongoing(drv, 1);
	spin_unlock_irqrestore(&drv->state_lck, flags);

	/* Set the reason & launch the work to handle the hangup */
	drv->hangup_causes |= MDM_CTRL_HU_RESET;
	queue_work(drv->hu_wq, &drv->hangup_work);

out:
	return IRQ_HANDLED;
}

/**
 *  mdm_ctrl_free_gpios - Gpio freeding
 *  @drv: Reference to the driver structure
 *
 */
static void mdm_ctrl_free_gpios(struct mdm_ctrl *drv)
{
	if (drv->irq_cdump > 0)
		free_irq(drv->irq_cdump, drv);

	drv->irq_cdump = 0;

	if (drv->irq_reset > 0)
		free_irq(drv->irq_reset, drv);

	drv->irq_reset = 0;

	gpio_free(drv->gpio_cdump);
	gpio_free(drv->gpio_rst_out);
	gpio_free(drv->gpio_pwr_on);
	gpio_free(drv->gpio_rst_bbn);
}

/**
 * mdm_ctrl_configure_gpio - Configure IRQs & GPIOs
 * @gpio: GPIO to configure
 * @direction: GPIO direction - 0: IN | 1: OUT
 *
 */
static inline int mdm_ctrl_configure_gpio(int gpio,
		int direction,
		int value,
		const char *desc)
{
	int ret;

	ret = gpio_request(gpio, "ModemControl");

	if (direction)
		ret += gpio_direction_output(gpio, value);
	else
		ret += gpio_direction_input(gpio);

	if (ret) {
		pr_err(DRVNAME": Unable to configure GPIO%d (%s)",
				gpio,
				desc);
		ret = -ENODEV;
	}

	return ret;
}

/**
 *  mdm_ctrl_setup_irq_gpio - Setup GPIO and IRQ
 *  @drv: Reference to the driver structure
 *
 *  - Request all needed gpios
 *  - Request all needed irqs
 *  - Register irqs callbacks
 */
static int mdm_ctrl_setup_irq_gpio(struct mdm_ctrl *drv)
{
	int ret;

	/* Configure the RESET_BB gpio */
	ret = mdm_ctrl_configure_gpio(drv->gpio_rst_bbn,
			1, 0, "RST_BB");
	if (ret)
		goto free_ctx4;

	/* Configure the ON gpio */
	ret = mdm_ctrl_configure_gpio(drv->gpio_pwr_on,
			1, 0, "ON");
	if (ret)
		goto free_ctx3;

	/* Configure the RESET_OUT gpio & irq */
	ret = mdm_ctrl_configure_gpio(drv->gpio_rst_out,
			0, 0, "RST_OUT");
	if (ret)
		goto free_ctx2;

	drv->irq_reset = gpio_to_irq(drv->gpio_rst_out);
	if (drv->irq_reset < 0) {
		ret = -ENODEV;
		goto free_ctx2;
	}

	ret = request_irq(drv->irq_reset,
			mdm_ctrl_reset_it,
			IRQF_TRIGGER_RISING |
			IRQF_TRIGGER_FALLING |
			IRQF_NO_SUSPEND,
			DRVNAME,
			drv);
	if (ret) {
		pr_err(DRVNAME": IRQ request failed for GPIO%d (RST_OUT)",
				drv->gpio_rst_out);
		ret = -ENODEV;
		goto free_ctx2;
	}

	/* Configure the CORE_DUMP gpio & irq */
	ret = mdm_ctrl_configure_gpio(drv->gpio_cdump,
			0, 0, "CORE_DUMP");
	if (ret)
		goto free_all;

	drv->irq_cdump = gpio_to_irq(drv->gpio_cdump);
	if (drv->irq_cdump < 0) {
		ret = -ENODEV;
		goto free_all;
	}

	ret = request_irq(drv->irq_cdump,
			mdm_ctrl_coredump_it,
			IRQF_TRIGGER_RISING | IRQF_NO_SUSPEND,
			DRVNAME,
			drv);
	if (ret) {
		pr_err(DRVNAME": IRQ request failed for GPIO%d (CORE DUMP)",
				drv->gpio_cdump);
		ret = -ENODEV;
		goto free_all;
	}

	pr_info(DRVNAME ": GPIO (rst_bbn: %d, pwr_on: %d, rst_out: %d, fcdp_rb: %d)\n",
			drv->gpio_rst_bbn,
			drv->gpio_pwr_on,
			drv->gpio_rst_out,
			drv->gpio_cdump);

	pr_info(DRVNAME ": IRQ  (rst_out: %d, fcdp_rb: %d)\n",
			drv->irq_reset, drv->irq_cdump);

	return ret;

free_all:
	mdm_ctrl_free_gpios(drv);
	return ret;

free_ctx2:
	if (drv->irq_reset > 0)
		free_irq(drv->irq_reset, drv);

	drv->irq_reset = 0;
	gpio_free(drv->gpio_rst_out);
free_ctx3:
	gpio_free(drv->gpio_pwr_on);
free_ctx4:
	gpio_free(drv->gpio_rst_bbn);

	return ret;
}

/**
 *  clear_hangup_reasons - Clear the hangup reasons flag
 */
static void clear_hangup_reasons(void)
{
	mdm_drv->hangup_causes = MDM_CTRL_NO_HU;
}

/**
 *  get_hangup_reasons - Hangup reason flag accessor
 */
static int get_hangup_reasons(void)
{
	return mdm_drv->hangup_causes;
}


/*****************************************************************************
 *
 * Char device functions
 *
 ****************************************************************************/

/**
 *  mdm_ctrl_dev_open - Manage device access
 *  @inode: The node
 *  @filep: Reference to file
 *
 *  Called when a process tries to open the device file
 */
static int mdm_ctrl_dev_open(struct inode *inode, struct file *filep)
{
	mutex_lock(&mdm_drv->lock);
	/* Only ONE instance of this device can be opened */
	if (mdm_ctrl_get_opened(mdm_drv)) {
		mutex_unlock(&mdm_drv->lock);
		return -EBUSY;
	}

	/* Save private data for futur use */
	filep->private_data = mdm_drv;

	/* Set the open flag */
	mdm_ctrl_set_opened(mdm_drv, 1);
	mutex_unlock(&mdm_drv->lock);
	return 0;
}

/**
 *  mdm_ctrl_dev_close - Reset open state
 *  @inode: The node
 *  @filep: Reference to file
 *
 *  Called when a process closes the device file.
 */
static int mdm_ctrl_dev_close(struct inode *inode, struct file *filep)
{
	struct mdm_ctrl *drv = filep->private_data;

	/* Set the open flag */
	mutex_lock(&drv->lock);
	mdm_ctrl_set_opened(drv, 0);
	mutex_unlock(&drv->lock);
	return 0;
}


/**
 *  mdm_ctrl_dev_ioctl - Process ioctl requests
 *  @filep: Reference to file that stores private data.
 *  @cmd: Command that should be executed.
 *  @arg: Command's arguments.
 *
 */
long mdm_ctrl_dev_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
{
	struct mdm_ctrl *drv = filep->private_data;
	struct mdm_ctrl_cmd cmd_params;
	long ret = 0;
	unsigned int mdm_state;
	unsigned int param;

	pr_info(DRVNAME ": ioctl request 0x%x received \r\n", cmd);
	flush_workqueue(drv->change_state_wq);
	mdm_state = mdm_ctrl_get_state(drv);

	switch (cmd) {
	case MDM_CTRL_POWER_OFF:
		/* Unconditionnal power off */
		drv->mdm_ctrl_power_off(drv);
		break;

	case MDM_CTRL_POWER_ON:
		/* Only allowed when modem is OFF or in unkown state */
		if ((mdm_state == MDM_CTRL_STATE_OFF) ||
				(mdm_state == MDM_CTRL_STATE_UNKNOWN))
			drv->mdm_ctrl_cold_boot(drv);
		else
			/* Specific log in COREDUMP state */
			if (mdm_state == MDM_CTRL_STATE_COREDUMP)
				pr_err(DRVNAME": Power ON not allowed (coredump)");
			else
				pr_info(DRVNAME": Powering on while already on");
		break;

	case MDM_CTRL_WARM_RESET:
		/* Allowed in any state unless OFF */
		if (mdm_state != MDM_CTRL_STATE_OFF)
			drv->mdm_ctrl_normal_warm_reset(drv);
		else
			pr_err(DRVNAME": Warm reset not allowed (Modem OFF)");
		break;

	case MDM_CTRL_FLASHING_WARM_RESET:
		/* Allowed in any state unless OFF */
		if (mdm_state != MDM_CTRL_STATE_OFF)
			drv->mdm_ctrl_flashing_warm_reset(drv);
		else
			pr_err(DRVNAME": Warm reset not allowed (Modem OFF)");
		break;

	case MDM_CTRL_COLD_RESET:
		/* Allowed in any state unless OFF */
		if (mdm_state != MDM_CTRL_STATE_OFF)
			drv->mdm_ctrl_cold_reset(drv);
		else
			pr_err(DRVNAME": Cold reset not allowed (Modem OFF)");
		break;

	case MDM_CTRL_SET_STATE:
		/* Read the user command params */
		ret = copy_from_user(&param,
				(void *)arg,
				sizeof(param));
		if (ret < 0) {
			pr_info(DRVNAME": copy from user failed ret = %ld\r\n",
					ret);
			goto out;
		}

		/* Filtering states. Allow any state ? */
		param &=
			(MDM_CTRL_STATE_OFF |
			 MDM_CTRL_STATE_COLD_BOOT |
			 MDM_CTRL_STATE_WARM_BOOT |
			 MDM_CTRL_STATE_COREDUMP |
			 MDM_CTRL_STATE_IPC_READY|
			 MDM_CTRL_STATE_FW_DOWNLOAD_READY);

		mdm_ctrl_launch_work(drv, param);
		flush_workqueue(drv->change_state_wq);
		break;

	case MDM_CTRL_GET_STATE:
		/* Return supposed current state.
		 * Real state can be different.
		 */
		param = mdm_state;

		ret = copy_to_user((void __user *)arg,
				&param,
				sizeof(param));
		if (ret < 0) {
			pr_info(DRVNAME ": copy to user failed ret = %ld\r\n",
					ret);
			return ret;
		}
		break;

	case MDM_CTRL_WAIT_FOR_STATE:
		/* Actively wait for state untill timeout */
		ret = copy_from_user(&cmd_params,
				(void __user *)arg,
				sizeof(cmd_params));
		if (ret < 0) {
			pr_info(DRVNAME": copy from user failed ret = %ld\r\n",
					ret);
			break;
		}
		pr_err(DRVNAME": WAIT_FOR_STATE 0x%x ! \r\n", cmd_params.param);

		ret = wait_event_interruptible_timeout(drv->event,
				drv->modem_state == cmd_params.param,
				msecs_to_jiffies(cmd_params.timeout));
		if (!ret)
			pr_err(DRVNAME": WAIT_FOR_STATE timed out ! \r\n");
		break;

	case MDM_CTRL_GET_HANGUP_REASONS:
		/* Return last hangup reason. Can be cumulative
		 * if they were not cleared since last hangup.
		 */
		param = get_hangup_reasons();

		ret = copy_to_user((void __user *)arg,
				&param,
				sizeof(param));
		if (ret < 0) {
			pr_info(DRVNAME ": copy to user failed ret = %ld\r\n",
					ret);
			return ret;
		}


		break;

	case MDM_CTRL_CLEAR_HANGUP_REASONS:
		clear_hangup_reasons();
		break;

	case MDM_CTRL_SET_POLLED_STATES:
		/* Set state to poll on. */
		/* Read the user command params */
		ret = copy_from_user(&param,
				(void *)arg,
				sizeof(param));
		if (ret < 0) {
			pr_info(DRVNAME": copy from user failed ret = %ld\r\n",
					ret);
			return ret;
		}
		drv->polled_states = param;
		/* Poll is active ? */
		if (waitqueue_active(&drv->wait_wq)) {
			flush_workqueue(drv->change_state_wq);
			mdm_state = mdm_ctrl_get_state(drv);
			/* Check if current state is awaited */
			if (mdm_state)
				drv->polled_state_reached = ((mdm_state & param)
						== mdm_state);

			/* Waking up the wait work queue to handle any
			 * polled state reached.
			 */
			wake_up(&drv->wait_wq);
		} else {
			/* Assume that mono threaded client are probably
			 * not polling yet and that they are not interested
			 * in the current state. This state may change until
			 * they start the poll. May be an issue for some cases.
			 */
			drv->polled_state_reached = false;
		}

		pr_info(DRVNAME ": states polled = 0x%x\r\n",
				drv->polled_states);
		break;

	default:
		pr_err(DRVNAME ": ioctl command %x unknown\r\n",
				cmd);
		ret = -ENOIOCTLCMD;
	}

out:
	return ret;
}

/**
 *  mdm_ctrl_dev_read - Device read function
 *  @filep: Reference to file
 *  @data: User data
 *  @count: Bytes read.
 *  @ppos: Reference to position in file.
 *
 *  Called when a process, which already opened the dev file, attempts to
 *  read from it. Not allowed.
 */
static ssize_t mdm_ctrl_dev_read(struct file *filep,
		char __user *data,
		size_t count,
		loff_t *ppos)
{
	pr_err(DRVNAME": Nothing to read\r\n");
	return -EINVAL;
}

/**
 *  mdm_ctrl_dev_write - Device write function
 *  @filep: Reference to file
 *  @data: User data
 *  @count: Bytes read.
 *  @ppos: Reference to position in file.
 *
 *  Called when a process writes to dev file.
 *  Not allowed.
 */
static ssize_t mdm_ctrl_dev_write(struct file *filep,
		const char __user *data,
		size_t count,
		loff_t *ppos)
{
	pr_err(DRVNAME": Nothing to write to\r\n");
	return -EINVAL;
}


/**
 *  mdm_ctrl_dev_poll - Poll function
 *  @filep: Reference to file storing private data
 *  @pt: Reference to poll table structure
 *
 *  Flush the change state workqueue to ensure there is no new state pending.
 *  Relaunch the poll wait workqueue.
 *  Return POLLHUP|POLLRDNORM if any of the polled states was reached.
 */
static unsigned int mdm_ctrl_dev_poll(struct file *filep,
		struct poll_table_struct *pt)
{
	struct mdm_ctrl *drv = filep->private_data;
	unsigned int ret = 0;

	/* Wait event change */
	flush_workqueue(drv->change_state_wq);
	poll_wait(filep, &drv->wait_wq, pt);

	/* State notify */
	if (drv->polled_state_reached ||
			(mdm_ctrl_get_state(drv) & drv->polled_states)) {

		drv->polled_state_reached = false;
		ret |= POLLHUP|POLLRDNORM;
		pr_info(DRVNAME ": POLLHUP occured. Current state = 0x%x\r\n",
				mdm_ctrl_get_state(drv));
	}

	return ret;
}


/**
 * Device driver file operations
 */
static const struct file_operations mdm_ctrl_ops = {
	.open	= mdm_ctrl_dev_open,
	.read	= mdm_ctrl_dev_read,
	.write	= mdm_ctrl_dev_write,
	.poll	= mdm_ctrl_dev_poll,
	.release = mdm_ctrl_dev_close,
	.unlocked_ioctl	= mdm_ctrl_dev_ioctl
};



/**
 *  mdm_ctrl_module_init - initialises the Modem Control driver
 *
 */
static int mdm_ctrl_module_probe(struct platform_device *pdev)
{
	int ret;
	struct mdm_ctrl *new_drv;

	/* Allocate modem struct data */
	new_drv = kzalloc(sizeof(struct mdm_ctrl), GFP_KERNEL);
	if (!new_drv) {
		pr_err(DRVNAME ": Out of memory(new_drv)");
		ret = -ENOMEM;
		goto out;
	}

	pr_info(DRVNAME ": Getting device infos");
	/* Pre-initialisation: Retrieve platform device data*/
	mdm_ctrl_get_device_info(new_drv, pdev);

	if (new_drv->is_mdm_ctrl_disabled) {
		/* KW fix can't happen. */
		if (unlikely(new_drv->pdata))
			kfree(new_drv->pdata);
		ret = -ENODEV;
		goto free_drv;
	}

	/* Initialization */
	spin_lock_init(&new_drv->state_lck);
	mutex_init(&new_drv->lock);
	init_waitqueue_head(&new_drv->event);
	init_waitqueue_head(&new_drv->wait_wq);

	INIT_LIST_HEAD(&new_drv->next_state_link);

	INIT_WORK(&new_drv->change_state_work, mdm_ctrl_set_state);
	/* Create a high priority ordered workqueue to change modem state */
	new_drv->change_state_wq =
		create_singlethread_workqueue(DRVNAME "-cs_wq");

	if (!new_drv->change_state_wq) {
		pr_err(DRVNAME ": Unable to create set state workqueue");
		ret = -EIO;
		goto free_drv;
	}

	INIT_WORK(&new_drv->hangup_work, mdm_ctrl_handle_hangup);

	/* Create a workqueue to manage hangup */
	new_drv->hu_wq = create_singlethread_workqueue(DRVNAME "-hu_wq");
	if (!new_drv->hu_wq) {
		pr_err(DRVNAME ": Unable to create control workqueue");
		ret = -EIO;
		goto free_change_state_wq;
	}

	/* Register the device */
	ret = alloc_chrdev_region(&new_drv->tdev, 0, 1, MDM_BOOT_DEVNAME);
	if (ret) {
		pr_err(DRVNAME ": alloc_chrdev_region failed (err: %d)", ret);
		goto free_hu_wq;
	}

	new_drv->major = MAJOR(new_drv->tdev);
	cdev_init(&new_drv->cdev, &mdm_ctrl_ops);
	new_drv->cdev.owner = THIS_MODULE;

	ret = cdev_add(&new_drv->cdev, new_drv->tdev, 1);
	if (ret) {
		pr_err(DRVNAME": cdev_add failed (err: %d)", ret);
		goto unreg_reg;
	}

	new_drv->class = class_create(THIS_MODULE, DRVNAME);
	if (IS_ERR(new_drv->class)) {
		pr_err(DRVNAME": class_create failed (err: %d)", ret);
		ret = -EIO;
		goto del_cdev;
	}

	new_drv->dev = device_create(new_drv->class,
			NULL,
			new_drv->tdev,
			NULL, MDM_BOOT_DEVNAME);

	if (IS_ERR(new_drv->dev)) {
		pr_err(DRVNAME": device_create failed (err: %ld)",
				PTR_ERR(new_drv->dev));
		ret = -EIO;
		goto del_class;
	}

	mdm_ctrl_launch_work(new_drv, MDM_CTRL_STATE_OFF);
	flush_workqueue(new_drv->change_state_wq);

	mdm_ctrl_set_gpio(new_drv);

	if (mdm_ctrl_setup_irq_gpio(new_drv))
		goto del_dev;

	/* Everything is OK */
	mdm_drv = new_drv;

	/* Init driver */
	init_timer(&mdm_drv->flashing_timer);

	/* Modem power off sequence */
	if (new_drv->pdata->cpu_data->early_pwr_off)
		mdm_drv->mdm_ctrl_power_off(new_drv);

	/* Modem cold boot sequence */
	if (new_drv->pdata->cpu_data->early_pwr_on)
		mdm_drv->mdm_ctrl_cold_boot(new_drv);

	return 0;

del_dev:
	device_destroy(new_drv->class, new_drv->tdev);

del_class:
	class_destroy(new_drv->class);

del_cdev:
	cdev_del(&new_drv->cdev);

unreg_reg:
	unregister_chrdev_region(new_drv->tdev, 1);

free_hu_wq:
	destroy_workqueue(new_drv->hu_wq);

free_change_state_wq:
	destroy_workqueue(new_drv->change_state_wq);

free_drv:
	kfree(new_drv);

out:
	return ret;
}

/**
 *  mdm_ctrl_module_exit - Frees the resources taken by the control driver
 */
static int mdm_ctrl_module_remove(struct platform_device *pdev)
{
	if (!mdm_drv)
		return 0;

	if (mdm_drv->pdata->is_mdm_ctrl_disabled)
		goto out;

	/* Delete the modem state worqueue */
	destroy_workqueue(mdm_drv->change_state_wq);

	/* Delete the modem hangup worqueue */
	destroy_workqueue(mdm_drv->hu_wq);

	/* Free IRQs & GPIOs */
	mdm_ctrl_free_gpios(mdm_drv);

	del_timer(&mdm_drv->flashing_timer);
	mutex_destroy(&mdm_drv->lock);

	/* Unregister the device */
	device_destroy(mdm_drv->class, mdm_drv->tdev);
	class_destroy(mdm_drv->class);
	cdev_del(&mdm_drv->cdev);
	unregister_chrdev_region(mdm_drv->tdev, 1);

out:
	/* Free the driver context */
	kfree(mdm_drv->pdata->device_data);
	kfree(mdm_drv->pdata);
	kfree(mdm_drv);
	mdm_drv = NULL;
	return 0;
}

static const struct platform_device_id mdm_ctrl_id_table[] = {
	{ DEVICE_NAME, 0 },
	{ },
};

MODULE_DEVICE_TABLE(platform, mdm_ctrl_id_table);

static struct platform_driver mcd_driver = {
	.probe		= mdm_ctrl_module_probe,
	.remove		= __devexit_p(mdm_ctrl_module_remove),
	.driver		= {
		.name	= DRVNAME,
		.owner	= THIS_MODULE,
	},
	.id_table	= mdm_ctrl_id_table,
};

static int __init mdm_ctrl_module_init(void)
{
	return platform_driver_register(&mcd_driver);
}

static void __exit mdm_ctrl_module_exit(void)
{
	platform_driver_unregister(&mcd_driver);
}

module_init(mdm_ctrl_module_init);
module_exit(mdm_ctrl_module_exit);

/**
 *  mdm_ctrl_modem_reset - Reset modem
 *
 *  Debug and integration purpose.
 */
static int mdm_ctrl_modem_reset(const char *val, struct kernel_param *kp)
{
	if (mdm_drv)
		mdm_drv->mdm_ctrl_silent_warm_reset(mdm_drv);
	return 0;
}

module_param_call(modem_reset, mdm_ctrl_modem_reset, NULL, NULL, 0644);

MODULE_AUTHOR("Faouaz Tenoutit <faouazx.tenoutit@intel.com>");
MODULE_AUTHOR("Frederic Berat <fredericx.berat@intel.com>");
MODULE_DESCRIPTION("Intel Modem control driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:"DEVICE_NAME);
