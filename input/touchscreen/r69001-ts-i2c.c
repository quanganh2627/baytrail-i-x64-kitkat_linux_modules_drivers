/*
 * R69001 Touchscreen Controller Driver
 * Source file
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input-polldev.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/miscdevice.h>
#include <linux/suspend.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>

#define CONFIG_R69001_POLLING_TIME 10
#include <linux/r69001-ts.h>

#define R69001_TS_NAME              "r69001-ts-i2c"

/* Coordinates data register address */
#define REG_COORDINATES_DATA        0x00    /* High */
#define REG_INFO1                   0x00    /* Low */
#define REG_INFO2                   0x01
#define REG_DATA0                   0x02
#define REG_DATA1                   0x0b
#define REG_DATA2                   0x14
#define REG_DATA3                   0x1d
#define REG_DATA4                   0x26

/* One set coordinates data size */
#define ONE_SET_COORD_DATA_SIZE     9

/* Boot Mode */
#define BOOT_MODE_BOOT_ROM          0x80

/* Commands */
#define COMMAND_BOOT                0x10
#define COMMAND_FIRMWARE_UPDATE     0x20

/* RESET */
#define MPU_RESET                   0x01

/* Control register address */
#define REG_CONTROL                 0x1c    /* High */
#define REG_SCAN_MODE               0x00    /* Low */
#define REG_SCAN_CYCLE              0x01
#define REG_INT_POLL_CTRL           0x02
#define REG_INT_SIGNAL_OUTPUT_CTRL  0x03
#define REG_WRITE_DATA_CTRL         0x04
#define REG_READY_DATA              0x05
#define REG_SCAN_COUNTER            0x06
#define REG_FW_UPDATE               0x07
#define REG_RESET                   0x09
#define REG_FUNC_CTRL               0x0b
#define REG_FW_VER                  0x14
#define REG_LOW_POWER               0x17

/* Ready data */
#define READY_COORDINATES           0x01
#define READY_RAW                   0x02
#define READY_BASELINE              0x04
#define READY_DIFF                  0x08
#define READY_LABElMAP              0x10
#define READY_CALIBRATION           0x20
#define READY_GESTURE               0x40

/* Scan Mode */
#define SCAN_MODE_STOP              R69001_SCAN_MODE_STOP
#define SCAN_MODE_LOW_POWER         R69001_SCAN_MODE_LOW_POWER
#define SCAN_MODE_FULL_SCAN         R69001_SCAN_MODE_FULL_SCAN
#define SCAN_MODE_CALIBRATION       R69001_SCAN_MODE_CALIBRATION

/* Interrupt/Polling mode */
#define INTERRUPT_MODE              R69001_TS_INTERRUPT_MODE
#define POLLING_MODE                R69001_TS_POLLING_MODE
#define POLLING_LOW_EDGE_MODE       R69001_TS_POLLING_LOW_EDGE_MODE
#define UNKNOW_MODE                 255

/* Firmware update mode */
#define FW_UPDATE_MODE              0x01

struct r69001_ts_finger {
	u16 x;
	u16 y;
	u8 z;
	u8 t;
};

struct r69001_ts_before_regs {
	u8 int_signal_output_ctrl;
	u8 scan_cycle;
};

struct r69001_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct r69001_ts_finger finger[MAX_FINGERS];
	struct r69001_io_data data;
	struct r69001_ts_before_regs regs;
	struct r69001_platform_data *pdata;
	struct workqueue_struct *workqueue;
	struct work_struct fw_update_work;
	u8 mode;
	u8 t_num;
	u8 fw_update_sig;
};

/* firmware update signal */
#define SIG_NO_INIT		0
#define SIG_INIT		1
#define SIG_UPDATE_NORMAL	2
#define SIG_UPDATE_FROM_BOOT	3
#define SIG_UPDATE_DONE		4
#define SIG_UPDATE_NO_NEED	5

/* following is for firmware update use case */
#define T_FW_V			0 /* fw version */
#define T_FW_D			1 /* fw data */

#define OP_READ			0
#define OP_WRITE		1

struct fw_block_data {
	u8 type;
	u8 num;
	u8 op;		/* read or write */
	u8 addr;	/* client addr */
	int delay;	/* after operator, delay */
	int size;	/* data size */
	u8 buf[512];	/* data buffer */
};

#define TOLOWER(x) ((x) | 0x20)
/* translate string to u8 value */
static u8 strtou8(u8 *fw, u32 *pointer, unsigned int base)
{
	char *str = (char *)(fw + *pointer);
	u8 value = 0;

	while (isxdigit(*str)) {
		unsigned int temp_value;

		if ('0' <= *str && *str <= '9')
			temp_value = *str - '0';
		else
			temp_value = TOLOWER(*str) - 'a' + 10;

		if (temp_value >= base)
			break;

		value = value * base + temp_value;

		str = (char *)(fw + ++(*pointer));
	}

	return value;
}

static int read_block_from_fw(u8 *fw, u32 *pointer, u32 max_size,
				struct fw_block_data *data)
{
	int ret = 1;

	while (*pointer < (max_size - 1)) {
		char *str = (char *)(fw + *pointer);

		if (str[0] == 'p') {
			*pointer += 1;
			ret = 0;
			break;
		} else if (str[0] == 'v') {
			*pointer += 1;
			if (data->type == T_FW_V) {
				continue;
			} else {
				ret = -1;
				break;
			}
		} else if (str[0] == 'w') {
			*pointer += 2;
			data->op = OP_WRITE;
			data->addr = strtou8(fw, pointer, 16);
			continue;
		} else if (str[0] == '[') {
			if ((str[1] == 'd') &&
				(str[2] == 'e') &&
				(str[3] == 'l') &&
				(str[4] == 'a') &&
				(str[5] == 'y')) {
				*pointer += 7;
				data->delay = strtou8(fw, pointer, 10);
			}

			if (data->type == T_FW_D) {
				*pointer += 2;
				if (data->num != strtou8(fw, pointer, 16)) {
					ret = -1;
					break;
				}
				else
					data->buf[data->size++] = data->num;
			}
			continue;
		} else if (isxdigit(str[0])) {
			if (data->type == T_FW_V ||
				data->type == T_FW_D)
				data->buf[data->size++]
					= strtou8(fw, pointer, 16);
		} else
			*pointer += 1;
	}

	return ret;
}

static void r69001_set_mode(struct r69001_ts_data *ts, u8 mode, u16 poll_time);

static int r69001_ts_read_data(struct r69001_ts_data *ts,
				u8 addr_h, u8 addr_l, u16 size, u8 *data)
{
	struct i2c_client *client = ts->client;
	struct i2c_msg msg[2];
	int error;
	u8 buf[2];

	buf[0] = addr_h;
	buf[1] = addr_l;

	/* Set data point */
	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = buf;

	/* Byte read */
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = size;
	msg[1].buf = data;

	error = i2c_transfer(client->adapter, msg, 1);
	if (error > 0)
		error = i2c_transfer(client->adapter, msg + 1, 1);
	if (error < 0)
		dev_err(&client->dev,
			"I2C read error high: 0x%x low:0x%x size:%d ret:%d\n",
			addr_h, addr_l, size, error);

	return error;
}

static int r69001_ts_write_data(struct r69001_ts_data *ts,
				u8 addr_h, u8 addr_l, u8 data)
{
	struct i2c_client *client = ts->client;
	struct i2c_msg msg;
	int error;
	u8 buf[3];

	buf[0] = addr_h;
	buf[1] = addr_l;
	buf[2] = data;

	/* Byte write */
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = sizeof(buf);
	msg.buf = buf;

	error = i2c_transfer(client->adapter, &msg, 1);
	if (error < 0)
		dev_err(&client->dev,
			"I2C write error high: 0x%x low:0x%x data:0x%x ret:%d\n",
			addr_h, addr_l, data, error);
	return error;
}

static int r69001_ts_write_fw(struct r69001_ts_data *ts, u8 *data, u16 size)
{
	struct i2c_client *client = ts->client;
	struct i2c_msg msg;
	int error;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = size;
	msg.buf = data;

	error = i2c_transfer(client->adapter, &msg, 1);
	if (error < 0)
		dev_err(&client->dev, "I2C write fw error ret:%d\n", error);

	return error;
}

static int r69001_ts_send_cmd(struct r69001_ts_data *ts, u8 command)
{
	struct i2c_client *client = ts->client;
	struct i2c_msg msg;
	int error;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 1;
	msg.buf = &command;

	error = i2c_transfer(client->adapter, &msg, 1);
	if (error < 0)
		dev_err(&client->dev, "I2C send command error ret:%d\n", error);

	return error;
}

static int r69001_ts_get_status(struct r69001_ts_data *ts, u8 *status)
{
	struct i2c_client *client = ts->client;
	struct i2c_msg msg;
	int error;

	msg.addr = client->addr;
	msg.flags = I2C_M_RD;
	msg.len = 1;
	msg.buf = status;

	error = i2c_transfer(client->adapter, &msg, 1);
	if (error < 0)
		dev_err(&client->dev, "I2C get status error ret:%d\n", error);

	return error;
}

static void r69001_ts_report_coordinates_data(struct r69001_ts_data *ts)
{
	struct r69001_ts_finger *finger = ts->finger;
	struct input_dev *input_dev = ts->input_dev;
	u8 i;

	for (i = 0; i < ts->t_num; i++) {
		input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, finger[i].t);
		input_report_abs(input_dev, ABS_MT_POSITION_X, finger[i].x);
		input_report_abs(input_dev, ABS_MT_POSITION_Y, finger[i].y);
		input_report_abs(input_dev, ABS_MT_PRESSURE, finger[i].z);
		input_mt_sync(input_dev);
	}

	/* SYN_MT_REPORT only if no contact */
	if (!ts->t_num)
		input_mt_sync(input_dev);

	/* SYN_REPORT */
	input_sync(input_dev);

	ts->t_num = 0;
}

static int r69001_ts_read_coordinates_data(struct r69001_ts_data *ts)
{
	struct r69001_ts_finger *finger = ts->finger;
	u8 i;
	u8 numt = 0;
	u8 data[ONE_SET_COORD_DATA_SIZE] = { 0 };
	u8 lowreg[5] = {REG_DATA0, REG_DATA1, REG_DATA2, REG_DATA3, REG_DATA4};
	int error;

	error = r69001_ts_read_data(ts,
			REG_COORDINATES_DATA, REG_INFO1, 1, &numt);
	if (error < 0)
		return error;

	numt &= 0x0f;
	if (numt > MAX_FINGERS)
		numt = MAX_FINGERS;

	for (i = 0; i < numt; i++) {
		if (i % 2) {
			finger[i].x =
				((u16)(data[7] & 0x0f) << 8) | (u16)(data[5]);
			finger[i].y =
				((u16)(data[7] & 0xf0) << 4) | (u16)(data[6]);
			finger[i].z = data[8];
			finger[i].t = (data[0] & 0xf0) >> 4;
		} else {
			error = r69001_ts_read_data(ts,
					REG_COORDINATES_DATA, lowreg[i / 2],
					ONE_SET_COORD_DATA_SIZE, data);
			if (error < 0)
				return error;
			finger[i].x =
				((u16)(data[3] & 0x0f) << 8) | (u16)(data[1]);
			finger[i].y =
				((u16)(data[3] & 0xf0) << 4) | (u16)(data[2]);
			finger[i].z = data[4];
			finger[i].t = data[0] & 0x0f;
		}
	}

	/* Only update the number when there is no error happened */
	ts->t_num = numt;
	return 0;
}

static irqreturn_t r69001_ts_irq_handler(int irq, void *dev_id)
{
	struct r69001_ts_data *ts = dev_id;
	u8 mode = 0, status = 0;

	if (ts->fw_update_sig != SIG_UPDATE_NO_NEED) {
		if (ts->fw_update_sig == SIG_NO_INIT)
			ts->fw_update_sig = SIG_INIT;
		else if (ts->fw_update_sig == SIG_INIT) {
			r69001_ts_get_status(ts, &status);
			if (status & (0xa0 | 0x80))
				ts->fw_update_sig = SIG_UPDATE_FROM_BOOT;
			else
				ts->fw_update_sig = SIG_UPDATE_NORMAL;

			schedule_work(&ts->fw_update_work);
		} else if (ts->fw_update_sig == SIG_UPDATE_DONE) {
			/* just update fw finished, so need calibration here */
			r69001_ts_write_data(ts, REG_CONTROL,
					REG_SCAN_MODE, SCAN_MODE_STOP);
			r69001_ts_write_data(ts, REG_CONTROL,
					REG_SCAN_MODE, SCAN_MODE_CALIBRATION);
			usleep_range(500000, 500000);
			r69001_ts_write_data(ts, REG_CONTROL,
					REG_SCAN_MODE, SCAN_MODE_STOP);

			ts->fw_update_sig = SIG_UPDATE_NO_NEED;
		}
	}

	r69001_ts_read_data(ts, REG_CONTROL, REG_SCAN_MODE, 1, &mode);

	if (mode == SCAN_MODE_STOP) {
		/* if receive a touchscreen interrupt, but the scan mode is stop
		 * that means touch panel just power on, so re-init it
		 */
		ts->data.mode.mode = UNKNOW_MODE;
		r69001_ts_write_data(ts, REG_CONTROL,
					REG_SCAN_CYCLE, SCAN_TIME);
		r69001_set_mode(ts, ts->mode, POLL_INTERVAL);
	}

	r69001_ts_read_coordinates_data(ts);
	r69001_ts_report_coordinates_data(ts);

	return IRQ_HANDLED;
}

static int check_and_update_fw(struct r69001_ts_data *ts, u8 *fw, int size)
{
	int ret = 0;
	int count;
	u32 pointer = 0;
	u16 current_fw_v = 0, file_fw_v = 0;
	u8 vbuf[2];
	struct fw_block_data data;
	struct r69001_platform_data *pdata = ts->client->dev.platform_data;

	/* read fw version from file */
	memset(&data, 0, sizeof(struct fw_block_data));
	data.type = T_FW_V;
	ret = read_block_from_fw(fw, &pointer, size, &data);
	if (!ret){
		file_fw_v = data.buf[0] + (data.buf[1] << 8);
	} else {
		dev_err(&ts->client->dev, "Read fw version failed!\n");
		return -1;
	}

	if (ts->fw_update_sig == SIG_UPDATE_NORMAL) {
		/* read current fw version from touch panel */
		if (r69001_ts_read_data(ts,
			REG_CONTROL, REG_FW_VER, 2, vbuf) >= 0) {
			current_fw_v = vbuf[0] + (vbuf[1] << 8);
		} else {
			dev_err(&ts->client->dev,
				"Read fw version from touchsreen failed!\n");
			return -1;
		}

		if (file_fw_v == current_fw_v) {
			ts->fw_update_sig = SIG_UPDATE_NO_NEED;
			printk("r69001: Current touch fw is up-to-date!\n");
			return 0;
		}

		/* if need to update fw, change to firmware update mode */
		if (r69001_ts_write_data(ts,
			REG_CONTROL, REG_FW_UPDATE, FW_UPDATE_MODE) < 0) {
			dev_err(&ts->client->dev,
				"fail to change to fw update mode!\n");
			return -1;
		}
	} else if (ts->fw_update_sig == SIG_UPDATE_FROM_BOOT) {
		/* firmware update failed at last time,
		 * so the only way to recovery is update from boot mode */
		if (r69001_ts_send_cmd(ts, COMMAND_FIRMWARE_UPDATE) < 0 ) {
			dev_err(&ts->client->dev,
				"fail to change to boot fw update mode!\n");
			return -1;
		}
	}

	free_irq(ts->client->irq, ts);

	/* then starting read data  */
	count = 0;

	do {
		memset(&data, 0, sizeof(struct fw_block_data));
		data.type = T_FW_D;
		data.num = count;
		ret = read_block_from_fw(fw, &pointer, size, &data);
		if (ret)
			break;

		if (r69001_ts_write_fw(ts, data.buf, data.size) < 0) {
			dev_err(&ts->client->dev,
				"write touch fw data failed!\n");
			ret = -1;
			break;
		}

		usleep_range(data.delay * 1000, data.delay * 1000);

		count++;
	} while (1);

	if (ret >= 0)
		ret = request_threaded_irq(ts->client->irq, NULL,
					r69001_ts_irq_handler,
					pdata->irq_type, ts->client->name, ts);

	return ret;
}

static void ts_fw_update_handler(struct work_struct *work)
{
	struct r69001_ts_data *ts =
		container_of(work, struct r69001_ts_data, fw_update_work);
	const struct firmware *fw_entry;

	disable_irq(ts->client->irq);

	if (!request_firmware(&fw_entry,
			"ts_firmware.iic", &ts->client->dev)) {
		if (fw_entry) {
			check_and_update_fw(ts, (u8 *)fw_entry->data,
						fw_entry->size);
			release_firmware(fw_entry);
		}
	}

	if (ts->fw_update_sig != SIG_UPDATE_NO_NEED)
		ts->fw_update_sig = SIG_UPDATE_DONE;

	enable_irq(ts->client->irq);
}

/*
 * Set Int Ctl
 * mode : 0 = INT Mode, 1 = POLL Mode, 2 = POLL + INT Mode
 * poll_time : Polling interval (msec, 1 - 1000)
 *
 * The msleep(100) in this driver comes directly from Vendor's
 * driver, and can't find any explanation in the datasheet, so
 * just keep it now.
 */
static void r69001_set_mode(struct r69001_ts_data *ts, u8 mode, u16 poll_time)
{
	struct i2c_client *client = ts->client;

	if (ts->data.mode.mode == mode)
		return;

	switch (mode) {
	case INTERRUPT_MODE:
		r69001_ts_write_data(ts, REG_CONTROL,
				REG_INT_POLL_CTRL, INTERRUPT_MODE);

		r69001_ts_write_data(ts, REG_CONTROL,
				REG_SCAN_MODE, SCAN_MODE_STOP);
		msleep(100);
		r69001_ts_write_data(ts, REG_CONTROL,
				REG_SCAN_MODE, SCAN_MODE_FULL_SCAN);
		ts->data.mode.mode = mode;
		break;
	case POLLING_MODE:
	case POLLING_LOW_EDGE_MODE:
		r69001_ts_write_data(ts, REG_CONTROL,
				REG_INT_POLL_CTRL, POLLING_MODE);
		if (mode == POLLING_LOW_EDGE_MODE)
			r69001_ts_write_data(ts, REG_CONTROL,
				REG_INT_SIGNAL_OUTPUT_CTRL, 0x01);
		else
			r69001_ts_write_data(ts, REG_CONTROL,
				REG_INT_SIGNAL_OUTPUT_CTRL, 0x00);
		r69001_ts_write_data(ts, REG_CONTROL,
				REG_SCAN_MODE, SCAN_MODE_STOP);
		msleep(100);
		r69001_ts_write_data(ts, REG_CONTROL,
				REG_SCAN_MODE, SCAN_MODE_FULL_SCAN);
		r69001_ts_write_data(ts, REG_CONTROL,
				REG_WRITE_DATA_CTRL, 0x01);
		if (poll_time && poll_time <= POLL_INTERVAL_MAX)
			ts->data.mode.poll_time = poll_time;
		else
			ts->data.mode.poll_time = POLL_INTERVAL;
		ts->data.mode.mode = mode;
		break;
	default:
		dev_err(&client->dev, "Set Int Ctl bad parameter = %d\n", mode);
		break;
	}
}


static int
r69001_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct r69001_ts_data *ts;
	struct input_dev *input_dev;
	struct r69001_platform_data *pdata = client->dev.platform_data;
	int error;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "Not I2C_FUNC_I2C\n");
		return -EIO;
	}

	ts = kzalloc(sizeof(struct r69001_ts_data), GFP_KERNEL);
	if (!ts) {
		dev_err(&client->dev, "Out of memory\n");
		return -ENOMEM;
	}
	if (!pdata) {
		dev_err(&client->dev, "No touch platform data\n");
		error = -EINVAL;
		goto err1;
	}
	ts->client = client;
	ts->pdata = pdata;

	if (!client->irq) /* not fast irq but a gpio one */
		client->irq = gpio_to_irq(pdata->gpio);

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&client->dev, "Unable to allocated input device\n");
		error =  -ENOMEM;
		goto err2;
	}

	ts->input_dev = input_dev;

	input_dev->name = "r69001-touchscreen";
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;

	__set_bit(EV_SYN, input_dev->evbit);
	__set_bit(EV_ABS, input_dev->evbit);

	input_set_abs_params(input_dev,
				ABS_MT_TOUCH_MAJOR, MIN_AREA, MAX_AREA, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, MIN_X, MAX_X, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, MIN_Y, MAX_Y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, MIN_Z, MAX_Z, 0, 0);

	error = input_register_device(ts->input_dev);
	if (error) {
		dev_err(&client->dev, "Failed to register %s input device\n",
							input_dev->name);
		goto err3;
	}

	i2c_set_clientdata(client, ts);

	ts->workqueue = create_singlethread_workqueue("r69001_ts_workqueue");
	if (!ts->workqueue) {
		dev_err(&client->dev, "Unable to create workqueue\n");
		error = -ENOMEM;
		goto err4;
	}

	INIT_WORK(&ts->fw_update_work, ts_fw_update_handler);

	ts->fw_update_sig = SIG_NO_INIT;
	ts->mode = INTERRUPT_MODE;

	error = request_threaded_irq(client->irq, NULL, r69001_ts_irq_handler,
			pdata->irq_type, client->name, ts);
	if (error) {
		dev_err(&client->dev, "Failed to register interrupt\n");
		goto err5;
	}

	return 0;
err5:
	destroy_workqueue(ts->workqueue);
err4:
	input_unregister_device(ts->input_dev);
err3:
	input_free_device(ts->input_dev);
err2:
err1:
	kfree(ts);
	return error;
}

static int r69001_ts_remove(struct i2c_client *client)
{
	struct r69001_ts_data *ts = i2c_get_clientdata(client);

	destroy_workqueue(ts->workqueue);
	input_unregister_device(ts->input_dev);
	if (client->irq)
		free_irq(client->irq, ts);
	kfree(ts);
	return 0;
}

static const struct i2c_device_id r69001_ts_id[] = {
	{ R69001_TS_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, r69001_ts_id);

static struct i2c_driver r69001_ts_driver = {
	.probe = r69001_ts_probe,
	.remove = r69001_ts_remove,
	.id_table = r69001_ts_id,
	.driver = {
		.name = R69001_TS_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init r69001_ts_init(void)
{
	return i2c_add_driver(&r69001_ts_driver);
}

static void __exit r69001_ts_exit(void)
{
	i2c_del_driver(&r69001_ts_driver);
}

module_init(r69001_ts_init);
module_exit(r69001_ts_exit);

MODULE_DESCRIPTION("Renesas SP Driver R69001 Touchscreen Controller Driver");
MODULE_LICENSE("GPL");
