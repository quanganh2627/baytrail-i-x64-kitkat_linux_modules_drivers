/*
 * Crystal Cove  --  Device access for Intel PMIC for VLV2
 *
 * Copyright (c) 2013, Intel Corporation.
 *
 * Author: Yang Bin <bin.yang@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/mfd/core.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/mfd/intel_mid_pmic.h>
#include <linux/acpi.h>
#include <asm/intel_vlv2.h>
#include <linux/version.h>
#include <linux/debugfs.h>

#define PMIC_IRQ_NUM	7

#define CHIPID		0x00
#define CHIPVER		0x01
#define IRQLVL1		0x02
#define MIRQLVL1	0x0E
enum {
	PWRSRC_IRQ = 0,
	THRM_IRQ,
	BCU_IRQ,
	ADC_IRQ,
	CHGR_IRQ,
	GPIO_IRQ,
	VHDMIOCP_IRQ
};

struct intel_mid_pmic {
	struct i2c_client *i2c;
	struct mutex io_lock;
	struct device *dev;
	int irq;
	struct mutex irq_lock;
	int irq_base;
	unsigned long irq_mask;
	struct workqueue_struct *workqueue;
	struct work_struct      work;
};

static struct device *gpio_dev;
static struct resource gpio_resources[] = {
	{
		.name	= "GPIO",
		.start	= GPIO_IRQ,
		.end	= GPIO_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource pwrsrc_resources[] = {
	{
		.name  = "PWRSRC",
		.start = PWRSRC_IRQ,
		.end   = PWRSRC_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource adc_resources[] = {
	{
		.name  = "ADC",
		.start = ADC_IRQ,
		.end   = ADC_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource thermal_resources[] = {
	{
		.name  = "THERMAL",
		.start = THRM_IRQ,
		.end   = THRM_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};
static struct resource bcu_resources[] = {
	{
		.name  = "BCU",
		.start = BCU_IRQ,
		.end   = BCU_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};
static struct mfd_cell crystal_cove_data[] = {
	{
		.name = "crystal_cove_pwrsrc",
		.id = 0,
		.num_resources = ARRAY_SIZE(pwrsrc_resources),
		.resources = pwrsrc_resources,
	},
	{
		.name = "crystal_cove_adc",
		.id = 0,
		.num_resources = ARRAY_SIZE(adc_resources),
		.resources = adc_resources,
	},
	{
		.name = "crystal_cove_thermal",
		.id = 0,
		.num_resources = ARRAY_SIZE(thermal_resources),
		.resources = thermal_resources,
	},
	{
		.name = "crystal_cove_bcu",
		.id = 0,
		.num_resources = ARRAY_SIZE(bcu_resources),
		.resources = bcu_resources,
	},
	{
		.name = "crystal_cove_gpio",
		.id = 0,
		.num_resources = ARRAY_SIZE(gpio_resources),
		.resources = gpio_resources,
		.platform_data = &gpio_dev,
		.pdata_size = sizeof(gpio_dev),
	},
	{NULL, },
};


/*TLP init - begin*/
struct tlp_data {
	u16 rsvd;
	u8  high;
	u8  low;
};

#define TLP_END_MASK_H 0xE0
#define TLP_END_MASK_L 0x03
#define TLP_ADDR_BASE 0xCA

#define VR_ON 0
#define VR_OFF 1

enum {
	_0_Us,
	_0_5_Us,
	_0_75_Us,
	_1_Us,
	_2_Us,
	_3_Us,
	_4_Us,
	_5_Us,
	_10_Us,
	_15_Us,
	_20_Us,
	_30_Us,
	_40_Us,
	_50_Us,
	_60_Us,
	_80_Us,
	_100_Us,
	_250_Us,
	_500_Us,
	_1ms,
	_2ms,
	_3ms,
	_5ms,
	_10ms,
	_20ms,
	_30ms,
	_50ms,
	_80ms,
	_100ms
};

enum {
	VCC,
	VNN,
	VDDQ,
	VSDIO,
	VDDQ_VTT,
	V_USBPHY,
	VSYS_U,
	VSYS_S,
	VSYS_SX,
	VHDMI,
	VHOST,
	VBUS,
	VREFDQ,
	V1P0A,
	V1P2A,
	V1P8A,
	V3P3A,
	V1P0S,
	V1P2S,
	V1P8S,
	V2P85S,
	V3P3S,
	V5P0S,
	V1P0SX,
	V1P05S,
	V1P2SX,
	V1P8SX,
	V2P85SX,
	V1P8U,
	V3P3U,
	RSVD
};

static struct tlp_data tlpdata[0x7F];
static u8 TLP_ADDR[10];

void tlp_write(u8 addr, u8 high, u8 low)
{
	int ret;

	/* write the address */
	ret = intel_mid_pmic_writeb(0xc7, addr);
	if (ret)
		printk(KERN_ALERT "pmic write failed\n");

	/* write data high */
	ret = intel_mid_pmic_writeb(0xc8, high);
	if (ret)
		printk(KERN_ALERT "pmic write failed\n");

	/* write data low */
	ret = intel_mid_pmic_writeb(0xc9, low);
	if (ret)
		printk(KERN_ALERT "pmic write failed\n");
}

void tlp_read(u8 addr, u8 *high, u8 *low)
{
	int ret;
	/* write the address */
	ret = intel_mid_pmic_writeb(0xc7, addr);
	if (ret)
		printk(KERN_ALERT "pmic write failed\n");

	/* read data high */
	*high = intel_mid_pmic_readb(0xc8);
	if (ret)
		printk(KERN_ALERT "pmic read failed\n");

	/* read data low */
	*low  = intel_mid_pmic_readb(0xc9);
	if (ret)
		printk(KERN_ALERT "pmic read failed\n");
}

bool is_tlp_end_instr(u8 h, u8 l)
{
	if (((h & TLP_END_MASK_H) == TLP_END_MASK_H) &&
		((l & TLP_END_MASK_L) == TLP_END_MASK_L))
		return true;
	else
		return false;
}

bool is_tlp_end_boundary(u8 index, u8 addr)
{
	int i;
	for (i = 0; i < 10; i++) {
		if (i == index)
			continue;
		if (TLP_ADDR[i] == addr)
			return true;
	}
	return false;
}

void tlp_insert(u8 index, u8 op, u8 wait, u8 vr)
{
	u8 addr = intel_mid_pmic_readb(TLP_ADDR_BASE + index);
	u8 high, low;
	u8 tlp_new_high, tlp_new_low;
	bool end, end_boundary;
	do {
		tlp_read(addr, &high, &low);
		end = is_tlp_end_instr(high, low);
		addr++;
		end_boundary = is_tlp_end_boundary(index, addr);
	} while (!((end == true) || (end_boundary == true)));
	tlp_new_high = op << 5 | wait;
	tlp_new_low  = vr;
	tlp_write(addr-1, tlp_new_high, tlp_new_low);
	tlp_write(addr, TLP_END_MASK_H, TLP_END_MASK_L);
}

u8 tlp_shift(u8 index, u8 new_addr)
{
	u8 old_addr = TLP_ADDR[index];
	bool end, end_boundary;

	intel_mid_pmic_writeb(TLP_ADDR_BASE+index, new_addr);
	do {
		tlp_write(new_addr, tlpdata[old_addr].high,
			tlpdata[old_addr].low);
		end = is_tlp_end_instr(tlpdata[old_addr].high,
			tlpdata[old_addr].low);
		end_boundary = is_tlp_end_boundary(index, old_addr);
		old_addr++;
		new_addr++;
	} while (!((end == true) || (end_boundary == true)));

	return new_addr;
}

static ssize_t tlp_program_write(struct file *file,
		const char __user *userbuf, size_t count, loff_t *ppos)
{
	char buf[32];
	int res;
	int buf_size = min(count, sizeof(buf)-1);
	u32 vr;

	if (copy_from_user(buf, userbuf, buf_size))
		return -EFAULT;

	buf[buf_size] = 0;

	res = kstrtou32(buf, 10, &vr);

	if (res)
		return -EINVAL;

	if ((vr < 0) || (vr > 0x1F))
		return -EINVAL;

	/*S0ix Entry*/
	tlp_insert(4, VR_OFF, _0_Us, vr);
	/*S0ix Exit*/
	tlp_insert(3, VR_ON, _0_Us, vr);

	return buf_size;
}

static int tlp_program_show(struct seq_file *s, void *unused)
{
	seq_printf(s, "PMIC Task List Processor Program...\n");
	return 0;
}

static int tlp_program_open(struct inode *inode, struct file *file)
{
	return single_open(file, tlp_program_show, NULL);
}

static const struct file_operations tlp_program_ops = {
	.open		= tlp_program_open,
	.read		= seq_read,
	.write		= tlp_program_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void tlp_init()
{
	int i;
	u8 last_addr;

	/*cache the existing TLP programming*/
	for (i = 0; i < 10; i++)
		TLP_ADDR[i] = intel_mid_pmic_readb(TLP_ADDR_BASE + i);

	/*Move the sections to make space for S0ix*/
	last_addr = tlp_shift(0, 0x6);
	last_addr = tlp_shift(7, last_addr);
	last_addr = tlp_shift(8, last_addr);
	last_addr = tlp_shift(9, last_addr);

	/*exit S4*/
	tlp_write(last_addr, TLP_END_MASK_H, TLP_END_MASK_L);
	intel_mid_pmic_writeb(0xCB, last_addr);
	last_addr++;

	/*exit S3*/
	tlp_write(last_addr, TLP_END_MASK_H, TLP_END_MASK_L);
	intel_mid_pmic_writeb(0xCC, last_addr);
	last_addr++;

	/*enter S3*/
	tlp_write(last_addr, TLP_END_MASK_H, TLP_END_MASK_L);
	intel_mid_pmic_writeb(0xCF, last_addr);
	last_addr++;

	/*enter S4*/
	tlp_write(last_addr, TLP_END_MASK_H, TLP_END_MASK_L);
	intel_mid_pmic_writeb(0xD0, last_addr);
	last_addr++;

	/*enter S0ix*/
	last_addr = tlp_shift(4, last_addr);

	/*exit S0ix*/
	last_addr = tlp_shift(3, last_addr+0x20);

	/*
	 * Insert the following to add any VR to OFF/ON 
	 * during standby. Example for V2P85S rail as follows:
	 * tlp_insert(4, VR_OFF, _0_Us, V2P85S);
	 * tlp_insert(3, VR_ON, _0_Us, V2P85S);
	*/

	/* /sys/kernel/debug/tlp_program */
	(void) debugfs_create_file("tlp_program",
		S_IFREG | S_IRUGO, NULL, NULL, &tlp_program_ops);
}
/*TLP init - end*/


int intel_mid_pmic_set_pdata(const char *name, void *data, int len)
{
	int i;
	struct mfd_cell *cell;

	for (i = 0; i < ARRAY_SIZE(crystal_cove_data); i++) {
		cell = &crystal_cove_data[i];
		if (!strcmp(cell->name, name)) {
			cell->platform_data = data;
			cell->pdata_size = len;
			return 0;
		}
	}
	return -EINVAL;
}
EXPORT_SYMBOL(intel_mid_pmic_set_pdata);

/* wrapper function needed by Baytrail BCU driver */
int intel_scu_ipc_read_mip(u8 *data, int len, int offset, int issigned)
{
	return 0;
}
EXPORT_SYMBOL(intel_scu_ipc_read_mip);

static struct intel_mid_pmic intel_mid_pmic;
static struct intel_mid_pmic *pmic = &intel_mid_pmic;

/* These intel_scu_ipc_* APIs are formed to
 * be compatible with old SCU IPC APIs.
 */
int intel_scu_ipc_ioread8(u16 addr, u8 *data)
{
	int ret;

	ret = intel_mid_pmic_readb(addr);
	if (ret < 0)
		return ret;

	*data = ret;
	return 0;
}
EXPORT_SYMBOL(intel_scu_ipc_ioread8);

int intel_scu_ipc_iowrite8(u16 addr, u8 data)
{
	return intel_mid_pmic_writeb(addr, data);
}
EXPORT_SYMBOL(intel_scu_ipc_iowrite8);

int intel_scu_ipc_update_register(u16 addr, u8 data, u8 mask)
{
	int ret;

	mutex_lock(&pmic->io_lock);

	ret = i2c_smbus_read_byte_data(pmic->i2c, addr);
	if (ret < 0)
		goto err;

	data &= mask;
	ret &= ~mask;
	ret |= data;

	ret = i2c_smbus_write_byte_data(pmic->i2c, addr, ret);

err:
	mutex_unlock(&pmic->io_lock);
	return ret;
}
EXPORT_SYMBOL(intel_scu_ipc_update_register);

int intel_scu_ipc_readv(u16 *addr, u8 *data, int len)
{
	int i;
	int ret;

	if (len < 1 || len > 4)
		return -EINVAL;

	for (i = 0; i < len; i++) {
		ret = intel_scu_ipc_ioread8(addr[i], &data[i]);
		if (ret)
			return ret;
	}

	return 0;
}
EXPORT_SYMBOL(intel_scu_ipc_readv);

int intel_scu_ipc_writev(u16 *addr, u8 *data, int len)
{
	int i;
	int ret;

	if (len < 1 || len > 4)
		return -EINVAL;

	for (i = 0; i < len; i++) {
		ret = intel_scu_ipc_iowrite8(addr[i], data[i]);
		if (ret)
			return ret;
	}

	return 0;
}
EXPORT_SYMBOL(intel_scu_ipc_writev);

int intel_mid_pmic_readb(int reg)
{
	int ret;

	mutex_lock(&pmic->io_lock);
	ret = i2c_smbus_read_byte_data(pmic->i2c, reg);
	mutex_unlock(&pmic->io_lock);
	return ret;
}
EXPORT_SYMBOL(intel_mid_pmic_readb);

int intel_mid_pmic_writeb(int reg, u8 val)
{
	int ret;

	mutex_lock(&pmic->io_lock);
	ret = i2c_smbus_write_byte_data(pmic->i2c, reg, val);
	mutex_unlock(&pmic->io_lock);
	return ret;
}
EXPORT_SYMBOL(intel_mid_pmic_writeb);

int intel_mid_pmic_setb(int reg, u8 mask)
{
	int ret;
	int val;

	mutex_lock(&pmic->io_lock);
	val = i2c_smbus_read_byte_data(pmic->i2c, reg);
	val |= mask;
	ret = i2c_smbus_write_byte_data(pmic->i2c, reg, val);
	mutex_unlock(&pmic->io_lock);
	return ret;
}

int intel_mid_pmic_clearb(int reg, u8 mask)
{
	int ret;
	int val;

	mutex_lock(&pmic->io_lock);
	val = i2c_smbus_read_byte_data(pmic->i2c, reg);
	val &= ~mask;
	ret = i2c_smbus_write_byte_data(pmic->i2c, reg, val);
	mutex_unlock(&pmic->io_lock);
	return ret;
}

static void pmic_irq_enable(struct irq_data *data)
{
	clear_bit(data->irq - pmic->irq_base, &pmic->irq_mask);
	queue_work(pmic->workqueue, &pmic->work);
}

static void pmic_irq_disable(struct irq_data *data)
{
	set_bit(data->irq - pmic->irq_base, &pmic->irq_mask);
	queue_work(pmic->workqueue, &pmic->work);
}

static void pmic_irq_sync_unlock(struct irq_data *data)
{
	mutex_unlock(&pmic->irq_lock);
}

static void pmic_irq_lock(struct irq_data *data)
{
	mutex_lock(&pmic->irq_lock);
}

static void pmic_work(struct work_struct *work)
{
	mutex_lock(&pmic->irq_lock);
	intel_mid_pmic_writeb(MIRQLVL1, (u8)pmic->irq_mask);
	mutex_unlock(&pmic->irq_lock);
}

static irqreturn_t pmic_irq_isr(int irq, void *data)
{
	return IRQ_WAKE_THREAD;
}

static irqreturn_t pmic_irq_thread(int irq, void *data)
{
	int i;
	int pending;

	mutex_lock(&pmic->irq_lock);
	intel_mid_pmic_writeb(MIRQLVL1, (u8)pmic->irq_mask);
	pending = intel_mid_pmic_readb(IRQLVL1) & (~pmic->irq_mask);
	for (i = 0; i < PMIC_IRQ_NUM; i++)
		if (pending & (1 << i))
			handle_nested_irq(pmic->irq_base + i);
	mutex_unlock(&pmic->irq_lock);
	return IRQ_HANDLED;
}

static struct irq_chip pmic_irq_chip = {
	.name			= "intel_mid_pmic",
	.irq_bus_lock		= pmic_irq_lock,
	.irq_bus_sync_unlock	= pmic_irq_sync_unlock,
	.irq_disable		= pmic_irq_disable,
	.irq_enable		= pmic_irq_enable,
};

static void pmic_shutdown(struct i2c_client *client)
{
	dev_dbg(&client->dev, "%s called\n", __func__);

	if (pmic->irq > 0)
		disable_irq(pmic->irq);

	return;
}

static int pmic_suspend(struct device *dev)
{
	dev_dbg(dev, "%s called\n", __func__);

	if (pmic->irq > 0)
		disable_irq(pmic->irq);

	return 0;
}

static int pmic_resume(struct device *dev)
{
	dev_dbg(dev, "%s called\n", __func__);

	if (pmic->irq > 0)
		enable_irq(pmic->irq);

	return 0;
}

static const struct dev_pm_ops pmic_pm_ops = {
		SET_SYSTEM_SLEEP_PM_OPS(pmic_suspend,
				pmic_resume)
};

static int pmic_irq_init(void)
{
	int cur_irq;
	int ret;

	pmic->irq_mask = 0xff;
	intel_mid_pmic_writeb(MIRQLVL1, pmic->irq_mask);
	pmic->irq_mask = intel_mid_pmic_readb(MIRQLVL1);
	pmic->irq_base = irq_alloc_descs(VV_PMIC_IRQBASE, 0, PMIC_IRQ_NUM, 0);
	if (pmic->irq_base < 0) {
		dev_warn(pmic->dev, "Failed to allocate IRQs: %d\n",
			 pmic->irq_base);
		pmic->irq_base = 0;
		return -EINVAL;
	}

	/* Register them with genirq */
	for (cur_irq = pmic->irq_base;
	     cur_irq < PMIC_IRQ_NUM + pmic->irq_base;
	     cur_irq++) {
		irq_set_chip_data(cur_irq, pmic);
		irq_set_chip_and_handler(cur_irq, &pmic_irq_chip,
					 handle_edge_irq);
		irq_set_nested_thread(cur_irq, 1);
		irq_set_noprobe(cur_irq);
	}

	ret = request_threaded_irq(pmic->irq, pmic_irq_isr, pmic_irq_thread,
			IRQF_TRIGGER_RISING | IRQF_ONESHOT,
			"intel_mid_pmic", pmic);
	if (ret != 0) {
		dev_err(pmic->dev, "Failed to request IRQ %d: %d\n",
				pmic->irq, ret);
		return ret;
	}
	ret = enable_irq_wake(pmic->irq);
	if (ret != 0) {
		dev_warn(pmic->dev, "Can't enable PMIC IRQ as wake source: %d\n",
			 ret);
	}

	return 0;
}

static int pmic_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	int i, ret;
	struct mfd_cell *cell_dev = crystal_cove_data;

	mutex_init(&pmic->io_lock);
	mutex_init(&pmic->irq_lock);
	pmic->workqueue =
		create_singlethread_workqueue("crystal cove");
	INIT_WORK(&pmic->work, pmic_work);
	gpio_dev = &i2c->dev;
	pmic->i2c = i2c;
	pmic->dev = &i2c->dev;
	pmic->irq = i2c->irq;
	pmic_irq_init();
	dev_info(&i2c->dev, "Crystal Cove: ID 0x%02X, VERSION 0x%02X\n",
		intel_mid_pmic_readb(CHIPID), intel_mid_pmic_readb(CHIPVER));
	for (i = 0; cell_dev[i].name != NULL; i++)
		;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 1))
	ret = mfd_add_devices(pmic->dev, -1, cell_dev, i,
			NULL, pmic->irq_base, NULL);
#else
	ret = mfd_add_devices(pmic->dev, -1, cell_dev, i,
			NULL, pmic->irq_base);
#endif

	/*TLP programming*/
	tlp_init();

	return ret;
}

static int pmic_i2c_remove(struct i2c_client *i2c)
{
	mfd_remove_devices(pmic->dev);
	return 0;
}

static const struct i2c_device_id pmic_i2c_id[] = {
	{ "crystal_cove", },
	{ }
};
MODULE_DEVICE_TABLE(i2c, pmic_i2c_id);

static struct acpi_device_id pmic_acpi_match[] = {
	{ "TEST0001", 0 },
	{ },
};
MODULE_DEVICE_TABLE(acpi, pmic_acpi_match);

static struct i2c_driver pmic_i2c_driver = {
	.driver = {
		.name = "intel_mid_i2c_pmic",
		.owner = THIS_MODULE,
		.pm = &pmic_pm_ops,
		.acpi_match_table = ACPI_PTR(pmic_acpi_match),
	},
	.probe = pmic_i2c_probe,
	.remove = pmic_i2c_remove,
	.id_table = pmic_i2c_id,
	.shutdown = pmic_shutdown,
};

static int __init pmic_i2c_init(void)
{
	int ret;

	ret = i2c_add_driver(&pmic_i2c_driver);
	if (ret != 0)
		pr_err("Failed to register pmic I2C driver: %d\n", ret);

	return ret;
}
subsys_initcall(pmic_i2c_init);

static void __exit pmic_i2c_exit(void)
{
	i2c_del_driver(&pmic_i2c_driver);
}
module_exit(pmic_i2c_exit);

MODULE_DESCRIPTION("Crystal Cove support for ValleyView2 PMIC");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Yang Bin <bin.yang@intel.com");


