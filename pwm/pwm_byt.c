/*
 * Intel Baytrail PWM driver.
 *
 * Copyright (C) 2013 Intel corporation.
 *
 * ----------------------------------------------------------------------------
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 * ----------------------------------------------------------------------------
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/pm_runtime.h>
#include <linux/fs.h>
#include <linux/pwm.h>

/* PWM registers and bits definitions */

#define PWMCR(chip)	(chip->mmio_base + 0)
#define PWMRESET(chip)	(chip->mmio_base + 0x804)
#define PWMCR_EN	(1 << 31)
#define PWMCR_UP	(1 << 30)
#define PWMRESET_EN	3

#define PWMCR_OTD_MASK	0xff
#define PWMCR_BU_MASK	0xff00
#define PWMCR_BUF_MASK	0xff0000

#define PWMCR_OTD_OFFSET	0
#define PWMCR_BU_OFFSET	8
#define PWMCR_BUF_OFFSET	16

struct byt_pwm_chip {
	struct mutex lock;
	unsigned int pwm_num;
	struct pwm_chip	chip;
	struct device	*dev;
	struct list_head list;
	void __iomem	*mmio_base;
	unsigned int   clk_khz;
};

static LIST_HEAD(pwm_chip_list);

static inline struct byt_pwm_chip *to_byt_pwm_chip(struct pwm_chip *chip)
{
	return container_of(chip, struct byt_pwm_chip, chip);
}

static int byt_pwm_wait_update_complete(struct byt_pwm_chip *byt_pwm)
{
	uint32_t update;
	int retry = 0;

	while (retry < 5) {
		update = ioread32(PWMCR(byt_pwm));
		if (!(update & PWMCR_UP))
			break;
		if (!(update & PWMCR_EN))
			break;
		/* typically, it needs about 6 ns to clear the update bit */
		ndelay(6);
		++retry;
	}

	if (retry >= 5) {
		pr_err("PWM update failed, update bit is not cleared!");
		return -EBUSY;
	} else {
		pr_info("PWM update succeeded with retry = %d \n", retry);
		return 0;
	}
}

static int byt_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
			  int duty_ns, int period_ns)
{
	struct byt_pwm_chip *byt_pwm = to_byt_pwm_chip(chip);
	uint32_t bu;
	uint32_t bu_f;
	uint32_t otd;
	uint32_t update;
	int r;

	pm_runtime_get_sync(byt_pwm->dev);

	/* frequency = clock * base_unit/256, so:
	   base_unit = frequency * 256 / clock, which result:
	   base_unit = 256 * 10^6 / (clock_khz * period_ns); */
	bu = (256 * 1000000) / (byt_pwm->clk_khz * period_ns);
	bu_f = (256 * 1000000) % (byt_pwm->clk_khz * period_ns);
	bu_f = bu_f * 256 / (byt_pwm->clk_khz * period_ns);

	/* one time divison calculation:
	   duty_ns / period_ns = (256 - otd) / 256 */
	otd = 256 - duty_ns * 256 / period_ns;

	mutex_lock(&byt_pwm->lock);

	/* update counter */
	update = ioread32(PWMCR(byt_pwm));
	update &= (~PWMCR_OTD_MASK & ~PWMCR_BU_MASK & ~PWMCR_BUF_MASK);
	update |= (otd & 0xff) << PWMCR_OTD_OFFSET;
	update |= (bu & 0xff) << PWMCR_BU_OFFSET;
	update |= (bu_f & 0xff) << PWMCR_BUF_OFFSET;
	iowrite32(update, PWMCR(byt_pwm));

	/* set update flag */
	update |= PWMCR_UP;
	iowrite32(update, PWMCR(byt_pwm));
	r = byt_pwm_wait_update_complete(byt_pwm);

	mutex_unlock(&byt_pwm->lock);

	pm_runtime_mark_last_busy(byt_pwm->dev);
	pm_runtime_put_autosuspend(byt_pwm->dev);

	return r;
}

static int byt_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct byt_pwm_chip *byt_pwm = to_byt_pwm_chip(chip);
	uint32_t val;
	int r;

	pm_runtime_get_sync(byt_pwm->dev);
	mutex_lock(&byt_pwm->lock);

	val = ioread32(PWMCR(byt_pwm));
	iowrite32(val | PWMCR_EN, PWMCR(byt_pwm));
	r = byt_pwm_wait_update_complete(byt_pwm);

	mutex_unlock(&byt_pwm->lock);
	pm_runtime_mark_last_busy(byt_pwm->dev);
	pm_runtime_put_autosuspend(byt_pwm->dev);

	return r;
}

static void byt_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct byt_pwm_chip *byt_pwm = to_byt_pwm_chip(chip);
	uint32_t val;

	pm_runtime_get_sync(byt_pwm->dev);
	mutex_lock(&byt_pwm->lock);

	val = ioread32(PWMCR(byt_pwm));
	iowrite32(val & ~PWMCR_EN, PWMCR(byt_pwm));

	mutex_unlock(&byt_pwm->lock);
	pm_runtime_mark_last_busy(byt_pwm->dev);
	pm_runtime_put_autosuspend(byt_pwm->dev);
}

static struct pwm_ops byt_pwm_ops = {
	.config = byt_pwm_config,
	.enable = byt_pwm_enable,
	.disable = byt_pwm_disable,
	.owner = THIS_MODULE,
};


static struct byt_pwm_chip *find_pwm_chip(unsigned int pwm_num)
{
	struct byt_pwm_chip *p;
	list_for_each_entry(p, &pwm_chip_list, list) {
		if (p->pwm_num == pwm_num)
			return p;
	}
	return NULL;
}

/* directly write a value to a PWM register */
int lpio_bl_write(uint8_t pwm_num, uint32_t reg, uint32_t val)
{
	struct byt_pwm_chip *byt_pwm;

	/* only PWM_CTRL register is supported */
	if (reg != LPIO_PWM_CTRL)
		return -EINVAL;

	byt_pwm = find_pwm_chip(pwm_num);
	if (!byt_pwm) {
		pr_err("%s: can't find pwm device with pwm_num %d\n",
				__func__, (int) pwm_num);
		return -EINVAL;
	}

	pm_runtime_get_sync(byt_pwm->dev);
	mutex_lock(&byt_pwm->lock);

	iowrite32(val, PWMCR(byt_pwm));

	mutex_unlock(&byt_pwm->lock);
	pm_runtime_mark_last_busy(byt_pwm->dev);
	pm_runtime_put_autosuspend(byt_pwm->dev);

	return 0;

}
EXPORT_SYMBOL(lpio_bl_write);

/* directly update bits of a PWM register */
int lpio_bl_write_bits(uint8_t pwm_num, uint32_t reg, uint32_t val,
		uint32_t mask)
{
	struct byt_pwm_chip *byt_pwm;
	uint32_t update;

	/* only PWM_CTRL register is supported */
	if (reg != LPIO_PWM_CTRL)
		return -EINVAL;

	byt_pwm = find_pwm_chip(pwm_num);
	if (!byt_pwm) {
		pr_err("%s: can't find pwm device with pwm_num %d\n",
				__func__, (int) pwm_num);
		return -EINVAL;
	}

	pm_runtime_get_sync(byt_pwm->dev);
	mutex_lock(&byt_pwm->lock);

	update = ioread32(PWMCR(byt_pwm));
	update = (update & ~mask) | (val & mask);
	iowrite32(update, PWMCR(byt_pwm));

	mutex_unlock(&byt_pwm->lock);
	pm_runtime_mark_last_busy(byt_pwm->dev);
	pm_runtime_put_autosuspend(byt_pwm->dev);

	return 0;
}
EXPORT_SYMBOL(lpio_bl_write_bits);

/* set the update bit of the PWM control register to force PWM device to use the
new configuration */
int lpio_bl_update(uint8_t pwm_num, uint32_t reg)
{
	struct byt_pwm_chip *byt_pwm;
	uint32_t update;
	int r;

	/* only PWM_CTRL register is supported */
	if (reg != LPIO_PWM_CTRL)
		return -EINVAL;

	byt_pwm = find_pwm_chip(pwm_num);
	if (!byt_pwm) {
		pr_err("%s: can't find pwm device with pwm_num %d\n",
				__func__, (int) pwm_num);
		return -EINVAL;
	}

	pm_runtime_get_sync(byt_pwm->dev);
	mutex_lock(&byt_pwm->lock);

	update = ioread32(PWMCR(byt_pwm));
	update |= PWMCR_UP;
	iowrite32(update, PWMCR(byt_pwm));
	r = byt_pwm_wait_update_complete(byt_pwm);

	mutex_unlock(&byt_pwm->lock);
	pm_runtime_mark_last_busy(byt_pwm->dev);
	pm_runtime_put_autosuspend(byt_pwm->dev);

	return r;
}
EXPORT_SYMBOL(lpio_bl_update);

static ssize_t attr_ctl_reg_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct byt_pwm_chip *byt_pwm = dev_get_drvdata(dev);
	uint32_t val;

	pm_runtime_get_sync(byt_pwm->dev);
	mutex_lock(&byt_pwm->lock);

	val = ioread32(PWMCR(byt_pwm));

	mutex_unlock(&byt_pwm->lock);
	pm_runtime_mark_last_busy(byt_pwm->dev);
	pm_runtime_put_autosuspend(byt_pwm->dev);

	return sprintf(buf, "0x%x\n", val);
}

static ssize_t attr_test_pwm_config(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct byt_pwm_chip *byt_pwm = dev_get_drvdata(dev);
	int duty_ns, period_ns;
	int r;
	int pwm_id;
	struct pwm_device *pwm;

	r = sscanf(buf, "%d %d", &duty_ns, &period_ns);
	if (r != 2)
		return -EINVAL;

	pwm_id = byt_pwm->chip.pwms[0].pwm;
	pwm = pwm_request(pwm_id, "test");
	if (!pwm)
		return -ENODEV;
	r = pwm_config(pwm, duty_ns, period_ns);
	pwm_free(pwm);
	if (r)
		return r;

	return size;
}

static ssize_t attr_test_write(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct byt_pwm_chip *byt_pwm = dev_get_drvdata(dev);
	u32 val;

	if (kstrtou32(buf, 16, &val))
		return -EINVAL;

	lpio_bl_write(byt_pwm->pwm_num, LPIO_PWM_CTRL, val);
	return size;
}

static ssize_t attr_test_update(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct byt_pwm_chip *byt_pwm = dev_get_drvdata(dev);
	lpio_bl_update(byt_pwm->pwm_num, LPIO_PWM_CTRL);
	return size;
}

static ssize_t attr_test_write_bits(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct byt_pwm_chip *byt_pwm = dev_get_drvdata(dev);
	unsigned int val, mask;
	int r;

	r = sscanf(buf, "%x %x", &val, &mask);
	if (r != 2)
		return -EINVAL;

	lpio_bl_write_bits(byt_pwm->pwm_num, LPIO_PWM_CTRL, val, mask);
	return size;
}

static DEVICE_ATTR(ctl_reg, S_IRUSR, attr_ctl_reg_show, NULL);
static DEVICE_ATTR(pwm_config, S_IWUSR, NULL, attr_test_pwm_config);
static DEVICE_ATTR(test_write, S_IWUSR, NULL, attr_test_write);
static DEVICE_ATTR(test_update, S_IWUSR, NULL, attr_test_update);
static DEVICE_ATTR(test_write_bits, S_IWUSR, NULL, attr_test_write_bits);

static struct attribute *byt_pwm_attrs[] = {
	&dev_attr_ctl_reg.attr,
	&dev_attr_pwm_config.attr,
	&dev_attr_test_write.attr,
	&dev_attr_test_update.attr,
	&dev_attr_test_write_bits.attr,
	NULL
};

static const struct attribute_group byt_pwm_attr_group = {
	.attrs = byt_pwm_attrs,
};

static int pwm_byt_probe(struct pci_dev *pdev,
			    const struct pci_device_id *id)
{
	struct byt_pwm_chip *byt_pwm;
	static int pwm_num;
	int r;


	r = pcim_enable_device(pdev);
	if (r) {
		dev_err(&pdev->dev, "Failed to enable PWM PCI device (%d)\n",
			r);
		return r;
	}

	r = pcim_iomap_regions(pdev, 1 << 0, pci_name(pdev));
	if (r) {
		dev_err(&pdev->dev, "I/O memory remapping failed\n");
		return r;
	}

	byt_pwm = devm_kzalloc(&pdev->dev, sizeof(*byt_pwm), GFP_KERNEL);
	if (!byt_pwm) {
		dev_err(&pdev->dev, "Failed to allocate memory\n");
		r = -ENOMEM;
		goto err_iounmap;
	}

	mutex_init(&byt_pwm->lock);
	byt_pwm->dev = &pdev->dev;
	byt_pwm->chip.dev = &pdev->dev;
	byt_pwm->chip.ops = &byt_pwm_ops;
	byt_pwm->chip.base = -1;
	byt_pwm->chip.npwm = 1;
	byt_pwm->mmio_base = pcim_iomap_table(pdev)[0];
	byt_pwm->pwm_num = pwm_num;
	byt_pwm->clk_khz = id->driver_data;
	++pwm_num;
	pci_set_drvdata(pdev, byt_pwm);

	r = pwmchip_add(&byt_pwm->chip);
	if (r < 0) {
		dev_err(&pdev->dev, "pwmchip_add() failed: %d\n", r);
		r = -ENODEV;
		goto err_kfree;
	}

	r = sysfs_create_group(&pdev->dev.kobj, &byt_pwm_attr_group);
	if (r) {
		dev_err(&pdev->dev, "failed to create sysfs files: %d\n", r);
		goto err_remove_chip;
	}
	list_add_tail(&byt_pwm->list, &pwm_chip_list);
	dev_info(&pdev->dev, "PWM device probed: pwm_num=%d, mmio_base=%p clk_khz=%d\n",
			byt_pwm->pwm_num, byt_pwm->mmio_base, byt_pwm->clk_khz);

	pm_runtime_set_autosuspend_delay(&pdev->dev, 5);
	pm_runtime_use_autosuspend(&pdev->dev);
	pm_runtime_put_noidle(&pdev->dev);
	pm_runtime_allow(&pdev->dev);
	return 0;

err_remove_chip:
	pwmchip_remove(&byt_pwm->chip);
err_kfree:
	devm_kfree(&pdev->dev, byt_pwm);
err_iounmap:
	pcim_iounmap_regions(pdev, 1 << 0);
	dev_info(&pdev->dev, "PWM device probe failed!\n");
	return r;
}

static void pwm_byt_remove(struct pci_dev *pdev)
{
	struct byt_pwm_chip *byt_pwm;

	pm_runtime_get_noresume(&pdev->dev);
	pm_runtime_forbid(&pdev->dev);

	byt_pwm = pci_get_drvdata(pdev);
	list_del(&byt_pwm->list);
	pwmchip_remove(&byt_pwm->chip);
	mutex_destroy(&byt_pwm->lock);
	devm_kfree(&pdev->dev, byt_pwm);

	sysfs_remove_group(&pdev->dev.kobj, &byt_pwm_attr_group);
	pcim_iounmap_regions(pdev, 1 << 0);
	pci_disable_device(pdev);
	pci_dev_put(pdev);
}


static int pwm_byt_suspend(struct device *dev)
{
	struct byt_pwm_chip *byt_pwm = dev_get_drvdata(dev);
	uint32_t val;
	int r = 0;

	if (!mutex_trylock(&byt_pwm->lock)) {
		dev_err(dev, "PWM suspend called! can't get lock\n");
		return -EAGAIN;
	}

	val = ioread32(PWMCR(byt_pwm));
	dev_info(dev, "PWM suspend called! ctl_reg = %x\n", val);
	r = (val & PWMCR_EN) ? -EAGAIN : 0;

	mutex_unlock(&byt_pwm->lock);
	return r;
}

static int pwm_byt_resume(struct device *dev)
{
	struct byt_pwm_chip *byt_pwm = dev_get_drvdata(dev);

	dev_info(dev, "PWM resume called!\n");
	if (!mutex_trylock(&byt_pwm->lock)) {
		dev_err(dev, "Can't get lock\n");
		return -EAGAIN;
	}

	iowrite32(PWMRESET_EN, PWMRESET(byt_pwm));

	mutex_unlock(&byt_pwm->lock);
	return 0;
}

static const struct dev_pm_ops pwm_byt_pm = {
	.suspend_late = pwm_byt_suspend,
	.resume_early = pwm_byt_resume,
	SET_RUNTIME_PM_OPS(pwm_byt_suspend, pwm_byt_resume, NULL)
};

static DEFINE_PCI_DEVICE_TABLE(pwm_byt_pci_ids) = {
	{ PCI_VDEVICE(INTEL, 0x0F08), 25000},
	{ PCI_VDEVICE(INTEL, 0x0F09), 25000},
	{ 0,}
};
MODULE_DEVICE_TABLE(pci, pwm_byt_pci_ids);

static struct pci_driver pwm_byt_driver = {
	.name	= "pwm-byt-pci",
	.id_table	= pwm_byt_pci_ids,
	.probe	= pwm_byt_probe,
	.remove	= pwm_byt_remove,
	.driver = {
		.pm = &pwm_byt_pm,
	},
};

module_pci_driver(pwm_byt_driver);

MODULE_ALIAS("pwm-byt-pci");
MODULE_AUTHOR("Wang, Zhifeng<zhifeng.wang@intel.com>");
MODULE_DESCRIPTION("Intel Baytrail PWM driver");
MODULE_LICENSE("GPL");
