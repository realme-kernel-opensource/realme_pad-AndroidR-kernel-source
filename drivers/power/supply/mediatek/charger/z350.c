/* Copyright (C) 2018 XiaoMi, Inc. All rights reserved.
 * Copyright (C) 2020 XiaoMi, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt)	"[z350] %s: " fmt, __func__

#include <linux/i2c.h>
#include <linux/debugfs.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/of.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/pinctrl/consumer.h>
#include <linux/firmware.h>

#include <mt-plat/charger_class.h>
#include <mt-plat/charger_type.h>
#include <mt-plat/upmu_common.h>

#include "mtk_charger_intf.h"
#include "mtk_charger_init.h"
#include "mtk_intf.h"

#include <linux/stat.h>
#include <linux/ctype.h>
#include <linux/irq.h>
#include <linux/irqdesc.h>
#include <linux/vmalloc.h>
#include <linux/preempt.h>

#include <linux/time.h>
#include "sgm41510.h"

#define z350_info	pr_info
#define z350_dbg	pr_debug
#define z350_err	pr_err
#define z350_log	pr_err

#define Z350_REG_RERUN_APSD		0x01
#define Z350_REG_QC_MS			0x02
#define Z350_REG_INTB_EN		0x03
#define Z350_REG_SOFT_RESET		0x04
#define Z350_REG_HVDCP_EN		0x05
#define Z350_REG_BC12_EN		0x06
#define Z350_REG_SLEEP_EN		0x07
#define Z350_REG_QC30_PM		0x73
#define Z350_REG_QC3_PLUS_PM	0x83
#define Z350_REG_CHGT_ERROR		0x11
#define Z350_REG_VIN			0x12
#define Z350_REG_VID			0x13
#define Z350_REG_VERSION		0x14

#define ENABLE		0x01
#define DISABLE		0x00

bool qc_module_z350 = false;
/*
enum qc35_error_state {
	QC35_ERROR_NO,
	QC35_ERROR_QC30_DET,
	QC35_ERROR_TA_DET,
	QC35_ERROR_TA_CAP,
	QC35_QC3_PLUS_DET_OK,
};
*/

enum z350_chg_type {
	QC35_NA = 0,
	QC35_OCP = 0x1,
	QC35_FLOAT = 0x2,
	QC35_SDP = 0x3,
	QC35_CDP = 0x4,
	QC35_DCP = 0x5,
	QC35_HVDCP_20 = 0x6,
	QC35_HVDCP_30 = 0x7,
	QC35_HVDCP_3_PLUS_18 = 0x8,
	QC35_HVDCP_3_PLUS_27 = 0x9,
	QC35_HVDCP = 0x10,
	QC35_UNKOWN = 0x11,
	QC35_HVDCP_3_PLUS_40 = 0x12,
};

#define Z350_MODE_QC20_V5			0x01
#define Z350_MODE_QC20_V9			0x02
#define Z350_MODE_QC20_V12			0x03
#define Z350_MODE_QC30_V5			0x04
#define Z350_MODE_QC3_PLUS_V5		0x05

struct z350_charger {
	struct i2c_client		*client;
	struct device			*dev;

	int			qc_rst_gpio;
	struct mutex		irq_complete;
	bool		otg_enable;
	bool		irq_waiting;
	bool		resume_completed;
	struct delayed_work		charger_type_det_work;
};

struct z350_charger *z350_chip = NULL;
/*
static struct charger_type_desc charger_type_table[] = {
	[QC35_NA]		= {POWER_SUPPLY_TYPE_UNKNOWN,		CHARGER_UNKNOWN,	USBSW_CHG},
	[QC35_FLOAT]		= {POWER_SUPPLY_TYPE_USB_FLOAT,		NONSTANDARD_CHARGER,	USBSW_CHG},
	[QC35_SDP]		= {POWER_SUPPLY_TYPE_USB,		STANDARD_HOST,		USBSW_USB},
	[QC35_CDP]		= {POWER_SUPPLY_TYPE_USB_CDP,		CHARGING_HOST,		USBSW_USB},
	[QC35_DCP]		= {POWER_SUPPLY_TYPE_USB_DCP,		STANDARD_CHARGER,	USBSW_CHG},
	[QC35_HVDCP_20]		= {POWER_SUPPLY_TYPE_USB_HVDCP,		STANDARD_CHARGER,	USBSW_CHG},
	[QC35_HVDCP_30]		= {POWER_SUPPLY_TYPE_USB_HVDCP_3,	STANDARD_CHARGER,	USBSW_CHG},
	[QC35_HVDCP_3_PLUS_18]	= {POWER_SUPPLY_TYPE_USB_HVDCP_3_PLUS,	STANDARD_CHARGER,	USBSW_CHG},
	[QC35_HVDCP_3_PLUS_27]	= {POWER_SUPPLY_TYPE_USB_HVDCP_3_PLUS,	STANDARD_CHARGER,	USBSW_CHG},
};*/

static int __z350_write_byte(struct z350_charger *chip, u8 reg, u8 val)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(chip->client, reg, val);
	if (ret < 0) {
		z350_err("%s: reg[0x%02X] write failed.\n",
		       __func__, reg);
		return ret;
	}
	return 0;
}

static int z350_write_byte(struct z350_charger *chip, u8 reg, u8 data)
{
	return __z350_write_byte(chip, reg, data);
}

static int __z350_read_block(struct z350_charger *chip, u8 reg, u8 *data)
{
	s32 ret;

	ret = i2c_smbus_read_i2c_block_data(chip->client, reg, 4, data);
	if (ret < 0) {
		z350_err("%s: reg[0x%02X] read failed.\n",
				__func__, reg);
		return ret;
	}

	return 0;
}

static int z350_read_block(struct z350_charger *chip, u8 reg, u8 *data)
{
	return __z350_read_block(chip, reg, data);
}

static int __z350_read_word(struct z350_charger *chip, u8 reg, u16 *data)
{
	s32 ret;

	ret = i2c_smbus_read_word_data(chip->client, reg);
	if (ret < 0) {
		z350_err("%s: reg[0x%02X] read failed.\n",
				__func__, reg);
		return ret;
	}

	*data = (u16) ret;

	return 0;
}

static int __z350_write_word(struct z350_charger *chip, u8 reg, u16 val)
{
	s32 ret;

	ret = i2c_smbus_write_word_data(chip->client, reg, val);
	if (ret < 0) {
		z350_err("%s: reg[0x%02X] write failed.\n",
		       __func__, reg);
		return ret;
	}
	return 0;
}

static int z350_read_word(struct z350_charger *chip, u8 reg, u16 *data)
{
	return __z350_read_word(chip, reg, data);
}


static int z350_write_word(struct z350_charger *chip, u8 reg, u16 data)
{
	return __z350_write_word(chip, reg, data);
}

static int z350_qc_mode_select(struct z350_charger *chip, u8 mode)
{
	int ret = -1;

	if (mode > Z350_MODE_QC3_PLUS_V5) {
		z350_err("%s: qc mode[%d] is wrong!\n", __func__, mode);
		return ret;
	}

	ret = z350_write_byte(chip, Z350_REG_QC_MS, mode);

	if (ret)
		z350_err("%s: select qc mode[%d] failed!\n", __func__, mode);
	else
		z350_err("%s: select qc mode[%d] success!\n", __func__, mode);

	mdelay(200);
	return ret;
}

/*
static int z350_get_error_state(struct z350_charger *chip, int *err_state)
{
	int ret;
	u16 val;

	ret = z350_read_word(chip, Z350_REG_CHGT_ERROR, &val);
	if (ret) {
		z350_err("get error/state failed!\n");
	} else {
		z350_log("get error/state success, 0x%04x!\n", val);
		*err_state = (int)((val >> 8) & 0xFF);
	}

	return ret;
}
*/

int z350_get_charger_type(u8 *chg_type)
{
	int ret;
	u16 val;

	ret = z350_read_word(z350_chip, Z350_REG_CHGT_ERROR, &val);
	if (ret) {
		z350_err("get error/state failed!\n");
		*chg_type = 0;
	} else {
		z350_log("get error/state success, 0x%04x!\n", val);
		val = (u16)(val & 0xFF);
		//enable_vbus_ovp(true);
		switch (val) {
		case QC35_NA:
		case QC35_UNKOWN:
			*chg_type = SGM41510_CHG_TYPE_NOVBUS;
			break;
		case QC35_FLOAT:
			*chg_type = SGM41510_CHG_TYPE_FLOAT;
			break;
		case QC35_SDP:
			*chg_type = SGM41510_CHG_TYPE_SDP;
			break;
		case QC35_CDP:
			*chg_type = SGM41510_CHG_TYPE_CDP;
			break;
		case QC35_DCP:
			*chg_type = SGM41510_CHG_TYPE_DCP;
			break;
		case QC35_HVDCP_20:
		case QC35_HVDCP_30:
		case QC35_HVDCP_3_PLUS_18:
		case QC35_HVDCP_3_PLUS_27:
		case QC35_HVDCP:
		case QC35_HVDCP_3_PLUS_40:
			*chg_type = SGM41510_CHG_TYPE_HVDCP;
			ret = z350_qc_mode_select(z350_chip, Z350_MODE_QC20_V5);
			ret = z350_qc_mode_select(z350_chip, Z350_MODE_QC20_V9);
			//enable_vbus_ovp(false);
			if (ret) {
				z350_err("select qc2 dpdm mode failed!\n");
				*chg_type = SGM41510_CHG_TYPE_DCP;
				//enable_vbus_ovp(true);
			}
			break;
		default:
			*chg_type = SGM41510_CHG_TYPE_MAX;
			break;
		}
	}

	return 0;
}
/*
static int z350_get_version(struct z350_charger *chip, u16 *val)
{
	int ret;
	u16 tmp = 0;

	ret = z350_read_word(chip, Z350_REG_VERSION, &tmp);
	if (ret) {
		z350_err("%s: get version failed!\n", __func__);
	} else {
		*val = (u16)(((tmp & 0xFF) << 8) | ((tmp >> 8) & 0xFF));
		z350_dbg("%s: get version success, %04x!\n", __func__, *val);
	}

	return ret;
}*/

static int z350_get_vid(struct z350_charger *chip, u8 *val)
{
	int ret;
	int vid;

	ret = z350_read_block(chip, Z350_REG_VID, val);
	if (ret) {
		z350_err("%s: get version failed!\n", __func__);
	} else {
		z350_dbg("%s: get vendor id success, %02x %02x %02x %02x!\n", __func__, val[0], val[1], val[2], val[3]);
		vid	= (int)((val[0] * 0x1000000) | (val[1] * 0x10000) | (val[2] * 0x100) | (val[3]));
		z350_log("%s: vid = %08x, %d\n", __func__, vid, vid);

		if (vid == 0x49333530) {
			ret = QC_MODULE_Z350;
		}
	}

	return ret;
}

static void z350_hard_reset(struct z350_charger *chip)
{
	z350_log("hard reset\n");
	gpio_set_value(chip->qc_rst_gpio, 1);
	mdelay(10);
	gpio_set_value(chip->qc_rst_gpio, 0);
	mdelay(60);
}

int z350_dump_register(void)
{
	int ret = 0;
	u16 data_word;

	ret = z350_read_word(z350_chip, Z350_REG_CHGT_ERROR, &data_word);
	if (!ret)
		z350_log("%s: 0x%02x = 0x%04x\n", __func__,
				Z350_REG_CHGT_ERROR, data_word);

	ret = z350_read_word(z350_chip, Z350_REG_VIN, &data_word);
	if (!ret)
		z350_log("%s: 0x%02x = 0x%04x\n", __func__,
				Z350_REG_VIN, data_word);

	ret = z350_read_word(z350_chip, Z350_REG_QC_MS, &data_word);
	if (!ret)
		z350_log("%s: 0x%02x = 0x%04x\n", __func__,
				Z350_REG_QC_MS, data_word);

	ret = z350_read_word(z350_chip, Z350_REG_VERSION, &data_word);
	if (!ret)
		z350_dbg("%s: 0x%02x = 0x%04x\n", __func__,
				Z350_REG_VERSION, data_word);

	return ret;
}

int z350_enable_chgdet_flow(bool en)
{
	int ret = 0;

	z350_err("_z350_enable_chgdet_flow enter!\n");
	if (en)
		z350_hard_reset(z350_chip);
/*
	if (!en) {
		ret = z350_write_word(z350_chip, Z350_DET_TEPY_EN, DISABLE);
		if (ret)
			z350_err("Set z350 detect type disable failed!\n");
		else
			z350_err("et z350 detect type disable success!\n");
	} else {
		ret = z350_write_word(z350_chip, Z350_DET_TEPY_EN, ENABLE);
		if (ret)
			z350_err("Set z350 detect type enable failed!\n");
		else
			z350_err("et z350 detect type enable success!\n");
	}
*/
	return ret;
}

void z350_enable_otg(bool en)
{
	int ret = 0;
	z350_info("en = %d\n", en);

	z350_chip->otg_enable = en;
	if (!en) {
		ret = z350_write_word(z350_chip, Z350_REG_SLEEP_EN, DISABLE);
		if (ret)
			z350_err("Set z350 detect type disable failed!\n");
	} else {
		ret = z350_write_word(z350_chip, Z350_REG_SLEEP_EN, ENABLE);
		if (ret)
			z350_err("Set z350 detect type enable failed!\n");
	}
	return;
}

static void z350_charger_type_det_work(struct work_struct *work)
{
	int ret = 0;
	ret = sgm41510_attachi_irq_handler();
	if (ret)
		z350_err("sgm41510_attachi_irq_handler failed!\n");
	return;
}

static irqreturn_t z350_interrupt(int irq, void *dev_id)
{
	struct z350_charger *chip = dev_id;

	z350_info("INT OCCURED\n");

	if (chip->otg_enable)
		return IRQ_HANDLED;

	mutex_lock(&chip->irq_complete);

	chip->irq_waiting = true;
	if (!chip->resume_completed) {
		z350_dbg("IRQ triggered before device-resume\n");
		disable_irq_nosync(irq);
		mutex_unlock(&chip->irq_complete);
		return IRQ_HANDLED;
	}
	schedule_delayed_work(&chip->charger_type_det_work, 0);
	chip->irq_waiting = false;

	mutex_unlock(&chip->irq_complete);
	return IRQ_HANDLED;
}

static int z350_parse_dt(struct z350_charger *chip)
{
	struct device_node *np = chip->dev->of_node;

	if (!np) {
		z350_err("device tree info missing\n");
		return -EINVAL;
	}

	chip->qc_rst_gpio = of_get_named_gpio(np,
					"mtk,qc_rst_gpio", 0);
	if ((!gpio_is_valid(chip->qc_rst_gpio))) {
		dev_err(chip->dev, "%s: no qc_rst_gpio\n", __func__);
		return -EINVAL;
	}
	z350_dbg("qc_rst_gpio: %d\n", chip->qc_rst_gpio);

	return 0;
}
/*
static int z350_pinctrl_init(struct z350_charger *chip)
{
	int ret;

	chip->z350_pinctrl = devm_pinctrl_get(chip->dev);
	if (IS_ERR_OR_NULL(chip->z350_pinctrl)) {
		ret = PTR_ERR(chip->z350_pinctrl);
		dev_err(chip->dev,
			"Target does not use pinctrl %d\n", ret);
		goto err_pinctrl_get;
	}

	chip->pinctrl_state_normal
		= pinctrl_lookup_state(chip->z350_pinctrl,
					"qc_normal");
	if (IS_ERR_OR_NULL(chip->pinctrl_state_normal)) {
		ret = PTR_ERR(chip->pinctrl_state_normal);
		dev_err(chip->dev,
			"Can not lookup qc_normal pinstate %d\n",
			ret);
		goto err_pinctrl_lookup;
	}

	chip->pinctrl_state_isp
		= pinctrl_lookup_state(chip->z350_pinctrl,
					"qc_isp");
	if (IS_ERR_OR_NULL(chip->pinctrl_state_isp)) {
		ret = PTR_ERR(chip->pinctrl_state_isp);
		dev_err(chip->dev,
			"Can not lookup qc_isp  pinstate %d\n",
			ret);
		goto err_pinctrl_lookup;
	}

	return 0;

err_pinctrl_get:
	devm_pinctrl_put(chip->z350_pinctrl);
err_pinctrl_lookup:
	chip->z350_pinctrl = NULL;

	return ret;
}
*/
static int z350_charger_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int rc;
	struct z350_charger *chip;
	u8 buf[4];

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		z350_err("Couldn't allocate memory\n");
		return -ENOMEM;
	}

	chip->client = client;
	chip->dev = &client->dev;
	mutex_init(&chip->irq_complete);

	chip->irq_waiting = false;
	chip->resume_completed = true;

	i2c_set_clientdata(client, chip);

	rc = z350_parse_dt(chip);
	if (rc) {
		z350_err("Couldn't parse DT nodes rc=%d\n", rc);
		goto err_mutex_init;
	}	
/*
	rc = z350_pinctrl_init(chip);
	if (!rc && chip->z350_pinctrl) {
		rc = pinctrl_select_state(chip->z350_pinctrl,
					chip->pinctrl_state_normal);
		if (rc < 0)
			dev_err(&client->dev,
				"%s: Failed to select active pinstate %d\n",
				__func__, rc);
	}
*/	
	z350_chip = chip;
	/*rc = devm_gpio_request(&client->dev,
				chip->qc_rst_gpio, "z350 reset gpio");
	if (rc) {
		z350_err("request z350 reset gpio failed, rc=%d\n",
				rc);
		goto err_mutex_init;
	}*/
	gpio_direction_output(chip->qc_rst_gpio, 1);
	mdelay(10);	
	gpio_set_value(chip->qc_rst_gpio, 0);
	mdelay(10);
 	rc = z350_get_vid(chip, buf);
	if (rc == QC_MODULE_Z350 ) {
		qc_module_z350 = true;
		z350_err("qc_module_z350 = true\n");
	} else {
		qc_module_z350 =false;
		z350_err("qc_module_z350 = false\n");
		goto err_mutex_init;
	}
	INIT_DELAYED_WORK(&chip->charger_type_det_work, z350_charger_type_det_work);

	if (client->irq) {
		rc = devm_request_threaded_irq(&client->dev, client->irq,
				NULL, z350_interrupt,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				"z350 irq", chip);
		if (rc < 0) {
			z350_err("request irq for irq=%d failed, rc =%d\n",
							client->irq, rc);
			goto err_mutex_init;
		}
	}
	device_init_wakeup(chip->dev, 1);

	z350_info("z350 successfully probed.\n");
	return 0;

err_mutex_init:
	mutex_destroy(&chip->irq_complete);
	return rc;
}

static int z350_charger_remove(struct i2c_client *client)
{
	struct z350_charger *chip = i2c_get_clientdata(client);

	z350_info("%s: remove\n", __func__);
	mutex_destroy(&chip->irq_complete);
	cancel_delayed_work_sync(&chip->charger_type_det_work);

	return 0;
}

static int z350_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct z350_charger *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->irq_complete);
	chip->resume_completed = false;
	mutex_unlock(&chip->irq_complete);

	return 0;
}

static int z350_suspend_noirq(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct z350_charger *chip = i2c_get_clientdata(client);

	if (chip->irq_waiting) {
		pr_err_ratelimited("Aborting suspend, an interrupt was detected while suspending\n");
		return -EBUSY;
	}
	return 0;
}

static int z350_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct z350_charger *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->irq_complete);
	chip->resume_completed = true;
	if (chip->irq_waiting) {
		mutex_unlock(&chip->irq_complete);
		enable_irq(client->irq);
	} else {
		mutex_unlock(&chip->irq_complete);
	}
	return 0;
}

static const struct dev_pm_ops z350_pm_ops = {
	.suspend	= z350_suspend,
	.suspend_noirq	= z350_suspend_noirq,
	.resume		= z350_resume,
};

static void z350_charger_shutdown(struct i2c_client *client)
{
	struct z350_charger *chip = i2c_get_clientdata(client);

	z350_info("%s: shutdown\n", __func__);
	z350_qc_mode_select(chip, Z350_MODE_QC20_V5);
	cancel_delayed_work_sync(&chip->charger_type_det_work);
	mutex_destroy(&chip->irq_complete);

}

static const struct of_device_id z350_match_table[] = {
	{ .compatible = "mediatek,z350_charger",},
	{ },
};

static const struct i2c_device_id z350_charger_id[] = {
	{"z350_charger", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, z350_charger_id);

static struct i2c_driver z350_charger_driver = {
	.driver		= {
		.name		= "z350_charger",
		.owner		= THIS_MODULE,
		.of_match_table	= z350_match_table,
		.pm		= &z350_pm_ops,
	},
	.probe		= z350_charger_probe,
	.remove		= z350_charger_remove,
	.id_table	= z350_charger_id,
	.shutdown	= z350_charger_shutdown,
};

module_i2c_driver(z350_charger_driver);

MODULE_DESCRIPTION("z350 Charger");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("i2c:z350-charger");
