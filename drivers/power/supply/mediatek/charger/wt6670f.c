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

#define pr_fmt(fmt)	"[wt6670f] %s: " fmt, __func__

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

#define wt6670f_info	pr_info
#define wt6670f_dbg	pr_debug
#define wt6670f_err	pr_err
#define wt6670f_log	pr_err

#define WT6670F_REG_RERUN_APSD		0xB0
#define WT6670F_REG_QC_MS			0xB1
#define WT6670F_REG_INTB_EN		0xB2
#define WT6670F_REG_SOFT_RESET		0xB3
#define WT6670F_REG_SLEEP_EN		0xB5
#define WT6670F_DET_TEPY_EN		0xB6
#define WT6670F_REG_QC30_PM		0xBA
#define WT6670F_REG_QC3_PLUS_PM	0xBB
#define WT6670F_REG_CHGT_ERROR		0xBD
#define WT6670F_REG_VIN			0xBE
#define WT6670F_REG_VERSION		0xBF

#define ENABLE		0x01
#define DISABLE		0x00

/*
enum qc35_error_state {
	QC35_ERROR_NO,
	QC35_ERROR_QC30_DET,
	QC35_ERROR_TA_DET,
	QC35_ERROR_TA_CAP,
	QC35_QC3_PLUS_DET_OK,
};
*/

enum wt6670f_chg_type {
	QC35_NA = 0,
	QC35_FLOAT = 0x1,
	QC35_SDP = 0x2,
	QC35_CDP = 0x3,
	QC35_DCP = 0x4,
	QC35_HVDCP_20 = 0x5,
	QC35_HVDCP_30 = 0x6,
	QC35_HVDCP_3_PLUS_18 = 0x8,
	QC35_HVDCP_3_PLUS_27 = 0x9,
};

#define WT6670F_MODE_QC20_V5			0x01
#define WT6670F_MODE_QC20_V9			0x02
#define WT6670F_MODE_QC20_V12			0x03
#define WT6670F_MODE_QC30_V5			0x04
#define WT6670F_MODE_QC3_PLUS_V5		0x05

struct wt6670f_charger {
	struct i2c_client		*client;
	struct device			*dev;

	struct pinctrl		*qc_pinctrl;
	struct pinctrl_state	*pinctrl_state_normal;
	struct pinctrl_state	*pinctrl_state_isp;
	int			qc_sda_gpio;
	int			qc_scl_gpio;
	int			qc_rst_gpio;
	struct mutex		irq_complete;
	struct mutex		isp_sequence_lock;
	bool		otg_enable;
	bool		irq_waiting;
	bool		resume_completed;
	bool		in_fw_upgrading;
	struct delayed_work		charger_type_det_work;
	struct delayed_work		chip_update_work;
	struct power_supply		*battery_psy;
};

struct wt6670f_charger *wt6670f_chip = NULL;
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

static int __wt6670f_write_byte(struct wt6670f_charger *chip, u8 reg, u8 val)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(chip->client, reg, val);
	if (ret < 0) {
		wt6670f_err("%s: reg[0x%02X] write failed.\n",
		       __func__, reg);
		return ret;
	}
	return 0;
}

static int wt6670f_write_byte(struct wt6670f_charger *chip, u8 reg, u8 data)
{
	return __wt6670f_write_byte(chip, reg, data);
}

static int __wt6670f_read_word(struct wt6670f_charger *chip, u8 reg, u16 *data)
{
	s32 ret;

	ret = i2c_smbus_read_word_data(chip->client, reg);
	if (ret < 0) {
		wt6670f_err("%s: reg[0x%02X] read failed.\n",
				__func__, reg);
		return ret;
	}

	*data = (u16) ret;

	return 0;
}

static int __wt6670f_write_word(struct wt6670f_charger *chip, u8 reg, u16 val)
{
	s32 ret;

	ret = i2c_smbus_write_word_data(chip->client, reg, val);
	if (ret < 0) {
		wt6670f_err("%s: reg[0x%02X] write failed.\n",
		       __func__, reg);
		return ret;
	}
	return 0;
}

static int wt6670f_read_word(struct wt6670f_charger *chip, u8 reg, u16 *data)
{
	return __wt6670f_read_word(chip, reg, data);
}


static int wt6670f_write_word(struct wt6670f_charger *chip, u8 reg, u16 data)
{
	return __wt6670f_write_word(chip, reg, data);
}

static int wt6670f_qc_mode_select(struct wt6670f_charger *chip, u8 mode)
{
	int ret = -1;

	if (mode > WT6670F_MODE_QC3_PLUS_V5) {
		wt6670f_err("%s: qc mode[%d] is wrong!\n", __func__, mode);
		return ret;
	}

	ret = wt6670f_write_byte(chip, WT6670F_REG_QC_MS, mode);

	if (ret)
		wt6670f_err("%s: select qc mode[%d] failed!\n", __func__, mode);
	else
		wt6670f_err("%s: select qc mode[%d] success!\n", __func__, mode);

	mdelay(200);
	return ret;
}

/*
static int wt6670f_get_error_state(struct wt6670f_charger *chip, int *err_state)
{
	int ret;
	u16 val;

	ret = wt6670f_read_word(chip, WT6670F_REG_CHGT_ERROR, &val);
	if (ret) {
		wt6670f_err("get error/state failed!\n");
	} else {
		wt6670f_log("get error/state success, 0x%04x!\n", val);
		*err_state = (int)((val >> 8) & 0xFF);
	}

	return ret;
}
*/
int wt6670f_get_charger_type(u8 *chg_type)
{
	int ret;
	u16 val;

	ret = wt6670f_read_word(wt6670f_chip, WT6670F_REG_CHGT_ERROR, &val);
	if (ret) {
		wt6670f_err("get error/state failed!\n");
		*chg_type = 0;
	} else {
		wt6670f_log("get error/state success, 0x%04x!\n", val);
		val = (u16)(val & 0xF);
		//enable_vbus_ovp(true);
		switch (val) {
		case QC35_NA:
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
			*chg_type = SGM41510_CHG_TYPE_HVDCP;
			ret = wt6670f_qc_mode_select(wt6670f_chip, WT6670F_MODE_QC20_V5);
			ret = wt6670f_qc_mode_select(wt6670f_chip, WT6670F_MODE_QC20_V9);
			//enable_vbus_ovp(false);
			if (ret) {
				wt6670f_err("select qc2 dpdm mode failed!\n");
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

static int wt6670f_get_version(struct wt6670f_charger *chip, u16 *val)
{
	int ret;
	u16 tmp = 0;

	ret = wt6670f_read_word(chip, WT6670F_REG_VERSION, &tmp);
	if (ret) {
		wt6670f_err("%s: get version failed!\n", __func__);
	} else {
		*val = (u16)(((tmp & 0xFF) << 8) | ((tmp >> 8) & 0xFF));
		wt6670f_dbg("%s: get version success, %04x!\n", __func__, *val);
	}

	return ret;
}

static void wt6670f_hard_reset(struct wt6670f_charger *chip)
{
	wt6670f_log("hard reset\n");
	gpio_set_value(chip->qc_rst_gpio, 1);
	mdelay(10);
	gpio_set_value(chip->qc_rst_gpio, 0);
	mdelay(60);
}

int wt6670f_dump_register(void)
{
	int ret = 0;
	u16 data_word;

	ret = wt6670f_read_word(wt6670f_chip, WT6670F_REG_CHGT_ERROR, &data_word);
	if (!ret)
		wt6670f_log("%s: 0x%02x = 0x%04x\n", __func__,
				WT6670F_REG_CHGT_ERROR, data_word);

	ret = wt6670f_read_word(wt6670f_chip, WT6670F_REG_VIN, &data_word);
	if (!ret)
		wt6670f_log("%s: 0x%02x = 0x%04x\n", __func__,
				WT6670F_REG_VIN, data_word);

	ret = wt6670f_read_word(wt6670f_chip, WT6670F_REG_QC_MS, &data_word);
	if (!ret)
		wt6670f_log("%s: 0x%02x = 0x%04x\n", __func__,
				WT6670F_REG_QC_MS, data_word);

	ret = wt6670f_read_word(wt6670f_chip, WT6670F_REG_VERSION, &data_word);
	if (!ret)
		wt6670f_dbg("%s: 0x%02x = 0x%04x\n", __func__,
				WT6670F_REG_VERSION, data_word);

	return ret;
}

int wt6670f_enable_chgdet_flow(bool en)
{
	int ret = 0;

	wt6670f_err("_wt6670f_enable_chgdet_flow enter!\n");
	if (wt6670f_chip->in_fw_upgrading) {
		wt6670f_err("wt6670f_enable_chgdet_flow in_fw_upgrading!\n");
		return ret;
	}
	if (en)
		wt6670f_hard_reset(wt6670f_chip);
/*
	if (!en) {
		ret = wt6670f_write_word(wt6670f_chip, WT6670F_DET_TEPY_EN, DISABLE);
		if (ret)
			wt6670f_err("Set wt6670f detect type disable failed!\n");
		else
			wt6670f_err("et wt6670f detect type disable success!\n");
	} else {
		ret = wt6670f_write_word(wt6670f_chip, WT6670F_DET_TEPY_EN, ENABLE);
		if (ret)
			wt6670f_err("Set wt6670f detect type enable failed!\n");
		else
			wt6670f_err("et wt6670f detect type enable success!\n");
	}
*/
	return ret;
}

void wt6670f_enable_otg(bool en)
{
	int ret = 0;
	wt6670f_info("en = %d\n", en);

	wt6670f_chip->otg_enable = en;
	if (!en) {
		ret = wt6670f_write_word(wt6670f_chip, WT6670F_REG_SLEEP_EN, DISABLE);
		if (ret)
			wt6670f_err("Set wt6670f detect type disable failed!\n");
	} else {
		ret = wt6670f_write_word(wt6670f_chip, WT6670F_REG_SLEEP_EN, ENABLE);
		if (ret)
			wt6670f_err("Set wt6670f detect type enable failed!\n");
	}
	return;
}

static void wt6670f_charger_type_det_work(struct work_struct *work)
{
	int ret = 0;
	ret = sgm41510_attachi_irq_handler();
	if (ret)
		wt6670f_err("sgm41510_attachi_irq_handler failed!\n");
	return;
}

static irqreturn_t wt6670f_interrupt(int irq, void *dev_id)
{
	struct wt6670f_charger *chip = dev_id;

	wt6670f_info("INT OCCURED\n");

	if (chip->in_fw_upgrading) {
		wt6670f_err("wt6670f_interrupt in_fw_upgrading!\n");
		return IRQ_HANDLED;
	}
	if (chip->otg_enable)
		return IRQ_HANDLED;

	mutex_lock(&chip->irq_complete);

	chip->irq_waiting = true;
	if (!chip->resume_completed) {
		wt6670f_dbg("IRQ triggered before device-resume\n");
		disable_irq_nosync(irq);
		mutex_unlock(&chip->irq_complete);
		return IRQ_HANDLED;
	}
	schedule_delayed_work(&chip->charger_type_det_work, 0);
	chip->irq_waiting = false;

	mutex_unlock(&chip->irq_complete);
	return IRQ_HANDLED;
}

static int wt6670f_parse_dt(struct wt6670f_charger *chip)
{
	struct device_node *np = chip->dev->of_node;

	if (!np) {
		wt6670f_err("device tree info missing\n");
		return -EINVAL;
	}

	chip->qc_rst_gpio = of_get_named_gpio(np,
					"mtk,qc_rst_gpio", 0);
	if ((!gpio_is_valid(chip->qc_rst_gpio))) {
		dev_err(chip->dev, "%s: no qc_rst_gpio\n", __func__);
		return -EINVAL;
	}
	wt6670f_dbg("qc_rst_gpio: %d\n", chip->qc_rst_gpio);

	chip->qc_sda_gpio = of_get_named_gpio(np,
					"mtk,qc_sda_gpio", 0);
	if ((!gpio_is_valid(chip->qc_sda_gpio))) {
		dev_err(chip->dev, "%s: no qc_sda_gpio\n", __func__);
	}
	wt6670f_dbg("qc_sda_gpio: %d\n", chip->qc_sda_gpio);

	chip->qc_scl_gpio = of_get_named_gpio(np,
					"mtk,qc_scl_gpio", 0);
	if ((!gpio_is_valid(chip->qc_scl_gpio))) {
		dev_err(chip->dev, "%s: no qc_scl_gpio\n", __func__);
	}
	wt6670f_dbg("qc_scl_gpio: %d\n", chip->qc_scl_gpio);

	return 0;
}

static int qc_pinctrl_init(struct wt6670f_charger *chip)
{
	int ret;

	chip->qc_pinctrl = devm_pinctrl_get(chip->dev);
	if (IS_ERR_OR_NULL(chip->qc_pinctrl)) {
		ret = PTR_ERR(chip->qc_pinctrl);
		dev_err(chip->dev,
			"Target does not use pinctrl %d\n", ret);
		goto err_pinctrl_get;
	}

	chip->pinctrl_state_normal
		= pinctrl_lookup_state(chip->qc_pinctrl,
					"qc_normal");
	if (IS_ERR_OR_NULL(chip->pinctrl_state_normal)) {
		ret = PTR_ERR(chip->pinctrl_state_normal);
		dev_err(chip->dev,
			"Can not lookup qc_normal pinstate %d\n",
			ret);
		goto err_pinctrl_lookup;
	}

	chip->pinctrl_state_isp
		= pinctrl_lookup_state(chip->qc_pinctrl,
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
	devm_pinctrl_put(chip->qc_pinctrl);
err_pinctrl_lookup:
	chip->qc_pinctrl = NULL;

	return ret;
}

char wt6670f_addr = 0x2B;
char wt6670f_data = 0x48;
char wt6670f_addr_real = 0x34;

int wt6670f_i2c_master_send(const struct i2c_client *client, const char *buf, int count)
{
	int ret;
	struct i2c_adapter *adap = client->adapter;
	struct i2c_msg msg;

	msg.addr = wt6670f_addr_real;
	msg.flags = 0x0000;
	msg.flags = msg.flags | I2C_M_STOP;
	msg.len = count;
	msg.buf = (char *)buf;

	ret = i2c_transfer(adap, &msg, 1);

	return (ret == 1) ? count : ret;
}

int wt6670f_i2c_get_chip_id(const struct i2c_client *client, const char *buf)
{
	int ret;
	struct i2c_adapter *adap = client->adapter;
	struct i2c_msg msg[2];
	char send_buf[2] = {0x80, 0x00};

	msg[0].addr = wt6670f_addr_real;
	msg[0].len = 2;
	msg[0].buf = send_buf;

	msg[1].addr = wt6670f_addr_real;
	msg[1].flags = I2C_M_STOP & I2C_M_NOSTART;
	msg[1].flags |= I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = (char *)buf;

	ret = i2c_transfer(adap, &msg[0], 2);

	return ret;
}
/*
static ssize_t isp_reset_store(struct device *dev,
struct device_attribute *attr,
const char *buf, size_t count)
{
	int cmd = 0;
	struct wt6670f_charger *chip = dev_get_drvdata(dev);

	if (sscanf(buf, "%d", &cmd) != 1)
		return -EINVAL;

	wt6670f_hard_reset(chip);

	dev_err(&chip->client->dev, "%s: reset finish.\n",
			__func__);
	return count;
}*/

static int wt6670f_i2c_sequence_send(const struct i2c_client *client, const char *buf, int count)
{
	int ret;
	struct i2c_adapter *adap = client->adapter;
	struct i2c_msg msg;

	msg.addr = wt6670f_addr;
	//msg.flags = 0x0000;
	msg.flags = 0x0000 | I2C_M_NO_RD_ACK | I2C_M_IGNORE_NAK | I2C_M_NOSTART;
	msg.len = count;
	msg.buf = (char *)buf;

	ret = i2c_transfer(adap, &msg, 1);

	return (ret == 1) ? count : ret;

}

static char enter_isp_and_get_chipid(struct device *dev)
{
	struct wt6670f_charger *chip = dev_get_drvdata(dev);
	char chip_id = 0x00;
	int rc = 0;
	//char i2c_buf[1] = {0};
	char enable_isp_cmd[7] = {0x57, 0x54, 0x36, 0x36, 0x37, 0x30, 0x46};

	mutex_lock(&chip->isp_sequence_lock);
	dev_err(chip->dev, "%s: step 1\n", __func__);
	gpio_set_value(chip->qc_rst_gpio, 1);
	mdelay(10);
	gpio_set_value(chip->qc_rst_gpio, 0);
	mdelay(1);

	//dev_err(chip->dev, "%s: step 2\n", __func__);
	dev_err(chip->dev, "%s: step 2.1: select pinstate\n", __func__);
	rc = pinctrl_select_state(chip->qc_pinctrl,
				chip->pinctrl_state_isp);
	if (rc < 0) {
		dev_err(chip->dev,
			"%s: Failed to select isp pinstate %d\n",
			__func__, rc);
		goto error;
	}

	dev_err(chip->dev, "%s: step 2.2: request 2 gpio\n", __func__);
	rc = devm_gpio_request(chip->dev,
				chip->qc_sda_gpio, "qc_sda_gpio");
	if (rc) {
		wt6670f_err("request qc_sda_gpio failed, rc=%d\n",
				rc);
		goto error;
	}
	rc = devm_gpio_request(chip->dev,
				chip->qc_scl_gpio, "qc_scl_gpio");
	if (rc) {
		wt6670f_err("request qc_scl_gpio failed, rc=%d\n",
				rc);
		goto error;
	}

	gpio_direction_output(chip->qc_sda_gpio, 1);
	gpio_direction_output(chip->qc_scl_gpio, 1);

	gpio_set_value(chip->qc_scl_gpio, 0);
	gpio_set_value(chip->qc_sda_gpio, 0);
	udelay(5);

	gpio_set_value(chip->qc_scl_gpio, 1);
	gpio_set_value(chip->qc_sda_gpio, 0);
	udelay(5);
	gpio_set_value(chip->qc_scl_gpio, 0);
	gpio_set_value(chip->qc_sda_gpio, 1);
	udelay(5);
	gpio_set_value(chip->qc_scl_gpio, 1);
	udelay(5);
	gpio_set_value(chip->qc_scl_gpio, 0);
	gpio_set_value(chip->qc_sda_gpio, 0);
	udelay(5);
	gpio_set_value(chip->qc_scl_gpio, 1);
	udelay(5);
	gpio_set_value(chip->qc_scl_gpio, 0);
	gpio_set_value(chip->qc_sda_gpio, 1);
	udelay(5);
	gpio_set_value(chip->qc_scl_gpio, 1);
	udelay(5);
	gpio_set_value(chip->qc_scl_gpio, 0);
	gpio_set_value(chip->qc_sda_gpio, 0);
	udelay(5);
	gpio_set_value(chip->qc_scl_gpio, 1);
	udelay(5);
	gpio_set_value(chip->qc_scl_gpio, 0);
	gpio_set_value(chip->qc_sda_gpio, 1);
	udelay(5);
	gpio_set_value(chip->qc_scl_gpio, 1);
	udelay(5);
	gpio_set_value(chip->qc_scl_gpio, 0);
	gpio_set_value(chip->qc_sda_gpio, 1);
	udelay(5);
	gpio_set_value(chip->qc_scl_gpio, 1);
	udelay(5);
	gpio_set_value(chip->qc_scl_gpio, 0);
	gpio_set_value(chip->qc_sda_gpio, 0);
	udelay(5);
	gpio_set_value(chip->qc_scl_gpio, 1);
	udelay(5);
	gpio_set_value(chip->qc_scl_gpio, 0);
	gpio_set_value(chip->qc_sda_gpio, 1);
	udelay(5);
	gpio_set_value(chip->qc_scl_gpio, 1);
	udelay(5);
	gpio_set_value(chip->qc_scl_gpio, 0);
	gpio_set_value(chip->qc_sda_gpio, 0);
	udelay(5);
	gpio_set_value(chip->qc_scl_gpio, 1);
	udelay(5);
	gpio_set_value(chip->qc_scl_gpio, 0);
	gpio_set_value(chip->qc_sda_gpio, 1);
	udelay(5);
	gpio_set_value(chip->qc_scl_gpio, 1);
	udelay(5);
	gpio_set_value(chip->qc_scl_gpio, 0);
	gpio_set_value(chip->qc_sda_gpio, 0);
	udelay(5);
	gpio_set_value(chip->qc_scl_gpio, 1);
	udelay(5);
	gpio_set_value(chip->qc_scl_gpio, 0);
	gpio_set_value(chip->qc_sda_gpio, 0);
	udelay(5);
	gpio_set_value(chip->qc_scl_gpio, 1);
	udelay(5);
	gpio_set_value(chip->qc_scl_gpio, 0);
	gpio_set_value(chip->qc_sda_gpio, 1);
	udelay(5);
	gpio_set_value(chip->qc_scl_gpio, 1);
	udelay(5);
	gpio_set_value(chip->qc_scl_gpio, 0);
	gpio_set_value(chip->qc_sda_gpio, 0);
	udelay(5);
	gpio_set_value(chip->qc_scl_gpio, 1);
	udelay(5);
	gpio_set_value(chip->qc_scl_gpio, 0);
	udelay(5);
	gpio_set_value(chip->qc_scl_gpio, 1);
	udelay(5);
	gpio_set_value(chip->qc_scl_gpio, 0);

	mdelay(10);

	gpio_set_value(chip->qc_scl_gpio, 1);
	gpio_set_value(chip->qc_sda_gpio, 1);

	devm_gpio_free(chip->dev,
				chip->qc_sda_gpio);
	devm_gpio_free(chip->dev,
				chip->qc_scl_gpio);
	dev_err(chip->dev, "%s: step 2.2: select pinstate normal.\n", __func__);
	rc = pinctrl_select_state(chip->qc_pinctrl,
				chip->pinctrl_state_normal);
	if (rc < 0) {
		dev_err(chip->dev,
			"%s: Failed to select normal pinstate %d\n",
			__func__, rc);
		goto error;
	}

	mutex_unlock(&chip->isp_sequence_lock);
	dev_err(chip->dev, "%s: step 3\n", __func__);
	rc = wt6670f_i2c_master_send(chip->client, enable_isp_cmd, 7);

	dev_err(chip->dev, "%s: step 4\n", __func__);
	rc = wt6670f_i2c_get_chip_id(chip->client, &chip_id);
	dev_err(chip->dev, "%s: rc=%d, chip_id=%02x\n", __func__, rc, chip_id);
	return chip_id;

error:
	pinctrl_select_state(chip->qc_pinctrl,
					chip->pinctrl_state_normal);
	devm_gpio_free(chip->dev,
				chip->qc_sda_gpio);
	devm_gpio_free(chip->dev,
				chip->qc_scl_gpio);
	return chip_id;
}

static ssize_t isp_pinctrl_show(struct device *dev,
struct device_attribute *attr, char *buf)
{
	char chip_id = 0x00;
	int rc = 0;
	u16 version = 0;
	//char i2c_buf_enable_cmd[7] = {0x57, 0x54, 0x36, 0x36, 0x37, 0x30, 0x46};
	struct wt6670f_charger *chip = dev_get_drvdata(dev);

	rc = wt6670f_get_version(chip, &version);
	if ((rc) || (version != 0x0300)) {
		chip_id = enter_isp_and_get_chipid(chip->dev);
	}
	return scnprintf(buf, PAGE_SIZE, "%02x\n", chip_id);
}

static ssize_t enter_isp_and_get_chipid_show(struct device *dev,
struct device_attribute *attr, char *buf)
{
	char chip_id = 0x00;
	int rc = 0;
	char i2c_buf[1] = {0};
	char i2c_buf_enable_cmd[7] = {0x57, 0x54, 0x36, 0x36, 0x37, 0x30, 0x46};

	struct wt6670f_charger *chip = dev_get_drvdata(dev);

	i2c_buf[0] = wt6670f_data;
	dev_err(chip->dev, "%s: step 1\n", __func__);
	gpio_set_value(chip->qc_rst_gpio, 1);
	mdelay(15);
	gpio_set_value(chip->qc_rst_gpio, 0);
	mdelay(1);

	wt6670f_i2c_sequence_send(chip->client, i2c_buf, 1);
	mdelay(10);
	rc = wt6670f_i2c_master_send(chip->client, i2c_buf_enable_cmd, 7);

	rc = wt6670f_i2c_get_chip_id(chip->client, &chip_id);
	dev_err(chip->dev, "%s: rc=%d, chip_id=%02x\n", __func__, rc, chip_id);

	return scnprintf(buf, PAGE_SIZE, "%02x\n", chip_id);
}

unsigned char AnalyseHEX(const u8 *inputHexData, int len)
{
    unsigned char m_checksum = 0x00;
	int i = 0;

    for(i = 0; i < len; i++) {
       m_checksum += *inputHexData;
       inputHexData++;
    }
    m_checksum %= 256;
    m_checksum = 0x100 - m_checksum;

    return m_checksum;
}

int wt6670f_i2c_read_cmd(const struct i2c_client *client, char read_addr, const char *buf)
{
	int ret;
	struct i2c_adapter *adap = client->adapter;
	struct i2c_msg msg[2];
	char send_buf[2] = {0x61, 0x00};

	send_buf[1] = read_addr;

	msg[0].addr = wt6670f_addr_real;
	msg[0].len = 2;
	msg[0].buf = send_buf;

	msg[1].addr = wt6670f_addr_real;
	msg[1].flags = I2C_M_STOP & I2C_M_NOSTART;
	msg[1].flags |= I2C_M_RD;
	msg[1].len = 64;
	msg[1].buf = (char *)buf;

	ret = i2c_transfer(adap, &msg[0], 2);

	return ret;
}

#define wt6670f_FW_NAME		"WT6670F_QC3p.bin"
#define wt6670f_FW_NAME_BACKUP		"WT6670F_QC3p_backup.bin"

#define FIRMWARE_FILE_LENGTH			15000
#define UPDATE_SUCCESS				0
#define ERROR_REQUESET_FIRMWARE			-1
#define ERROR_CHECK_FIREWARE_FORMAT		-2
#define ERROR_GET_CHIPID_FAILED			-3
#define ERROR_ERASE_FAILED			-4
#define ERROR_FINISH_CMD_FAILED			-5
#define ERROR_FILE_CRC_FAILED			-6
#define ERROR_HIGHADD_CMD_FAILED		-7
#define ERROR_PROGRAM_CMD_FAILED		-8
#define ERROR_CALLBACK_FAILED			-9
#define FIRMWARE_LATEST				-10
static int load_fw(struct device *dev, const char *fn, bool force)
{
	struct wt6670f_charger *chip = dev_get_drvdata(dev);
	const struct firmware *fw = NULL;
	unsigned int pos = 0;
	char chip_id = 0x00;
	int rc = 0;
	//char i2c_buf[1] = {0};
	//char enable_isp_cmd[7] = {0x57, 0x54, 0x36, 0x36, 0x37, 0x30, 0x46};
	char enable_isp_flash_mode_cmd[3] = {0x10, 0x02, 0x08};
	char chip_erase_cmd[3] = {0x20, 0x00, 0x00};
	char finish_cmd[3] = {0x00, 0x00, 0x00};
	char set_addr_high_byte_cmd[3] = {0x10, 0x01, 0x00};
	char program_cmd[68] = {0x00};
	//char end_cmd[3] = {0x10, 0x00, 0x02};
	char high_addr = 0;
	char low_addr = 0;
	//char length = 0;
	//int i = 0;
	int j = 0;
	char mem_data[64] = {0x00};
	unsigned int pos_file = 0;
	u8 *file_data;
	//char flash_addr = 0;
	u16 chip_version = 0, file_version = 0;
	//int pinv = 0;

	chip->in_fw_upgrading = true;
	if (in_interrupt())
		file_data = kmalloc(FIRMWARE_FILE_LENGTH, GFP_ATOMIC);
	else
		file_data = kmalloc(FIRMWARE_FILE_LENGTH, GFP_KERNEL);

	memset(file_data, 0, FIRMWARE_FILE_LENGTH);

	rc = request_firmware(&fw, fn, dev);
	if (rc) {
		dev_err(dev, "Unable to open firmware %s\n", fn);
		rc = ERROR_REQUESET_FIRMWARE;
		goto release_firmware;
	}

	file_version = (u16)(((fw->data[62] & 0x0F) << 8) | fw->data[63]);
	dev_err(chip->dev, "%s: firmware file version is %02x%02x, %04x\n", __func__,
		fw->data[62], fw->data[63], file_version);

	gpio_set_value(chip->qc_rst_gpio, 1);
	mdelay(15);
	gpio_set_value(chip->qc_rst_gpio, 0);
	mdelay(15);
	rc = wt6670f_get_version(chip, &chip_version);
	if (rc) {
		dev_err(dev, "get chip version fail\n");
		rc = ERROR_REQUESET_FIRMWARE;
		//goto release_firmware;
	}
	dev_err(chip->dev, "%s: rc=%d, chip_version=%02x\n", __func__, rc, chip_version);

	if ((force == false) && (file_version == chip_version)) {
		dev_err(chip->dev, "%s: chip and file version is same, no need update.\n", __func__);
		rc = FIRMWARE_LATEST;
		goto release_firmware;
	}

	//wt6670f_addr = 0x2B;
	//wt6670f_data = 0x48;
	//i2c_buf[0] = wt6670f_data;
	program_cmd[0] = 0x41;

	/*dev_dbg(chip->dev, "%s: step 1\n", __func__);
	gpio_set_value(chip->qc_rst_gpio, 1);
	mdelay(15);
	gpio_set_value(chip->qc_rst_gpio, 0);
	mdelay(3);

	wt6670f_i2c_sequence_send(chip->client, i2c_buf, 1);
	mdelay(10);

	rc = wt6670f_i2c_master_send(chip->client, enable_isp_cmd, 7);
	if (rc != 7)
		dev_err(chip->dev, "%s: enable_isp_cmd is failed, rc=%d\n", __func__, rc);

	wt6670f_i2c_get_chip_id(chip->client, &chip_id);*/
	chip_id = enter_isp_and_get_chipid(chip->dev);
	dev_err(chip->dev, "%s: rc=%d, chip_id=%02x\n", __func__, rc, chip_id);

	if (chip_id == 0x70) {
		rc = wt6670f_i2c_master_send(chip->client, enable_isp_flash_mode_cmd, 3);
		dev_err(chip->dev, "%s: enable_isp_flash_mode_cmd, rc=%d\n", __func__, rc);

		rc = wt6670f_i2c_master_send(chip->client, chip_erase_cmd, 3);
		if (rc != 3) {
			dev_err(chip->dev, "%s: chip_erase_cmd is failed, rc=%d\n", __func__, rc);
			rc = ERROR_ERASE_FAILED;
			goto update_failed;
		}
		mdelay(20);

		rc = wt6670f_i2c_master_send(chip->client, finish_cmd, 3);
		if (rc != 3) {
			dev_err(chip->dev, "%s: finish_cmd is failed, rc=%d\n", __func__, rc);
			rc = ERROR_FINISH_CMD_FAILED;
			goto update_failed;
		}

		while (pos < (fw->size - 1)) {
			high_addr = pos/256;
			low_addr = pos%256;
			dev_dbg(chip->dev, "%s: high_addr = %02x, low_addr = %02x, fw->size = %d\n",
					__func__, high_addr, low_addr, fw->size);
			set_addr_high_byte_cmd[2] = high_addr;
			rc = wt6670f_i2c_master_send(chip->client, set_addr_high_byte_cmd, 3);
			if (rc != 3) {
				dev_err(chip->dev, "%s: set_addr_high_byte_cmd is failed, rc=%d\n", __func__, rc);
				rc = ERROR_HIGHADD_CMD_FAILED;
				goto update_failed;
			}

			program_cmd[1] = low_addr;
			memcpy(program_cmd + 2, fw->data + pos, 64);
			memcpy(file_data + pos_file, fw->data + pos, 64);
			pos_file += 64;
			rc = wt6670f_i2c_master_send(chip->client, program_cmd, 64 + 2);
			if (rc != 64 + 2) {
				dev_err(chip->dev, "%s: program_cmd is failed, rc=%d\n", __func__, rc);
				rc = ERROR_PROGRAM_CMD_FAILED;
				goto update_failed;
			}

			rc = wt6670f_i2c_master_send(chip->client, finish_cmd, 3);
			if (rc != 3) {
				dev_err(chip->dev, "%s: finish_cmd is failed, rc=%d\n", __func__, rc);
				rc = ERROR_FINISH_CMD_FAILED;
				goto update_failed;
			}

			pos = pos + 64;
			dev_dbg(chip->dev, "%s: pos=%d\n", __func__, pos);
		}
		pos = 0;
		while (pos < (fw->size - 1)) {
			high_addr = pos/256;
			low_addr = pos%256;
			set_addr_high_byte_cmd[2] = high_addr;
			rc = wt6670f_i2c_master_send(chip->client, set_addr_high_byte_cmd, 3);
			if (rc != 3) {
				dev_err(chip->dev, "%s: set_addr_high_byte_cmd is failed, rc=%d\n", __func__, rc);
				rc = ERROR_HIGHADD_CMD_FAILED;
				goto update_failed;
			}

			wt6670f_i2c_read_cmd(chip->client, low_addr, mem_data);
			for (j = 0; j < 64; j++) {
				dev_dbg(chip->dev, "%s: mem_data[%d]=%02x\n", __func__, j + low_addr + high_addr*256, mem_data[j]);
				dev_dbg(chip->dev, "%s: file_data[%d]=%02x\n", __func__, j + low_addr + high_addr*256, file_data[j + low_addr + high_addr*256]);
				if (file_data[j + low_addr + high_addr*256] != mem_data[j]) {
					dev_err(chip->dev, "%s: flash data is wrong.\n", __func__);
					rc = ERROR_CALLBACK_FAILED;
					goto update_failed;
				}
			}
			pos = pos + 64;
		}

		rc = UPDATE_SUCCESS;
	} else {
		dev_err(chip->dev, "%s: chip id is not right, and end update.\n", __func__);
		rc = ERROR_GET_CHIPID_FAILED;
		goto update_failed;
	}

update_failed:
	wt6670f_hard_reset(chip);
release_firmware:
	release_firmware(fw);
	kfree(file_data);
	chip->in_fw_upgrading = false;
	return rc;
}

#define CHIP_UPDATE_DELAY	10000
static void wt6670f_chip_update_work(struct work_struct *work)
{
	struct wt6670f_charger *chip = container_of(work, struct wt6670f_charger,
						chip_update_work.work);
	int error = 0, rc = 0;
	static int count = 0;
	union power_supply_propval val = {0,};

	if (!chip->battery_psy)
		chip->battery_psy = power_supply_get_by_name("battery");

	if (!chip->battery_psy) {
		wt6670f_err("get battery psy fail\n");
		return;
	}

	rc = power_supply_get_property(chip->battery_psy, POWER_SUPPLY_PROP_CAPACITY, &val);
	if (rc < 0 || val.intval <= 3) {
		wt6670f_err("read soc fail or soc is too low, rc = %d, soc = %d\n", rc, val.intval);
		count++;
		if (count < 5)
			schedule_delayed_work(&chip->chip_update_work, msecs_to_jiffies(CHIP_UPDATE_DELAY));
		return;
	}

	error = load_fw(chip->dev, wt6670f_FW_NAME, false);
	if (error == FIRMWARE_LATEST) {
		wt6670f_err("firmware is latest\n");
		return;
	} else if (error == UPDATE_SUCCESS) {
		wt6670f_err("update firmware success\n");
		return;
	} else {
		wt6670f_err("update firmware fail, try backup firmware, error = %d\n", error);
		error = load_fw(chip->dev, wt6670f_FW_NAME_BACKUP, false);
		if (error == UPDATE_SUCCESS) {
			wt6670f_err("update backup firmware success%d\n");
			return;
		} else {
			wt6670f_err("update backup firmware fail, error = %d\n", error);
			return;
		}
	}
}

static DEVICE_ATTR(enter_isp_and_get_chipid, S_IRUGO,
		enter_isp_and_get_chipid_show,
		NULL);

static DEVICE_ATTR(isp_pinctrl, S_IRUGO,
		isp_pinctrl_show,
		NULL);

static struct attribute *wt6670f_attributes[] = {
	&dev_attr_enter_isp_and_get_chipid.attr,
	&dev_attr_isp_pinctrl.attr,
	NULL,
};

static const struct attribute_group wt6670f_attr_group = {
	.attrs = wt6670f_attributes,
};
static int wt6670f_charger_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int rc;
	struct wt6670f_charger *chip;

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		wt6670f_err("Couldn't allocate memory\n");
		return -ENOMEM;
	}

	chip->client = client;
	chip->dev = &client->dev;
	mutex_init(&chip->irq_complete);
	mutex_init(&chip->isp_sequence_lock);

	chip->irq_waiting = false;
	chip->resume_completed = true;
	chip->in_fw_upgrading = false;

	i2c_set_clientdata(client, chip);

	rc = wt6670f_parse_dt(chip);
	if (rc) {
		wt6670f_err("Couldn't parse DT nodes rc=%d\n", rc);
		goto err_mutex_init;
	}

	rc = qc_pinctrl_init(chip);
	if (!rc && chip->qc_pinctrl) {
		rc = pinctrl_select_state(chip->qc_pinctrl,
					chip->pinctrl_state_normal);
		if (rc < 0)
			dev_err(&client->dev,
				"%s: Failed to select active pinstate %d\n",
				__func__, rc);
	}
	wt6670f_chip = chip;
	/*rc = devm_gpio_request(&client->dev,
				chip->qc_rst_gpio, "wt6670f reset gpio");
	if (rc) {
		wt6670f_err("request wt6670f reset gpio failed, rc=%d\n",
				rc);
		goto err_mutex_init;
	}*/
	gpio_direction_output(chip->qc_rst_gpio, 1);
	gpio_set_value(chip->qc_rst_gpio, 0);

	if (client->irq) {
		rc = devm_request_threaded_irq(&client->dev, client->irq,
				NULL, wt6670f_interrupt,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				"wt6670f irq", chip);
		if (rc < 0) {
			wt6670f_err("request irq for irq=%d failed, rc =%d\n",
							client->irq, rc);
			goto err_mutex_init;
		}
	}

	INIT_DELAYED_WORK(&chip->chip_update_work, wt6670f_chip_update_work);
	schedule_delayed_work(&chip->chip_update_work, msecs_to_jiffies(CHIP_UPDATE_DELAY));
	INIT_DELAYED_WORK(&chip->charger_type_det_work, wt6670f_charger_type_det_work);

	device_init_wakeup(chip->dev, 1);
	rc = sysfs_create_group(&chip->dev->kobj, &wt6670f_attr_group);
	if (rc) {
		wt6670f_err("Failed to register sysfs, err:%d\n", rc);
		goto err_mutex_init;
	}
	wt6670f_info("wt6670f successfully probed.\n");
	return 0;

err_mutex_init:
	mutex_destroy(&chip->irq_complete);
	mutex_destroy(&chip->isp_sequence_lock);
	return rc;
}

static int wt6670f_charger_remove(struct i2c_client *client)
{
	struct wt6670f_charger *chip = i2c_get_clientdata(client);

	wt6670f_info("%s: remove\n", __func__);
	mutex_destroy(&chip->irq_complete);
	mutex_destroy(&chip->isp_sequence_lock);
	cancel_delayed_work_sync(&chip->charger_type_det_work);
	cancel_delayed_work_sync(&chip->chip_update_work);

	return 0;
}

static int wt6670f_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct wt6670f_charger *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->irq_complete);
	chip->resume_completed = false;
	mutex_unlock(&chip->irq_complete);

	return 0;
}

static int wt6670f_suspend_noirq(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct wt6670f_charger *chip = i2c_get_clientdata(client);

	if (chip->irq_waiting) {
		pr_err_ratelimited("Aborting suspend, an interrupt was detected while suspending\n");
		return -EBUSY;
	}
	return 0;
}

static int wt6670f_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct wt6670f_charger *chip = i2c_get_clientdata(client);

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

static const struct dev_pm_ops wt6670f_pm_ops = {
	.suspend	= wt6670f_suspend,
	.suspend_noirq	= wt6670f_suspend_noirq,
	.resume		= wt6670f_resume,
};

static void wt6670f_charger_shutdown(struct i2c_client *client)
{
	struct wt6670f_charger *chip = i2c_get_clientdata(client);

	wt6670f_info("%s: shutdown\n", __func__);
	wt6670f_qc_mode_select(chip, WT6670F_MODE_QC20_V5);
	cancel_delayed_work_sync(&chip->charger_type_det_work);
	cancel_delayed_work_sync(&chip->chip_update_work);
	mutex_destroy(&chip->irq_complete);
	mutex_destroy(&chip->isp_sequence_lock);
}

static const struct of_device_id wt6670f_match_table[] = {
	{ .compatible = "mediatek,wt6670f_charger",},
	{ },
};

static const struct i2c_device_id wt6670f_charger_id[] = {
	{"wt6670f_charger", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, wt6670f_charger_id);

static struct i2c_driver wt6670f_charger_driver = {
	.driver		= {
		.name		= "wt6670f_charger",
		.owner		= THIS_MODULE,
		.of_match_table	= wt6670f_match_table,
		.pm		= &wt6670f_pm_ops,
	},
	.probe		= wt6670f_charger_probe,
	.remove		= wt6670f_charger_remove,
	.id_table	= wt6670f_charger_id,
	.shutdown	= wt6670f_charger_shutdown,
};

module_i2c_driver(wt6670f_charger_driver);

MODULE_DESCRIPTION("wt6670f Charger");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("i2c:wt6670f-charger");
