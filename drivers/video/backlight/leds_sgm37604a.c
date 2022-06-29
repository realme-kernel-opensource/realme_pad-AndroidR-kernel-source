/*
* sgm37604a.c   sgm37604a backlight module
*
* Version: v1.0.3
*
* Copyright (c) 2019 AWINIC Technology CO., LTD
*
*  Author: Joseph <zhangzetao@awinic.com.cn>
*
* This program is free software; you can redistribute  it and/or modify it
* under  the terms of  the GNU General  Public License as published by the
* Free Software Foundation;  either version 2 of the  License, or (at your
* option) any later version.
*/

#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/types.h>
#include <linux/regulator/consumer.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/backlight.h>

#define sgm37604a_LED_DEV "sgm_bl"
#define sgm37604a_NAME "sgm_bl"

#define sgm37604a_VERSION "v1.0.3"
#define MAX_BRIGHTNESS 2047
struct sgm37604a_data {
	struct led_classdev led_dev;
	struct i2c_client *client;
	struct device dev;
	struct i2c_adapter *adapter;
	unsigned short addr;
	struct mutex lock;
	struct work_struct work;
	enum led_brightness brightness;
	bool enable;
	u8 pwm_cfg;
	u8 full_scale_current;
	bool brt_code_enable;
	u16 *brt_code_table;
	int hwen_gpio;
	unsigned int  pwm_mode;
	bool using_lsb;
	unsigned int pwm_period;
	unsigned int full_scale_led;
	unsigned int ramp_on_time;
	unsigned int ramp_off_time;
	unsigned int pwm_trans_dim;
	unsigned int i2c_trans_dim;
	unsigned int channel;
	unsigned int ovp_level;
	unsigned int frequency;
	unsigned int default_brightness;
	unsigned int max_brightness;
	unsigned int induct_current;
	unsigned int flash_current;
	unsigned int flash_timeout;
	unsigned int bl_map;
	struct backlight_device *bl_dev;
};

struct sgm37604a_data *g_sgm37604a_data;

static int platform_read_i2c_block(struct i2c_client *client, char *writebuf,
			   int writelen, char *readbuf, int readlen)
{
	int ret;

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
				 .addr = client->addr,
				 .flags = 0,
				 .len = writelen,
				 .buf = writebuf,
			 },
			{
				 .addr = client->addr,
				 .flags = I2C_M_RD,
				 .len = readlen,
				 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			dev_err(&client->dev, "%s: i2c read error.\n",
				__func__);
	} else {
		struct i2c_msg msgs[] = {
			{
				 .addr = client->addr,
				 .flags = I2C_M_RD,
				 .len = readlen,
				 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			dev_err(&client->dev, "%s:i2c read error.\n", __func__);
	}
	return ret;
}

static int sgm37604a_i2c_read(struct i2c_client *client, u8 addr, u8 *val)
{
	return platform_read_i2c_block(client, &addr, 1, val, 1);
}

static int platform_write_i2c_block(struct i2c_client *client,
		char *writebuf, int writelen)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			 .addr = client->addr,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
		 },
	};
	ret = i2c_transfer(client->adapter, msgs, 1);
	if (ret < 0)
		dev_err(&client->dev, "%s: i2c write error.\n", __func__);

	return ret;
}

static int sgm37604a_i2c_write(struct i2c_client *client, u8 addr, const u8 val)
{
	u8 buf[2] = {0};

	buf[0] = addr;
	buf[1] = val;

	return platform_write_i2c_block(client, buf, sizeof(buf));
}
int sgm_write_i2c(u8 addr, const u8 val)
{
	sgm37604a_i2c_write(g_sgm37604a_data->client,addr,val);

	return 0;
}
EXPORT_SYMBOL(sgm_write_i2c);

static void sgm37604a_hwen_pin_ctrl(struct sgm37604a_data *drvdata, int en)
{
	if (gpio_is_valid(drvdata->hwen_gpio)) {
		if (en) {
			pr_info("hwen pin is going to be high!---<%d>\n", en);
			gpio_set_value(drvdata->hwen_gpio, true);
			usleep_range(3500, 4000);
		} else {
			pr_info("hwen pin is going to be low!---<%d>\n", en);
			gpio_set_value(drvdata->hwen_gpio, false);
			usleep_range(1000, 2000);
		}
	}
}

static int sgm37604a_backlight_init(struct sgm37604a_data *drvdata)
{
	pr_info("%s enter.\n", __func__);

	pr_info("%s enter\n", __func__);
	int ret = 0;
	ret |= sgm37604a_i2c_write(drvdata->client,0x10, 0x1f);
	ret |= sgm37604a_i2c_write(drvdata->client,0x11, 0x25);
	ret |= sgm37604a_i2c_write(drvdata->client,0x19, 0xb3);
	ret |= sgm37604a_i2c_write(drvdata->client,0x1A, 0x02);


	return 0;
}

static int sgm37604a_backlight_enable(struct sgm37604a_data *drvdata)
{
	pr_info("%s enter.\n", __func__);
	drvdata->enable = true;
	return 0;
}

int  sgm37604a_set_brightness(struct sgm37604a_data *drvdata, int brt_val)
{
	pr_info("%s brt_val is %d\n", __func__, brt_val);

	if (drvdata->enable == false) {
		sgm37604a_backlight_init(drvdata);
		sgm37604a_backlight_enable(drvdata);
	}

	if (brt_val < 0)
		brt_val =0;

	if (brt_val > MAX_BRIGHTNESS)
		brt_val = MAX_BRIGHTNESS;

	/* set the brightness in brightness control register*/
	sgm37604a_i2c_write(drvdata->client, 0x19, (brt_val >> 4));
	sgm37604a_i2c_write(drvdata->client, 0x1A, (brt_val & 0xf));

	drvdata->brightness = brt_val;
	if (drvdata->brightness == 0)
		drvdata->enable = false;

	return 0;
}


static int sgm37604a_bl_get_brightness(struct backlight_device *bl_dev)
{
		return bl_dev->props.brightness;
}
static int sgm37604a_bl_update_status(struct backlight_device *bl_dev)
{
		struct sgm37604a_data *drvdata = bl_get_data(bl_dev);
		int brt;

		if (bl_dev->props.state & BL_CORE_SUSPENDED)
				bl_dev->props.brightness = 0;

		brt = bl_dev->props.brightness;
		/*
		 * Brightness register should always be written
		 * not only register based mode but also in PWM mode.
		 */
		return sgm37604a_set_brightness(drvdata, brt);
}

static const struct backlight_ops sgm37604a_bl_ops = {
		.update_status = sgm37604a_bl_update_status,
		.get_brightness = sgm37604a_bl_get_brightness,
};

/*int sgm37604a_backlight_device_set_brightness(struct backlight_device *bl_dev,
				    unsigned long brightness)
{
	//struct sgm37604a_data *drvdata = bl_get_data(bl_dev);
	int rc = -ENXIO;

	mutex_lock(&bl_dev->ops_lock);
	if (bl_dev->ops) {
			if (brightness > bl_dev->props.max_brightness)
			brightness = bl_dev->props.max_brightness;

			pr_debug("set brightness to %lu\n", brightness);
			bl_dev->props.brightness = brightness;
			rc = sgm37604a_bl_update_status(bl_dev);
		}
	mutex_unlock(&bl_dev->ops_lock);

	//backlight_generate_event(bl_dev, BACKLIGHT_UPDATE_SYSFS);

	return rc;
}
EXPORT_SYMBOL(sgm37604a_backlight_device_set_brightness);*/

static void __sgm37604a_work(struct sgm37604a_data *led,
				enum led_brightness value)
{
	mutex_lock(&led->lock);
	sgm37604a_set_brightness(led, value);
	mutex_unlock(&led->lock);
}

static void sgm37604a_work(struct work_struct *work)
{
	struct sgm37604a_data *drvdata = container_of(work,
					struct sgm37604a_data, work);

	__sgm37604a_work(drvdata, drvdata->led_dev.brightness);
}


static void sgm37604a_brightness_set(struct led_classdev *led_cdev,
			enum led_brightness brt_val)
{
	struct sgm37604a_data *drvdata;

	drvdata = container_of(led_cdev, struct sgm37604a_data, led_dev);
	schedule_work(&drvdata->work);
}

static void
sgm37604a_get_dt_data(struct device *dev, struct sgm37604a_data *drvdata)
{
	int rc;
	struct device_node *np = dev->of_node;
	u32 bl_channel, temp;

	drvdata->hwen_gpio = of_get_named_gpio(np, "sgm37604a,hwen-gpio", 0);
	pr_info("%s drvdata->hwen_gpio --<%d>\n", __func__, drvdata->hwen_gpio);

	rc = of_property_read_u32(np, "sgm37604a,pwm-mode", &drvdata->pwm_mode);
	if (rc != 0)
		pr_err("%s pwm-mode not found\n", __func__);
	else
		pr_info("%s pwm_mode=%d\n", __func__, drvdata->pwm_mode);

	drvdata->using_lsb = of_property_read_bool(np, "sgm37604a,using-lsb");
	pr_info("%s using_lsb --<%d>\n", __func__, drvdata->using_lsb);

	if (drvdata->using_lsb) {
		drvdata->default_brightness = 0x7ff;
		drvdata->max_brightness = 2047;
	} else {
		drvdata->default_brightness = 0xff;
		drvdata->max_brightness = 255;
	}

	rc = of_property_read_u32(np, "sgm37604a,bl-fscal-led", &temp);
	if (rc) {
		pr_err("Invalid backlight full-scale led current!\n");
	} else {
		drvdata->full_scale_led = temp;
		pr_info("%s full-scale led current --<%d mA>\n",
			__func__, drvdata->full_scale_led);
	}

	rc = of_property_read_u32(np, "sgm37604a,turn-on-ramp", &temp);
	if (rc) {
		pr_err("Invalid ramp timing ,turnon!\n");
	} else {
		drvdata->ramp_on_time = temp;
		pr_info("%s ramp on time --<%d ms>\n",
			__func__, drvdata->ramp_on_time);
	}

	rc = of_property_read_u32(np, "sgm37604a,turn-off-ramp", &temp);
	if (rc) {
		pr_err("Invalid ramp timing ,,turnoff!\n");
	} else {
		drvdata->ramp_off_time = temp;
		pr_info("%s ramp off time --<%d ms>\n",
			__func__, drvdata->ramp_off_time);
	}

	rc = of_property_read_u32(np, "sgm37604a,pwm-trans-dim", &temp);
	if (rc) {
		pr_err("Invalid pwm-tarns-dim value!\n");
	} else {
		drvdata->pwm_trans_dim = temp;
		pr_info("%s pwm trnasition dimming	--<%d ms>\n",
			__func__, drvdata->pwm_trans_dim);
	}

	rc = of_property_read_u32(np, "sgm37604a,i2c-trans-dim", &temp);
	if (rc) {
		pr_err("Invalid i2c-trans-dim value !\n");
	} else {
		drvdata->i2c_trans_dim = temp;
		pr_info("%s i2c transition dimming --<%d ms>\n",
			__func__, drvdata->i2c_trans_dim);
	}

	rc = of_property_read_u32(np, "sgm37604a,bl-channel", &bl_channel);
	if (rc) {
		pr_err("Invalid channel setup\n");
	} else {
		drvdata->channel = bl_channel;
		pr_info("%s bl-channel --<%x>\n", __func__, drvdata->channel);
	}

	rc = of_property_read_u32(np, "sgm37604a,bl-map", &drvdata->bl_map);
	if (rc != 0)
		pr_err("%s bl_map not found\n", __func__);
	else
		pr_info("%s bl_map=%d\n", __func__, drvdata->bl_map);
}

/******************************************************
 *
 * sys group attribute: reg
 *
 ******************************************************/
static struct attribute *sgm37604a_attributes[] = {
	NULL
};

static struct attribute_group sgm37604a_attribute_group = {
	.attrs = sgm37604a_attributes
};

static int sgm37604a_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct sgm37604a_data *drvdata;

	struct backlight_device *bl_dev;
	struct backlight_properties props;

	int err = 0;

	pr_info("%s enter! driver version %s\n", __func__, sgm37604a_VERSION);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s : I2C_FUNC_I2C not supported\n", __func__);
		err = -EIO;
		goto err_out;
	}

	if (!client->dev.of_node) {
		pr_err("%s : no device node\n", __func__);
		err = -ENOMEM;
		goto err_out;
	}

	drvdata = kzalloc(sizeof(struct sgm37604a_data), GFP_KERNEL);
	if (drvdata == NULL) {
		pr_err("%s : kzalloc failed\n", __func__);
		err = -ENOMEM;
		goto err_out;
	}

	drvdata->client = client;
	drvdata->adapter = client->adapter;
	drvdata->addr = client->addr;
	drvdata->brightness = LED_OFF;
	drvdata->enable = true;
	drvdata->led_dev.default_trigger = "bkl-trigger";
	drvdata->led_dev.name = sgm37604a_LED_DEV;
	drvdata->led_dev.brightness_set = sgm37604a_brightness_set;
	drvdata->led_dev.max_brightness = MAX_BRIGHTNESS;
	mutex_init(&drvdata->lock);
	INIT_WORK(&drvdata->work, sgm37604a_work);
	sgm37604a_get_dt_data(&client->dev, drvdata);
	i2c_set_clientdata(client, drvdata);

	/*err = sgm37604a_read_chipid(drvdata);
	if (err < 0) {
		pr_err("%s : ID idenfy failed\n", __func__);
		goto err_init;
	}*/

	err = led_classdev_register(&client->dev, &drvdata->led_dev);
	if (err < 0) {
		pr_err("%s : Register led class failed\n", __func__);
		err = -ENODEV;
		goto err_init;
	} else {
	pr_debug("%s: Register led class successful\n", __func__);
	}

	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_RAW;
	props.brightness = MAX_BRIGHTNESS;
	props.max_brightness = MAX_BRIGHTNESS;
	bl_dev = backlight_device_register(sgm37604a_NAME, &client->dev,
					drvdata, &sgm37604a_bl_ops, &props);

	g_sgm37604a_data = drvdata;
	sgm37604a_backlight_init(drvdata);
	sgm37604a_backlight_enable(drvdata);

	sgm37604a_set_brightness(drvdata, MAX_BRIGHTNESS);
	err = sysfs_create_group(&client->dev.kobj, &sgm37604a_attribute_group);
	if (err < 0) {
		dev_info(&client->dev, "%s error creating sysfs attr files\n",
			__func__);
		goto err_sysfs;
	}
	pr_info("%s exit\n", __func__);
	return 0;

err_sysfs:
err_init:
	kfree(drvdata);
err_out:
	return err;
}

static int sgm37604a_remove(struct i2c_client *client)
{
	struct sgm37604a_data *drvdata = i2c_get_clientdata(client);

	led_classdev_unregister(&drvdata->led_dev);

	kfree(drvdata);
	return 0;
}

static const struct i2c_device_id sgm37604a_id[] = {
	{sgm37604a_NAME, 0},
	{}
};
static struct of_device_id match_table[] = {
		{.compatible = "mediatek,sgm_bl",}
};

MODULE_DEVICE_TABLE(i2c, sgm37604a_id);

static struct i2c_driver sgm37604a_i2c_driver = {
	.probe = sgm37604a_probe,
	.remove = sgm37604a_remove,
	.id_table = sgm37604a_id,
	.driver = {
		.name = sgm37604a_NAME,
		.owner = THIS_MODULE,
		.of_match_table = match_table,
	},
};

module_i2c_driver(sgm37604a_i2c_driver);
MODULE_DESCRIPTION("Back Light driver for sgm37604a");
MODULE_LICENSE("GPL v2");