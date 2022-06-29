#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#include <asm-generic/gpio.h>
/* Define -------------------------------------------------------------------*/
#define I2C_I2C_LCD_BIAS_CHANNEL 7  //for I2C channel 2
#define DCDC_I2C_BUSNUM  I2C_I2C_LCD_BIAS_CHANNEL//for I2C channel 6
#define DCDC_I2C_ID_NAME "ocp2138"
#define DCDC_I2C_ADDR 0x3E

//extern int ext_ocp2138_i2c_write_byte(unsigned char addr, unsigned char value);

struct OCP2138_SETTING_TABLE {
	unsigned char cmd;
	unsigned char data;
};

/*static struct OCP2138_SETTING_TABLE ocp2138_cmd_data[2] = {
	{ 0x00, 0x14 },
	{ 0x01, 0x14 }
};*/

/* Variable -----------------------------------------------------------------*/
static const struct of_device_id lcm_of_match[] = {
	{.compatible = "mediatek,i2c_lcd_bias"},
	{},
};

struct i2c_client *ocp2138_i2c_client;

/* Functions Prototype --------------------------------------------------------*/
static int ocp2138_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int ocp2138_remove(struct i2c_client *client);

/* Data Structure -------------------------------------------------------------*/
struct ocp2138_dev {
	struct i2c_client *client;
};

static const struct i2c_device_id ocp2138_id[] = {
	{ DCDC_I2C_ID_NAME, 0 },
	{ }
};

/* I2C Driver  ----------------------------------------------------------------*/
static struct i2c_driver ocp2138_iic_driver = {
	.id_table		= ocp2138_id,
	.probe			= ocp2138_probe,
	.remove			= ocp2138_remove,
	.driver			= {
		.owner			= THIS_MODULE,
		.name			= "ocp2138",
		.of_match_table	= lcm_of_match,
	},
};

/* Functions ------------------------------------------------------------------*/
static int ocp2138_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	ocp2138_i2c_client  = client;
	return 0;
}

static int ocp2138_remove(struct i2c_client *client)
{
	ocp2138_i2c_client = NULL;
	i2c_unregister_device(client);
	return 0;
}

int ext_ocp2138_i2c_write_byte(unsigned char addr, unsigned char value)
{
	int ret = 0;
	struct i2c_client *client = ocp2138_i2c_client;
	char write_data[2]={0};
	pr_info("ext_ocp2138_i2c_write_byte\n");
	if(client == NULL)
	{
		pr_info("ERROR!! ocp2138_i2c_client is null\n");
		return 0;
	}
	write_data[0]= addr;
	write_data[1] = value;
	ret=i2c_master_send(client, write_data, 2);
	if(ret<0)
		pr_info("ocp2138 write data fail !!\n");
	return ret ;
}

static int __init ocp2138_iic_init(void)
{
	pr_info("ocp2138_iic_init\n");
	i2c_add_driver(&ocp2138_iic_driver);
	return 0;
}

static void __exit ocp2138_iic_exit(void)
{
	i2c_del_driver(&ocp2138_iic_driver);
}

module_init(ocp2138_iic_init);
module_exit(ocp2138_iic_exit);
MODULE_DESCRIPTION("MTK OCP2138 I2C Driver");