/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2015. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 */

#define LOG_TAG	"LCM"
#ifndef BUILD_LK
#  include <linux/string.h>
#  include <linux/kernel.h>
#endif

#include "lcm_drv.h"
#include <linux/regulator/consumer.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#include <asm-generic/gpio.h>

#ifdef BUILD_LK
#  include <platform/upmu_common.h>
#  include <platform/mt_gpio.h>
#  include <platform/mt_i2c.h>
#  include <platform/mt_pmic.h>
#  include <string.h>
#elif defined(BUILD_UBOOT)
#  include <asm/arch/mt_gpio.h>
#endif

#ifdef BUILD_LK
#  define LCM_LOGI(string, args...)  dprintf(0, "[LK/"LOG_TAG"]"string, ##args)
#  define LCM_LOGD(string, args...)  dprintf(1, "[LK/"LOG_TAG"]"string, ##args)
#else
#  define LCM_LOGI(fmt, args...)  pr_info("[KERNEL/"LOG_TAG"]"fmt, ##args)
#  define LCM_LOGD(fmt, args...)  pr_info("[KERNEL/"LOG_TAG"]"fmt, ##args)
#endif

extern int sgm_write_i2c(u8 addr, const u8 val);
extern int ext_ocp2138_i2c_write_byte(unsigned char addr, unsigned char value);
static struct LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)	(lcm_util.set_reset_pin((v)))
#define MDELAY(n)		(lcm_util.mdelay(n))
#define UDELAY(n)		(lcm_util.udelay(n))

#define dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update) \
		lcm_util.dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) \
		lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update) \
		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd) lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums) \
		lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)	lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size) \
		lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#ifndef BUILD_LK
#  include <linux/kernel.h>
#  include <linux/module.h>
#  include <linux/fs.h>
#  include <linux/slab.h>
#  include <linux/init.h>
#  include <linux/list.h>
#  include <linux/i2c.h>
#  include <linux/irq.h>
#  include <linux/uaccess.h>
#  include <linux/interrupt.h>
#  include <linux/io.h>
#  include <linux/platform_device.h>
#endif

#define FRAME_WIDTH			(1200)
#define FRAME_HEIGHT			(2000)

/* physical size in um */
#define LCM_PHYSICAL_WIDTH		(135360)
#define LCM_PHYSICAL_HEIGHT		(225600)
#define LCM_DENSITY			(240)

#define REGFLAG_DELAY			0xFFFC
#define REGFLAG_UDELAY			0xFFFB
#define REGFLAG_END_OF_TABLE		0xFFFD
#define REGFLAG_RESET_LOW		0xFFFE
#define REGFLAG_RESET_HIGH		0xFFFF


// ---------------------------------------------------------------------------
//  LCM power is provided by I2C
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
//  Local Variable
// ---------------------------------------------------------------------------

extern unsigned long oplus_max_normal_brightness;
static int map_exp[4096] = {0};

static void init_global_exp_backlight(void)
{
	int lut_index[41] = {0, 4, 99, 144, 187, 227, 264, 300, 334, 366, 397, 427, 456, 484, 511, 537, 563, 587, 611, 635, 658, 680,
						702, 723, 744, 764, 784, 804, 823, 842, 861, 879, 897, 915, 933, 950, 967, 984, 1000, 1016, 1023};
	int lut_value1[41] = {0, 4, 6, 14, 24, 37, 52, 69, 87, 107, 128, 150, 173, 197, 222, 248, 275, 302, 330, 358, 387, 416, 446, 
						477, 507, 539, 570, 602, 634, 667, 700, 733, 767, 801, 835, 869, 903, 938, 973, 1008, 1023};
	int index_start = 0, index_end = 0;
	int value1_start = 0, value1_end = 0;
	int i,j;
	int index_len = sizeof(lut_index) / sizeof(int);
	int value_len = sizeof(lut_value1) / sizeof(int);
	int res =0, sub = 0, mod = 0;
	if (index_len == value_len) {
		for (i = 0; i < index_len - 1; i++) {
			index_start = lut_index[i] * oplus_max_normal_brightness / 1023;
			index_end = lut_index[i+1] * oplus_max_normal_brightness / 1023;
			value1_start = lut_value1[i] * oplus_max_normal_brightness / 1023;
			value1_end = lut_value1[i+1] * oplus_max_normal_brightness / 1023;
			for (j = index_start; j <= index_end; j++) {
				//map_exp[j] = value1_start + (value1_end - value1_start) * (j - index_start) / (index_end - index_start);
				res = 2 * (value1_end - value1_start) * (j - index_start) / (index_end - index_start);
				sub = res / 2;
				mod = res % 2;
				map_exp[j] = value1_start + sub + mod;
			}
		}
	}
}

static unsigned int GPIO_LCD_RST;
static unsigned int GPIO_LCD_BL_EN;
static unsigned int GPIO_LCD_BIAS_ENP;
static unsigned int GPIO_LCD_BIAS_ENN;
static unsigned int GPIO_LCD_1V8;
static void lcm_request_gpio_control(struct device *dev)
{
	int ret;
	pr_notice("[Kernel/LCM] %s enter\n", __func__);
	GPIO_LCD_RST = of_get_named_gpio(dev->of_node, "gpio_lcd_rst", 0);
	ret = gpio_request(GPIO_LCD_RST, "GPIO_LCD_RST");
	if(ret < 0)
	{
		LCM_LOGI("gpio_request hx GPIO_LCD_RST fail!\n");
	}
	GPIO_LCD_BL_EN = of_get_named_gpio(dev->of_node, "gpio_lcd_bl_en", 0);
	ret = gpio_request(GPIO_LCD_BL_EN, "GPIO_LCD_BL_EN");
	if(ret < 0)
	{
		LCM_LOGI("gpio_request hx GPIO_LCD_BL_EN fail!\n");
	}
	GPIO_LCD_BIAS_ENP = of_get_named_gpio(dev->of_node,"gpio_lcd_bias_enp", 0);
	ret = gpio_request(GPIO_LCD_BIAS_ENP, "GPIO_LCD_BIAS_ENP");
	if(ret < 0)
	{
		LCM_LOGI("gpio_request hx GPIO_LCD_BIAS_ENP fail!\n");
	}
	GPIO_LCD_BIAS_ENN = of_get_named_gpio(dev->of_node,"gpio_lcd_bias_enn", 0);
	ret = gpio_request(GPIO_LCD_BIAS_ENN, "GPIO_LCD_BIAS_ENN");
	if(ret < 0)
	{
		LCM_LOGI("gpio_request hx GPIO_LCD_BIAS_ENN fail!\n");
	}
	GPIO_LCD_1V8 = of_get_named_gpio(dev->of_node,"gpio_lcd_1v8", 0);
	ret = gpio_request(GPIO_LCD_1V8, "GPIO_LCD_1V8");
	if(ret < 0)
	{
		LCM_LOGI("gpio_request hx GPIO_LCD_1V8 fail!\n");
	}
}
static int lcm_driver_probe(struct device *dev, void const *data)
{
	lcm_request_gpio_control(dev);
	oplus_max_normal_brightness = 3604;
	init_global_exp_backlight();
	return 0;
}
static const struct of_device_id lcm_platform_of_match[] = {
	{
		.compatible = "hx,hx83102",
		.data = 0,
	}, {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(of, platform_of_match);
static int lcm_platform_probe(struct platform_device *pdev)
{
	const struct of_device_id *id;
	id = of_match_node(lcm_platform_of_match, pdev->dev.of_node);
	if (!id)
		return -ENODEV;

	return lcm_driver_probe(&pdev->dev, id->data);
}
static struct platform_driver lcm_driver = {
	.probe = lcm_platform_probe,
	.driver = {
		.name = "hx83_hdp_dsi_vdo_txd",
		.owner = THIS_MODULE,
		.of_match_table = lcm_platform_of_match,
	},
};
extern char *saved_command_line;
static int __init lcm_drv_init(void)
{
	pr_notice("[Kernel/LCM] %s hx enter__yaf\n", __func__);
	if (!strstr(saved_command_line, "hx83_hdp_dsi_vdo_txd")) {
		pr_notice("it is not himax83102!\n");
		return -1;
	}
	if (platform_driver_register(&lcm_driver)) {
		pr_notice("LCM: failed to register disp driver\n");
		return -ENODEV;
	}
	return 0;
}
static void __exit lcm_drv_exit(void)
{
	platform_driver_unregister(&lcm_driver);
	pr_notice("LCM: Unregister lcm driver done\n");
}
late_initcall(lcm_drv_init);
module_exit(lcm_drv_exit);
MODULE_AUTHOR("mediatek");
MODULE_DESCRIPTION("Display subsystem Driver");
MODULE_LICENSE("GPL");
// ---------------------------------------------------------------------------
//  Local function
// ---------------------------------------------------------------------------
#define MDELAY(n)                       (lcm_util.mdelay(n))
#define UDELAY(n)                       (lcm_util.udelay(n))
#ifdef BUILD_LK
	#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))
#endif

#ifndef BUILD_LK
#define dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update) \
		lcm_util.dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update)
#endif
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) \
		lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update) \
		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd) \
		lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums) \
		lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd) \
		lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size) \
		lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[120];
};

static struct LCM_setting_table lcm_initialization_setting[] = {
{0xB9, 3, {0x83,0x10,0x2E}},
	{0xD1, 4, {0x67,0x0C,0xFF,0x05}},
	{0xB1, 17, {0x10,0xFA,0xAF,0xAF,0x2B,0x2B,0xA2,0x44,0x59,0x38,0x38,0x38,0x38,0x22,0x21,0x15,0x00}},
	{0xD2, 2, {0x2B,0x2B}},
	{0xB2, 16, {0x00,0xB0,0x47,0xD0,0x00,0x2C,0x2C,0x2C,0x00,0x00,0x00,0x00,0x15,0x20,0xD7,0x00}},
	{0xB4, 16, {0x88,0x64,0x01,0x01,0x88,0x64,0x68,0x50,0x01,0x8E,0x01,0x58,0x00,0xFF,0x00,0xFF}},
	{0xBA, 8, {0x70,0x03,0xA8,0x83,0xF2,0x00,0xC0,0x0D}},
	{0xBF, 3, {0xFC,0x85,0x80}},
	{0xC0, 14, {0x23,0x23,0x22,0x11,0xA2,0x10,0x00,0x80,0x00,0x00,0x08,0x00,0x63,0x63}},
	{0xC7, 1, {0x30}},
	{0xC8, 8, {0x00,0x04,0x04,0x00,0x00,0x82,0x13,0xFF}},
	{0xD0, 3, {0x07,0x04,0x05}},
	{0xD3, 43, {0x00,0x00,0x00,0x00,0x3C,0x04,0x00,0x0C,0x0C,0x4B,0x40,0x44,0x4F,0x2B,0x2B,0x04,0x04,0x32,0x10,0x29,0x00,0x29,0x54,0x17,0xFB,0x07,0xFB,0x32,0x10,0x10,0x00,0x10,0x00,0x00,0x21,0x38,0x01,0x55,0x21,0x38,0x01,0x55,0x0F}},
	{0xD5, 44, {0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x27,0x26,0x25,0x24,0x1A,0x1A,0x1B,0x1B,0x0B,0x0A,0x09,0x08,0x07,0x06,0x05,0x04,0x03,0x02,0x01,0x00,0x21,0x20,0x18,0x18,0x18,0x18}},
	{0xD6, 44, {0x98,0x98,0x98,0x98,0x98,0x98,0x98,0x98,0x98,0x98,0x98,0x98,0x98,0x98,0x98,0x98,0x18,0x18,0x26,0x27,0x20,0x21,0x1A,0x1A,0x1B,0x1B,0x08,0x09,0x0A,0x0B,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x24,0x25,0x18,0x18,0x18,0x18}},
	{0xE0, 46, {0x00,0x05,0x0E,0x15,0x1C,0x31,0x4A,0x52,0x5B,0x58,0x74,0x7B,0x83,0x94,0x94,0x9D,0xA6,0xBA,0xB8,0x5B,0x62,0x64,0x65,0x00,0x05,0x0E,0x15,0x1C,0x31,0x4A,0x52,0x5B,0x58,0x74,0x7B,0x83,0x94,0x94,0x9D,0xA6,0xBA,0xB8,0x5B,0x62,0x6A,0x73}},
	{0xE7, 23, {0x12,0x13,0x02,0x02,0x55,0x00,0x0E,0x0E,0x00,0x26,0x27,0x74,0x0D,0x73,0x01,0x27,0x00,0x00,0x00,0x00,0x17,0x00,0x68}},
	{0xBD, 1, {0x01}},
	{0xB1, 4, {0x01,0x9B,0x01,0x31}},
	{0xCB, 10, {0x80,0x36,0x12,0x16,0xC0,0x28,0x54,0x84,0x02,0x34}},
	{0xD3, 11, {0x01,0x00,0xF8,0x00,0x00,0x11,0x10,0x00,0x0A,0x00,0x01}},
	{0xE7, 7, {0x02,0x30,0x01,0x94,0x0D,0xC4,0x0E}},
	{0xBD, 1, {0x02}},
	{0xB4, 6, {0x4E,0x00,0x33,0x11,0x33,0x88}},
	{0xBF, 3, {0xF2,0x00,0x02}},
	{0xE7, 28, {0xFD,0x01,0xFD,0x01,0x00,0x00,0x24,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x81,0x00,0x02,0x40}},
	{0xD8, 12, {0xFF,0xFF,0xFC,0x3F,0xFF,0xF0,0xFF,0xFF,0xFC,0x3F,0xFF,0xF0}},
	{0xBD, 1, {0x03}},
	{0xD8, 24, {0xAA,0xAA,0xAA,0xAA,0xAA,0xA0,0xAA,0xAA,0xAA,0xAA,0xAA,0xA0,0xFF,0xFF,0xFC,0x3F,0xFF,0xF0,0xFF,0xFF,0xFC,0x3F,0xFF,0xF0}},
	{0xBD, 1, {0x00}},
	{0xE9, 1, {0xCD}},
	{0xBB, 1, {0x01}}, 
	{0xE9, 1, {0x00}},
	{REGFLAG_DELAY, 1, {}},
	{0xB8, 2, {0x40,0x00}},
	{0xBD, 1, {0x01}},
	{0xB8, 1, {0x15}},
	{0xBD, 1, {0x00}},
	{0xCC, 1, {0x02}},
	{0xE1, 2, {0x00,0x01}},
	{0x53, 1, {0x2C}},
	{0xC9, 3, {0x00,0x0E,0x10}},
	{0x35, 1, {0x00}},
	{0xB7, 2, {0x00,0x62}},
	{0xC7, 2, {0x30,0xB0}},
	{0x55, 0x01, {0x01} },
	{REGFLAG_DELAY, 5, {}},
	{0xBD, 1, {0x00}},
	{0xE4, 15, {0x2D,0x41,0x2C,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x03}},
	{0xBD, 1, {0x01}},
	{0xE4, 41, {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x54,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x00,0x77}},
	{0xBD, 1, {0x00}},
	{0x11, 0, {0x00}},
	{REGFLAG_DELAY, 150, {}},
	{0x29, 0, {0x00}},
	{REGFLAG_DELAY, 20, {}},
};

static struct LCM_setting_table lcm_sleep_mode_in_setting[] = {
	// Display off sequence
	{0x28, 0, {0x00}},
	{REGFLAG_DELAY, 50, {}},
	// Sleep Mode On
	{0x10, 0, {0x00}},
	{REGFLAG_DELAY, 120, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}},
};

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	// Display off sequence
	{0x28, 0, {0x00}},
	{REGFLAG_DELAY, 40, {}},
	// Sleep Mode On
	{0x10, 0, {0x00}},
	{REGFLAG_DELAY, 70, {}},
	{0xB9, 3, {0x83,0x10,0x2E}},
	{0xB1, 1, {0x11}},
	{REGFLAG_DELAY, 10, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}},
};

static struct LCM_setting_table bl_level[] = {
	{0x51, 0x02, {0x0F, 0xFF} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

static struct LCM_setting_table cabc_mode0[] = {
	{0xB9, 3, {0x83,0x10,0x2E}},
	{0xBD, 1, {0x00}},
	{0xE4, 15, {0x2D,0x41,0x2C,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x03}},
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

static struct LCM_setting_table cabc_mode1[] = {
	{0xB9, 3, {0x83,0x10,0x2E}},
	{0xBD, 1, {0x00}},
	{0xE4, 0x0F, {0x2D,0x41,0x2C,0x0D,0x8D,0x8D,0xA7,0xA7,0xA7,0xC3,0xC3,0xC3,0xFF,0xFF,0x03} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

static struct LCM_setting_table cabc_mode2[] = {
	{0xB9, 3, {0x83,0x10,0x2E}},
	{0xBD, 1, {0x00}},
	{0xE4, 0x0F, {0x2D,0x41,0x2C,0x40,0x22,0x49,0x76,0x82,0x0F,0x59,0x1D,0x93,0xAA,0xFF,0x03} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

static struct LCM_setting_table esd_check[] = {
	{0xCC, 1, {0x02} },
	{0xB0, 4, {0x00, 0x35, 0x3C, 0x0F} },
	{0x20, 0, {0x00} },
	{0x38, 0, {0x00} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

static void push_table(void *cmdq, struct LCM_setting_table *table,
		       unsigned int count, unsigned char force_update)
{
	unsigned int i;
	unsigned int cmd;

	for (i = 0; i < count; i++) {
		cmd = table[i].cmd;
		switch (cmd) {
		case REGFLAG_DELAY:
				MDELAY(table[i].count);
			break;
		case REGFLAG_UDELAY:
			UDELAY(table[i].count);
			break;
		case REGFLAG_END_OF_TABLE:
			break;
		default:
			dsi_set_cmdq_V22(cmdq, cmd, table[i].count,
					 table[i].para_list, force_update);
			break;
		}
	}
}

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------
static void lcm_set_util_funcs(const struct LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(struct LCM_UTIL_FUNCS));
}

static void lcm_get_params(struct LCM_PARAMS *params)
{
	memset(params, 0, sizeof(struct LCM_PARAMS));

	params->type = LCM_TYPE_DSI;

	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;
	params->physical_width = LCM_PHYSICAL_WIDTH / 1000;
	params->physical_height = LCM_PHYSICAL_HEIGHT / 1000;
	params->physical_width_um = LCM_PHYSICAL_WIDTH;
	params->physical_height_um = LCM_PHYSICAL_HEIGHT;
	params->density = LCM_DENSITY;

	params->dsi.mode = BURST_VDO_MODE;
	params->dsi.switch_mode = CMD_MODE;
	lcm_dsi_mode = BURST_VDO_MODE;
	LCM_LOGI("%s: lcm_dsi_mode %d\n", __func__, lcm_dsi_mode);
	params->dsi.switch_mode_enable = 0;

	/* DSI */
	/* Command mode setting */
	params->dsi.LANE_NUM = LCM_FOUR_LANE;
	/* The following defined the fomat for data coming from LCD engine. */
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

	/* Highly depends on LCD driver capability. */
	params->dsi.packet_size = 256;
	/* video mode timing */

	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active = 8;
	params->dsi.vertical_backporch = 38;
	params->dsi.vertical_frontporch = 44;
	//params->dsi.vertical_frontporch_for_low_power = 750;	//OTM no data
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 10;
	params->dsi.horizontal_backporch = 15;
	params->dsi.horizontal_frontporch = 15;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;
	/* params->dsi.ssc_disable = 1; */

	params->dsi.PLL_CLOCK = 507;
	params->dsi.PLL_CK_CMD = 440;
	params->dsi.LPX = 8;

	params->dsi.CLK_HS_POST = 36;
	params->dsi.clk_lp_per_line_enable = 0;
	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 1;
	params->dsi.lcm_esd_check_table[0].cmd = 0x09;
	params->dsi.lcm_esd_check_table[0].count = 4;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x80;
	params->dsi.lcm_esd_check_table[0].para_list[1] = 0x73;
	params->dsi.lcm_esd_check_table[0].para_list[2] = 0x06;
	params->dsi.lcm_esd_check_table[0].para_list[3] = 0x00;
	params->dsi.lcm_esd_check_table[1].cmd 			= 0x0A;
	params->dsi.lcm_esd_check_table[1].count 		= 1;
	params->dsi.lcm_esd_check_table[1].para_list[0] = 0x9D;
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	params->round_corner_en = 1;
	params->corner_pattern_width = 1200;
	params->corner_pattern_height = 31;
	params->corner_pattern_height_bot = 31;
#endif
	
}

static void lcm_set_gpio_output(unsigned int GPIO, unsigned int output)
{
	gpio_direction_output(GPIO, output);
	gpio_set_value(GPIO, output);
}

static void lcm_bias_regulator_init()
{
    // GPIO for DSV setting
	lcm_set_gpio_output(GPIO_LCD_1V8, 1);
	MDELAY(10);
	lcm_set_gpio_output(GPIO_LCD_BIAS_ENP, 1);
	MDELAY(10);
	lcm_set_gpio_output(GPIO_LCD_BIAS_ENN, 1);
	MDELAY(10);
}

static int lcm_bias_enable(void)
{
	int ret = 0;
	int retval = 0;

	lcm_bias_regulator_init();
	/* set voltage with min & max*/
	ret = ext_ocp2138_i2c_write_byte(0x00, 0x14);
	if (ret < 0)
		pr_info("set voltage disp_bias_pos fail, ret = %d\n", ret);
	retval |= ret;

	ret = ext_ocp2138_i2c_write_byte(0x01, 0x14);
	if (ret < 0)
		pr_info("set voltage disp_bias_neg fail, ret = %d\n", ret);
	retval |= ret;


	return retval;
}


static int lcm_bias_disable(void)
{
	MDELAY(5);
	lcm_set_gpio_output(GPIO_LCD_BIAS_ENN, 0);
	MDELAY(10);
	lcm_set_gpio_output(GPIO_LCD_BIAS_ENP, 0);
	MDELAY(10);
	lcm_set_gpio_output(GPIO_LCD_1V8, 0);
	MDELAY(10);
	return 0;
}

/* turn on gate ic & control voltage to 5.5V */
/* equle display_bais_enable ,mt6768 need +/-5.5V */
static void lcm_init_power(void)
{
	lcm_bias_enable();
}

extern int tp_gesture_enable_flag(void);
extern int tp_control_reset_gpio(bool enable);
static void lcm_suspend_power(void)
{
	LCM_LOGI("%s,HX83 \n", __func__);
	if (0 == tp_gesture_enable_flag()) {
		SET_RESET_PIN(0);
		lcm_bias_disable();
		tp_control_reset_gpio(false);
		MDELAY(2);
	}
}
/* turn on gate ic & control voltage to 5.5V */
static void lcm_resume_power(void)
{
	SET_RESET_PIN(0);
	lcm_init_power();
}

extern void lcd_queue_load_tp_fw(void);
static void lcm_init(void)
{
	SET_RESET_PIN(1);
	MDELAY(2);
	SET_RESET_PIN(0);
	MDELAY(5);

	SET_RESET_PIN(1);
	MDELAY(100);

	tp_control_reset_gpio(true);

	lcd_queue_load_tp_fw();
	MDELAY(5);

	push_table(NULL, lcm_initialization_setting, ARRAY_SIZE(lcm_initialization_setting), 1);
	LCM_LOGI("HX83----tps6132----lcm mode = vdo mode :%d----\n",
		 lcm_dsi_mode);
	lcm_set_gpio_output(GPIO_LCD_BL_EN, 1);
	sgm_write_i2c(0x10, 0x1f);
	sgm_write_i2c(0x11, 0x25);
}

static void lcm_suspend(void)
{
	lcm_set_gpio_output(GPIO_LCD_BL_EN, 0);
	if (0 == tp_gesture_enable_flag()) {
		push_table(NULL, lcm_deep_sleep_mode_in_setting,
			ARRAY_SIZE(lcm_deep_sleep_mode_in_setting), 1);
	}else{
		push_table(NULL, lcm_sleep_mode_in_setting,
			ARRAY_SIZE(lcm_sleep_mode_in_setting), 1);
	}
}

static void lcm_resume(void)
{
	lcm_init();
}
static unsigned int lcm_compare_id(void)
{
	return 0;
}

static unsigned int global_lcm2_level = 0;
static void lcm_setbacklight_cmdq(void *handle, unsigned int level)
{
	//LCM_LOGI("%s,HX83 backlight: level = %d\n", __func__, level);
	//level = level*4095/255;
	LCM_LOGI("%s,HX83 backlight: level = %d\n", __func__, level);
	if (level > 4095)
		level = 4095;
	global_lcm2_level = level;
	if ((level > 0) && (level < oplus_max_normal_brightness)) {
		level = map_exp[level];
	}
	pr_err ("%s after change backlight = %d\n", __func__, level);
	bl_level[0].para_list[0] = 0x0F&(level >> 8);
	bl_level[0].para_list[1] = 0xFF&level;

	push_table(handle, bl_level, ARRAY_SIZE(bl_level), 1);
}

unsigned int global_lcm2_getbacklight(void)
{
	return global_lcm2_level;
}
EXPORT_SYMBOL(global_lcm2_getbacklight);

void global_lcm2_setbacklight(unsigned int level)
{
	LCM_LOGI("%s,HX83 backlight: level = %d\n", __func__, level);
	if (level > 4095)
		level = 4095;
	if ((level > 0) && (level < oplus_max_normal_brightness)) {
		level = map_exp[level];
	}
	pr_err ("%s after change backlight = %d\n", __func__, level);
	bl_level[0].para_list[0] = 0x0F&(level >> 8);
	bl_level[0].para_list[1] = 0xFF&level;

	push_table(NULL, bl_level, ARRAY_SIZE(bl_level), 1);
}
EXPORT_SYMBOL(global_lcm2_setbacklight);

static int cabc_status;
static void lcm_set_cabc_cmdq(void *handle, unsigned int level){
	pr_err("[lcm] 1234567 cabc set level %d\n", level);
	if (level==1){
		push_table(handle, cabc_mode1, ARRAY_SIZE(cabc_mode1), 1);
	}else if(level==2){
		push_table(handle, cabc_mode2, ARRAY_SIZE(cabc_mode2), 1);
	}else{
		push_table(handle, cabc_mode0, ARRAY_SIZE(cabc_mode0), 1);
	}
	cabc_status = level;
}

static void lcm_get_cabc_status(int *status){
	pr_info("[lcm] cabc get to %d\n", cabc_status);
	*status = cabc_status;
}

void esdcheck_cmdq(void)
{
	LCM_LOGI("%s,HX83 esd check\n", __func__);
	push_table(NULL, esd_check, ARRAY_SIZE(esd_check), 1);
}

struct LCM_DRIVER hx83_hdp_dsi_vdo_txd_lcm_drv =
{
	.name			= "hx83_hdp_dsi_vdo_txd",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.compare_id = lcm_compare_id,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.init_power = lcm_init_power,
	.resume_power = lcm_resume_power,
	.suspend_power = lcm_suspend_power,
	.set_cabc_mode_cmdq = lcm_set_cabc_cmdq,
	.get_cabc_status = lcm_get_cabc_status,
};
