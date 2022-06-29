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
#define LCM_DENSITY			(480)

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
		LCM_LOGI("gpio_request nt GPIO_LCD_RST fail!\n");
	}
	GPIO_LCD_BL_EN = of_get_named_gpio(dev->of_node, "gpio_lcd_bl_en", 0);
	ret = gpio_request(GPIO_LCD_BL_EN, "GPIO_LCD_BL_EN");
	if(ret < 0)
	{
		LCM_LOGI("gpio_request nt GPIO_LCD_BL_EN fail!\n");
	}
	GPIO_LCD_BIAS_ENP = of_get_named_gpio(dev->of_node,"gpio_lcd_bias_enp", 0);
	ret = gpio_request(GPIO_LCD_BIAS_ENP, "GPIO_LCD_BIAS_ENP");
	if(ret < 0)
	{
		LCM_LOGI("gpio_request nt GPIO_LCD_BIAS_ENP fail!\n");
	}
	GPIO_LCD_BIAS_ENN = of_get_named_gpio(dev->of_node,"gpio_lcd_bias_enn", 0);
	ret = gpio_request(GPIO_LCD_BIAS_ENN, "GPIO_LCD_BIAS_ENN");
	if(ret < 0)
	{
		LCM_LOGI("gpio_request nt GPIO_LCD_BIAS_ENN fail!\n");
	}
	GPIO_LCD_1V8 = of_get_named_gpio(dev->of_node,"gpio_lcd_1v8", 0);
	ret = gpio_request(GPIO_LCD_1V8, "GPIO_LCD_1V8");
	if(ret < 0)
	{
		LCM_LOGI("gpio_request nt GPIO_LCD_1V8 fail!\n");
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
		.compatible = "nt,nt36523b",
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
		.name = "nt36523b_hdp_dsi_vdo_dijing",
		.owner = THIS_MODULE,
		.of_match_table = lcm_platform_of_match,
	},
};
extern char *saved_command_line;
static int __init lcm_drv_init(void)
{
	pr_notice("[Kernel/LCM] %s nt enter__yaf\n", __func__);
	if (!strstr(saved_command_line, "nt36523b_hdp_dsi_vdo_dijing")) {
		pr_notice("it is not nt36523b!\n");
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
	{0xFF,1, {0x26}},
	{0xFB,1, {0x01}},
	{0xA7,1, {0x10}},

	{0xFF,1, {0x20}},
	{0xFB,1, {0x01}},
	{0x05,1, {0xB9}},
	//VGH=14V/VGL=-14V
	{0x07,1, {0x5A}},
	{0x08,1, {0x5A}},
	{0x0D,1, {0x60}},
	//VGHO=14V/VGLO=-14V
	{0x0E,1, {0x7D}},
	{0x0F,1, {0x7D}},
	//ASW
	{0x1F,1, {0x1A}},
	//VGMP=5.5
	{0x94,1, {0x00}},
	{0x95,1, {0xEB}},
	{0x96,1, {0xEB}},
	//VGSP=0.3
	{0x9D,1, {0x14}},
	{0x9E,1, {0x14}},
	//ISOP
	//{0x6D,1, {0xFF}},
	{0x58,1, {0x60}},
	{0x69,1, {0x98}},
	{0x75,1, {0xA2}},
	{0x77,1, {0xB3}},
	//audible noise
	{0x01,1, {0x52}},
	{0x1C,1, {0x74}},
	{0xEE,1, {0x70}},
	{0xEF,1, {0x70}},
	//GAMMA
	{0xFF,1, {0x20}},
	{0xFB,1, {0x01}},
	//R(+)
	{0xB0,16, {0x00,0x08,0x00,0x20,0x00,0x46,0x00,0x66,0x00,0x80,0x00,0x98,0x00,0xAE,0x00,0xC1}},
	{0xB1,16, {0x00,0xD2,0x01,0x0E,0x01,0x39,0x01,0x7E,0x01,0xB2,0x02,0x02,0x02,0x41,0x02,0x43}},
	{0xB2,16, {0x02,0x7F,0x02,0xC2,0x02,0xEF,0x03,0x23,0x03,0x48,0x03,0x70,0x03,0x7F,0x03,0x8C}},
	{0xB3,12, {0x03,0x9B,0x03,0xAA,0x03,0xB9,0x03,0xD6,0x03,0xCF,0x03,0xCB}},
	//G(+)
	{0xB4,16, {0x00,0x08,0x00,0x20,0x00,0x46,0x00,0x66,0x00,0x80,0x00,0x98,0x00,0xAE,0x00,0xC1}},
	{0xB5,16, {0x00,0xD2,0x01,0x0E,0x01,0x39,0x01,0x7E,0x01,0xB2,0x02,0x02,0x02,0x41,0x02,0x43}},
	{0xB6,16, {0x02,0x7F,0x02,0xC2,0x02,0xEF,0x03,0x23,0x03,0x48,0x03,0x70,0x03,0x7F,0x03,0x8C}},
	{0xB7,12, {0x03,0x9B,0x03,0xAA,0x03,0xB9,0x03,0xD6,0x03,0xCF,0x03,0xCB}},
	//B(+)
	{0xB8,16, {0x00,0x08,0x00,0x20,0x00,0x46,0x00,0x66,0x00,0x80,0x00,0x98,0x00,0xAE,0x00,0xC1}},
	{0xB9,16, {0x00,0xD2,0x01,0x0E,0x01,0x39,0x01,0x7E,0x01,0xB2,0x02,0x02,0x02,0x41,0x02,0x43}},
	{0xBA,16, {0x02,0x7F,0x02,0xC2,0x02,0xEF,0x03,0x23,0x03,0x48,0x03,0x70,0x03,0x7F,0x03,0x8C}},
	{0xBB,12, {0x03,0x9B,0x03,0xAA,0x03,0xB9,0x03,0xD6,0x03,0xCF,0x03,0xCB}},

	//R+C1
	{0xC6,1, {0x11}},
	{0xC7,1, {0x11}},
	{0xC8,1, {0x11}},
	{0xC9,1, {0x11}},
	{0xCA,1, {0x11}},
	//R-C1
	{0xCB,1, {0x11}},
	{0xCC,1, {0x11}},
	{0xCD,1, {0x11}},
	{0xCE,1, {0x11}},
	{0xCF,1, {0x11}},
	//G+C1
	{0xD0,1, {0x11}},
	{0xD1,1, {0x11}},
	{0xD2,1, {0x11}},
	{0xD3,1, {0x11}},
	{0xD4,1, {0x11}},
	//G-C1
	{0xD5,1, {0x11}},
	{0xD6,1, {0x11}},
	{0xD7,1, {0x11}},
	{0xD8,1, {0x11}},
	{0xD9,1, {0x11}},
	//B+C1
	{0xDA,1, {0x11}},
	{0xDB,1, {0x11}},
	{0xDC,1, {0x11}},
	{0xDD,1, {0x11}},
	{0xDE,1, {0x11}},
	//B-C1
	{0xDF,1, {0x11}},
	{0xE0,1, {0x11}},
	{0xE1,1, {0x11}},
	{0xE2,1, {0x11}},
	{0xE3,1, {0x11}},

	{0xFF,1, {0x21}},
	{0xFB,1, {0x01}},
	//R(-)
	{0xB0,16, {0x00,0x00,0x00,0x18,0x00,0x3E,0x00,0x5E,0x00,0x78,0x00,0x90,0x00,0xA6,0x00,0xB9}},
	{0xB1,16, {0x00,0xCA,0x01,0x06,0x01,0x31,0x01,0x76,0x01,0xAA,0x01,0xFA,0x02,0x39,0x02,0x3B}},
	{0xB2,16, {0x02,0x77,0x02,0xBA,0x02,0xE7,0x03,0x1B,0x03,0x40,0x03,0x68,0x03,0x77,0x03,0x84}},
	{0xB3,12, {0x03,0x93,0x03,0xA2,0x03,0xB1,0x03,0xCE,0x03,0xEF,0x03,0xFD}},
	//G(-)
	{0xB4,16, {0x00,0x00,0x00,0x18,0x00,0x3E,0x00,0x5E,0x00,0x78,0x00,0x90,0x00,0xA6,0x00,0xB9}},
	{0xB5,16, {0x00,0xCA,0x01,0x06,0x01,0x31,0x01,0x76,0x01,0xAA,0x01,0xFA,0x02,0x39,0x02,0x3B}},
	{0xB6,16, {0x02,0x77,0x02,0xBA,0x02,0xE7,0x03,0x1B,0x03,0x40,0x03,0x68,0x03,0x77,0x03,0x84}},
	{0xB7,12, {0x03,0x93,0x03,0xA2,0x03,0xB1,0x03,0xCE,0x03,0xEF,0x03,0xFD}},
	//B(-)
	{0xB8,16, {0x00,0x00,0x00,0x18,0x00,0x3E,0x00,0x5E,0x00,0x78,0x00,0x90,0x00,0xA6,0x00,0xB9}},
	{0xB9,16, {0x00,0xCA,0x01,0x06,0x01,0x31,0x01,0x76,0x01,0xAA,0x01,0xFA,0x02,0x39,0x02,0x3B}},
	{0xBA,16, {0x02,0x77,0x02,0xBA,0x02,0xE7,0x03,0x1B,0x03,0x40,0x03,0x68,0x03,0x77,0x03,0x84}},
	{0xBB,12, {0x03,0x93,0x03,0xA2,0x03,0xB1,0x03,0xCE,0x03,0xEF,0x03,0xFD}},

	//CABC Setting
	{0xFF,1, {0x23}},
	{0xFB,1, {0x01}},
	//APL_WT
	{0x10,1, {0x40}},
	//APL_THD
	{0x11,1, {0x01}},
	{0x12,1, {0x77}},
	//APL_COMP
	{0x15,1, {0xE0}},
	{0x16,1, {0x0C}},

	{0x19,1, {0x00}},
	{0x1A,1, {0x04}},
	{0x1B,1, {0x08}},
	{0x1C,1, {0x0C}},
	{0x1D,1, {0x10}},
	{0x1E,1, {0x14}},
	{0x1F,1, {0x18}},
	{0x20,1, {0x1C}},
	{0x21,1, {0x20}},
	{0x22,1, {0x24}},
	{0x23,1, {0x28}},
	{0x24,1, {0x2C}},
	{0x25,1, {0x30}},
	{0x26,1, {0x34}},
	{0x27,1, {0x38}},
	{0x28,1, {0x3C}},
	//GAMMACMP
	{0x29,1, {0x10}},
	{0x2A,1, {0x20}},
	{0x2B,1, {0x30}},

	//CABC_PWM_UI++++++++++++++++++++++++++++++++++++++
	//White
	{0x30,1, {0xF8}},
	{0x31,1, {0xF6}},
	{0x32,1, {0xF3}},
	{0x33,1, {0xF0}},
	{0x34,1, {0xEE}},
	{0x35,1, {0xEB}},
	{0x36,1, {0xE8}},
	{0x37,1, {0xE4}},

	{0x38,1, {0xE2}},
	{0x39,1, {0xE0}},
	//Gray
	{0x3A,1, {0xDD}},
	{0x3B,1, {0xD8}},
	{0x3D,1, {0xD4}},
	{0x3F,1, {0xD0}},
	{0x40,1, {0xCD}},
	//Black
	{0x41,1, {0xC8}},

	//CABC_PWM_STILL++++++++++++++++++++++++++++++++++++++
	//White
	{0x45,1, {0xEE}},
	{0x46,1, {0xEB}},
	{0x47,1, {0xE6}},
	{0x48,1, {0xD3}},
	{0x49,1, {0xD0}},
	{0x4A,1, {0xCD}},
	{0x4B,1, {0xC8}},
	{0x4C,1, {0xC3}},
	{0x4D,1, {0xC0}},
	{0x4E,1, {0xBD}},
	//Gray
	{0x4F,1, {0xBB}},
	{0x50,1, {0xB4}},
	{0x51,1, {0xB0}},
	{0x52,1, {0xAD}},
	{0x53,1, {0xA8}},
	//Black
	{0x54,1, {0xA4}},
	{0xFF,1, {0x20}},
	{0xFB,1, {0x01}},
	{0xA0,1, {0x02}},

	//{0xFF,1, {0x10}},
	//CABC UI Mode (5%)
	//{0x55,1, {0x01}},
	//CABC STILL Mode (25%)
	//{0x55,1, {0x02}},
	//CABC MOV Mode (40%)
	//{0x55,1, {0x03}},

	{0xFF,1, {0x23}},
	{0xFB,1, {0x01}},
	{0x00,1, {0x80}},
	{0x07,1, {0x00}},
	{0x08,1, {0x01}},
	{0x09,1, {0x2C}},
	//{0x11,1, {0x01}},
	//{0x12,1, {0x77}},
	//{0x15,1, {0x07}},
	//{0x16,1, {0x07}},

	{0xFF,1, {0x24}},
	{REGFLAG_DELAY, 10, {}},
	{0xFB,1, {0x01}},
	{0x91,1, {0x44}},
	{0x92,1, {0x80}},
	{0x93,1, {0x1A}},
	{0x94,1, {0x5B}},
	{0x9A,1, {0x08}},
	{0x60,1, {0x96}},
	{0x61,1, {0xD0}},
	{0x63,1, {0x70}},
	{0xC2,1, {0xC6}},

	{0x00,1, {0x03}},
	{0x01,1, {0x03}},
	{0x02,1, {0x03}},
	{0x03,1, {0x03}},
	{0x04,1, {0x03}},
	{0x05,1, {0x03}},
	{0x06,1, {0x03}},
	{0x07,1, {0x03}},
	{0x08,1, {0x22}},
	{0x09,1, {0x06}},
	{0x0A,1, {0x05}},
	{0x0B,1, {0x1D}},
	{0x0C,1, {0x1C}},
	{0x0D,1, {0x11}},
	{0x0E,1, {0x10}},
	{0x0F,1, {0x0F}},
	{0x10,1, {0x0E}},
	{0x11,1, {0x0D}},
	{0x12,1, {0x0C}},
	{0x13,1, {0x04}},
	{0x14,1, {0x03}},
	{0x15,1, {0x03}},
	{0x16,1, {0x03}},
	{0x17,1, {0x03}},
	{0x18,1, {0x03}},
	{0x19,1, {0x03}},
	{0x1A,1, {0x03}},
	{0x1B,1, {0x03}},
	{0x1C,1, {0x03}},
	{0x1D,1, {0x03}},
	{0x1E,1, {0x22}},
	{0x1F,1, {0x06}},
	{0x20,1, {0x05}},
	{0x21,1, {0x1D}},
	{0x22,1, {0x1C}},
	{0x23,1, {0x11}},
	{0x24,1, {0x10}},
	{0x25,1, {0x0F}},
	{0x26,1, {0x0E}},
	{0x27,1, {0x0D}},
	{0x28,1, {0x0C}},
	{0x29,1, {0x04}},
	{0x2A,1, {0x03}},
	{0x2B,1, {0x03}},
	//STV
	{0x2F,1, {0x04}},
	{0x30,1, {0x32}},
	{0x31,1, {0x41}},
	{0x33,1, {0x32}},
	{0x34,1, {0x04}},
	{0x35,1, {0x41}},
	{0x37,1, {0x44}},
	{0x38,1, {0x40}},
	{0x39,1, {0x00}},
	{0x3A,1, {0x01}},
	{0x3B,1, {0x4E}},
	{0x3D,1, {0x93}},
	{0xAB,1, {0x44}},
	{0xAC,1, {0x44}},
	//GCK
	{0x4D,1, {0x21}},
	{0x4E,1, {0x43}},
	{0x4F,1, {0x65}},
	{0x51,1, {0x34}},
	{0x52,1, {0x12}},
	{0x53,1, {0x56}},
	{0x55,2, {0x83,0x03}},
	{0x56,1, {0x06}},
	{0x58,1, {0x21}},
	{0x59,1, {0x40}},
	{0x5A,1, {0x01}},
	{0x5B,1, {0x4E}},
	{0x5E,2, {0x00,0x0C}},
	{0x5F,1, {0x00}},
	//EQ
	{0x5C,1, {0x88}},
	{0x5D,1, {0x08}},
	{0x8D,1, {0x88}},
	{0x8E,1, {0x08}},
	//POL
	{0x7A,1, {0x00}},
	{0x7B,1, {0x00}},
	{0x7C,1, {0x00}},
	{0x7D,1, {0x00}},
	{0x7E,1, {0x20}},
	{0x7F,1, {0x3C}},
	{0x80,1, {0x00}},
	{0x81,1, {0x00}},

	{0x82,1, {0x08}},

	//{0x83,1, {0x1B}},
	{0x97,1, {0x02}},
	//SOE
	{0xD7,1, {0x55}},
	{0xD8,1, {0x55}},
	{0xD9,1, {0x23}},
	{0xDA,1, {0x05}},
	{0xDB,1, {0x01}},
	{0xDC,1, {0x80}},
	{0xDD,1, {0x55}},
	{0xDE,1, {0x27}},
	{0xDF,1, {0x01}},
	{0xE0,1, {0x80}},
	{0xE1,1, {0x01}},
	{0xE2,1, {0x80}},
	{0xE3,1, {0x01}},
	{0xE4,1, {0x80}},
	{0xE5,1, {0x01}},
	{0xE6,1, {0x80}},
	{0xE7,1, {0x00}},
	{0xE8,1, {0x00}},
	{0xE9,1, {0x01}},
	{0xEA,1, {0x80}},
	{0xEB,1, {0x01}},
	{0xEE,1, {0x80}},
	{0xEF,1, {0x01}},
	{0xF0,1, {0x80}},

	{0xC5,1, {0x10}},

	{0xFF,1, {0x25}},
	{REGFLAG_DELAY, 10, {}},
	{0xFB,1, {0x01}},
	//Auto porch
	//{0x05,1, {0x00}},
	//LINE_N
	{0x1E,1, {0x00}},
	{0x1F,1, {0x01}},
	{0x20,1, {0x4E}},
	//LINE_N+1
	{0x25,1, {0x00}},
	{0x26,1, {0x01}},
	{0x27,1, {0x4E}},
	//TP3
	{0x3F,1, {0x80}},
	{0x40,1, {0x00}},
	{0x43,1, {0x00}},
	{0x44,1, {0x01}},
	{0x45,1, {0x4E}},

	{0x48,1, {0x01}},
	{0x49,1, {0x4E}},
	//LAST_TP0
	{0x5B,1, {0x80}},
	{0x5C,1, {0x00}},
	{0x5D,1, {0x01}},
	{0x5E,1, {0x4E}},

	{0x61,1, {0x01}},
	{0x62,1, {0x4E}},
	{0x68,1, {0x0C}},

	{0xFF,1, {0x26}},
	{REGFLAG_DELAY, 10, {}},
	{0xFB,1, {0x01}},
	{0x01,1, {0x30}},
	{0x00,1, {0xA1}},
	{0x02,1, {0x31}},
	{0x04,1, {0x28}},
	{0x0A,1, {0xF3}},
	{0x06,1, {0x32}},
	{0x0C,1, {0x11}},
	{0x0D,1, {0x00}},
	{0x0F,1, {0x09}},
	{0x11,1, {0x00}},
	{0x12,1, {0x50}},
	{0x13,1, {0x6C}},
	{0x14,1, {0x6C}},
	{0x15,1, {0x00}},
	{0x16,1, {0x90}},
	{0x17,1, {0xA0}},
	{0x18,1, {0x86}},
	{0x19,1, {0x15}},
	{0x1A,1, {0x00}},
	{0x1B,1, {0x15}},
	{0x1C,1, {0x00}},
	{0x2A,1, {0x15}},
	{0x2B,1, {0x00}},
	//RTNA_LINE N/ N+1
	{0x1D,1, {0x00}},
	{0x1E,1, {0x80}},
	{0x1F,1, {0x80}},
	//RTNA_TP3
	{0x2F,1, {0x05}},
	{0x30,1, {0x80}},
	{0x33,1, {0x11}},
	{0x34,1, {0x78}},
	{0x35,1, {0x16}},
	//LAST_TP0
	{0x39,1, {0x0C}},
	{0x3A,1, {0x80}},
	{0x3B,1, {0x06}},

	{0xC8,1, {0x04}},
	{0xC9,1, {0x08}},
	{0xCA,1, {0x4E}},
	{0xCB,1, {0x00}},

	{0xA9,1, {0x79}},
	{0xAA,1, {0x74}},
	{0xAB,1, {0x6F}},
	{0xAC,1, {0x6D}},
	{0xAD,1, {0x71}},
	{0xAE,1, {0x80}},
	{0xAF,1, {0x7E}},
	{0xB0,1, {0x77}},
	{0xB1,1, {0x79}},
	{0xB2,1, {0x80}},

	{0xFF,1, {0x27}},
	{REGFLAG_DELAY, 10, {}},
	{0xFB,1, {0x01}},
	{0x14,1, {0x52}},
	{0xD2,1, {0x30}},
	{0xC0,1, {0x18}},
	{0xC1,1, {0x00}},
	{0xC2,1, {0x00}},
	//VPOR_DYNH_EN=1, VPOR_CNT_REV=1
	{0x56,1, {0x06}},
	//FR0
	{0x58,1, {0x00}},
	{0x59,1, {0x46}},
	{0x5A,1, {0x00}},

	{0x5B,1, {0x13}},
	{0x5C,1, {0x00}},
	{0x5D,1, {0x01}},
	{0x5E,1, {0x20}},
	{0x5F,1, {0x10}},
	{0x60,1, {0x00}},
	{0x61,1, {0x11}},
	{0x62,1, {0x00}},
	{0x63,1, {0x01}},
	{0x64,1, {0x21}},
	{0x65,1, {0x0F}},
	{0x66,1, {0x00}},
	{0x67,1, {0x01}},
	{0x68,1, {0x22}},

	//TC
	{0x98,1, {0x01}},
	{0xB4,1, {0x03}},
	//HT
	{0x9B,1, {0xBE}},
	{0x9C,1, {0x00}},
	{0xA0,1, {0x15}},
	//LT
	{0xAB,1, {0x14}},
	{0xAC,1, {0x00}},
	{0xB0,1, {0x83}},
	//0C_60C
	{0xBC,1, {0x10}},
	{0xBD,1, {0x28}},

	{0xFF,1, {0x2A}},
	{REGFLAG_DELAY, 10, {}},
	{0xFB,1, {0x01}},
	{0xF1,1, {0x00}},
	//PEN_EN=1,UL_FREQ=0
	{0x22,1, {0x2F}},
	{0x23,1, {0x08}},
	//FR0 (60Hz,VFP=26)
	{0x24,1, {0x00}},
	{0x25,1, {0x7F}},
	{0x26,1, {0xF8}},
	{0x27,1, {0x00}},
	{0x28,1, {0x1A}},
	{0x29,1, {0x00}},
	{0x2A,1, {0x1A}},
	{0x2B,1, {0x00}},
	{0x2D,1, {0x1A}},
	{0x64,1, {0x96}},
	{0x65,1, {0x00}},
	{0x66,1, {0x00}},
	{0x6A,1, {0x96}},
	{0x6B,1, {0x00}},
	{0x6C,1, {0x00}},
	{0x70,1, {0x92}},
	{0x71,1, {0x00}},
	{0x72,1, {0x00}},

	{0xA2,1, {0x33}},
	{0xA3,1, {0x30}},
	{0xA4,1, {0xC0}},
	{0xE8,1, {0x00}},

	{0xFF,1, {0xF0}},
	{0xFB,1, {0x01}},
	{0x3A,1, {0x08}},

	{0xFF,1, {0xD0}},
	{0xFB,1, {0x01}},
	{0x00,1, {0x33}},
	//ESD check setting
	{0xFF,1, {0xF0}},
	{0xFB,1, {0x01}},
	{0x84,1, {0x08}},
	{0x85,1, {0x0C}},

	{0xFF,1, {0x20}},
	{0xFB,1, {0x01}},
	{0x30,1, {0x00}},
	{0x51,1, {0x00}},

	{0xFF,1, {0x25}},
	{0xFB,1, {0x01}},
	{0x91,1, {0x1F}},
	{0x92,1, {0x0F}},
	{0x93,1, {0x01}},
	{0x94,1, {0x18}},
	{0x95,1, {0x03}},
	{0x96,1, {0x01}},
	{0xFF,1, {0x10}},
	{REGFLAG_DELAY, 10, {}},
	{0xB9,1, {0x01}},
	{0xFF,1, {0x20}},
	{REGFLAG_DELAY, 10, {}},
	{0x18,1, {0x40}},
	{0xFF,1, {0x10}},
	{REGFLAG_DELAY, 10, {}},
	{0xB9,1, {0x02}},

	{0xFF,1, {0x10}},
	{REGFLAG_DELAY, 10, {}},
	{0xFB,1, {0x01}},
	{0xBB,1, {0x13}},
	{0x53,1, {0x2C}},
	{0x68,2, {0x02,0x01}},
	{0x3B,5, {0x03,0x08,0x1A,0x04,0x04}},
	{0x35,1, {0x00}},
	{0x11,0, {}},
	{REGFLAG_DELAY, 100, {}},
	{0x29,0, {}},
	{REGFLAG_DELAY, 10, {}},

	{0xFF,1, {0x27}},
	{REGFLAG_DELAY, 10, {}},
	{0xD0,1, {0x11}},
	{0xD1,1, {0x54}},
	{0xDE,1, {0x43}},
	{0xDF,1, {0x02}},
	{0xFF,1, {0x10}},
	{REGFLAG_DELAY, 10, {}},
};

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	// Display off sequence
	{0x28, 0, {0x00}},
	{REGFLAG_DELAY, 20, {}},
	// Sleep Mode On
	{0x10, 0, {0x00}},
	{REGFLAG_DELAY, 120, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}},
};

static struct LCM_setting_table bl_level[] = {
	{0xFF, 0x01, {0x10} },
	{0xFB, 0x01, {0x01} },
	{0x51, 0x02, {0x0F, 0xFF} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

static struct LCM_setting_table cabc_mode[] = {
	{0xFF, 0x01, {0x10} },
	{0xFB, 0x01, {0x01} },
	{0x55, 0x01, {0x00} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

static struct LCM_setting_table cabc_mode0[] = {
	{0xFF, 0x01, {0x10} },
	{0xB9, 0x01, {0x00} },
	{0x55, 0x01, {0x00} },
	{0xB9, 0x01, {0x02} },
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
			if (table[i].count <= 10)
				MDELAY(table[i].count);
			else
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

	params->dsi.vertical_sync_active = 1;
	params->dsi.vertical_backporch = 7;
	params->dsi.vertical_frontporch = 26;
	//params->dsi.vertical_frontporch_for_low_power = 750;	//OTM no data
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 1;
	params->dsi.horizontal_backporch = 25;
	params->dsi.horizontal_frontporch = 25;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;
	/* params->dsi.ssc_disable = 1; */

	params->dsi.PLL_CLOCK = 488;
	params->dsi.PLL_CK_CMD = 440;

	params->dsi.CLK_HS_POST = 36;
	params->dsi.clk_lp_per_line_enable = 0;
	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 0;
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	params->round_corner_en = 1;
	params->corner_pattern_width = 1200;
	params->corner_pattern_height = 31;
	params->corner_pattern_height_bot = 31;
#endif
}

static void lcm_set_gpio_output(unsigned int GPIO, unsigned int output)
{
	LCM_LOGI("%s,nt36523b\n", __func__);
	gpio_direction_output(GPIO, output);
	gpio_set_value(GPIO, output);
}

static void lcm_bias_regulator_init()
{
	LCM_LOGI("%s,nt36523b\n", __func__);
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
	LCM_LOGI("%s,nt36523b\n", __func__);

	/* set voltage with min & max*/
	lcm_bias_regulator_init();
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
	LCM_LOGI("%s,nt36523b\n", __func__);
	lcm_set_gpio_output(GPIO_LCD_BIAS_ENN, 0);
	MDELAY(10);
	lcm_set_gpio_output(GPIO_LCD_BIAS_ENP, 0);
	MDELAY(10);
	lcm_set_gpio_output(GPIO_LCD_1V8, 1);
	MDELAY(10);
	return 0;
}

/* turn on gate ic & control voltage to 5.5V */
/* equle display_bais_enable ,mt6768 need +/-5.5V */
static void lcm_init_power(void)
{
	LCM_LOGI("%s,nt36523b\n", __func__);
	lcm_bias_enable();
}

extern int tp_gesture_enable_flag(void);
static void lcm_suspend_power(void)
{
	LCM_LOGI("%s,nt36523b \n", __func__);
	if (0 == tp_gesture_enable_flag()) {
		SET_RESET_PIN(1);
		lcm_bias_disable();
	}
}

/* turn on gate ic & control voltage to 5.5V */
static void lcm_resume_power(void)
{
	LCM_LOGI("%s,nt36523b\n", __func__);
	lcm_init_power();
}
extern void lcd_queue_load_tp_fw(void);
static void lcm_init(void)
{
	SET_RESET_PIN(0);
	MDELAY(15);
	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(10);

	lcd_queue_load_tp_fw();
	MDELAY(5);
	push_table(NULL, lcm_initialization_setting, ARRAY_SIZE(lcm_initialization_setting), 1);
	LCM_LOGI("nt36523_fhdp----tps6132----lcm mode = vdo mode :%d----\n",
		 lcm_dsi_mode);
	lcm_set_gpio_output(GPIO_LCD_BL_EN, 1);
	sgm_write_i2c(0x10, 0x1f);
	sgm_write_i2c(0x11, 0x25);
}

static void lcm_suspend(void)
{
	LCM_LOGI("%s,nt36523b\n", __func__);
	lcm_set_gpio_output(GPIO_LCD_BL_EN, 0);
	push_table(NULL, lcm_deep_sleep_mode_in_setting,
		   ARRAY_SIZE(lcm_deep_sleep_mode_in_setting), 1);
}

static void lcm_resume(void)
{
	LCM_LOGI("%s,nt36523b\n", __func__);
	lcm_init();
}
static unsigned int lcm_compare_id(void)
{
	return 1;
}

static unsigned int global_lcm1_level = 0;
static void lcm_setbacklight_cmdq(void *handle, unsigned int level)
{
	//LCM_LOGI("%s,nt36523b backlight: level = %d\n", __func__, level);
	//level = level*4095/255;
	MDELAY(10);
	LCM_LOGI("%s,nt36523b backlight: level = %d\n", __func__, level);
	if (level > 4095)
		level = 4095;
	global_lcm1_level = level;
	if ((level > 0) && (level < oplus_max_normal_brightness)) {
		level = map_exp[level];
	}
	pr_err ("%s after change backlight = %d\n", __func__, level);
	bl_level[2].para_list[0] = 0x0F&(level >> 8);
	bl_level[2].para_list[1] = 0xFF&level;

	push_table(handle, bl_level, ARRAY_SIZE(bl_level), 1);
}

unsigned int global_lcm1_getbacklight(void)
{
	return global_lcm1_level;
}
EXPORT_SYMBOL(global_lcm1_getbacklight);

void global_lcm1_setbacklight(unsigned int level)
{
	MDELAY(10);
	LCM_LOGI("%s,nt36523b backlight: level = %d\n", __func__, level);
	if (level > 4095)
		level = 4095;
	if ((level > 0) && (level < oplus_max_normal_brightness)) {
		level = map_exp[level];
	}
	pr_err ("%s after change backlight = %d\n", __func__, level);
	bl_level[2].para_list[0] = 0x0F&(level >> 8);
	bl_level[2].para_list[1] = 0xFF&level;

	push_table(NULL, bl_level, ARRAY_SIZE(bl_level), 1);
}
EXPORT_SYMBOL(global_lcm1_setbacklight);

static int cabc_status;
static void lcm_set_cabc_cmdq(void *handle, unsigned int level){
	pr_err("[lcm] cabc set level %d\n", level);
	if (level==1){
		cabc_mode[2].para_list[0] = 0x01;
		push_table(handle, cabc_mode, ARRAY_SIZE(cabc_mode), 1);
	}else if(level==2){
		cabc_mode[2].para_list[0] = 0x02;
		push_table(handle, cabc_mode, ARRAY_SIZE(cabc_mode), 1);
	}else{
		push_table(handle, cabc_mode0, ARRAY_SIZE(cabc_mode0), 1);
	}
	cabc_status = level;
}

static void lcm_get_cabc_status(int *status){
	pr_info("[lcm] cabc get to %d\n", cabc_status);
	*status = cabc_status;
}

struct LCM_DRIVER nt36523b_hdp_dsi_vdo_dijing_lcm_drv =
{
	.name			= "nt36523b_hdp_dsi_vdo_dijing",
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
