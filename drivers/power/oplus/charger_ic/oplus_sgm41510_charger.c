
/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#include <linux/types.h>
#include <linux/init.h>		/* For init/exit macros */
#include <linux/module.h>	/* For MODULE_ marcros  */
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <mt-plat/mtk_boot.h>
#include <mt-plat/upmu_common.h>
#include <mt-plat/charger_type.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include "oplus_sgm41510.h"
#include "../../supply/mediatek/charger/mtk_charger_intf.h"
#include <linux/power_supply.h>
#include <mt-plat/charger_class.h>
#include "../oplus_charger.h"

static const u32 sgm41510_wdt[] = {
	0, 40, 80, 160,
};

enum sgm41510_usbsw_state {
	SGM41510_USBSW_CHG = 0,
	SGM41510_USBSW_USB,
};
enum hvdcp_type {
	HVDCP_5V,
	HVDCP_9V,
	HVDCP_12V,
	HVDCP_20V,
	HVDCP_CONTINOUS,
	HVDCP_DPF_DMF,
};

struct chg_para{
	int vlim;
	int ilim;

	int vreg;
	int ichg;
};

struct sgm41510_platform_data {
	int iprechg;
	int iterm;

	int boostv;
	int boosti;

	struct chg_para usb;
};

struct sgm41510 {
	struct device *dev;
	struct i2c_client *client;
	struct delayed_work sgm41510_aicr_setting_work;
	struct delayed_work sgm41510_retry_adapter_detection;
	struct delayed_work sgm41510_current_setting_work;

	int revision;

	const char *chg_dev_name;
	const char *eint_name;

	bool chg_det_enable;
	bool otg_enable;

	enum charger_type chg_type;
	enum power_supply_type oplus_chg_type;
	
	int status;
	int irq;

	struct mutex i2c_rw_lock;

	bool charge_enabled;	/* Register bit status */
	bool power_good;

	struct sgm41510_platform_data *platform_data;
	struct charger_device *chg_dev;
	struct timespec ptime[2];

	struct power_supply *psy;
	struct charger_consumer *chg_consumer;
	struct charger_properties chg_props;
	bool disable_hight_vbus;
	bool pdqc_setup_5v;
	bool hvdcp_can_enabled;
	int pre_current_ma;
	int aicr;
	int chg_cur;
	int vbus_type;
	bool hvdcp_checked;
	int hw_aicl_point;
	bool retry_hvdcp_algo;
	bool nonstand_retry_bc;
	bool camera_on;
	bool calling_on;
	struct mutex bc12_access_lock;
	int sgm41510_psel_gpio;
	int sgm41510_otgen_gpio;
#ifdef CONFIG_TCPC_CLASS
	atomic_t tcpc_usb_connected;
#endif
	bool bc12_en;
	atomic_t bc12_sdp_cnt;
	atomic_t bc12_wkard;
	bool pwr_rdy;
#ifndef CONFIG_COMPILE_FACTORY_VERSION
	struct mutex		irq_complete;
	bool		resume_completed;
	bool		irq_waiting;
	struct delayed_work		charge_status_detect;
#endif
};

#define MODE_QC20_V5			0x01
#define MODE_QC20_V9			0x02
#define MODE_QC20_V12			0x03
#define MODE_QC30_V5			0x04
#define MODE_QC3_PLUS_V5		0x05

static bool disable_PE = 0;
static bool disable_QC = 0;
static bool disable_PD = 0;
static bool dumpreg_by_irq = 0;

module_param(disable_PE, bool, 0644);
module_param(disable_QC, bool, 0644);
module_param(disable_PD, bool, 0644);

module_param(dumpreg_by_irq, bool, 0644);
static struct sgm41510 *g_sgm;

extern struct oplus_chg_chip *g_oplus_chip;

static struct i2c_client *new_client;
static const struct i2c_device_id sgm41510_i2c_id[] = { {"sgm41510", 0}, {} };

extern void set_charger_ic(int sel);

static int sgm41510_driver_probe(struct i2c_client *client,
				const struct i2c_device_id *id);
static int sgm41510_set_watch_dog_timer(struct sgm41510 *sgm, u32 sec);

/**********************************************************
 *
 *   [Global Variable]
 *
 *********************************************************/
unsigned char sgm41510_reg[sgm41510_REG_NUM] = { 0 };

static DEFINE_MUTEX(sgm41510_i2c_access);
static DEFINE_MUTEX(sgm41510_access_lock);

int g_sgm41510_hw_exist;

/**********************************************************
 *
 *   [I2C Function For Read/Write sgm41510]
 *
 *********************************************************/
#ifdef CONFIG_MTK_I2C_EXTENSION
unsigned int sgm41510_read_byte(unsigned char cmd,
			       unsigned char *returnData)
{
	char cmd_buf[1] = { 0x00 };
	char readData = 0;
	int ret = 0;

	mutex_lock(&sgm41510_i2c_access);

	/* new_client->addr = ((new_client->addr) & I2C_MASK_FLAG) |
	 * I2C_WR_FLAG;
	 */
	new_client->ext_flag =
		((new_client->ext_flag) & I2C_MASK_FLAG) | I2C_WR_FLAG |
		I2C_DIRECTION_FLAG;

	cmd_buf[0] = cmd;
	ret = i2c_master_send(new_client, &cmd_buf[0], (1 << 8 | 1));
	if (ret < 0) {
		/* new_client->addr = new_client->addr & I2C_MASK_FLAG; */
		new_client->ext_flag = 0;
		mutex_unlock(&sgm41510_i2c_access);

		return 0;
	}

	readData = cmd_buf[0];
	*returnData = readData;

	/* new_client->addr = new_client->addr & I2C_MASK_FLAG; */
	new_client->ext_flag = 0;
	mutex_unlock(&sgm41510_i2c_access);

	return 1;
}

unsigned int sgm41510_write_byte(unsigned char cmd,
				unsigned char writeData)
{
	char write_data[2] = { 0 };
	int ret = 0;

	mutex_lock(&sgm41510_i2c_access);

	write_data[0] = cmd;
	write_data[1] = writeData;

	new_client->ext_flag = ((new_client->ext_flag) & I2C_MASK_FLAG) |
			       I2C_DIRECTION_FLAG;

	ret = i2c_master_send(new_client, write_data, 2);
	if (ret < 0) {
		new_client->ext_flag = 0;
		mutex_unlock(&sgm41510_i2c_access);
		return 0;
	}

	new_client->ext_flag = 0;
	mutex_unlock(&sgm41510_i2c_access);
	return 1;
}
#else
unsigned int sgm41510_read_byte(unsigned char cmd,
			       unsigned char *returnData)
{
	unsigned char xfers = 2;
	int ret, retries = 1;

	mutex_lock(&sgm41510_i2c_access);

	do {
		struct i2c_msg msgs[2] = {
			{
				.addr = new_client->addr,
				.flags = 0,
				.len = 1,
				.buf = &cmd,
			},
			{

				.addr = new_client->addr,
				.flags = I2C_M_RD,
				.len = 1,
				.buf = returnData,
			}
		};

		/*
		 * Avoid sending the segment addr to not upset non-compliant
		 * DDC monitors.
		 */
		ret = i2c_transfer(new_client->adapter, msgs, xfers);

		if (ret == -ENXIO) {
			pr_info("skipping non-existent adapter %s\n",
				new_client->adapter->name);
			break;
		}
	} while (ret != xfers && --retries);

	mutex_unlock(&sgm41510_i2c_access);

	return ret == xfers ? 1 : -1;
}

unsigned int sgm41510_write_byte(unsigned char cmd,
				unsigned char writeData)
{
	unsigned char xfers = 1;
	int ret, retries = 1;
	unsigned char buf[8];

	mutex_lock(&sgm41510_i2c_access);

	buf[0] = cmd;
	memcpy(&buf[1], &writeData, 1);

	do {
		struct i2c_msg msgs[1] = {
			{
				.addr = new_client->addr,
				.flags = 0,
				.len = 1 + 1,
				.buf = buf,
			},
		};

		/*
		 * Avoid sending the segment addr to not upset non-compliant
		 * DDC monitors.
		 */
		ret = i2c_transfer(new_client->adapter, msgs, xfers);

		if (ret == -ENXIO) {
			pr_info("skipping non-existent adapter %s\n",
				new_client->adapter->name);
			break;
		}
	} while (ret != xfers && --retries);

	mutex_unlock(&sgm41510_i2c_access);

	return ret == xfers ? 1 : -1;
}
#endif
/**********************************************************
 *
 *   [Read / Write Function]
 *
 *********************************************************/
unsigned int sgm41510_read_interface(unsigned char RegNum,
				    unsigned char *val, unsigned char MASK,
				    unsigned char SHIFT)
{
	unsigned char sgm41510_reg = 0;
	unsigned int ret = 0;

	ret = sgm41510_read_byte(RegNum, &sgm41510_reg);

	pr_debug_ratelimited("[%s] Reg[%x]=0x%x\n", __func__,
			     RegNum, sgm41510_reg);

	sgm41510_reg &= (MASK << SHIFT);
	*val = (sgm41510_reg >> SHIFT);

	pr_debug_ratelimited("[%s] val=0x%x\n", __func__, *val);

	return ret;
}

unsigned int sgm41510_config_interface(unsigned char RegNum,
				      unsigned char val, unsigned char MASK,
				      unsigned char SHIFT)
{
	unsigned char sgm41510_reg = 0;
	unsigned char sgm41510_reg_ori = 0;
	unsigned int ret = 0;

	mutex_lock(&sgm41510_access_lock);
	ret = sgm41510_read_byte(RegNum, &sgm41510_reg);

	sgm41510_reg_ori = sgm41510_reg;
	sgm41510_reg &= ~(MASK << SHIFT);
	sgm41510_reg |= (val << SHIFT);

	ret = sgm41510_write_byte(RegNum, sgm41510_reg);
	mutex_unlock(&sgm41510_access_lock);
	pr_debug_ratelimited("[%s] write Reg[%x]=0x%x from 0x%x\n", __func__,
			     RegNum,
			     sgm41510_reg, sgm41510_reg_ori);

	/* Check */
	/* sgm41510_read_byte(RegNum, &sgm41510_reg);
	pr_info("[%s] Check Reg[%x]=0x%x\n", __func__,
			RegNum, sgm41510_reg); */

	return ret;
}

/* write one register directly */
unsigned int sgm41510_reg_config_interface(unsigned char RegNum,
		unsigned char val)
{
	unsigned int ret = 0;

	ret = sgm41510_write_byte(RegNum, val);

	return ret;
}

/**********************************************************
 *
 *   [Internal Function]
 *
 *********************************************************/
/* CON0---------------------------------------------------- */
void sgm41510_set_en_hiz(unsigned int val)
{
	unsigned int ret = 0;

	ret = sgm41510_config_interface((unsigned char) (sgm41510_CON0),
				       (unsigned char) (val),
				       (unsigned char) (CON0_EN_HIZ_MASK),
				       (unsigned char) (CON0_EN_HIZ_SHIFT)
				      );
}

void sgm41510_set_iinlim(unsigned int val)
{
	unsigned int ret = 0;

	ret = sgm41510_config_interface((unsigned char) (sgm41510_CON0),
				       (unsigned char) (val),
				       (unsigned char) (CON0_IINLIM_MASK),
				       (unsigned char) (CON0_IINLIM_SHIFT)
				      );
}

static int sgm41510_get_iinlim(unsigned char *val)
{
	unsigned int ret = 0;

	ret = sgm41510_read_interface((unsigned char) (sgm41510_CON0),
				       (val),
				       (unsigned char) (CON0_IINLIM_MASK),
				       (unsigned char) (CON0_IINLIM_SHIFT)
				      );
	return ret;
}

void sgm41510_set_ilim_pin(unsigned int val)
{
	unsigned int ret = 0;

	ret = sgm41510_config_interface((unsigned char) (sgm41510_CON0),
				   (unsigned char) (val),
				   (unsigned char) (CON0_EN_ILIM_PIN_MASK),
				   (unsigned char) (CON0_EN_ILIM_PIN_SHIFT)
				   );
}

void sgm41510_set_auto_dpdm(unsigned int val)
{
	unsigned int ret = 0;

	ret = sgm41510_config_interface((unsigned char) (sgm41510_CON2),
				   (unsigned char) (val),
				   (unsigned char) (CON2_AUTO_DPDM_EN_MASK),
				   (unsigned char) (CON2_AUTO_DPDM_EN_SHIFT)
				   );
}

/* CON1---------------------------------------------------- */

void sgm41510_set_reg_rst(unsigned int val)
{
	unsigned int ret = 0;

	ret = sgm41510_config_interface((unsigned char) (sgm41510_CON20),
				       (unsigned char) (val),
				       (unsigned char) (CON20_REG_RST_MASK),
				       (unsigned char) (CON20_REG_RST_SHIFT)
				      );
}

void sgm41510_set_wdt_rst(unsigned int val)
{
	unsigned int ret = 0;

	ret = sgm41510_config_interface((unsigned char) (sgm41510_CON3),
				       (unsigned char) (val),
				       (unsigned char) (CON3_WDT_RST_MASK),
				       (unsigned char) (CON3_WDT_RST_SHIFT)
				      );
}

void sgm41510_set_otg_config(unsigned int val)
{
	unsigned int ret = 0;

	ret = sgm41510_config_interface((unsigned char) (sgm41510_CON3),
				       (unsigned char) (val),
				       (unsigned char) (CON3_OTG_CONFIG_MASK),
				       (unsigned char) (CON3_OTG_CONFIG_SHIFT)
				      );
}


void sgm41510_set_chg_config(unsigned int val)
{
	unsigned int ret = 0;

	ret = sgm41510_config_interface((unsigned char) (sgm41510_CON3),
				       (unsigned char) (val),
				       (unsigned char) (CON3_CHG_CONFIG_MASK),
				       (unsigned char) (CON3_CHG_CONFIG_SHIFT)
				      );
}

static int sgm41510_get_chg_config(unsigned char *val)
{
	unsigned int ret = 0;

	ret = sgm41510_read_interface((unsigned char) (sgm41510_CON3),
				       (val),
				       (unsigned char) (CON3_CHG_CONFIG_MASK),
				       (unsigned char) (CON3_CHG_CONFIG_SHIFT)
				      );
	return ret;
}

void sgm41510_set_sys_min(unsigned int val)
{
	unsigned int ret = 0;

	ret = sgm41510_config_interface((unsigned char) (sgm41510_CON3),
				       (unsigned char) (val),
				       (unsigned char) (CON3_SYS_MIN_MASK),
				       (unsigned char) (CON3_SYS_MIN_SHIFT)
				      );
}

void sgm41510_set_batlowv(unsigned int val)
{
	unsigned int ret = 0;

	ret = sgm41510_config_interface((unsigned char) (sgm41510_CON6),
				       (unsigned char) (val),
				       (unsigned char) (CON6_BATLOWV_MASK),
				       (unsigned char) (CON6_BATLOWV_SHIFT)
				      );
}

/* CON2---------------------------------------------------- */
void sgm41510_set_boost_lim(unsigned int val)
{
	unsigned int ret = 0;

	ret = sgm41510_config_interface((unsigned char) (sgm41510_CON10),
				       (unsigned char) (val),
				       (unsigned char) (CON10_BOOST_LIM_MASK),
				       (unsigned char) (CON10_BOOST_LIM_SHIFT)
				      );
}

static int sgm41510_get_ichg(unsigned char *val)
{
	unsigned int ret = 0;

	ret = sgm41510_read_interface((unsigned char) (sgm41510_CON4),
				       (val),
				       (unsigned char) (CON4_ICHG_MASK),
				       (unsigned char) (CON4_ICHG_SHIFT)
				      );
	return ret;
}

void sgm41510_set_ichg(unsigned int val)
{
	unsigned int ret = 0;

	ret = sgm41510_config_interface((unsigned char) (sgm41510_CON4),
				       (unsigned char) (val),
				       (unsigned char) (CON4_ICHG_MASK),
				       (unsigned char) (CON4_ICHG_SHIFT)
				      );
}

void sgm41510_set_iprechg(unsigned int val)
{
	unsigned int ret = 0;

	ret = sgm41510_config_interface((unsigned char) (sgm41510_CON5),
				       (unsigned char) (val),
				       (unsigned char) (CON5_IPRECHG_MASK),
				       (unsigned char) (CON5_IPRECHG_SHIFT)
				      );
}

void sgm41510_set_iterm(unsigned int val)
{
	unsigned int ret = 0;

	ret = sgm41510_config_interface((unsigned char) (sgm41510_CON5),
				       (unsigned char) (val),
				       (unsigned char) (CON5_ITERM_MASK),
				       (unsigned char) (CON5_ITERM_SHIFT)
				      );
}

static int sgm41510_get_iterm(unsigned char *val)
{
	unsigned int ret = 0;

	ret = sgm41510_read_interface((unsigned char) (sgm41510_CON5),
				       (val),
				       (unsigned char) (CON5_ITERM_MASK),
				       (unsigned char) (CON5_ITERM_SHIFT)
				      );
	return ret;
}

/* CON4---------------------------------------------------- */

void sgm41510_set_vreg(unsigned int val)
{
	unsigned int ret = 0;

	ret = sgm41510_config_interface((unsigned char) (sgm41510_CON6),
				       (unsigned char) (val),
				       (unsigned char) (CON6_VREG_MASK),
				       (unsigned char) (CON6_VREG_SHIFT)
				      );
}

static int sgm41510_get_vreg(unsigned char *val)
{
	unsigned int ret = 0;

	ret = sgm41510_read_interface((unsigned char) (sgm41510_CON6),
				       (val),
				       (unsigned char) (CON6_VREG_MASK),
				       (unsigned char) (CON6_VREG_SHIFT)
				      );
	return ret;
}

void sgm41510_set_vrechg(unsigned int val)
{
	unsigned int ret = 0;

	ret = sgm41510_config_interface((unsigned char) (sgm41510_CON6),
				       (unsigned char) (val),
				       (unsigned char) (CON6_VRECHG_MASK),
				       (unsigned char) (CON6_VRECHG_SHIFT)
				      );
}

/* CON5---------------------------------------------------- */

void sgm41510_set_en_term(unsigned int val)
{
	unsigned int ret = 0;

	ret = sgm41510_config_interface((unsigned char) (sgm41510_CON7),
				       (unsigned char) (val),
				       (unsigned char) (CON7_EN_TERM_MASK),
				       (unsigned char) (CON7_EN_TERM_SHIFT)
				      );
}



void sgm41510_set_watchdog(unsigned int val)
{
	unsigned int ret = 0;

	ret = sgm41510_config_interface((unsigned char) (sgm41510_CON7),
				       (unsigned char) (val),
				       (unsigned char) (CON7_WATCHDOG_MASK),
				       (unsigned char) (CON7_WATCHDOG_SHIFT)
				      );
}

void sgm41510_set_en_timer(unsigned int val)
{
	unsigned int ret = 0;

	ret = sgm41510_config_interface((unsigned char) (sgm41510_CON7),
				       (unsigned char) (val),
				       (unsigned char) (CON7_EN_TIMER_MASK),
				       (unsigned char) (CON7_EN_TIMER_SHIFT)
				      );
}

void sgm41510_set_chg_timer(unsigned int val)
{
	unsigned int ret = 0;

	ret = sgm41510_config_interface((unsigned char) (sgm41510_CON7),
				       (unsigned char) (val),
				       (unsigned char) (CON7_CHG_TIMER_MASK),
				       (unsigned char) (CON7_CHG_TIMER_SHIFT)
				      );
}

/* CON6---------------------------------------------------- */
static int sgm41510_set_vindpm(unsigned char val)
{
	unsigned int ret = 0;

	ret = sgm41510_config_interface((unsigned char) (sgm41510_CON13),
				       (unsigned char) (val),
				       (unsigned char) (CON13_VINDPM_MASK),
				       (unsigned char) (CON13_VINDPM_SHIFT)
				      );
	return ret;
}

static int sgm41510_get_vindpm(unsigned char *val)
{
	unsigned int ret = 0;

	ret = sgm41510_read_interface((unsigned char) (sgm41510_CON13),
				       (val),
				       (unsigned char) (CON13_VINDPM_MASK),
				       (unsigned char) (CON13_VINDPM_SHIFT)
				      );
	return ret;
}

void sgm41510_set_boostv(unsigned int val)
{

	unsigned int ret = 0;

	ret = sgm41510_config_interface((unsigned char) (sgm41510_CON10),
				       (unsigned char) (val),
				       (unsigned char) (CON10_BOOSTV_MASK),
				       (unsigned char) (CON10_BOOSTV_SHIFT)
				      );
}

unsigned int sgm41510_get_vbus_stat(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = sgm41510_read_interface((unsigned char) (sgm41510_CON11),
				     (&val),
				     (unsigned char) (CON11_VBUS_STAT_MASK),
				     (unsigned char) (CON11_VBUS_STAT_SHIFT)
				    );
	return val;
}

unsigned int sgm41510_get_chrg_stat(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = sgm41510_read_interface((unsigned char) (sgm41510_CON11),
				     (&val),
				     (unsigned char) (CON11_CHRG_STAT_MASK),
				     (unsigned char) (CON11_CHRG_STAT_SHIFT)
				    );
	return val;
}

unsigned int sgm41510_get_vsys_stat(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = sgm41510_read_interface((unsigned char) (sgm41510_CON11),
				     (&val),
				     (unsigned char) (CON11_VSYS_STAT_MASK),
				     (unsigned char) (CON11_VSYS_STAT_SHIFT)
				    );
	return val;
}

unsigned int sgm41510_get_pg_stat(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = sgm41510_read_interface((unsigned char) (sgm41510_CON11),
				     (&val),
				     (unsigned char) (CON11_PG_STAT_MASK),
				     (unsigned char) (CON11_PG_STAT_SHIFT)
				    );
	return val;
}
/*CON9----------------------------------------------------------*/
void sgm41510_set_batfet_rst_en(unsigned int val)
{
	unsigned int ret = 0;

	ret = sgm41510_config_interface((unsigned char) (sgm41510_CON9),
				       (unsigned char) (val),
				       (unsigned char) (CON9_BATFET_RST_EN_MASK),
				       (unsigned char) (CON9_BATFET_RST_EN_SHIFT)
				      );
}

/*CON10----------------------------------------------------------*/
/**********************************************************
 *
 *   [Internal Function]
 *
 *********************************************************/
static int sgm41510_dump_register(struct charger_device *chg_dev)
{
	unsigned char i = 0;
	unsigned int ret = 0;
	unsigned char buf[400];
	unsigned char *s = buf;

	sgm41510_set_wdt_rst(0x1);	/* Kick watchdog */
	if (sgm41510_get_pg_stat()) {
		s+=sprintf(s,"sgm41510_dump_regs:");
		for (i = 0; i < sgm41510_REG_NUM; i++) {
			ret = sgm41510_read_byte(i, &sgm41510_reg[i]);
			if (ret == 0) {
				pr_info("[sgm41510] i2c transfor error\n");
				return 1;
			}
			s+=sprintf(s,"[0x%.2x,0x%.2x]", i, sgm41510_reg[i]);
		}
		s+=sprintf(s,"\n");
		pr_err("%s",buf);
		if (qc_module_z350)
			z350_dump_register();
		else
			wt6670f_dump_register();
	}
	return 0;
}

static int sgm41510_enable_te(struct charger_device *chg_dev, bool en)
{
	unsigned int status = true;
	sgm41510_set_en_term(en);
	return status;
}

static inline u8 sgm41510_closest_reg(u32 min, u32 max, u32 step, u32 target)
{
	/* Smaller than minimum supported value, use minimum one */
	if (target < min)
		return 0;

	/* Greater than maximum supported value, use maximum one */
	if (target >= max)
		return (max - min) / step;

	return (target - min) / step;
}

static inline u32 sgm41510_closest_value(u32 min, u32 max, u32 step, u8 reg_val)
{
	u32 ret_val = 0;

	ret_val = min + reg_val * step;
	if (ret_val > max)
		ret_val = max;

	return ret_val;
}

static int sgm41510_get_mivr(struct charger_device *chg_dev, u32 *uV)
{
	u8 reg_mivr = 0;
	int ret = 0;

	ret = sgm41510_get_vindpm(&reg_mivr);
	if (ret < 0)
		return ret;

	*uV = sgm41510_closest_value(CON13_MIVR_MIN, CON13_MIVR_MAX,
		CON13_MIVR_STEP, reg_mivr);

	pr_info("%s: uV = %d(0x%02X)\n", __func__, *uV, reg_mivr);

	return ret;

}

static int sgm41510_set_mivr(struct charger_device *chg_dev, u32 uV)
{
	u8 reg_mivr = 0;
	reg_mivr = sgm41510_closest_reg(CON13_MIVR_MIN, CON13_MIVR_MAX,
		CON13_MIVR_STEP, uV);

	pr_info("%s: uV = %d(0x%02X)\n", __func__, uV, reg_mivr);

	return sgm41510_set_vindpm(reg_mivr);
}

static int sgm41510_get_mivr_state(struct charger_device *chg_dev, bool *in_loop)
{
	u8 mivr_en = 0;
	int ret = 0;

	ret = sgm41510_read_interface((unsigned char) (sgm41510_CON19),
				     (&mivr_en),
				     (unsigned char) (CON19_VINDPM_STAT_MASK),
				     (unsigned char) (CON19_VINDPM_STAT_SHIFT)
				    );
	*in_loop = (mivr_en ? true : false);
	return ret;
}

static int sgm41510_enable_charging(struct charger_device *chg_dev,
				   bool en)
{
	int status = 0;
	struct sgm41510 *info = dev_get_drvdata(&chg_dev->dev);

	pr_info("enable state : %d\n", en);
	if (en) {
		sgm41510_set_en_hiz(0x0);
		sgm41510_set_chg_config(en);
		sgm41510_set_watch_dog_timer(info, 160);
		pr_info("[sgm41510_enable_charging]: enable charging\n");
	} else {
		sgm41510_set_chg_config(en);
		sgm41510_set_watch_dog_timer(info, 0);
		pr_info("[sgm41510_enable_charging]: disable charging\n");
	}

	return status;
}

static int sgm41510_is_charging_enabled(struct charger_device *chg_dev,
				   bool *en)
{
	int status = 0;
	unsigned char val;
	status = sgm41510_get_chg_config(&val);
	if (val)
		*en = true;
	else
		*en = false;
	pr_info("sgm41510_is_charging_enabled :get enable state : %d\n", *en);

	return status;
}

static int sgm41510_get_current(struct charger_device *chg_dev,
				     u32 *current_value)
{
	unsigned int status = true;
	unsigned char register_value;

	status = sgm41510_get_ichg(&register_value);
	*current_value = sgm41510_closest_value(CON4_ICHG_MIN, CON4_ICHG_MAX,
				     CON4_ICHG_STEP, register_value);
	pr_info("&&&& %s register_value = %d current_value = %d\n", __func__, register_value, *current_value );

	return status;
}

static int sgm41510_set_current(struct charger_device *chg_dev,
			       u32 current_value)
{
	unsigned int status = true;
	unsigned int register_value;

	pr_info("&&&& charge_current_value = %d\n", current_value);

	register_value = sgm41510_closest_reg(CON4_ICHG_MIN, CON4_ICHG_MAX,
			CON4_ICHG_STEP, current_value);

	pr_info("&&&& %s register_value = %d\n", __func__, register_value);
	sgm41510_set_ichg(register_value);

	return status;
}

static int sgm41510_get_min_ichg(struct charger_device *chg_dev, u32 *uA)
{
	*uA = sgm41510_closest_value(CON4_ICHG_MIN, CON4_ICHG_MAX,
				   CON4_ICHG_STEP, 0);
	return 0;
}

static int sgm41510_set_input_current(struct charger_device *chg_dev,
				     u32 current_value)
{
	unsigned int status = true;
	unsigned int register_value;

	pr_info("&&&& current_value = %d\n", current_value);

	register_value = sgm41510_closest_reg(CON0_IINLIM_MIN, CON0_IINLIM_MAX,
				    CON0_IINLIM_STEP, current_value);
	pr_info("&&&& %s register_value = %d\n", __func__,
		register_value);
	sgm41510_set_iinlim(register_value);

	return status;
}

static int sgm41510_get_input_current(struct charger_device *chg_dev,
				     u32 *current_value)
{
	unsigned int status = true;
	unsigned char register_value;

	status = sgm41510_get_iinlim(&register_value);
	*current_value = sgm41510_closest_value(CON0_IINLIM_MIN, CON0_IINLIM_MAX,
				     CON0_IINLIM_STEP, register_value);
	pr_info("&&&& %s register_value = %d current_value = %d\n", __func__, register_value, *current_value );

	return status;
}

static int sgm41510_get_min_input_current(struct charger_device *chg_dev, u32 *uA)
{
	*uA = sgm41510_closest_value(CON0_IINLIM_MIN, CON0_IINLIM_MAX,
				   CON0_IINLIM_STEP, 0);
	return 0;
}

static int sgm41510_get_ieoc(struct charger_device *chg_dev, u32 *ieoc)
{
	int ret = 0;
	unsigned char regval = 0;

	ret = sgm41510_get_iterm(&regval);
	*ieoc = sgm41510_closest_value(CON5_ITERM_MIN, CON5_ITERM_MAX,
				     CON5_ITERM_STEP, regval);

	pr_info("&&&& %s regval = %d ieoc = %d\n", __func__, regval, *ieoc );
	return ret;
}

static int sgm41510_set_ieoc(struct charger_device *chg_dev, u32 ieoc)
{
	unsigned int status = true;
	unsigned int register_value;

	pr_info("&&&& ieoc = %d\n", ieoc);

	register_value = sgm41510_closest_reg(CON5_ITERM_MIN, CON5_ITERM_MAX,
				    CON5_ITERM_STEP, ieoc);
	pr_info("&&&& %s register_value = %d\n", __func__, register_value);
	sgm41510_set_iterm(register_value);

	return status;
}

static int sgm41510_set_cv_voltage(struct charger_device *chg_dev,
				  u32 cv)
{
	unsigned int status = true;
	unsigned short register_value;

	register_value = sgm41510_closest_reg(CON6_CV_MIN, CON6_CV_MAX,
				    CON6_CV_STEP, cv);

	sgm41510_set_vreg(register_value);
	pr_info("&&&& cv = %d reg value = %d\n", cv, register_value);

	return status;
}

static int sgm41510_get_cv_voltage(struct charger_device *chg_dev,
				  u32 *cv)
{
	unsigned int status = true;
	unsigned char register_value;

	status = sgm41510_get_vreg(&register_value);
	*cv = sgm41510_closest_value(CON6_CV_MIN, CON6_CV_MAX,
				    CON6_CV_STEP, register_value);

	pr_info("&&&& %s cv = %d reg value = %d\n", __func__, *cv, register_value);

	return status;
}

static inline u8 sgm41510_closest_reg_via_tbl(const u32 *tbl, u32 tbl_size,
					    u32 target)
{
	u32 i = 0;

	if (target < tbl[0])
		return 0;

	for (i = 0; i < tbl_size - 1; i++) {
		if (target >= tbl[i] && target < tbl[i + 1])
			return i;
	}

	return tbl_size - 1;
}

static int sgm41510_set_watch_dog_timer(struct sgm41510 *sgm, u32 sec)
{
	u8 regval = 0;

	pr_info("charging_reset_watch_dog_timer\n");
	/* 40s is the minimum, set to 40 except sec == 0 */
	if (sec <= 40 && sec > 0)
		sec = 40;

	regval = sgm41510_closest_reg_via_tbl(sgm41510_wdt, ARRAY_SIZE(sgm41510_wdt),
					    sec);
	//sgm41510_set_wdt_rst(0x1);	/* Kick watchdog */
	sgm41510_set_watchdog(regval);

	return true;
}

static int sgm41510_kick_wdt(struct charger_device *chg_dev)
{
	pr_info("%s \n", __func__);
	sgm41510_set_wdt_rst(0x1);	/* Kick watchdog */
	return true;
}

static int sgm41510_get_charging_status(struct charger_device *chg_dev,
				       bool *is_done)
{
	unsigned int status = true;
	unsigned int ret_val;

	ret_val = sgm41510_get_chrg_stat();

	if (ret_val == 0x3)
		*is_done = true;
	else
		*is_done = false;

	return status;
}

static int sgm41510_enable_otg(struct charger_device *chg_dev, bool en)
{
	int ret = 0;
	struct sgm41510 *sgm = dev_get_drvdata(&chg_dev->dev);

	pr_info("%s en = %d\n", __func__, en);
	if (en) {
		sgm41510_set_chg_config(0);
		sgm41510_set_otg_config(1);
		sgm41510_set_watch_dog_timer(sgm, 160);
		gpio_set_value(sgm->sgm41510_otgen_gpio, 1);
	} else {
		sgm41510_set_otg_config(0);
		sgm41510_set_chg_config(1);
		sgm41510_set_watch_dog_timer(sgm, 0);
		gpio_set_value(sgm->sgm41510_otgen_gpio, 0);
	}
	if (qc_module_z350)
		z350_enable_otg(en);
	else
		wt6670f_enable_otg(en);
	return ret;
}

static int sgm41510_set_boost_current_limit(struct charger_device
		*chg_dev, u32 uA)
{
	int ret = 0;
	u8 boost_reg = 0;

	boost_reg = sgm41510_closest_reg(CON10_BOOST_LIM_MIN, CON10_BOOST_LIM_MAX,
				    CON10_BOOST_LIM_STEP, uA);
	sgm41510_set_boost_lim(boost_reg);

	return ret;
}

static int sgm41510_enable_safetytimer(struct charger_device *chg_dev,
				      bool en)
{
	int status = 0;

	if (en)
		sgm41510_set_en_timer(0x1);
	else
		sgm41510_set_en_timer(0x0);
	return status;
}

static int sgm41510_get_is_safetytimer_enable(struct charger_device
		*chg_dev, bool *en)
{
	unsigned char val = 0;

	sgm41510_read_interface(sgm41510_CON7, &val, CON7_EN_TIMER_MASK,
			       CON7_EN_TIMER_SHIFT);
	*en = (bool)val;
	return val;
}

static unsigned int charging_hw_init(void)
{
	unsigned int status = 0;

	sgm41510_set_en_hiz(0x0);
	sgm41510_set_ilim_pin(0x0);
	sgm41510_set_auto_dpdm(0x0);
	//sgm41510_set_vindpm(0x6);	/* VIN DPM check 4.6V */
	sgm41510_set_chg_timer(0x3);/* chg timer 20h */
	sgm41510_set_wdt_rst(0x1);	/* Kick watchdog */
	sgm41510_set_sys_min(0x5);	/* Minimum system voltage 3.5V */
	sgm41510_set_iprechg(0x8);	/* Precharge current 540mA */
	sgm41510_set_iterm(0x2);	/* Termination current 180mA */
	sgm41510_set_batlowv(0x1);	/* BATLOWV 3.0V */
	sgm41510_set_vrechg(0x0);	/* VRECHG 0.1V (4.108V) */
	sgm41510_set_en_term(0x1);	/* Enable termination */
	sgm41510_set_watchdog(0x0);	/* WDT s */
	sgm41510_set_en_timer(0x0);	/* Enable charge timer */
	sgm41510_set_batfet_rst_en(0x0); /* BATFET Full System Reset Enable */
	pr_info("%s: hw_init down!\n", __func__);
	return status;
}

static int sgm41510_parse_dt(struct sgm41510 *info,
			    struct device *dev)
{
	struct device_node *np = dev->of_node;

	pr_info("%s\n", __func__);
	if (!np) {
		pr_info("%s: no of node\n", __func__);
		return -ENODEV;
	}

	info->sgm41510_psel_gpio = of_get_named_gpio(np,
					"mtk,sgm41510_psel_gpio", 0);
	if ((!gpio_is_valid(info->sgm41510_psel_gpio))) {
		return -EINVAL;
	}

	info->sgm41510_otgen_gpio = of_get_named_gpio(np,
					"mtk,sgm41510_otgen_gpio", 0);
	if ((!gpio_is_valid(info->sgm41510_otgen_gpio))) {
		return -EINVAL;
	}

	if (of_property_read_string(np, "charger_name",
				    &info->chg_dev_name) < 0) {
		info->chg_dev_name = "primary_chg";
		pr_info("%s: no charger name\n", __func__);
	}

	if (of_property_read_string(np, "alias_name",
				    &(info->chg_props.alias_name)) < 0) {
		info->chg_props.alias_name = "sgm41510";
		pr_info("%s: no alias name\n", __func__);
	}
	return 0;
}

static int sgm41510_do_event(struct charger_device *chg_dev, u32 event,
			    u32 args)
{
	if (chg_dev == NULL)
		return -EINVAL;

	pr_info("%s: event = %d\n", __func__, event);
	switch (event) {
	case EVENT_EOC:
		charger_dev_notify(chg_dev, CHARGER_DEV_NOTIFY_EOC);
		break;
	case EVENT_RECHARGE:
		charger_dev_notify(chg_dev, CHARGER_DEV_NOTIFY_RECHG);
		break;
	default:
		break;
	}

	return 0;
}

static int sgm41510_plug_in(struct charger_device *chg_dev)
{
	int ret = 0;
	struct sgm41510 *info = dev_get_drvdata(&chg_dev->dev);

	dev_info(info->dev, "%s\n", __func__);

	/* Enable charging */
	if (strcmp(info->chg_dev_name, "primary_chg") == 0) {
		ret = sgm41510_enable_charging(chg_dev, true);
		if (ret < 0)
			dev_notice(info->dev, "%s: en chg fail\n", __func__);
	}

	return ret;
}

static int sgm41510_inform_psy_changed(struct sgm41510 *info)
{
	int ret = 0;
	union power_supply_propval propval;

	dev_info(info->dev, "%s: pwr_rdy = %d, type = %d\n", __func__,
		info->pwr_rdy, info->chg_type);

	/* Get chg type det power supply */
	info->psy = power_supply_get_by_name("charger");
	if (!info->psy) {
		dev_notice(info->dev, "%s: get power supply fail\n", __func__);
		return -EINVAL;
	}

	/* inform chg det power supply */
	propval.intval = info->pwr_rdy;
	ret = power_supply_set_property(info->psy, POWER_SUPPLY_PROP_ONLINE,
		&propval);
	if (ret < 0)
		dev_notice(info->dev, "%s: psy online fail(%d)\n", __func__,
			ret);

	propval.intval = info->chg_type;
	ret = power_supply_set_property(info->psy,
		POWER_SUPPLY_PROP_CHARGE_TYPE, &propval);
	if (ret < 0)
		dev_notice(info->dev, "%s: psy type fail(%d)\n", __func__, ret);

	return ret;
}
#ifndef CONFIG_COMPILE_FACTORY_VERSION
static void sgm41510_init_work_handler(struct work_struct *work)
{
	int ret = 0;
	union power_supply_propval propval;
	struct sgm41510 *info = container_of(work, struct sgm41510,
						charge_status_detect.work);
	bool pwr_rdy = false;

	pr_err("sgm41510_init_work_handler\n");
	if (sgm41510_get_pg_stat()) {
		if(info->chg_type != CHARGER_UNKNOWN) {
			pr_err("chg_type is not CHARGER_UNKNOWN!\n");
			return;
		}
		info->pwr_rdy = 1;
		info->chg_type = STANDARD_CHARGER;
		info->oplus_chg_type = POWER_SUPPLY_TYPE_USB_DCP;
		g_oplus_chip->first_det_flg = 1;
	} else {
		info->pwr_rdy = 0;
		info->chg_type = CHARGER_UNKNOWN;
		info->oplus_chg_type = POWER_SUPPLY_TYPE_UNKNOWN;
		g_oplus_chip->first_det_flg = 0;
		charging_poweron = false;
	}
	dev_info(info->dev, "%s: pwr_rdy = %d, type = %d\n", __func__,
		info->pwr_rdy, info->chg_type);
	sgm41510_inform_psy_changed(info);
}
static irqreturn_t sgm41510_interrupt(int irq, void *dev_id)
{
	struct sgm41510 *chip = dev_id;
	pr_err("INT OCCURED\n");

	mutex_lock(&chip->irq_complete);

	chip->irq_waiting = true;
	if (chip->chg_det_enable && (get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT))
	{
		disable_irq_nosync(irq);
		mutex_unlock(&chip->irq_complete);
		return IRQ_HANDLED;
	}
	/*if (!chip->resume_completed) {
		pr_err("IRQ triggered before device-resume\n");
		disable_irq_nosync(irq);
		mutex_unlock(&chip->irq_complete);
		return IRQ_HANDLED;
	}*/
	schedule_delayed_work(&chip->charge_status_detect, 0);
	chip->irq_waiting = false;
	chip->hvdcp_can_enabled = false;
	mutex_unlock(&chip->irq_complete);
	return IRQ_HANDLED;
}
#endif
static int sgm41510_plug_out(struct charger_device *chg_dev)
{
	int ret = 0;
	struct sgm41510 *info = dev_get_drvdata(&chg_dev->dev);

	dev_info(info->dev, "%s\n", __func__);

	/* Disable charging */
	ret = sgm41510_enable_charging(chg_dev, false);
	if (ret < 0) {
		dev_notice(info->dev, "%s: disable chg fail\n", __func__);
		return ret;
	}

	return ret;
}
bool oplus_sgm41510_get_pd_type(void);
static inline int __sgm41510_enable_chgdet_flow(struct sgm41510 *info, bool en);
static int __sgm41510_chgdet_handler(struct sgm41510 *info)
{
	int ret = 0;
	bool pwr_rdy = false, inform_psy = true, sdp_flag = false;
	u8 usb_status = 0;

	dev_info(info->dev, "%s\n", __func__);

	/* disabled by user, do nothing */
	//if (!info->desc->en_chgdet) {
	//	dev_info(info->dev, "%s: bc12 is disabled by dts\n", __func__);
	//	return 0;
	//}

#ifdef CONFIG_TCPC_CLASS
	pwr_rdy = atomic_read(&info->tcpc_usb_connected);
#else
	/* check power ready */
	/*ret = sgm41510_i2c_test_bit(info, sgm41510_REG_CHG_STATC,
		sgm41510_SHIFT_PWR_RDY, &pwr_rdy);
	if (ret < 0) {
		dev_notice(info->dev, "%s: read pwr rdy state fail\n",
			__func__);
		return ret;
	}*/
#endif

	/* no change in pwr_rdy state */
	/*if (info->pwr_rdy == pwr_rdy &&
		atomic_read(&info->bc12_wkard) == 0) {
		dev_info(info->dev, "%s: pwr_rdy(%d) state is the same\n",
			__func__, pwr_rdy);
		inform_psy = false;
		goto out;
	}*/
	info->pwr_rdy = pwr_rdy;
	
	/* plug out */
	
	if (!pwr_rdy) {
		info->chg_type = CHARGER_UNKNOWN;
		info->oplus_chg_type = POWER_SUPPLY_TYPE_UNKNOWN;
		atomic_set(&info->bc12_sdp_cnt, 0);
		goto out;
	}

	/* plug in */
	if (qc_module_z350) {
		ret = z350_get_charger_type(&usb_status);
		if (ret < 0) {
			dev_notice(info->dev, "%s: read type fail\n", __func__);
			return ret;
		}
	} else {
		ret = wt6670f_get_charger_type(&usb_status);
		if (ret < 0) {
			dev_notice(info->dev, "%s: read type fail\n", __func__);
			return ret;
		}
	}
	switch (usb_status) {
	case SGM41510_CHG_TYPE_NOVBUS:
		dev_info(info->dev, "%s: under going...\n", __func__);
		info->chg_type = CHARGER_UNKNOWN;
		info->oplus_chg_type = POWER_SUPPLY_TYPE_UNKNOWN;
		break;
	case SGM41510_CHG_TYPE_SDP:
		info->chg_type = STANDARD_HOST;
		info->oplus_chg_type = POWER_SUPPLY_TYPE_USB;
		break;
	case SGM41510_CHG_TYPE_CDP:
		info->chg_type = CHARGING_HOST;
		info->oplus_chg_type = POWER_SUPPLY_TYPE_USB_CDP;
		break;
	case SGM41510_CHG_TYPE_DCP:
		info->chg_type = STANDARD_CHARGER;
		info->oplus_chg_type = POWER_SUPPLY_TYPE_USB_DCP;
		break;
	case SGM41510_CHG_TYPE_HVDCP:
		info->chg_type = STANDARD_CHARGER;
		info->oplus_chg_type = POWER_SUPPLY_TYPE_USB_DCP;
		if (oplus_sgm41510_get_pd_type()) {
			dev_notice(info->dev, "%s: is pd now\n", __func__);
			info->hvdcp_can_enabled = false;
		} else {
			dev_notice(info->dev, "%s: is not pd now\n", __func__);
			info->hvdcp_can_enabled = true;
		}
		break;
	case SGM41510_CHG_TYPE_FLOAT:
		info->chg_type = NONSTANDARD_CHARGER;
		info->oplus_chg_type = POWER_SUPPLY_TYPE_USB_DCP;
		break;
	default:
		info->chg_type = NONSTANDARD_CHARGER;
		info->oplus_chg_type = POWER_SUPPLY_TYPE_USB_DCP;
		break;
	}
out:
	if ((info->chg_type == STANDARD_HOST || info->chg_type == CHARGING_HOST) && (atomic_read(&info->bc12_sdp_cnt) < 2) && (!(charging_poweron))) {
	/*	if (atomic_read(&info->bc12_sdp_cnt) == 0) {
			sgm41510_inform_psy_changed(info);
		}*/

		if (qc_module_z350) {
			z350_set_apsd_rerun(1);
			dev_notice(info->dev, "%s: z350 apsd rerun\n",
						__func__);
		} else {
			wt6670f_set_apsd_rerun(1);
			dev_notice(info->dev, "%s: wt6670f apsd rerun\n",
						__func__);
		}

		atomic_inc(&info->bc12_sdp_cnt);
		atomic_set(&info->bc12_wkard, 1);
		return 0;
	}
	if (info->chg_type != STANDARD_CHARGER) {
		/* turn off USB charger detection */
		/*if (sdp_flag == true) {
			atomic_set(&info->bc12_wkard, 0);*/
			ret = __sgm41510_enable_chgdet_flow(info, false);
			if (ret < 0)
				dev_notice(info->dev, "%s: disable chrdet fail\n",
							__func__);
		//}
	}

	if ((info->oplus_chg_type == POWER_SUPPLY_TYPE_USB_DCP) && (g_oplus_chip->first_det_flg == 1))
		g_oplus_chip->first_det_flg = 2;
	else
		g_oplus_chip->first_det_flg = 0;

	if (inform_psy)
		sgm41510_inform_psy_changed(info);

	return 0;
}

static int sgm41510_chgdet_handler(struct sgm41510 *info)
{
	int ret = 0;

	mutex_lock(&info->bc12_access_lock);
	ret = __sgm41510_chgdet_handler(info);
	mutex_unlock(&info->bc12_access_lock);

	return ret;
}

int sgm41510_attachi_irq_handler(void)
{
	int ret = 0;

	dev_notice(g_sgm->dev, "%s\n", __func__);

	// check bc12_en state
	mutex_lock(&g_sgm->bc12_access_lock);
	if (!g_sgm->bc12_en) {
		dev_notice(g_sgm->dev, "%s: bc12 disabled, ignore irq\n",
			__func__);
		goto out;
	}
	ret = __sgm41510_chgdet_handler(g_sgm);
out:
	mutex_unlock(&g_sgm->bc12_access_lock);
	return ret;
}
bool sgm41510_charger_exist_detect(void)
{
	if(g_oplus_chip != NULL) {
		if (g_oplus_chip->charger_exist)
			return true;
		else
			return false;
	}
	return false;
}
static int sgm41510_set_usbsw_state(struct sgm41510 *info, int state)
{
	dev_info(info->dev, "%s: state = %d\n", __func__, state);

	if (state == SGM41510_USBSW_CHG)
		Charger_Detect_Init();
	else
		Charger_Detect_Release();

	return 0;
}

static inline int __sgm41510_enable_chgdet_flow(struct sgm41510 *info, bool en)
{
	int ret = 0;
	enum sgm41510_usbsw_state usbsw =
		en ? SGM41510_USBSW_CHG : SGM41510_USBSW_USB;

	dev_info(info->dev, "%s: en = %d\n", __func__, en);
	sgm41510_set_usbsw_state(info, usbsw);
	if (qc_module_z350)
		ret = z350_enable_chgdet_flow(en);
	else
		ret = wt6670f_enable_chgdet_flow(en);
	if (ret >= 0)
		info->bc12_en = en;

	return ret;
}

static int sgm41510_enable_chgdet_flow(struct sgm41510 *info, bool en)
{
	int ret = 0, i = 0;
	bool pwr_rdy = false;
	const int max_wait_cnt = 200;

	dev_info(info->dev, "%s: en = %d\n", __func__, en);


	if (en) {
		// Workaround for CDP port
		for (i = 0; i < max_wait_cnt; i++) {
			if (is_usb_rdy())
				break;
			dev_dbg(info->dev, "%s: CDP block\n", __func__);
			if (!sgm41510_get_pg_stat()) {
				dev_info(info->dev, "%s: plug out\n",
					__func__);
				return 0;
			}
			msleep(100);
		}
		if (i == max_wait_cnt)
			dev_notice(info->dev, "%s: CDP timeout\n", __func__);
		else
			dev_info(info->dev, "%s: CDP free\n", __func__);
	}

	mutex_lock(&info->bc12_access_lock);
	ret = __sgm41510_enable_chgdet_flow(info, en);
	mutex_unlock(&info->bc12_access_lock);

	return ret;
}

static int sgm41510_enable_chg_type_det(struct charger_device *chg_dev, bool en)
{
	int ret = 0;

#ifdef CONFIG_TCPC_CLASS
	struct sgm41510 *info = dev_get_drvdata(&chg_dev->dev);
	info->chg_det_enable = en;
	/*if (!info->desc->en_chgdet) {
		dev_info(info->dev, "%s: bc12 is disabled by dts\n", __func__);
		return 0;
	}*/

	dev_info(info->dev, "%s: en = %d\n", __func__, en);
	atomic_set(&info->tcpc_usb_connected, en);

	/* TypeC detach */
	if (!en) {
		charging_poweron = false;
		ret = sgm41510_chgdet_handler(info);
		return ret;
	}
	if (en) {
		/* plug in, make usb switch to sgm41510 */
		ret = sgm41510_enable_chgdet_flow(info, true);
		if (ret < 0)
			dev_notice(info->dev, "%s: en chgdet fail(%d)\n", __func__,
				ret);
		//msleep(2000);
	}
	//ret = sgm41510_chgdet_handler(info);

#endif /* CONFIG_TCPC_CLASS */

	return ret;
}

static int sgm41510_get_ibus(struct charger_device *chg_dev, u32 *ibus)
{
	*ibus = 1000000;

	return 0;
}
static struct charger_ops sgm41510_chg_ops = {
	/* cable plug in/out for primary charger */
	.plug_in = sgm41510_plug_in,
	.plug_out = sgm41510_plug_out,

	/* enable/disable charger */
	.enable = sgm41510_enable_charging,
	.is_enabled = sgm41510_is_charging_enabled,
	.is_charging_done = sgm41510_get_charging_status,

	/* get/set minimun input voltage regulation */
	.get_mivr = sgm41510_get_mivr,
	.set_mivr = sgm41510_set_mivr,
	.get_mivr_state = sgm41510_get_mivr_state,

	/* get/set input current */
	.get_input_current = sgm41510_get_input_current,
	.set_input_current = sgm41510_set_input_current,
	.get_min_input_current = sgm41510_get_min_input_current,

	/* get/set charging voltage */
	.get_constant_voltage = sgm41510_get_cv_voltage,
	.set_constant_voltage = sgm41510_set_cv_voltage,

	/* get/set charging current*/
	.get_charging_current = sgm41510_get_current,
	.set_charging_current = sgm41510_set_current,
	.get_min_charging_current = sgm41510_get_min_ichg,

	/* get/set termination current */
	.get_eoc_current = sgm41510_get_ieoc,
	.set_eoc_current = sgm41510_set_ieoc,
	//.reset_eoc_state = rt9471_reset_eoc_state,

	/* enable te */
	.enable_termination = sgm41510_enable_te,

	.dump_registers = sgm41510_dump_register,

	.kick_wdt = sgm41510_kick_wdt,

	/* enable/disable powerpath for primary charger */
	.enable_powerpath = NULL,//rt9471_enable_powerpath,
	.is_powerpath_enabled = NULL,//rt9471_is_powerpath_enabled,

	/* enable/disable chip for secondary charger */
	//.enable_chip = rt9471_enable_powerpath,
	//.is_chip_enabled = rt9471_is_powerpath_enabled,

	/* Safety timer */
	.enable_safety_timer = sgm41510_enable_safetytimer,
	.is_safety_timer_enabled = sgm41510_get_is_safetytimer_enable,

	/* Charger type detection */
	.enable_chg_type_det = sgm41510_enable_chg_type_det,

	/* OTG */
	.enable_otg = sgm41510_enable_otg,
	.set_boost_current_limit = sgm41510_set_boost_current_limit,
	.enable_discharge = NULL,

	/* ADC */
	.get_tchg_adc = NULL,
	.get_ibus_adc = sgm41510_get_ibus,
	.event = sgm41510_do_event,
};


void oplus_sgm41510_dump_registers(void)
{
	sgm41510_dump_register(g_sgm->chg_dev);
	if((g_oplus_chip != NULL) && (g_oplus_chip->is_double_charger_support)) {
		g_oplus_chip->sub_chg_ops->kick_wdt();
        g_oplus_chip->sub_chg_ops->dump_registers();
	}
}

int oplus_sgm41510_kick_wdt(void)
{
	return sgm41510_kick_wdt(g_sgm->chg_dev);
}

int oplus_sgm41510_set_ichg(int cur)
{
	struct oplus_chg_chip *chip = g_oplus_chip;
	u32 uA = cur*1000;
	u32 temp_uA;
	int ret = 0;

	//if(cur <= 1000){   // <= 1A
		ret = sgm41510_set_current(g_sgm->chg_dev, uA);
	/*	chip->sub_chg_ops->charging_current_write_fast(0);
	} else {
		temp_uA = uA  * PRIMARY_CHG_PERCENT / TOTAL_CHG_PERCENT;
		ret = __sgm41510_set_ichg(sgm41510, temp_uA);
		ret = __sgm41510_get_ichg(sgm41510, &temp_uA);
		uA-=temp_uA;
		chip->sub_chg_ops->charging_current_write_fast(uA/1000);
	}*/
	return ret;
}

void oplus_sgm41510_set_mivr(int vbatt)
{

	u32 uV = vbatt*1000 + 200000;
#ifdef OPLUS_FEATURE_CHG_BASIC
//Junbo.Guo@ODM_WT.BSP.CHG, 2019/11/11, Modify for pe20
    if(uV<4200000)
        uV = 4200000;
#endif
	sgm41510_set_mivr(g_sgm->chg_dev, uV);
}

void oplus_sgm41510_safe_charging_status_check()
{
	struct oplus_chg_chip *chip = g_oplus_chip;
	struct charger_manager *info = NULL;
	if(g_oplus_chip == NULL) {
		return;
	}
	if(g_sgm->chg_consumer != NULL)
		info = g_sgm->chg_consumer->cm;
	dev_info(g_sgm->dev, "%s:calling_on = %d, camera_on = %d,\n",__func__, chip->calling_on, chip->camera_on);
	if((chip->calling_on) || (chip->camera_on)) {
			if (qc_module_z350)
				z350_qc_mode_select(MODE_QC20_V5);
			else
				wt6670f_qc_mode_select(MODE_QC20_V5);
			g_sgm->pdqc_setup_5v = 1;
			g_sgm->calling_on = chip->calling_on;
			g_sgm->camera_on = chip->camera_on;
			g_sgm->hvdcp_can_enabled = false;
			dev_info(g_sgm->dev, "%s:calling is on, disable hvdcp\n", __func__);
	} else if(((g_sgm->calling_on) && (chip->calling_on == false)) || ((chip->camera_on ==false) && (g_sgm->camera_on))) {
		if (qc_module_z350) {
			z350_qc_mode_select(MODE_QC20_V5);
			z350_qc_mode_select(MODE_QC20_V9);
		}
		else {
			wt6670f_qc_mode_select(MODE_QC20_V5);
			wt6670f_qc_mode_select(MODE_QC20_V9);
		}
			g_sgm->pdqc_setup_5v = 0;
			g_sgm->hvdcp_can_enabled = true;
			dev_info(g_sgm->dev, "%s:calling is off, enable hvdcp\n", __func__);
		g_sgm->calling_on = chip->calling_on;
		g_sgm->camera_on = chip->camera_on;
	}
}

static int usb_icl[] = {
	300, 500, 900, 1200, 1500, 1750, 2000, 3000,
};
int oplus_sgm41510_set_aicr(int current_ma)
{
	struct oplus_chg_chip *chip = g_oplus_chip;
	int rc = 0, i = 0;
	int chg_vol = 0;
	int aicl_point = 0;
	int aicl_point_temp = 0;

	if (chip->first_det_flg == 1)
		current_ma = 500;

	if (g_sgm->pre_current_ma == current_ma)
		return rc;
	else
		g_sgm->pre_current_ma = current_ma;

	if(chip->is_double_charger_support)
		chip->sub_chg_ops->input_current_write(0);
		
	dev_info(g_sgm->dev, "%s usb input max current limit=%d\n", __func__,current_ma);
	aicl_point_temp = aicl_point = 4500;
	sgm41510_set_mivr(g_sgm->chg_dev, 4200000);
	
	if (current_ma < 500) {
		i = 0;
		goto aicl_end;
	}
	
	i = 1; /* 500 */
	sgm41510_set_input_current(g_sgm->chg_dev, usb_icl[i] * 1000);
	msleep(90);
	
	chg_vol = battery_meter_get_charger_voltage();
	if (chg_vol < aicl_point_temp) {
		chg_debug( "use 500 here\n");
		goto aicl_end;
	} else if (current_ma < 900)
		goto aicl_end;

	i = 2; /* 900 */
	sgm41510_set_input_current(g_sgm->chg_dev, usb_icl[i] * 1000);
	msleep(90);
	chg_vol = battery_meter_get_charger_voltage();
	if (chg_vol < aicl_point_temp) {
		i = i - 1;
		goto aicl_pre_step;
	} else if (current_ma < 1200)
		goto aicl_end;

	i = 3; /* 1200 */
	sgm41510_set_input_current(g_sgm->chg_dev, usb_icl[i] * 1000);
	msleep(90);
	chg_vol = battery_meter_get_charger_voltage();
	if (chg_vol < aicl_point_temp) {
		i = i - 1;
		goto aicl_pre_step;
	}

	i = 4; /* 1500 */
	aicl_point_temp = aicl_point + 50;
	sgm41510_set_input_current(g_sgm->chg_dev, usb_icl[i] * 1000);
	msleep(120);
	chg_vol = battery_meter_get_charger_voltage();
	if (chg_vol < aicl_point_temp) {
		i = i - 2; //We DO NOT use 1.2A here
		goto aicl_pre_step;
	} else if (current_ma < 1500) {
		i = i - 1; //We use 1.2A here
		goto aicl_end;
	} else if (current_ma < 2000)
		goto aicl_end;

	i = 5; /* 1750 */
	aicl_point_temp = aicl_point + 50;
	sgm41510_set_input_current(g_sgm->chg_dev, usb_icl[i] * 1000);
	msleep(120);
	chg_vol = battery_meter_get_charger_voltage();
	if (chg_vol < aicl_point_temp) {
		i = i - 2; //1.2
		goto aicl_pre_step;
	}

	i = 6; /* 2000 */
	aicl_point_temp = aicl_point;
	sgm41510_set_input_current(g_sgm->chg_dev, usb_icl[i] * 1000);
	msleep(90);
	chg_vol = battery_meter_get_charger_voltage();
	if (chg_vol < aicl_point_temp) {
		i =  i - 2;//1.5
		goto aicl_pre_step;
	} else if (current_ma < 3000)
		goto aicl_end;

	i = 7; /* 3000 */
	sgm41510_set_input_current(g_sgm->chg_dev, usb_icl[i] * 1000);
	msleep(90);
	chg_vol = battery_meter_get_charger_voltage();
	if (chg_vol < aicl_point_temp) {
		i = i - 1;
		goto aicl_pre_step;
	} else if (current_ma >= 3000)
		goto aicl_end;

aicl_pre_step:
	if(chip->is_double_charger_support){
		if(usb_icl[i]>1000){
			sgm41510_set_input_current(g_sgm->chg_dev, usb_icl[i] * 1000*70/100);
			chip->sub_chg_ops->input_current_write(usb_icl[i]*30/100);
		}else{
			sgm41510_set_input_current(g_sgm->chg_dev, usb_icl[i] * 1000);
		}
	}else{
		sgm41510_set_input_current(g_sgm->chg_dev, usb_icl[i] * 1000);
	}
	dev_info(g_sgm->dev, "%s:usb input max current limit aicl chg_vol=%d j[%d]=%d sw_aicl_point:%d aicl_pre_step\n",__func__, chg_vol, i, usb_icl[i], aicl_point_temp);
	return rc;
aicl_end:
	if(chip->is_double_charger_support){
		if(usb_icl[i]>1000){
			sgm41510_set_input_current(g_sgm->chg_dev, usb_icl[i] * 1000*70/100);
			chip->sub_chg_ops->input_current_write(usb_icl[i]*30/100);
		}else{
			sgm41510_set_input_current(g_sgm->chg_dev, usb_icl[i] * 1000);
		}
	}else{
		sgm41510_set_input_current(g_sgm->chg_dev, usb_icl[i] * 1000);
	}
	dev_info(g_sgm->dev, "%s:usb input max current limit aicl chg_vol=%d j[%d]=%d sw_aicl_point:%d aicl_end\n",__func__, chg_vol, i, usb_icl[i], aicl_point_temp);
	return rc;
}

int oplus_sgm41510_set_cv(int cur)
{
	u32 uV = cur*1000;
	return sgm41510_set_cv_voltage(g_sgm->chg_dev, uV);
}

int oplus_sgm41510_set_ieoc(int cur)
{
	u32 uA = cur*1000;
	return sgm41510_set_ieoc(g_sgm->chg_dev, uA);
}

int oplus_sgm41510_charging_enable(void)
{
	return sgm41510_enable_charging(g_sgm->chg_dev, true);
}

int oplus_sgm41510_charging_disable(void)
{
#ifdef OPLUS_FEATURE_CHG_BASIC
//Junbo.Guo@ODM_WT.BSP.CHG, 2019/11/11, Modify for PD
	struct charger_manager *info = NULL;
	
	if(g_sgm->chg_consumer != NULL)
		info = g_sgm->chg_consumer->cm;

	if(!info){
		dev_info(g_sgm->dev, "%s:error\n", __func__);
		return false;
	}
	
	mtk_pdc_plugout(info);
	if(g_sgm->hvdcp_can_enabled){
		//sgm41510_i2c_update_bits(sgm41510, sgm41510_REG_CHG_DPDM2, 0x80, 0xE0);
		dev_info(g_sgm->dev, "%s: set qc to 5V", __func__);
	}

	sgm41510_set_watchdog(0x00);
#endif
	g_sgm->pre_current_ma = -1;
	return sgm41510_enable_charging(g_sgm->chg_dev, false);
}

int oplus_sgm41510_hardware_init(void)
{
	int ret = 0;

	dev_info(g_sgm->dev, "%s\n", __func__);

	/* Enable WDT */
	//if (sgm41510->desc->en_wdt) {
		ret = sgm41510_set_watch_dog_timer(g_sgm, 160);
		if (ret < 0)
			dev_notice(g_sgm->dev, "%s: en wdt fail\n", __func__);
	//}

	/* Enable charging */
	if (strcmp(g_sgm->chg_dev_name, "primary_chg") == 0) {
		ret = sgm41510_enable_charging(g_sgm->chg_dev, true);
		if (ret < 0)
			dev_notice(g_sgm->dev, "%s: en chg fail\n", __func__);
	}

	return ret;
}

int oplus_sgm41510_is_charging_enabled(void)
{
	bool en;
	sgm41510_is_charging_enabled(g_sgm->chg_dev, &en);
	return en;
}

int oplus_sgm41510_is_charging_done(void)
{
	bool ret = 0;
	//enum sgm41510_charging_status chg_stat = sgm41510_CHG_STATUS_READY;

	sgm41510_get_charging_status(g_sgm->chg_dev, &ret);

	return (int)ret;

}

int oplus_sgm41510_enable_otg(void)
{
	int ret = 0;
	ret = sgm41510_enable_otg(g_sgm->chg_dev, true);

	if (ret < 0) {
		dev_notice(g_sgm->dev, "%s en otg fail(%d)\n", __func__, ret);
		return ret;
	}
	return ret;
}

int oplus_sgm41510_disable_otg(void)
{
	int ret = 0;
	ret = sgm41510_enable_otg(g_sgm->chg_dev, false);

	if (ret < 0) {
		dev_notice(g_sgm->dev, "%s disable otg fail(%d)\n", __func__, ret);
		return ret;
	}

	return ret;

}

int oplus_sgm41510_disable_te(void)
{
	return sgm41510_enable_te(g_sgm->chg_dev, false);
}

int oplus_sgm41510_get_chg_current_step(void)
{
	return CON4_ICHG_STEP;
}

int oplus_sgm41510_get_charger_type(void)
{
	return g_sgm->oplus_chg_type;
}

int oplus_sgm41510_charger_suspend(void)
{
	if((g_oplus_chip != NULL) && (g_oplus_chip->is_double_charger_support)) {
		g_oplus_chip->sub_chg_ops->charger_suspend();
	}

	sgm41510_enable_charging(g_sgm->chg_dev, false);
	return 0;
}

int oplus_sgm41510_charger_unsuspend(void)
{
	if(g_oplus_chip->mmi_chg == 0)
		return 0;
	if((g_oplus_chip != NULL) && (g_oplus_chip->is_double_charger_support)) {
		g_oplus_chip->sub_chg_ops->charger_unsuspend();
	}

	sgm41510_enable_charging(g_sgm->chg_dev, true);
	return 0;
}

int oplus_sgm41510_set_rechg_vol(int vol)
{
	return 0;
}

int oplus_sgm41510_reset_charger(void)
{
	return 0;
}

int oplus_sgm41510_set_hz_mode(bool en)
{
	sgm41510_set_en_hiz(en);
	return 0;
}

bool oplus_sgm41510_check_charger_resume(void)
{
	return true;
}
bool oplus_sgm41510_check_hvdcp_charging_on(void)
{
	if(g_sgm->hvdcp_can_enabled)
		return true;
	else
		return false;
}
void oplus_sgm41510_set_chargerid_switch_val(int value)
{
	return;
}

int oplus_sgm41510_get_chargerid_switch_val(void)
{
	return 0;
}

int oplus_sgm41510_get_charger_subtype(void)
{
	struct charger_manager *info = NULL;
		
	if(g_sgm->chg_consumer != NULL)
		info = g_sgm->chg_consumer->cm;

	if(!info){
		dev_info(g_sgm->dev, "%s:error\n", __func__);
		return false;
	}
	
	if(mtk_pdc_check_charger(info)&&(!disable_PD)){
		return CHARGER_SUBTYPE_PD;
	}else if(g_sgm->hvdcp_can_enabled){
		return CHARGER_SUBTYPE_QC;
	}else if(mtk_pe20_get_is_connect(info)){
		return CHARGER_SUBTYPE_PE20;
	}else{
		return CHARGER_SUBTYPE_DEFAULT;
	}
}

bool oplus_sgm41510_need_to_check_ibatt(void)
{
	return false;
}

int oplus_sgm41510_get_dyna_aicl_result(void)
{
	u32 uA;
	int mA = 0;

	sgm41510_get_input_current(g_sgm->chg_dev, &uA);
	mA = (int)uA/1000;
	return mA;
}

int oplus_sgm41510_set_qc_config(void)
{
#ifdef OPLUS_FEATURE_CHG_BASIC
	//Junbo.Guo@ODM_WT.BSP.CHG, 2019/11/11, Modify for qc
	struct oplus_chg_chip *chip = g_oplus_chip;
	struct charger_manager *info = NULL;
		
	if(g_sgm->chg_consumer != NULL)
		info = g_sgm->chg_consumer->cm;

	if(!info){
		dev_info(g_sgm->dev, "%s:error\n", __func__);
		return false;
	}
	
	if (!chip) {
		dev_info(g_sgm->dev, "%s: error\n", __func__);
		return false;
	}

	if(disable_QC){
		dev_info(g_sgm->dev, "%s:disable_QC\n", __func__);
		return false;
	}

	if(g_sgm->disable_hight_vbus==1){
		dev_info(g_sgm->dev, "%s:disable_hight_vbus\n", __func__);
		return false;
	}

	if(chip->charging_state == CHARGING_STATUS_FAIL) {
		dev_info(g_sgm->dev, "%s:charging_status_fail\n", __func__);
		return false;
	}

	if (chip->temperature >= 430 && chip->charger_volt > 7500) {
		if (qc_module_z350)
			z350_qc_mode_select(MODE_QC20_V5);
		else
			wt6670f_qc_mode_select(MODE_QC20_V5);
		dev_info(g_sgm->dev, "%s: qc set to 5V, batt temperature hot or cold\n", __func__);
		return false;
	}

	if (chip->limits.vbatt_pdqc_to_5v_thr > 0 && chip->charger_volt > 7500
		&& chip->batt_volt > chip->limits.vbatt_pdqc_to_5v_thr&&chip->ui_soc>=90&&chip->icharging > -1000) {
		if (qc_module_z350)
			z350_qc_mode_select(MODE_QC20_V5);
		else
			wt6670f_qc_mode_select(MODE_QC20_V5);
		g_sgm->pdqc_setup_5v = 1;
		g_sgm->hvdcp_can_enabled = false;
		dev_info(g_sgm->dev, "%s: set qc to 5V", __func__);
	} else { // 9v
		dev_info(g_sgm->dev, "%s: soc high,or qc is 5V, enter 9v", __func__);
		if ((chip->ui_soc >= 90 && chip->batt_volt > chip->limits.vbatt_pdqc_to_5v_thr) || chip->charger_volt > 7500 || chip->temperature >= 430) {
			dev_info(g_sgm->dev, "%s: soc high,or qc is 9V return", __func__);
			return false;
		}
		if (qc_module_z350) {
			z350_qc_mode_select(MODE_QC20_V5);
			z350_qc_mode_select(MODE_QC20_V9);
		}
		else {
			wt6670f_qc_mode_select(MODE_QC20_V5);
			wt6670f_qc_mode_select(MODE_QC20_V9);
		}
		g_sgm->pdqc_setup_5v = 0;
		g_sgm->hvdcp_can_enabled = true;
		dev_info(g_sgm->dev, "%s:qc Force output 9V\n", __func__);
	}

	return true;
#endif
}

int oplus_sgm41510_enable_qc_detect(void)
{
	return 0;
}

bool oplus_sgm41510_get_shortc_hw_gpio_status(void)
{
	return false;
}

#ifdef OPLUS_FEATURE_CHG_BASIC
//Junbo.Guo@ODM_WT.BSP.CHG, 2019/11/11, Modify for PD
bool oplus_sgm41510_get_pd_type(void)
{
	struct charger_manager *info = NULL;

	if(g_sgm->chg_consumer != NULL)
		info = g_sgm->chg_consumer->cm;

	if(!info){
		dev_info(g_sgm->dev, "%s:error\n", __func__);
		return false;
	}

	if(disable_PD){
		dev_info(g_sgm->dev, "%s:disable_PD\n", __func__);
		return false;
	}

	info->enable_pe_4 = false;
	if(mtk_pdc_check_charger(info))
	{
		return true;
	}

	return false;
}

#define VBUS_VOL_5V	5000
#define VBUS_VOL_9V	9000
#define IBUS_VOL_2A	2000
#define IBUS_VOL_3A	3000
int oplus_sgm41510_pd_setup (void)
{
	struct oplus_chg_chip *chip = g_oplus_chip;
	struct charger_manager *info = NULL;
	int vbus = 0, ibus = 0;
		
	if(g_sgm->chg_consumer != NULL)
		info = g_sgm->chg_consumer->cm;

	if(!info){
		dev_info(g_sgm->dev, "%s:error\n", __func__);
		return false;
	}
	
	if (!chip) {
		dev_info(g_sgm->dev, "%s: error\n", __func__);
		return false;
	}

	if(disable_PD){
		dev_info(g_sgm->dev, "%s:disable_PD\n", __func__);
		return false;
	}

	if(g_sgm->disable_hight_vbus==1){
		dev_info(g_sgm->dev, "%s:disable_hight_vbus\n", __func__);
		return false;
	}

	if(chip->charging_state == CHARGING_STATUS_FAIL) {
		dev_info(g_sgm->dev, "%s:charging_status_fail\n", __func__);
		return false;
	}

	if ((chip->charger_volt > 7500) &&
			((chip->temperature >= 430) || (chip->temperature < 0))) {
		vbus = VBUS_VOL_5V;
		ibus = IBUS_VOL_2A;
		oplus_pdc_setup(&vbus, &ibus);
		dev_info(g_sgm->dev, "%s: pd set 5V, batt temperature hot or cold\n", __func__);
		return false;
	}

	if (chip->limits.vbatt_pdqc_to_5v_thr > 0 && chip->charger_volt > 7500
		&& chip->batt_volt > chip->limits.vbatt_pdqc_to_5v_thr&&chip->ui_soc>=90&&chip->icharging > -1000) {
		dev_info(g_sgm->dev, "%s: pd set qc to 5V", __func__);
		vbus = VBUS_VOL_5V;
		ibus = IBUS_VOL_2A;
		oplus_pdc_setup(&vbus, &ibus);
		g_sgm->pdqc_setup_5v = 1;
	}else{
		if ((chip->ui_soc >= 90 && chip->batt_volt > chip->limits.vbatt_pdqc_to_5v_thr) || chip->charger_volt > 7500 || chip->temperature >= 430) {
			dev_info(g_sgm->dev, "%s: soc high,or pd is 9V return", __func__);
			return false;
		}
		dev_info(g_sgm->dev, "%s:pd Force output 9V\n",__func__);
		vbus = VBUS_VOL_9V;
		ibus = IBUS_VOL_2A;
		oplus_pdc_setup(&vbus, &ibus);
		g_sgm->pdqc_setup_5v = 0;
	}
	
	return true;
}
int oplus_sgm41510_chg_set_high_vbus(bool en)
{
	int subtype;
	struct oplus_chg_chip *chip = g_oplus_chip;
	struct charger_manager *info = NULL;
	int vbus = 0, ibus = 0;
		
	if(g_sgm->chg_consumer != NULL)
		info = g_sgm->chg_consumer->cm;

	if(!info){
		dev_info(g_sgm->dev, "%s:error\n", __func__);
		return false;
	}
	
	if (!chip) {
		dev_info(g_sgm->dev, "%s: error\n", __func__);
		return false;
	}

	if(en){
		g_sgm->disable_hight_vbus= 0;
		if(chip->charger_volt >7500){
			dev_info(g_sgm->dev, "%s:charger_volt already 9v\n", __func__);
			return false;
		}

		if(g_sgm->pdqc_setup_5v){
			dev_info(g_sgm->dev, "%s:pdqc already setup5v no need 9v\n", __func__);
			return false;
		}

	}else{
		g_sgm->disable_hight_vbus= 1;
		if(chip->charger_volt < 5400){
			dev_info(g_sgm->dev, "%s:charger_volt already 5v\n", __func__);
			return false;
		}
	}
	
	subtype=oplus_sgm41510_get_charger_subtype();
	if (subtype==CHARGER_SUBTYPE_PD) {
		if (en) {
			dev_info(g_sgm->dev, "%s:pd Force output 9V\n",__func__);
			vbus = VBUS_VOL_9V;
			ibus = IBUS_VOL_2A;
			oplus_pdc_setup(&vbus, &ibus);
		} else {
			vbus = VBUS_VOL_5V;
			ibus = IBUS_VOL_2A;
			oplus_pdc_setup(&vbus, &ibus);
			dev_info(g_sgm->dev, "%s: set pd to 5V", __func__);
		}
	} else if (subtype==CHARGER_SUBTYPE_QC) {
		if (en) {
			dev_info(g_sgm->dev, "%s:QC Force output 9V\n",__func__);
			//sgm41510_i2c_update_bits(g_sgm, sgm41510_REG_CHG_DPDM2,0x20, 0xE0);
		} else {
			//sgm41510_i2c_update_bits(g_sgm, sgm41510_REG_CHG_DPDM2, 0x80, 0xE0);
			dev_info(g_sgm->dev, "%s: set qc to 5V", __func__);
		}
	}

	return false;
}
#endif

static int oplus_sgm41510_get_rtc_spare_oplus_fg_value(void)
{
	return 0;
}

static int oplus_sgm41510_set_rtc_spare_oplus_fg_value(int soc)
{
	return 0;
}


extern int oplus_battery_meter_get_battery_voltage(void);
extern int oplus_get_rtc_ui_soc(void);
extern int oplus_set_rtc_ui_soc(int value);
extern int set_rtc_spare_fg_value(int val);
extern void mt_power_off(void);
extern bool pmic_chrdet_status(void);
extern void mt_usb_connect(void);
extern void mt_usb_disconnect(void);
struct oplus_chg_operations  oplus_chg_sgm41510_ops = {
	.dump_registers = oplus_sgm41510_dump_registers,
	.kick_wdt = oplus_sgm41510_kick_wdt,
	.hardware_init = oplus_sgm41510_hardware_init,
	.charging_current_write_fast = oplus_sgm41510_set_ichg,
	.set_aicl_point = oplus_sgm41510_set_mivr,
	.input_current_write = oplus_sgm41510_set_aicr,
	.float_voltage_write = oplus_sgm41510_set_cv,
	.term_current_set = oplus_sgm41510_set_ieoc,
	.charging_enable = oplus_sgm41510_charging_enable,
	.charging_disable = oplus_sgm41510_charging_disable,
	.get_charging_enable = oplus_sgm41510_is_charging_enabled,
	.charger_suspend = oplus_sgm41510_charger_suspend,
	.charger_unsuspend = oplus_sgm41510_charger_unsuspend,
	.set_rechg_vol = oplus_sgm41510_set_rechg_vol,
	.reset_charger = oplus_sgm41510_reset_charger,
	.read_full = oplus_sgm41510_is_charging_done,
	.otg_enable = oplus_sgm41510_enable_otg,
	.otg_disable = oplus_sgm41510_disable_otg,
	.set_charging_term_disable = oplus_sgm41510_disable_te,
	.check_charger_resume = oplus_sgm41510_check_charger_resume,

	.get_charger_type = oplus_sgm41510_get_charger_type,
	.get_charger_volt = battery_get_vbus,
//	int (*get_charger_current)(void);
	.get_chargerid_volt = NULL,
    .set_chargerid_switch_val = oplus_sgm41510_set_chargerid_switch_val,
    .get_chargerid_switch_val = oplus_sgm41510_get_chargerid_switch_val,
	.check_chrdet_status = (bool (*) (void)) pmic_chrdet_status,

	.get_boot_mode = (int (*)(void))get_boot_mode,
	.get_boot_reason = (int (*)(void))get_boot_reason,
	.get_instant_vbatt = oplus_battery_meter_get_battery_voltage,
	.get_rtc_soc = oplus_sgm41510_get_rtc_spare_oplus_fg_value,
	.set_rtc_soc = oplus_sgm41510_set_rtc_spare_oplus_fg_value,
	.set_power_off = mt_power_off,
	.usb_connect = mt_usb_connect,
	.usb_disconnect = mt_usb_disconnect,
    .get_chg_current_step = oplus_sgm41510_get_chg_current_step,
    .need_to_check_ibatt = oplus_sgm41510_need_to_check_ibatt,
    .get_dyna_aicl_result = oplus_sgm41510_get_dyna_aicl_result,
    .get_shortc_hw_gpio_status = oplus_sgm41510_get_shortc_hw_gpio_status,
//	void (*check_is_iindpm_mode) (void);
    .oplus_chg_get_pd_type = oplus_sgm41510_get_pd_type,
    .oplus_chg_pd_setup = oplus_sgm41510_pd_setup,
	.get_charger_subtype = oplus_sgm41510_get_charger_subtype,
	.set_qc_config = oplus_sgm41510_set_qc_config,
	.enable_qc_detect = oplus_sgm41510_enable_qc_detect,
	.oplus_chg_set_high_vbus = oplus_sgm41510_chg_set_high_vbus,
	//.enable_shipmode = sgm41510_enable_shipmode,
	.oplus_chg_set_hz_mode = oplus_sgm41510_set_hz_mode,
	.set_safe_charging_status = oplus_sgm41510_safe_charging_status_check,
};

static int sgm41510_driver_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int ret = 0;
	int rc;
	struct sgm41510 *info = NULL;

	pr_info("[%s]\n", __func__);

	info = devm_kzalloc(&client->dev, sizeof(struct sgm41510),
			    GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->chg_consumer =
		charger_manager_get_by_name(&client->dev, "sgm41510");

	i2c_set_clientdata(client, info);
	mutex_init(&info->bc12_access_lock);
#ifdef CONFIG_TCPC_CLASS
	atomic_set(&info->tcpc_usb_connected, 0);
#endif /* CONFIG_TCPC_CLASS */
	new_client = client;
	info->dev = &client->dev;
	info->bc12_en = true;
	atomic_set(&info->bc12_sdp_cnt, 0);
	atomic_set(&info->bc12_wkard, 0);

	g_sgm = info;
	ret = sgm41510_parse_dt(info, &client->dev);
	if (ret < 0)
		return ret;
	g_sgm = info;
	ret = devm_gpio_request(&client->dev,
				info->sgm41510_psel_gpio, "sgm41510 psel gpio");
	if (ret) {
		pr_info("request sgm41510 psel gpio failed, ret=%d\n",
				ret);
		//goto err_mutex_init;
	}
	gpio_direction_output(info->sgm41510_psel_gpio, 0);
	gpio_set_value(info->sgm41510_psel_gpio, 0);

	ret = devm_gpio_request(&client->dev,
				info->sgm41510_otgen_gpio, "sgm41510 otgen gpio");
	if (ret) {
		pr_info("request sgm41510 otgen gpio failed, ret=%d\n",
				ret);
		//goto err_mutex_init;
	}
	gpio_direction_output(info->sgm41510_otgen_gpio, 0);
	gpio_set_value(info->sgm41510_otgen_gpio, 0);
	mdelay(10);
	charging_hw_init();

	info->oplus_chg_type = POWER_SUPPLY_TYPE_UNKNOWN;
	info->pre_current_ma = -1;

	/* Register charger device */
	info->chg_dev = charger_device_register(info->chg_dev_name,
						&client->dev, info,
						&sgm41510_chg_ops,
						&info->chg_props);
	if (IS_ERR_OR_NULL(info->chg_dev)) {
		pr_info("%s: register charger device  failed\n", __func__);
		ret = PTR_ERR(info->chg_dev);
		mutex_destroy(&info->bc12_access_lock);
		return ret;
	}
	info->camera_on = false;
	info->calling_on = false;
	sgm41510_dump_register(info->chg_dev);
#ifndef CONFIG_COMPILE_FACTORY_VERSION
	INIT_DELAYED_WORK(&info->charge_status_detect, sgm41510_init_work_handler);

	if (client->irq) {
		rc = devm_request_threaded_irq(&client->dev, client->irq,
				NULL, sgm41510_interrupt,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				"sgm41510 irq", info);
		if (rc < 0) {
			pr_err("request irq for irq=%d failed, rc =%d\n",
							client->irq, rc);
			goto err_mutex_init;
		}
	}
	device_init_wakeup(info->dev, 1);
#endif
	set_charger_ic(SGM41510);

	return 0;
#ifndef CONFIG_COMPILE_FACTORY_VERSION
err_mutex_init:
	mutex_destroy(&info->irq_complete);
	return rc;
#endif
}

/**********************************************************
 *
 *   [platform_driver API]
 *
 *********************************************************/
unsigned char g_reg_value_sgm41510;
static ssize_t show_sgm41510_access(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	pr_info("[%s] 0x%x\n", __func__, g_reg_value_sgm41510);
	return sprintf(buf, "%u\n", g_reg_value_sgm41510);
}

static ssize_t store_sgm41510_access(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t size)
{
	int ret = 0;
	char *pvalue = NULL, *addr, *val;
	unsigned int reg_value = 0;
	unsigned int reg_address = 0;

	pr_info("[%s]\n", __func__);

	if (buf != NULL && size != 0) {
		pr_info("[%s] buf is %s and size is %zu\n", __func__, buf,
			size);

		pvalue = (char *)buf;
		if (size > 3) {
			addr = strsep(&pvalue, " ");
			ret = kstrtou32(addr, 16,
				(unsigned int *)&reg_address);
		} else
			ret = kstrtou32(pvalue, 16,
				(unsigned int *)&reg_address);

		if (size > 3) {
			val = strsep(&pvalue, " ");
			ret = kstrtou32(val, 16, (unsigned int *)&reg_value);
			pr_info(
			"[%s] write sgm41510 reg 0x%x with value 0x%x !\n",
			__func__,
			(unsigned int) reg_address, reg_value);
			ret = sgm41510_config_interface(reg_address,
				reg_value, 0xFF, 0x0);
		} else {
			ret = sgm41510_read_interface(reg_address,
					     &g_reg_value_sgm41510, 0xFF, 0x0);
			pr_info(
			"[%s] read sgm41510 reg 0x%x with value 0x%x !\n",
			__func__,
			(unsigned int) reg_address, g_reg_value_sgm41510);
			pr_info(
			"[%s] use \"cat sgm41510_access\" to get value\n",
			__func__);
		}
	}
	return size;
}

static DEVICE_ATTR(sgm41510_access, 0664, show_sgm41510_access,
		   store_sgm41510_access);	/* 664 */

static int sgm41510_user_space_probe(struct platform_device *dev)
{
	int ret_device_file = 0;

	pr_info("******** %s!! ********\n", __func__);

	ret_device_file = device_create_file(&(dev->dev),
					     &dev_attr_sgm41510_access);

	return 0;
}

static int sgm41510_charger_remove(struct i2c_client *client)
{
	//struct sgm41510 *bq = i2c_get_clientdata(client);

	//mutex_destroy(&bq->i2c_rw_lock);

	//sysfs_remove_group(&bq->dev->kobj, &sgm41510_attr_group);

	return 0;
}

static void sgm41510_charger_shutdown(struct i2c_client *client)
{
	//if((g_oplus_chip != NULL) && g_bq != NULL) {
	//	if((g_bq->hvdcp_can_enabled) && (g_oplus_chip->charger_exist)) {
	//		oplus_sgm41510_charging_disable();
	//	}
	//}
	disable_irq_nosync(client->irq);
	oplus_sgm41510_set_hz_mode(0);
	pr_info("******** %s!! ********\n", __func__);
}

struct platform_device sgm41510_user_space_device = {
	.name = "sgm41510-user",
	.id = -1,
};

static struct platform_driver sgm41510_user_space_driver = {
	.probe = sgm41510_user_space_probe,
	.driver = {
		.name = "sgm41510-user",
	},
};

static const struct of_device_id sgm41510_of_match[] = {
	{.compatible = "mediatek,sgm41510"},
	{},
};

static struct i2c_driver sgm41510_driver = {
	.driver = {
		.name = "sgm41510",
		.owner = THIS_MODULE,
		.of_match_table = sgm41510_of_match,
	},
	.probe = sgm41510_driver_probe,
	.remove = sgm41510_charger_remove,
	.shutdown = sgm41510_charger_shutdown,
	.id_table = sgm41510_i2c_id,
};

static int __init sgm41510_init(void)
{
	int ret = 0;

	/* i2c registeration using DTS instead of boardinfo*/
	pr_info("[%s] init start with i2c DTS", __func__);
	if (i2c_add_driver(&sgm41510_driver) != 0) {
		pr_info(
			"[%s] failed to register sgm41510 i2c driver.\n",
			__func__);
	} else {
		pr_info(
			"[%s] Success to register sgm41510 i2c driver.\n",
			__func__);
	}

	ret = platform_driver_register(&sgm41510_user_space_driver);
	if (ret) {
		pr_info("****[%s] Unable to register driver (%d)\n", __func__,
			ret);
		return ret;
	}

	return 0;
}

static void __exit sgm41510_exit(void)
{
	i2c_del_driver(&sgm41510_driver);
}
module_init(sgm41510_init);
module_exit(sgm41510_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("I2C sgm41510 Driver");
MODULE_AUTHOR("will cai <will.cai@mediatek.com>");
