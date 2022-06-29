/*
 * Copyright (C) 2018 MediaTek Inc.
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

#ifndef _sgm41510_SW_H_
#define _sgm41510_SW_H_

#include "../oplus_charger.h"


#define sgm41510_CON0      0x00
#define sgm41510_CON1      0x01
#define sgm41510_CON2      0x02
#define sgm41510_CON3      0x03
#define sgm41510_CON4      0x04
#define sgm41510_CON5      0x05
#define sgm41510_CON6      0x06
#define sgm41510_CON7      0x07
#define sgm41510_CON8      0x08
#define sgm41510_CON9      0x09
#define sgm41510_CON10      0x0A
#define	sgm41510_CON11		0x0B
#define	sgm41510_CON12		0x0C
#define	sgm41510_CON13		0x0D
#define	sgm41510_CON14		0x0E
#define	sgm41510_CON15		0x0F
#define	sgm41510_CON16		0x10
#define	sgm41510_CON17		0x11
#define	sgm41510_CON18		0x12
#define	sgm41510_CON19		0x13
#define	sgm41510_CON20		0x14
#define	sgm41510_CON21		0x15
#define sgm41510_REG_NUM 22

/**********************************************************
 *
 *   [MASK/SHIFT]
 *
 *********************************************************/
//CON0
#define CON0_EN_HIZ_MASK   0x01
#define CON0_EN_HIZ_SHIFT  7

#define	CON0_EN_ILIM_PIN_MASK	0x01
#define	CON0_EN_ILIM_PIN_SHIFT 6

#define CON0_IINLIM_MASK   0x3F
#define CON0_IINLIM_SHIFT  0
#define CON0_IINLIM_MIN		100000
#define CON0_IINLIM_MAX		4900000
#define CON0_IINLIM_STEP	100000

//CON1

//CON2
#define CON2_CONV_START_MASK   0x01
#define CON2_CONV_START_SHIFT  7

#define CON2_BOOST_FREQ_MASK   0x01
#define CON2_BOOST_FREQ_SHIFT  5

#define CON2_FORCE_DPM_MASK   0x01
#define CON2_FORCE_DPM_SHIFT  1

#define	CON2_AUTO_DPDM_EN_MASK   0x01
#define	CON2_AUTO_DPDM_EN_SHIFT  0

//CON3
#define CON3_BAT_LOAD_EN_MASK   0x01
#define CON3_BAT_LOAD_EN_SHIFT  7

#define CON3_WDT_RST_MASK   0x01
#define CON3_WDT_RST_SHIFT  6

#define CON3_OTG_CONFIG_MASK   0x01
#define CON3_OTG_CONFIG_SHIFT  5

#define CON3_CHG_CONFIG_MASK   0x01
#define CON3_CHG_CONFIG_SHIFT  4

#define CON3_SYS_MIN_MASK   0x07
#define CON3_SYS_MIN_SHIFT  1

#define CON3_MIN_BAT_SEL_MASK   0x01
#define CON3_MIN_BAT_SEL_SHIFT  0

//CON4
#define CON4_PUMPX_EN_MASK     0x01
#define CON4_PUMPX_EN_SHIFT    7

#define CON4_ICHG_MASK   0x7F
#define CON4_ICHG_SHIFT  0
#define CON4_ICHG_MIN		0
#define CON4_ICHG_MAX		5056000
#define CON4_ICHG_STEP	64000

//CON5
#define CON5_IPRECHG_MASK   0x0F
#define CON5_IPRECHG_SHIFT  4

#define CON5_ITERM_MASK   0x0F
#define CON5_ITERM_SHIFT  0
#define CON5_ITERM_MIN		64000
#define CON5_ITERM_MAX		1024000
#define CON5_ITERM_STEP		64000

//CON6
#define	CON6_VREG_MASK		0x3F
#define	CON6_VREG_SHIFT		2
#define CON6_CV_MIN		3840000
#define CON6_CV_MAX		4608000
#define CON6_CV_STEP		16000

#define	CON6_BATLOWV_MASK	0x1
#define	CON6_BATLOWV_SHIFT	1

#define	CON6_VRECHG_MASK	0x01
#define	CON6_VRECHG_SHIFT	0

//CON7
#define	CON7_EN_TERM_MASK	0x01
#define	CON7_EN_TERM_SHIFT	7

#define CON7_STAT_DIS_MASK      0x01
#define CON7_STAT_DIS_SHIFT     6

#define CON7_WATCHDOG_MASK      0x03
#define CON7_WATCHDOG_SHIFT     4

#define	CON7_EN_TIMER_MASK		0x01
#define	CON7_EN_TIMER_SHIFT		3

#define	CON7_CHG_TIMER_MASK		0x03
#define	CON7_CHG_TIMER_SHIFT	1

//CON8
#define CON8_BATCMP_MASK      0x07
#define CON8_BATCMP_SHIFT     5

#define CON8_VCLAMP_MASK           0x07
#define CON8_VCLAMP_SHIFT          2

#define CON8_TREG_MASK           0x03
#define CON8_TREG_SHIFT          0

//CON9
#define CON9_TMR2X_EN_MASK           0x01
#define CON9_TMR2X_EN_SHIFT          6

#define CON9_BATFET_DIS_MASK           0x01
#define CON9_BATFET_DIS_SHIFT          5

#define CON9_BATFET_DLY_MASK           0x01
#define CON9_BATFET_DLY_SHIFT          3

#define CON9_BATFET_RST_EN_MASK           0x01
#define CON9_BATFET_RST_EN_SHIFT          2

#define CON9_PUMPX_UP_MASK           0x01
#define CON9_PUMPX_UP_SHIFT          1

#define CON9_PUMPX_DN_MASK           0x01
#define CON9_PUMPX_DN_SHIFT          0

//CON10
#define	CON10_BOOSTV_MASK				0x0F
#define	CON10_BOOSTV_SHIFT				4

#define	CON10_PFM_OTG_DIS_MASK			0x01
#define	CON10_PFM_OTG_DIS_SHIFT			3

#define	CON10_BOOST_LIM_MASK			0x7
#define	CON10_BOOST_LIM_SHIFT			0
#define CON10_BOOST_LIM_MIN		1200000
#define CON10_BOOST_LIM_MAX		4000000
#define CON10_BOOST_LIM_STEP	400000

//CON11
#define CON11_VBUS_STAT_MASK     0x07
#define CON11_VBUS_STAT_SHIFT    5

#define CON11_CHRG_STAT_MASK		0x03
#define CON11_CHRG_STAT_SHIFT		3

#define CON11_PG_STAT_MASK           0x01
#define CON11_PG_STAT_SHIFT          2

#define CON11_VSYS_STAT_MASK           0x01
#define CON11_VSYS_STAT_SHIFT          0

//CON12
#define CON12_WD_FAULT_MASK      0x01
#define CON12_WD_FAULT_SHIFT     7

#define CON12_BOOST_FAULT_MASK           0x01
#define CON12_BOOST_FAULT_SHIFT          6

#define CON12_CHG_FAULT_MASK           0x03
#define CON12_CHG_FAULT_SHIFT          4

#define CON12_BAT_FAULT_MASK           0x01
#define CON12_BAT_FAULT_SHIFT          3

//CON13
#define	CON13_FORCE_VINDPM_MASK	0x01
#define	CON13_FORCE_VINDPM_SHIFT	7

#define	CON13_VINDPM_MASK	0x7F
#define	CON13_VINDPM_SHIFT	0
#define CON13_MIVR_MIN		2600000
#define CON13_MIVR_MAX		15300000
#define CON13_MIVR_STEP	100000

//CON14
#define	CON14_THERM_STAT_MASK	0x01
#define	CON14_THERM_STAT_SHIFT	7

//CON15

//CON16

//CON17
#define	CON17_VBUS_GD_MASK	0x01
#define	CON17_VBUS_GD_SHIFT	7

//CON18

//CON19
#define CON19_VINDPM_STAT_MASK           0x01
#define CON19_VINDPM_STAT_SHIFT          7

#define CON19_IINLIM_STAT_MASK           0x01
#define CON19_IINLIM_STAT_SHIFT          6

//CON20
#define CON20_REG_RST_MASK           0x01
#define CON20_REG_RST_SHIFT          7

#define CON20_PN_MASK           0x07
#define CON20_PN_SHIFT          3

#define CON20_DEV_REV_MASK           0x03
#define CON20_DEV_REV_SHIFT          0

//CON21
#define CON21_CM_OUT_MASK           0x07
#define CON21_CM_OUT_SHIFT          5

#define CON21_VBUS_OV_MASK           0x07
#define CON21_VBUS_OV_SHIFT          2

enum sgm41510_chg_type {
	SGM41510_CHG_TYPE_NOVBUS = 0,
	SGM41510_CHG_TYPE_FLOAT,
	SGM41510_CHG_TYPE_SDP,
	SGM41510_CHG_TYPE_CDP,
	SGM41510_CHG_TYPE_DCP,
	SGM41510_CHG_TYPE_HVDCP,
	SGM41510_CHG_TYPE_MAX,
};

enum {
	QC_MODULE_NONE = 0,
	QC_MODULE_WT6670F,
	QC_MODULE_Z350,
};


/**********************************************************
 *
 *   [Extern Function]
 *
 *********************************************************/
/*//CON0----------------------------------------------------
extern void sgm41510_set_en_hiz(unsigned int val);
extern void sgm41510_set_vindpm(unsigned int val);
extern void sgm41510_set_iinlim(unsigned int val);
//CON1----------------------------------------------------
extern void sgm41510_set_reg_rst(unsigned int val);
//extern void sgm41510_set_pfm(unsigned int val);
extern void sgm41510_set_wdt_rst(unsigned int val);
extern void sgm41510_set_chg_config(unsigned int val);
extern void sgm41510_set_otg_config(unsigned int val);
extern void sgm41510_set_sys_min(unsigned int val);
extern void sgm41510_set_boost_lim(unsigned int val);
//CON2----------------------------------------------------
extern void sgm41510_set_ichg(unsigned int val);
//CON3----------------------------------------------------
extern void sgm41510_set_iprechg(unsigned int val);
extern void sgm41510_set_iterm(unsigned int val);
//CON4----------------------------------------------------
extern void sgm41510_set_vreg(unsigned int val);
extern void sgm41510_set_batlowv(unsigned int val);
extern void sgm41510_set_vrechg(unsigned int val);
//CON5----------------------------------------------------
extern void sgm41510_set_en_term(unsigned int val);
extern void sgm41510_set_term_stat(unsigned int val);
extern void sgm41510_set_watchdog(unsigned int val);
extern void sgm41510_set_en_timer(unsigned int val);
extern void sgm41510_set_chg_timer(unsigned int val);
//CON6----------------------------------------------------
extern void sgm41510_set_treg(unsigned int val);
//CON7----------------------------------------------------
extern void sgm41510_set_batfet_disable(unsigned int val);
extern void sgm41510_set_int_mask(unsigned int val);
//CON8----------------------------------------------------
extern unsigned int sgm41510_get_vbus_stat(void);
extern unsigned int sgm41510_get_chrg_stat(void);
extern unsigned int sgm41510_get_vsys_stat(void);
extern unsigned int sgm41510_get_pg_stat(void);
//---------------------------------------------------------

extern unsigned int sgm41510_reg_config_interface(unsigned char RegNum,
		unsigned char val);

extern unsigned int sgm41510_read_interface(unsigned char RegNum,
		unsigned char *val, unsigned char MASK, unsigned char SHIFT);
extern unsigned int sgm41510_config_interface(unsigned char RegNum,
		unsigned char val, unsigned char MASK, unsigned char SHIFT);*/
extern bool qc_module_z350;
extern bool charging_poweron;
extern int sgm41510_attachi_irq_handler(void);
extern int z350_get_charger_type(u8 *chg_type);
extern int z350_enable_chgdet_flow(bool en);
extern void z350_enable_otg(bool en);
extern int z350_dump_register(void);
extern int z350_qc_mode_select(u8 mode);
extern int wt6670f_get_charger_type(u8 *chg_type);
extern int wt6670f_enable_chgdet_flow(bool en);
extern void wt6670f_enable_otg(bool en);
extern int wt6670f_dump_register(void);
extern int wt6670f_qc_mode_select(u8 mode);
extern bool sgm41510_charger_exist_detect(void);
extern int wt6670f_set_apsd_rerun(bool en);
extern int z350_set_apsd_rerun(bool en);
#endif // _sgm41510_SW_H_

