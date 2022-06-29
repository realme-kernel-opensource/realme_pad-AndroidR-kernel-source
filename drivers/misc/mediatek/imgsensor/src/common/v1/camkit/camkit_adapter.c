/*
 * camkit_adapter.c
 *
 * Copyright (c) 2021-2021 LongCheer Technologies Co., Ltd.
 *
 * camkit interface adapted on the platform
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */

#include <linux/slab.h>
#include <linux/uaccess.h>

#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif

#ifdef CONFIG_MTK_CCU
#include "ccu_imgsensor_if.h"
#endif

#include "kd_imgsensor_errcode.h"
#include "imgsensor_sensor.h"
#include "imgsensor_hw.h"
#ifdef IMGSENSOR_OC_ENABLE
#include "imgsensor_oc.h"
#endif
#include "imgsensor.h"

#if defined(CONFIG_MTK_CAM_SECURE_I2C)
#include "imgsensor_ca.h"
#endif

#include "camkit_driver.h"
#include "kd_camkit_define.h"

#include "camkit_adapter.h"

#define PFX "[camkit_adapter]"
#define DEBUG_CAMKIT 0
#define LOG_DBG(fmt, args...) \
	do { \
		if (DEBUG_CAMKIT) \
			pr_info(PFX fmt, ##args); \
	} while (0)
#define LOG_INF(fmt, args...) pr_info(PFX fmt, ##args)
#define LOG_ERR(fmt, args...) pr_err(PFX fmt, ##args)

extern struct IMGSENSOR gimgsensor;

extern void IMGSENSOR_PROFILE_INIT(struct timeval *ptv);
extern void IMGSENSOR_PROFILE(struct timeval *ptv, char *tag);
extern void imgsensor_mutex_init(struct IMGSENSOR_SENSOR_INST *psensor_inst);
extern void imgsensor_mutex_lock(struct IMGSENSOR_SENSOR_INST *psensor_inst);
extern void imgsensor_mutex_unlock(struct IMGSENSOR_SENSOR_INST *psensor_inst);
extern char mtkcam0_name[4096];
extern char mtkcam1_name[4096];

struct IMGSENSOR_SENSOR *camkit_get_sensor(enum IMGSENSOR_SENSOR_IDX idx)
{
	if (idx < IMGSENSOR_SENSOR_IDX_MIN_NUM ||
		idx >= IMGSENSOR_SENSOR_IDX_MAX_NUM)
		return NULL;
	else
		return &gimgsensor.sensor[idx];
}

static int camkit_print_mode_info(struct camkit_mode_info *mode_info)
{
	LOG_DBG("pclk: %u", mode_info->pclk);
	LOG_DBG("linelength: %u", mode_info->linelength);
	LOG_DBG("framelength: %u", mode_info->framelength);
	LOG_DBG("startx: %u", mode_info->startx);
	LOG_DBG("starty: %u", mode_info->starty);
	LOG_DBG("grabwindow_width: %u", mode_info->grabwindow_width);
	LOG_DBG("grabwindow_height: %u", mode_info->grabwindow_height);
	LOG_DBG("mipi_settle_dc: %u", mode_info->mipi_data_lp2hs_settle_dc);
	LOG_DBG("max_framerate: %u", mode_info->max_framerate);
	LOG_DBG("mipi_pixel_rate: %u", mode_info->mipi_pixel_rate);
	LOG_DBG("mipi_trail_val: %u", mode_info->mipi_trail_val);

	return ERROR_NONE;
}

static uint32 camkit_translate_sensor_setting(
	struct camkit_i2c_reg_setting *i2c_setting)
{
	int len;
	int i;
	struct camkit_i2c_reg *setting = NULL;

	if (i2c_setting == NULL) {
		pr_err("[camkit][%s] invalid input arg\n", __func__);
		return EFAULT;
	}

	if (!i2c_setting->size || i2c_setting->setting == NULL) {
		pr_info("[camkit][%s] setting need't configure?\n", __func__);
		i2c_setting->setting = NULL;
		return ERROR_NONE;
	}

	LOG_DBG("sensor setting size: %u\n", i2c_setting->size);

	len = i2c_setting->size;
	setting = kzalloc(sizeof(struct camkit_i2c_reg) * len, GFP_KERNEL);
	if (!setting) {
		pr_err("[%s] memory not enough\n", __func__);
		i2c_setting->setting = NULL;
		return ENOMEM;
	}

	if (copy_from_user(setting,
		(void *)i2c_setting->setting,
		sizeof(struct camkit_i2c_reg) * len)) {
		pr_err("failed: copy_from_user");
		kfree(setting);
		i2c_setting->setting = NULL;
		return EFAULT;
	}

	i2c_setting->setting = setting;
	for (i = 0; i < len; i++) {
		LOG_DBG("setting[%d].addr: 0x%x\n", i, setting[i].addr);
		LOG_DBG("setting[%d].data: 0x%x\n", i, setting[i].data);
		LOG_DBG("setting[%d].delay: 0x%x\n", i, setting[i].delay);
	}

	LOG_DBG("setting size: %u\n", i2c_setting->size);
	LOG_DBG("setting addr_type: %u\n", i2c_setting->addr_type);
	LOG_DBG("setting data_type: %u\n", i2c_setting->data_type);
	LOG_DBG("setting delay: %u\n", i2c_setting->delay);

	return ERROR_NONE;
}

static uint32 camkit_translate_dump_setting(
	struct camkit_i2c_reg_table_array *dump_setting)
{
	int len;
	int i;
	struct camkit_i2c_reg_table *setting = NULL;

	if (dump_setting == NULL) {
		pr_err("[camkit][%s] invalid input arg\n", __func__);
		return EFAULT;
	}

	if (!dump_setting->size || !dump_setting->setting) {
		pr_info("[camkit][%s] dump setting not configure\n", __func__);
		dump_setting->setting = NULL;
		return ERROR_NONE;
	}

	LOG_DBG("dump setting size: %u\n", dump_setting->size);

	len = dump_setting->size;
	setting = kzalloc(sizeof(struct camkit_i2c_reg_table) * len, GFP_KERNEL);
	if (!setting) {
		pr_err("[%s] memory not enough\n", __func__);
		dump_setting->setting = NULL;
		return ENOMEM;
	}

	if (copy_from_user(setting,
		(void *)dump_setting->setting,
		sizeof(struct camkit_i2c_reg_table) * len)) {
		pr_err("failed: copy_from_user");
		kfree(setting);
		dump_setting->setting = NULL;
		return EFAULT;
	}

	dump_setting->setting = setting;
	for (i = 0; i < len; i++) {
		LOG_DBG("setting[%d].addr: 0x%x\n", i, setting[i].addr);
		LOG_DBG("setting[%d].data: 0x%x\n", i, setting[i].data);
		LOG_DBG("setting[%d] type: %u\n", i, setting[i].data_type);
		LOG_DBG("setting[%d] op: %u\n", i, setting[i].i2c_operation);
		LOG_DBG("setting[%d].delay: 0x%x\n", i, setting[i].delay);
	}

	return ERROR_NONE;
}

static uint32 camkit_translate_sensor_info(
	struct camkit_sensor_info_t *sensor_info)
{
	uint32 ret = ERROR_NONE;
	int i;

	LOG_DBG("sensor_id_reg: %u\n", sensor_info->sensor_id_reg);
	LOG_DBG("sensor_id: %u\n", sensor_info->sensor_id);
	LOG_DBG("checksum_value: %u\n", sensor_info->checksum_value);

	camkit_print_mode_info(&sensor_info->pre);
	camkit_print_mode_info(&sensor_info->cap);
	camkit_print_mode_info(&sensor_info->cap1);
	camkit_print_mode_info(&sensor_info->normal_video);
	camkit_print_mode_info(&sensor_info->hs_video);
	camkit_print_mode_info(&sensor_info->slim_video);
	camkit_print_mode_info(&sensor_info->custom1);
	camkit_print_mode_info(&sensor_info->custom2);
	camkit_print_mode_info(&sensor_info->custom3);
	camkit_print_mode_info(&sensor_info->custom4);
	camkit_print_mode_info(&sensor_info->custom5);

	ret |= camkit_translate_sensor_setting(&sensor_info->id_init_setting);
	ret |= camkit_translate_sensor_setting(&sensor_info->init_setting);
	ret |= camkit_translate_sensor_setting(&sensor_info->init_burst_setting);
	ret |= camkit_translate_sensor_setting(&sensor_info->pre_setting);
	ret |= camkit_translate_sensor_setting(&sensor_info->cap_setting);
	ret |= camkit_translate_sensor_setting(&sensor_info->cap1_setting);
	ret |= camkit_translate_sensor_setting(&sensor_info->normal_video_setting);
	ret |= camkit_translate_sensor_setting(&sensor_info->hs_video_setting);
	ret |= camkit_translate_sensor_setting(&sensor_info->slim_setting);
	ret |= camkit_translate_sensor_setting(&sensor_info->custom1_setting);
	ret |= camkit_translate_sensor_setting(&sensor_info->custom2_setting);
	ret |= camkit_translate_sensor_setting(&sensor_info->custom3_setting);
	ret |= camkit_translate_sensor_setting(&sensor_info->custom4_setting);
	ret |= camkit_translate_sensor_setting(&sensor_info->custom5_setting);
	ret |= camkit_translate_sensor_setting(&sensor_info->streamon_setting);
	ret |= camkit_translate_sensor_setting(&sensor_info->streamoff_setting);
	ret |= camkit_translate_sensor_setting(&sensor_info->test_pattern_on_setting);
	ret |= camkit_translate_sensor_setting(&sensor_info->test_pattern_off_setting);
	ret |= camkit_translate_dump_setting(&sensor_info->dump_info);
	ret |= camkit_translate_sensor_setting(&sensor_info->normal_to_long_ready_settings);
	ret |= camkit_translate_sensor_setting(&sensor_info->normal_to_long_end_settings);

	LOG_DBG("ae_shut_delay_frame: %u\n", sensor_info->ae_shut_delay_frame);
	LOG_DBG("ae_sensor_gain_delay_frame: %u\n",
		sensor_info->ae_sensor_gain_delay_frame);
	LOG_DBG("ae_ispGain_delay_frame: %u\n", sensor_info->ae_ispGain_delay_frame);
	LOG_DBG("ihdr_support: %u\n", sensor_info->ihdr_support);
	LOG_DBG("ihdr_le_firstline: %u\n", sensor_info->ihdr_le_firstline);
	LOG_DBG("sensor_mode_num: %u\n", sensor_info->sensor_mode_num);
	LOG_DBG("cap_delay_frame: %u\n", sensor_info->cap_delay_frame);
	LOG_DBG("pre_delay_frame: %u\n", sensor_info->pre_delay_frame);
	LOG_DBG("video_delay_frame: %u\n", sensor_info->video_delay_frame);
	LOG_DBG("hs_video_delay_frame: %u\n", sensor_info->hs_video_delay_frame);
	LOG_DBG("slim_video_delay_frame: %u\n", sensor_info->slim_video_delay_frame);
	LOG_DBG("custom1_delay_frame: %u\n", sensor_info->custom1_delay_frame);
	LOG_DBG("custom2_delay_frame: %u\n", sensor_info->custom2_delay_frame);
	LOG_DBG("custom3_delay_frame: %u\n", sensor_info->custom3_delay_frame);
	LOG_DBG("custom4_delay_frame: %u\n", sensor_info->custom4_delay_frame);
	LOG_DBG("custom5_delay_frame: %u\n", sensor_info->custom5_delay_frame);
	LOG_DBG("isp_driving_current: %u\n", sensor_info->isp_driving_current);
	LOG_DBG("sensor_interface_type: %u\n", sensor_info->sensor_interface_type);
	LOG_DBG("mipi_sensor_type: %u\n", sensor_info->mipi_sensor_type);
	LOG_DBG("mipi_settle_delay_mode: %u\n", sensor_info->mipi_settle_delay_mode);
	LOG_DBG("sensor_output_dataformat: %u\n", sensor_info->sensor_output_dataformat);
	LOG_DBG("mclk: %u\n", sensor_info->mclk);
	LOG_DBG("mipi_lane_num: %u\n", sensor_info->mipi_lane_num);
	LOG_DBG("i2c_addr_table[0]: %u\n", sensor_info->i2c_addr_table[0]);
	LOG_DBG("i2c_addr_table[1]: %u\n", sensor_info->i2c_addr_table[1]);
	LOG_DBG("i2c_addr_table[2]: %u\n", sensor_info->i2c_addr_table[2]);
	LOG_DBG("i2c_addr_table[3]: %u\n", sensor_info->i2c_addr_table[3]);
	LOG_DBG("i2c_addr_table[4]: %u\n", sensor_info->i2c_addr_table[4]);
	LOG_DBG("i2c_speed: %u\n", sensor_info->i2c_speed);
	LOG_DBG("addr_type: %u\n", sensor_info->addr_type);
	LOG_DBG("pdaf_support: %u\n", sensor_info->pdaf_support);

	// needn't translate, only dump info now
	for (i = 0; i < MAX_OUTPUT_INFO_SIZE; i++) {
		LOG_DBG("binning ratio[%d]: %u\n", i, sensor_info->binning_ratio[i]);
	}

	return ERROR_NONE;
}

static uint32 camkit_translate_sensor_ctrl(
	struct camkit_sensor_ctrl_t *sensor_ctrl)
{
	if (sensor_ctrl == NULL) {
		pr_err("[camkit][%s] invalid input arg\n", __func__);
		return EFAULT;
	}

	// needn't translate, only dump info now
	LOG_DBG("mirror: %u\n", sensor_ctrl->mirror);
	LOG_DBG("sensor_mode: %u\n", sensor_ctrl->sensor_mode);
	LOG_DBG("shutter: %u\n", sensor_ctrl->shutter);
	LOG_DBG("gain: %u\n", sensor_ctrl->gain);
	LOG_DBG("pclk: %u\n", sensor_ctrl->pclk);
	LOG_DBG("frame_length: %u\n", sensor_ctrl->frame_length);
	LOG_DBG("line_length: %u\n", sensor_ctrl->line_length);
	LOG_DBG("min_frame_length: %u\n", sensor_ctrl->min_frame_length);
	LOG_DBG("dummy_pixel: %u\n", sensor_ctrl->dummy_pixel);
	LOG_DBG("dummy_line: %u\n", sensor_ctrl->dummy_line);
	LOG_DBG("current_fps: %u\n", sensor_ctrl->current_fps);
	LOG_DBG("autoflicker_en: %u\n", sensor_ctrl->autoflicker_en);
	LOG_DBG("test_pattern: %u\n", sensor_ctrl->test_pattern);
	LOG_DBG("current_scenario_id: %d\n", sensor_ctrl->current_scenario_id);
	LOG_DBG("ihdr_en: %u\n", sensor_ctrl->ihdr_en);
	LOG_DBG("i2c_write_id: %u\n", sensor_ctrl->i2c_write_id);
	LOG_DBG("i2c_speed: %u\n", sensor_ctrl->i2c_speed);
	LOG_DBG("addr_type: %d\n", sensor_ctrl->addr_type);

	return ERROR_NONE;
}

static uint32 camkit_translate_sensor_output_info(
	struct camkit_sensor_output_info_t *output_info)
{
	int i;

	if (output_info == NULL) {
		pr_err("invalid input arg\n");
		return EFAULT;
	}

	// needn't translate, only dump info now
	for (i = 0; i < MAX_OUTPUT_INFO_SIZE; i++) {
		LOG_DBG("output_info[%d].full_w: %u\n", i, output_info[i].full_w);
		LOG_DBG("output_info[%d].full_h: %u\n", i, output_info[i].full_h);
		LOG_DBG("output_info[%d].x0_offset: %u\n", i, output_info[i].x0_offset);
		LOG_DBG("output_info[%d].y0_offset: %u\n", i, output_info[i].y0_offset);
		LOG_DBG("output_info[%d].w0_size: %u\n", i, output_info[i].w0_size);
		LOG_DBG("output_info[%d].h0_size: %u\n", i, output_info[i].h0_size);
		LOG_DBG("output_info[%d].scale_w: %u\n", i, output_info[i].scale_w);
		LOG_DBG("output_info[%d].scale_h: %u\n", i, output_info[i].scale_h);
		LOG_DBG("output_info[%d].x1_offset: %u\n", i, output_info[i].x1_offset);
		LOG_DBG("output_info[%d].y1_offset: %u\n", i, output_info[i].y1_offset);
		LOG_DBG("output_info[%d].w1_size: %u\n", i, output_info[i].w1_size);
		LOG_DBG("output_info[%d].h1_size: %u\n", i, output_info[i].h1_size);
		LOG_DBG("output_info[%d].x2_tg_offset: %u\n", i, output_info[i].x2_tg_offset);
		LOG_DBG("output_info[%d].y2_tg_offset: %u\n", i, output_info[i].y2_tg_offset);
		LOG_DBG("output_info[%d].w2_tg_size: %u\n", i, output_info[i].w2_tg_size);
		LOG_DBG("output_info[%d].h2_tg_size: %u\n", i, output_info[i].h2_tg_size);
	}

	return ERROR_NONE;
}

static uint32 camkit_translate_pdaf_info(
	struct camkit_sensor_pdaf_info_t *pdaf_info)
{
	int i;

	if (pdaf_info == NULL) {
		pr_err("invalid input arg\n");
		return EFAULT;
	}

	LOG_DBG("pdaf_info.i4OffsetX: %u\n", pdaf_info->i4OffsetX);
	LOG_DBG("pdaf_info.i4OffsetY: %u\n", pdaf_info->i4OffsetY);
	LOG_DBG("pdaf_info.i4PitchX: %u\n", pdaf_info->i4PitchX);
	LOG_DBG("pdaf_info.i4PitchY: %u\n", pdaf_info->i4PitchY);
	LOG_DBG("pdaf_info.i4PairNum: %u\n", pdaf_info->i4PairNum);
	LOG_DBG("pdaf_info.i4SubBlkW: %u\n", pdaf_info->i4SubBlkW);
	LOG_DBG("pdaf_info.i4SubBlkH: %u\n", pdaf_info->i4SubBlkH);

	for (i = 0; i < ARRAY_SIZE(pdaf_info->i4PosL); i++) {
		LOG_DBG("pdaf_info.i4PosL[%d][0]: %u\n", i, pdaf_info->i4PosL[i][0]);
		LOG_DBG("pdaf_info.i4PosL[%d][1]: %u\n", i, pdaf_info->i4PosL[i][1]);
	}
	for (i = 0; i < ARRAY_SIZE(pdaf_info->i4PosR); i++) {
		LOG_DBG("pdaf_info.i4PosR[%d][0]: %u\n", i, pdaf_info->i4PosR[i][0]);
		LOG_DBG("pdaf_info.i4PosR[%d][1]: %u\n", i, pdaf_info->i4PosR[i][1]);
	}

	LOG_DBG("pdaf_info.iMirrorFlip: %u\n", pdaf_info->iMirrorFlip);
	LOG_DBG("pdaf_info.i4BlockNumX: %u\n", pdaf_info->i4BlockNumX);
	LOG_DBG("pdaf_info.i4BlockNumY: %u\n", pdaf_info->i4BlockNumY);
	LOG_DBG("pdaf_info.i4LeFirst: %u\n", pdaf_info->i4LeFirst);

	for (i = 0; i < ARRAY_SIZE(pdaf_info->i4Crop); i++) {
		LOG_DBG("pdaf_info.i4Crop[%d][0]: %u\n", i, pdaf_info->i4Crop[i][0]);
		LOG_DBG("pdaf_info.i4Crop[%d][1]: %u\n", i, pdaf_info->i4Crop[i][1]);
	}

	return ERROR_NONE;

}

static uint32 camkit_translate_pin_type(int32 *pin_type)
{
	int pin;

	if (pin_type == NULL) {
		pr_err("invalid input arg\n");
		return EFAULT;
	}

	pin = *pin_type;
	switch (pin) {
	case CAMKIT_HW_PIN_NONE:
		*pin_type = (int32)IMGSENSOR_HW_PIN_NONE;
		break;
	case CAMKIT_HW_PIN_PDN:
		*pin_type = (int32)IMGSENSOR_HW_PIN_PDN;
		break;
	case CAMKIT_HW_PIN_RST:
		*pin_type = (int32)IMGSENSOR_HW_PIN_RST;
		break;
	/*case CAMKIT_HW_PIN_AVDD_EN:
		*pin_type = (int32)IMGSENSOR_HW_PIN_AVDD_EN;
		break;
	case CAMKIT_HW_PIN_AVDD_SEL:
		*pin_type = (int32)IMGSENSOR_HW_PIN_AVDD_SEL;
		break;
	case CAMKIT_HW_PIN_DVDD_EN:
		*pin_type = (int32)IMGSENSOR_HW_PIN_DVDD_EN;
		break;
	case CAMKIT_HW_PIN_DVDD_SEL:
		*pin_type = (int32)IMGSENSOR_HW_PIN_DVDD_SEL;
		break;
	case CAMKIT_HW_PIN_IOVDD_EN:
		*pin_type = (int32)IMGSENSOR_HW_PIN_IOVDD_EN;
		break;
	case CAMKIT_HW_PIN_AVDD1_EN:
		*pin_type = (int32)IMGSENSOR_HW_PIN_AVDD1_EN;
		break;
	case CAMKIT_HW_PIN_AFVDD_EN:
		*pin_type = (int32)IMGSENSOR_HW_PIN_AFVDD_EN;
		break;*/
	case CAMKIT_HW_PIN_AVDD:
		*pin_type = (int32)IMGSENSOR_HW_PIN_AVDD;
		break;
	case CAMKIT_HW_PIN_DVDD:
		*pin_type = (int32)IMGSENSOR_HW_PIN_DVDD;
		break;
	case CAMKIT_HW_PIN_DOVDD:
		*pin_type = (int32)IMGSENSOR_HW_PIN_DOVDD;
		break;
	case CAMKIT_HW_PIN_AFVDD:
		*pin_type = (int32)IMGSENSOR_HW_PIN_AFVDD;
		break;
#ifdef MIPI_SWITCH
	case CAMKIT_HW_PIN_MIPI_SWITCH_EN:
		*pin_type = (int32)IMGSENSOR_HW_PIN_MIPI_SWITCH_EN;
		break;
	case CAMKIT_HW_PIN_MIPI_SWITCH_SEL:
		*pin_type = (int32)IMGSENSOR_HW_PIN_MIPI_SWITCH_SEL;
		break;
#endif
	case CAMKIT_HW_PIN_MCLK:
		*pin_type = (int32)IMGSENSOR_HW_PIN_MCLK;
		break;
	case CAMKIT_HW_PIN_UNDEF:
		*pin_type = (int32)IMGSENSOR_HW_PIN_UNDEF;
		break;
	default:
		*pin_type = (int32)IMGSENSOR_HW_PIN_UNDEF;
		break;
	}

	return ERROR_NONE;
}

static uint32 camkit_translate_pin_value(int32 *pin_val)
{
	int pin_value;

	if (pin_val == NULL) {
		pr_err("invalid input arg\n");
		return EFAULT;
	}

	pin_value = *pin_val;
	switch (pin_value) {
	case CAMKIT_HW_PIN_VALUE_NONE:
		*pin_val = (int32)IMGSENSOR_HW_PIN_STATE_NONE;
		break;
	case CAMKIT_HW_PIN_VALUE_LOW:
		*pin_val = (int32)IMGSENSOR_HW_PIN_STATE_LEVEL_0;
		break;
	case CAMKIT_HW_PIN_VALUE_HIGH:
		*pin_val = (int32)IMGSENSOR_HW_PIN_STATE_LEVEL_HIGH;
		break;
	case CAMKIT_HW_PIN_VALUE_1000:
		*pin_val = (int32)IMGSENSOR_HW_PIN_STATE_LEVEL_1000;
		break;
	case CAMKIT_HW_PIN_VALUE_1050:
		*pin_val = (int32)IMGSENSOR_HW_PIN_STATE_LEVEL_1050;
		break;
	case CAMKIT_HW_PIN_VALUE_1100:
		*pin_val = (int32)IMGSENSOR_HW_PIN_STATE_LEVEL_1100;
		break;
	case CAMKIT_HW_PIN_VALUE_1200:
		*pin_val = (int32)IMGSENSOR_HW_PIN_STATE_LEVEL_1200;
		break;
	case CAMKIT_HW_PIN_VALUE_1210:
		*pin_val = (int32)IMGSENSOR_HW_PIN_STATE_LEVEL_1210;
		break;
	case CAMKIT_HW_PIN_VALUE_1220:
		*pin_val = (int32)IMGSENSOR_HW_PIN_STATE_LEVEL_1220;
		break;
	case CAMKIT_HW_PIN_VALUE_1250:
		*pin_val = (int32)IMGSENSOR_HW_PIN_STATE_LEVEL_1250;
		break;
	case CAMKIT_HW_PIN_VALUE_1500:
		*pin_val = (int32)IMGSENSOR_HW_PIN_STATE_LEVEL_1500;
		break;
	case CAMKIT_HW_PIN_VALUE_1800:
		*pin_val = (int32)IMGSENSOR_HW_PIN_STATE_LEVEL_1800;
		break;
	case CAMKIT_HW_PIN_VALUE_2500:
		*pin_val = (int32)IMGSENSOR_HW_PIN_STATE_LEVEL_2500;
		break;
	case CAMKIT_HW_PIN_VALUE_2800:
		*pin_val = (int32)IMGSENSOR_HW_PIN_STATE_LEVEL_2800;
		break;
	case CAMKIT_HW_PIN_VALUE_2900:
		*pin_val = (int32)IMGSENSOR_HW_PIN_STATE_LEVEL_2900;
		break;
	case CAMKIT_HW_PIN_VALUE_3000:
		*pin_val = (int32)IMGSENSOR_HW_PIN_STATE_LEVEL_3000;
		break;
	default:
		*pin_val = (int32)IMGSENSOR_HW_PIN_STATE_NONE;
		break;
	}

	return ERROR_NONE;
}

static uint32 camkit_translate_power_info(
	struct camkit_hw_power_info_t *power_info)
{
	uint32 ret = ERROR_NONE;

	if (power_info == NULL) {
		pr_err("invalid input arg\n");
		return EFAULT;
	}

	LOG_DBG("power_info.pin_type: %d\n", power_info->pin_type);
	LOG_DBG("power_info.pin_val: %d\n", power_info->pin_val);
	LOG_DBG("power_info.delay: %u\n", power_info->pin_delay);
	ret |= camkit_translate_pin_type(&(power_info->pin_type));
	ret |= camkit_translate_pin_value(&(power_info->pin_val));
	LOG_DBG("translated pin_type: %d\n", power_info->pin_type);
	LOG_DBG("translated pin_val: %d\n", power_info->pin_val);
	LOG_DBG("translated delay: %u\n", power_info->pin_delay);

	return ret;
}

static uint32 camkit_translate_sensor_priv_gain_info(
	struct private_again_info *again_info)
{
	int32 len;
	int32 i;
	//kal_int32 j;
	struct priv_again_map *again_map = NULL;

	if (again_info == NULL) {
		LOG_ERR("invalid input arg");
		return EFAULT;
	}

	LOG_DBG("sensor again info size: %u gain map size: %u",
		again_info->size, (uint32)(sizeof(struct priv_again_map)));

	len = again_info->size;
	if (len <= 0) {
		LOG_ERR("the gain map not configure");
		again_info->again_map = NULL;
		return ERROR_NONE;
	}

	again_map = kzalloc(sizeof(struct priv_again_map) * len, GFP_KERNEL);
	if (!again_map) {
		LOG_ERR("memory not enough");
		again_info->again_map = NULL;
		return ENOMEM;
	}

	if (copy_from_user(again_map,
		(void *)again_info->again_map,
		sizeof(struct priv_again_map) * len)) {
		pr_err("failed: copy_from_user");
		kfree(again_map);
		again_info->again_map = NULL;
		return EFAULT;
	}

	again_info->again_map = again_map;
	for (i = 0; i < len; i++) {
		LOG_DBG("again_map[%d].again: %u", i, again_map[i].again_val);
		LOG_DBG("again_map[%d].size: %u", i, again_map[i].size);
	}

	return ERROR_NONE;
}

static uint32 camkit_translate_sensor_aec_map(
	struct aec_ops_map *aec_map)
{
	int32 len;
	int32 i;
	int32 j;
	struct camkit_aec_op *aec_ops = NULL;

	if (aec_map == NULL) {
		pr_err("invalid input arg\n");
		return EFAULT;
	}

	LOG_DBG("sensor aec map size: %u\n", aec_map->size);

	len = aec_map->size;
	if (len <= 0) {
		pr_err("the aec map need not configure\n");
		aec_map->aec_ops = NULL;
		return ERROR_NONE;
	}
	aec_ops = kzalloc(sizeof(struct camkit_aec_op) * len, GFP_KERNEL);
	if (!aec_ops) {
		pr_err("[%s] memory not enough\n", __func__);
		aec_map->aec_ops = NULL;
		return ENOMEM;
	}

	if (copy_from_user(aec_ops,
		(void *)aec_map->aec_ops,
		sizeof(struct camkit_aec_op) * len)) {
		pr_err("failed: copy_from_user");
		kfree(aec_ops);
		aec_map->aec_ops = NULL;
		return EFAULT;
	}

	aec_map->aec_ops = aec_ops;
	for (i = 0; i < len; i++) {
		LOG_DBG("aec_ops[%d].op_type: %d\n", i, aec_ops[i].op_type);
		LOG_DBG("aec_ops[%d].size: %d\n", i, aec_ops[i].size);
		if (aec_ops[i].size > 0 && aec_ops[i].size < MAX_AEC_REGS) {
			for (j = 0; j < aec_ops[i].size; j++) {
				LOG_DBG("aec_ops[%d].i2c_setting[%d].addr: 0x%x\n",
					i, j, aec_ops[i].i2c_setting[j].addr);
				LOG_DBG("aec_ops[%d].i2c_setting[%d].val: 0x%x\n",
					i, j, aec_ops[i].i2c_setting[j].val);
				LOG_DBG("aec_ops[%d].i2c_setting[%d].len: %d\n",
					i, j, aec_ops[i].i2c_setting[j].len);
				LOG_DBG("aec_ops[%d].i2c_setting[%d].mask: 0x%x\n",
					i, j, aec_ops[i].i2c_setting[j].mask);
				LOG_DBG("aec_ops[%d].i2c_setting[%d].mode: %d\n",
					i, j, aec_ops[i].i2c_setting[j].mode);
				LOG_DBG("aec_ops[%d].i2c_setting[%d].addr_type: %d\n",
					i, j, aec_ops[i].i2c_setting[j].addr_type);
				LOG_DBG("aec_ops[%d].i2c_setting[%d].data_type: %d\n",
					i, j, aec_ops[i].i2c_setting[j].data_type);

				LOG_DBG("aec_ops[%d].aec_setting[%d].addr: 0x%x\n",
					i, j, aec_ops[i].aec_setting[j].addr);
				LOG_DBG("aec_ops[%d].aec_setting[%d].max_val: 0x%x\n",
					i, j, aec_ops[i].aec_setting[j].max_val);
				LOG_DBG("aec_ops[%d].aec_setting[%d].shift: %d\n",
					i, j, aec_ops[i].aec_setting[j].shift);
				LOG_DBG("aec_ops[%d].aec_setting[%d].mask: 0x%x\n",
					i, j, aec_ops[i].aec_setting[j].mask);
				LOG_DBG("aec_ops[%d].aec_setting[%d].addr_type: %d\n",
					i, j, aec_ops[i].aec_setting[j].addr_type);
				LOG_DBG("aec_ops[%d].aec_setting[%d].data_type: %d\n",
					i, j, aec_ops[i].aec_setting[j].data_type);
			}
		}
	}

	return ERROR_NONE;
}

static uint32 camkit_translate_aec_info(
	struct camkit_aec_info_t *aec_info)
{
	uint32 ret = ERROR_NONE;

	if (aec_info == NULL) {
		pr_err("invalid input arg\n");
		return EFAULT;
	}

	LOG_DBG("aec_info.min_again: %u\n", aec_info->min_again);
	LOG_DBG("aec_info.max_again: %u\n", aec_info->max_again);
	LOG_DBG("aec_info.min_dgain: %u\n", aec_info->min_dgain);
	LOG_DBG("aec_info.max_dgain: %u\n", aec_info->max_dgain);
	LOG_DBG("aec_info.reg_gain_1x: %u\n", aec_info->reg_gain_1x);
	LOG_DBG("aec_info.dgain_decimator: %u\n", aec_info->dgain_decimator);
	LOG_DBG("aec_info.min_linecount: %u\n", aec_info->min_linecount);
	LOG_DBG("aec_info.max_linecount: %u\n", aec_info->max_linecount);
	LOG_DBG("aec_info.vts_offset: %u\n", aec_info->vts_offset);
	LOG_DBG("aec_info.again_type: %u\n", aec_info->again_type);
	LOG_DBG("aec_info.smia_coeff.m0: %u\n", aec_info->smia_coeff.m0);
	LOG_DBG("aec_info.smia_coeff.m1: %u\n", aec_info->smia_coeff.m1);
	LOG_DBG("aec_info.smia_coeff.m2: %u\n", aec_info->smia_coeff.m2);
	LOG_DBG("aec_info.smia_coeff.c0: %u\n", aec_info->smia_coeff.c0);
	LOG_DBG("aec_info.smia_coeff.c1: %u\n", aec_info->smia_coeff.c1);
	LOG_DBG("aec_info.smia_coeff.c2: %u\n", aec_info->smia_coeff.c2);

	ret |= camkit_translate_sensor_priv_gain_info(&aec_info->priv_again_info);
	ret |= camkit_translate_sensor_aec_map(&aec_info->gain_ops_map);
	ret |= camkit_translate_sensor_aec_map(&aec_info->expo_ops_map);
	ret |= camkit_translate_sensor_aec_map(&aec_info->vts_ops_map);

	return ret;
}

static uint32 camkit_translate_vc_info(
	struct camkit_sensor_vc_info_t *vc_info)
{
	uint32 i;

	if (vc_info == NULL) {
		pr_err("invalid input arg\n");
		return EFAULT;
	}

	for (i = 0; i < MAX_OUTPUT_INFO_SIZE; i++) {
		LOG_DBG("vc[%d].vc_num: %u\n", i, vc_info[i].vc_num);
		LOG_DBG("vc[%d].vc_pixel_num: %u\n", i, vc_info[i].vc_pixel_num);
		LOG_DBG("vc[%d].mode_select: %u\n", i, vc_info[i].mode_select);
		LOG_DBG("vc[%d].expo_ratio: %u\n", i, vc_info[i].expo_ratio);
		LOG_DBG("vc[%d].od_value: %u\n", i, vc_info[i].od_value);
		LOG_DBG("vc[%d].rg_stats_mode: %u\n", i, vc_info[i].rg_stats_mode);

		LOG_DBG("vc[%d].vc0_id: %u\n", i, vc_info[i].vc0_id);
		LOG_DBG("vc[%d].vc0_data_type: %u\n", i, vc_info[i].vc0_data_type);
		LOG_DBG("vc[%d].vc0_sizeh: %u\n", i, vc_info[i].vc0_sizeh);
		LOG_DBG("vc[%d].vc0_sizev: %u\n", i, vc_info[i].vc0_sizev);
		LOG_DBG("vc[%d].vc1_id: %u\n", i, vc_info[i].vc1_id);
		LOG_DBG("vc[%d].vc1_data_type: %u\n", i, vc_info[i].vc1_data_type);
		LOG_DBG("vc[%d].vc1_sizeh: %u\n", i, vc_info[i].vc1_sizeh);
		LOG_DBG("vc[%d].vc1_sizev: %u\n", i, vc_info[i].vc1_sizev);
		LOG_DBG("vc[%d].vc2_id: %u\n", i, vc_info[i].vc2_id);
		LOG_DBG("vc[%d].vc2_data_type: %u\n", i, vc_info[i].vc2_data_type);
		LOG_DBG("vc[%d].vc2_sizeh: %u\n", i, vc_info[i].vc2_sizeh);
		LOG_DBG("vc[%d].vc2_sizev: %u\n", i, vc_info[i].vc2_sizev);
		LOG_DBG("vc[%d].vc3_id: %u\n", i, vc_info[i].vc3_id);
		LOG_DBG("vc[%d].vc3_data_type: %u\n", i, vc_info[i].vc3_data_type);
		LOG_DBG("vc[%d].vc3_sizeh: %u\n", i, vc_info[i].vc3_sizeh);
		LOG_DBG("vc[%d].vc3_sizev: %u\n", i, vc_info[i].vc3_sizev);
		LOG_DBG("vc[%d].vc4_id: %u\n", i, vc_info[i].vc4_id);
		LOG_DBG("vc[%d].vc4_data_type: %u\n", i, vc_info[i].vc4_data_type);
		LOG_DBG("vc[%d].vc4_sizeh: %u\n", i, vc_info[i].vc4_sizeh);
		LOG_DBG("vc[%d].vc4_sizev: %u\n", i, vc_info[i].vc4_sizev);
		LOG_DBG("vc[%d].vc5_id: %u\n", i, vc_info[i].vc5_id);
		LOG_DBG("vc[%d].vc5_data_type: %u\n", i, vc_info[i].vc5_data_type);
		LOG_DBG("vc[%d].vc5_sizeh: %u\n", i, vc_info[i].vc5_sizeh);
		LOG_DBG("vc[%d].vc5_sizev: %u\n", i, vc_info[i].vc5_sizev);
	}

	return ERROR_NONE;
}

static uint32 camkit_translate_correction_info(
	struct camkit_sensor_correction_t *correction_info)
{
	uint32 ret = ERROR_NONE;

	if (correction_info == NULL) {
		pr_err("invalid input arg\n");
		return EFAULT;
	}

	LOG_DBG("qsc_apply: %u\n", correction_info->qsc_apply);
	LOG_DBG("eeprom_qsc_addr: %u\n", correction_info->eeprom_qsc_addr);
	LOG_DBG("sensor_qsc_addr: %u\n", correction_info->sensor_qsc_addr);
	LOG_DBG("qsc_len: %u\n", correction_info->qsc_len);
	LOG_DBG("spc_apply: %u\n", correction_info->spc_apply);
	LOG_DBG("eeprom_pdaf_addr: %u\n", correction_info->eeprom_pdaf_addr);
	LOG_DBG("pdaf_len: %u\n", correction_info->pdaf_len);
	LOG_DBG("lsc_start_addr: %u\n", correction_info->lsc_start_addr);
	LOG_DBG("lsc_addr_len: %u\n", correction_info->lsc_addr_len);
	LOG_DBG("rsc_start_addr: %u\n", correction_info->rsc_start_addr);
	LOG_DBG("rsc_addr_len: %u\n", correction_info->rsc_addr_len);

	return ret;
}

static uint32 camkit_translate_sensor_params(
	struct camkit_params *kit_params)
{
	struct camkit_sensor_params *sensor_params = NULL;
	uint32 ret = ERROR_NONE;
	int i;

	if (kit_params == NULL) {
		pr_err("invalid input arg\n");
		return EFAULT;
	}

	sensor_params = kzalloc(sizeof(struct camkit_sensor_params), GFP_KERNEL);
	if (!sensor_params) {
		pr_err("memory not enough\n");
		kit_params->sensor_params = NULL;
		return ENOMEM;
	}

	if (copy_from_user(sensor_params,
		(void *)kit_params->sensor_params,
		sizeof(struct camkit_sensor_params))) {
		pr_err("failed: copy camkit parameters from user\n");
		kfree(sensor_params);
		kit_params->sensor_params = NULL;
		return EFAULT;
	}
	kit_params->sensor_params = sensor_params;

	ret |= camkit_translate_sensor_info(&sensor_params->sensor_info);
	ret |= camkit_translate_sensor_ctrl(&sensor_params->sensor_ctrl);
	ret |= camkit_translate_sensor_output_info(sensor_params->output_info);
	ret |= camkit_translate_pdaf_info(&sensor_params->pdaf_info);
	for (i = 0; i < ARRAY_SIZE(sensor_params->power_info); i++) {
		ret |= camkit_translate_power_info(&(sensor_params->power_info[i]));
		ret |= camkit_translate_power_info(&(sensor_params->power_down_info[i]));
	}
	ret |= camkit_translate_aec_info(&sensor_params->aec_info);
	ret |= camkit_translate_vc_info(sensor_params->vc_info);
	ret |= camkit_translate_correction_info(&sensor_params->correction_info);

	return ret;
}

static uint32 camkit_translate_module_params(
	struct camkit_params *kit_params)
{
	struct camkit_module_params *module_params = NULL;

	if (kit_params == NULL) {
		pr_err("invalid input arg\n");
		return EFAULT;
	}

	module_params = kzalloc(sizeof(struct camkit_module_params), GFP_KERNEL);
	if (!module_params) {
		pr_err("memory not enough\n");
		kit_params->module_params = NULL;
		return ENOMEM;
	}

	if (copy_from_user(module_params,
		(void *)kit_params->module_params,
		sizeof(struct camkit_module_params))) {
		pr_err("failed: copy camkit parameters from user\n");
		kfree(module_params);
		kit_params->module_params = NULL;
		return EFAULT;
	}
	kit_params->module_params = module_params;

	LOG_DBG("skip_module_id: %u\n", module_params->skip_module_id);
	LOG_DBG("eeprom_i2c_addr: %u\n", module_params->eeprom_i2c_addr);
	LOG_DBG("module_code_addr: %u\n", module_params->module_code_addr);
	LOG_DBG("module_code: %u\n", module_params->module_code);
	LOG_DBG("lens_type_addr: %u\n", module_params->lens_type_addr);
	LOG_DBG("lens_type: %u\n", module_params->lens_type);
	LOG_DBG("addr_type: %u\n", module_params->addr_type);
	LOG_DBG("data_type: %u\n", module_params->data_type);
	LOG_DBG("sensor_name: %s\n", module_params->sensor_name);

	return ERROR_NONE;
}

static uint32 camkit_fill_sensor_params(struct IMGSENSOR_SENSOR *sensor,
	struct camkit_params *user_kit_params)
{
	struct camkit_params *kit_params = NULL;
	uint32 ret = ERROR_NONE;

	if (sensor == NULL || user_kit_params == NULL) {
		pr_err("invalid input arg\n");
		return EFAULT;
	}

	kit_params = kzalloc(sizeof(struct camkit_params), GFP_KERNEL);
	if (!kit_params) {
		pr_err("memory not enough\n");
		return ENOMEM;
	}

	if (copy_from_user(kit_params, (void *)user_kit_params,
		sizeof(struct camkit_params))) {
		pr_err("failed: copy camkit parameters from user\n");
		kfree(kit_params);
		return EFAULT;
	}
	sensor->kit_params = kit_params;

	ret |= camkit_translate_sensor_params(kit_params);
	ret |= camkit_translate_module_params(kit_params);

	return ret;
}

static int camkit_free_sensor_params(struct camkit_params *kit_params)
{
	struct camkit_sensor_info_t *sensor_info = NULL;
	struct camkit_aec_info_t *aec_info = NULL;
	struct camkit_sensor_params *sensor_params = NULL;

	if (kit_params != NULL && kit_params->sensor_params != NULL) {
		sensor_params = kit_params->sensor_params;
		sensor_info = &(sensor_params->sensor_info);
		if (sensor_info->id_init_setting.setting != NULL)
			kfree(sensor_info->id_init_setting.setting);
		if (sensor_info->init_setting.setting != NULL)
			kfree(sensor_info->init_setting.setting);
		if (sensor_info->init_burst_setting.setting != NULL)
			kfree(sensor_info->init_burst_setting.setting);
		if (sensor_info->pre_setting.setting != NULL)
			kfree(sensor_info->pre_setting.setting);
		if (sensor_info->cap_setting.setting != NULL)
			kfree(sensor_info->cap_setting.setting);
		if (sensor_info->cap1_setting.setting != NULL)
			kfree(sensor_info->cap1_setting.setting);
		if (sensor_info->normal_video_setting.setting != NULL)
			kfree(sensor_info->normal_video_setting.setting);
		if (sensor_info->hs_video_setting.setting != NULL)
			kfree(sensor_info->hs_video_setting.setting);
		if (sensor_info->slim_setting.setting != NULL)
			kfree(sensor_info->slim_setting.setting);
		if (sensor_info->custom1_setting.setting != NULL)
			kfree(sensor_info->custom1_setting.setting);
		if (sensor_info->custom2_setting.setting != NULL)
			kfree(sensor_info->custom2_setting.setting);
		if (sensor_info->custom3_setting.setting != NULL)
			kfree(sensor_info->custom3_setting.setting);
		if (sensor_info->custom4_setting.setting != NULL)
			kfree(sensor_info->custom4_setting.setting);
		if (sensor_info->custom5_setting.setting != NULL)
			kfree(sensor_info->custom5_setting.setting);
		if (sensor_info->streamon_setting.setting != NULL)
			kfree(sensor_info->streamon_setting.setting);
		if (sensor_info->streamoff_setting.setting != NULL)
			kfree(sensor_info->streamoff_setting.setting);
		if (sensor_info->test_pattern_on_setting.setting != NULL)
			kfree(sensor_info->test_pattern_on_setting.setting);
		if (sensor_info->test_pattern_off_setting.setting != NULL)
			kfree(sensor_info->test_pattern_off_setting.setting);
		if (sensor_info->normal_to_long_ready_settings.setting != NULL)
			kfree(sensor_info->normal_to_long_ready_settings.setting);
		if (sensor_info->normal_to_long_end_settings.setting != NULL)
			kfree(sensor_info->normal_to_long_end_settings.setting);

		aec_info = &(sensor_params->aec_info);
		if (aec_info->gain_ops_map.aec_ops != NULL)
			kfree(aec_info->gain_ops_map.aec_ops);
		if (aec_info->expo_ops_map.aec_ops != NULL)
			kfree(aec_info->expo_ops_map.aec_ops);
		if (aec_info->vts_ops_map.aec_ops != NULL)
			kfree(aec_info->vts_ops_map.aec_ops);
		if (aec_info->priv_again_info.again_map != NULL)
			kfree(aec_info->priv_again_info.again_map);

		kfree(sensor_params);
		sensor_params = NULL;
	}

	if (kit_params != NULL && kit_params->module_params != NULL) {
		kfree(kit_params->module_params);
		kit_params->module_params = NULL;
	}

	if (kit_params != NULL) {
		kfree(kit_params);
		kit_params = NULL;
	}

	return ERROR_NONE;
}

int32 camkit_sensor_open(struct IMGSENSOR_SENSOR *psensor)
{
	int32 ret = ERROR_NONE;
#if defined(CONFIG_MTK_CAM_SECURE_I2C)
	int32 ret_sec = ERROR_NONE;
	struct command_params c_params = {0};
#endif
	struct IMGSENSOR             *pimgsensor   = &gimgsensor;
	struct IMGSENSOR_SENSOR_INST *psensor_inst = &psensor->inst;
	struct sensor_kit_ops          *sensor_ops = psensor->sensor_ops;
	struct camkit_params               *params = psensor->kit_params;

#ifdef CONFIG_MTK_CCU
	struct ccu_sensor_info ccuSensorInfo;
	enum IMGSENSOR_SENSOR_IDX sensor_idx = psensor->inst.sensor_idx;
	struct i2c_client *pi2c_client = NULL;
#endif

	if (sensor_ops && sensor_ops->sensor_open && psensor_inst) {

		/* turn on power */
		IMGSENSOR_PROFILE_INIT(&psensor_inst->profile_time);

		ret = camkit_hw_power(&pimgsensor->hw,
				psensor,
				IMGSENSOR_HW_POWER_STATUS_ON);

		if (ret != IMGSENSOR_RETURN_SUCCESS) {
			pr_err("[%s]", __func__);
			return ret;
		}

		IMGSENSOR_PROFILE(&psensor_inst->profile_time,
			"kdCISModulePowerOn");

		imgsensor_mutex_lock(psensor_inst);

#if defined(CONFIG_MTK_CAM_SECURE_I2C)
		PK_INFO("%s secure state %d", __func__,
		(int)(&gimgsensor)->imgsensor_sec_flag);
		if ((&gimgsensor)->imgsensor_sec_flag) {
			ret = imgsensor_ca_invoke_command(
				IMGSENSOR_TEE_CMD_OPEN, c_params, &ret_sec);

		} else {
#endif
			ret = sensor_ops->sensor_open(params);
#if defined(CONFIG_MTK_CAM_SECURE_I2C)
		}
#endif

		if (ret != ERROR_NONE) {
			// imgsensor_hw_dump(&pimgsensor->hw);
			camkit_hw_power(&pimgsensor->hw,
				psensor,
				IMGSENSOR_HW_POWER_STATUS_OFF);
			pr_err("sensor_open fail");
		} else {
			psensor_inst->state = IMGSENSOR_STATE_OPEN;
		}

#ifdef IMGSENSOR_OC_ENABLE
		if (ret == ERROR_NONE)
			imgsensor_oc_interrupt(IMGSENSOR_HW_POWER_STATUS_ON);
#endif

#ifdef CONFIG_MTK_CCU
		ccuSensorInfo.slave_addr =
		    (psensor_inst->i2c_cfg.pinst->msg->addr << 1); //youhongtao_temp
		ccuSensorInfo.sensor_name_string =
		    (char *)(params->module_params->sensor_name);
		pi2c_client = psensor_inst->i2c_cfg.pinst->pi2c_client;
		if (pi2c_client)
			ccuSensorInfo.i2c_id = (((struct mt_i2c *)
				i2c_get_adapdata(pi2c_client->adapter))->id);
		else
			ccuSensorInfo.i2c_id = -1;
		ccu_set_sensor_info(sensor_idx, &ccuSensorInfo);
#endif

		imgsensor_mutex_unlock(psensor_inst);

		IMGSENSOR_PROFILE(&psensor_inst->profile_time, "sensor_open");
	}

	return ret;
}

uint32 camkit_sensor_get_info(
	struct IMGSENSOR_SENSOR *psensor,
	uint32 scenario_id,
	MSDK_SENSOR_INFO_STRUCT *sensor_info,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	uint32 ret = ERROR_NONE;
	struct IMGSENSOR_SENSOR_INST *psensor_inst = &psensor->inst;

	if (psensor->sensor_ops &&
	    psensor->sensor_ops->sensor_get_info &&
	    psensor_inst &&
	    sensor_info &&
	    sensor_config_data) {

		imgsensor_mutex_lock(psensor_inst);

		ret = psensor->sensor_ops->sensor_get_info(
			psensor->kit_params,
		    (enum MSDK_SCENARIO_ID_ENUM)(scenario_id),
		    sensor_info,
		    sensor_config_data);

		if (ret != ERROR_NONE)
			pr_err("[%s] sensor_get_info failed\n", __func__);

		imgsensor_mutex_unlock(psensor_inst);
	}

	return ret;
}

uint32 camkit_sensor_get_resolution(
	struct IMGSENSOR_SENSOR *psensor,
	MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	MUINT32 ret = ERROR_NONE;
	struct IMGSENSOR_SENSOR_INST *psensor_inst = &psensor->inst;

	if (psensor->sensor_ops &&
	    psensor->sensor_ops->sensor_get_resolution &&
	    psensor_inst) {

		imgsensor_mutex_lock(psensor_inst);

		ret = psensor->sensor_ops->sensor_get_resolution(
			psensor->kit_params,
			sensor_resolution);
		if (ret != ERROR_NONE)
			pr_err("[%s]\n", __func__);

		imgsensor_mutex_unlock(psensor_inst);
	}

	return ret;
}

uint32 camkit_sensor_control(
	struct IMGSENSOR_SENSOR *psensor,
	enum MSDK_SCENARIO_ID_ENUM scenario_id)
{
	uint32 ret = ERROR_NONE;
#if defined(CONFIG_MTK_CAM_SECURE_I2C)
	int32 ret_sec = ERROR_NONE;
	struct command_params c_params;
#endif
	struct IMGSENSOR_SENSOR_INST *psensor_inst = &psensor->inst;

	MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT image_window;
	MSDK_SENSOR_CONFIG_STRUCT sensor_config_data;

	if (psensor->sensor_ops &&
	    psensor->sensor_ops->sensor_control &&
	    psensor_inst) {

		IMGSENSOR_PROFILE_INIT(&psensor_inst->profile_time);

		imgsensor_mutex_lock(psensor_inst);

		//psensor_func->psensor_inst = psensor_inst;
		//psensor_func->ScenarioId = ScenarioId;

#if defined(CONFIG_MTK_CAM_SECURE_I2C)
	PK_INFO("%s secure state %d", __func__,
		(int)(&gimgsensor)->imgsensor_sec_flag);
	if ((&gimgsensor)->imgsensor_sec_flag) {
		c_params.param0 = (void *)scenario_id;
		c_params.param1 = (void *)&image_window;
		c_params.param2 = (void *)&sensor_config_data;
		ret = imgsensor_ca_invoke_command(
			IMGSENSOR_TEE_CMD_CONTROL, c_params, &ret_sec);
	} else {
#endif
		ret = psensor->sensor_ops->sensor_control(psensor->kit_params,
			scenario_id, &image_window, &sensor_config_data);
#if defined(CONFIG_MTK_CAM_SECURE_I2C)
	}
#endif
		if (ret != ERROR_NONE)
			pr_err("[%s]\n", __func__);

		imgsensor_mutex_unlock(psensor_inst);

		IMGSENSOR_PROFILE(&psensor_inst->profile_time, "SensorControl");
	}

	return ret;
}

uint32 camkit_sensor_feature_control(
		struct IMGSENSOR_SENSOR *psensor,
		MSDK_SENSOR_FEATURE_ENUM feature_id,
		MUINT8 *feature_param,
		MUINT32 *feature_param_len)
{
	MUINT32 ret = ERROR_NONE;
#if defined(CONFIG_MTK_CAM_SECURE_I2C)
	MINT32 ret_sec = ERROR_NONE;
	struct command_params c_params;
#endif
	struct IMGSENSOR_SENSOR_INST  *psensor_inst = &psensor->inst;

	if ((&gimgsensor)->camkit_enabled && psensor->sensor_ops &&
		psensor->sensor_ops->sensor_feature_control) {
		imgsensor_mutex_lock(psensor_inst);

#if defined(CONFIG_MTK_CAM_SECURE_I2C)
		PK_INFO("%s secure state %d", __func__,
			(int)(&gimgsensor)->imgsensor_sec_flag);
		if ((&gimgsensor)->imgsensor_sec_flag) {
			c_params.param0 = (void *)feature_id;
			c_params.param1 = (void *)feature_param;
			c_params.param2 = (void *)feature_param_len;
			ret = imgsensor_ca_invoke_command(
				IMGSENSOR_TEE_CMD_FEATURE_CONTROL, c_params, &ret_sec);
		} else {
#endif
			ret = psensor->sensor_ops->sensor_feature_control(
					psensor->kit_params, feature_id,
					feature_param, feature_param_len);
#if defined(CONFIG_MTK_CAM_SECURE_I2C)
		}
#endif
		if (ret != ERROR_NONE)
			pr_err("[%s]\n", __func__);

		imgsensor_mutex_unlock(psensor_inst);
	}

	return ret;
}

int32 camkit_sensor_close(struct IMGSENSOR_SENSOR *psensor)
{
	int32 ret = ERROR_NONE;
#if defined(CONFIG_MTK_CAM_SECURE_I2C)
	int32 ret_sec = ERROR_NONE;
	struct command_params c_params = {0};
#endif
	struct IMGSENSOR *pimgsensor = &gimgsensor;
	struct IMGSENSOR_SENSOR_INST  *psensor_inst = &psensor->inst;

	if (psensor->sensor_ops &&
	    psensor->sensor_ops->sensor_close &&
	    psensor_inst) {

		imgsensor_mutex_lock(psensor_inst);

#ifdef IMGSENSOR_OC_ENABLE
		imgsensor_oc_interrupt(IMGSENSOR_HW_POWER_STATUS_OFF);
#endif

#if defined(CONFIG_MTK_CAM_SECURE_I2C)
		PK_INFO("%s secure state %d", __func__,
			(int)(&gimgsensor)->imgsensor_sec_flag);
		if ((&gimgsensor)->imgsensor_sec_flag) {
			ret = imgsensor_ca_invoke_command(
				IMGSENSOR_TEE_CMD_CLOSE, c_params, &ret_sec);
		} else {
#endif
			ret = psensor->sensor_ops->sensor_close(psensor->kit_params);
#if defined(CONFIG_MTK_CAM_SECURE_I2C)
		}
#endif
		if (ret != ERROR_NONE) {
			pr_err("[%s]", __func__);
		} else {
			camkit_hw_power(&pimgsensor->hw,
				psensor,
				IMGSENSOR_HW_POWER_STATUS_OFF);

			psensor_inst->state = IMGSENSOR_STATE_CLOSE;
		}

		imgsensor_mutex_unlock(psensor_inst);
	}

	return ret;
}

static int camkit_probe_sensor(struct IMGSENSOR_SENSOR *psensor)
{
	int32 ret = ERROR_NONE;
	uint32 err = 0;
	uint32 sensor_id = 0;
	uint32 len = sizeof(uint32);
	struct IMGSENSOR *pimgsensor = &gimgsensor;
	struct IMGSENSOR_SENSOR_INST *psensor_inst = &psensor->inst;

	IMGSENSOR_PROFILE_INIT(&psensor_inst->profile_time);

	ret = camkit_hw_power(&pimgsensor->hw, psensor, IMGSENSOR_HW_POWER_STATUS_ON);

	if (ret != IMGSENSOR_RETURN_SUCCESS)
		return ERROR_SENSOR_CONNECT_FAIL;

	camkit_sensor_feature_control(psensor, SENSOR_FEATURE_CHECK_SENSOR_ID,
		(MUINT8 *)&sensor_id, &len);

	/* not implement this feature ID */
	if (sensor_id == 0 || sensor_id == 0xFFFFFFFF) {
		pr_err("Fail to get sensor ID %x\n", sensor_id);
		err = ERROR_SENSOR_CONNECT_FAIL;
	} else {
		pr_err("Sensor found ID = 0x%x\n", sensor_id);
		err = ERROR_NONE;
	}

	ret = camkit_hw_power(&pimgsensor->hw, psensor, IMGSENSOR_HW_POWER_STATUS_OFF);

	IMGSENSOR_PROFILE(&psensor_inst->profile_time, "CheckIsAlive");

	return err ? -EIO : err;
}

int adopt_camera_hw_probe_sensor(void *pBuf)
{
	uint32 ret;
	static open_flag;

	struct camkit_probe_sensor_params *user_params = NULL;
	struct IMGSENSOR_SENSOR               *psensor = NULL;
	struct IMGSENSOR_SENSOR_INST      *sensor_inst = NULL;
	struct camkit_params          *user_kit_params = NULL;
	struct IMGSENSOR                   *pimgsensor = &gimgsensor;
	struct IMGSENSOR_HW                       *phw = &pimgsensor->hw;

	if (open_flag == 0) {
		pr_err("KKK init hw again\n");
		imgsensor_hw_init(phw);
		open_flag = 1;
	}

	pimgsensor->camkit_enabled = 1;
	pr_info("camkit_enabled = %d\n", pimgsensor->camkit_enabled);

	user_params = (struct camkit_probe_sensor_params *)pBuf;
	if (user_params == NULL) {
		pr_err("[%s] NULL arg\n", __func__);
		return -EFAULT;
	}

	user_kit_params = user_params->kit_params;
	if (user_kit_params == NULL) {
		pr_err("[%s] camkit params from user is NULL\n", __func__);
		return -EFAULT;
	}

	pr_err("sensor index = %d\n", user_params->sensor_idx);
	psensor = camkit_get_sensor(user_params->sensor_idx);
	if (psensor == NULL) {
		pr_err("[%s] NULL psensor\n", __func__);
		return -EFAULT;
	}
	psensor->inst.sensor_idx = user_params->sensor_idx;

	ret = camkit_fill_sensor_params(psensor, user_kit_params);
	if (ret != ERROR_NONE) {
		camkit_free_sensor_params(psensor->kit_params);
		psensor->kit_params = NULL;
		pr_err("[%s] fill sensor parameters failed\n", __func__);
		return -EFAULT;
	}
	sensor_inst = &psensor->inst;
	imgsensor_mutex_init(sensor_inst);
	imgsensor_i2c_init(&sensor_inst->i2c_cfg,
		imgsensor_custom_config[psensor->inst.sensor_idx].i2c_dev);

	imgsensor_i2c_filter_msg(&sensor_inst->i2c_cfg, true);

	if (camkit_probe_sensor(psensor)) {
		if (psensor->kit_params && psensor->kit_params->module_params)
			pr_info("sensor[%d]: %s mismatch\n", psensor->inst.sensor_idx,
				psensor->kit_params->module_params->sensor_name);

		imgsensor_i2c_filter_msg(&sensor_inst->i2c_cfg, false);
		camkit_free_sensor_params(psensor->kit_params);
		psensor->kit_params = NULL;

		return -EFAULT;
	}

	pr_info("probe sensor[%d]: %s successfully\n", psensor->inst.sensor_idx,
		psensor->kit_params->module_params->sensor_name);

	user_params->probe_succeed = 1;

	imgsensor_i2c_filter_msg(&sensor_inst->i2c_cfg, false);

	if (user_params->probe_succeed == 1) {
		if (user_params->sensor_idx == 0) {
			strcpy(mtkcam0_name, psensor->kit_params->module_params->sensor_name);
		} else if(user_params->sensor_idx == 1) {
			strcpy(mtkcam1_name, psensor->kit_params->module_params->sensor_name);
		}
	}

	return ERROR_NONE;
}

static enum IMGSENSOR_RETURN camkit_sensor_power_sequence(
	struct IMGSENSOR_HW             *phw,
	enum   IMGSENSOR_SENSOR_IDX      sensor_idx,
	enum   IMGSENSOR_HW_POWER_STATUS pwr_status,
	struct camkit_hw_power_info_t   *ppower_info)
{
	struct IMGSENSOR_HW_SENSOR_POWER *psensor_pwr = NULL;
	struct camkit_hw_power_info_t    *ppwr_info = ppower_info;
	struct IMGSENSOR_HW_DEVICE       *pdev = NULL;
	enum IMGSENSOR_HW_PIN             pin_type;
	enum IMGSENSOR_HW_PIN_STATE       pin_val;

	if (!phw || !ppower_info) {
		pr_err("%s invalid parameter\n", __func__);
		return IMGSENSOR_RETURN_ERROR;
	}

	psensor_pwr = &phw->sensor_pwr[sensor_idx];
	pin_type = (enum IMGSENSOR_HW_PIN)ppwr_info->pin_type;
	pin_val = (enum IMGSENSOR_HW_PIN_STATE)ppwr_info->pin_val;
	while (pin_type != CAMKIT_HW_PIN_NONE &&
		ppwr_info < ppower_info + CAMKIT_HW_PIN_MAX_NUM) {

		if (pin_type != IMGSENSOR_HW_PIN_UNDEF &&
			psensor_pwr->id[pin_type] != IMGSENSOR_HW_ID_NONE) {
			pdev = phw->pdev[psensor_pwr->id[pin_type]];
			pr_info("sensor_idx = %d, pin=%d, pin_val=%d, hw_id =%d\n",
				sensor_idx, pin_type, pin_val, psensor_pwr->id[pin_type]);

			if (pdev && pdev->set)
				pdev->set(pdev->pinstance, sensor_idx, pin_type, pin_val);

			mdelay(ppwr_info->pin_delay);
		}

		ppwr_info++;
		pin_type = (enum IMGSENSOR_HW_PIN)ppwr_info->pin_type;
		pin_val = (enum IMGSENSOR_HW_PIN_STATE)ppwr_info->pin_val;
	}

	/* wait for power stable */
	if (pwr_status == IMGSENSOR_HW_POWER_STATUS_ON)
		mdelay(5);

	return IMGSENSOR_RETURN_SUCCESS;
}

enum IMGSENSOR_RETURN camkit_hw_power(
		struct IMGSENSOR_HW *phw,
		struct IMGSENSOR_SENSOR *psensor,
		enum IMGSENSOR_HW_POWER_STATUS pwr_status)
{
	struct camkit_hw_power_info_t *power_info = NULL;
	enum IMGSENSOR_SENSOR_IDX sensor_idx;
	struct camkit_sensor_params *sensor_params = NULL;

	if (!phw || !psensor || !(psensor->kit_params) ||
		!(psensor->kit_params->sensor_params))
		return IMGSENSOR_RETURN_ERROR;

	sensor_params = psensor->kit_params->sensor_params;
	sensor_idx = psensor->inst.sensor_idx;
	if (pwr_status == IMGSENSOR_HW_POWER_STATUS_ON)
		power_info = sensor_params->power_info;
	else
		power_info = sensor_params->power_down_info;

	camkit_sensor_power_sequence(
		phw,
		sensor_idx,
		pwr_status,
		power_info);

	return IMGSENSOR_RETURN_SUCCESS;
}

int camkit_driver_init(struct IMGSENSOR *img_sensor)
{
	int i;
	struct sensor_kit_ops *sensor_ops = NULL;

	if (!img_sensor) {
		pr_err("%s invalid parameter\n", __func__);
		return 0;
	}

	for (i = 0; i < IMGSENSOR_SENSOR_IDX_MAX_NUM; i++) {
		sensor_ops = img_sensor->sensor[i].sensor_ops;
		sensor_driver_init(&img_sensor->sensor[i].sensor_ops);
		pr_info("%s set options for camera[%d]\n", __func__, i);
	}

	return 0;
}

