/*
 * camkit_driver.c
 *
 * Copyright (c) 2021-2021 LongCheer Technologies Co., Ltd.
 *
 * image sensor driver
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

#include "camkit_driver.h"
#include <linux/spinlock.h>
#include <linux/printk.h>

#include "kd_imgsensor_errcode.h"
#include "camkit_sensor_i2c.h"

#define PLATFORM_BASE_GAIN 64
#define CAMKIT_PIX_10_BITS 10

#define PFX "[camkit_3a]"
#define DEBUG_SENSOR_KIT 0
#define LOG_DBG(fmt, args...) \
	do { \
		if (DEBUG_SENSOR_KIT) \
			pr_info(PFX "%s %d " \
			fmt, __func__, __LINE__, ##args); \
	} while (0)
#define LOG_INF(fmt, args...) \
	pr_info(PFX "%s %d " fmt, __func__, __LINE__, ##args)
#define LOG_ERR(fmt, args...) \
	pr_err(PFX "%s %d " fmt, __func__, __LINE__, ##args)

extern spinlock_t camkit_lock;

static uint16 real_gain_to_register_gain(
	struct camkit_aec_info_t *aec_info, uint16 real_gain)
{
	struct smia_again_coeff *sima_coeff = NULL;
	uint16 reg_gain = aec_info->reg_gain_1x;
	int32 x;
	int32 y;

	sima_coeff = &(aec_info->smia_coeff);
	if (real_gain < aec_info->min_again)
		real_gain = aec_info->min_again;
	else if (real_gain > aec_info->max_again)
		real_gain = aec_info->max_again;

	x = sima_coeff->c0 * PLATFORM_BASE_GAIN - sima_coeff->c1 * real_gain;
	y = sima_coeff->m1 * real_gain - sima_coeff->m0 * PLATFORM_BASE_GAIN;
	if (y != 0)
		reg_gain = x / y;

	return reg_gain;
}

static uint16 register_gain_to_real_again(
	struct camkit_aec_info_t *aec_info, uint16 reg_gain)
{
	struct smia_again_coeff *sima_coeff = NULL;
	uint16 gain = aec_info->min_again;
	int32 x;
	int32 y;

	sima_coeff = &(aec_info->smia_coeff);
	x = (sima_coeff->m0 * reg_gain + sima_coeff->c0) * PLATFORM_BASE_GAIN;
	y = sima_coeff->m1 * reg_gain + sima_coeff->c1;

	if (y != 0)
		gain = x / y;

	return gain;
}

static uint16 calc_register_dgain(struct camkit_aec_info_t *aec_info,
	uint16 real_gain, uint16 real_again)
{
	uint16 reg_dgain;

	if (real_again != 0 && (real_gain > aec_info->max_again ||
		aec_info->again_type == CAMKIT_AGAIN_OV13855))
		reg_dgain = real_gain * aec_info->dgain_decimator / real_again;
	else
		reg_dgain = aec_info->min_dgain;

	if (reg_dgain < aec_info->min_dgain)
		reg_dgain = aec_info->min_dgain;
	else if  (reg_dgain > aec_info->max_dgain)
		reg_dgain = aec_info->max_dgain;

	LOG_DBG("digital reg gain %d", reg_dgain);

	return reg_dgain;
}

static uint32 camkit_calc_gain(struct camkit_sensor_params *params,
	struct camkit_aec_info_t *aec_info, uint16 platform_gain,
	struct aec_ctrl_cfg *aec_ctrl)
{
	uint16 reg_again = 0;
	uint16 reg_dgain = 0;
	uint16 real_again = 0;

	LOG_DBG("[%s] 3A platform_gain %d", params->sensor_name, platform_gain);

	// transfer real gain to sensor register gain
	reg_again = real_gain_to_register_gain(aec_info, platform_gain);
	LOG_DBG("[%s] sensor reg_again 0x%x", params->sensor_name, reg_again);
	aec_ctrl->again = reg_again;
	aec_ctrl->again_valid = true;

	// in order to calculate digital gain, need to recalculate
	// the analog again for accuracy
	real_again = register_gain_to_real_again(aec_info, reg_again);
	LOG_DBG("[%s] real_again %d", params->sensor_name, real_again);

	reg_dgain = calc_register_dgain(aec_info, platform_gain, real_again);
	LOG_DBG("[%s] sensor reg_dgain 0x%x", params->sensor_name, reg_dgain);
	aec_ctrl->dgain = reg_dgain;
	aec_ctrl->dgain_valid = true;

	return ERROR_NONE;
}

static void camkit_fill_aec_item(struct camkit_aec_op *aec_ops, uint32 val,
	bool item_valid, struct sensor_setting_cfg *setting_cfg,
	uint16 *reg_count)
{
	int i = 0;
	uint16 shift;
	struct camkit_aec_cfg *aec_setting = aec_ops->aec_setting;

	if ((aec_ops == NULL) || (aec_ops->aec_setting == NULL) ||
		(setting_cfg == NULL) || (reg_count == NULL)) {
		LOG_ERR("invalid arguments");
		return;
	}

	if (!item_valid)
		return;

	for (i = 0; i < aec_ops->size; i++) {
		if (*reg_count >= MAX_I2C_REG_NUM) {
			LOG_INF("regs too long, may not be filled");
			return;
		}

		setting_cfg->i2c_setting[*reg_count].addr = aec_setting[i].addr;
		if (aec_setting[i].shift >= 0) {
			shift = (uint16)aec_setting[i].shift;
			setting_cfg->i2c_setting[*reg_count].val =
				(val << shift) & aec_setting[i].max_val;
		} else {
			shift = (uint16)(-aec_setting[i].shift);
			setting_cfg->i2c_setting[*reg_count].val =
				(val >> shift) & aec_setting[i].max_val;
		}
		setting_cfg->i2c_setting[*reg_count].addr_type =
			aec_setting[i].addr_type;
		setting_cfg->i2c_setting[*reg_count].data_type =
			aec_setting[i].data_type;

		LOG_DBG("addr = 0x%x; val = 0x%x",
			aec_setting[i].addr,
			setting_cfg->i2c_setting[*reg_count].val);

		*reg_count = *reg_count + 1;
	}
}

static void camkit_fill_i2c_item(struct camkit_i2c_cfg *i2c_setting,
	uint32 size, struct sensor_setting_cfg *setting_cfg,
	uint16 *reg_count)
{
	uint32 i = 0;

	for (i = 0; i < size; i++) {
		if (*reg_count >= MAX_I2C_REG_NUM) {
			LOG_INF("regs too large, may not be filled");
			return;
		}

		setting_cfg->i2c_setting[*reg_count].addr =
			i2c_setting[i].addr;
		setting_cfg->i2c_setting[*reg_count].val =
			i2c_setting[i].val;
		setting_cfg->i2c_setting[*reg_count].addr_type =
			i2c_setting[i].addr_type;
		setting_cfg->i2c_setting[*reg_count].data_type =
			i2c_setting[i].data_type;
		LOG_DBG("addr = 0x%x; val = 0x%x",
			i2c_setting[i].addr, i2c_setting[i].val);

		*reg_count = *reg_count + 1;
	}
}

static void camkit_fill_aec_array(struct aec_ops_map *aec_map,
	struct aec_ctrl_cfg *aec_ctrl, struct sensor_setting_cfg *setting_cfg)
{
	int16  i;
	uint16 reg_count = 0;
	int32  index = 0;
	struct camkit_i2c_cfg *camkit_i2c_setting = NULL;
	struct camkit_aec_op *aec_op = NULL;
	uint32 camkit_i2c_size;

	LOG_DBG("aec_map.size: %d", aec_map->size);
	for (index = 0; index < aec_map->size; index++) {
		switch (aec_map->aec_ops[index].op_type) {
		case SENSOR_AEC_OP_GROUPON:
		case SENSOR_AEC_OP_GROUPOFF:
			camkit_i2c_setting =
				aec_map->aec_ops[index].i2c_setting;
			camkit_i2c_size = aec_map->aec_ops[index].size;
			camkit_fill_i2c_item(camkit_i2c_setting,
				camkit_i2c_size, setting_cfg, &reg_count);
			break;

		case SENSOR_AEC_OP_VTS:
			aec_op = &aec_map->aec_ops[index];
			camkit_fill_aec_item(aec_op, aec_ctrl->frame_length,
				aec_ctrl->fl_valid, setting_cfg, &reg_count);
			break;

		case SENSOR_AEC_OP_SHIFT:
			aec_op = &aec_map->aec_ops[index];
			camkit_fill_aec_item(aec_op, aec_ctrl->shift,
				aec_ctrl->shift_valid, setting_cfg, &reg_count);
			break;

		case SENSOR_AEC_OP_LC:
			aec_op = &aec_map->aec_ops[index];
			camkit_fill_aec_item(aec_op, aec_ctrl->line_count,
				aec_ctrl->lc_valid, setting_cfg, &reg_count);
			break;

		case SENSOR_AEC_OP_AGAIN:
			aec_op = &aec_map->aec_ops[index];
			camkit_fill_aec_item(aec_op, aec_ctrl->again,
				aec_ctrl->again_valid, setting_cfg, &reg_count);
			break;

		case SENSOR_AEC_OP_DGAIN:
			aec_op = &aec_map->aec_ops[index];
			camkit_fill_aec_item(aec_op, aec_ctrl->dgain,
				aec_ctrl->dgain_valid, setting_cfg, &reg_count);
			break;

		case SENSOR_AEC_OP_CTRL:
			camkit_i2c_setting =
				aec_map->aec_ops[index].i2c_setting;
			camkit_i2c_size = aec_map->aec_ops[index].size;
			camkit_fill_i2c_item(camkit_i2c_setting,
				camkit_i2c_size, setting_cfg, &reg_count);
			break;

		default:
			break;
		}
	}

	for (i = 0; i < reg_count; i++) {
		LOG_DBG("Addr: 0x%X", setting_cfg->i2c_setting[i].addr);
		LOG_DBG("Data: 0x%X", setting_cfg->i2c_setting[i].val);
	}

	setting_cfg->size = reg_count;
	setting_cfg->delay = 0;
}

static uint32 camkit_calc_gc_gain(struct camkit_sensor_params *params,
	struct camkit_aec_info_t *aec_info, uint16 platform_gain,
	struct aec_ctrl_cfg *aec_ctrl)
{
	uint16 reg_dgain = 0;
	uint16 real_again = 0;
	uint16 i;
	uint16 gain;

	struct private_again_info *gc_again_info = &(aec_info->priv_again_info);

	if (gc_again_info->size == 0 ||
		gc_again_info->again_map == NULL) {
		LOG_ERR("gc gain table not configure?");
		return ERROR_NONE;
	}
	if (gc_again_info->again_map[0].again_val <
		aec_info->min_gain) {
		LOG_ERR("gc gain table configured incorrectly");
		return ERROR_NONE;
	}

	LOG_DBG("3A platform_gain %d", platform_gain);
	gain = platform_gain;  // gain = real_gain * 64

	if (gain < aec_info->min_gain)
		gain = aec_info->min_gain;
	else if (gain > aec_info->max_gain)
		gain = aec_info->max_gain;

	// get gain index from gain table
	for (i = 0; i < gc_again_info->size; i++) {
		if (gain < gc_again_info->again_map[i].again_val)
			break;
	}

	if (i == 0) {
		LOG_ERR("min gain configured incorrectly");
		return ERROR_NONE;
	}
	if (i == gc_again_info->size) {
		LOG_ERR("max gain configured incorrectly");
	}

	aec_ctrl->again = i - 1;   // i is the gain index, not reg gain
	aec_ctrl->again_valid = true;
	LOG_DBG("gc sensor gain index %u", i - 1);

	// in order to calculate digital gain, need to recalculate
	// the analog again for accuracy
	real_again = gc_again_info->again_map[i - 1].again_val;
	if (real_again == 0) {
		LOG_ERR("gc gain table not configure?");
		return ERROR_NONE;
	}
	LOG_DBG("real_again %d", real_again);
	reg_dgain = gain * aec_info->dgain_decimator / real_again;
	LOG_DBG("sensor reg_dgain 0x%x", reg_dgain);
	aec_ctrl->dgain = reg_dgain;
	aec_ctrl->dgain_valid = true;

	return ERROR_NONE;
}

static void camkit_fill_gc_gain_array(struct camkit_aec_info_t *aec_info,
	struct aec_ctrl_cfg *aec_ctrl, struct sensor_setting_cfg *setting_cfg)
{
	int16  i;
	uint16 reg_count = 0;
	int32  index = 0;
	struct camkit_i2c_cfg *camkit_i2c_setting = NULL;
	uint32 camkit_i2c_size;
	struct camkit_i2c_reg *again_setting = NULL;
	struct camkit_aec_op *aec_op = NULL;

	uint32 again_idx = aec_ctrl->again;
	struct private_again_info *gc_again_info = &(aec_info->priv_again_info);
	struct aec_ops_map *aec_map = &(aec_info->gain_ops_map);

	if (aec_map->size <= 0 || again_idx >= gc_again_info->size) {
		LOG_ERR("invalid parameters");
		return;
	}

	LOG_DBG("aec_map.size: %d", aec_map->size);
	for (index = 0; index < aec_map->size; index++) {
		switch (aec_map->aec_ops[index].op_type) {
		case SENSOR_AEC_OP_GROUPON:
		case SENSOR_AEC_OP_GROUPOFF:
			camkit_i2c_setting =
				aec_map->aec_ops[index].i2c_setting;
			camkit_i2c_size = aec_map->aec_ops[index].size;
			camkit_fill_i2c_item(camkit_i2c_setting,
				camkit_i2c_size, setting_cfg, &reg_count);
			break;

		case SENSOR_AEC_OP_AGAIN_TABLE:
			again_setting = gc_again_info->again_map[again_idx].setting;
			camkit_i2c_size = gc_again_info->again_map[again_idx].size;
			for (i = 0; i < camkit_i2c_size; i++) {
				if (reg_count >= MAX_AEC_REGS) {
					LOG_INF("out of range, the will not be filled.");
					break;
				}
				setting_cfg->i2c_setting[reg_count].addr =
					again_setting[i].addr;
				setting_cfg->i2c_setting[reg_count].val =
					again_setting[i].data;
				setting_cfg->i2c_setting[reg_count].addr_type =
					gc_again_info->again_map[again_idx].addr_type;
				setting_cfg->i2c_setting[reg_count].data_type =
					gc_again_info->again_map[again_idx].data_type;
				reg_count = reg_count + 1;
			}
			break;

		case SENSOR_AEC_OP_DGAIN:
			aec_op = &aec_map->aec_ops[index];
			camkit_fill_aec_item(aec_op,
				aec_ctrl->dgain, aec_ctrl->dgain_valid,
				setting_cfg, &reg_count);
			break;

		case SENSOR_AEC_OP_CTRL:
			camkit_i2c_setting =
				aec_map->aec_ops[index].i2c_setting;
			camkit_i2c_size = aec_map->aec_ops[index].size;
			camkit_fill_i2c_item(camkit_i2c_setting,
				camkit_i2c_size, setting_cfg, &reg_count);
			break;

		default:
			LOG_INF("unsupported operator type: %u",
				aec_map->aec_ops[index].op_type);
			break;
		}
	}

	for (i = 0; i < reg_count; i++) {
		LOG_DBG("Addr: 0x%x", setting_cfg->i2c_setting[i].addr);
		LOG_DBG("Data: 0x%x", setting_cfg->i2c_setting[i].val);
	}

	setting_cfg->size = reg_count;
	setting_cfg->delay = 0;
}

// for gc sensor, such as: gc2375, gc5035 and gc8034 and so on
static uint16 set_gc_gain(struct camkit_sensor_params *params,
	uint16 gain)
{
	int32 ret = ERROR_NONE;
	struct camkit_aec_info_t *aec_info = NULL;
	struct aec_ops_map *gain_ops_map = NULL;
	struct sensor_setting_cfg setting_cfg;
	struct aec_ctrl_cfg aec_ctrl;

	unsigned long flags;
	uint16 i;

	LOG_DBG("ENTER\n");
	if (!params) {
		LOG_ERR("invalid input parameters\n");
		return ERROR_NONE;
	}

	aec_info = &(params->aec_info);
	gain_ops_map = &(aec_info->gain_ops_map);

	memset(&setting_cfg, 0, sizeof(setting_cfg));
	memset(&aec_ctrl, 0, sizeof(aec_ctrl));

	camkit_calc_gc_gain(params, aec_info, gain, &aec_ctrl);
	camkit_fill_gc_gain_array(aec_info, &aec_ctrl, &setting_cfg);

	// write registers to sensor
	for (i = 0; i < setting_cfg.size; i++) {
		ret = camkit_sensor_i2c_write(&params->sensor_ctrl,
			setting_cfg.i2c_setting[i].addr,
			setting_cfg.i2c_setting[i].val,
			setting_cfg.i2c_setting[i].data_type);
		if (ret < 0) {
			LOG_ERR("write gain failed, gain: %d\n", gain);
			return ERROR_SENSOR_CONNECT_FAIL;
		}
	}

	spin_lock_irqsave(&camkit_lock, flags);
	params->sensor_ctrl.gain = gain;
	spin_unlock_irqrestore(&camkit_lock, flags);

	return ERROR_NONE;
}

uint16 set_gain(struct camkit_sensor_params *params, uint16 gain)
{
	int32 ret = ERROR_NONE;

	struct camkit_aec_info_t *aec_info = NULL;
	struct aec_ops_map *gain_ops_map = NULL;
	struct sensor_setting_cfg setting_cfg;
	struct aec_ctrl_cfg aec_ctrl;

	unsigned long flags;
	uint16 i;

	LOG_DBG("ENTER\n");
	if (!params) {
		LOG_ERR("invalid input parameters\n");
		return ERROR_NONE;
	}

	aec_info = &(params->aec_info);
	if (aec_info->again_type == CAMKIT_AGAIN_GC)
		return set_gc_gain(params, gain);

	gain_ops_map = &(aec_info->gain_ops_map);

	memset(&setting_cfg, 0, sizeof(setting_cfg));
	memset(&aec_ctrl, 0, sizeof(aec_ctrl));

	camkit_calc_gain(params, aec_info, gain, &aec_ctrl);
	camkit_fill_aec_array(gain_ops_map, &aec_ctrl, &setting_cfg);

	// write registers to sensor
	for (i = 0; i < setting_cfg.size; i++) {
		ret = camkit_sensor_i2c_write(&params->sensor_ctrl,
			setting_cfg.i2c_setting[i].addr,
			setting_cfg.i2c_setting[i].val,
			setting_cfg.i2c_setting[i].data_type);
		if (ret < 0) {
			LOG_ERR("write gain failed, gain: %d\n", gain);
			return ERROR_SENSOR_CONNECT_FAIL;
		}
	}

	spin_lock_irqsave(&camkit_lock, flags);
	params->sensor_ctrl.gain = gain;
	spin_unlock_irqrestore(&camkit_lock, flags);

	LOG_DBG("EXIT\n");

	return ERROR_NONE;
}

static void camkit_calc_vblank(struct camkit_aec_info_t *aec_info,
	uint32 *frame_length)
{
	uint32 base_fl = aec_info->base_fl;      // vblank = frame length - base_fl
	uint16 min_vblank = aec_info->min_vblank;
	uint16 max_vblank = aec_info->max_vblank;

	uint32 vblank = *frame_length - base_fl;
	vblank = vblank < min_vblank ? min_vblank : vblank;
	vblank = vblank > max_vblank ? max_vblank : vblank;

	LOG_DBG("base_fl = %d, vblank [%d-%d]: %d\n", base_fl, min_vblank, max_vblank, vblank);
	*frame_length = vblank;
}

void set_dummy(struct camkit_sensor_params *params)
{
	int32 ret = ERROR_NONE;
	struct camkit_aec_info_t *aec_info = NULL;
	struct aec_ops_map *vts_ops_map = NULL;
	struct sensor_setting_cfg setting_cfg;
	struct aec_ctrl_cfg aec_ctrl;

	//unsigned long flags;
	uint16 i;

	LOG_DBG("ENTER\n");
	if (!params) {
		LOG_ERR("invalid input parameters\n");
		return;
	}

	aec_info = &(params->aec_info);
	vts_ops_map = &(aec_info->vts_ops_map);

	memset(&setting_cfg, 0, sizeof(setting_cfg));
	memset(&aec_ctrl, 0, sizeof(aec_ctrl));

	aec_ctrl.frame_length = params->sensor_ctrl.frame_length;
	if (aec_info->vblank_flag)
		camkit_calc_vblank(aec_info, &aec_ctrl.frame_length);
	aec_ctrl.fl_valid = true;

	camkit_fill_aec_array(vts_ops_map, &aec_ctrl, &setting_cfg);

	// write registers to sensor
	for (i = 0; i < setting_cfg.size; i++) {
		ret = camkit_sensor_i2c_write(&params->sensor_ctrl,
			setting_cfg.i2c_setting[i].addr,
			setting_cfg.i2c_setting[i].val,
			setting_cfg.i2c_setting[i].data_type);
		if (ret < 0) {
			LOG_ERR("write fl failed, frame_length: 0x%x\n",
				params->sensor_ctrl.frame_length);
			return;
		}
	}

	LOG_DBG("EXIT\n");
}

void set_max_framerate(struct camkit_sensor_params *params,
	uint16 framerate, bool min_framelength_en)
{
	uint32 frame_length;
	struct camkit_sensor_ctrl_t *sensor_ctrl = NULL;

	LOG_DBG("ENTER\n");
	if (!params) {
		LOG_ERR("invalid input parameters\n");
		return;
	}

	sensor_ctrl = &(params->sensor_ctrl);
	if (!framerate || !sensor_ctrl->line_length) {
		LOG_ERR("Invalid params. framerate=%u, line_length=%u\n",
			framerate, sensor_ctrl->line_length);
		return;
	}

	frame_length = sensor_ctrl->pclk / framerate *
		CAMKIT_PIX_10_BITS / sensor_ctrl->line_length;

	spin_lock(&camkit_lock);
	sensor_ctrl->frame_length = frame_length;
	if (frame_length < sensor_ctrl->min_frame_length)
		sensor_ctrl->frame_length = sensor_ctrl->min_frame_length;
	if (frame_length > params->aec_info.max_frame_length)
		sensor_ctrl->frame_length = params->aec_info.max_frame_length;
	if (min_framelength_en)
		sensor_ctrl->min_frame_length = sensor_ctrl->frame_length;
	spin_unlock(&camkit_lock);

	set_dummy(params);
}

void set_shutter(struct camkit_sensor_params *params, uint32 shutter)
{
	uint32 frame_length;
	struct camkit_sensor_ctrl_t *sensor_ctrl = NULL;

	LOG_DBG("ENTER\n");

	if (!params) {
		LOG_ERR("invalid input parameters\n");
		return;
	}

	sensor_ctrl = &(params->sensor_ctrl);
	spin_lock(&camkit_lock);
	if (shutter > sensor_ctrl->min_frame_length -
		params->aec_info.vts_offset)
		sensor_ctrl->frame_length =
			shutter + params->aec_info.vts_offset;
	else
		sensor_ctrl->frame_length =
			sensor_ctrl->min_frame_length;

	frame_length = sensor_ctrl->frame_length;
	spin_unlock(&camkit_lock);

	set_shutter_frame_length(params, shutter, frame_length);
	LOG_DBG("EXIT\n");
}

void set_shutter_frame_length(struct camkit_sensor_params *params,
	uint32 shutter, uint32 frame_length)
{
	int32 ret = ERROR_NONE;

	struct camkit_aec_info_t *aec_info = NULL;
	struct aec_ops_map *expo_ops_map = NULL;
	struct sensor_setting_cfg setting_cfg;
	struct aec_ctrl_cfg aec_ctrl;
	uint16 realtime_fps = 0;
	struct camkit_sensor_ctrl_t *sensor_ctrl = NULL;
	unsigned long flags;
	uint16 i;
	uint16 shift = 0;

	LOG_DBG("ENTER\n");

	if (!params) {
		LOG_ERR("invalid input parameters\n");
		return;
	}

	sensor_ctrl = &(params->sensor_ctrl);
	aec_info = &(params->aec_info);
	expo_ops_map = &(aec_info->expo_ops_map);

	memset(&setting_cfg, 0, sizeof(setting_cfg));
	memset(&aec_ctrl, 0, sizeof(aec_ctrl));

	/*
	 * 1. confirm the shutter whether it is
	 * in the min and max line count threshold
	 */
	if (shutter < aec_info->min_linecount)
		shutter = aec_info->min_linecount;

	if (aec_info->lexpo_type == SENSOR_LONG_EXPO_SONY) {
		while (shutter > aec_info->max_linecount) {
			shift++;
			if (shift > aec_info->max_shift) {
				shutter = aec_info->max_linecount;
				shift = aec_info->max_shift;
				break;
			}
			shutter >>= 1;
		}
	} else {
		if (shutter > aec_info->max_linecount)
			shutter = aec_info->max_linecount;
	}

	aec_ctrl.line_count = shutter;
	aec_ctrl.lc_valid = true;

	aec_ctrl.shift = shift;
	aec_ctrl.shift_valid = true;

	/* 2. calc frame length depend on shutter */
	LOG_DBG("[%s] lock\n", params->sensor_name);
	spin_lock(&camkit_lock);
	LOG_DBG("[%s] lock in\n", params->sensor_name);
	if (frame_length < sensor_ctrl->min_frame_length)
		frame_length = sensor_ctrl->min_frame_length;
	if (shutter > frame_length - aec_info->vts_offset ||
           shift > 0)
		sensor_ctrl->frame_length = shutter + aec_info->vts_offset;
	else
		sensor_ctrl->frame_length = frame_length;

	if (sensor_ctrl->frame_length > aec_info->max_frame_length)
		sensor_ctrl->frame_length = aec_info->max_frame_length;

	/* 3. calc and update framelength to abandon the flicker issue */
	if (sensor_ctrl->autoflicker_en &&
		sensor_ctrl->line_length != 0 &&
		sensor_ctrl->frame_length != 0) {
		realtime_fps = sensor_ctrl->pclk / sensor_ctrl->line_length *
			CAMKIT_PIX_10_BITS / sensor_ctrl->frame_length;

		/*
		 * 1) calc fps between 298~305, real fps set to 298;
		 * 2) calc fps between 147~150, real fps set to 146
		 */
		if (realtime_fps >= 298 && realtime_fps <= 305) {
			realtime_fps = 298;
			sensor_ctrl->frame_length = sensor_ctrl->pclk /
				realtime_fps * CAMKIT_PIX_10_BITS /
				sensor_ctrl->line_length;
		} else if (realtime_fps >= 147 && realtime_fps <= 150) {
			realtime_fps = 146;
			sensor_ctrl->frame_length = sensor_ctrl->pclk /
				realtime_fps * CAMKIT_PIX_10_BITS /
				sensor_ctrl->line_length;
		}
	}

	aec_ctrl.frame_length = sensor_ctrl->frame_length;
	if (aec_info->vblank_flag)
		camkit_calc_vblank(aec_info, &aec_ctrl.frame_length);
	aec_ctrl.fl_valid = true;

	LOG_DBG("[%s] lock down\n", params->sensor_name);
	spin_unlock(&camkit_lock);
	LOG_DBG("[%s] unlock\n", params->sensor_name);

	LOG_DBG("[%s] shutter = 0x%x, fl = 0x%x\n", params->sensor_name,
		shutter, aec_ctrl.frame_length);

	camkit_fill_aec_array(expo_ops_map, &aec_ctrl, &setting_cfg);

	// write registers to sensor
	for (i = 0; i < setting_cfg.size; i++) {
		ret = camkit_sensor_i2c_write(sensor_ctrl,
			setting_cfg.i2c_setting[i].addr,
			setting_cfg.i2c_setting[i].val,
			setting_cfg.i2c_setting[i].data_type);
		if (ret < 0) {
			LOG_ERR("write shutter failed, shutter: %d\n", shutter);
			return;
		}
	}

	spin_lock_irqsave(&camkit_lock, flags);
	sensor_ctrl->shutter = shutter;
	spin_unlock_irqrestore(&camkit_lock, flags);

	LOG_DBG("EXIT\n");
}
