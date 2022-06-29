/*
 * camkit_driver.c
 *
 * Copyright (c) 2021-2021 LongCheer Technologies Co., Ltd.
 *
 * camkit interface
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
#include <linux/delay.h>
#include <linux/slab.h>

#include "kd_imgsensor_errcode.h"
#include "camkit_sensor_i2c.h"

#define RETRY_TIMES 2
#define PIX_10_BITS 10

#define PFX "[sensorkit]"
#define DEBUG_SENSOR_KIT 0
#define LOG_DBG(fmt, args...) \
	do { \
		if (DEBUG_SENSOR_KIT) \
			pr_info(PFX "%s %d " \
			fmt, __func__, __LINE__, ##args); \
	} while (0)
#define LOG_INF(fmt, args...) pr_info(PFX "%s %d " fmt, __func__, __LINE__, ##args)
#define LOG_ERR(fmt, args...) pr_err(PFX "%s %d " fmt, __func__, __LINE__, ##args)

DEFINE_SPINLOCK(camkit_lock);

static int32 read_data_from_eeprom(uint16 i2c_addr,
	uint16 addr, uint16 len, uint8 *data)
{
	uint16 i;
	uint16 offset = addr;
	int32 ret;

	for (i = 0; i < len; i++) {
		ret = camkit_read_eeprom(i2c_addr, offset, &data[i]);
		if (ret < 0) {
			LOG_ERR("read fail, addr: 0x%x", offset);
			return -EFAULT;
		}

		offset++;
	}

	return ERROR_NONE;
}

static int32 read_qsc_calib_data_from_eeprom(
	struct camkit_module_params *module_params,
	struct camkit_sensor_params *sensor_params)
{
	struct camkit_sensor_correction_t *correction_info = NULL;
	int32 ret;

	correction_info = &(sensor_params->correction_info);
	if (!correction_info->qsc_apply || !correction_info->eeprom_qsc_addr ||
		!correction_info->qsc_len) {
		LOG_INF("need not read qsc data from eeprom");
		return ERROR_NONE;
	}
	if (correction_info->qsc_read_flag) {
		LOG_INF("qsc data had read from eeprom");
		return ERROR_NONE;
	}

	correction_info->qsc_buf = kzalloc(correction_info->qsc_len, GFP_KERNEL);
	if (!correction_info->qsc_buf) {
		LOG_ERR("memory not enough");
		return -ENOMEM;
	}
	ret = read_data_from_eeprom(module_params->eeprom_i2c_addr,
		correction_info->eeprom_qsc_addr, correction_info->qsc_len,
		correction_info->qsc_buf);
	if (ret < 0) {
		LOG_ERR("read qsc data from eeprom fail");
		kfree(correction_info->qsc_buf);
		correction_info->qsc_buf = NULL;
		return -EFAULT;
	}
	correction_info->qsc_read_flag = 1;

	return ERROR_NONE;
}

uint16 pdc_addr[][2] = {
	// { start_addr, end_addr }
	{ 0x00C, 0x01C }, { 0x03F, 0x04F }, { 0x072, 0x082 },
	{ 0x0A5, 0x0B5 }, { 0x0D8, 0x0E8 }, { 0x0E9, 0x0F9 },
	{ 0x11C, 0x12C }, { 0x14F, 0x15F }, { 0x182, 0x192 },
	{ 0x1B5, 0x1C5 }, { 0x1C6, 0x1D6 }, { 0x1F9, 0x209 },
	{ 0x22C, 0x23C }, { 0x25F, 0x26F }, { 0x292, 0x2A2 },
	{ 0x2A3, 0x2B3 }, { 0x2D6, 0x2E6 }, { 0x309, 0x319 },
	{ 0x33C, 0x34C }, { 0x36F, 0x37F }, { 0x380, 0x59B },
};

static void extract_valid_data_for_ov48b(uint8 *buf)
{
	uint32 index = 0;
	uint8 *pdc_buf = buf;
	uint32 size = sizeof(pdc_addr) / sizeof(pdc_addr[0]);
	uint32 i;
	uint32 j;

	for (i = 0; i < size - 1; i++) {
		for (j = pdc_addr[i][1]; j >= pdc_addr[i][0]; j = j - 2)
			pdc_buf[index++] = buf[j];
	}

	for (i = pdc_addr[size - 1][0]; i <= pdc_addr[size - 1][1]; i++)
		pdc_buf[index++] = buf[i];
}

static int32 read_spc_calib_data_from_eeprom(
	struct camkit_module_params *module_params,
	struct camkit_sensor_params *sensor_params)
{
	struct camkit_sensor_correction_t *correction_info = NULL;
	int32 ret;

	correction_info = &(sensor_params->correction_info);
	if (!correction_info->spc_apply || !correction_info->eeprom_pdaf_addr ||
		!correction_info->pdaf_len) {
		LOG_INF("need not read spc data from eeprom");
		return ERROR_NONE;
	}
	if (correction_info->spc_read_flag) {
		LOG_INF("spc data had read from eeprom");
		return ERROR_NONE;
	}

	correction_info->pdaf_buf = kzalloc(correction_info->pdaf_len, GFP_KERNEL);
	if (!correction_info->pdaf_buf) {
		LOG_ERR("memory not enough");
		return -ENOMEM;
	}
	ret = read_data_from_eeprom(module_params->eeprom_i2c_addr,
		correction_info->eeprom_pdaf_addr, correction_info->pdaf_len,
		correction_info->pdaf_buf);
	if (ret < 0) {
		LOG_ERR("read spc data from eeprom fail");
		kfree(correction_info->pdaf_buf);
		correction_info->pdaf_buf = NULL;
		return -EFAULT;
	}
	correction_info->spc_read_flag = 1;

	if (correction_info->spc_type == PDAF_SPC_PDC)
		extract_valid_data_for_ov48b(correction_info->pdaf_buf);

	return ERROR_NONE;
}

static void read_calib_data_from_eeprom(struct camkit_params *kit_params)
{
	(void)read_qsc_calib_data_from_eeprom(kit_params->module_params,
		kit_params->sensor_params);
	(void)read_spc_calib_data_from_eeprom(kit_params->module_params,
		kit_params->sensor_params);
}

static int32 write_lrc_data_to_sensor(struct camkit_params *kit_params)
{
	struct camkit_sensor_correction_t *correction_info = NULL;
	struct camkit_sensor_params *sensor_params = NULL;
	int32 ret;

	LOG_INF("ENTER");

	if (!kit_params || !kit_params->sensor_params) {
		LOG_ERR("invalid parameters");
		return -EFAULT;
	}
	sensor_params = kit_params->sensor_params;

	correction_info = &(sensor_params->correction_info);
	if (correction_info->pdaf_len != (correction_info->lsc_addr_len +
		correction_info->rsc_addr_len)) {
		LOG_ERR("pdaf correction info configure error, please check");
		return -EFAULT;
	}

	ret = camkit_i2c_write_block(sensor_params->sensor_ctrl.i2c_write_id,
		sensor_params->sensor_ctrl.i2c_speed,
		correction_info->lsc_start_addr,
		sensor_params->sensor_ctrl.addr_type,
		correction_info->pdaf_buf, correction_info->lsc_addr_len);
	if (ret < 0) {
		LOG_ERR("write lsc fail");
		return -EFAULT;
	}

	ret = camkit_i2c_write_block(sensor_params->sensor_ctrl.i2c_write_id,
		sensor_params->sensor_ctrl.i2c_speed,
		correction_info->rsc_start_addr,
		sensor_params->sensor_ctrl.addr_type,
		correction_info->pdaf_buf + correction_info->lsc_addr_len,
		correction_info->rsc_addr_len);
	if (ret < 0) {
		LOG_ERR("write rsc fail");
		return -EFAULT;
	}

	LOG_INF("EXIT");

	return ERROR_NONE;
}

static int32 write_pdc_data_to_sensor(struct camkit_params *kit_params)
{
	struct camkit_sensor_correction_t *correction_info = NULL;
	struct camkit_sensor_params *sensor_params = NULL;
	int32 ret;

	LOG_INF("ENTER");

	if (!kit_params || !kit_params->sensor_params) {
		LOG_ERR("invalid parameters");
		return -EFAULT;
	}
	sensor_params = kit_params->sensor_params;

	correction_info = &(sensor_params->correction_info);
	if (correction_info->pdaf_len < correction_info->pdc_len) {
		LOG_ERR("pdaf correction info configure error, please check");
		return -EFAULT;
	}

	ret = camkit_i2c_write_block(sensor_params->sensor_ctrl.i2c_write_id,
		sensor_params->sensor_ctrl.i2c_speed,
		correction_info->pdc_addr,
		sensor_params->sensor_ctrl.addr_type,
		correction_info->pdaf_buf, correction_info->pdc_len);
	if (ret < 0) {
		LOG_ERR("write lsc fail");
		return -EFAULT;
	}

	LOG_INF("EXIT");

	return ERROR_NONE;
}

static int32 write_qsc_data_to_sensor(struct camkit_params *kit_params)
{
	struct camkit_sensor_correction_t *correction_info = NULL;
	struct camkit_module_params *module_params = NULL;
	struct camkit_sensor_params *sensor_params = NULL;
	int32 ret;

	LOG_INF("ENTER");

	if (!kit_params || !kit_params->sensor_params ||
		!kit_params->module_params) {
		LOG_ERR("invalid parameters");
		return -EFAULT;
	}
	sensor_params = kit_params->sensor_params;
	module_params = kit_params->module_params;

	correction_info = &(sensor_params->correction_info);

	ret = camkit_i2c_write_block(sensor_params->sensor_ctrl.i2c_write_id,
		sensor_params->sensor_ctrl.i2c_speed,
		correction_info->sensor_qsc_addr,
		sensor_params->sensor_ctrl.addr_type,
		correction_info->qsc_buf, correction_info->qsc_len);
	if (ret < 0) {
		LOG_ERR("write lsc fail");
		return -EFAULT;
	}

	LOG_INF("EXIT");

	return ERROR_NONE;
}

static uint32 sensor_init(struct camkit_params *kit_params)
{
	int32 rc;
	struct camkit_sensor_params *params = NULL;

	LOG_INF("ENTER\n");

	if (!kit_params || !kit_params->sensor_params ||
		!kit_params->module_params) {
		LOG_ERR("invalid parameter");
		return ERROR_INVALID_PARA;
	}

	params = kit_params->sensor_params;
	rc = camkit_sensor_write_setting(&params->sensor_ctrl,
		&params->sensor_info.init_setting);
	if (rc < 0) {
		LOG_ERR("Failed\n");
		return ERROR_DRIVER_INIT_FAIL;
	}

	if (params->sensor_info.init_burst_setting.setting != NULL &&
		params->sensor_info.init_burst_setting.size > 0) {
		rc = camkit_sensor_write_block(&params->sensor_ctrl,
			params->sensor_info.init_burst_setting.setting,
			params->sensor_info.init_burst_setting.size,
			params->sensor_info.init_burst_setting.data_type);
		if (rc < 0)
			LOG_ERR("something abnormal\n");
	}

	LOG_INF("qsc_apply:%d", params->correction_info.qsc_apply);
	if (params->correction_info.qsc_apply)
		(void)write_qsc_data_to_sensor(kit_params);

	LOG_INF("EXIT\n");

	return ERROR_NONE;
}

static uint32 set_test_pattern_mode(struct camkit_sensor_params *params,
	bool enable)
{
	spin_lock(&camkit_lock);
	params->sensor_ctrl.test_pattern = enable;
	spin_unlock(&camkit_lock);
	return ERROR_NONE;
}

static uint32 sensor_dump_reg(struct camkit_sensor_params *params)
{
	int32 rc;

	LOG_INF("ENTER\n");
	rc = camkit_sensor_i2c_process(&params->sensor_ctrl,
		&params->sensor_info.dump_info);
	if (rc < 0)
		LOG_ERR("Failed\n");
	LOG_INF("EXIT\n");
	return ERROR_NONE;
}

static uint32 match_sensor_id(struct camkit_params *params)
{
	int32 rc;
	uint8 i = 0;
	uint8 retry = RETRY_TIMES;
	uint8 size = 0;
	uint16 sensor_id = 0;
	struct camkit_sensor_params *sensor_params = params->sensor_params;
	struct camkit_module_params *module_params = params->module_params;
	struct camkit_sensor_info_t *sensor_info = &sensor_params->sensor_info;
	uint16 expect_id = sensor_info->sensor_id;
	camkit_i2c_data_type data_type = sensor_info->sensor_id_dt;
	struct camkit_i2c_reg_setting *setting = &(sensor_info->id_init_setting);

	spin_lock(&camkit_lock);
	/* init i2c config */
	sensor_params->sensor_ctrl.i2c_speed = sensor_info->i2c_speed;
	sensor_params->sensor_ctrl.addr_type = sensor_info->addr_type;
	spin_unlock(&camkit_lock);

	size = CAMKIT_ARRAY_SIZE(sensor_info->i2c_addr_table);
	LOG_INF("sensor i2c addr num = %u", size);

	while ((i < size) && (sensor_info->i2c_addr_table[i] != 0xff)) {
		spin_lock(&camkit_lock);
		sensor_params->sensor_ctrl.i2c_write_id = sensor_info->i2c_addr_table[i];
		spin_unlock(&camkit_lock);
		do {
			// workaround, TODO
			LOG_INF("sensor name: %s\n", module_params->sensor_name);

			if (setting->setting && setting->size > 0)
				(void)camkit_sensor_write_setting(&sensor_params->sensor_ctrl,
					setting);

			if (!data_type)
				rc = camkit_sensor_i2c_read(&sensor_params->sensor_ctrl,
					sensor_info->sensor_id_reg,
					&sensor_id, CAMKIT_I2C_WORD_DATA);
			else
				rc = camkit_sensor_i2c_read(&sensor_params->sensor_ctrl,
					sensor_info->sensor_id_reg,
					&sensor_id, data_type);
			if (rc == 0 && sensor_id == expect_id) {
				LOG_INF("sensor id: 0x%x matched", sensor_id);
				return ERROR_NONE;
			}
			retry--;
		} while (retry > 0);
		i++;
		retry = RETRY_TIMES;
	}

	LOG_INF("sensor id mismatch, expect:0x%x, real:0x%x",
		expect_id, sensor_id);

	return ERROR_SENSOR_CONNECT_FAIL;
}

static uint32 match_module_id(struct camkit_module_params *params)
{
	int32 rc;
	uint16 module_code = 0;
	uint16 expect_code = params->module_code;
	uint8 i2c_addr = params->eeprom_i2c_addr;
	uint16 module_addr = params->module_code_addr;
	uint16 lens_addr = params->lens_type_addr;
	uint16 expect_lens = params->lens_type;
	uint16 lens_type = 0;
	uint8 retry = RETRY_TIMES;

	if (params->skip_module_id) {
		LOG_INF("not to match module code");
		return ERROR_NONE;
	}

	do {
		rc = camkit_i2c_read(i2c_addr, module_addr, params->addr_type,
			&module_code, params->data_type);
		if (rc == 0 && module_code == expect_code) {
			LOG_INF("module code: 0x%x matched", module_code);
			if (params->lens_type_addr) {
				rc = camkit_i2c_read(i2c_addr, lens_addr, params->addr_type,
					&lens_type, params->data_type);
				if (rc == 0 && lens_type == expect_lens) {
					LOG_INF("lens type: 0x%x matched", lens_type);
					return ERROR_NONE;
				}
			} else {
				return ERROR_NONE;
			}
		}
		retry--;
	} while (retry > 0);

	LOG_ERR("module code, expect:0x%x, real:0x%x",
		expect_code, module_code);
	LOG_ERR("lens type, expect:0x%x, real:0x%x",
		expect_lens, lens_type);

	return ERROR_SENSOR_CONNECT_FAIL;
}

static uint32 get_match_id(struct camkit_params *kit_params,
	uint32 *match_id)
{
	uint32 rc;

	if (!kit_params || !kit_params->sensor_params ||
		!kit_params->module_params) {
		LOG_ERR("invalid parameter");
		return ERROR_INVALID_PARA;
	}

	rc = match_sensor_id(kit_params);
	if (rc != ERROR_NONE) {
		*match_id = 0xFFFFFFFF;
		return rc;
	}

	rc = match_module_id(kit_params->module_params);
	if (rc != ERROR_NONE) {
		*match_id = 0xFFFFFFFF;
		return rc;
	}

	if (kit_params->sensor_params->sensor_info.need_correction)
		read_calib_data_from_eeprom(kit_params);

	*match_id = kit_params->module_params->match_id;
	LOG_INF("match id successfully, sensor id: 0x%x, \
		module code: 0x%x, match id: 0x%x\n",
		kit_params->sensor_params->sensor_info.sensor_id,
		kit_params->module_params->module_code,
		kit_params->module_params->match_id);

	return ERROR_NONE;
}

static uint32 open(struct camkit_params *params)
{
	uint32 sensor_id = 0;
	uint32 rc;
	struct camkit_sensor_params *sensor_params = NULL;

	LOG_INF("ENTER\n");

	if (!params || !params->sensor_params || !params->module_params) {
		LOG_ERR("invalid parameter");
		return ERROR_INVALID_PARA;
	}

	sensor_params = params->sensor_params;
	rc = get_match_id(params, &sensor_id);
	if (rc != ERROR_NONE) {
		LOG_ERR("probe sensor failed\n");
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	LOG_DBG("sensor probe successfully. sensor_id = 0x%x\n", sensor_id);

	/* initail sequence write in  */
	rc = sensor_init(params);
	if (rc != ERROR_NONE) {
		LOG_ERR("init failed\n");
		return ERROR_DRIVER_INIT_FAIL;
	}

	spin_lock(&camkit_lock);
	sensor_params->sensor_ctrl.autoflicker_en = FALSE;
	sensor_params->sensor_ctrl.sensor_mode = CAMKIT_MODE_INIT;
	sensor_params->sensor_ctrl.pclk =
		sensor_params->sensor_info.pre.pclk;
	sensor_params->sensor_ctrl.frame_length =
		sensor_params->sensor_info.pre.framelength;
	sensor_params->sensor_ctrl.line_length =
		sensor_params->sensor_info.pre.linelength;
	sensor_params->sensor_ctrl.min_frame_length =
		sensor_params->sensor_info.pre.framelength;
	sensor_params->sensor_ctrl.dummy_pixel = 0;
	sensor_params->sensor_ctrl.dummy_line = 0;
	sensor_params->sensor_ctrl.ihdr_en = FALSE;
	sensor_params->sensor_ctrl.test_pattern = FALSE;
	sensor_params->sensor_ctrl.current_fps =
		sensor_params->sensor_info.pre.max_framerate;
	spin_unlock(&camkit_lock);

	LOG_INF("EXIT\n");

	return ERROR_NONE;
}

static uint32 close(struct camkit_params *params)
{
	LOG_INF("E\n");
	/* No Need to implement this function */
	return ERROR_NONE;
}

static uint32 set_setting_by_scenario(struct camkit_sensor_params *params,
	enum MSDK_SCENARIO_ID_ENUM scenario_id,
	MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	int32 rc;
	uint8 mode;
	struct camkit_mode_info *mode_info = NULL;
	struct camkit_i2c_reg_setting *setting = NULL;

	LOG_DBG("ENTER\n");

	spin_lock(&camkit_lock);
	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		mode = CAMKIT_MODE_PREVIEW;
		mode_info = &(params->sensor_info.pre);
		setting = &(params->sensor_info.pre_setting);
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		mode = CAMKIT_MODE_CAPTURE;
		mode_info = &(params->sensor_info.cap);
		setting = &(params->sensor_info.cap_setting);
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		mode = CAMKIT_MODE_VIDEO;
		mode_info = &(params->sensor_info.normal_video);
		setting = &(params->sensor_info.normal_video_setting);
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		mode = CAMKIT_MODE_HIGH_SPEED_VIDEO;
		mode_info = &(params->sensor_info.hs_video);
		setting = &(params->sensor_info.hs_video_setting);
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		mode = CAMKIT_MODE_SLIM_VIDEO;
		mode_info = &(params->sensor_info.slim_video);
		setting = &(params->sensor_info.slim_setting);
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		mode = CAMKIT_MODE_CUSTOM1;
		mode_info = &(params->sensor_info.custom1);
		setting = &(params->sensor_info.custom1_setting);
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		mode = CAMKIT_MODE_CUSTOM2;
		mode_info = &(params->sensor_info.custom2);
		setting = &(params->sensor_info.custom2_setting);
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		mode = CAMKIT_MODE_CUSTOM3;
		mode_info = &(params->sensor_info.custom3);
		setting = &(params->sensor_info.custom3_setting);
		break;
	case MSDK_SCENARIO_ID_CUSTOM4:
		mode = CAMKIT_MODE_CUSTOM4;
		mode_info = &(params->sensor_info.custom4);
		setting = &(params->sensor_info.custom4_setting);
		break;
	case MSDK_SCENARIO_ID_CUSTOM5:
		mode = CAMKIT_MODE_CUSTOM5;
		mode_info = &(params->sensor_info.custom5);
		setting = &(params->sensor_info.custom5_setting);
		break;
	default:
		LOG_ERR("Error scenario_id setting. scenario_id:%d\n",
			scenario_id);
		mode = CAMKIT_MODE_PREVIEW;
		mode_info = &(params->sensor_info.pre);
		setting = &(params->sensor_info.pre_setting);
		// return ERROR_INVALID_SCENARIO_ID;
	}

	params->sensor_ctrl.sensor_mode = mode;
	params->sensor_ctrl.pclk = mode_info->pclk;
	params->sensor_ctrl.line_length = mode_info->linelength;
	params->sensor_ctrl.frame_length = mode_info->framelength;
	params->sensor_ctrl.min_frame_length = mode_info->framelength;
	params->sensor_ctrl.autoflicker_en = FALSE;
	spin_unlock(&camkit_lock);

	LOG_INF("Enter scenario_id:%d\n", scenario_id);
	rc = camkit_sensor_write_setting(&params->sensor_ctrl, setting);
	if (rc < 0) {
		LOG_ERR("Write sensorMode[%u] settings Failed\n", mode);
		return ERROR_DRIVER_INIT_FAIL;
	}

	LOG_DBG("EXIT\n");
	return ERROR_NONE;
}

static uint32 get_resolution(struct camkit_params *kit_params,
	MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	struct camkit_sensor_params *params = NULL;

	if (!kit_params || !kit_params->sensor_params ||
		!sensor_resolution) {
		LOG_ERR("NULL ptr. params:%pK, sensor_resolution:%pK\n",
			kit_params, sensor_resolution);
		return ERROR_NONE;
	}

	params = kit_params->sensor_params;
	if (sensor_resolution) {
		sensor_resolution->SensorFullWidth =
			params->sensor_info.cap.grabwindow_width;
		sensor_resolution->SensorFullHeight =
			params->sensor_info.cap.grabwindow_height;

		sensor_resolution->SensorPreviewWidth =
			params->sensor_info.pre.grabwindow_width;
		sensor_resolution->SensorPreviewHeight =
			params->sensor_info.pre.grabwindow_height;

		sensor_resolution->SensorVideoWidth =
			params->sensor_info.normal_video.grabwindow_width;
		sensor_resolution->SensorVideoHeight =
			params->sensor_info.normal_video.grabwindow_height;

		sensor_resolution->SensorHighSpeedVideoWidth =
			params->sensor_info.hs_video.grabwindow_width;
		sensor_resolution->SensorHighSpeedVideoHeight =
			params->sensor_info.hs_video.grabwindow_height;

		sensor_resolution->SensorSlimVideoWidth =
			params->sensor_info.slim_video.grabwindow_width;
		sensor_resolution->SensorSlimVideoHeight =
			params->sensor_info.slim_video.grabwindow_height;

		sensor_resolution->SensorCustom1Width =
			params->sensor_info.custom1.grabwindow_width;
		sensor_resolution->SensorCustom1Height =
			params->sensor_info.custom1.grabwindow_height;

		sensor_resolution->SensorCustom2Width =
			params->sensor_info.custom2.grabwindow_width;
		sensor_resolution->SensorCustom2Height =
			params->sensor_info.custom2.grabwindow_height;

		sensor_resolution->SensorCustom3Width =
			params->sensor_info.custom3.grabwindow_width;
		sensor_resolution->SensorCustom3Height =
			params->sensor_info.custom3.grabwindow_height;

		sensor_resolution->SensorCustom4Width =
			params->sensor_info.custom4.grabwindow_width;
		sensor_resolution->SensorCustom4Height =
			params->sensor_info.custom4.grabwindow_height;

		sensor_resolution->SensorCustom5Width =
			params->sensor_info.custom5.grabwindow_width;
		sensor_resolution->SensorCustom5Height =
			params->sensor_info.custom5.grabwindow_height;
	}

	return ERROR_NONE;
}

static uint32 get_mode_info(MSDK_SENSOR_INFO_STRUCT *sensor_info,
	struct camkit_mode_info *mode_info)
{
	sensor_info->SensorGrabStartX = mode_info->startx;
	sensor_info->SensorGrabStartY = mode_info->starty;
	sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			mode_info->mipi_data_lp2hs_settle_dc;

	return ERROR_NONE;
}

static uint32 get_info(struct camkit_params *kit_params,
	enum MSDK_SCENARIO_ID_ENUM scenario_id,
	MSDK_SENSOR_INFO_STRUCT *sensor_info,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	struct camkit_sensor_params *params = NULL;
	struct camkit_mode_info *mode_info = NULL;

	if (!kit_params || !kit_params->sensor_params ||
		!sensor_info || !sensor_config_data) {
		LOG_ERR("NULL ptr. sensor_info:%pK, sensor_config_data:%pK\n",
			sensor_info, sensor_config_data);
		return ERROR_INVALID_PARA;
	}

	params = kit_params->sensor_params;

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity =
		SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorHsyncPolarity =
		SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4;
	sensor_info->SensorResetActiveHigh = FALSE;
	sensor_info->SensorResetDelayCount = 5;

	sensor_info->SensroInterfaceType = params->sensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = params->sensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = params->sensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat =
		params->sensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = params->sensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = params->sensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = params->sensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame =
		params->sensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame =
		params->sensor_info.slim_video_delay_frame;
	sensor_info->SlimVideoDelayFrame =
		params->sensor_info.slim_video_delay_frame;
	sensor_info->Custom1DelayFrame =
		params->sensor_info.custom1_delay_frame;
	sensor_info->Custom2DelayFrame =
		params->sensor_info.custom2_delay_frame;
	sensor_info->Custom3DelayFrame =
		params->sensor_info.custom3_delay_frame;
	sensor_info->Custom4DelayFrame =
		params->sensor_info.custom4_delay_frame;
	sensor_info->Custom5DelayFrame =
		params->sensor_info.custom5_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0;
	sensor_info->SensorDrivingCurrent = params->sensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame = params->sensor_info.ae_shut_delay_frame;
	sensor_info->AESensorGainDelayFrame =
		params->sensor_info.ae_sensor_gain_delay_frame;
	sensor_info->AEISPGainDelayFrame =
		params->sensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = params->sensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = params->sensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = params->sensor_info.sensor_mode_num;
	sensor_info->PDAF_Support = params->sensor_info.pdaf_support;

	sensor_info->SensorMIPILaneNumber = params->sensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = params->sensor_info.mclk;
	sensor_info->SensorClockDividCount = 3;
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2;
	sensor_info->SensorPixelClockCount = 3;
	sensor_info->SensorDataLatchCount = 2;

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;
	sensor_info->SensorHightSampling = 0;
	sensor_info->SensorPacketECCOrder = 1;

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		mode_info = &(params->sensor_info.pre);
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		mode_info = &(params->sensor_info.cap);
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		mode_info = &(params->sensor_info.normal_video);
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		mode_info = &(params->sensor_info.hs_video);
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		mode_info = &(params->sensor_info.slim_video);
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		mode_info = &(params->sensor_info.custom1);
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		mode_info = &(params->sensor_info.custom2);
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		mode_info = &(params->sensor_info.custom3);
		break;
	case MSDK_SCENARIO_ID_CUSTOM4:
		mode_info = &(params->sensor_info.custom4);
		break;
	case MSDK_SCENARIO_ID_CUSTOM5:
		mode_info = &(params->sensor_info.custom5);
		break;
	default:
		mode_info = &(params->sensor_info.pre);
		break;
	}

	return get_mode_info(sensor_info, mode_info);
}

static uint32 control(struct camkit_params *kit_params,
	enum MSDK_SCENARIO_ID_ENUM scenario_id,
	MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	struct camkit_sensor_params *params = NULL;
	uint32 ret;

	if (!kit_params || !kit_params->sensor_params) {
		LOG_ERR("invalid parameter");
		return ERROR_INVALID_PARA;
	}
	params = kit_params->sensor_params;

	LOG_DBG("scenario_id = %d\n", scenario_id);
	spin_lock(&camkit_lock);
	params->sensor_ctrl.current_scenario_id = (camkit_scenario_type)scenario_id;
	spin_unlock(&camkit_lock);

	ret = set_setting_by_scenario(params, scenario_id,
		image_window, sensor_config_data);
	if (ret) {
		LOG_ERR("set setting fail");
		return ret;
	}

	if (params->sensor_ctrl.pdaf_mode && params->correction_info.spc_apply) {
		if (params->correction_info.spc_type == PDAF_SPC_LRC)
			write_lrc_data_to_sensor(kit_params);
		else if (params->correction_info.spc_type == PDAF_SPC_PDC)
			write_pdc_data_to_sensor(kit_params);
	}

	return ERROR_NONE;
}

static uint32 set_video_mode(struct camkit_sensor_params *params,
	uint16 framerate)
{
	LOG_INF("framerate = %u\n ", framerate);
	if (framerate == 0)
		return ERROR_NONE;

	spin_lock(&camkit_lock);
	/* fps set to 298 for anti-flicker */
	if ((framerate == 300) && (params->sensor_ctrl.autoflicker_en == TRUE))
		params->sensor_ctrl.current_fps = 298;
	/* fps set to 146 for anti-flicker */
	else if ((framerate == 150) && (params->sensor_ctrl.autoflicker_en == TRUE))
		params->sensor_ctrl.current_fps = 146;
	else
		params->sensor_ctrl.current_fps = framerate;
	spin_unlock(&camkit_lock);

	set_max_framerate(params, params->sensor_ctrl.current_fps, 1);

	return ERROR_NONE;
}

static uint32 set_auto_flicker_mode(struct camkit_sensor_params *params,
	bool enable, uint16 framerate)
{
	LOG_INF("enable = %d, framerate = %u\n", enable, framerate);
	spin_lock(&camkit_lock);
	if (enable)
		params->sensor_ctrl.autoflicker_en = TRUE;
	else
		params->sensor_ctrl.autoflicker_en = FALSE;
	spin_unlock(&camkit_lock);

	return ERROR_NONE;
}

static uint32 set_max_framerate_by_modeinfo(
	struct camkit_sensor_params *params,
	struct camkit_mode_info *mode_info, uint32 framerate)
{
	uint32 frame_length;

	if ((framerate == 0) || (mode_info->linelength == 0))
		return ERROR_NONE;

	frame_length = mode_info->pclk / framerate * PIX_10_BITS / mode_info->linelength;

	spin_lock(&camkit_lock);
	params->sensor_ctrl.dummy_line = (frame_length > mode_info->framelength) ?
		(frame_length - mode_info->framelength) : 0;
	params->sensor_ctrl.frame_length = mode_info->framelength +
		params->sensor_ctrl.dummy_line;
	params->sensor_ctrl.min_frame_length = params->sensor_ctrl.frame_length;
	spin_unlock(&camkit_lock);

	if (params->sensor_ctrl.frame_length > params->sensor_ctrl.shutter)
		set_dummy(params);

	return ERROR_NONE;
}

static uint32 set_max_framerate_by_scenario(
	struct camkit_sensor_params *params,
	enum MSDK_SCENARIO_ID_ENUM scenario_id, uint32 framerate)
{
	struct camkit_mode_info *mode_info = NULL;

	LOG_DBG("scenario_id = %d, framerate = %u\n", scenario_id, framerate);
	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		mode_info = &params->sensor_info.pre;
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		mode_info = &params->sensor_info.cap;
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		mode_info = &params->sensor_info.normal_video;
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		mode_info = &params->sensor_info.hs_video;
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		mode_info = &params->sensor_info.slim_video;
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		mode_info = &params->sensor_info.custom1;
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		mode_info = &params->sensor_info.custom2;
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		mode_info = &params->sensor_info.custom3;
		break;
	case MSDK_SCENARIO_ID_CUSTOM4:
		mode_info = &params->sensor_info.custom4;
		break;
	case MSDK_SCENARIO_ID_CUSTOM5:
		mode_info = &params->sensor_info.custom5;
		break;
	default:  /* coding with  preview scenario by default */
		mode_info = &params->sensor_info.pre;
		LOG_ERR("error scenario_id = %d, use preview scenario\n", scenario_id);
		break;
	}

	return set_max_framerate_by_modeinfo(params, mode_info, framerate);
}

static void get_pclk_by_scenario(struct camkit_sensor_params *params,
	unsigned long long *feature_data)
{
	struct camkit_mode_info *mode_info = NULL;

	switch (*feature_data) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		mode_info = &(params->sensor_info.pre);
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		mode_info = &(params->sensor_info.cap);
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		mode_info = &(params->sensor_info.normal_video);
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		mode_info = &(params->sensor_info.hs_video);
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		mode_info = &(params->sensor_info.slim_video);
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		mode_info = &(params->sensor_info.custom1);
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		mode_info = &(params->sensor_info.custom2);
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		mode_info = &(params->sensor_info.custom3);
		break;
	case MSDK_SCENARIO_ID_CUSTOM4:
		mode_info = &(params->sensor_info.custom4);
		break;
	case MSDK_SCENARIO_ID_CUSTOM5:
		mode_info = &(params->sensor_info.custom5);
		break;
	default:
		mode_info = &(params->sensor_info.pre);
		break;
	}

	*(uint32 *)(uintptr_t)(*(feature_data + 1)) = mode_info->pclk;
}

static void get_period_scenario(struct camkit_sensor_params *params,
	unsigned long long *feature_data)
{
	struct camkit_mode_info *mode_info = NULL;

	switch (*feature_data) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		mode_info = &(params->sensor_info.pre);
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		mode_info = &(params->sensor_info.cap);
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		mode_info = &(params->sensor_info.normal_video);
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		mode_info = &(params->sensor_info.hs_video);
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		mode_info = &(params->sensor_info.slim_video);
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		mode_info = &(params->sensor_info.custom1);
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		mode_info = &(params->sensor_info.custom2);
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		mode_info = &(params->sensor_info.custom3);
		break;
	case MSDK_SCENARIO_ID_CUSTOM4:
		mode_info = &(params->sensor_info.custom4);
		break;
	case MSDK_SCENARIO_ID_CUSTOM5:
		mode_info = &(params->sensor_info.custom5);
		break;
	default:
		mode_info = &(params->sensor_info.pre);
		break;
	}

	*(uint32 *)(uintptr_t)(*(feature_data + 1)) =
		(mode_info->framelength << 16) + mode_info->linelength;
}

static uint32 get_default_framerate_by_scenario(
	struct camkit_sensor_params *params,
	enum MSDK_SCENARIO_ID_ENUM scenario_id, uint32 *framerate)
{
	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		*framerate = params->sensor_info.pre.max_framerate;
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		*framerate = params->sensor_info.normal_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		*framerate = params->sensor_info.cap.max_framerate;
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		*framerate = params->sensor_info.hs_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		*framerate = params->sensor_info.slim_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		*framerate = params->sensor_info.custom1.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		*framerate = params->sensor_info.custom2.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		*framerate = params->sensor_info.custom3.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM4:
		*framerate = params->sensor_info.custom4.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM5:
		*framerate = params->sensor_info.custom5.max_framerate;
		break;
	default:
		break;
	}

	return ERROR_NONE;
}

static uint32 streaming_control(struct camkit_sensor_params *params,
	bool enable)
{
	int32 rc;

	LOG_INF("Enter.enable:%d\n", enable);
	if (enable)
		rc = camkit_sensor_write_setting(&params->sensor_ctrl,
			&params->sensor_info.streamon_setting);
	else
		rc = camkit_sensor_write_setting(&params->sensor_ctrl,
			&params->sensor_info.streamoff_setting);

	if (rc < 0) {
		LOG_ERR("Failed enable:%d\n", enable);
		return ERROR_SENSOR_POWER_ON_FAIL;
	}
	LOG_INF("Exit.enable:%d\n", enable);

	return ERROR_NONE;
}

static void get_sensor_crop_info(struct camkit_sensor_params *params,
	uint32 feature, struct SENSOR_WINSIZE_INFO_STRUCT *win_info)
{
	uint32 index;
	switch (feature) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		index = 0;
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		/* arry 5 is custom1 setting */
		index = 5;
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		/* arry 6 is custom2 setting */
		index = 6;
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		/* arry 1 is capture setting */
		index = 1;
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		/* arry 2 is video setting */
		index = 2;
		break;
	default:
		/* arry 0 is preview setting */
		index = 0;
		break;
	}

	memcpy((void *)win_info,
		(void *)&params->output_info[index],
		sizeof(params->output_info[index]));
}

static void get_sensor_mipi_pixel_rate(struct camkit_sensor_params *params,
	unsigned long long *feature_data)
{
	struct camkit_mode_info *mode_info = NULL;

	switch (*feature_data) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		mode_info = &(params->sensor_info.pre);
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		mode_info = &(params->sensor_info.cap);
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		mode_info = &(params->sensor_info.normal_video);
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		mode_info = &(params->sensor_info.hs_video);
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		mode_info = &(params->sensor_info.slim_video);
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		mode_info = &(params->sensor_info.custom1);
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		mode_info = &(params->sensor_info.custom2);
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		mode_info = &(params->sensor_info.custom3);
		break;
	case MSDK_SCENARIO_ID_CUSTOM4:
		mode_info = &(params->sensor_info.custom4);
		break;
	case MSDK_SCENARIO_ID_CUSTOM5:
		mode_info = &(params->sensor_info.custom5);
		break;
	default:
		mode_info = &(params->sensor_info.pre);
		break;
	}

	*(uint32 *)(uintptr_t)(*(feature_data + 1)) = mode_info->mipi_pixel_rate;
}

static void get_sensor_pixel_rate(struct camkit_sensor_params *params,
	unsigned long long *feature_data)
{
	struct camkit_mode_info *mode_info = NULL;

	switch (*feature_data) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		mode_info = &(params->sensor_info.pre);
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		mode_info = &(params->sensor_info.cap);
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		mode_info = &(params->sensor_info.normal_video);
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		mode_info = &(params->sensor_info.hs_video);
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		mode_info = &(params->sensor_info.slim_video);
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		mode_info = &(params->sensor_info.custom1);
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		mode_info = &(params->sensor_info.custom2);
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		mode_info = &(params->sensor_info.custom3);
		break;
	case MSDK_SCENARIO_ID_CUSTOM4:
		mode_info = &(params->sensor_info.custom4);
		break;
	case MSDK_SCENARIO_ID_CUSTOM5:
		mode_info = &(params->sensor_info.custom5);
		break;
	default:
		mode_info = &(params->sensor_info.pre);
		break;
	}

	if (mode_info->linelength > CAMKIT_LINGLENGTH_GAP)
		*(uint32 *)(uintptr_t)(*(feature_data + 1)) = (mode_info->pclk /
			(mode_info->linelength - CAMKIT_LINGLENGTH_GAP)) *
			mode_info->grabwindow_width;
}

static void main_camera_get_pdaf_info(struct camkit_sensor_params *params,
	unsigned long long *feature_data)
{
	struct SET_PD_BLOCK_INFO_T *PDAFinfo = NULL;
	LOG_INF("KYM_PDAF SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%d\n",
			(uint32)*feature_data);
	PDAFinfo = (struct SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data + 1));
	switch (*feature_data) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
	case MSDK_SCENARIO_ID_CUSTOM1:
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		memcpy((void *)PDAFinfo, (void *)&params->pdaf_info,
			sizeof(struct SET_PD_BLOCK_INFO_T));
		break;
	default:
		break;
	}
}

static void main_camera_get_pdaf_capacity(unsigned long long *feature_data)
{
	LOG_DBG("PDAF_CAPACITY scenarioId:%llu\n", *feature_data);
	switch (*feature_data) {
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		*(uint32 *)(uintptr_t)(*(feature_data+1)) = 1;
		break;
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		*(uint32 *)(uintptr_t)(*(feature_data + 1)) = 1;
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
	default:
		*(uint32 *)(uintptr_t)(*(feature_data+1)) = 0;
		break;
	}
}

static void get_binning_type(struct camkit_sensor_params *params,
	unsigned long long *feature_data, uint32 *feature_param)
{
	struct camkit_sensor_info_t *sensor_info = &params->sensor_info;
	int32 index = 0;

	switch (*(feature_data + 1)) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		index = CAMKIT_SCENARIO_ID_CAMERA_PREVIEW;
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		index = MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG;
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		index = MSDK_SCENARIO_ID_VIDEO_PREVIEW;
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		index = MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO;
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		index = MSDK_SCENARIO_ID_SLIM_VIDEO;
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		index = MSDK_SCENARIO_ID_CUSTOM1;
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		index = MSDK_SCENARIO_ID_CUSTOM2;
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		index = MSDK_SCENARIO_ID_CUSTOM3;
		break;
	case MSDK_SCENARIO_ID_CUSTOM4:
		index = MSDK_SCENARIO_ID_CUSTOM4;
		break;
	case MSDK_SCENARIO_ID_CUSTOM5:
		index = MSDK_SCENARIO_ID_CUSTOM5;
		break;
	default:
		index = CAMKIT_SCENARIO_ID_CAMERA_PREVIEW;
		break;
	}

	if (index >= 0 && index < MAX_OUTPUT_INFO_SIZE)
		*feature_param = sensor_info->binning_ratio[index];

	// default ratio is 1
	if (*feature_param == 0)
		*feature_param = 1;

	LOG_INF("binning type ratio %d\n", (*feature_param));
}

static void get_vc_info(struct camkit_sensor_params *params,
	unsigned long long *feature_data)
{
	struct SENSOR_VC_INFO_STRUCT *mtk_vc_info = NULL;
	int mtk_vc_info_len = sizeof(struct SENSOR_VC_INFO_STRUCT);
	int camkit_vc_info_len = sizeof(struct camkit_sensor_vc_info_t);
	struct camkit_sensor_vc_info_t *vc_info = params->vc_info;

	if (mtk_vc_info_len != camkit_vc_info_len) {
		LOG_ERR("mtk&camkit vc info struct mismatch");
		return;
	}

	mtk_vc_info = (struct SENSOR_VC_INFO_STRUCT *)
		(uintptr_t) (*(feature_data + 1));

	switch (*feature_data) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		memcpy((void *)mtk_vc_info,
			(void *)&vc_info[0],
			mtk_vc_info_len);
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		memcpy((void *)mtk_vc_info,
			(void *)&vc_info[1],
			mtk_vc_info_len);
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		 memcpy((void *)mtk_vc_info,
			(void *)&vc_info[2],
			mtk_vc_info_len);
		break;
	default:
		memcpy((void *)mtk_vc_info,
			(void *)&vc_info[0],
			mtk_vc_info_len);
		break;
	}
}

static void get_pdaf_regs_data(struct camkit_sensor_params *params,
	uint16 *reg_pairs, uint32 reg_num)
{
	uint32 i;
	uint32 reg_idx;
	int32 ret;

	for (i = 0; i < reg_num; i++) {
		reg_idx = i * 2;  // register pairs: [addr, data, addr, data...]
		ret = camkit_sensor_i2c_read(&params->sensor_ctrl, reg_pairs[reg_idx],
			reg_pairs + reg_idx + 1, CAMKIT_I2C_BYTE_DATA);
		if (ret) {
			LOG_ERR("read register fail: 0x%x\n", reg_pairs[reg_idx]);
			return;
		}
		LOG_DBG("[0x%x 0x%x]\n", reg_pairs[reg_idx], reg_pairs[reg_idx + 1]);
	}
}

static void set_pdaf_setting(struct camkit_sensor_params *params,
	uint16 *setting, uint32 reg_num)
{
	int32 ret;
	ret = camkit_i2c_write_table(&params->sensor_ctrl, setting,
		reg_num * 2, CAMKIT_I2C_BYTE_DATA);
	if (ret) {
		LOG_ERR("read register fail\n");
		return;
	}
}

static uint32 feature_control(struct camkit_params *kit_params,
	MSDK_SENSOR_FEATURE_ENUM feature_id,
	uint8 *feature_para, uint32 *feature_para_len)
{
	uint32 rc = ERROR_NONE;
	struct camkit_sensor_params *params = NULL;
	uint16 *feature_return_para_16 = (uint16 *)feature_para;
	uint16 *feature_data_16 = (uint16 *)feature_para;
	uint32 *feature_return_para_32 = (uint32 *)feature_para;
	uint32 *feature_data_32 = (uint32 *)feature_para;
	unsigned long long *feature_data = (unsigned long long *)feature_para;

	struct SENSOR_WINSIZE_INFO_STRUCT *win_info = NULL;

	if (!kit_params || !kit_params->sensor_params ||
		!feature_para || !feature_para_len) {
		LOG_ERR("Fatal null. params:%pK,feature_para:%pK,feature_para_len:%pK\n",
			kit_params, feature_para, feature_para_len);
		return ERROR_NONE;
	}

	params = kit_params->sensor_params;
	LOG_DBG("feature_id = %d\n", feature_id);
	switch (feature_id) {
	case SENSOR_FEATURE_GET_GAIN_RANGE_BY_SCENARIO:
		*(feature_data + 1) = params->aec_info.min_gain;
		*(feature_data + 2) = params->aec_info.max_gain;
		break;
	case SENSOR_FEATURE_GET_BASE_GAIN_ISO_AND_STEP:
		*(feature_data + 0) = params->aec_info.min_iso;
		*(feature_data + 1) = params->aec_info.gain_step;
		*(feature_data + 2) = params->aec_info.gain_type;
		break;
	case SENSOR_FEATURE_GET_MIN_SHUTTER_BY_SCENARIO:
		*(feature_data + 1) = params->aec_info.min_linecount;
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ_BY_SCENARIO:
		get_pclk_by_scenario(params, feature_data);
		break;
	case SENSOR_FEATURE_GET_PERIOD_BY_SCENARIO:
		get_period_scenario(params, feature_data);
		break;
	case SENSOR_FEATURE_GET_PERIOD:
		*feature_return_para_16++ = params->sensor_ctrl.line_length;
		*feature_return_para_16 = params->sensor_ctrl.frame_length;
		*feature_para_len = 4; /* return 4 byte data */
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
		*feature_return_para_32 = params->sensor_ctrl.pclk;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_ESHUTTER:
		set_shutter(params, (uint32)*feature_data);
		break;
	case SENSOR_FEATURE_SET_GAIN:
		set_gain(params, (uint16)*feature_data);
		break;
	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
		/* get the lens driver ID from EEPROM */
		/* or just return LENS_DRIVER_ID_DO_NOT_CARE */
		/* if EEPROM does not exist in camera module. */
		*feature_return_para_32 = LENS_DRIVER_ID_DO_NOT_CARE;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_VIDEO_MODE:
		set_video_mode(params, (uint16)*feature_data);
		break;
	case SENSOR_FEATURE_CHECK_SENSOR_ID:
		get_match_id(kit_params, feature_return_para_32);
		break;
	case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
		set_auto_flicker_mode(params, (bool)*feature_data_16,
			*(feature_data_16 + 1));
		break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		set_max_framerate_by_scenario(params,
			(enum MSDK_SCENARIO_ID_ENUM)*feature_data,
			(uint32)*(feature_data + 1));
		break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		get_default_framerate_by_scenario(params,
			(enum MSDK_SCENARIO_ID_ENUM)*(feature_data),
			(uint32 *)(uintptr_t)(*(feature_data + 1)));
		break;
	case SENSOR_FEATURE_GET_PDAF_DATA:
		LOG_INF("SENSOR_FEATURE_GET_PDAF_DATA. No support\n");
		break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		(void)set_test_pattern_mode(params, (bool)*feature_data);
		break;
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
		/* for factory mode auto testing */
		*feature_return_para_32 = params->sensor_info.checksum_value;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_FRAMERATE:
		LOG_INF("current fps :%d\n", *feature_data_32);
		spin_lock(&camkit_lock);
		params->sensor_ctrl.current_fps = (uint16)*feature_data_32;
		spin_unlock(&camkit_lock);
		break;
	case SENSOR_FEATURE_GET_CROP_INFO:
		win_info = (struct SENSOR_WINSIZE_INFO_STRUCT *)
			(uintptr_t)(*(feature_data + 1));
		get_sensor_crop_info(params, *feature_data_32, win_info);
		break;
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		rc = streaming_control(params, FALSE);
		if (rc != ERROR_NONE)
			LOG_ERR("stream off failed\n");
		break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		if (*feature_data != 0)
			set_shutter(params, (uint32)*feature_data);
		rc = streaming_control(params, TRUE);
		if (rc != ERROR_NONE)
			LOG_ERR("stream on failed\n");
		break;
	case SENSOR_FEATURE_GET_PDAF_INFO:
		main_camera_get_pdaf_info(params, feature_data);
		break;
	case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
		if (params->sensor_info.pdaf_support)
			main_camera_get_pdaf_capacity(feature_data);
		break;
	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
		set_shutter_frame_length(params, (uint32)*feature_data,
			(uint32)*(feature_data + 1));
		break;
	case SENSOR_FEATURE_DUMP_REG_LCT:
		sensor_dump_reg(params);
		break;
	case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
		get_sensor_mipi_pixel_rate(params, feature_data);
		break;
	case SENSOR_FEATURE_GET_PIXEL_RATE:
		get_sensor_pixel_rate(params, feature_data);
		break;
	case SENSOR_FEATURE_GET_BINNING_TYPE:
		get_binning_type(params, feature_data, feature_return_para_32);
		LOG_INF("binning type ratio %d\n", (*feature_return_para_32));
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_VC_INFO:
		LOG_INF("SENSOR_FEATURE_GET_VC_INFO %d\n", (uint16)*feature_data);
		get_vc_info(params, feature_data);
		break;
	case SENSOR_FEATURE_GET_PDAF_REG_SETTING:
		LOG_INF("get pdaf setting size %d\n", (*feature_para_len));
		get_pdaf_regs_data(params, feature_data_16,
			(*feature_para_len) / sizeof(uint32));
		break;
	case SENSOR_FEATURE_SET_PDAF_REG_SETTING:
		LOG_INF("set pdaf setting size %d\n", (*feature_para_len));
		set_pdaf_setting(params, feature_data_16,
			(*feature_para_len) / sizeof(uint32));
		break;
	case SENSOR_FEATURE_SET_PDAF:
		LOG_INF("PDAF mode :%d\n", *feature_data_16);
		params->sensor_ctrl.pdaf_mode = *feature_data_16;
		break;
	default:
		LOG_INF("Not support the feature_id:%d\n", feature_id);
		break;
	}

	return ERROR_NONE;
}

static struct sensor_kit_ops sensor_driver_ops = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

uint32 sensor_driver_init(struct sensor_kit_ops **ops)
{
	LOG_INF("%s ops:%pK\n", __func__, ops);
	if (ops != NULL)
		*ops = &sensor_driver_ops;

	return ERROR_NONE;
}
