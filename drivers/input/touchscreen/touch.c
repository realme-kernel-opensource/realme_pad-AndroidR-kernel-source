/***************************************************
 * File:touch.c
 * VENDOR_EDIT
 * Copyright (c)  2008- 2030  Oppo Mobile communication Corp.ltd.
 * Description:
 *             tp dev
 * Version:1.0:
 * Date created:2016/09/02
 * Author: hao.wang@Bsp.Driver
 * TAG: BSP.TP.Init
*/

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/serio.h>
#include "oppo_touchscreen/tp_devices.h"
#include "oppo_touchscreen/touchpanel_common.h"
/* Pan.Chen@BSP.TP.Function, 2020/09/11, bringup add for touchscreen mould. */
#include <soc/oplus/system/oplus_project.h>
#include <soc/oppo/device_info.h>
#include "touch.h"

#define MAX_LIMIT_DATA_LENGTH         100
extern char *saved_command_line;
int g_tp_dev_vendor = TP_UNKNOWN;
int g_tp_prj_id = 0;
struct hw_resource *g_hw_res;
static bool is_tp_type_got_in_match = false;    /*indicate whether the tp type is specified in the dts*/
int tp_type = 0;
/*if can not compile success, please update vendor/oppo_touchsreen*/
struct tp_dev_name tp_dev_names[] = {
	{TP_NT36523B_DJN, "DJN"},
	{TP_HX83102_TXD, "TXD"},
	{TP_UNKNOWN, "UNKNOWN"},
};

typedef enum {
	TP_INDEX_NULL,
	nt36523b_djn,
	hx83102_txd,

} TP_USED_INDEX;
TP_USED_INDEX tp_used_index  = TP_INDEX_NULL;

#define GET_TP_DEV_NAME(tp_type) ((tp_dev_names[tp_type].type == (tp_type))?tp_dev_names[tp_type].name:"UNMATCH")

#ifndef CONFIG_MTK_FB
void primary_display_esd_check_enable(int enable)
{
	return;
}
EXPORT_SYMBOL(primary_display_esd_check_enable);
#endif /*CONFIG_MTK_FB*/

bool __init tp_judge_ic_match(char *tp_ic_name)
{
	pr_err("[TP] tp_ic_name = %s \n", tp_ic_name);

	int prj_id = 0;
	prj_id = get_project();

	switch(get_project()) {
		case 21697:
			pr_info("tp judge ic forward for 21697\n");
			pr_err("[TP] boot_command_line = %s \n", saved_command_line);
			if (strstr(tp_ic_name, "novatek,nf_nt36523") && strstr(saved_command_line, "nt36523b_hdp_dsi_vdo_dijing")
				&& strstr(saved_command_line, "lcm is connected")) {
				return true;
			}
			if (strstr(tp_ic_name, "himax,hx83102e_nf") && strstr(saved_command_line, "hx83_hdp_dsi_vdo_txd")
				&& strstr(saved_command_line, "lcm is connected")) {
				return true;
			}
			return false;

		case 21698:
			pr_info("tp judge ic forward for 21698\n");
			pr_err("[TP] boot_command_line = %s \n", saved_command_line);
			if (strstr(tp_ic_name, "novatek,nf_nt36523") && strstr(saved_command_line, "nt36523b_hdp_dsi_vdo_dijing")
				&& strstr(saved_command_line, "lcm is connected")) {
				return true;
			}
			if (strstr(tp_ic_name, "himax,hx83102e_nf") && strstr(saved_command_line, "hx83_hdp_dsi_vdo_txd")
				&& strstr(saved_command_line, "lcm is connected")) {
				return true;
			}
			return false;

		default:
			break;
	}
	return true;
}

EXPORT_SYMBOL(tp_judge_ic_match);
bool  tp_judge_ic_match_commandline(struct panel_info *panel_data)
{
	int prj_id = 0;
	int i = 0;
	bool ic_matched = false;
	prj_id = get_project();
	pr_err("[TP] get_project() = %d \n", prj_id);
	pr_err("[TP] boot_command_line = %s \n", saved_command_line);

	for (i = 0; i < panel_data->project_num; i++) {
		if (prj_id == panel_data->platform_support_project[i]) {
			g_tp_prj_id = panel_data->platform_support_project_dir[i];

			if (strstr(saved_command_line, panel_data->platform_support_commandline[i])
					|| strstr("nt36523b_hdp_dsi_vdo_dijing", panel_data->platform_support_commandline[i])) {
				pr_err("[TP] Driver match the project\n");
				ic_matched = true;
			}
			if (strstr(saved_command_line, panel_data->platform_support_commandline[i])
					|| strstr("hx83_hdp_dsi_vdo_txd", panel_data->platform_support_commandline[i])) {
				pr_err("[TP] Driver match the project\n");
				ic_matched = true;
			}
		}
	}

	if (!ic_matched) {
		pr_err("[TP] Driver does not match the project\n");
		pr_err("Lcd module not found\n");
		return false;
	}

	switch (prj_id) {
		case 21697:
			pr_info("[TP] case 21697\n");
			is_tp_type_got_in_match = true;

			if (strstr(saved_command_line, "nt36523b_hdp_dsi_vdo_dijing")) {
				tp_used_index = nt36523b_djn;
				g_tp_dev_vendor = TP_NT36523B_DJN;

			} else if (strstr(saved_command_line, "hx83_hdp_dsi_vdo_txd")) {
				tp_used_index = hx83102_txd;
				g_tp_dev_vendor = TP_HX83102_TXD;

			} else {
				g_tp_dev_vendor = TP_UNKNOWN;
			}

			pr_err("[TP] g_tp_dev_vendor: %s\n", tp_dev_names[g_tp_dev_vendor].name);
			break;

		case 21698:
			pr_info("[TP] case 21698\n");
			is_tp_type_got_in_match = true;

			if (strstr(saved_command_line, "nt36523b_hdp_dsi_vdo_dijing")) {
				tp_used_index = nt36523b_djn;
				g_tp_dev_vendor = TP_NT36523B_DJN;

			} else if (strstr(saved_command_line, "hx83_hdp_dsi_vdo_txd")) {
				tp_used_index = hx83102_txd;
				g_tp_dev_vendor = TP_HX83102_TXD;

			} else {
				g_tp_dev_vendor = TP_UNKNOWN;
			}

			pr_err("[TP] g_tp_dev_vendor: %s\n", tp_dev_names[g_tp_dev_vendor].name);
			break;

		default:
			pr_info("other project, no need process here!prj_id=%d\n", prj_id);
			break;
	}

	pr_info("[TP]ic:%d, vendor:%d\n", tp_used_index, g_tp_dev_vendor);
	return true;
}

EXPORT_SYMBOL(tp_judge_ic_match_commandline);
int tp_util_get_vendor(struct hw_resource *hw_res,
		       struct panel_info *panel_data)
{
	char *vendor;
	int prj_id = 0;
	prj_id = get_project();
	g_hw_res = hw_res;

	panel_data->test_limit_name = kzalloc(MAX_LIMIT_DATA_LENGTH, GFP_KERNEL);

	if (panel_data->test_limit_name == NULL) {
		pr_err("[TP]panel_data.test_limit_name kzalloc error\n");
	}

	panel_data->extra = kzalloc(MAX_LIMIT_DATA_LENGTH, GFP_KERNEL);

	if (panel_data->extra == NULL) {
		pr_err("[TP]panel_data.extra kzalloc error\n");
	}

	if (is_tp_type_got_in_match) {
		panel_data->tp_type = g_tp_dev_vendor;
	}

	if (panel_data->tp_type == TP_UNKNOWN) {
		pr_err("[TP]%s type is unknown\n", __func__);
		return 0;
	}

	vendor = GET_TP_DEV_NAME(panel_data->tp_type);

	if (21697 == prj_id || 21698 == prj_id) {
		if (nt36523b_djn == tp_used_index) {
		memcpy(panel_data->manufacture_info.version, "nova_", 5);
		}
		if (hx83102_txd == tp_used_index) {
		memcpy(panel_data->manufacture_info.version, "himax_", 6);
		}
	}

	strcpy(panel_data->manufacture_info.manufacture, vendor);

	switch (prj_id) {
		case 21697:
			pr_info("[TP] enter case 21697\n");

			if ((tp_used_index == nt36523b_djn) && (g_tp_dev_vendor == TP_NT36523B_DJN)){
				snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
					 "tp/21697/FW_%s_%s.bin",
					 "NT36523B", vendor);

				if (panel_data->test_limit_name) {
					snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
						 "tp/21697/LIMIT_%s_%s.img",
						 "NT36523B", vendor);
				}

				panel_data->manufacture_info.fw_path = panel_data->fw_name;
				pr_info("[TP]: firmware_headfile = FW_21697_NT36523B_DJN\n");
				memcpy(panel_data->manufacture_info.version, "NT36523B_DJN", 12);
				panel_data->firmware_headfile.firmware_data = FW_21697_NT36523B_DJN;
				panel_data->firmware_headfile.firmware_size = sizeof(FW_21697_NT36523B_DJN);
			}

			if ((tp_used_index == hx83102_txd) && (g_tp_dev_vendor == TP_HX83102_TXD)){
				snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
					 "tp/21697/FW_%s_%s.bin",
					 "HX83102E", vendor);

				if (panel_data->test_limit_name) {
					snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
						 "tp/21697/LIMIT_%s_%s.img",
						 "HX83102E", vendor);
				}

				panel_data->manufacture_info.fw_path = panel_data->fw_name;
				pr_info("[TP]: firmware_headfile = FW_21697_HX83102E_TXD\n");
				memcpy(panel_data->manufacture_info.version, "HX83102E_TXD", 12);
				panel_data->firmware_headfile.firmware_data = FW_21697_HX83102E_TXD;
				panel_data->firmware_headfile.firmware_size = sizeof(FW_21697_HX83102E_TXD);
			}
			break;
		case 21698:
			pr_info("[TP] enter case 21698\n");

			if ((tp_used_index == nt36523b_djn) && (g_tp_dev_vendor == TP_NT36523B_DJN)){
				snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
					 "tp/21698/FW_%s_%s.bin",
					 "NT36523B", vendor);

				if (panel_data->test_limit_name) {
					snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
						 "tp/21698/LIMIT_%s_%s.img",
						 "NT36523B", vendor);
				}

				panel_data->manufacture_info.fw_path = panel_data->fw_name;
				pr_info("[TP]: firmware_headfile = FW_21698_NT36523B_DJN\n");
				memcpy(panel_data->manufacture_info.version, "NT36523B_DJN", 12);
				panel_data->firmware_headfile.firmware_data = FW_21698_NT36523B_DJN;
				panel_data->firmware_headfile.firmware_size = sizeof(FW_21698_NT36523B_DJN);
			}

			if ((tp_used_index == hx83102_txd) && (g_tp_dev_vendor == TP_HX83102_TXD)){
				snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
					 "tp/21698/FW_%s_%s.bin",
					 "HX83102E", vendor);

				if (panel_data->test_limit_name) {
					snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
						 "tp/21698/LIMIT_%s_%s.img",
						 "HX83102E", vendor);
				}

				panel_data->manufacture_info.fw_path = panel_data->fw_name;
				pr_info("[TP]: firmware_headfile = FW_21698_HX83102E_TXD\n");
				memcpy(panel_data->manufacture_info.version, "HX83102E_TXD", 12);
				panel_data->firmware_headfile.firmware_data = FW_21698_HX83102E_TXD;
				panel_data->firmware_headfile.firmware_size = sizeof(FW_21698_HX83102E_TXD);
			}
			break;

		default:
			snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
				 "vendor/firmware/tp/%d/FW_%s_%s.img",
				 g_tp_prj_id, panel_data->chip_name, vendor);

			if (panel_data->test_limit_name) {
				snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
					 "vendor/firmware/tp/%d/LIMIT_%s_%s.img",
					 g_tp_prj_id, panel_data->chip_name, vendor);
			}

			if (panel_data->extra) {
				snprintf(panel_data->extra, MAX_LIMIT_DATA_LENGTH,
					 "tp/%d/BOOT_FW_%s_%s.ihex",
					 prj_id, panel_data->chip_name, vendor);
			}

			panel_data->manufacture_info.fw_path = panel_data->fw_name;
			break;
	}

	pr_info("[TP]vendor:%s fw:%s limit:%s\n",
		vendor,
		panel_data->fw_name,
		panel_data->test_limit_name == NULL ? "NO Limit" : panel_data->test_limit_name);

	return 0;
}
EXPORT_SYMBOL(tp_util_get_vendor);

/**
 * Description:
 * pulldown spi7 cs to avoid current leakage
 * because of current sourcing from cs (pullup state) flowing into display module
 **/
void switch_spi7cs_state(bool normal)
{
    if(normal){
        if( !IS_ERR_OR_NULL(g_hw_res->pin_set_high) ) {
            pr_info("%s: going to set spi7 cs to spi mode .\n", __func__);
            pinctrl_select_state(g_hw_res->pinctrl, g_hw_res->pin_set_high);
        }else{
            pr_info("%s: cannot to set spi7 cs to spi mode .\n", __func__);
        }
    } else {
        if( !IS_ERR_OR_NULL(g_hw_res->pin_set_low) ) {
            pr_info("%s: going to set spi7 cs to pulldown .\n", __func__);
            pinctrl_select_state(g_hw_res->pinctrl, g_hw_res->pin_set_low);
        }else{
            pr_info("%s: cannot to set spi7 cs to pulldown .\n", __func__);
        }
    }
}
EXPORT_SYMBOL(switch_spi7cs_state);
