/*
 * camkit_driver.h
 *
 * Copyright (c) 2021-2021 LongCheer Technologies Co., Ltd.
 *
 * sensor interface define: 3A, initial operators
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

#ifndef CAMKIT_DRIVER_H
#define CAMKIT_DRIVER_H

#include "camkit_driver_types.h"

uint32 sensor_driver_init(struct sensor_kit_ops **ops);

void set_shutter(struct camkit_sensor_params *params, uint32 shutter);
uint16 set_gain(struct camkit_sensor_params *params, uint16 gain);
void set_dummy(struct camkit_sensor_params *params);
void set_max_framerate(struct camkit_sensor_params *params,
	uint16 framerate, bool min_framelength_en);
void set_shutter_frame_length(struct camkit_sensor_params *params,
	uint32 shutter, uint32 frame_length);

#endif // CAMKIT_DRIVER_H
