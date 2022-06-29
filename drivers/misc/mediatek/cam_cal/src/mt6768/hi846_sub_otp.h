/*
 * Copyright (C) 2019 MediaTek Inc.
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

#ifndef __HI846_SUB_OTP_H__
#define __HI846_SUB_OTP_H__

#define HI846_I2C_WRITE_ID       0x40

#define HI846_FLAG_GROUP1        0x01
#define HI846_FLAG_GROUP2        0x13
#define HI846_FLAG_GROUP3        0x37

#define HI846_FLAG_MODULE_ADDR   0x0201
#define HI846_FLAG_LSC_ADDR      0x0262
#define HI846_FLAG_AWB_ADDR      0x184A
#define HI846_FLAG_AF_ADDR       0x187E

#define HI846_CHECKSUM_LEN       1
#define HI846_DATA_LEN_MODULE    (0x1F+HI846_CHECKSUM_LEN)
#define HI846_DATA_ADDR_MODULE1  0x0202
#define HI846_DATA_ADDR_MODULE2  0x0222
#define HI846_DATA_ADDR_MODULE3  0x0242

#define HI846_DATA_LEN_LSC       (0x74C+HI846_CHECKSUM_LEN)
#define HI846_DATA_ADDR_LSC1     0x0263
#define HI846_DATA_ADDR_LSC2     0x09B0
#define HI846_DATA_ADDR_LSC3     0x10FD

#define HI846_DATA_LEN_AWB       (0x10+HI846_CHECKSUM_LEN)
#define HI846_DATA_ADDR_AWB1     0x184B
#define HI846_DATA_ADDR_AWB2     0x185C
#define HI846_DATA_ADDR_AWB3     0x186D

#define HI846_DATA_LEN_AF        (0x04+HI846_CHECKSUM_LEN)
#define HI846_DATA_ADDR_AF1      0x187F
#define HI846_DATA_ADDR_AF2      0x1884
#define HI846_DATA_ADDR_AF3      0x1889

// HI846_OTP_SIZE = 0x783
#define HI846_OTP_SIZE           (HI846_DATA_LEN_MODULE + HI846_DATA_LEN_LSC + HI846_DATA_LEN_AWB + HI846_DATA_LEN_AF)

#define HI846_DATA_READ_REG      0x0708

#endif
