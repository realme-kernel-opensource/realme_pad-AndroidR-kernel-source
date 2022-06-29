/*
 * 
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/of.h>

/* file system */
//#include <stdio.h>
//#include <sys/stat.h>

#include "cam_cal.h"
#include "cam_cal_define.h"
#include "cam_cal_list.h"
#include "kd_camera_typedef.h"
#include <linux/dma-mapping.h>

#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_typedef.h"

#include "hi846_main_otp.h"



#define Sleep(ms) mdelay(ms)

static int debug_log = 0;

#define PFX "hi846_main_otp"
#define LOG_INF(format, args...)    \
	pr_debug(PFX "[%s] " format, __func__, ##args)
#define LOG_INF_IF(...)      do { if ( (debug_log) ) { LOG_INF(__VA_ARGS__); } }while(0)

//static DEFINE_SPINLOCK(hi846_otp_lock);

static struct i2c_client *g_pstI2CclientG;

static int read_done = 0;

struct hi846_otp {
    kal_uint8 module[HI846_DATA_LEN_MODULE];
    kal_uint8 lsc[HI846_DATA_LEN_LSC];
    kal_uint8 awb[HI846_DATA_LEN_AWB];
    kal_uint8 af[HI846_DATA_LEN_AF];
};

static struct hi846_otp hi846_otp_data = {0};

static int iReadRegI2C(u8 *a_pSendData, u16 a_sizeSendData,
		u8 *a_pRecvData, u16 a_sizeRecvData){
	int  i4RetValue = 0;

	i4RetValue = i2c_master_send(g_pstI2CclientG,
		a_pSendData, a_sizeSendData);
	if (i4RetValue != a_sizeSendData) {
		pr_debug("I2C send failed!!, Addr = 0x%x\n", a_pSendData[0]);
		return -1;
	}
	i4RetValue = i2c_master_recv(g_pstI2CclientG,
		(char *)a_pRecvData, a_sizeRecvData);
	if (i4RetValue != a_sizeRecvData) {
		pr_debug("I2C read failed!!\n");
		return -1;
	}
	return 0;
}
static int iWriteRegI2C(u8 *a_pSendData, u16 a_sizeSendData){
	int  i4RetValue = 0;

	i4RetValue = i2c_master_send(g_pstI2CclientG,
		a_pSendData, a_sizeSendData);
	if (i4RetValue != a_sizeSendData) {
		pr_debug("I2C send failed!!, Addr = 0x%x\n", a_pSendData[0]);
		return -1;
	}
	return 0;

}

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };

	iReadRegI2C(pu_send_cmd, 2, (u8 *)&get_byte, 1);

	return get_byte;
}

/*
static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[4] = {(char)(addr >> 8),
		(char)(addr & 0xFF), (char)(para >> 8), (char)(para & 0xFF)};

	iWriteRegI2C(pu_send_cmd, 4);
}
*/

static void write_cmos_sensor_8(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[4] = {(char)(addr >> 8),
		(char)(addr & 0xFF), (char)(para & 0xFF)};

	iWriteRegI2C(pu_send_cmd, 3);
}

static void  hi846_otp_enable(){
	LOG_INF("start");
    write_cmos_sensor_8(0x0A02, 0x01);
    write_cmos_sensor_8(0x0A00, 0x00);
    Sleep(10);
    write_cmos_sensor_8(0x0F02, 0x00);
    write_cmos_sensor_8(0x071A, 0x01);
    write_cmos_sensor_8(0x071B, 0x09);
    write_cmos_sensor_8(0x0D04, 0x00);
    write_cmos_sensor_8(0x0D00, 0x07);
    write_cmos_sensor_8(0x003E, 0x10);
    write_cmos_sensor_8(0x0A00, 0x01);
    Sleep(1);
}

static void  hi846_otp_disable(){
	LOG_INF("start");
	write_cmos_sensor_8(0x0A00, 0x00); // stand by on
	Sleep(10);
	write_cmos_sensor_8(0x003E, 0x00); // display mode
	write_cmos_sensor_8(0x0A00, 0x01); // stand by off
	Sleep(1);

}

static kal_uint16 hi846_otp_read_flag(kal_uint32 addr){
	kal_uint16 flag = 0x100;
	// LOG_INF_IF("init flag=0x%x", flag);
    write_cmos_sensor_8 (0x070a, (addr>>8) & 0xff);  // start address H
    write_cmos_sensor_8 (0x070b, addr & 0xff);       // start address L
    write_cmos_sensor_8 (0x0702, 0x01);              // read mode   

    flag = read_cmos_sensor(HI846_DATA_READ_REG);
	LOG_INF("addr = 0x%x, read flag=0x%x", addr, flag);
	return flag;
}

static kal_uint16 hi846_otp_read_data(kal_uint32 addr, kal_uint8* buf, kal_uint16 len){
    kal_uint16 i = 0;
    kal_uint32 checksum = 0;

	LOG_INF("addr=0x%x, size=0x%x", addr, len);
    write_cmos_sensor_8 (0x070a, (addr>>8) & 0xff);  // start address H
    write_cmos_sensor_8 (0x070b, addr & 0xff);       // start address L
    write_cmos_sensor_8 (0x0702, 0x01);              // read mode   

    for(i=0; i<len; i++) {
        buf[i] = (kal_uint8)read_cmos_sensor(HI846_DATA_READ_REG);
        checksum += buf[i];
		LOG_INF_IF("read otp 0x%04x, 0x%02x, checksum[0x%04x]=0x%04x", i, buf[i], i, checksum);
    }

	LOG_INF("checksum=0x%x, checkvalue=0x%x, i=0x%x(%d)", checksum, buf[len-1], i, i);
    if((((checksum-buf[len-1])%255)+1) == buf[len-1]) {
		LOG_INF("checksum pass");
    }
    else {
		LOG_INF("checksum fail!");
		i=0;
    }

    return i;
}

static kal_uint16 hi846_otp_read_data_retry(kal_uint32 addr, kal_uint8* buf, kal_uint16 len){
	kal_uint16 retry_times = 0;

	for(retry_times=0; retry_times<3; retry_times++) {
    	if(len == hi846_otp_read_data(addr, buf, len)) {
			break;
    	} else {
    		LOG_INF("retry_times=%d", retry_times);
    		hi846_otp_disable();
			hi846_otp_enable();
    	}
	}

	if(retry_times >= 3) {
		LOG_INF("read af error! retry_times=%d", retry_times);
		return 0;
	}
	return len;
}

static kal_bool hi846_otp_read_module_info(void){
    kal_uint32 addr = 0;
    kal_uint16 flag = 0x100;

	LOG_INF("start");
	flag = hi846_otp_read_flag(HI846_FLAG_MODULE_ADDR);
    if (HI846_FLAG_GROUP1 == flag) {
        addr = HI846_DATA_ADDR_MODULE1;
    } else  if (HI846_FLAG_GROUP2 == flag) {
        addr = HI846_DATA_ADDR_MODULE2;
    } else  if (HI846_FLAG_GROUP3 == flag) {
        addr = HI846_DATA_ADDR_MODULE3;
    } else {
    	LOG_INF("read flag(0x%x) error!", flag);
        return KAL_FALSE;
    }

    hi846_otp_read_data_retry(addr, (kal_uint8*)&(hi846_otp_data.module), HI846_DATA_LEN_MODULE);
    return KAL_TRUE;
}

static kal_bool hi846_otp_read_lsc(void){
    kal_uint32 addr = 0;
    kal_uint16 flag = 0x100;

	LOG_INF("start");
	flag = hi846_otp_read_flag(HI846_FLAG_LSC_ADDR);

    if (HI846_FLAG_GROUP1 == flag) {
        addr = HI846_DATA_ADDR_LSC1;
    } else  if (HI846_FLAG_GROUP2 == flag) {
        addr = HI846_DATA_ADDR_LSC2;
    } else  if (HI846_FLAG_GROUP3 == flag) {
        addr = HI846_DATA_ADDR_LSC3;
    } else {
    	LOG_INF("read flag(0x%x) error!", flag);
        return KAL_FALSE;
    }

    hi846_otp_read_data_retry(addr, (kal_uint8*)&(hi846_otp_data.lsc), HI846_DATA_LEN_LSC);
    return KAL_TRUE;
}

static kal_bool hi846_otp_read_awb(void){
    kal_uint32 addr = 0;
    kal_uint16 flag = 0x100;

	LOG_INF("start");
	flag = hi846_otp_read_flag(HI846_FLAG_AWB_ADDR);

    if (HI846_FLAG_GROUP1 == flag) {
        addr = HI846_DATA_ADDR_AWB1;
    } else  if (HI846_FLAG_GROUP2 == flag) {
        addr = HI846_DATA_ADDR_AWB2;
    } else  if (HI846_FLAG_GROUP3 == flag) {
        addr = HI846_DATA_ADDR_AWB3;
    } else {
    	LOG_INF("read flag(0x%x) error!", flag);
        return KAL_FALSE;
    }

    hi846_otp_read_data_retry(addr, (kal_uint8*)&(hi846_otp_data.awb), HI846_DATA_LEN_AWB);
    return KAL_TRUE;
}

static kal_bool hi846_otp_read_af(void){
    kal_uint32 addr = 0;
    kal_uint16 flag = 0x100;

	LOG_INF("start");
	flag = hi846_otp_read_flag(HI846_FLAG_AF_ADDR);

    
    if (HI846_FLAG_GROUP1 == flag) {
        addr = HI846_DATA_ADDR_AF1;
    } else  if (HI846_FLAG_GROUP2 == flag) {
        addr = HI846_DATA_ADDR_AF2;
    } else  if (HI846_FLAG_GROUP3 == flag) {
        addr = HI846_DATA_ADDR_AF3;
    } else {
    	LOG_INF("read flag(0x%x) error!", flag);
        return KAL_FALSE;
    }

	hi846_otp_read_data_retry(addr, (kal_uint8*)&(hi846_otp_data.af), HI846_DATA_LEN_AF);
    return KAL_TRUE;
}

static void hi846_dump_otp(kal_uint8 *data, kal_uint32 OtpSize, unsigned int sensor_id)
{
    UINT32 idx = 0;

#if 1
	if(debug_log == 0) {
		LOG_INF("dump otp, dump enable=%d, return", debug_log);
		return;
	}
	LOG_INF("dump otp, sensor_id=0x%x, OtpSize=0x%x", sensor_id, OtpSize);
    for(idx=0; idx<OtpSize; idx++) {
		LOG_INF(" 0x%04x, 0x%02x", idx, data[idx]);
    }
#else
    INT32 ioctlerr, ret;
    char info[dumpSize];

    // open file
    char targetFile[50];
    ioctlerr = snprintf(targetFile, sizeof(targetFile), "/data/vendor/camera_dump/eeprom_sensor_%d", sensor_id);
    if (ioctlerr < 0 || ioctlerr > sizeof(targetFile)) {
        LOG_INF("generate path fail!");
        return;
    }

    FILE *fp = fopen(targetFile, "w");

    if(fp == NULL) {
        LOG_INF("open file fail!");
        return;
    }

    // get data
    info = new char[dumpSize];
        ret = fprintf(fp, "SensorID=0x%x\n", sensor_id);
        if(ret < 0) {
			LOG_INF("fprintf sensorID err=%d", ret);
            ret = fclose(fp);
            if (ret != 0) {
                CAM_CAL_ERR("fclose err\n");
            }
            return;
        }
        for(idx = 0 ; idx < dumpSize; idx++) {
            ret = fprintf(fp, "0x%04x,0x%02x\n", idx, info[idx]);
            if(ret < 0) {
				LOG_INF("fprintf otp err=%d, idx=%d", ret, idx);
                ret = fclose(fp);
                if (ret != 0) {
                    LOG_INF("fclose err\n");
                }
                return;
            }
        }
		LOG_INF("dump otp success! SensorID=0x%x, OTP Size=0x%x,\n", dumpSize, sensor_id, idx);

    ret = fclose(fp);
    if (ret != 0) {
        LOG_INF("fclose err\n");
    }
#endif
    return;
}


static void hi846_read_otp(){
	LOG_INF_IF("start");
    hi846_otp_enable();

	//spin_lock(&hi846_otp_lock);
    hi846_otp_read_module_info();
    hi846_otp_read_lsc();
    hi846_otp_read_awb();
    hi846_otp_read_af();
	//spin_unlock(&hi846_otp_lock);

    hi846_otp_disable();
}

unsigned int hi846_main_read_region(struct i2c_client *client, unsigned int addr, unsigned char *data, unsigned int size){
    unsigned char * buffer_temp = (unsigned char *)data;
    g_pstI2CclientG = client;

    if(g_pstI2CclientG == NULL){
        LOG_INF("g_pstI2CclientG==NULL");
        return 0;
    }
	LOG_INF("addr=0x%x, input size=%d, buffer_temp=0x%p", addr, size, buffer_temp);
	LOG_INF_IF("hi846_otp_data size=0x%x, HI846_OTP_SIZE=0x%x", sizeof(hi846_otp_data), HI846_OTP_SIZE);

	if(buffer_temp == NULL) {
		LOG_INF("iput para error! buffer_temp == NULL");
		return 0;
	}

	if((addr > HI846_OTP_SIZE)
		|| (size > HI846_OTP_SIZE)
		|| ((addr+size) > HI846_OTP_SIZE)) {
		LOG_INF("iput para error! addr=0x%x, size=0x%x, HI846_OTP_SIZE=0x%x", addr, size, HI846_OTP_SIZE);
		return 1;
	}

    LOG_INF("read_done=%d", read_done);
    if(read_done == 0){
		memset((void*)&hi846_otp_data, 0, sizeof(hi846_otp_data));
        hi846_read_otp();
        read_done = 1;
    }

	if(size == HI846_OTP_SIZE) {
	    memcpy(buffer_temp, hi846_otp_data.module, HI846_DATA_LEN_MODULE);
	    buffer_temp += HI846_DATA_LEN_MODULE;

	    memcpy(buffer_temp, hi846_otp_data.lsc, HI846_DATA_LEN_LSC);
	    buffer_temp += HI846_DATA_LEN_LSC;

	    memcpy(buffer_temp, hi846_otp_data.awb, HI846_DATA_LEN_AWB);
	    buffer_temp += HI846_DATA_LEN_AWB;

	    memcpy(buffer_temp, hi846_otp_data.af, HI846_DATA_LEN_AF);
	    buffer_temp += HI846_DATA_LEN_AF;
	} else {
		LOG_INF("hi846_otp_data addr=0x%p, hi846_otp_data.module addr=0x%p", &hi846_otp_data, hi846_otp_data.module);
		memcpy(buffer_temp, hi846_otp_data.module, size);
	}

	hi846_dump_otp((kal_uint8 *)&hi846_otp_data, sizeof(hi846_otp_data), 0x846);

    return size;
}

