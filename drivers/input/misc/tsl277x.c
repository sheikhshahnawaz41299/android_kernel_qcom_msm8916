/* drivers/staging/taos/tsl277x.c
 *
 * Copyright (C) 2011 Sony Ericsson Mobile Communications AB.
 *
 * Author: Aleksej Makarov <aleksej.makarov@sonyericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/unistd.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include "tsl277x.h"
#include <linux/module.h>
#include <linux/sensors.h>

#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#endif

#include <linux/gpio.h>
#include <linux/irq.h>

/*[BUFFIX]-Mod-Begin- by TCTNB.ZXZ,FR-1034674, 2015/07/13, add misoperation interface */
#ifdef CONFIG_TCT_8X16_IDOL3
static struct i2c_client *g_client;
struct class *prx_misoperation_class = NULL;
#endif
/*[BUFFIX]-Mod-Begin- by TCTNB.ZXZ*/

#define DEVICE_NAME		"tsl2772"
#define ALS_NAME "tsl2772-als"
#define PS_NAME "tsl2772-ps"

struct input_dev *ido3_taos_proximity_dev = NULL;// [PLATFORM]-Mod-BEGIN by TCTNB.YQJ, PR-912196, 2015/01/24

//modify(add) by junfeng.zhou.sz for add print time begin . 20140214
#define PRINT_TIME
#ifdef PRINT_TIME
#include <linux/rtc.h>
static bool tslx_log_state=0;
#define TSLX(fmt, ...) \
	if(true==tslx_log_state){\
	printk(KERN_ERR pr_fmt("tsl277x.c:" "%s():"fmt),__func__, ##__VA_ARGS__);}

void print_local_time_taos(char *param)
{
    struct timespec ts;
	struct rtc_time tm;

	getnstimeofday(&ts);
	rtc_time_to_tm(ts.tv_sec+28800, &tm);
	if(param == NULL)
	    return ;
	TSLX( "%s %d-%02d-%02d %02d:%02d:%02d\n",param,
			 tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,tm.tm_hour, tm.tm_min, tm.tm_sec);
}
#endif
//modify(add) by junfeng.zhou.sz for add print time end .
/*[BUFFIX]-modify by TCTNB.ZXZ,PR949424, 2015/3/13,remove read tracebility of factory calibration*/
//#define USE_TRACEBILITY		// add by ning.wei for opimize p-sensor calibration pr743248 2014-08-21
#define ALS_POLL
#ifdef ALS_POLL
#include <linux/workqueue.h>
#include <linux/wakelock.h>

#endif

#define POWER_REGULATOR
#ifdef POWER_REGULATOR
#include <linux/regulator/consumer.h>
/* POWER SUPPLY VOLTAGE RANGE */
#define TSL2772_VDD_MIN_UV	2000000
#define TSL2772_VDD_MAX_UV	3300000
#define TSL2772_VIO_MIN_UV	1750000
#define TSL2772_VIO_MAX_UV	1950000
#endif

#define CALI_EVERY_TIME
#ifdef CALI_EVERY_TIME
/* [BUFFIX]-Mod- by TCTNB.XQJ,PR-804886, 2014/10/23, p sensor debug*/
#define TAOS_THD_H_OFFSET 300	// mod by ning.wei for opimize p-sensor pr743248 2014-08-21
#define TAOS_THD_L_OFFSET 150	// mod by ning.wei for opimize p-sensor pr743248 2014-08-21
/* [BUFFIX]-End-byTCTNB.XQJ*/
#endif
struct PS_CALI_DATA_STRUCT
{
    int close;
    int far_away;
    int valid;
    int offset;
    bool is_have_good_cali;
    u8  last_good_offset;
    int last_good_close;
    int last_good_far_away;
};
/*[BUFFIX]-Mod-Begin by TCTNB.ZXZ,PR-994478, 2015/05/08, add this flag for irq enable and disable*/
static bool irq_enable_status;
/*[BUFFIX]-Mod-End by TCTNB.ZXZ*/
/* [BUFFIX]-Mod- by TCTNB.XQJ,PR-892619, 2015/01/05, light sensor debug,ALS_TUNE_AUTOBRIGHTNESS no need*/
#ifndef CONFIG_TCT_8X16_IDOL3
#define ALS_TUNE_AUTOBRIGHTNESS
#endif
#define IR_INK_TRANSMISSION_COMPENSATE          (100/75)
#define AMBIENT_INK_TRANSMISSION_COMPENSATE     (100/10)
/* [BUFFIX]-End- by TCTNB.XQJ*/
#ifdef ALS_TUNE_AUTOBRIGHTNESS
#define TAOS_ALS_THD_BRIGHT	100	
#define TAOS_ALS_THD_DARK	15
int taos_dark_code_flag = 0;

#endif

static struct PS_CALI_DATA_STRUCT ps_cali={0,0,0,0,false,0,0,0};	// mod by ning.wei for opimize p-sensor calibration pr743248 2014-08-21

enum tsl277x_regs {
	TSL277X_ENABLE,
	TSL277X_ALS_TIME,
	TSL277X_PRX_TIME,
	TSL277X_WAIT_TIME,
	TSL277X_ALS_MINTHRESHLO,
	TSL277X_ALS_MINTHRESHHI,
	TSL277X_ALS_MAXTHRESHLO,
	TSL277X_ALS_MAXTHRESHHI,
	TSL277X_PRX_MINTHRESHLO,
	TSL277X_PRX_MINTHRESHHI,
	TSL277X_PRX_MAXTHRESHLO,
	TSL277X_PRX_MAXTHRESHHI,
	TSL277X_PERSISTENCE,
	TSL277X_CONFIG,
	TSL277X_PRX_PULSE_COUNT,
	TSL277X_CONTROL,

	TSL277X_REVID = 0x11,
	TSL277X_CHIPID,
	TSL277X_STATUS,
	TSL277X_ALS_CHAN0LO,
	TSL277X_ALS_CHAN0HI,
	TSL277X_ALS_CHAN1LO,
	TSL277X_ALS_CHAN1HI,
	TSL277X_PRX_LO,
	TSL277X_PRX_HI,

	TSL277X_REG_PRX_OFFS = 0x1e,
	TSL277X_REG_MAX,
};

enum tsl277x_cmd_reg {
	TSL277X_CMD_REG           = (1 << 7),
	TSL277X_CMD_INCR          = (0x1 << 5),
	TSL277X_CMD_SPL_FN        = (0x3 << 5),
	TSL277X_CMD_PROX_INT_CLR  = (0x5 << 0),
	TSL277X_CMD_ALS_INT_CLR   = (0x6 << 0),
};

enum tsl277x_en_reg {
	TSL277X_EN_PWR_ON   = (1 << 0),
	TSL277X_EN_ALS      = (1 << 1),
	TSL277X_EN_PRX      = (1 << 2),
	TSL277X_EN_WAIT     = (1 << 3),
	TSL277X_EN_ALS_IRQ  = (1 << 4),
	TSL277X_EN_PRX_IRQ  = (1 << 5),
	TSL277X_EN_SAI      = (1 << 6),
};

// add by ning.wei for opimize p-sensor pr743248 2014-08-21 start
enum tsl277x_ctr_reg {
	TSL277X_CTR_AGAIN    = (3 << 0),
	TSL277X_CTR_PGAIN    = (3 << 2),
	TSL277X_CTR_PDIODE   = (3 << 4),
	TSL277X_CTR_PDRIVE   = (3 << 6),
};
// mod by ning.wei for opimize p-sensor pr743248 2014-08-21 end

enum tsl277x_status {
	TSL277X_ST_ALS_VALID  = (1 << 0),
	TSL277X_ST_PRX_VALID  = (1 << 1),
	TSL277X_ST_ALS_IRQ    = (1 << 4),
	TSL277X_ST_PRX_IRQ    = (1 << 5),
	TSL277X_ST_PRX_SAT    = (1 << 6),
};

enum {
	TSL277X_ALS_GAIN_MASK = (3 << 0),
	TSL277X_ALS_AGL_MASK  = (1 << 2),
	TSL277X_ALS_AGL_SHIFT = 2,
	TSL277X_ATIME_PER_100 = 273,
	TSL277X_ATIME_DEFAULT_MS = 50,
	SCALE_SHIFT = 11,
	RATIO_SHIFT = 10,
	MAX_ALS_VALUE = 0xffff,
	MIN_ALS_VALUE = 10,
	GAIN_SWITCH_LEVEL = 100,
	GAIN_AUTO_INIT_VALUE = 16,
};

static u8 const tsl277x_ids[] = {
	0x39,
	0x30,
};

static char const *tsl277x_names[] = {
	"tsl27721 / tsl27725",
	"tsl27723 / tsl2777",
};

static u8 const restorable_regs[] = {
	TSL277X_ALS_TIME,
	TSL277X_PRX_TIME,
	TSL277X_WAIT_TIME,
	TSL277X_PERSISTENCE,
	TSL277X_CONFIG,
	TSL277X_PRX_PULSE_COUNT,
	TSL277X_CONTROL,
	TSL277X_REG_PRX_OFFS,
};

static u8 const als_gains[] = {
	1,
	8,
	16,
	120
};
/* [BUFFIX]-Mod- by TCTNB.XQJ,PR-804886, 2014/10/23, p sensor debug*/
static u8 const prox_gains[] = {
	1,
	2,
	4,
	8
};
/* [BUFFIX]-End- by TCTNB.XQJ*/
struct taos_als_info {
	int ch0;
	int ch1;
	u32 cpl;
	u32 saturation;
	int lux;
};

struct taos_prox_info {
	int raw;
	int detected;
};

static struct lux_segment segment_default[] = {
	{
		.ratio = (435 << RATIO_SHIFT) / 1000,  //445
		.k0 = (46516 << SCALE_SHIFT) / 1000,   //95264
		.k1 = (95381 << SCALE_SHIFT) / 1000,   //195340
	},
	{
		.ratio = (551 << RATIO_SHIFT) / 1000,   //564
		.k0 = (23740 << SCALE_SHIFT) / 1000,    //48619
		.k1 = (43044 << SCALE_SHIFT) / 1000,    //88154
	},
};

// add by ning.wei for opimize p-sensor pr743248 2014-08-21 start 
#ifdef USE_TRACEBILITY

#define TRACEBILITY_BASE 0x8380000
#define TRACE_OFFSET 31
#define PSENSOR_OFFSET 381

#define TRACE_TAG 0x4646
#define DEF_TH_TAG 0x7878
#define CALI_DATA_TAG 0x9595

struct factory_thredhold {
	int tag;
	int HT;
	int LT;
}__attribute__ ((__packed__));

struct factory_cali_cfg {
	int tag;
	int ppcount;
	u8 ctrl;
}__attribute__ ((__packed__));

struct trace_data {
	int tag;
	struct factory_thredhold factory_def_thredhold;
	struct factory_cali_cfg  factory_cali_data;
}__attribute__ ((__packed__));
#endif

// add by ning.wei for opimize p-sensor pr743248 2014-08-21 end

struct tsl2772_chip {
	struct mutex lock;
    struct mutex tsl2772_i2c_lock;
	struct i2c_client *client;
	struct taos_prox_info prx_inf;
	struct taos_als_info als_inf;
	struct taos_parameters params;
	struct tsl2772_i2c_platform_data *pdata;
	u8 shadow[TSL277X_REG_MAX];
	char const *prox_name;
	char const *als_name;
	struct input_dev *p_idev;
	struct input_dev *a_idev;
    int irq;
	int in_suspend;
	int wake_irq;
	int irq_pending;
	bool unpowered;
	bool als_enabled;
	bool prx_enabled;
	struct lux_segment *segment;
	int segment_num;
	int seg_num_max;
	bool als_gain_auto;
#ifdef POWER_REGULATOR
	struct regulator *vdd;
	struct regulator *vio;
	bool power_enabled; 
#endif
    bool als_can_wake;
    bool proximity_can_wake;

#ifdef ALS_POLL
    struct hrtimer als_timer;		
    ktime_t als_poll_delay;	       
    struct work_struct taos_als_work;	
    struct workqueue_struct *taos_als_wq;  
    //struct wake_lock taos_wakelock;    
    struct wake_lock taos_nosuspend_wl;
    struct mutex io_als_lock;
    bool als_enable_pre;
    bool prx_enable_pre;
#endif
// add by ning.wei for opimize p-sensor pr743248 2014-08-21 start
#ifdef USE_TRACEBILITY
    struct delayed_work trace_work;
    struct trace_data factory_data;
#endif
// add by ning.wei for opimize p-sensor pr743248 2014-08-21 end
	struct sensors_classdev als_cdev;
	struct sensors_classdev ps_cdev;
/* [BUFFIX]-Add- Begin by TCTNB.XQJ,PR-886227, 2015/2/2,add a lock, and initize int gpio*/
	struct pinctrl *pinctrl;
	struct pinctrl_state *pin_default;
	struct pinctrl_state *pin_sleep;
	struct work_struct taos_int_work;
       struct workqueue_struct *taos_int_wq;
	struct wake_lock taos_int_wl;// this lock for  avoiding  enter sleep before ps data report done.
/* [BUFFIX]-End- by TCTNB.XQJ*/
};
/*TR961495 modify by ZXZ ,2015/03/28  change min_delay from 0 to 200000 for framwork get data rate*/
static struct sensors_classdev sensors_light_cdev = {
	.name = ALS_NAME,
	.vendor = "tsl2772",
	.version = 1,
	.handle = SENSORS_LIGHT_HANDLE,
	.type = SENSOR_TYPE_LIGHT,
	.max_range = "60000",
	.resolution = "0.0125",
	.sensor_power = "0.20",
	.min_delay = 200000, /* in microseconds */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

static struct sensors_classdev sensors_proximity_cdev = {
	.name = PS_NAME,
	.vendor = "tsl2772",
	.version = 1,
	.handle = SENSORS_PROXIMITY_HANDLE,
	.type = SENSOR_TYPE_PROXIMITY,
	.max_range = "5",
	.resolution = "5.0",
	.sensor_power = "3",
	.min_delay = 0, /* in microseconds */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

/*[BUFFIX]-Mod-Begin- by TCTNB.ZXZ,PR-994478, 2015/05/08, add enable and disable irq in one function*/
static void taos_irq_enable(int irq, bool on, bool nosync)
{
       if (on) {
           if (!irq_enable_status) {
               enable_irq(irq);
               irq_enable_status = true;
           } else {
               printk("taos_irq_enable: tsl277x has already enabled!");
           }
       } else {
           if (irq_enable_status) {
               if (nosync)
                   disable_irq_nosync(irq);
               else
                   disable_irq(irq);
		   irq_enable_status = false;
           } else {
               printk("taos_irq_enable: tsl277x has already disabled!");
           }
       }

}
/*[BUFFIX]-Mod-End by TCTNB.ZXZ*/


static int taos_als_enable(struct tsl2772_chip *chip, int on);
static int taos_prox_enable(struct tsl2772_chip *chip, int on);


static int tsl2772_als_set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	
	struct tsl2772_chip *chip = 
		container_of(sensors_cdev, struct tsl2772_chip, als_cdev);

	return taos_als_enable(chip, enable);
}

static int tsl2772_ps_set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	struct tsl2772_chip *chip = 
		container_of(sensors_cdev, struct tsl2772_chip, ps_cdev);

	return taos_prox_enable(chip, enable);
}


static int tsl2772_als_poll_delay(struct sensors_classdev *sensors_cdev,
		unsigned int delay_msec)
{
	return 0;
}



static int taos_i2c_read(struct tsl2772_chip *chip, u8 reg, u8 *val)
{
	int ret;
	s32 read;
	struct i2c_client *client = chip->client;
       /*if(chip->in_suspend == 1)
       return -EIO;*//* [BUFFIX]-Del- by TCTNB.XQJ,PR-925962, 2015/02/28,because of qcom i2c  improve,here is unnecessary*/
    mutex_lock(&chip->tsl2772_i2c_lock);
	ret = i2c_smbus_write_byte(client, (TSL277X_CMD_REG | reg));
	if (ret < 0) {
		dev_err(&client->dev, "%s: failed to write register %x\n",
				__func__, reg);
        mutex_unlock(&chip->tsl2772_i2c_lock);
		return ret;
	}
	read = i2c_smbus_read_byte(client);
	if (read < 0) {
		dev_err(&client->dev, "%s: failed to read from register %x\n",
				__func__, reg);
        mutex_unlock(&chip->tsl2772_i2c_lock);
		return ret;
	}
	*val = read;
    mutex_unlock(&chip->tsl2772_i2c_lock);
	return 0;
}

static int taos_i2c_blk_read(struct tsl2772_chip *chip,
		u8 reg, u8 *val, int size)
{
	s32 ret;
	struct i2c_client *client = chip->client;
       /*if(chip->in_suspend == 1)
        return -EIO;*//* [BUFFIX]-Del- by TCTNB.XQJ,PR-925962, 2015/02/28,because of qcom i2c  improve,here is unnecessary*/
    mutex_lock(&chip->tsl2772_i2c_lock);
	ret =  i2c_smbus_read_i2c_block_data(client,
			TSL277X_CMD_REG | TSL277X_CMD_INCR | reg, size, val);
	if (ret < 0)
		dev_err(&client->dev, "%s: failed at address %x (%d bytes)\n",
				__func__, reg, size);
    mutex_unlock(&chip->tsl2772_i2c_lock);
	return ret;
}

static int taos_i2c_write(struct tsl2772_chip *chip, u8 reg, u8 val)
{
	int ret;
	struct i2c_client *client = chip->client;
       /*if(chip->in_suspend == 1)
        return -EIO;*//* [BUFFIX]-Del- by TCTNB.XQJ,PR-925962, 2015/02/28,because of qcom i2c  improve,here is unnecessary*/
    mutex_lock(&chip->tsl2772_i2c_lock);
	ret = i2c_smbus_write_byte_data(client, TSL277X_CMD_REG | reg, val);
	if (ret < 0)
		dev_err(&client->dev, "%s: failed to write register %x\n",
				__func__, reg);
    mutex_unlock(&chip->tsl2772_i2c_lock);
	return ret;
}

static int taos_i2c_blk_write(struct tsl2772_chip *chip,
		u8 reg, u8 *val, int size)
{
	s32 ret;
	struct i2c_client *client = chip->client;
       /*if(chip->in_suspend == 1)
        return -EIO;*//* [BUFFIX]-Del- by TCTNB.XQJ,PR-925962, 2015/02/28,because of qcom i2c  improve,here is unnecessary*/
    mutex_lock(&chip->tsl2772_i2c_lock);
	ret =  i2c_smbus_write_i2c_block_data(client,
			TSL277X_CMD_REG | TSL277X_CMD_INCR | reg, size, val);
	if (ret < 0)
		dev_err(&client->dev, "%s: failed at address %x (%d bytes)\n",
				__func__, reg, size);
    mutex_unlock(&chip->tsl2772_i2c_lock);
	return ret;
}

static int set_segment_table(struct tsl2772_chip *chip,
		struct lux_segment *segment, int seg_num)
{
	int i;
	struct device *dev = &chip->client->dev;

	//chip->seg_num_max = chip->pdata->segment_num ?
	//  	chip->pdata->segment_num : ARRAY_SIZE(segment_default);
    chip->seg_num_max = ARRAY_SIZE(segment_default);
	if (!chip->segment) {
		dev_dbg(dev, "%s: allocating segment table\n", __func__);
		chip->segment = kzalloc(sizeof(*chip->segment) *
				chip->seg_num_max, GFP_KERNEL);
		if (!chip->segment) {
			dev_err(dev, "%s: no memory!\n", __func__);
			return -ENOMEM;
		}
	}
	if (seg_num > chip->seg_num_max) {
		dev_warn(dev, "%s: %d segment requested, %d applied\n",
				__func__, seg_num, chip->seg_num_max);
		chip->segment_num = chip->seg_num_max;
	} else {
		chip->segment_num = seg_num;
	}
	memcpy(chip->segment, segment,
			chip->segment_num * sizeof(*chip->segment));
	dev_dbg(dev, "%s: %d segment requested, %d applied\n", __func__,
			seg_num, chip->seg_num_max);
	for (i = 0; i < chip->segment_num; i++)
		dev_dbg(dev, "segment %d: ratio %6u, k0 %6u, k1 %6u\n",
				i, chip->segment[i].ratio,
				chip->segment[i].k0, chip->segment[i].k1);
	return 0;
}

static void taos_calc_cpl(struct tsl2772_chip *chip) //calc CPL and saturation (80% saturation )
{
	u32 cpl; 
	u32 sat;
	u8 atime = chip->shadow[TSL277X_ALS_TIME]; 
	u8 agl = (chip->shadow[TSL277X_CONFIG] & TSL277X_ALS_AGL_MASK)  //TSL277X_ALS_AGL_MASK = 4 , TSL277X_ALS_AGL_SHIFT = 2
			>> TSL277X_ALS_AGL_SHIFT;			//confirm AGL is used ?
	u32 time_scale = (256 - atime ) * 2730 / 200;   //2730 =2.73*1000  ( CPL magnified 1000 times)
	
	
	cpl = time_scale * chip->params.als_gain;				// cpl = 2012 * 8 = 16096
	if (agl)   							//AGL function is on ? (original value is 0)
		cpl = cpl * 16 / 1000;						// cpl = 16096*16/1000 = 257
	sat = min_t(u32, MAX_ALS_VALUE, (u32)(256 - atime) << 10); 	//  (256-238)<<10 = 18432,min_t(u32,X,u32,Y) 
	sat = sat * 8 / 10;					  	// = 80% * saturation
	dev_dbg(&chip->client->dev,
			"%s: cpl = %u [time_scale %u, gain %u, agl %u], "
			"saturation %u\n", __func__, cpl, time_scale,
			chip->params.als_gain, agl, sat);
	chip->als_inf.cpl = cpl;					//CPL = chip->als_inf.cpl
	chip->als_inf.saturation = sat;					//sat = chip->als_inf.saturation
}

static int set_als_gain(struct tsl2772_chip *chip, int gain)
{
	int rc;
	u8 ctrl_reg  = chip->shadow[TSL277X_CONTROL] & ~TSL277X_ALS_GAIN_MASK;

	switch (gain) {
	case 1:
		ctrl_reg |= AGAIN_1;
		break;
	case 8:
		ctrl_reg |= AGAIN_8;
		break;
	case 16:
		ctrl_reg |= AGAIN_16;
		break;
	case 120:
		ctrl_reg |= AGAIN_120;
		break;
	default:
		dev_err(&chip->client->dev, "%s: wrong als gain %d\n",
				__func__, gain);
		return -EINVAL;
	}
	rc = taos_i2c_write(chip, TSL277X_CONTROL, ctrl_reg);
	if (!rc) {
		chip->shadow[TSL277X_CONTROL] = ctrl_reg;
		chip->params.als_gain = gain;
		dev_dbg(&chip->client->dev, "%s: new gain %d\n",
				__func__, gain);
	}
	return rc;
}

static int taos_get_lux(struct tsl2772_chip *chip)
{
	unsigned i;
	int ret = 0;
	struct device *dev = &chip->client->dev;	/*????*/
	struct lux_segment *s = chip->segment;		//????//
	u32 c0 = chip->als_inf.ch0 * AMBIENT_INK_TRANSMISSION_COMPENSATE; //ambient
	u32 c1 = chip->als_inf.ch1 * IR_INK_TRANSMISSION_COMPENSATE; //IR
	u32 sat = chip->als_inf.saturation;
	u32 ratio;
	u64 lux_0, lux_1;
	u32 cpl = chip->als_inf.cpl;
	u32 lux, k0 = 0, k1 = 0;

	if (!chip->als_gain_auto) {  					//when als_gain is not  auto
									// ch0 < 10  ? lux = 0:change the als_gain
		if (c0 <= MIN_ALS_VALUE) {  				//MIN_ALS_VALUE = 10
			dev_dbg(dev, "%s: darkness\n", __func__);
			lux = 0;						
			goto exit;
		} else if (c0 >= sat) {
			dev_dbg(dev, "%s: saturation, keep lux\n", __func__);
		/*	lux = chip->als_inf.lux;
			goto exit;
		*/
		}
	} else {							//chip->als_gain_auto = "AUTO"  this is not 0 , change the  gain
		u8 gain = chip->params.als_gain;			// auto gain , 1x , 16x , 120x
		int rc = -EIO;

		if (gain == 16 && c0 >= sat) {				
			rc = set_als_gain(chip, 1);
		} else if (gain == 16 && c0 < GAIN_SWITCH_LEVEL) {	//GAIN_SWITCH_LEVEL = 100
			rc = set_als_gain(chip, 120);
		} else if ((gain == 120 && c0 >= sat) ||
				(gain == 1 && c0 < GAIN_SWITCH_LEVEL)) {
			rc = set_als_gain(chip, 16);
		}
		if (!rc) {							//confirm write gain successful
			dev_dbg(dev, "%s: gain adjusted, skip\n", __func__);	//if exception ,this will process
			taos_calc_cpl(chip);
			ret = -EAGAIN;
			lux = chip->als_inf.lux;
			goto exit;
		}

		if (c0 <= MIN_ALS_VALUE) {
			dev_dbg(dev, "%s: darkness\n", __func__);
			lux = 0;
			goto exit;
		} else if (c0 >= sat) {
			dev_dbg(dev, "%s: saturation, keep lux\n", __func__);
			lux = chip->als_inf.lux;
			goto exit;
		}
	}

	//atime = 50ms, gain = 8x , ch0 = 3891 , ch1 = 424

	ratio = (c1 << RATIO_SHIFT) / c0;						//ratio = c1 * 1024 / c0     //ratio = 111
	for (i = 0; i < chip->segment_num; i++, s++) {  
		if (ratio <= s->ratio) {
			dev_dbg(&chip->client->dev, "%s: ratio %u segment %u "
					"[r %u, k0 %u, k1 %u]\n", __func__,
					ratio, i, s->ratio, s->k0, s->k1);
			k0 = s->k0;
			k1 = s->k1;
			break;
		}
	}
	if (i >= chip->segment_num) {
		dev_dbg(&chip->client->dev, "%s: ratio %u - darkness\n",
				__func__, ratio);
		lux = 0;
		goto exit;
	}

	lux_0 = ( ( ( c0 * 100 ) - ( c1 * 175 ) ) * 10 ) / cpl; //( CPL already magnified 1000 times 	//20120830 	lux
	lux_1 = ( ( ( c0 *  63 ) - ( c1 * 100 ) ) * 10 ) / cpl; 				//20120830	lux

	//20120830	debug to verify the lux .
	//snprintf( buf, "lux_0 = 0x%16llx , lux_1 = 0x%16llx , c0 = 0x%16llx , c1 = 0x%16llx ", lux_0 , lux_1 , c0 , c1 );
	

	lux = max(lux_0, lux_1);							//20120830	lux
	lux = max(lux , (u32)0);							//20120830	lux	
 


							//LUX = 17883 or 1120031  
exit:
	dev_dbg(&chip->client->dev, "%s: lux %u (%u x %u - %u x %u) / %u\n",
		__func__, lux, k0, c0, k1, c1, cpl);
	chip->als_inf.lux = lux;
	return ret;
}


static int taos_irq_clr(struct tsl2772_chip *chip, u8 bits)
{
	int ret;
      /*if(chip->in_suspend == 1)
          return -EIO;*//* [BUFFIX]-Del- by TCTNB.XQJ,PR-925962, 2015/02/28,because of qcom i2c  improve,here is unnecessary*/
    mutex_lock(&chip->tsl2772_i2c_lock);
    ret = i2c_smbus_write_byte(chip->client, TSL277X_CMD_REG |
			TSL277X_CMD_SPL_FN | bits);
	if (ret < 0)
		dev_err(&chip->client->dev, "%s: failed, bits %x\n",
				__func__, bits);
    mutex_unlock(&chip->tsl2772_i2c_lock);
      TSLX( "%s done \n",__func__);
	return ret;
}

static void taos_get_als(struct tsl2772_chip *chip)
{
	u32 ch0, ch1;
	u8 *buf = &chip->shadow[TSL277X_ALS_CHAN0LO];

	ch0 = le16_to_cpup((const __le16 *)&buf[0]);
	ch1 = le16_to_cpup((const __le16 *)&buf[2]);
	chip->als_inf.ch0 = ch0;
	chip->als_inf.ch1 = ch1;
	dev_dbg(&chip->client->dev, "%s: ch0 %u, ch1 %u\n", __func__, ch0, ch1);
}

static void taos_get_prox(struct tsl2772_chip *chip)
{
	u8 *buf = &chip->shadow[TSL277X_PRX_LO];
	bool d = chip->prx_inf.detected;

	chip->prx_inf.raw = (buf[1] << 8) | buf[0];
	chip->prx_inf.detected =
			(d && (chip->prx_inf.raw > chip->params.prox_th_min)) ||
			(!d && (chip->prx_inf.raw > chip->params.prox_th_max));
	TSLX( "%s: raw %d, detected %d,prox_th_min=%d,prox_th_max=%d\n", __func__,
			chip->prx_inf.raw, chip->prx_inf.detected,chip->params.prox_th_min,chip->params.prox_th_max);
}

static int taos_read_all(struct tsl2772_chip *chip)
{
	struct i2c_client *client = chip->client;
	s32 ret;

	dev_dbg(&client->dev, "%s\n", __func__);
	ret = taos_i2c_blk_read(chip, TSL277X_STATUS,
			&chip->shadow[TSL277X_STATUS],
			TSL277X_PRX_HI - TSL277X_STATUS + 1);
	return (ret < 0) ? ret : 0;
}

static int update_prox_thresh(struct tsl2772_chip *chip, bool on_enable)
{
	s32 ret;
	u8 *buf = &chip->shadow[TSL277X_PRX_MINTHRESHLO];
	u16 from, to;

	if (on_enable) {
		/* zero gate to force irq */
		from = to = 0;
	} else {
		if (chip->prx_inf.detected) {
            if(ps_cali.far_away != 0)
			    from = ps_cali.far_away;
            else
                from = chip->pdata->ps_thd_l;
			to = 0xffff;
		} else {
			from = 0;
            if(ps_cali.close != 0)
			    to = ps_cali.close;
            else
                to = chip->pdata->ps_thd_h;
		}
	}
	dev_dbg(&chip->client->dev, "%s: %u - %u\n", __func__, from, to);
	*buf++ = from & 0xff;
	*buf++ = from >> 8;
	*buf++ = to & 0xff;
	*buf++ = to >> 8;
	ret = taos_i2c_blk_write(chip, TSL277X_PRX_MINTHRESHLO,
			&chip->shadow[TSL277X_PRX_MINTHRESHLO],
			TSL277X_PRX_MAXTHRESHHI - TSL277X_PRX_MINTHRESHLO + 1);
       TSLX( "%s: ps_thd_h=%d ps_thd_l=%d \n",__func__,to,from);
	return (ret < 0) ? ret : 0;
}

static int update_als_thres(struct tsl2772_chip *chip, bool on_enable)
{
	s32 ret;
	u8 *buf = &chip->shadow[TSL277X_ALS_MINTHRESHLO];
	u16 gate = chip->params.als_gate;
	u16 from, to, cur;

	cur = chip->als_inf.ch0;
	if (on_enable) {
		/* zero gate far away form current position to force an irq */
		from = to = cur > 0xffff / 2 ? 0 : 0xffff;
	} else {
		gate = cur * gate / 100;
		if (!gate)
			gate = 1;
		if (cur > gate)
			from = cur - gate;
		else
			from = 0;
		if (cur < (0xffff - gate))
			to = cur + gate;
		else
			to = 0xffff;
	}
	dev_dbg(&chip->client->dev, "%s: [%u - %u]\n", __func__, from, to);
	*buf++ = from & 0xff;
	*buf++ = from >> 8;
	*buf++ = to & 0xff;
	*buf++ = to >> 8;
	ret = taos_i2c_blk_write(chip, TSL277X_ALS_MINTHRESHLO,
			&chip->shadow[TSL277X_ALS_MINTHRESHLO],
			TSL277X_ALS_MAXTHRESHHI - TSL277X_ALS_MINTHRESHLO + 1);
	return (ret < 0) ? ret : 0;
}

static void report_prox(struct tsl2772_chip *chip)
{
	ktime_t timestamp;
	if (chip->p_idev) {
		input_report_abs(chip->p_idev, ABS_DISTANCE,
				chip->prx_inf.detected ? 0 : 1);
		timestamp = ktime_get_boottime();
		input_event(chip->p_idev,EV_SYN, SYN_TIME_SEC,ktime_to_timespec(timestamp).tv_sec);
		input_event(chip->p_idev,EV_SYN, SYN_TIME_NSEC,ktime_to_timespec(timestamp).tv_nsec);
		input_sync(chip->p_idev);
	TSLX("ps-distant:===> %s \n",chip->prx_inf.detected?"near" : "far" );        
#ifdef PRINT_TIME
    print_local_time_taos("taos_report_prx_data_time"); 
#endif
	}
}

static void report_als(struct tsl2772_chip *chip)
{
	ktime_t timestamp;
	if (chip->a_idev) {
		int rc = taos_get_lux(chip);
		if (!rc) {
			int lux = chip->als_inf.lux;
#if defined CONFIG_TCT_8X16_IDOL3
			lux =lux*10/145;
#endif
            //printk(KERN_INFO "%s lux : %d@@devname :%s runtime_status : %d",__func__,lux,dev_name(&(chip->client->adapter->dev)),chip->client->adapter->dev.power.runtime_status);
        #ifdef ALS_TUNE_AUTOBRIGHTNESS
        mutex_lock(&chip->io_als_lock);
            if(lux <= TAOS_ALS_THD_DARK)
            {   if(taos_dark_code_flag > 20)
                {
                    lux = 0;
                }
                else if(taos_dark_code_flag %2 == 0)
                {
                    lux = 1;
                    taos_dark_code_flag++;
                }
                else
                {
                    lux = 0;
                    taos_dark_code_flag++;
                }
               
                input_report_abs(chip->a_idev, ABS_MISC, lux);
		timestamp = ktime_get_boottime();
		input_event(chip->a_idev,EV_SYN, SYN_TIME_SEC,ktime_to_timespec(timestamp).tv_sec);
		input_event(chip->a_idev,EV_SYN, SYN_TIME_NSEC,ktime_to_timespec(timestamp).tv_nsec);
                input_sync(chip->a_idev); 
                mutex_unlock(&chip->io_als_lock);
                return;
            }
            else if((taos_dark_code_flag!=0) && (lux < TAOS_ALS_THD_BRIGHT))
            {
                input_report_abs(chip->a_idev, ABS_MISC, 0);
		timestamp = ktime_get_boottime();
		input_event(chip->a_idev,EV_SYN, SYN_TIME_SEC,ktime_to_timespec(timestamp).tv_sec);
		input_event(chip->a_idev,EV_SYN, SYN_TIME_NSEC,ktime_to_timespec(timestamp).tv_nsec);
                input_sync(chip->a_idev); 
                mutex_unlock(&chip->io_als_lock);
                return;
            }
            else
            {
                taos_dark_code_flag = 0;
            }
            mutex_unlock(&chip->io_als_lock);
        #endif
			input_report_abs(chip->a_idev, ABS_MISC, lux);
			timestamp = ktime_get_boottime();
			input_event(chip->a_idev,EV_SYN, SYN_TIME_SEC,ktime_to_timespec(timestamp).tv_sec);
			input_event(chip->a_idev,EV_SYN, SYN_TIME_NSEC,ktime_to_timespec(timestamp).tv_nsec);
			input_sync(chip->a_idev);
			update_als_thres(chip, 0);
		} else {
			update_als_thres(chip, 1);
		}
	}
}

static int taos_check_and_report(struct tsl2772_chip *chip)
{
	u8 status;
    
	int ret = taos_read_all(chip);
	if (ret)
		goto exit_clr;
      TSLX( "%s \n",__func__);
	status = chip->shadow[TSL277X_STATUS];
	dev_dbg(&chip->client->dev, "%s: status 0x%02x\n", __func__, status);
    
	if ((status & (TSL277X_ST_PRX_VALID | TSL277X_ST_PRX_IRQ)) ==
			(TSL277X_ST_PRX_VALID | TSL277X_ST_PRX_IRQ)) {
		taos_get_prox(chip);
		report_prox(chip);
		update_prox_thresh(chip, 0);     
	}
#ifndef ALS_POLL
	if ((status & (TSL277X_ST_ALS_VALID | TSL277X_ST_ALS_IRQ)) ==
			(TSL277X_ST_ALS_VALID | TSL277X_ST_ALS_IRQ)) {
		taos_get_als(chip);
		report_als(chip);    
	}
#endif
exit_clr: 
	taos_irq_clr(chip, TSL277X_CMD_PROX_INT_CLR | TSL277X_CMD_ALS_INT_CLR);
	return ret;
}
/* [BUFFIX]-Mod- Begin by TCTNB.XQJ,PR-886227, 2015/2/2, add lock to avoid enter sleep before report psensor data,and use thread replace direct call in irq function */
static void taos_int_work_func(struct work_struct *work)
{
      struct tsl2772_chip *chip = container_of(work, struct tsl2772_chip, taos_int_work);
      mutex_lock(&chip->lock);
      TSLX( "%s \n",__func__);
      taos_check_and_report(chip);
      mutex_unlock(&chip->lock);
      wake_unlock(&chip->taos_int_wl);
      return;
}
static irqreturn_t taos_irq(int irq, void *handle)
{
      struct tsl2772_chip *chip = handle;
      TSLX( "taos_irq in_suspend=%d\n",chip->in_suspend);
      wake_lock_timeout(&chip->taos_int_wl, 3*HZ);
/*[BUFFIX]-Mod-Begin- by TCTNB.ZXZ,PR-994478, 2015/05/08, add enable and disable irq in one function*/
taos_irq_enable(chip->client->irq,false,true);
/*[BUFFIX]-Mod-End by TCTNB.ZXZ*/
      queue_work(chip->taos_int_wq, &chip->taos_int_work);
/*[BUFFIX]-Mod-Begin- by TCTNB.ZXZ,PR-994478, 2015/05/08, add enable and disable irq in one function*/
taos_irq_enable(chip->client->irq,true,false);
/*[BUFFIX]-Mod-End by TCTNB.ZXZ*/
	return IRQ_HANDLED;
}
/* [BUFFIX]-End-n by TCTNB.XQJ*/
#ifdef ALS_POLL
static int taos_als_enable(struct tsl2772_chip *chip, int on);
static int flush_regs(struct tsl2772_chip *chip);


static enum hrtimer_restart taos_als_timer_func(struct hrtimer *timer)
{
	struct tsl2772_chip *chip = container_of(timer, struct tsl2772_chip, als_timer);
    //flush_regs(chip);
	queue_work(chip->taos_als_wq, &chip->taos_als_work);
	hrtimer_forward_now(&chip->als_timer, chip->als_poll_delay);
	return HRTIMER_RESTART;
}


static void taos_als_work_func(struct work_struct *work)
{	
    struct tsl2772_chip *chip = container_of(work, struct tsl2772_chip, taos_als_work);    
    int ret;        
	u8 prx_raw_lo,prx_raw_hi,als_raw_lo,als_raw_hi;    

   if(true==tslx_log_state && chip->prx_enabled==true ){/* [BUFFIX]-Add- by TCTNB.XQJ,PR-925962, 2015/02/28,for slave log information in debug state */	
	taos_i2c_read(chip, TSL277X_PRX_LO, &prx_raw_lo);
	taos_i2c_read(chip, TSL277X_PRX_HI, &prx_raw_hi);
	taos_i2c_read(chip, TSL277X_ALS_CHAN0LO, &als_raw_lo);
	taos_i2c_read(chip, TSL277X_ALS_CHAN0HI, &als_raw_hi);
	//printk("als_raw=%d, ps_raw_data=%d\n",(als_raw_hi << 8 | als_raw_lo),(prx_raw_lo | prx_raw_hi << 8));
}
    //if(chip->in_suspend == 1)
	if(chip->in_suspend == 1 || chip->wake_irq == 1)/* [BUGFIX]-Mod-by TCTNB.XQJ, 2014/12/30, BUG 888563,for remove disable and enable psensor and light sensor ,also make p-sensor, can wake up system */
        return ;
    if(chip->als_enabled != true)    
    {       
        msleep(500);        
        taos_als_enable(chip,1);    
    }    
    mutex_lock(&chip->lock);    
    wake_lock_timeout(&chip->taos_nosuspend_wl, 3*HZ);    
    //flush_regs(chip);    
    ret = taos_read_all(chip); 
    if (ret)    
    {        
        printk(KERN_INFO "%s,taos_read_all error",__func__);        
        mutex_unlock(&chip->lock);        
        return ;    
    }    
    taos_get_als(chip); 
    report_als(chip);       
    wake_unlock(&chip->taos_nosuspend_wl); /* [BUGFIX]-Mod-by TCTNB.XQJ, 2014/12/30, BUG 888563,for remove disable and enable psensor and light sensor ,also make p-sensor, can wake up system */  
    mutex_unlock(&chip->lock);
}

#endif
static void set_pltf_settings(struct tsl2772_chip *chip)
{
	struct taos_raw_settings const *s = NULL; //s == NULL;
	u8 *sh = chip->shadow;
	struct device *dev = &chip->client->dev;

	if (s) {
		dev_dbg(dev, "%s: form pltf data\n", __func__);
		sh[TSL277X_ALS_TIME] = s->als_time;
		sh[TSL277X_PRX_TIME] = s->prx_time;
		sh[TSL277X_WAIT_TIME] = s->wait_time;
		sh[TSL277X_PERSISTENCE] = s->persist;
		sh[TSL277X_CONFIG] = s->cfg_reg;
		sh[TSL277X_PRX_PULSE_COUNT] = s->prox_pulse_cnt;
		sh[TSL277X_CONTROL] = s->ctrl_reg;
		sh[TSL277X_REG_PRX_OFFS] = s->prox_offs;
	} else {
		dev_dbg(dev, "%s: use defaults\n", __func__);
		sh[TSL277X_ALS_TIME] = 238; /* ~50 ms */
		sh[TSL277X_PRX_TIME] = 255;
		sh[TSL277X_WAIT_TIME] = 238;/* [BUFFIX]-Mod- by TCTNB.XQJ,PR-957940, 2015/04/07, slove psenosr red hot problem ,insert wait cycle*/
		sh[TSL277X_PERSISTENCE] = PRX_PERSIST(1) | ALS_PERSIST(3);
		sh[TSL277X_CONFIG] = 0;
		sh[TSL277X_PRX_PULSE_COUNT] = chip->pdata->ps_ppcount;	// modify by ning.wei for pr743248 2014-07-18
		/*sh[TSL277X_CONTROL] = AGAIN_8 | PGAIN_4 |
				PDIOD_CH0 | PDRIVE_120MA;*/
		sh[TSL277X_CONTROL] = chip->pdata->control_reg;
		sh[TSL277X_REG_PRX_OFFS] = ps_cali.offset;
	}
	chip->params.als_gate = chip->pdata->als_gate;
	chip->params.als_gain = chip->pdata->als_gain;
    if(ps_cali.far_away != 0 && ps_cali.close != 0)
    {
        chip->params.prox_th_max = ps_cali.close; //add by junfeng.zhou . 
	    chip->params.prox_th_min = ps_cali.far_away; //add by junfeng.zhou . 
    }
    else
    {
        chip->params.prox_th_max = chip->pdata->ps_thd_h;
	    chip->params.prox_th_min = chip->pdata->ps_thd_l;
    }
	if (chip->pdata->als_gain) {
		chip->params.als_gain = chip->pdata->als_gain;
	} else {
		chip->als_gain_auto = true;
		chip->params.als_gain = GAIN_AUTO_INIT_VALUE;
		dev_dbg(&chip->client->dev, "%s: auto als gain.\n", __func__);
	}
	(void)set_als_gain(chip, chip->params.als_gain);
	taos_calc_cpl(chip);
}

static int flush_regs(struct tsl2772_chip *chip)
{
	unsigned i;
	int rc;
	u8 reg;

	dev_dbg(&chip->client->dev, "%s\n", __func__);
	for (i = 0; i < ARRAY_SIZE(restorable_regs); i++) {
		reg = restorable_regs[i];
		rc = taos_i2c_write(chip, reg, chip->shadow[reg]);
		if (rc) {
			dev_err(&chip->client->dev, "%s: err on reg 0x%02x\n",
					__func__, reg);
			break;
		}
	}
	return rc;
}

static int update_enable_reg(struct tsl2772_chip *chip)
{
	dev_dbg(&chip->client->dev, "%s: %02x\n", __func__,
			chip->shadow[TSL277X_ENABLE]);
	return taos_i2c_write(chip, TSL277X_ENABLE,
			chip->shadow[TSL277X_ENABLE]);
}

#ifdef POWER_REGULATOR
static int taos_power_ctl(struct tsl2772_chip *data, bool on)
{
	int ret = 0;
    //dev_info(&data->client->dev,"%s enable : %d,power_enabled = %d",__func__,on,data->power_enabled);
	if (!on && data->power_enabled) {
		if (!IS_ERR_OR_NULL(data->pinctrl)) { /* [BUFFIX]-Add-by TCTNB.XQJ,PR-886227, 2015/2/2,for int gpio configure*/
			pinctrl_select_state(data->pinctrl,data->pin_sleep);
		}
		ret = regulator_disable(data->vdd);
		if (ret) {
			dev_err(&data->client->dev,
				"Regulator vdd disable failed ret=%d\n", ret);
			return ret;
		}

		ret = regulator_disable(data->vio);
		if (ret) {
			dev_err(&data->client->dev,
				"Regulator vio disable failed ret=%d\n", ret);
			ret=regulator_enable(data->vdd);
			return ret;
		}
		data->power_enabled = on;
		dev_dbg(&data->client->dev, "tsl2772_power_ctl on=%d\n",
				on);
	} else if (on && !data->power_enabled) {
		if (!IS_ERR_OR_NULL(data->pinctrl)) { /* [BUFFIX]-Add-by TCTNB.XQJ,PR-886227, 2015/2/2,for int gpio configure*/
			pinctrl_select_state(data->pinctrl,data->pin_default);
		}

		ret = regulator_enable(data->vdd);
		if (ret) {
			dev_err(&data->client->dev,
				"Regulator vdd enable failed ret=%d\n", ret);
			return ret;
		}

		ret = regulator_enable(data->vio);
		if (ret) {
			dev_err(&data->client->dev,
				"Regulator vio enable failed ret=%d\n", ret);
			regulator_disable(data->vdd);
			return ret;
		}
		data->power_enabled = on;
		dev_dbg(&data->client->dev, "tsl2772_power_ctl on=%d\n",
				on);
	} else {
		dev_warn(&data->client->dev,
				"Power on=%d. enabled=%d\n",
				on, data->power_enabled);
	}

	return ret;
}

static int taos_power_init(struct tsl2772_chip *data, bool on)
{
	int ret;

	if (!on) {
		if (regulator_count_voltages(data->vdd) > 0)
			regulator_set_voltage(data->vdd,
					0, TSL2772_VDD_MAX_UV);

		regulator_put(data->vdd);

		if (regulator_count_voltages(data->vio) > 0)
			regulator_set_voltage(data->vio,
					0, TSL2772_VIO_MAX_UV);

		regulator_put(data->vio);
	} else {
		data->vdd = regulator_get(&data->client->dev, "vdd");
		if (IS_ERR(data->vdd)) {
			ret = PTR_ERR(data->vdd);
			dev_err(&data->client->dev,
				"Regulator get failed vdd ret=%d\n", ret);
			return ret;
		}

		if (regulator_count_voltages(data->vdd) > 0) {
			ret = regulator_set_voltage(data->vdd,
					TSL2772_VDD_MIN_UV,
					TSL2772_VDD_MAX_UV);
			if (ret) {
				dev_err(&data->client->dev,
					"Regulator set failed vdd ret=%d\n",
					ret);
				goto reg_vdd_put;
			}
		}

		data->vio = regulator_get(&data->client->dev, "vio");
		if (IS_ERR(data->vio)) {
			ret = PTR_ERR(data->vio);
			dev_err(&data->client->dev,
				"Regulator get failed vio ret=%d\n", ret);
			goto reg_vdd_set;
		}

		if (regulator_count_voltages(data->vio) > 0) {
			ret = regulator_set_voltage(data->vio,
					TSL2772_VIO_MIN_UV,
					TSL2772_VIO_MAX_UV);
			if (ret) {
				dev_err(&data->client->dev,
				"Regulator set failed vio ret=%d\n", ret);
				goto reg_vio_put;
			}
		}
	}

	return 0;

reg_vio_put:
	regulator_put(data->vio);
reg_vdd_set:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, TSL2772_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return ret;
}

static int taos_device_ctl(struct tsl2772_chip *ps_data, bool enable)
{
	int ret;
	struct device *dev = &ps_data->client->dev;

	if (enable && !ps_data->power_enabled) {
		ret = taos_power_ctl(ps_data, true);
		if (ret) {
			dev_err(dev, "Failed to enable device power\n");
			goto err_exit;
		}
		mdelay(10);	// add by ning.wei for pr743248 2014-07-18
		flush_regs(ps_data);

	} else if (!enable && ps_data->power_enabled) {
		if (!ps_data->als_enabled && !ps_data->prx_enabled) {
			ret = taos_power_ctl(ps_data, false);
			if (ret) {
				dev_err(dev, "Failed to disable device power\n");
				goto err_exit;
			}
		} else {
			dev_dbg(dev, "device control: als_enabled=%d, ps_enabled=%d\n",
				ps_data->als_enabled, ps_data->prx_enabled);
		}
	} else {
		dev_dbg(dev, "device control: enable=%d, power_enabled=%d\n",
			enable, ps_data->power_enabled);
	}
	return 0;

err_exit:
	return ret;
}
#endif
//modify(add) by junfeng.zhou.sz for add power supply end . 

//modify(add) by junfeng.zhou.sz for cali ps begin . 
#ifdef CALI_EVERY_TIME
int taos_read_ps(struct i2c_client *client, u16 *data)
{
    struct tsl2772_chip *chip = i2c_get_clientdata(client);
	u8 prx_raw_lo,prx_raw_hi;
    int rc;
	if(client == NULL)
	{
		printk(KERN_INFO "CLIENT CANN'T EQUL NULL\n");
		return -1;
	}
    
    rc = taos_i2c_read(chip, TSL277X_PRX_LO, &prx_raw_lo);
	if(rc)
	{
		printk(KERN_ERR "%s fail, rc=%d", __func__, rc);	
		return rc;
	}
    rc = taos_i2c_read(chip, TSL277X_PRX_HI, &prx_raw_hi);
    if(rc)
	{
		printk(KERN_ERR "%s fail, rc=%d", __func__, rc);	
		return rc;
	}

	*data = prx_raw_lo | prx_raw_hi << 8 ;
	//printk(KERN_INFO "%s:ps_data=%d, low:%d  high:%d",__func__, *data, prx_raw_lo, prx_raw_hi);
	return 0;    
}


static int taos_read_cali_val(struct i2c_client *client,int n)
{
	int i=0,err=0;
	u16 data[21],sum,data_cali=0,max = 0,min = 0;
	sum = 0;

	for(i = 0;i<n+2;i++)
	{
        	err = taos_read_ps(client,&data[i]); 
		if(err != 0)
		{
			printk(KERN_INFO "tmd2772_read_data_for_cali fail: %d\n", i); 
			break;
		}
		else
		{
			if(i == 0)
			{
				max = min = data[0];
			}
			if(data[i] > max)
			{
				max = data[i];
			}
			if(data[i] < min)
			{
				min = data[i];
			}
			sum += data[i];
		}
		mdelay(10);//160
	}

	if ((max-min) > 150)
	{
		goto EXIT_ERR;
	}

	if(i==n+2)
	{
		data_cali = (sum - max - min)/n;
		//printk(KERN_INFO "%s sum=%d\t n=%d\t  data = %d\n",__func__, sum,n,data_cali);
	}else
	{
		goto EXIT_ERR;
	}
	return data_cali;

EXIT_ERR:
	printk(KERN_INFO "tmd2772_read_cali_val fail\n");
	return -1;
	
}

// add by ning.wei for opimize p-sensor calibration pr743248 2014-07-18 start
static int taos_backup_reg_for_cali(struct i2c_client *client, struct taos_raw_settings *reg_bak)
{
	int rc=0;
	struct tsl2772_chip *chip = i2c_get_clientdata(client); 

	// back up some reg value
	rc = taos_i2c_read(chip, TSL277X_ENABLE, &reg_bak->reg_enable);
	if(rc)
	{
		printk(KERN_ERR "%s fail, rc=%d", __func__, rc);	
		return rc;
	}

	rc = taos_i2c_read(chip, TSL277X_ALS_TIME, &reg_bak->als_time);
	if(rc)
	{
		printk(KERN_ERR "%s fail, rc=%d", __func__, rc);	
		return rc;
	}

	rc = taos_i2c_read(chip, TSL277X_PRX_TIME, &reg_bak->prx_time);
	if(rc)
	{
		printk(KERN_ERR "%s fail, rc=%d", __func__, rc);	
		return rc;
	}

	rc = taos_i2c_read(chip, TSL277X_WAIT_TIME, &reg_bak->wait_time);
	if(rc)
	{
		printk(KERN_ERR "%s fail, rc=%d", __func__, rc);	
		return rc;
	}

	rc = taos_i2c_read(chip, TSL277X_REG_PRX_OFFS, &reg_bak->prox_offs);
	if(rc)
	{
		printk(KERN_ERR "%s fail, rc=%d", __func__, rc);	
		return rc;
	}

	//printk("TSL277X_ENABLE=%x , TSL277X_ALS_TIME=%x , TSL277X_PRX_TIME=%x , TSL277X_WAIT_TIME=%x , TSL277X_REG_PRX_OFFS=%x\n", reg_bak->reg_enable, reg_bak->als_time, reg_bak->prx_time, reg_bak->wait_time, reg_bak->prox_offs);

	return rc;
}

static int taos_restore_reg_for_cali(struct i2c_client *client, struct taos_raw_settings *reg_bak)
{
	int rc=0;
	struct tsl2772_chip *chip = i2c_get_clientdata(client); 

	rc = taos_i2c_write(chip, TSL277X_ALS_TIME, reg_bak->als_time);
	if(rc)
	{
		printk(KERN_ERR "%s fail, rc=%d", __func__, rc);	
		return rc;
	}

	rc = taos_i2c_write(chip, TSL277X_PRX_TIME, reg_bak->prx_time);
	if(rc)
	{
		printk(KERN_ERR "%s fail, rc=%d", __func__, rc);	
		return rc;
	}

	rc = taos_i2c_write(chip, TSL277X_WAIT_TIME, reg_bak->wait_time);
	if(rc)
	{
		printk(KERN_ERR "%s fail, rc=%d", __func__, rc);	
		return rc;
	}

	rc = taos_i2c_write(chip, TSL277X_ENABLE, reg_bak->reg_enable);
	if(rc)
	{
		printk(KERN_ERR "%s fail, rc=%d", __func__, rc);	
		return rc;
	}

	return rc;
}

static int taos_config_reg_for_cali(struct i2c_client *client)
{
	int rc=0;
	struct tsl2772_chip *chip = i2c_get_clientdata(client);

	rc = taos_i2c_write(chip, TSL277X_REG_PRX_OFFS, 0x00);
	if(rc)
	{
		printk(KERN_ERR "%s fail, rc=%d", __func__, rc);	
		return rc;
	}

	rc = taos_i2c_write(chip, TSL277X_ALS_TIME, 0xff);
	if(rc)
	{
		printk(KERN_ERR "%s fail, rc=%d", __func__, rc);	
		return rc;
	}
	rc = taos_i2c_write(chip, TSL277X_PRX_TIME, 0xff);
	if(rc)
	{
		printk(KERN_ERR "%s fail, rc=%d", __func__, rc);	
		return rc;
	}
	rc = taos_i2c_write(chip, TSL277X_WAIT_TIME, 0xff);
	if(rc)
	{
		printk(KERN_ERR "%s fail, rc=%d", __func__, rc);	
		return rc;
	}

	rc = taos_i2c_write(chip, TSL277X_ENABLE, 0x07);
	if(rc)
	{
		printk(KERN_ERR "%s fail, rc=%d", __func__, rc);	
		return rc;
	}

	return rc;
}

#define MAX_IR(als_time) ((256 - als_time) * 1024)

static int taos_is_sunshine(struct i2c_client *client)
{
	u8 als_time = 0;
	u8 ch1_lo = 0;
	u8 ch1_hi = 0;
	u16 ch1_val = 0;
	int ret = 0;
	struct tsl2772_chip *chip = i2c_get_clientdata(client);

	taos_i2c_read(chip, TSL277X_ALS_TIME, &als_time);

	taos_i2c_read(chip, TSL277X_ALS_CHAN1LO, &ch1_lo);

	taos_i2c_read(chip, TSL277X_ALS_CHAN1HI, &ch1_hi);
	
	ch1_val = ch1_lo | (ch1_hi<<8);

	if (ch1_val > (MAX_IR(als_time)*90/100))
	{
		ret = 1;
	}
	TSLX("ret=%d,ch1_val=%d,ch1_lo=%d,ch1_hi=%d,als_time=%d\n",ret,ch1_val,ch1_lo,ch1_hi,als_time);
	return ret;	
}
/* [BUFFIX]-Mod-Begin by TCTNB.XQJ,PR-804886, 2014/10/23,offset value compute ways modify,
From FAE,and modify range which not need been writen offset value to register
*/
static int taos_offset_cali(struct i2c_client *client, struct PS_CALI_DATA_STRUCT *ps_data_cali,int n)
{
	u8 databuf[2]={0};  //databuf[0] read,databuf[1] write ;
	int data_cali=0;
	int rc=0;
	struct taos_raw_settings reg_bak;

	struct tsl2772_chip *chip = i2c_get_clientdata(client);    
	u8 prox_gainidx= (chip->pdata->control_reg&0x0c) >>2;/* [BUFFIX]-Mod- by TCTNB.XQJ,PR-804886, 2014/10/23, p sensor debug*/
	taos_backup_reg_for_cali(client, &reg_bak);
    
	taos_config_reg_for_cali(client);
	mdelay(60);/* [BUFFIX]-Mod- by TCTNB.XQJ,PR-870118, 2014/12/11, check if in sun shine envoriment to fellow place,for obtaining enough time to let light sensor startup normal*/
	data_cali = taos_read_cali_val(client,n);
	TSLX("data_cali=%d,prox_gain=%d\n ",data_cali,prox_gains[prox_gainidx]);
	if((data_cali>=0)&&(data_cali<50))
	{   
		databuf[1] = 0X80 | min(150*8/5/chip->pdata->ps_ppcount/prox_gains[prox_gainidx], 0x7f);
        rc = taos_i2c_write(chip, TSL277X_ENABLE, 0x06);
		rc = taos_i2c_write(chip, TSL277X_REG_PRX_OFFS, databuf[1]);
    		if(rc)
    		{
    			printk(KERN_ERR "%s fail, rc=%d", __func__, rc);	
    			return rc;
    		}
        rc = taos_i2c_write(chip, TSL277X_ENABLE, 0x07);
		mdelay(25);
		data_cali = taos_read_cali_val(client,n);
	} else if((data_cali>=50)&&(data_cali<120)){   
		databuf[1] = 0X80 | min(100*8/5/chip->pdata->ps_ppcount/prox_gains[prox_gainidx], 0x7f);
		 rc = taos_i2c_write(chip, TSL277X_ENABLE, 0x06);
		rc = taos_i2c_write(chip, TSL277X_REG_PRX_OFFS, databuf[1]);
    		if(rc)
    		{
    			printk(KERN_ERR "%s fail, rc=%d", __func__, rc);	
    			return rc;
    		}
         rc = taos_i2c_write(chip, TSL277X_ENABLE, 0x07);
		mdelay(25);
		data_cali = taos_read_cali_val(client,n);
	} else if((data_cali>=300)&&(data_cali<400)){
		databuf[1] = 0X00 | min(150*8/5/chip->pdata->ps_ppcount/prox_gains[prox_gainidx], 0x7f);/* [BUFFIX]-Mod-by TCTNB.XQJ,PR-877928, 2014/12/24,correct the right offset value,here should be - not +*/
		rc = taos_i2c_write(chip, TSL277X_ENABLE, 0x06);
		rc = taos_i2c_write(chip, TSL277X_REG_PRX_OFFS, databuf[1]);
    		if(rc)
    		{
    			printk(KERN_ERR "%s fail, rc=%d", __func__, rc);	
    			return rc;
    		}
	        rc = taos_i2c_write(chip, TSL277X_ENABLE, 0x07);
		mdelay(25);
		data_cali = taos_read_cali_val(client,n);
	} else if((data_cali>=400)&&(data_cali<500)){
		databuf[1] = 0X00 | min(250*8/5/chip->pdata->ps_ppcount/prox_gains[prox_gainidx], 0x7f);
		 rc = taos_i2c_write(chip, TSL277X_ENABLE, 0x06);
		rc = taos_i2c_write(chip, TSL277X_REG_PRX_OFFS, databuf[1]);
    		if(rc)
    		{
    			printk(KERN_ERR "%s fail, rc=%d", __func__, rc);	
    			return rc;
    		}
		rc = taos_i2c_write(chip, TSL277X_ENABLE, 0x07);
		mdelay(25);
		data_cali = taos_read_cali_val(client,n);
	}
	else if((data_cali>=500)&&(data_cali<600)){
		databuf[1] = 0X00 | min(350*8/5/chip->pdata->ps_ppcount/prox_gains[prox_gainidx], 0x7f);
		 rc = taos_i2c_write(chip, TSL277X_ENABLE, 0x06);
		rc = taos_i2c_write(chip, TSL277X_REG_PRX_OFFS, databuf[1]);
    		if(rc)
    		{
    			printk(KERN_ERR "%s fail, rc=%d", __func__, rc);	
    			return rc;
    		}
		rc = taos_i2c_write(chip, TSL277X_ENABLE, 0x07);
		mdelay(25);
		data_cali = taos_read_cali_val(client,n);
	} else if((data_cali>=600)&&(data_cali<700)){    
		databuf[1] = 0X00 | min(450*8/5/chip->pdata->ps_ppcount/prox_gains[prox_gainidx], 0x7f);
		 rc = taos_i2c_write(chip, TSL277X_ENABLE, 0x06);
		rc = taos_i2c_write(chip, TSL277X_REG_PRX_OFFS, databuf[1]);
    		if(rc)
    		{
    			printk(KERN_ERR "%s fail, rc=%d", __func__, rc);	
    			return rc;
    		}
		rc = taos_i2c_write(chip, TSL277X_ENABLE, 0x07);
		mdelay(25);
		data_cali = taos_read_cali_val(client,n);
	} else if((data_cali>=700)&&(data_cali<800)){    
		databuf[1] = 0X00 | min(550*8/5/chip->pdata->ps_ppcount/prox_gains[prox_gainidx], 0x7f);
		 rc = taos_i2c_write(chip, TSL277X_ENABLE, 0x06);
		rc = taos_i2c_write(chip, TSL277X_REG_PRX_OFFS, databuf[1]);
    		if(rc)
    		{
    			printk(KERN_ERR "%s fail, rc=%d", __func__, rc);	
    			return rc;
    		}
		rc = taos_i2c_write(chip, TSL277X_ENABLE, 0x07);
		mdelay(25);
		data_cali = taos_read_cali_val(client,n);
	} else if((data_cali>=800)&&(data_cali<900)){ 
		databuf[1] = 0X00 | min(650*8/5/chip->pdata->ps_ppcount/prox_gains[prox_gainidx], 0x7f);
		 rc = taos_i2c_write(chip, TSL277X_ENABLE, 0x06);
		rc = taos_i2c_write(chip, TSL277X_REG_PRX_OFFS, databuf[1]);
    		if(rc)
    		{
    			printk(KERN_ERR "%s fail, rc=%d", __func__, rc);	
    			return rc;
    		}
       rc = taos_i2c_write(chip, TSL277X_ENABLE, 0x07);
		mdelay(25);
		data_cali = taos_read_cali_val(client,n);
	} else if((data_cali>=900)&&(data_cali<1023)){
		databuf[1] = 0X00 | min(750*8/5/chip->pdata->ps_ppcount/prox_gains[prox_gainidx], 0x7f);
	    rc = taos_i2c_write(chip, TSL277X_ENABLE, 0x06);
		rc = taos_i2c_write(chip, TSL277X_REG_PRX_OFFS, databuf[1]);
    		if(rc)
    		{
                printk(KERN_ERR "%s fail, rc=%d", __func__, rc);
    			return rc;
    		}
        rc = taos_i2c_write(chip, TSL277X_ENABLE, 0x07);
        mdelay(25);
		data_cali = taos_read_cali_val(client,n);
	} else if(data_cali>=1023){ 
		databuf[1] = 0X00 | 0x7f;
        rc = taos_i2c_write(chip, TSL277X_ENABLE, 0x06);
		rc = taos_i2c_write(chip, TSL277X_REG_PRX_OFFS, databuf[1]);
    		if(rc)
    		{
    			printk(KERN_ERR "%s fail, rc=%d", __func__, rc);	
    			return rc;
    		}
        rc = taos_i2c_write(chip, TSL277X_ENABLE, 0x07);
		mdelay(25);
		data_cali = taos_read_cali_val(client,n);

		//printk("%s:cali read 1023,after set offset data_cali=%d\n", __func__, data_cali);
		
		if (data_cali == 0) {
			databuf[1] = 0X00 | min(680*8/5/chip->pdata->ps_ppcount/prox_gains[prox_gainidx], 0x7f);
             rc = taos_i2c_write(chip, TSL277X_ENABLE, 0x06);
			rc = taos_i2c_write(chip, TSL277X_REG_PRX_OFFS, databuf[1]);
    			if(rc)
    			{
    				printk(KERN_ERR "%s fail, rc=%d", __func__, rc);	
    				return rc;
    			}
                     rc = taos_i2c_write(chip, TSL277X_ENABLE, 0x07);
			mdelay(25);
			data_cali = taos_read_cali_val(client,n);
		}
		else if ((data_cali>0)&&(data_cali<=150)) {
			databuf[1] = 0X00 | min(750*8/5/chip->pdata->ps_ppcount/prox_gains[prox_gainidx], 0x7f);
            rc = taos_i2c_write(chip, TSL277X_ENABLE, 0x06);
			rc = taos_i2c_write(chip, TSL277X_REG_PRX_OFFS, databuf[1]);
    			if(rc)
    			{
    				printk(KERN_ERR "%s fail, rc=%d", __func__, rc);	
    				return rc;
    			}
            rc = taos_i2c_write(chip, TSL277X_ENABLE, 0x07);
			mdelay(25);
			data_cali = taos_read_cali_val(client,n);
		}
		else if ((data_cali>150)&&(data_cali<=250)) {
			databuf[1] = 0X00 | min(800*8/5/chip->pdata->ps_ppcount/prox_gains[prox_gainidx], 0x7f);
            rc = taos_i2c_write(chip, TSL277X_ENABLE, 0x06);
			rc = taos_i2c_write(chip, TSL277X_REG_PRX_OFFS, databuf[1]);
    			if(rc)
    			{
                    printk(KERN_ERR "%s fail, rc=%d", __func__, rc);
    				return rc;
    			}
           rc = taos_i2c_write(chip, TSL277X_ENABLE, 0x07);
			mdelay(25);
			data_cali = taos_read_cali_val(client,n);
		}
	}
/* [BUFFIX]-Mod- by TCTNB.XQJ,PR-870118, 2014/12/11, check if in sun shine envoriment from upper to here for obtaining enough time to let light sensor startup normal*/
	if (taos_is_sunshine(client))
	{
		goto cali_fail;
	}
/* [BUFFIX]-Mod- by TCTNB.XQJ*/
	if((data_cali>=0)&&(data_cali<=(1023-TAOS_THD_H_OFFSET)))
	{
		ps_data_cali->close =data_cali+TAOS_THD_H_OFFSET;
		ps_data_cali->far_away = data_cali+TAOS_THD_L_OFFSET;
		ps_data_cali->valid =1;
		ps_data_cali->is_have_good_cali = true;
		ps_data_cali->last_good_offset = databuf[1];
		ps_data_cali->last_good_close = ps_data_cali->close;
		ps_data_cali->last_good_far_away = ps_data_cali->far_away;
		
	}else{
cali_fail:	
		ps_data_cali->close = chip->pdata->ps_thd_h;
		ps_data_cali->far_away = chip->pdata->ps_thd_l;
		
		if (ps_data_cali->is_have_good_cali) {
			rc = taos_i2c_write(chip, TSL277X_REG_PRX_OFFS, ps_data_cali->last_good_offset);
		}
		else {
			rc = taos_i2c_write(chip, TSL277X_REG_PRX_OFFS, 0x7f);
		}

	    	if(rc)
	    	{
	    		printk(KERN_ERR "%s fail, rc=%d", __func__, rc);	
	    		return rc;
	    	}
		
	}

	ps_data_cali->offset = databuf[1];
	chip->shadow[TSL277X_REG_PRX_OFFS] = ps_data_cali->offset;
	ps_data_cali->valid = 1;
	//end write back reg 00 01 saved
	
	rc = taos_restore_reg_for_cali(client, &reg_bak);
	if(rc)
	{
		printk(KERN_ERR "%s fail, rc=%d", __func__, rc);	
		return rc;
	}

	databuf[1] = 0;
	//printk(KERN_INFO "%s: close  = %d,far_away = %d,data_cali=%d,offset = 0x%X\n",__func__,ps_data_cali->close,ps_data_cali->far_away,data_cali,ps_data_cali->offset);
	chip->params.prox_th_max = ps_data_cali->close;  //add by junfeng.zhou . 
	chip->params.prox_th_min = ps_data_cali->far_away;  //add by junfeng.zhou . 
	return rc;
}
/* [BUFFIX]-Mod-End by TCTNB.XQJ*/
// add by ning.wei for opimize p-sensor calibration pr743248 2014-07-18 end

// del by ning.wei for opimize p-sensor pr743248 2014-07-18 end
#endif

//modify(add) by junfeng.zhou.sz for cali ps end .


// add by ning.wei for opimize p-sensor pr743248 2014-08-21 start
#ifdef USE_TRACEBILITY

static int read_trace(struct tsl2772_chip *chip)
{
	struct file *filp;
	mm_segment_t old_fs;
	loff_t pos = TRACEBILITY_BASE+TRACE_OFFSET+PSENSOR_OFFSET; //traceability address
	int res = -1;
	int len = sizeof(struct trace_data);
	char buf[40]={0};

	filp = filp_open("/dev/block/mmcblk0", O_RDWR, 0);

        if (IS_ERR(filp)) {
		printk("%s filp_open err: %ld\n","/dev/block/mmcblk0", PTR_ERR(filp));
		return res;
	}
	
        old_fs = get_fs();
        set_fs(get_ds());
 

       	res = vfs_read(filp, buf, len, &pos);

        set_fs(old_fs);

        if (res == len) {
		memcpy(&chip->factory_data, buf ,sizeof(struct trace_data));

		if (chip->factory_data.tag == TRACE_TAG) {
			if (chip->factory_data.factory_def_thredhold.tag == DEF_TH_TAG) {
				chip->pdata->ps_thd_h = chip->factory_data.factory_def_thredhold.HT;
				chip->pdata->ps_thd_l = chip->factory_data.factory_def_thredhold.LT;
				printk("%s:use def thredhold read from tracebility HT=%d,LT=%d\n", __func__, chip->pdata->ps_thd_h, chip->pdata->ps_thd_l);
			}
			
	 		if (chip->factory_data.factory_cali_data.tag == CALI_DATA_TAG) {
				chip->shadow[TSL277X_PRX_PULSE_COUNT] = chip->factory_data.factory_cali_data.ppcount;
				chip->shadow[TSL277X_CONTROL] &=  ~TSL277X_CTR_PDRIVE;
				chip->shadow[TSL277X_CONTROL] |=  (TSL277X_CTR_PDRIVE & chip->factory_data.factory_cali_data.ctrl);
				printk("%s:use LED current: 0x%x,ppcount:%d\n", __func__, chip->shadow[TSL277X_CONTROL], chip->shadow[TSL277X_PRX_PULSE_COUNT]);
			}
		}
		else {	
			printk("%s:no factory data found!\n", __func__);
		}
		
	} else {
		printk("%s:read err \n", __func__);
	}
 	
        filp_close(filp, NULL);
	return res;
} 
#endif
// add by ning.wei for opimize p-sensor pr743248 2014-08-21 end
static int taos_prox_enable(struct tsl2772_chip *chip, int on)
{
	int rc;
    uint8_t curr_ps_enable; 
    u8 *buf;
    bool d ;
    u8 regsstatus;
    //judgment the current status .begin
    curr_ps_enable = chip->prx_enabled?1:0;
    if(curr_ps_enable == on)
    {
        return 0;
    }
    //judgment the current status .end
#ifdef POWER_REGULATOR
    if (on) 
    {
		rc = taos_device_ctl(chip, on);
		if (rc)
			return rc; 
    }
#endif
	if (on) {
         //add by junfeng.zhou.sz for everytime cali begin .
#ifdef CALI_EVERY_TIME
        taos_offset_cali(chip->client,&ps_cali,3);
#endif
        //add by junfeng.zhou.sz for everytime cali end .
		//update_prox_thresh(chip, 1);/* [BUFFIX]-Del- by TCTNB.XQJ,PR-925962, 2015/02/28,in fact ,this not been used,it cause confusion  */
		chip->shadow[TSL277X_ENABLE] |=
				(TSL277X_EN_PWR_ON | TSL277X_EN_PRX |TSL277X_EN_WAIT|    /* [BUFFIX]-Mod- by TCTNB.XQJ,PR-957940, 2015/04/07, slove psenosr red hot problem ,insert wait cycle*/
				TSL277X_EN_PRX_IRQ);
		rc = update_enable_reg(chip);
		TSLX(" xxxx rc=%d\n",rc);
		if (rc)
		{
			dev_err(&chip->client->dev, "%s: update_enable_reg fail , rc=%d, on=%d \n", __func__, rc, on);
			return rc;
		}
        {
            usleep_range(4000, 5000);
            rc = taos_i2c_read(chip, TSL277X_PRX_LO, &chip->shadow[TSL277X_PRX_LO]);
            rc = taos_i2c_read(chip, TSL277X_PRX_HI, &chip->shadow[TSL277X_PRX_HI]);
            buf = &chip->shadow[TSL277X_PRX_LO];
    	    d = chip->prx_inf.detected;
    	    chip->prx_inf.raw = (buf[1] << 8) | buf[0];
    	    chip->prx_inf.detected =
    			(d && (chip->prx_inf.raw > chip->params.prox_th_min)) ||
    			(!d && (chip->prx_inf.raw > chip->params.prox_th_max));
/* [BUFFIX]-Del- by TCTNB.XQJ,PR-841025, 2014/12/14,aovid data can't report to upper,remove the old solution the new is sloved in HAL*/

            report_prox(chip);
    		update_prox_thresh(chip, 0); 
        }
        taos_irq_clr(chip, TSL277X_CMD_PROX_INT_CLR | TSL277X_CMD_ALS_INT_CLR);/* [BUFFIX]-Move- by TCTNB.XQJ,PR-925962, 2015/02/28 ,move from  above move to here */
        rc = taos_i2c_read(chip, TSL277X_STATUS, &regsstatus);
        TSLX( "taos enable irq status 0x13 reg=0x%x\n",regsstatus);
/*[BUFFIX]-Mod-Begin- by TCTNB.ZXZ,PR-994478, 2015/05/08, add enable and disable irq in one function*/
	taos_irq_enable(chip->client->irq,true,false);
/*[BUFFIX]-Mod-End by TCTNB.ZXZ*/
		msleep(3);
	} else {
/* [BUGFIX]-Mod-by TCTNB.XQJ, 2014/12/30, BUG 888563,when disable psensor ,clear state*/
		chip->prx_inf.detected = 0;
		if (chip->wake_irq) {
			irq_set_irq_wake(chip->client->irq, 0);
			chip->wake_irq = 0;
		}
/* [BUGFIX]-Mod-by TCTNB.XQJ*/
           TSLX( "taos disable  irq\n");
/*[BUFFIX]-Mod-Begin- by TCTNB.ZXZ,PR-994478, 2015/05/08, add enable and disable irq in one function*/
	taos_irq_enable(chip->client->irq,false,false);
/*[BUFFIX]-Mod-End by TCTNB.ZXZ*/
		chip->shadow[TSL277X_ENABLE] &=
				~(TSL277X_EN_PRX_IRQ | TSL277X_EN_PRX);
		if (!(chip->shadow[TSL277X_ENABLE] & TSL277X_EN_ALS))
			chip->shadow[TSL277X_ENABLE] &= ~TSL277X_EN_PWR_ON;
		rc = update_enable_reg(chip);
		if (rc)
		{
			dev_err(&chip->client->dev, "%s: update_enable_reg fail , rc=%d, on=%d \n", __func__, rc, on);
		}
/*[BUFFIX]-Mod-Begin- by TCTNB.ZXZ,PR-994478, 2015/05/08, not return if update register fail */
		#if 0
		if (rc)
			return rc;
		#endif
/*[BUFFIX]-Mod-End by TCTNB.ZXZ*/
              taos_irq_clr(chip, TSL277X_CMD_PROX_INT_CLR | TSL277X_CMD_ALS_INT_CLR);
	}
    mutex_lock(&chip->lock);
/*[BUFFIX]-Mod-Begin- by TCTNB.ZXZ,PR-994478, 2015/05/08, change pSensor status if update register fail */
    #if 0
	if (!rc)
    #endif
/*[BUFFIX]-Mod-End by TCTNB.ZXZ*/
		chip->prx_enabled = on;
    mutex_unlock(&chip->lock);
#ifdef POWER_REGULATOR
	if (!on) {
		rc = taos_device_ctl(chip, on);
		if (rc)
		{
			dev_err(&chip->client->dev, "%s: taos_device_ctl  fail , rc=%d, on=%d \n", __func__,rc, on);
			return rc;
		}
	}
#endif
	return rc;
}

static int taos_als_enable(struct tsl2772_chip *chip, int on)
{
    int rc;
    uint8_t curr_als_enable; 

     curr_als_enable = chip->als_enabled?1:0;
	if(curr_als_enable == on)
	{
		return 0;
	}
	//judgment the current status .end
#ifdef POWER_REGULATOR    
	if (on) {
		rc = taos_device_ctl(chip, on);
		if (rc)
		{
			dev_err(&chip->client->dev, "%s: taos_device_ctl  fail , rc=%d, on=%d \n", __func__,rc, on);
			return rc;
		}
	}
#endif
	if (on) {
        #ifndef ALS_POLL
		taos_irq_clr(chip, TSL277X_CMD_ALS_INT_CLR);
		update_als_thres(chip, 1);
		chip->shadow[TSL277X_ENABLE] |=
				(TSL277X_EN_PWR_ON | TSL277X_EN_ALS |
				TSL277X_EN_ALS_IRQ);
        #endif
        #ifdef ALS_POLL
        chip->shadow[TSL277X_ENABLE] |=
				(TSL277X_EN_PWR_ON | TSL277X_EN_ALS );
        #endif
		rc = update_enable_reg(chip);
		if (rc)
		{
			dev_err(&chip->client->dev, "%s: update_enable_reg  fail , rc=%d, on=%d \n", __func__,rc, on);
			return rc;
		}
        #ifdef ALS_POLL
        hrtimer_start(&chip->als_timer, chip->als_poll_delay, HRTIMER_MODE_REL);
        #endif
		msleep(3);
	} else {
	    #ifdef ALS_POLL
	    hrtimer_cancel(&chip->als_timer);
        chip->shadow[TSL277X_ENABLE] &=
				~(TSL277X_EN_ALS);
        #endif
        #ifndef ALS_POLL
		chip->shadow[TSL277X_ENABLE] &=
				~(TSL277X_EN_ALS_IRQ | TSL277X_EN_ALS);
        #endif
		if (!(chip->shadow[TSL277X_ENABLE] & TSL277X_EN_PRX))
			chip->shadow[TSL277X_ENABLE] &= ~TSL277X_EN_PWR_ON;
		rc = update_enable_reg(chip);
		if (rc)
		{
			dev_err(&chip->client->dev, "%s: update_enable_reg  fail , rc=%d, on=%d \n", __func__,rc, on);
			return rc;
		}
        #ifndef ALS_POLL
		taos_irq_clr(chip, TSL277X_CMD_ALS_INT_CLR);
        #endif
	}
    mutex_lock(&chip->lock);
	if (!rc)
		chip->als_enabled = on;
    mutex_unlock(&chip->lock);
#ifdef POWER_REGULATOR
	if (!on) {
		rc = taos_device_ctl(chip, on);
		if (rc)
		{
			dev_err(&chip->client->dev, "%s: taos_device_ctl  fail , rc=%d, on=%d \n", __func__,rc, on);
			return rc;
		}
	}
#endif
	return rc;
}

static ssize_t taos_device_als_ch0(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tsl2772_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.ch0);
}

static ssize_t taos_device_als_ch1(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tsl2772_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.ch1);
}

static ssize_t taos_device_als_cpl(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tsl2772_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.cpl);
}

static ssize_t taos_device_als_lux(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tsl2772_chip *chip = dev_get_drvdata(dev);
    int rc;
    u8 als_raw_lo,als_raw_hi;
    rc = taos_i2c_read(chip, TSL277X_ALS_CHAN0LO, &als_raw_lo);
	if(rc)
	{	
		return rc;
	}
    rc = taos_i2c_read(chip, TSL277X_ALS_CHAN0HI, &als_raw_hi);
    if(rc)
	{
		return rc;
	}
	return snprintf(buf, PAGE_SIZE, "%d\n", (als_raw_hi << 8 | als_raw_lo));
}

static ssize_t taos_lux_table_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tsl2772_chip *chip = dev_get_drvdata(dev);
	struct lux_segment *s = chip->segment;
	int i, k;

	for (i = k = 0; i < chip->segment_num; i++)
		k += snprintf(buf + k, PAGE_SIZE - k, "%d:%u,%u,%u\n", i,
				(s[i].ratio * 1000) >> RATIO_SHIFT,
				(s[i].k0 * 1000) >> SCALE_SHIFT,
				(s[i].k1 * 1000) >> SCALE_SHIFT);
	return k;
}

static ssize_t taos_lux_table_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	int i;
	struct tsl2772_chip *chip = dev_get_drvdata(dev);
	u32 ratio, k0, k1;

	if (4 != sscanf(buf, "%10d:%10u,%10u,%10u", &i, &ratio, &k0, &k1))
		return -EINVAL;
	if (i >= chip->segment_num)
		return -EINVAL;
	mutex_lock(&chip->lock);
	chip->segment[i].ratio = (ratio << RATIO_SHIFT) / 1000;
	chip->segment[i].k0 = (k0 << SCALE_SHIFT) / 1000;
	chip->segment[i].k1 = (k1 << SCALE_SHIFT) / 1000;
	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t taos_tslx_log_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{	
	return snprintf(buf, PAGE_SIZE, "%d\n", tslx_log_state);
}

static ssize_t taos_tslx_log_store(struct device *dev,
	struct device_attribute *attr,const char *buf, size_t size)
{
	bool tslx_log;
	if (strtobool(buf, &tslx_log)){
		return -EINVAL;
		}
		
	tslx_log_state=!!tslx_log;
	return size;
}

static ssize_t taos_als_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tsl2772_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_enabled);
}

static ssize_t taos_als_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tsl2772_chip *chip = dev_get_drvdata(dev);
	bool value;
	if (strtobool(buf, &value))
		return -EINVAL;

#ifdef PRINT_TIME
    print_local_time_taos("taos_enabel_ALS_time"); 
#endif
    if (value)
    	taos_als_enable(chip,1);    
    else
        taos_als_enable(chip,0);    

	return size;
}

static ssize_t taos_prox_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tsl2772_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->prx_enabled);
}

static ssize_t taos_prox_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tsl2772_chip *chip = dev_get_drvdata(dev);
	bool value;

	if (strtobool(buf, &value))
		return -EINVAL;
#ifdef PRINT_TIME
    print_local_time_taos("taos_enabel_PS_time"); 
#endif
    if (value)
    	taos_prox_enable(chip,1);
    else
    	taos_prox_enable(chip,0);  

	return size;
}

static ssize_t taos_als_gain_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tsl2772_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d (%s)\n", chip->params.als_gain,
			chip->als_gain_auto ? "auto" : "manual");
}

static ssize_t taos_als_gain_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	unsigned long gain;
	int rc;
	struct tsl2772_chip *chip = dev_get_drvdata(dev);

	rc = strict_strtoul(buf, 10, &gain);
	if (rc)
		return -EINVAL;
	if (gain != 0 && gain != 1 && gain != 8 && gain != 16 && gain != 120)
		return -EINVAL;
	mutex_lock(&chip->lock);
	if (gain) {
		chip->als_gain_auto = false;
		rc = set_als_gain(chip, gain);
		if (!rc)
			taos_calc_cpl(chip);
	} else {
		chip->als_gain_auto = true;
	}
	mutex_unlock(&chip->lock);
	return rc ? rc : size;
}

static ssize_t taos_als_gate_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tsl2772_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d (in %%)\n", chip->params.als_gate);
}

static ssize_t taos_als_gate_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	unsigned long gate;
	int rc;
	struct tsl2772_chip *chip = dev_get_drvdata(dev);

	rc = strict_strtoul(buf, 10, &gate);
	if (rc || gate > 100)
		return -EINVAL;
	mutex_lock(&chip->lock);
	chip->params.als_gate = gate;
	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t taos_device_prx_raw(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tsl2772_chip *chip = dev_get_drvdata(dev);
    int rc;
    u8 prx_raw_lo,prx_raw_hi,prx_offset,prx_ppcount; // add by ning.wei for pr743248 2014-07-18
    int offset_value = 0;	

	rc = taos_i2c_read(chip, TSL277X_REG_PRX_OFFS, &prx_offset);
    if(rc)
	{
		printk(KERN_ERR "%s fail 1, rc=%d", __func__, rc);	
		return rc;
	}
	msleep(10);
	rc = taos_i2c_write(chip, TSL277X_REG_PRX_OFFS, 0x00);
	if(rc)
	{
		printk(KERN_ERR "%s fail 21, rc=%d", __func__, rc);	
		return rc;
	}	
	msleep(100);
	
	
    rc = taos_i2c_read(chip, TSL277X_PRX_LO, &prx_raw_lo);
	if(rc)
	{
		printk(KERN_ERR "%s fail, rc=%d", __func__, rc);	
		return rc;
	}
    rc = taos_i2c_read(chip, TSL277X_PRX_HI, &prx_raw_hi);
    if(rc)
	{
		printk(KERN_ERR "%s fail, rc=%d", __func__, rc);	
		return rc;
	}
	
	// modify by ning.wei for pr743248 2014-07-18 start
    rc = taos_i2c_write(chip, TSL277X_REG_PRX_OFFS, prx_offset);
    if(rc)
	{
		printk(KERN_ERR "%s fail, rc=%d", __func__, rc);	
		return rc;
	}
	msleep(100);
	
    rc = taos_i2c_read(chip, TSL277X_PRX_PULSE_COUNT, &prx_ppcount);
    if(rc)
	{
		printk(KERN_ERR "%s fail, rc=%d", __func__, rc);	
		return rc;
	}

	offset_value = 5*prx_ppcount*(prx_offset&0x7f)/8;
	offset_value = (prx_offset & 0x80) == 0 ?  offset_value : offset_value*(-1);

	TSLX("offset=%d,prx_raw=%d\n",(-1)*offset_value,(prx_raw_hi << 8 | prx_raw_lo));

	return snprintf(buf, PAGE_SIZE, "%d\n", (prx_raw_hi << 8 | prx_raw_lo));
}



/**
 * SysFS support
 */
static ssize_t taos_show_ps_sensor_thld(struct device *dev,
				struct device_attribute *attr, char *buf)
{
    struct tsl2772_chip *chip = dev_get_drvdata(dev);
	return sprintf(buf, "%d %d\n", chip->pdata->ps_thd_h, chip->pdata->ps_thd_l);
}

static ssize_t taos_store_ps_sensor_thld(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	char *next_buf;
	unsigned long hsyt_val = simple_strtoul(buf, &next_buf, 10);
	unsigned long det_val = simple_strtoul(++next_buf, NULL, 10);
	struct tsl2772_chip *chip = dev_get_drvdata(dev);

	if ((det_val < 0) || (det_val > 1023) || (hsyt_val < 0) || (hsyt_val >= det_val)) {
		printk("%s:store unvalid det_val=%ld, hsyt_val=%ld\n", __func__, det_val, hsyt_val);
		return -EINVAL;
	}
	mutex_lock(&chip->lock);
	chip->pdata->ps_thd_l= det_val;
	chip->pdata->ps_thd_h = hsyt_val;
	mutex_unlock(&chip->lock);

	return count;
}

static ssize_t taos_all_reg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    u8 ps_reg[0x22];
	u8 cnt;
	int len = 0;
    int rc ;
    struct tsl2772_chip *chip = dev_get_drvdata(dev);	
	
	for(cnt=0;cnt<0x10;cnt++)
	{
		rc = taos_i2c_read(chip, cnt, &(ps_reg[cnt]));
		if(rc)
		{
			printk(KERN_ERR "%s fail, rc=%d", __func__, rc);	
			return rc;
		}
		else
		{
			printk(KERN_INFO "reg[0x%2X]=0x%2X\n", cnt, ps_reg[cnt]);
			len += scnprintf(buf+len, PAGE_SIZE-len, "[%2X]%2X,", cnt, ps_reg[cnt]);
		}
	}
    cnt++;
    len += scnprintf(buf+len, PAGE_SIZE-len, "[%2X]-%2X-,", 0x10, 0xFF);
    
    for(;cnt<0x1A;cnt++)
	{
		rc = taos_i2c_read(chip, cnt, &(ps_reg[cnt]));
		if(rc)
		{
			printk(KERN_ERR "%s fail, rc=%d", __func__, rc);	
			return rc;
		}
		else
		{
			printk(KERN_INFO "reg[0x%2X]=0x%2X\n", cnt, ps_reg[cnt]);
			len += scnprintf(buf+len, PAGE_SIZE-len, "[%2X]%2X,", cnt, ps_reg[cnt]);
		}
	}
    cnt++;
	rc = taos_i2c_read(chip, TSL277X_REG_PRX_OFFS, &(ps_reg[cnt]));
	if(rc)
	{
		printk( KERN_ERR "%s fail, rc=%d", __func__, rc);	
		return rc;
	}		
	len += scnprintf(buf+len, PAGE_SIZE-len, "[%2X]%2X\n", 0x1e, ps_reg[cnt]);
	return len;
}

//echo 1e 7F > write_reg   //echo addr date > write_reg
static ssize_t taos_write_reg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int addr, cmd;
	int32_t ret, i;
	char *token[10];
	struct tsl2772_chip *chip = dev_get_drvdata(dev);		
	
	for (i = 0; i < 2; i++)
		token[i] = strsep((char **)&buf, " ");
	if((ret = strict_strtoul(token[0], 16, (unsigned long *)&(addr))) < 0)
	{
		printk(KERN_ERR "%s:strict_strtoul failed, ret=0x%x\n", __func__, ret);
		return ret;	
	}
	if((ret = strict_strtoul(token[1], 16, (unsigned long *)&(cmd))) < 0)
	{
		printk(KERN_ERR "%s:strict_strtoul failed, ret=0x%x\n", __func__, ret);
		return ret;	
	}	
    ret = taos_i2c_write(chip, addr, cmd);
	if (ret)
	{	
		printk(KERN_ERR "%s: taos_i2c_write fail\n", __func__);
		return ret;
	}
	
	return size;
}

static ssize_t taos_write_reg_show(struct device *dev,struct device_attribute *attr, char *buf)
{	return snprintf(buf, PAGE_SIZE, "%d\n", 0);
}

static struct device_attribute prox_attrs[] = {
	__ATTR(prx_raw, 0444, taos_device_prx_raw, NULL),
	__ATTR(enable, 0664, taos_prox_enable_show, taos_prox_enable_store),
	__ATTR(ps_sensor_thld, 0664, taos_show_ps_sensor_thld, taos_store_ps_sensor_thld),
	__ATTR(allreg, 0444, taos_all_reg_show, NULL),
	__ATTR(write_reg,0664,taos_write_reg_show, taos_write_reg_store),
	__ATTR(tslx_log, 0664, taos_tslx_log_show, taos_tslx_log_store),
};

static struct device_attribute als_attrs[] = {
	__ATTR(als_ch0, 0444, taos_device_als_ch0, NULL),
	__ATTR(als_ch1, 0444, taos_device_als_ch1, NULL),
	__ATTR(als_cpl, 0444, taos_device_als_cpl, NULL),
	__ATTR(als_lux, 0444, taos_device_als_lux, NULL),
	__ATTR(als_gain, 0664, taos_als_gain_show, taos_als_gain_store),
	__ATTR(als_gate, 0664, taos_als_gate_show, taos_als_gate_store),
	__ATTR(lux_table, 0664, taos_lux_table_show, taos_lux_table_store),
	__ATTR(enable, 0664, taos_als_enable_show, taos_als_enable_store),
	

};

static int add_sysfs_interfaces(struct device *dev,
	struct device_attribute *a, int size)
{
	int i;
	for (i = 0; i < size; i++)
		if (device_create_file(dev, a + i))
			goto undo;
	return 0;
undo:
	for (; i >= 0 ; i--)
		device_remove_file(dev, a + i);
	dev_err(dev, "%s: failed to create sysfs interface\n", __func__);
	return -ENODEV;
}

static void remove_sysfs_interfaces(struct device *dev,
	struct device_attribute *a, int size)
{
	int i;
	for (i = 0; i < size; i++)
		device_remove_file(dev, a + i);
}

static int taos_get_id(struct tsl2772_chip *chip, u8 *id, u8 *rev)
{
	int rc = taos_i2c_read(chip, TSL277X_REVID, rev);
	if (rc)
		return rc;
	return taos_i2c_read(chip, TSL277X_CHIPID, id);
}


#ifdef CONFIG_OF
static int taos_parse_dt(struct device *dev,
			struct tsl2772_i2c_platform_data *pdata)
{
	int rc;
	struct device_node *np = dev->of_node;
	u32 temp_val;

	pdata->int_pin = of_get_named_gpio_flags(np, "tsl,irq-gpio",
				0, &pdata->int_flags);
	if (pdata->int_pin < 0) {
		dev_err(dev, "Unable to read irq-gpio\n");
		return pdata->int_pin;
	}
    rc = of_property_read_u32(np, "tsl,control-reg", &temp_val);
	if (!rc)
		pdata->control_reg= temp_val;
	else {
		dev_err(dev, "Unable to read state-reg\n");
		return rc;
	}
    
	rc = of_property_read_u32(np, "tsl,als-gate", &temp_val);
	if (!rc)
		pdata->als_gate= (u8)temp_val;
	else {
		dev_err(dev, "Unable to read alsctrl-reg\n");
		return rc;
	}

	rc = of_property_read_u32(np, "tsl,als-gain", &temp_val);
	if (!rc)
		pdata->als_gain= (u8)temp_val;
	else {
		dev_err(dev, "Unable to read ledctrl-reg\n");
		return rc;
	}

	rc = of_property_read_u32(np, "tsl,wait-reg", &temp_val);
	if (!rc)
		pdata->wait_reg = (u8)temp_val;
	else {
		dev_err(dev, "Unable to read wait-reg\n");
		return rc;
	}

	rc = of_property_read_u32(np, "tsl,ps-thdh", &temp_val);
	if (!rc)
		pdata->ps_thd_h = (u16)temp_val;
	else {
		dev_err(dev, "Unable to read ps-thdh\n");
		return rc;
	}
	//pdata->ps_high_thd_def = pdata->ps_thd_h;

	rc = of_property_read_u32(np, "tsl,ps-thdl", &temp_val);
	if (!rc)
		pdata->ps_thd_l = (u16)temp_val;
	else {
		dev_err(dev, "Unable to read ps-thdl\n");
		return rc;
	}
	//pdata->ps_low_thd_def = pdata->ps_thd_l;

	// add by ning.wei for pr743248 2014-07-18 start 
	rc = of_property_read_u32(np, "tsl,ps-ppcount", &temp_val);
	if (!rc)
		pdata->ps_ppcount = (u8)temp_val;
	else {
		dev_err(dev, "Unable to read ps-thdl\n");
		return rc;
	}
	// add by ning.wei for pr743248 2014-07-18 end
	
	pdata->use_fir = of_property_read_bool(np, "tsl,use-fir");

	return 0;
}
#else
static int taos_parse_dt(struct device *dev,
			struct tsl2772_i2c_platform_data *pdata)
{
	return -ENODEV;
}
#endif /* !CONFIG_OF */

// add by ning.wei for opimize p-sensor pr743248 2014-08-21 start
#ifdef USE_TRACEBILITY
static void trace_work_func(struct work_struct *work)
{
	int ret = 0;
	struct delayed_work *dwork = to_delayed_work(work);
	struct tsl2772_chip *chip = container_of(dwork, struct tsl2772_chip, trace_work);

	ret = read_trace(chip);
	
	if (ret > 0) {
		cancel_delayed_work(dwork);
	}
	else {
		schedule_delayed_work(dwork, msecs_to_jiffies(1000));
	}
}
#endif
// add by ning.wei for opimize p-sensor pr743248 2014-08-21 end
/* [PLATFORM]-Mod-BEGIN by TCTNB.YQJ, PR-912196, 2015/01/24 */
struct input_dev* get_ido3_taos_input_device(void)
{
	return ido3_taos_proximity_dev;
}
EXPORT_SYMBOL(get_ido3_taos_input_device);
/* [PLATFORM]-Mod-END by TCTNB.YQJ */
/* [BUFFIX]-Add- Begin by TCTNB.XQJ,PR-886227, 2015/2/2,and configure int gpio*/
static int tsl2772x_pinctrl_init(struct tsl2772_chip *chip)
{
	struct i2c_client *client = chip->client;

	chip->pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR_OR_NULL(chip->pinctrl)) {
		dev_err(&client->dev, "Failed to get pinctrl\n");
		return PTR_ERR(chip->pinctrl);
	}

	chip->pin_default =
		pinctrl_lookup_state(chip->pinctrl, "default");
	if (IS_ERR_OR_NULL(chip->pin_default)) {
		dev_err(&client->dev, "Failed to look up default state\n");
		return PTR_ERR(chip->pin_default);
	}

	chip->pin_sleep =
		pinctrl_lookup_state(chip->pinctrl, "sleep");
	if (IS_ERR_OR_NULL(chip->pin_sleep)) {
		dev_err(&client->dev, "Failed to look up sleep state\n");
		return PTR_ERR(chip->pin_sleep);
	}

	return 0;
}
/* [BUFFIX]-End-by TCTNB.XQJ*/


/*[BUFFIX]-Mod-Begin- by TCTNB.ZXZ,FR-1034674, 2015/07/13, add misoperation interface */
#ifdef CONFIG_TCT_8X16_IDOL3
/* prx_status  attribute (/<sysfs>/class/prx_misoperation/status) */

static int taos_prox_enable_nocal(struct tsl2772_chip *chip, int on)
{
	int rc;
    uint8_t curr_ps_enable;

	curr_ps_enable = chip->prx_enabled?1:0;
	if(curr_ps_enable == on)
		return 0;
    dev_info(&chip->client->dev, "%s: on = %d\n", __func__, on);
#ifdef POWER_REGULATOR
    if (on)
    {
		rc = taos_device_ctl(chip, on);
		if (rc)
			return rc;
    }
#endif
	if (on) {
		//adb cali by huangshenglin@hoperun, defect 2127533++
		#ifdef CALI_EVERY_TIME
			taos_offset_cali(chip->client,&ps_cali,3);
		#endif
		//adb cali by huangshenglin@hoperun, defect 2127533--

/*[BUFFIX]-Mod-Begin- by TCTNB.ZXZ,PR-1082275, 2015/09/09, needn't enable or disable IRQ ,
this may cause phone can't sleep if when APP invoke this node press powerkey (APP should not do like this) */
		chip->shadow[TSL277X_ENABLE] |=(TSL277X_EN_PWR_ON | TSL277X_EN_PRX);
		rc = update_enable_reg(chip);
		if (rc)
			return rc;
	} else {
		chip->shadow[TSL277X_ENABLE] &=~TSL277X_EN_PRX;
/*[BUFFIX]-Mod-Begin- by TCTNB.ZXZ*/
		if (!(chip->shadow[TSL277X_ENABLE] & TSL277X_EN_ALS))
			chip->shadow[TSL277X_ENABLE] &= ~TSL277X_EN_PWR_ON;
		rc = update_enable_reg(chip);
		if (rc)
			return rc;
	}
    mutex_lock(&chip->lock);
	if (!rc)
		chip->prx_enabled = on;
    mutex_unlock(&chip->lock);
#ifdef POWER_REGULATOR
	if (!on) {
		rc = taos_device_ctl(chip, on);
		if (rc)
			return rc;
	}
#endif
	return rc;
}


static ssize_t taos_device_prx_detected(struct class *class,
				struct class_attribute *attr, char *buf)
{
	struct tsl2772_chip *chip = i2c_get_clientdata(g_client);
	int rc;
    bool d ;
    u8 prx_raw_lo,prx_raw_hi;
    chip->prx_enable_pre = chip->prx_enabled;
	taos_prox_enable_nocal(chip, 1);
    msleep(10);
    rc = taos_i2c_read(chip, TSL277X_PRX_LO, &prx_raw_lo);
	if(rc)
	{
		printk(KERN_ERR "%s fail, rc=%d", __func__, rc);
		return rc;
	}
    rc = taos_i2c_read(chip, TSL277X_PRX_HI, &prx_raw_hi);
    if(rc)
	{
		printk(KERN_ERR "%s fail, rc=%d", __func__, rc);
		return rc;
	}
    d = chip->prx_inf.detected;
    chip->prx_inf.raw =  prx_raw_hi << 8 | prx_raw_lo ;


	if(chip->prx_inf.raw>950)
		chip->prx_inf.detected =1;
	else
		chip->prx_inf.detected =0;

/* sometimes get inaccurate rawdata if not calibrate every time detect, but if calibrate every time will affect threshold in call ,
so we use fixed value '950' as the threshold */
#if 0
      chip->prx_inf.detected =
            (d && (chip->prx_inf.raw > chip->params.prox_th_min)) ||
			(!d && (chip->prx_inf.raw > chip->params.prox_th_max));
#endif

     chip->prx_inf.detected=(chip->prx_inf.detected)*5;
     taos_prox_enable_nocal(chip, chip->prx_enable_pre);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->prx_inf.detected);
}



static struct class_attribute prx_status =
	__ATTR(status, 0444, taos_device_prx_detected, NULL);



static int prx_misoperation_creat_file(void)
{
	int ret;

	/*  /<sysfs>/class/prx_misoperation/status */

	prx_misoperation_class = class_create(THIS_MODULE, "prx_misoperation");
	if (IS_ERR(prx_misoperation_class)) {
		ret = PTR_ERR(prx_misoperation_class);
		printk(KERN_ERR "prx_misoperation_class: couldn't create prx_misoperation\n");
	}
	ret = class_create_file(prx_misoperation_class, &prx_status);
	if (ret) {
		printk(KERN_ERR "prx_misoperation: couldn't create status\n");
	}

	return ret;

}

static void remove_prx_create_file(void)
{
	class_remove_file(prx_misoperation_class, &prx_status);
}

#endif
/*[BUFFIX]-Mod-Begin- by TCTNB.ZXZ*/



static int /*__devinit*/ taos_probe(struct i2c_client *client,
	const struct i2c_device_id *idp)
{
	int i, ret;
	u8 id, rev;
	struct device *dev = &client->dev;
	static struct tsl2772_chip *chip;
	struct tsl2772_i2c_platform_data *pdata;
	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(dev, "%s: i2c smbus byte data unsupported\n", __func__);
		ret = -EOPNOTSUPP;
		goto init_failed;
	}

	chip = kzalloc(sizeof(struct tsl2772_chip), GFP_KERNEL);
	if (!chip) {
		ret = -ENOMEM;
		goto init_failed;
	}
    chip->als_can_wake = false;
    chip->proximity_can_wake = true;
    chip->in_suspend = 0;
    if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
			sizeof(struct tsl2772_i2c_platform_data), GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}
		ret = taos_parse_dt(&client->dev, pdata);
		if (ret)
			return -ENOMEM;
	} else
		pdata = client->dev.platform_data;
   
	if (!pdata) {
		dev_err(dev, "%s: platform data required\n", __func__);
		ret = -EINVAL;
		goto init_failed;
	}
	chip->prox_name = PS_NAME;
	chip->als_name = ALS_NAME;

	chip->client = client;
	chip->pdata = pdata;
	i2c_set_clientdata(client, chip);

	/*[BUFFIX]-Mod-Begin- by TCTNB.ZXZ,FR-1034674, 2015/07/13, add misoperation interface */
	#ifdef CONFIG_TCT_8X16_IDOL3
	g_client=client;
	#endif
	/*[BUFFIX]-Mod-Begin- by TCTNB.ZXZ*/
    chip->seg_num_max = ARRAY_SIZE(segment_default);
    ret =  set_segment_table(chip, segment_default,
			ARRAY_SIZE(segment_default));
	if (ret)
		goto set_segment_failed;
    
	mutex_init(&chip->lock);
    mutex_init(&chip->tsl2772_i2c_lock);
    mutex_init(&chip->io_als_lock);
    //wake_lock_init(&chip->taos_wakelock,WAKE_LOCK_SUSPEND, "taos_input_wakelock");    
    wake_lock_init(&chip->taos_nosuspend_wl,WAKE_LOCK_SUSPEND, "taos_nosuspend_wakelock");
    wake_lock_init(&chip->taos_int_wl,WAKE_LOCK_SUSPEND, "taos_taos_int_wakelock"); /* [BUFFIX]-Add-by TCTNB.XQJ,PR-886227, 2015/2/2,add lock for avoiding etern sleep before report psensor datae*/

#ifdef POWER_REGULATOR
    chip->power_enabled = 0;
    ret = taos_power_init(chip, true);
    if (ret)
        {
            dev_err(dev,"power_init failed");
            goto id_failed;
        }
    
    ret = taos_power_ctl(chip, true);
    if (ret)
    {
        dev_err(dev,"power_ctl failed ,enable = %d",true);
        taos_power_init(chip, false);
        goto power_ctl_failed;
    }
#endif    
    
	ret = taos_get_id(chip, &id, &rev);
	if (ret)
	{
		dev_err(dev,"taos_probe taos_get_id fail");
		goto id_failed;
	}
	for (i = 0; i < ARRAY_SIZE(tsl277x_ids); i++) {
		if (id == tsl277x_ids[i])
			break;
	}
	if (i < ARRAY_SIZE(tsl277x_names)) {
		dev_info(dev, "%s: '%s rev. %d' detected\n", __func__,
			tsl277x_names[i], rev);
	} else {
		dev_err(dev, "%s: not supported chip id\n", __func__);
		ret = -EOPNOTSUPP;
		goto id_failed;
	}
    //modify(add) by junfeng.zhou.sz for cali ps begin . 
#ifdef CALI_EVERY_TIME
    set_pltf_settings(chip);
	ret = flush_regs(chip);
	if (ret)
		goto flush_regs_failed;
    //taos_init_client_for_cali(chip->client);
    taos_offset_cali(chip->client,&ps_cali,10);
#endif
    //modify(add) by junfeng.zhou.sz for cali ps end . 
	set_pltf_settings(chip);
	ret = flush_regs(chip);
	if (ret)
		goto flush_regs_failed;

	if (!chip->prox_name)
		goto bypass_prox_idev;
	chip->p_idev = input_allocate_device();
	if (!chip->p_idev) {
		dev_err(dev, "%s: no memory for input_dev '%s'\n",
				__func__, chip->prox_name);
		ret = -ENODEV;
		goto input_p_alloc_failed;
	}
	chip->p_idev->name = chip->prox_name;
	chip->p_idev->id.bustype = BUS_I2C;
	set_bit(EV_ABS, chip->p_idev->evbit);
	set_bit(ABS_DISTANCE, chip->p_idev->absbit);
	input_set_abs_params(chip->p_idev, ABS_DISTANCE, 0, 1, 0, 0);

	dev_set_drvdata(&chip->p_idev->dev, chip);
	ret = input_register_device(chip->p_idev);
	if (ret) {
		input_free_device(chip->p_idev);
		dev_err(dev, "%s: cant register input '%s'\n",
				__func__, chip->prox_name);
		goto input_p_alloc_failed;
	}

	ido3_taos_proximity_dev = chip->p_idev; // [PLATFORM]-Mod-BEGIN by TCTNB.YQJ, PR-912196, 2015/01/24 

	ret = add_sysfs_interfaces(&chip->p_idev->dev,
			prox_attrs, ARRAY_SIZE(prox_attrs));
	if (ret)
		goto input_p_sysfs_failed;
/* [BUFFIX]-Mod- by TCTNB.XQJ,PR-839822, 2014/11/14,add interface for misoperation*/
/*[BUFFIX]-Mod-Begin- by TCTNB.ZXZ,FR-1034674, 2015/07/13, add misoperation interface */
	#ifdef CONFIG_TCT_8X16_IDOL3
	ret = prx_misoperation_creat_file();
	if(ret)
		goto prx_misoperation_creat_fail;
	#endif
/*[BUFFIX]-Mod-Begin- by TCTNB.ZXZ*/

//add by junfeng.zhou . for add the mmi test interface 
	for (i = 0; i < ARRAY_SIZE(prox_attrs); i++)
     {
        ret = device_create_file(&(client->dev),prox_attrs+i);
        if (ret)
	       goto input_a_alloc_failed;
      }
	//ret = device_create_file(&(client->dev),prox_attrs);
/* [BUFFIX]-Mod- by TCTNB.XQJ*/
	if (ret)
		goto input_a_alloc_failed;
bypass_prox_idev:
	if (!chip->als_name)
		goto bypass_als_idev;
	chip->a_idev = input_allocate_device();
	if (!chip->a_idev) {
		dev_err(dev, "%s: no memory for input_dev '%s'\n",
				__func__, chip->als_name);
		ret = -ENODEV;
		goto input_a_alloc_failed;
	}
	chip->a_idev->name = chip->als_name;
	chip->a_idev->id.bustype = BUS_I2C;
	set_bit(EV_ABS, chip->a_idev->evbit);
	set_bit(ABS_MISC, chip->a_idev->absbit);
	input_set_abs_params(chip->a_idev, ABS_MISC, 0, 65535, 0, 0);
	dev_set_drvdata(&chip->a_idev->dev, chip);
	ret = input_register_device(chip->a_idev);
	if (ret) {
		input_free_device(chip->a_idev);
		dev_err(dev, "%s: cant register input '%s'\n",
				__func__, chip->prox_name);
		goto input_a_alloc_failed;
	}
	ret = add_sysfs_interfaces(&chip->a_idev->dev,
			als_attrs, ARRAY_SIZE(als_attrs));
	if (ret)
		goto input_a_sysfs_failed;
    
	ret = device_create_file(&(client->dev),&(als_attrs[3]));
	if (ret)
	{
		dev_err(dev,"taos_probe device_create_file fail");
		goto input_a_alloc_failed;
	}
bypass_als_idev:
    
#ifdef ALS_POLL
    chip->taos_als_wq = create_singlethread_workqueue("taos_als_wq");	
    INIT_WORK(&chip->taos_als_work, taos_als_work_func);	
    hrtimer_init(&chip->als_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);	
    chip->als_poll_delay = ns_to_ktime(200 * NSEC_PER_MSEC);	
    chip->als_timer.function = taos_als_timer_func;
#endif
    chip->taos_int_wq = create_singlethread_workqueue("taos_int_wq");/* [BUFFIX]-Add- by TCTNB.XQJ,PR-886227, 2015/2/2,add a thread to replace direct calling in irq function*/
    INIT_WORK(&chip->taos_int_work, taos_int_work_func);/* [BUFFIX]-Add- by TCTNB.XQJ,PR-886227, 2015/2/2,add a thread to replace direct calling in irq function*/
    
    chip->irq = gpio_to_irq(pdata->int_pin);
    chip->client->irq = chip->irq;
/* [BUFFIX]-Add- Begin by TCTNB.XQJ,PR-886227, 2015/2/2,and configure int gpio*/
    ret = tsl2772x_pinctrl_init(chip);
    if (ret) {
		dev_err(&client->dev, "Can't initialize pinctrl\n");
			goto input_a_sysfs_failed;
      }
     ret = pinctrl_select_state(chip->pinctrl, chip->pin_default);
     if (ret) {
		dev_err(&client->dev,
			"Can't select pinctrl default state\n");
		goto input_a_sysfs_failed;
    }
/* [BUFFIX]-End-by TCTNB.XQJ*/
    ret = gpio_request(pdata->int_pin,"tsl-int");        
	if(ret < 0)
	{
		printk(KERN_ERR "%s: gpio_request, err=%d", __func__, ret);
		goto input_a_sysfs_failed;
	}
	ret = gpio_direction_input(pdata->int_pin);
	if(ret < 0)
	{
		printk(KERN_ERR "%s: gpio_direction_input, err=%d", __func__, ret);
		goto gpio_free;
	}	 
	ret = request_threaded_irq(chip->irq, NULL, taos_irq,IRQF_TRIGGER_FALLING| IRQF_ONESHOT, DEVICE_NAME, chip);
	if (ret) {
		dev_info(dev, "Failed to request irq %d\n", client->irq);
		goto irq_register_fail;
	}
       TSLX( "taos  disable irq\n");
/*[BUFFIX]-Mod-Begin- by TCTNB.ZXZ,PR-994478, 2015/05/08, add enable and disable irq in one function*/
	irq_enable_status=true;
	taos_irq_enable(chip->irq,false,false);
/*[BUFFIX]-Mod-End by TCTNB.ZXZ*/
	/*yongzhong.cheng,2014/08/12,PR-763890,psensor fail after bootup,START
	als never be opend by taos_als_enable_store when bootup,so we open here*/
	//taos_als_enable(chip, true); /* [BUFFIX]-Add-by TCTNB.XQJ,PR-886227, 2015/2/2,no need now*/
    //taos_prox_enable(chip, true);
	/*yongzhong.cheng,2014/08/12,PR-763890,psensor fail after bootup,END*/
	
    //init complete disable the als & ps
    taos_als_enable(chip, false);
    taos_prox_enable(chip, false);
#ifdef POWER_REGULATOR
            chip->als_enabled = false;
            chip->prx_enabled = false;
     //add by junfeng.zhou.sz for off the ps&als begin .
     //add by junfeng.zhou.sz for off the ps&als end .
            ret = taos_power_ctl(chip, false);
            if (ret)
            {
                dev_err(dev,"power_ctl failed ,enable = %d",false);
                taos_power_init(chip, false);
                goto irq_register_fail;
            }
#endif

// add by ning.wei for opimize p-sensor pr743248 2014-08-21 start
#ifdef USE_TRACEBILITY
	INIT_DELAYED_WORK(&chip->trace_work, trace_work_func);
	schedule_delayed_work(&chip->trace_work, msecs_to_jiffies(1000));
#endif

// add by ning.wei for opimize p-sensor pr743248 2014-08-21 end
	/* Register to sensors class */
	chip->als_cdev = sensors_light_cdev;
	chip->als_cdev.sensors_enable = tsl2772_als_set_enable;
	chip->als_cdev.sensors_poll_delay = tsl2772_als_poll_delay;
	chip->ps_cdev = sensors_proximity_cdev;
	chip->ps_cdev.sensors_enable = tsl2772_ps_set_enable;
	chip->ps_cdev.sensors_poll_delay = NULL;

	ret = sensors_classdev_register(&client->dev, &chip->als_cdev);
	if (ret) {
		dev_err(dev,"%s: Unable to register to sensors class: %d\n",
				__func__, ret);
		goto exit_unregister_als_ioctl;
	}

	ret = sensors_classdev_register(&client->dev, &chip->ps_cdev);
	if (ret) {
		dev_err(dev,"%s: Unable to register to sensors class: %d\n",
			       __func__, ret);
		goto exit_unregister_ps_class;
	}

	dev_info(dev, "%s ok.\n", __func__);
	return 0;

exit_unregister_ps_class:
	sensors_classdev_unregister(&chip->ps_cdev);
exit_unregister_als_ioctl:
	sensors_classdev_unregister(&chip->als_cdev);    
irq_register_fail:
    free_irq(chip->irq, chip);
gpio_free:
    gpio_free(pdata->int_pin);
    if (chip->a_idev) {
            remove_sysfs_interfaces(&chip->a_idev->dev,
                als_attrs, ARRAY_SIZE(als_attrs));
input_a_sysfs_failed:
		input_unregister_device(chip->a_idev);
	}
input_a_alloc_failed:
/*[BUFFIX]-Mod-Begin- by TCTNB.ZXZ,FR-1034674, 2015/07/13, add misoperation interface */
	#ifdef CONFIG_TCT_8X16_IDOL3
	remove_prx_create_file();
	prx_misoperation_creat_fail:
	#endif
/*[BUFFIX]-Mod-Begin- by TCTNB.ZXZ*/

	if (chip->p_idev) {
		remove_sysfs_interfaces(&chip->p_idev->dev,
			prox_attrs, ARRAY_SIZE(prox_attrs));
input_p_sysfs_failed:
		input_unregister_device(chip->p_idev);
	}
input_p_alloc_failed:
flush_regs_failed:
id_failed:
#ifdef POWER_REGULATOR
    taos_power_ctl(chip, false);
power_ctl_failed:
#endif
    wake_lock_destroy(&chip->taos_int_wl);/* [BUFFIX]-Add- by TCTNB.XQJ,PR-886227, 2015/2/2,destory thread*/
    wake_lock_destroy(&chip->taos_nosuspend_wl);
    mutex_destroy(&chip->io_als_lock);
    mutex_destroy(&chip->tsl2772_i2c_lock);
    mutex_destroy(&chip->lock);
	kfree(chip->segment);
set_segment_failed:
	i2c_set_clientdata(client, NULL);
    if (client->dev.of_node && (pdata != NULL))
		devm_kfree(&client->dev, pdata);
	kfree(chip);    

init_failed:
	dev_err(dev, "Probe failed.\n");
	return ret;
}
/* [BUFFIX]-Add-Begin by TCTNB.XQJ,PR-886227, 2015/2/2,add clear irq fla,for sure all the happend irq flag clear in suspend*/
/* [BUGFIX]-Mod-by TCTNB.XQJ, 2014/12/30, BUG 888563,for remove disable and enable psensor and light sensor ,also make p-sensor, can wake up system */
static int taos_suspend(struct device *dev)
{
	struct tsl2772_chip *chip = dev_get_drvdata(dev);
	//struct tsl2772_i2c_platform_data *pdata = dev->platform_data;

	dev_info(dev, "%s\n", __func__);
	//mutex_lock(&chip->lock);/* [BUGFIX]-Del- by TCTNB.XQJ, 2014/12/09, BUG 799932 ,for remove unncecssary mutex lock*/
       if (chip->in_suspend == 1) {
             return 0;
        }

	if (chip->p_idev && chip->p_idev->users && chip->prx_inf.detected) {
		if (chip->proximity_can_wake) {
			dev_dbg(dev, "set wake on proximity\n");
			chip->wake_irq = 1;
		} else {
			dev_dbg(dev, "proximity off\n");
			//taos_prox_enable(chip, 0);
		}
	}
	if (chip->a_idev && chip->a_idev->users) {
		if (chip->als_can_wake) {
			dev_dbg(dev, "set wake on als\n");
			chip->wake_irq = 1;
		} else {
			dev_dbg(dev, "als off\n");
		//	taos_als_enable(chip, 0);
		}
	}
	if (chip->wake_irq) {/* [BUFFIX]-Add-Begin by TCTNB.XQJ,PR-886227, 2015/2/2,add clear irq fla,for sure all the happend irq flag clear in suspend*/
		irq_set_irq_wake(chip->client->irq, 1);
	} else if (!chip->unpowered) {
		dev_dbg(dev, "powering off\n");
		//pltf_power_off(chip);
	}
	if (chip->prx_enabled) {
		taos_irq_clr(chip, TSL277X_CMD_PROX_INT_CLR | TSL277X_CMD_ALS_INT_CLR);
	}
	chip->in_suspend = 1;
	//mutex_unlock(&chip->lock);/* [BUGFIX]-Del- by TCTNB.XQJ, 2014/12/09, BUG 799932 ,for remove unncecssary mutex lock*/

	return 0;
}

static int taos_resume(struct device *dev)
{
	struct tsl2772_chip *chip = dev_get_drvdata(dev);
	bool als_on, prx_on;
	//mutex_lock(&chip->lock);/* [BUGFIX]-Del- by TCTNB.XQJ, 2014/12/09, BUG 799932 ,for remove unncecssary mutex lock*/
      if (chip->in_suspend == 0) {
            return 0;
        }
	prx_on = chip->p_idev && chip->p_idev->users;
	als_on = chip->a_idev && chip->a_idev->users;
	chip->in_suspend = 0;
	dev_dbg(dev, "%s: powerd %d, als: needed %d  enabled %d,"
			" prox: needed %d  enabled %d\n", __func__,
			!chip->unpowered, als_on, chip->als_enabled,
			prx_on, chip->prx_enabled);
	if (chip->wake_irq) {
		irq_set_irq_wake(chip->client->irq, 0);
		chip->wake_irq = 0;
	}
	if (chip->unpowered && (prx_on || als_on)) {
		dev_dbg(dev, "powering on\n");
	}

	if (chip->prx_enabled) {/* [BUFFIX]-Add-Begin by TCTNB.XQJ,PR-886227, 2015/2/2,add a repoart data,main for more safe,in some special case,p sensor irq lost*/
		(void)taos_check_and_report(chip);
	}
    dev_info(dev, "%s\n", __func__);
	//mutex_unlock(&chip->lock);/* [BUGFIX]-Del- by TCTNB.XQJ, 2014/12/09, BUG 799932 ,for remove unncecssary mutex lock*/
      chip->in_suspend = 0;

	return 0;
}
/* [BUGFIX]-Mod-by TCTNB.XQJ, 2014/12/30*/
/* [BUGFIX]-Mod-by TCTNB.XQJ, 2015/2/02*/
static int /*__devexit*/ taos_remove(struct i2c_client *client)
{
	struct tsl2772_chip *chip = i2c_get_clientdata(client); 
    struct tsl2772_i2c_platform_data *pdata = chip->pdata;
	free_irq(client->irq, chip);   
	if (chip->a_idev) {
		remove_sysfs_interfaces(&chip->a_idev->dev,
			als_attrs, ARRAY_SIZE(als_attrs));
		input_unregister_device(chip->a_idev);
	}
	if (chip->p_idev) {
		remove_sysfs_interfaces(&chip->p_idev->dev,
			prox_attrs, ARRAY_SIZE(prox_attrs));
		input_unregister_device(chip->p_idev);
	}
	/*if (chip->pdata->platform_teardown)
		chip->pdata->platform_teardown(&client->dev);*/
	i2c_set_clientdata(client, NULL);
	kfree(chip->segment);
    if (client->dev.of_node && (pdata != NULL))
		devm_kfree(&client->dev, pdata);
	kfree(chip);  
	return 0;
}

static struct i2c_device_id taos_idtable[] = {
	{ DEVICE_NAME, 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, taos_idtable);

static const struct dev_pm_ops taos_pm_ops = {
	.suspend = taos_suspend,
	.resume  = taos_resume,
};

static struct i2c_driver taos_driver = {
	.driver = {
		.name = DEVICE_NAME,
		.owner = THIS_MODULE,
		.pm = &taos_pm_ops,
	},
	.id_table = taos_idtable,
	.probe    = taos_probe,
	.remove   = taos_remove,//__devexit_p(taos_remove),
};

static int __init taos_init(void)
{
	return i2c_add_driver(&taos_driver);
}

static void __exit taos_exit(void)
{
	i2c_del_driver(&taos_driver);
}

module_init(taos_init);
module_exit(taos_exit);

MODULE_AUTHOR("Aleksej Makarov <aleksej.makarov@sonyericsson.com>");
MODULE_DESCRIPTION("TAOS tsl2772 ambient light and proximity sensor driver");
MODULE_LICENSE("GPL");
