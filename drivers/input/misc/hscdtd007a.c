/* drivers/input/misc/hscdtd007a.c
 *
 * GeoMagneticField device driver (HSCDTD007A)
 *
 * Copyright (C) 2011-2015 ALPS ELECTRIC CO., LTD. All Rights Reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
/* #define DEBUG */
/* #define VERBOSE_DEBUG */
#ifdef ALPS_MAG_DEBUG
#define DEBUG 1
#endif
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/hscdtd.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/sensors.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif


#define HSCDTD_DRIVER_NAME		"hscdtd007a"
#define HSCDTD_SENSOE_CLASS_NAME	"hscdtd007a-mag"
#define HSCDTD_INPUT_DEVICE_NAME	"compass"
#define HSCDTD_SNS_KIND			4
#define HSCDTD_LOG_TAG			"[HSCDTD], "


#define I2C_RETRIES		5


#define HSCDTD_CHIP_ID		0x1511

#define HSCDTD_STBA		0x0C
#define HSCDTD_INFO		0x0D
#define HSCDTD_XOUT		0x10
#define HSCDTD_YOUT		0x12
#define HSCDTD_ZOUT		0x14
#define HSCDTD_XOUT_H		0x11
#define HSCDTD_XOUT_L		0x10
#define HSCDTD_YOUT_H		0x13
#define HSCDTD_YOUT_L		0x12
#define HSCDTD_ZOUT_H		0x15
#define HSCDTD_ZOUT_L		0x14

#define HSCDTD_STATUS		0x18
#define HSCDTD_CTRL1		0x1B
#define HSCDTD_CTRL2		0x1C
#define HSCDTD_CTRL3		0x1D
#define HSCDTD_CTRL4		0x1E

#define HSCDTD_TCS_TIME		10000	/* Measure temp. of every 10 sec */
#define HSCDTD_DATA_ACCESS_NUM	6
#define HSCDTD_3AXIS_NUM	3
#define HSCDTD_INITIALL_DELAY	20
#define STBB_OUTV_THR		3838

#define HSCDTD_DELAY(us)	usleep(us)

/* Self-test resiter value */
#define HSCDTD_ST_REG_DEF	0x55
#define HSCDTD_ST_REG_PASS	0xAA
#define HSCDTD_ST_REG_X		0x01
#define HSCDTD_ST_REG_Y		0x02
#define HSCDTD_ST_REG_Z		0x04
#define HSCDTD_ST_REG_XYZ	0x07

/* Self-test error number */
#define HSCDTD_ST_OK		0x00
#define HSCDTD_ST_ERR_I2C	0x01
#define HSCDTD_ST_ERR_INIT	0x02
#define HSCDTD_ST_ERR_1ST	0x03
#define HSCDTD_ST_ERR_2ND	0x04
#define HSCDTD_ST_ERR_VAL	0x10

/* Save last device state for power down */
struct hscdtd_sensor_state {
	bool power_on;
	/*uint8_t mode;*/
};

struct hscdtd_data {
	struct input_dev	*input;
	struct i2c_client	*i2c;
	struct delayed_work	work_data;
	struct mutex		lock;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend	early_suspend_h;
#endif
	struct sensors_classdev	cdev;
	unsigned int		kind;
	unsigned int		delay_msec;
	unsigned int		tcs_thr;
	unsigned int		tcs_cnt;
	bool			factive;
	bool			fsuspend;
	bool			fskip;
	int			matrix[9];
	/* regulator data*/
	int	power_enabled;
	struct regulator	*vdd;
	struct regulator	*vio;
	struct hscdtd_sensor_state state;

	char layout;
};

static struct sensors_classdev sensors_cdev = {
	.name = HSCDTD_SENSOE_CLASS_NAME,
	.vendor = "ALPS ELECTRIC CO., LTD.",
	.version = 1,
	.handle = SENSORS_MAGNETIC_FIELD_HANDLE,
	.type = SENSOR_TYPE_MAGNETIC_FIELD,
	.max_range = "4800.0",
	.resolution = "0.15",
	.sensor_power = "0.6",
	.min_delay = 10000,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 10000,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

static int hscdtd_compass_power_set(struct hscdtd_data *data, bool on);
/*--------------------------------------------------------------------------
 * i2c read/write function
 *--------------------------------------------------------------------------*/
static int hscdtd_i2c_read(struct i2c_client *i2c, u8 *rxData, int length)
{
	int err;
	int tries = 0;

	struct i2c_msg msgs[] = {
		{
			.addr	= i2c->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= rxData,
		},
		{
			.addr	= i2c->addr,
			.flags	= I2C_M_RD,
			.len	= length,
			.buf	= rxData,
		 },
	};

	do {
		err = i2c_transfer(i2c->adapter, msgs, ARRAY_SIZE(msgs));
	} while ((err != ARRAY_SIZE(msgs)) && (++tries < I2C_RETRIES));

	if (err != ARRAY_SIZE(msgs)) {
		dev_err(&i2c->adapter->dev, "read transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int hscdtd_i2c_write(struct i2c_client *i2c, u8 *txData, int length)
{
	int err;
	int tries = 0;

	struct i2c_msg msgs[] = {
		{
			.addr	= i2c->addr,
			.flags	= 0,
			.len	= length,
			.buf	= txData,
		},
	};

	do {
		err = i2c_transfer(i2c->adapter, msgs, ARRAY_SIZE(msgs));
	} while ((err != ARRAY_SIZE(msgs)) && (++tries < I2C_RETRIES));

	if (err != ARRAY_SIZE(msgs)) {
		dev_err(&i2c->adapter->dev, "write transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}


/*--------------------------------------------------------------------------
 * hscdtd function
 *--------------------------------------------------------------------------*/
static void hscdtd_convert_mount(
			struct hscdtd_data *hscdtd, int *xyz)
{
	int i, j, tmp[3];
	memcpy(tmp, xyz, sizeof(tmp));
	for (i = 0; i < 3; i++)
		for (j = 0; j < 3; j++)
			xyz[i] += hscdtd->matrix[j + i * 3] * tmp[j];
}

static int hscdtd_get_magnetic_field_data(
			struct hscdtd_data *hscdtd, int *xyz)
{
	int err = -1;
	int i;
	u8  sx[HSCDTD_DATA_ACCESS_NUM];
	int tmp;

	sx[0] = HSCDTD_XOUT;
	err = hscdtd_i2c_read(hscdtd->i2c, sx,
	    HSCDTD_DATA_ACCESS_NUM);
	if (err < 0)
		return err;
	for (i = 0; i < HSCDTD_3AXIS_NUM; i++)
		xyz[i] = (int) ((short)((sx[2*i+1] << 8) | (sx[2*i])));

	hscdtd_convert_mount(hscdtd, xyz);

	/** choose the right position/layout **/
	switch (hscdtd->layout) {
	case 0:
	case 1:
		/* Fall into the default direction */
		break;
	case 2:
		tmp = xyz[0];
		xyz[0] = xyz[1];
		xyz[1] = -tmp;
		break;
	case 3:
		xyz[0] = -xyz[0];
		xyz[1] = -xyz[1];
		break;
	case 4:
		tmp = xyz[0];
		xyz[0] = -xyz[1];
		xyz[1] = tmp;
		break;
	case 5:
		xyz[0] = -xyz[0];
		xyz[2] = -xyz[2];
		break;
	case 6:
		tmp = xyz[0];
		xyz[0] = xyz[1];
		xyz[1] = tmp;
		xyz[2] = -xyz[2];
		break;
	case 7:
		xyz[1] = -xyz[1];
		xyz[2] = -xyz[2];
		break;
	case 8:
		tmp = xyz[0];
		xyz[0] = -xyz[1];
		xyz[1] = -tmp;
		xyz[2] = -xyz[2];
		break;
	}

	dev_dbg(&hscdtd->i2c->adapter->dev,
	    HSCDTD_LOG_TAG "x:%d,y:%d,z:%d\n", xyz[0], xyz[1], xyz[2]);

	return err;
}

static int hscdtd_soft_reset(struct hscdtd_data *hscdtd)
{
	int rc;
	u8 buf[2];

	dev_dbg(&hscdtd->i2c->adapter->dev,
	    HSCDTD_LOG_TAG "%s\n", __func__);

	buf[0] = HSCDTD_CTRL3;
	buf[1] = 0x80;
	rc = hscdtd_i2c_write(hscdtd->i2c, buf, 2);
	HSCDTD_DELAY(5000);

	return rc;
}

static int hscdtd_tcs_setup(struct hscdtd_data *hscdtd)
{
	int rc;
	u8 buf[2];

	dev_dbg(&hscdtd->i2c->adapter->dev,
	    HSCDTD_LOG_TAG "%s\n", __func__);

	buf[0] = HSCDTD_CTRL3;
	buf[1] = 0x02;
	rc = hscdtd_i2c_write(hscdtd->i2c, buf, 2);
	HSCDTD_DELAY(600);
	hscdtd->tcs_thr = HSCDTD_TCS_TIME / hscdtd->delay_msec;
	hscdtd->tcs_cnt = 0;

	return rc;
}

static int hscdtd_force_setup(struct hscdtd_data *hscdtd)
{
	u8 buf[2];

	buf[0] = HSCDTD_CTRL3;
	buf[1] = 0x40;

	return hscdtd_i2c_write(hscdtd->i2c, buf, 2);
}

static void hscdtd_measure_setup(
			struct hscdtd_data *hscdtd, bool en)
{
	u8 buf[2];

	if (en) {
		buf[0] = HSCDTD_CTRL1;
		buf[1] = 0x8A;
		hscdtd_i2c_write(hscdtd->i2c, buf, 2);
		HSCDTD_DELAY(10);

		buf[0] = HSCDTD_CTRL4;
		buf[1] = 0x90;
		hscdtd_i2c_write(hscdtd->i2c, buf, 2);
		HSCDTD_DELAY(10);

		hscdtd_tcs_setup(hscdtd);
		hscdtd_force_setup(hscdtd);
	} else
		hscdtd_soft_reset(hscdtd);
}

static void hscdtd_schedule_setup(
			struct hscdtd_data *hscdtd, bool en)
{
	dev_dbg(&hscdtd->i2c->adapter->dev,
	    HSCDTD_LOG_TAG "%s, en = %d\n", __func__, en);

	if (en)
		schedule_delayed_work(&hscdtd->work_data,
		    msecs_to_jiffies(hscdtd->delay_msec));
	else
		cancel_delayed_work(&hscdtd->work_data);
}

static int hscdtd_get_hardware_data(
			struct hscdtd_data *hscdtd, int *xyz)
{
	int ret = 0;
	hscdtd_measure_setup(hscdtd, true);
	HSCDTD_DELAY(5000);
	ret = hscdtd_get_magnetic_field_data(hscdtd, xyz);
	hscdtd_measure_setup(hscdtd, false);
	if (ret)
		dev_err(&hscdtd->i2c->adapter->dev, HSCDTD_LOG_TAG
		    "measurement error.\n");
	return ret;
}

static int hscdtd_self_test_A(struct hscdtd_data *hscdtd)
{
	int rc = HSCDTD_ST_OK;
	u8 sx[2], cr1[1];

	dev_dbg(&hscdtd->i2c->adapter->dev,
	    HSCDTD_LOG_TAG "%s\n", __func__);

	/* Control resister1 backup  */
	cr1[0] = HSCDTD_CTRL1;
	if (hscdtd_i2c_read(hscdtd->i2c, cr1, 1))
		return HSCDTD_ST_ERR_I2C;
	dev_dbg(&hscdtd->i2c->adapter->dev,
	    HSCDTD_LOG_TAG "Control resister1 value, %02X\n", cr1[0]);

	/* Move active mode (force state)  */
	sx[0] = HSCDTD_CTRL1;
	sx[1] = 0x8A;
	if (hscdtd_i2c_write(hscdtd->i2c, sx, 2))
		return HSCDTD_ST_ERR_I2C;
	HSCDTD_DELAY(10);

	/* Get inital value of self-test-A register  */
	sx[0] = HSCDTD_STBA;
	hscdtd_i2c_read(hscdtd->i2c, sx, 1);
	sx[0] = HSCDTD_STBA;
	if (hscdtd_i2c_read(hscdtd->i2c, sx, 1))
		return HSCDTD_ST_ERR_I2C;
	dev_dbg(&hscdtd->i2c->adapter->dev,
	    HSCDTD_LOG_TAG "STB reg. initial value, %02X\n", sx[0]);
	if (sx[0] != HSCDTD_ST_REG_DEF) {
		dev_err(&hscdtd->i2c->adapter->dev, HSCDTD_LOG_TAG
		    "Err: Initial value of STB reg. is %02X\n", sx[0]);
		rc = HSCDTD_ST_ERR_INIT;
		goto err_STBA;
	}

	/* do self-test-A  */
	sx[0] = HSCDTD_CTRL3;
	sx[1] = 0x10;
	if (hscdtd_i2c_write(hscdtd->i2c, sx, 2))
		return HSCDTD_ST_ERR_I2C;
	HSCDTD_DELAY(3000);

	/* Get 1st value of self-test-A register  */
	sx[0] = HSCDTD_STBA;
	if (hscdtd_i2c_read(hscdtd->i2c, sx, 1))
		return HSCDTD_ST_ERR_I2C;
	dev_dbg(&hscdtd->i2c->adapter->dev,
	    HSCDTD_LOG_TAG "STB reg. 1st value, %02X\n", sx[0]);
	if (sx[0] != HSCDTD_ST_REG_PASS) {
		dev_err(&hscdtd->i2c->adapter->dev, HSCDTD_LOG_TAG
		    "Err: 1st value of STB reg. is %02X\n", sx[0]);
		rc = HSCDTD_ST_ERR_1ST;
		goto err_STBA;
	}
	HSCDTD_DELAY(3000);

	/* Get 2nd value of self-test-A register  */
	sx[0] = HSCDTD_STBA;
	if (hscdtd_i2c_read(hscdtd->i2c, sx, 1))
		return HSCDTD_ST_ERR_I2C;
	dev_dbg(&hscdtd->i2c->adapter->dev,
	    HSCDTD_LOG_TAG "STB reg. 2nd value, %02X\n", sx[0]);
	if (sx[0] != HSCDTD_ST_REG_DEF) {
		dev_err(&hscdtd->i2c->adapter->dev, HSCDTD_LOG_TAG
		    "Err: 2nd value of STB reg. is %02X\n", sx[0]);
		rc = HSCDTD_ST_ERR_2ND;
	}

err_STBA:
	/* Resume */
	sx[0] = HSCDTD_CTRL1;
	sx[1] = cr1[0];
	if (hscdtd_i2c_write(hscdtd->i2c, sx, 2))
		return HSCDTD_ST_ERR_I2C;
	HSCDTD_DELAY(10);

	return rc;
}

static int hscdtd_self_test_B(struct hscdtd_data *hscdtd)
{
	int rc = HSCDTD_ST_OK, xyz[3];

	dev_dbg(&hscdtd->i2c->adapter->dev,
	    HSCDTD_LOG_TAG "%s\n", __func__);

	/* Measurement sensor value */
	if (hscdtd_get_hardware_data(hscdtd, xyz))
		return HSCDTD_ST_ERR_I2C;

	/* Check output value */
	if ((xyz[0] <= -STBB_OUTV_THR) || (xyz[0] >= STBB_OUTV_THR))
		rc |= HSCDTD_ST_REG_X;
	if ((xyz[1] <= -STBB_OUTV_THR) || (xyz[1] >= STBB_OUTV_THR))
		rc |= HSCDTD_ST_REG_Y;
	if ((xyz[2] <= -STBB_OUTV_THR) || (xyz[2] >= STBB_OUTV_THR))
		rc |= HSCDTD_ST_REG_Z;
	if (rc)
		rc |= HSCDTD_ST_ERR_VAL;

	return rc;
}

static int hscdtd_register_init(struct hscdtd_data *hscdtd)
{
	int v[HSCDTD_3AXIS_NUM], ret = 0;
	u8  buf[2];
	u16 chip_info;

	dev_dbg(&hscdtd->i2c->adapter->dev,
	    HSCDTD_LOG_TAG "%s\n", __func__);

	if (hscdtd_soft_reset(hscdtd)) {
		dev_err(&hscdtd->i2c->adapter->dev, HSCDTD_LOG_TAG
		    "Err. Can't execute software reset");
		return -EPERM;
	}

	buf[0] = HSCDTD_INFO;
	ret = hscdtd_i2c_read(hscdtd->i2c, buf, 2);
	if (ret < 0)
		return ret;

	chip_info = (u16)((buf[1]<<8) | buf[0]);
	dev_dbg(&hscdtd->i2c->adapter->dev,
	    HSCDTD_LOG_TAG "chip_info, 0x%04X\n", chip_info);
	if (chip_info != HSCDTD_CHIP_ID) {
		dev_err(&hscdtd->i2c->adapter->dev, HSCDTD_LOG_TAG
		    "chipID error(0x%04X).\n", chip_info);
		return -EPERM;
	}

	mutex_lock(&hscdtd->lock);
	ret = hscdtd_get_hardware_data(hscdtd, v);
	hscdtd->kind = HSCDTD_SNS_KIND;
	mutex_unlock(&hscdtd->lock);
	dev_info(&hscdtd->i2c->adapter->dev,
	    HSCDTD_LOG_TAG "x:%d y:%d z:%d\n", v[0], v[1], v[2]);

	return ret;
}

static void hscdtd_enable_set(struct hscdtd_data *hscdtd, int en)
{
	int ret = 0;

	mutex_lock(&hscdtd->lock);
	hscdtd->fskip = true;
	if (en) {
		ret = hscdtd_compass_power_set(hscdtd, true);
		if (ret) {
			dev_err(&hscdtd->i2c->dev,
				"Fail to power on the device!\n");
			goto exit;
		}

		if (!hscdtd->factive) {
			hscdtd_measure_setup(hscdtd, true);
			hscdtd_schedule_setup(hscdtd, true);
		}
		hscdtd->factive = true;
	} else {
		hscdtd_schedule_setup(hscdtd, false);
		hscdtd_measure_setup(hscdtd, false);
		hscdtd->factive = false;

		ret = hscdtd_compass_power_set(hscdtd, false);
		if (ret) {
			dev_err(&hscdtd->i2c->dev,
				"Fail to power off the device!\n");
			goto exit;
		}
	}
	/**mutex_unlock(&hscdtd->lock);**/

	dev_dbg(&hscdtd->i2c->adapter->dev,
	    HSCDTD_LOG_TAG "%s, enable = %d\n", __func__, en);
exit:
	mutex_unlock(&hscdtd->lock);
}

static void hscdtd_delay_set(struct hscdtd_data *hscdtd, int delay)
{
	if (delay < 10)
		delay = 10;
	else if (delay > 200)
		delay = 200;
	mutex_lock(&hscdtd->lock);
	hscdtd->tcs_cnt =
	    hscdtd->tcs_cnt * hscdtd->delay_msec / delay;
	hscdtd->tcs_thr = HSCDTD_TCS_TIME / delay;
	hscdtd->delay_msec = delay;
	mutex_unlock(&hscdtd->lock);

	dev_dbg(&hscdtd->i2c->adapter->dev,
	    HSCDTD_LOG_TAG "%s, rate = %d (msec)\n",
	    __func__, hscdtd->delay_msec);
}

/*--------------------------------------------------------------------------
 * suspend/resume function
 *--------------------------------------------------------------------------*/
static int hscdtd_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct hscdtd_data *hscdtd = i2c_get_clientdata(client);

	dev_dbg(&hscdtd->i2c->adapter->dev,
	    HSCDTD_LOG_TAG "%s\n", __func__);

	mutex_lock(&hscdtd->lock);
	hscdtd->fsuspend = true;
	hscdtd->fskip = true;
	hscdtd_schedule_setup(hscdtd, false);
	hscdtd_measure_setup(hscdtd, false);
	mutex_unlock(&hscdtd->lock);

	hscdtd->state.power_on = hscdtd->power_enabled;
	if (hscdtd->state.power_on)
		hscdtd_compass_power_set(hscdtd, false);

	return 0;
}

static int hscdtd_resume(struct i2c_client *client)
{
	struct hscdtd_data *hscdtd = i2c_get_clientdata(client);
	int ret = 0;

	dev_dbg(&hscdtd->i2c->adapter->dev,
	    HSCDTD_LOG_TAG "%s\n", __func__);

	if (hscdtd->state.power_on) {
		ret = hscdtd_compass_power_set(hscdtd, true);
		if (ret) {
			dev_err(&hscdtd->i2c->adapter->dev, "Sensor power resume fail!\n");
			goto exit;
		}
	}

	mutex_lock(&hscdtd->lock);
	hscdtd->fskip = true;
	if (hscdtd->factive)
		hscdtd_measure_setup(hscdtd, true);
	if (hscdtd->factive)
		hscdtd_schedule_setup(hscdtd, true);
	hscdtd->fsuspend = false;
	mutex_unlock(&hscdtd->lock);

exit:
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void hscdtd_early_suspend(struct early_suspend *handler)
{
	struct hscdtd_data *hscdtd =
	    container_of(handler, struct hscdtd_data, early_suspend_h);
	hscdtd_suspend(hscdtd->i2c, PMSG_SUSPEND);
}

static void hscdtd_early_resume(struct early_suspend *handler)
{
	struct hscdtd_data *hscdtd =
	    container_of(handler, struct hscdtd_data, early_suspend_h);
	hscdtd_resume(hscdtd->i2c);
}
#endif


/*--------------------------------------------------------------------------
 * sysfs
 *--------------------------------------------------------------------------*/
static ssize_t hscdtd_kind_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct hscdtd_data *hscdtd = dev_get_drvdata(dev);
	/* return sprintf(buf, "%d\n", (int)hscdtd->kind); */
	return snprintf(buf, PAGE_SIZE, "%d\n", (int)hscdtd->kind);
}

static ssize_t hscdtd_enable_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct hscdtd_data *hscdtd = dev_get_drvdata(dev);
	/* return sprintf(buf, "%d\n", (hscdtd->factive) ? 1 : 0); */
	return snprintf(buf, PAGE_SIZE, "%d\n", (hscdtd->factive) ? 1 : 0);
}

static ssize_t hscdtd_enable_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t size)
{
	struct hscdtd_data *hscdtd = dev_get_drvdata(dev);
	int new_value;

	if (hscdtd->fsuspend) {
		dev_err(&hscdtd->i2c->adapter->dev,
		    HSCDTD_LOG_TAG "Error: Please resume device\n");
		return size;
	}

	if (sysfs_streq(buf, "1"))
		new_value = 1;
	else if (sysfs_streq(buf, "0"))
		new_value = 0;
	else {
		dev_err(&hscdtd->i2c->adapter->dev,
		    HSCDTD_LOG_TAG "%s: invalid value %d\n",
		    __func__, *buf);
		return -EINVAL;
	}

	hscdtd_enable_set(hscdtd, new_value);

	return size;
}

static ssize_t hscdtd_axis_matrix_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct hscdtd_data *hscdtd = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
		hscdtd->matrix[0], hscdtd->matrix[1], hscdtd->matrix[2],
		hscdtd->matrix[3], hscdtd->matrix[4], hscdtd->matrix[5],
		hscdtd->matrix[6], hscdtd->matrix[7], hscdtd->matrix[8]);
}

static ssize_t hscdtd_delay_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct hscdtd_data *hscdtd = dev_get_drvdata(dev);
	/* return sprintf(buf, "%d\n", hscdtd->delay_msec); */
	return snprintf(buf, PAGE_SIZE, "%d\n", hscdtd->delay_msec);
}

static ssize_t hscdtd_delay_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t size)
{
	int err;
	long new_delay;
	struct hscdtd_data *hscdtd = dev_get_drvdata(dev);

	/*err = strict_strtol(buf, 10, &new_delay);*/
	err = kstrtol(buf, 10, &new_delay);
	if (err < 0)
		return err;

	hscdtd_delay_set(hscdtd, (int)new_delay);

	return size;
}

static ssize_t hscdtd_self_test_A_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = -1;
	struct hscdtd_data *hscdtd = dev_get_drvdata(dev);

	dev_dbg(&hscdtd->i2c->adapter->dev,
	    HSCDTD_LOG_TAG "%s\n", __func__);

	if (hscdtd->fsuspend) {
		dev_err(&hscdtd->i2c->adapter->dev,
		    HSCDTD_LOG_TAG "Error: Please resume device\n");
		return snprintf(buf, PAGE_SIZE, "%d\n", ret);
	}

	if (!hscdtd->factive) {
		mutex_lock(&hscdtd->lock);
		ret = hscdtd_self_test_A(hscdtd);
		mutex_unlock(&hscdtd->lock);
		dev_dbg(&hscdtd->i2c->adapter->dev,
		    HSCDTD_LOG_TAG "Self test-A result : %d\n", ret);
	} else
		dev_err(&hscdtd->i2c->adapter->dev,
		    HSCDTD_LOG_TAG "Error: Please turn off sensor\n");

	return snprintf(buf, PAGE_SIZE, "%d\n", ret);
}

static ssize_t hscdtd_self_test_B_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = -1;
	struct hscdtd_data *hscdtd = dev_get_drvdata(dev);

	dev_dbg(&hscdtd->i2c->adapter->dev,
	    HSCDTD_LOG_TAG "%s\n", __func__);

	if (hscdtd->fsuspend) {
		dev_err(&hscdtd->i2c->adapter->dev,
		    HSCDTD_LOG_TAG "Error: Please resume device\n");
		return snprintf(buf, PAGE_SIZE, "%d\n", ret);
	}

	if (!hscdtd->factive) {
		mutex_lock(&hscdtd->lock);
		ret = hscdtd_self_test_B(hscdtd);
		mutex_unlock(&hscdtd->lock);
		dev_dbg(&hscdtd->i2c->adapter->dev,
		    HSCDTD_LOG_TAG "Self test-B result : %d\n", ret);
	} else
		dev_err(&hscdtd->i2c->adapter->dev,
		    HSCDTD_LOG_TAG "Error: Please turn off sensor\n");

	return snprintf(buf, PAGE_SIZE, "%d\n", ret);
}

static ssize_t hscdtd_get_hw_data_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int xyz[] = {0, 0, 0};
	struct hscdtd_data *hscdtd = dev_get_drvdata(dev);

	if (hscdtd->fsuspend) {
		dev_err(&hscdtd->i2c->adapter->dev,
		    HSCDTD_LOG_TAG "Error: Please resume device\n");
		return snprintf(buf, PAGE_SIZE, "%d,%d,%d\n",
			xyz[0], xyz[1], xyz[2]);
	}

	if (!hscdtd->factive) {
		mutex_lock(&hscdtd->lock);
		hscdtd_get_hardware_data(hscdtd, xyz);
		mutex_unlock(&hscdtd->lock);
		dev_dbg(&hscdtd->i2c->adapter->dev,
		    HSCDTD_LOG_TAG "%s: %d, %d, %d\n",
		    __func__, xyz[0], xyz[1], xyz[2]);
	} else
		dev_err(&hscdtd->i2c->adapter->dev,
		    HSCDTD_LOG_TAG "Error: Please turn off sensor\n");

	return snprintf(buf, PAGE_SIZE, "%d,%d,%d\n", xyz[0], xyz[1], xyz[2]);
}

static struct device_attribute attributes[] = {
	__ATTR(kind, S_IRUGO,
		hscdtd_kind_show, NULL),
	__ATTR(axis, S_IRUGO,
		hscdtd_axis_matrix_show, NULL),
	__ATTR(enable, S_IWUGO | S_IRUGO,
		hscdtd_enable_show, hscdtd_enable_store),
	__ATTR(delay, S_IWUGO | S_IRUGO,
		hscdtd_delay_show, hscdtd_delay_store),
	__ATTR(self_test_A, S_IRUGO,
		hscdtd_self_test_A_show, NULL),
	__ATTR(self_test_B, S_IRUGO,
		hscdtd_self_test_B_show, NULL),
	__ATTR(get_hw_data, S_IRUGO,
		hscdtd_get_hw_data_show, NULL)
};

static int hscdtd_enable_sensors_class(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	struct hscdtd_data *hscdtd = container_of(sensors_cdev,
			struct hscdtd_data, cdev);

	hscdtd_enable_set(hscdtd, (int)enable);

	return 0;
}

static int hscdtd_delay_sensors_class(struct sensors_classdev *sensors_cdev,
		unsigned int delay_msec)
{
	struct hscdtd_data *hscdtd = container_of(sensors_cdev,
			struct hscdtd_data, cdev);
	hscdtd_delay_set(hscdtd, (int)delay_msec);

	return 0;
}

static int hscdtd_create_sysfs(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(dev, attributes + i))
			goto out_sysfs;

	return 0;

out_sysfs:
	for (; i >= 0; i--)
		device_remove_file(dev, attributes + i);
	dev_err(dev, "Unable to create interface\n");

	return -EIO;
}

static void hscdtd_remove_sysfs(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
}

/****************** power contrl start *************************/
static int hscdtd_compass_power_set(struct hscdtd_data *data, bool on)
{
	int rc = 0;

	if (!on && data->power_enabled) {
		rc = regulator_disable(data->vdd);
		if (rc) {
			dev_err(&data->i2c->dev,
				"Regulator vdd disable failed rc=%d\n", rc);
			goto err_vdd_disable;
		}

		rc = regulator_disable(data->vio);
		if (rc) {
			dev_err(&data->i2c->dev,
				"Regulator vio disable failed rc=%d\n", rc);
			goto err_vio_disable;
		}
		data->power_enabled = false;
		return rc;
	} else if (on && !data->power_enabled) {
		rc = regulator_enable(data->vdd);
		if (rc) {
			dev_err(&data->i2c->dev,
				"Regulator vdd enable failed rc=%d\n", rc);
			goto err_vdd_enable;
		}

		rc = regulator_enable(data->vio);
		if (rc) {
			dev_err(&data->i2c->dev,
				"Regulator vio enable failed rc=%d\n", rc);
			goto err_vio_enable;
		}
		data->power_enabled = true;

		/*
		 * The max time for the power supply rise time is 50ms.
		 * Use 80ms to make sure it meets the requirements.
		 */
		msleep(80);
		return rc;
	} else {
		dev_warn(&data->i2c->dev,
				"Power on=%d. enabled=%d\n",
				on, data->power_enabled);
		return rc;
	}

err_vio_enable:
	regulator_disable(data->vio);
err_vdd_enable:
	return rc;

err_vio_disable:
	if (regulator_enable(data->vdd))
		dev_warn(&data->i2c->dev, "Regulator vdd enable failed\n");
err_vdd_disable:
	return rc;
}

static int hscdtd_compass_power_init(struct hscdtd_data *data, bool on)
{
	int rc;

	if (!on) {
		if (regulator_count_voltages(data->vdd) > 0)
			regulator_set_voltage(data->vdd, 0,
				HSCDTD007A_VDD_MAX_UV);

		regulator_put(data->vdd);

		if (regulator_count_voltages(data->vio) > 0)
			regulator_set_voltage(data->vio, 0,
				HSCDTD007A_VIO_MAX_UV);

		regulator_put(data->vio);

	} else {
		data->vdd = regulator_get(&data->i2c->dev, "vdd");
		if (IS_ERR(data->vdd)) {
			rc = PTR_ERR(data->vdd);
			dev_err(&data->i2c->dev,
				"Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}

		if (regulator_count_voltages(data->vdd) > 0) {
			rc = regulator_set_voltage(data->vdd,
				HSCDTD007A_VDD_MIN_UV, HSCDTD007A_VDD_MAX_UV);
			if (rc) {
				dev_err(&data->i2c->dev,
					"Regulator set failed vdd rc=%d\n",
					rc);
				goto reg_vdd_put;
			}
		}

		data->vio = regulator_get(&data->i2c->dev, "vio");
		if (IS_ERR(data->vio)) {
			rc = PTR_ERR(data->vio);
			dev_err(&data->i2c->dev,
				"Regulator get failed vio rc=%d\n", rc);
			goto reg_vdd_set;
		}

		if (regulator_count_voltages(data->vio) > 0) {
			rc = regulator_set_voltage(data->vio,
				HSCDTD007A_VIO_MIN_UV, HSCDTD007A_VIO_MAX_UV);
			if (rc) {
				dev_err(&data->i2c->dev,
				"Regulator set failed vio rc=%d\n", rc);
				goto reg_vio_put;
			}
		}
	}

	return 0;

reg_vio_put:
	regulator_put(data->vio);
reg_vdd_set:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, HSCDTD007A_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;
}
/**************************************************************/
#ifdef CONFIG_OF
static int hscdtd_compass_parse_dt(struct device *dev,
				struct hscdtd_data *data)
{
	struct device_node *np = dev->of_node;
	u32 temp_val;
	int rc;

	rc = of_property_read_u32(np, "hscdtd,layout", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read hscdtd,layout\n");
		return rc;
	} else {
		data->layout = temp_val;
	}

	return 0;
}
#else
static int hscdtd_compass_parse_dt(struct device *dev,
				struct hscdtd_data *data)
{
	return -EINVAL;
}
#endif /* !CONFIG_OF */


/*--------------------------------------------------------------------------
 * work function
 *--------------------------------------------------------------------------*/
static void hscdtd_polling(struct work_struct *work)
{
	int xyz[HSCDTD_3AXIS_NUM];
	struct hscdtd_data *hscdtd =
	    container_of(work, struct hscdtd_data, work_data.work);

	mutex_lock(&hscdtd->lock);
	if (hscdtd->fsuspend) {
		mutex_unlock(&hscdtd->lock);
		return;
	}
	if (hscdtd->fskip)
		hscdtd->fskip = false;
	else {
		if (hscdtd->factive) {
			if (!hscdtd_get_magnetic_field_data(hscdtd, xyz)) {
				input_report_abs(hscdtd->input,
				    ABS_X, xyz[0]);
				input_report_abs(hscdtd->input,
				    ABS_Y, xyz[1]);
				input_report_abs(hscdtd->input,
				    ABS_Z, xyz[2]);
				/* hscdtd->input->sync = 0; */
				input_event(hscdtd->input,
				    EV_SYN, SYN_REPORT, 0);
			}
			if (++hscdtd->tcs_cnt > hscdtd->tcs_thr)
				hscdtd_tcs_setup(hscdtd);
			hscdtd_force_setup(hscdtd);	/* For next polling */
		}
	}
	if (hscdtd->factive) {
		schedule_delayed_work(&hscdtd->work_data,
		    msecs_to_jiffies(hscdtd->delay_msec));
	}
	mutex_unlock(&hscdtd->lock);
}


/*--------------------------------------------------------------------------
 * i2c device
 *--------------------------------------------------------------------------*/
static int hscdtd_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int rc;
	struct hscdtd_data *hscdtd;
	struct hscdtd_platform_data *pdata;

	dev_dbg(&client->adapter->dev,
		HSCDTD_LOG_TAG "%s\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->adapter->dev, "client not i2c capable\n");
		rc = -EIO;
		goto out_region;
	}

	hscdtd = kzalloc(sizeof(struct hscdtd_data), GFP_KERNEL);
	if (!hscdtd) {
		dev_err(&client->adapter->dev,
		    "failed to allocate memory for module data\n");
		rc = -ENOMEM;
		goto out_region;
	}
	/******* parsing devicetree *********/
	if (client->dev.of_node) {
		rc = hscdtd_compass_parse_dt(&client->dev, hscdtd);
		if (rc) {
			dev_err(&client->dev,
				"Unable to parse platfrom data err=%d\n", rc);
			goto out_region;
		}
	} else {
		if (client->dev.platform_data) {
			/* Copy platform data to local. */
			pdata = client->dev.platform_data;
			hscdtd->layout = pdata->layout;
		} else {
		/* Platform data is not available.
		   Layout and information should be set by each application. */
			hscdtd->layout = 0;
			dev_warn(&client->dev, "%s: No platform data.",
				__func__);
		}
	}

	/***** I2C initialization *****/
	hscdtd->i2c = client;
	i2c_set_clientdata(client, hscdtd);

	/* check connection */
	rc = hscdtd_compass_power_init(hscdtd, 1);
	if (rc < 0)
		goto exit_power_init_failed;
	rc = hscdtd_compass_power_set(hscdtd, 1);
	if (rc < 0)
		goto exit_power_set_failed;

	pdata = (struct hscdtd_platform_data *) client->dev.platform_data;
	if (!pdata) {
		dev_warn(&client->adapter->dev,
			 "Warning no platform data for hscdtd\n");
		memset(hscdtd->matrix, 0, sizeof(hscdtd->matrix));
		hscdtd->matrix[0] = 1;
		hscdtd->matrix[4] = 1;
		hscdtd->matrix[8] = 1;
	} else {
		memcpy(hscdtd->matrix, pdata->axis,
			sizeof(hscdtd->matrix));
		dev_dbg(&client->adapter->dev, "get platform_data");
	}

	mutex_init(&hscdtd->lock);

	hscdtd->factive = false;
	hscdtd->fsuspend = false;
	hscdtd->delay_msec = HSCDTD_INITIALL_DELAY;
	hscdtd->tcs_thr = HSCDTD_TCS_TIME / hscdtd->delay_msec;
	hscdtd->tcs_cnt = 0;

	rc = hscdtd_register_init(hscdtd);
	if (rc) {
		rc = -EIO;
		dev_err(&client->adapter->dev, "hscdtd_register_init\n");
		goto out_kzalloc;
	}
	dev_dbg(&client->adapter->dev,
	    "initialize %s sensor\n", HSCDTD_DRIVER_NAME);

	hscdtd->input = input_allocate_device();
	if (!hscdtd->input) {
		rc = -ENOMEM;
		dev_err(&client->adapter->dev, "input_allocate_device\n");
		goto out_kzalloc;
	}
	dev_dbg(&client->adapter->dev, "input_allocate_device\n");

	input_set_drvdata(hscdtd->input, hscdtd);

	hscdtd->input->name		= HSCDTD_INPUT_DEVICE_NAME;
	hscdtd->input->id.bustype	= BUS_I2C;
	hscdtd->input->evbit[0]		= BIT_MASK(EV_ABS);

	input_set_abs_params(hscdtd->input, ABS_X, -16384, 16383, 0, 0);
	input_set_abs_params(hscdtd->input, ABS_Y, -16384, 16383, 0, 0);
	input_set_abs_params(hscdtd->input, ABS_Z, -16384, 16383, 0, 0);

	rc = input_register_device(hscdtd->input);
	if (rc) {
		rc = -ENOMEM;
		dev_err(&client->adapter->dev, "input_register_device\n");
		goto out_idev_allc;
	}
	dev_dbg(&client->adapter->dev, "input_register_device\n");

	INIT_DELAYED_WORK(&hscdtd->work_data, hscdtd_polling);

	rc = hscdtd_create_sysfs(&hscdtd->input->dev);
	if (rc) {
		rc = -ENOMEM;
		dev_err(&client->adapter->dev, "hscdtd_create_sysfs\n");
		goto out_idev_reg;
	}
	dev_dbg(&client->adapter->dev, "hscdtd_create_sysfs\n");

	hscdtd->cdev = sensors_cdev;
	hscdtd->cdev.sensors_enable = hscdtd_enable_sensors_class;
	hscdtd->cdev.sensors_poll_delay = hscdtd_delay_sensors_class;
	rc = sensors_classdev_register(&client->dev, &hscdtd->cdev);
	if (rc) {
		dev_err(&client->adapter->dev,
		    "sensors_classdev_register\n");
		goto out_sysfs;
	}
	dev_dbg(&client->adapter->dev, "sensors_classdev_register\n");

#ifdef CONFIG_HAS_EARLYSUSPEND
	hscdtd->early_suspend_h.suspend = hscdtd_early_suspend;
	hscdtd->early_suspend_h.resume  = hscdtd_early_resume;
	register_early_suspend(&hscdtd->early_suspend_h);
	dev_dbg(&client->adapter->dev, "register_early_suspend\n");
#endif

	hscdtd_compass_power_set(hscdtd, false);

	dev_info(&client->adapter->dev,
	    HSCDTD_LOG_TAG "detected %s geomagnetic field sensor\n",
	    HSCDTD_DRIVER_NAME);

	return 0;

out_sysfs:
	hscdtd_remove_sysfs(&hscdtd->input->dev);
out_idev_reg:
	input_unregister_device(hscdtd->input);
out_idev_allc:
	input_free_device(hscdtd->input);
out_kzalloc:
	hscdtd_compass_power_set(hscdtd, 0);
	mutex_destroy(&hscdtd->lock);
exit_power_set_failed:
	hscdtd_compass_power_init(hscdtd, 0);
exit_power_init_failed:
	kfree(hscdtd);
out_region:

	return rc;
}

static int hscdtd_remove(struct i2c_client *client)
{
	struct hscdtd_data *hscdtd = i2c_get_clientdata(client);

	dev_dbg(&client->adapter->dev, HSCDTD_LOG_TAG "%s\n", __func__);

/***********************************************/
	if (hscdtd_compass_power_set(hscdtd, 0))
		dev_err(&client->dev, "power set failed.");
	if (hscdtd_compass_power_init(hscdtd, 0))
		dev_err(&client->dev, "power deinit failed.");
/**********************************************/
	hscdtd_measure_setup(hscdtd, false);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&hscdtd->early_suspend_h);
#endif
	sensors_classdev_unregister(&hscdtd->cdev);
	hscdtd_remove_sysfs(&hscdtd->input->dev);
	input_unregister_device(hscdtd->input);
	input_free_device(hscdtd->input);
	mutex_destroy(&hscdtd->lock);
	kfree(hscdtd);

	return 0;
}


/*--------------------------------------------------------------------------
 * module
 *--------------------------------------------------------------------------*/
static const struct i2c_device_id HSCDTD_id[] = {
	{ HSCDTD_DRIVER_NAME, 0 },
	{ }
};

static struct of_device_id hscdtd007a_match_table[] = {
	{ .compatible = "alps,hscdtd007a", },
	{ },
};

static struct i2c_driver hscdtd_driver = {
	.probe		= hscdtd_probe,
	.remove		= hscdtd_remove,
	.id_table	= HSCDTD_id,
	.driver		= {
		.owner	= THIS_MODULE,
		.name = HSCDTD_DRIVER_NAME,
		.of_match_table = hscdtd007a_match_table,
	},
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= hscdtd_suspend,
	.resume		= hscdtd_resume,
#endif
};

static int __init hscdtd_init(void)
{
	pr_debug(HSCDTD_LOG_TAG "%s\n", __func__);
	return i2c_add_driver(&hscdtd_driver);
}

static void __exit hscdtd_exit(void)
{
	pr_debug(HSCDTD_LOG_TAG "%s\n", __func__);
	i2c_del_driver(&hscdtd_driver);
}

module_init(hscdtd_init);
module_exit(hscdtd_exit);

MODULE_DESCRIPTION("ALPS Geomagnetic Device");
MODULE_AUTHOR("ALPS ELECTRIC CO., LTD.");
MODULE_LICENSE("GPL v2");