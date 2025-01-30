// SPDX-License-Identifier: GPL-2.0
/*
 * ov13b10 driver
 *
 * Copyright (C) 2023 Rockchip Electronics Co., Ltd.
 *
 * V0.0X01.0X01 add poweron function.
 * V0.0X01.0X02 fix mclk issue when probe multiple camera.
 * V0.0X01.0X03 add enum_frame_interval function.
 * V0.0X01.0X04 add quick stream on/off
 * V0.0X01.0X05 add function g_mbus_config
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/rk-camera-module.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <linux/pinctrl/consumer.h>

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x05)

#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN		V4L2_CID_GAIN
#endif

#define OV13B10_LINK_FREQ_560MHZ	560000000ULL
#define OV13B10_PIXEL_RATE		(OV13B10_LINK_FREQ_560MHZ * 2 * 4 / 10)
#define OV13B10_XVCLK_FREQ		24000000

#define CHIP_ID				0x560d42
#define OV13B10_REG_CHIP_ID		0x300a

#define OV13B10_REG_CTRL_MODE		0x0100
#define OV13B10_MODE_SW_STANDBY		0x0
#define OV13B10_MODE_STREAMING		BIT(0)

#define OV13B10_REG_EXPOSURE		0x3500
#define	OV13B10_EXPOSURE_MIN		4
#define	OV13B10_EXPOSURE_STEP		1
#define OV13B10_VTS_MAX			0x7fff

#define OV13B10_REG_GAIN_H		0x350a
#define OV13B10_REG_GAIN_L		0x350b
#define OV13B10_GAIN_H_MASK		0x07
#define OV13B10_GAIN_H_SHIFT		8
#define OV13B10_GAIN_L_MASK		0xff
#define OV13B10_GAIN_MIN		0x80
#define OV13B10_GAIN_MAX		0x07c0
#define OV13B10_GAIN_STEP		1
#define OV13B10_GAIN_DEFAULT		0x80

#define OV13B10_REG_TEST_PATTERN	0x5080
#define	OV13B10_TEST_PATTERN_ENABLE	BIT(7)
#define	OV13B10_TEST_PATTERN_DISABLE	0x0

#define OV13B10_REG_VTS			0x380e

#define REG_NULL			0xFFFF

#define OV13B10_REG_VALUE_08BIT		1
#define OV13B10_REG_VALUE_16BIT		2
#define OV13B10_REG_VALUE_24BIT		3

#define OV13B10_LANES			4
#define OV13B10_BITS_PER_SAMPLE		10

#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"

#define OV13B10_NAME			"ov13b10"

static const char * const ov13b10_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define OV13B10_NUM_SUPPLIES ARRAY_SIZE(ov13b10_supply_names)

struct regval {
	u16 addr;
	u8 val;
};

struct ov13b10_mode {
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	const struct regval *reg_list;
};

struct ov13b10 {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*power_gpio;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct regulator_bulk_data supplies[OV13B10_NUM_SUPPLIES];

	struct pinctrl		*pinctrl;
	struct pinctrl_state	*pins_default;
	struct pinctrl_state	*pins_sleep;

	struct v4l2_subdev	subdev;
	struct media_pad	pad;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl	*exposure;
	struct v4l2_ctrl	*anal_gain;
	struct v4l2_ctrl	*digi_gain;
	struct v4l2_ctrl	*hblank;
	struct v4l2_ctrl	*vblank;
	struct v4l2_ctrl	*test_pattern;
	struct mutex		mutex;
	bool			streaming;
	bool			power_on;
	const struct ov13b10_mode *cur_mode;
	u32			module_index;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
};

#define to_ov13b10(sd) container_of(sd, struct ov13b10, subdev)

static const struct regval ov13b10_global_regs[] = {
	{0x0103, 0x01},
	{0x0303, 0x04},
	{0x0305, 0xaf},
	{0x0321, 0x00},
	{0x0323, 0x04},
	{0x0324, 0x01},
	{0x0325, 0xa4},
	{0x0326, 0x81},
	{0x0327, 0x04},
	{0x3012, 0x07},
	{0x3013, 0x32},
	{0x3107, 0x23},
	{0x3501, 0x0c},
	{0x3502, 0x10},
	{0x3504, 0x08},
	{0x3508, 0x07},
	{0x3509, 0xc0},
	{0x3600, 0x16},
	{0x3601, 0x54},
	{0x3612, 0x4e},
	{0x3620, 0x00},
	{0x3621, 0x68},
	{0x3622, 0x66},
	{0x3623, 0x03},
	{0x3662, 0x92},
	{0x3666, 0xbb},
	{0x3667, 0x44},
	{0x366e, 0xff},
	{0x366f, 0xf3},
	{0x3675, 0x44},
	{0x3676, 0x00},
	{0x367f, 0xe9},
	{0x3681, 0x32},
	{0x3682, 0x1f},
	{0x3683, 0x0b},
	{0x3684, 0x0b},
	{0x3704, 0x0f},
	{0x3706, 0x40},
	{0x3708, 0x3b},
	{0x3709, 0x72},
	{0x370b, 0xa2},
	{0x3714, 0x24},
	{0x371a, 0x3e},
	{0x3725, 0x42},
	{0x3739, 0x12},
	{0x3767, 0x00},
	{0x377a, 0x0d},
	{0x3789, 0x18},
	{0x3790, 0x40},
	{0x3791, 0xa2},
	{0x37c2, 0x04},
	{0x37c3, 0xf1},
	{0x37d9, 0x0c},
	{0x37da, 0x02},
	{0x37dc, 0x02},
	{0x37e1, 0x04},
	{0x37e2, 0x0a},
	{0x3800, 0x00},
	{0x3801, 0x00},
	{0x3802, 0x00},
	{0x3803, 0x08},
	{0x3804, 0x10},
	{0x3805, 0x8f},
	{0x3806, 0x0c},
	{0x3807, 0x47},
	{0x3808, 0x10},
	{0x3809, 0x70},
	{0x380a, 0x0c},
	{0x380b, 0x30},
	{0x380c, 0x04},
	{0x380d, 0x98},
	{0x380e, 0x0c},
	{0x380f, 0x7c},
	{0x3811, 0x0f},
	{0x3813, 0x09},
	{0x3814, 0x01},
	{0x3815, 0x01},
	{0x3816, 0x01},
	{0x3817, 0x01},
	{0x381f, 0x08},
	{0x3820, 0x88},
	{0x3821, 0x00},
	{0x3822, 0x14},
	{0x382e, 0xe6},
	{0x3c80, 0x00},
	{0x3c87, 0x01},
	{0x3c8c, 0x19},
	{0x3c8d, 0x1c},
	{0x3ca0, 0x00},
	{0x3ca1, 0x00},
	{0x3ca2, 0x00},
	{0x3ca3, 0x00},
	{0x3ca4, 0x50},
	{0x3ca5, 0x11},
	{0x3ca6, 0x01},
	{0x3ca7, 0x00},
	{0x3ca8, 0x00},
	{0x4008, 0x02},
	{0x4009, 0x0f},
	{0x400a, 0x01},
	{0x400b, 0x19},
	{0x4011, 0x21},
	{0x4017, 0x08},
	{0x4019, 0x04},
	{0x401a, 0x58},
	{0x4032, 0x1e},
	{0x4050, 0x02},
	{0x4051, 0x09},
	{0x405e, 0x00},
	{0x4066, 0x02},
	{0x4501, 0x00},
	{0x4502, 0x10},
	{0x4505, 0x00},
	{0x4800, 0x64},
	{0x481b, 0x3e},
	{0x481f, 0x30},
	{0x4825, 0x34},
	{0x4837, 0x0e},
	{0x484b, 0x01},
	{0x4883, 0x02},
	{0x5000, 0xff},
	{0x5001, 0x0f},
	{0x5045, 0x20},
	{0x5046, 0x20},
	{0x5047, 0xa4},
	{0x5048, 0x20},
	{0x5049, 0xa4},
	{0x0100, 0x01},
};

static const struct regval mode_4208x3120_regs[] = {
	{0x0305, 0xaf},
	{0x3501, 0x0c},
	{0x3662, 0x92},
	{0x3714, 0x24},
	{0x3739, 0x12},
	{0x37c2, 0x04},
	{0x37d9, 0x0c},
	{0x37e2, 0x0a},
	{0x3800, 0x00},
	{0x3801, 0x00},
	{0x3802, 0x00},
	{0x3803, 0x08},
	{0x3804, 0x10},
	{0x3805, 0x8f},
	{0x3806, 0x0c},
	{0x3807, 0x47},
	{0x3808, 0x10},
	{0x3809, 0x70},
	{0x380a, 0x0c},
	{0x380b, 0x30},
	{0x380c, 0x04},
	{0x380d, 0x98},
	{0x380e, 0x0c},
	{0x380f, 0x7c},
	{0x3810, 0x00},
	{0x3811, 0x0f},
	{0x3812, 0x00},
	{0x3813, 0x09},
	{0x3814, 0x01},
	{0x3816, 0x01},
	{0x3820, 0x88},
	{0x3c8c, 0x19},
	{0x4008, 0x02},
	{0x4009, 0x0f},
	{0x4050, 0x02},
	{0x4051, 0x09},
	{0x4501, 0x00},
	{0x4505, 0x00},
	{0x4837, 0x0e},
	{0x5000, 0xff},
	{0x5001, 0x0f},
};

static const struct regval mode_4160x3120_regs[] = {
	{0x0305, 0xaf},
	{0x3501, 0x0c},
	{0x3662, 0x92},
	{0x3714, 0x24},
	{0x3739, 0x12},
	{0x37c2, 0x04},
	{0x37d9, 0x0c},
	{0x37e2, 0x0a},
	{0x3800, 0x00},
	{0x3801, 0x00},
	{0x3802, 0x00},
	{0x3803, 0x08},
	{0x3804, 0x10},
	{0x3805, 0x8f},
	{0x3806, 0x0c},
	{0x3807, 0x47},
	{0x3808, 0x10},
	{0x3809, 0x40},
	{0x380a, 0x0c},
	{0x380b, 0x30},
	{0x380c, 0x04},
	{0x380d, 0x98},
	{0x380e, 0x0c},
	{0x380f, 0x7c},
	{0x3810, 0x00},
	{0x3811, 0x27},
	{0x3812, 0x00},
	{0x3813, 0x09},
	{0x3814, 0x01},
	{0x3816, 0x01},
	{0x3820, 0x88},
	{0x3c8c, 0x19},
	{0x4008, 0x02},
	{0x4009, 0x0f},
	{0x4050, 0x02},
	{0x4051, 0x09},
	{0x4501, 0x00},
	{0x4505, 0x00},
	{0x4837, 0x0e},
	{0x5000, 0xff},
	{0x5001, 0x0f},
};

static const struct regval mode_4160x2340_regs[] = {
	{0x0305, 0xaf},
	{0x3501, 0x0c},
	{0x3662, 0x92},
	{0x3714, 0x24},
	{0x3739, 0x12},
	{0x37c2, 0x04},
	{0x37d9, 0x0c},
	{0x37e2, 0x0a},
	{0x3800, 0x00},
	{0x3801, 0x00},
	{0x3802, 0x00},
	{0x3803, 0x08},
	{0x3804, 0x10},
	{0x3805, 0x8f},
	{0x3806, 0x0c},
	{0x3807, 0x47},
	{0x3808, 0x10},
	{0x3809, 0x40},
	{0x380a, 0x09},
	{0x380b, 0x24},
	{0x380c, 0x04},
	{0x380d, 0x98},
	{0x380e, 0x0c},
	{0x380f, 0x7c},
	{0x3810, 0x00},
	{0x3811, 0x27},
	{0x3812, 0x01},
	{0x3813, 0x8f},
	{0x3814, 0x01},
	{0x3816, 0x01},
	{0x3820, 0x88},
	{0x3c8c, 0x19},
	{0x4008, 0x02},
	{0x4009, 0x0f},
	{0x4050, 0x02},
	{0x4051, 0x09},
	{0x4501, 0x00},
	{0x4505, 0x00},
	{0x4837, 0x0e},
	{0x5000, 0xff},
	{0x5001, 0x0f},
};

static const struct regval mode_2104x1560_regs[] = {
	{0x0305, 0xaf},
	{0x3501, 0x06},
	{0x3662, 0x88},
	{0x3714, 0x28},
	{0x3739, 0x10},
	{0x37c2, 0x14},
	{0x37d9, 0x06},
	{0x37e2, 0x0c},
	{0x3800, 0x00},
	{0x3801, 0x00},
	{0x3802, 0x00},
	{0x3803, 0x08},
	{0x3804, 0x10},
	{0x3805, 0x8f},
	{0x3806, 0x0c},
	{0x3807, 0x47},
	{0x3808, 0x08},
	{0x3809, 0x38},
	{0x380a, 0x06},
	{0x380b, 0x18},
	{0x380c, 0x04},
	{0x380d, 0x98},
	{0x380e, 0x06},
	{0x380f, 0x3e},
	{0x3810, 0x00},
	{0x3811, 0x07},
	{0x3812, 0x00},
	{0x3813, 0x05},
	{0x3814, 0x03},
	{0x3816, 0x03},
	{0x3820, 0x8b},
	{0x3c8c, 0x18},
	{0x4008, 0x00},
	{0x4009, 0x05},
	{0x4050, 0x00},
	{0x4051, 0x05},
	{0x4501, 0x08},
	{0x4505, 0x00},
	{0x4837, 0x0e},
	{0x5000, 0xfd},
	{0x5001, 0x0d},
};

static const struct regval mode_2080x1170_regs[] = {
	{0x0305, 0xaf},
	{0x3501, 0x06},
	{0x3662, 0x88},
	{0x3714, 0x28},
	{0x3739, 0x10},
	{0x37c2, 0x14},
	{0x37d9, 0x06},
	{0x37e2, 0x0c},
	{0x3800, 0x00},
	{0x3801, 0x00},
	{0x3802, 0x00},
	{0x3803, 0x08},
	{0x3804, 0x10},
	{0x3805, 0x8f},
	{0x3806, 0x0c},
	{0x3807, 0x47},
	{0x3808, 0x08},
	{0x3809, 0x20},
	{0x380a, 0x04},
	{0x380b, 0x92},
	{0x380c, 0x04},
	{0x380d, 0x98},
	{0x380e, 0x06},
	{0x380f, 0x3e},
	{0x3810, 0x00},
	{0x3811, 0x13},
	{0x3812, 0x00},
	{0x3813, 0xc9},
	{0x3814, 0x03},
	{0x3816, 0x03},
	{0x3820, 0x8b},
	{0x3c8c, 0x18},
	{0x4008, 0x00},
	{0x4009, 0x05},
	{0x4050, 0x00},
	{0x4051, 0x05},
	{0x4501, 0x08},
	{0x4505, 0x00},
	{0x4837, 0x0e},
	{0x5000, 0xfd},
	{0x5001, 0x0d},
};

static const struct ov13b10_mode supported_modes[] = {
	{
		.width = 4208,
		.height = 3120,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x0c00,
		.hts_def = 0x0498,
		.vts_def = 0x0c7c,
		.reg_list = mode_4208x3120_regs,
	}, {
		.width = 4160,
		.height = 3120,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x0c00,
		.hts_def = 0x0498,
		.vts_def = 0x0c7c,
		.reg_list = mode_4160x3120_regs,
	}, {
		.width = 4160,
		.height = 2340,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x0c00,
		.hts_def = 0x0498,
		.vts_def = 0x0c7c,
		.reg_list = mode_4160x2340_regs,
	},
	{
		.width = 2104,
		.height = 1560,
		.max_fps = {
			.numerator = 10000,
			.denominator = 600000,
		},
		.exp_def = 0x0c00,
		.hts_def = 0x0498,
		.vts_def = 0x0c7c,
		.reg_list = mode_2104x1560_regs,
	},
	{
		.width = 2080,
		.height = 1170,
		.max_fps = {
			.numerator = 10000,
			.denominator = 600000,
		},
		.exp_def = 0x0c00,
		.hts_def = 0x0498,
		.vts_def = 0x0c7c,
		.reg_list = mode_2080x1170_regs,
	}
};

static const s64 link_freq_menu_items[] = {
	OV13B10_LINK_FREQ_560MHZ
};

static const char * const ov13b10_test_pattern_menu[] = {
	"Disabled",
	"Vertical Color Bar Type 1",
	"Vertical Color Bar Type 2",
	"Vertical Color Bar Type 3",
	"Vertical Color Bar Type 4"
};

/* Write registers up to 4 at a time */
static int ov13b10_write_reg(struct i2c_client *client, u16 reg,
			     u32 len, u32 val)
{
	u32 buf_i, val_i;
	u8 buf[6];
	u8 *val_p;
	__be32 val_be;

	dev_dbg(&client->dev, "write reg(0x%x val:0x%x)!\n", reg, val);

	if (len > 4)
		return -EINVAL;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;

	val_be = cpu_to_be32(val);
	val_p = (u8 *)&val_be;
	buf_i = 2;
	val_i = 4 - len;

	while (val_i < 4)
		buf[buf_i++] = val_p[val_i++];

	if (i2c_master_send(client, buf, len + 2) != len + 2)
		return -EIO;

	return 0;
}

static int ov13b10_write_array(struct i2c_client *client,
			       const struct regval *regs)
{
	u32 i;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++)
		ret = ov13b10_write_reg(client, regs[i].addr,
					OV13B10_REG_VALUE_08BIT,
					regs[i].val);

	return ret;
}

/* Read registers up to 4 at a time */
static int ov13b10_read_reg(struct i2c_client *client, u16 reg,
			    unsigned int len, u32 *val)
{
	struct i2c_msg msgs[2];
	u8 *data_be_p;
	__be32 data_be = 0;
	__be16 reg_addr_be = cpu_to_be16(reg);
	int ret;

	if (len > 4 || !len)
		return -EINVAL;

	data_be_p = (u8 *)&data_be;
	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = (u8 *)&reg_addr_be;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_be_p[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = be32_to_cpu(data_be);

	return 0;
}

static int ov13b10_get_reso_dist(const struct ov13b10_mode *mode,
				 struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct ov13b10_mode *
ov13b10_find_best_fit(struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		dist = ov13b10_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

static int ov13b10_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *sd_state,
			  struct v4l2_subdev_format *fmt)
{
	struct ov13b10 *ov13b10 = to_ov13b10(sd);
	const struct ov13b10_mode *mode;
	s64 h_blank, vblank_def;

	mutex_lock(&ov13b10->mutex);

	mode = ov13b10_find_best_fit(fmt);
	fmt->format.code = MEDIA_BUS_FMT_SGRBG10_1X10;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, sd_state, fmt->pad) = fmt->format;
#else
		mutex_unlock(&ov13b10->mutex);
		return -ENOTTY;
#endif
	} else {
		ov13b10->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(ov13b10->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(ov13b10->vblank, vblank_def,
					 OV13B10_VTS_MAX - mode->height,
					 1, vblank_def);
	}

	mutex_unlock(&ov13b10->mutex);

	return 0;
}

static int ov13b10_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *sd_state,
			   struct v4l2_subdev_format *fmt)
{
	struct ov13b10 *ov13b10 = to_ov13b10(sd);
	const struct ov13b10_mode *mode = ov13b10->cur_mode;

	mutex_lock(&ov13b10->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, sd_state, fmt->pad);
#else
		mutex_unlock(&ov13b10->mutex);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = MEDIA_BUS_FMT_SGRBG10_1X10;
		fmt->format.field = V4L2_FIELD_NONE;
	}
	mutex_unlock(&ov13b10->mutex);

	return 0;
}

static int ov13b10_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index != 0)
		return -EINVAL;
	code->code = MEDIA_BUS_FMT_SGRBG10_1X10;

	return 0;
}

static int ov13b10_enum_frame_sizes(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *sd_state,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fse->code != MEDIA_BUS_FMT_SGRBG10_1X10)
		return -EINVAL;

	fse->min_width  = supported_modes[fse->index].width;
	fse->max_width  = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;

	return 0;
}

static int ov13b10_enable_test_pattern(struct ov13b10 *ov13b10, u32 pattern)
{
	u32 val;

	if (pattern)
		val = (pattern - 1) | OV13B10_TEST_PATTERN_ENABLE;
	else
		val = OV13B10_TEST_PATTERN_DISABLE;

	return ov13b10_write_reg(ov13b10->client,
				 OV13B10_REG_TEST_PATTERN,
				 OV13B10_REG_VALUE_08BIT,
				 val);
}

static int ov13b10_g_frame_interval(struct v4l2_subdev *sd,
				    struct v4l2_subdev_frame_interval *fi)
{
	struct ov13b10 *ov13b10 = to_ov13b10(sd);
	const struct ov13b10_mode *mode = ov13b10->cur_mode;

	fi->interval = mode->max_fps;

	return 0;
}

static void ov13b10_get_module_inf(struct ov13b10 *ov13b10,
				   struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strlcpy(inf->base.sensor, OV13B10_NAME, sizeof(inf->base.sensor));
	strlcpy(inf->base.module, ov13b10->module_name,
		sizeof(inf->base.module));
	strlcpy(inf->base.lens, ov13b10->len_name, sizeof(inf->base.lens));
}

static long ov13b10_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct ov13b10 *ov13b10 = to_ov13b10(sd);
	long ret = 0;
	u32 stream = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		ov13b10_get_module_inf(ov13b10, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		stream = *((u32 *)arg);
		if (stream)
			ret = ov13b10_write_reg(ov13b10->client,
				 OV13B10_REG_CTRL_MODE,
				 OV13B10_REG_VALUE_08BIT,
				 OV13B10_MODE_STREAMING);
		else
			ret = ov13b10_write_reg(ov13b10->client,
				 OV13B10_REG_CTRL_MODE,
				 OV13B10_REG_VALUE_08BIT,
				 OV13B10_MODE_SW_STANDBY);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long ov13b10_compat_ioctl32(struct v4l2_subdev *sd,
				   unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	struct rkmodule_awb_cfg *cfg;
	long ret;
	u32 stream = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = ov13b10_ioctl(sd, cmd, inf);
		if (!ret)
			ret = copy_to_user(up, inf, sizeof(*inf));
		kfree(inf);
		break;
	case RKMODULE_AWB_CFG:
		cfg = kzalloc(sizeof(*cfg), GFP_KERNEL);
		if (!cfg) {
			ret = -ENOMEM;
			return ret;
		}

		ret = copy_from_user(cfg, up, sizeof(*cfg));
		if (!ret)
			ret = ov13b10_ioctl(sd, cmd, cfg);
		kfree(cfg);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		ret = copy_from_user(&stream, up, sizeof(u32));
		if (!ret)
			ret = ov13b10_ioctl(sd, cmd, &stream);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif

static int __ov13b10_start_stream(struct ov13b10 *ov13b10)
{
	int ret;

	ret = ov13b10_write_array(ov13b10->client, ov13b10->cur_mode->reg_list);
	if (ret)
		return ret;

	mutex_unlock(&ov13b10->mutex);
	ret = v4l2_ctrl_handler_setup(&ov13b10->ctrl_handler);
	mutex_lock(&ov13b10->mutex);
	if (ret)
		return ret;

	return ov13b10_write_reg(ov13b10->client,
				 OV13B10_REG_CTRL_MODE,
				 OV13B10_REG_VALUE_08BIT,
				 OV13B10_MODE_STREAMING);
}

static int __ov13b10_stop_stream(struct ov13b10 *ov13b10)
{
	return ov13b10_write_reg(ov13b10->client,
				 OV13B10_REG_CTRL_MODE,
				 OV13B10_REG_VALUE_08BIT,
				 OV13B10_MODE_SW_STANDBY);
}

static int ov13b10_s_stream(struct v4l2_subdev *sd, int on)
{
	struct ov13b10 *ov13b10 = to_ov13b10(sd);
	struct i2c_client *client = ov13b10->client;
	int ret = 0;

	mutex_lock(&ov13b10->mutex);
	on = !!on;
	if (on == ov13b10->streaming)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __ov13b10_start_stream(ov13b10);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__ov13b10_stop_stream(ov13b10);
		pm_runtime_put(&client->dev);
	}

	ov13b10->streaming = on;

unlock_and_return:
	mutex_unlock(&ov13b10->mutex);

	return ret;
}

static int ov13b10_s_power(struct v4l2_subdev *sd, int on)
{
	struct ov13b10 *ov13b10 = to_ov13b10(sd);
	struct i2c_client *client = ov13b10->client;
	int ret = 0;

	mutex_lock(&ov13b10->mutex);

	if (ov13b10->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = ov13b10_write_array(client, ov13b10_global_regs);
		if (ret) {
			v4l2_err(sd, "could not set init registers\n");
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ov13b10->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		ov13b10->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&ov13b10->mutex);

	return ret;
}

static inline u32 ov13b10_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, OV13B10_XVCLK_FREQ / 1000 / 1000);
}

static int __ov13b10_power_on(struct ov13b10 *ov13b10)
{
	int ret;
	u32 delay_us;
	struct device *dev = &ov13b10->client->dev;

	if (!IS_ERR(ov13b10->power_gpio))
		gpiod_set_value_cansleep(ov13b10->power_gpio, 1);

	usleep_range(1000, 2000);

	if (!IS_ERR_OR_NULL(ov13b10->pins_default)) {
		ret = pinctrl_select_state(ov13b10->pinctrl,
					   ov13b10->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}
	ret = clk_set_rate(ov13b10->xvclk, OV13B10_XVCLK_FREQ);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate (24MHz)\n");
	if (clk_get_rate(ov13b10->xvclk) != OV13B10_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched, modes are based on 24MHz\n");
	ret = clk_prepare_enable(ov13b10->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}
	if (!IS_ERR(ov13b10->reset_gpio))
		gpiod_set_value_cansleep(ov13b10->reset_gpio, 0);

	ret = regulator_bulk_enable(OV13B10_NUM_SUPPLIES, ov13b10->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	if (!IS_ERR(ov13b10->reset_gpio))
		gpiod_set_value_cansleep(ov13b10->reset_gpio, 1);

	usleep_range(500, 1000);
	if (!IS_ERR(ov13b10->pwdn_gpio))
		gpiod_set_value_cansleep(ov13b10->pwdn_gpio, 1);

	delay_us = ov13b10_cal_delay(8192);
	usleep_range(delay_us, delay_us * 2);

	return 0;

disable_clk:
	clk_disable_unprepare(ov13b10->xvclk);

	return ret;
}

static void __ov13b10_power_off(struct ov13b10 *ov13b10)
{
	int ret;
	struct device *dev = &ov13b10->client->dev;

	if (!IS_ERR(ov13b10->pwdn_gpio))
		gpiod_set_value_cansleep(ov13b10->pwdn_gpio, 0);
	clk_disable_unprepare(ov13b10->xvclk);
	if (!IS_ERR(ov13b10->reset_gpio))
		gpiod_set_value_cansleep(ov13b10->reset_gpio, 0);

	if (!IS_ERR_OR_NULL(ov13b10->pins_sleep)) {
		ret = pinctrl_select_state(ov13b10->pinctrl,
					   ov13b10->pins_sleep);
		if (ret < 0)
			dev_dbg(dev, "could not set pins\n");
	}
	if (!IS_ERR(ov13b10->power_gpio))
		gpiod_set_value_cansleep(ov13b10->power_gpio, 0);

	regulator_bulk_disable(OV13B10_NUM_SUPPLIES, ov13b10->supplies);
}

static int __maybe_unused ov13b10_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov13b10 *ov13b10 = to_ov13b10(sd);

	return __ov13b10_power_on(ov13b10);
}

static int __maybe_unused ov13b10_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov13b10 *ov13b10 = to_ov13b10(sd);

	__ov13b10_power_off(ov13b10);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int ov13b10_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct ov13b10 *ov13b10 = to_ov13b10(sd);
	struct v4l2_mbus_framefmt *try_fmt =
				v4l2_subdev_get_try_format(sd, fh->state, 0);
	const struct ov13b10_mode *def_mode = &supported_modes[0];

	mutex_lock(&ov13b10->mutex);
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = MEDIA_BUS_FMT_SGRBG10_1X10;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&ov13b10->mutex);

	return 0;
}
#endif

static int ov13b10_enum_frame_interval(struct v4l2_subdev *sd,
				       struct v4l2_subdev_state *sd_state,
				       struct v4l2_subdev_frame_interval_enum *fie)
{
	if (fie->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	fie->code = MEDIA_BUS_FMT_SGRBG10_1X10;
	fie->width = supported_modes[fie->index].width;
	fie->height = supported_modes[fie->index].height;
	fie->interval = supported_modes[fie->index].max_fps;
	return 0;
}

static int ov13b10_g_mbus_config(struct v4l2_subdev *sd, unsigned int pad_id,
				struct v4l2_mbus_config *config)
{
	config->type = V4L2_MBUS_CSI2_DPHY;
	config->bus.mipi_csi2.num_data_lanes = OV13B10_LANES;

	return 0;
}

static const struct dev_pm_ops ov13b10_pm_ops = {
	SET_RUNTIME_PM_OPS(ov13b10_runtime_suspend,
			   ov13b10_runtime_resume, NULL)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops ov13b10_internal_ops = {
	.open = ov13b10_open,
};
#endif

static const struct v4l2_subdev_core_ops ov13b10_core_ops = {
	.s_power = ov13b10_s_power,
	.ioctl = ov13b10_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = ov13b10_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops ov13b10_video_ops = {
	.s_stream = ov13b10_s_stream,
	.g_frame_interval = ov13b10_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops ov13b10_pad_ops = {
	.enum_mbus_code = ov13b10_enum_mbus_code,
	.enum_frame_size = ov13b10_enum_frame_sizes,
	.enum_frame_interval = ov13b10_enum_frame_interval,
	.get_fmt = ov13b10_get_fmt,
	.set_fmt = ov13b10_set_fmt,
	.get_mbus_config = ov13b10_g_mbus_config,
};

static const struct v4l2_subdev_ops ov13b10_subdev_ops = {
	.core	= &ov13b10_core_ops,
	.video	= &ov13b10_video_ops,
	.pad	= &ov13b10_pad_ops,
};

static int ov13b10_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ov13b10 *ov13b10 = container_of(ctrl->handler,
					     struct ov13b10, ctrl_handler);
	struct i2c_client *client = ov13b10->client;
	s64 max;
	int ret = 0;

	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		max = ov13b10->cur_mode->height + ctrl->val - 16;
		__v4l2_ctrl_modify_range(ov13b10->exposure,
					 ov13b10->exposure->minimum, max,
					 ov13b10->exposure->step,
					 ov13b10->exposure->default_value);
		break;
	}

	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		ret = ov13b10_write_reg(ov13b10->client,
					OV13B10_REG_EXPOSURE,
					OV13B10_REG_VALUE_24BIT,
					ctrl->val);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = ov13b10_write_reg(ov13b10->client,
					OV13B10_REG_GAIN_H,
					OV13B10_REG_VALUE_08BIT,
					(ctrl->val >> OV13B10_GAIN_H_SHIFT) &
					OV13B10_GAIN_H_MASK);
		ret |= ov13b10_write_reg(ov13b10->client,
					 OV13B10_REG_GAIN_L,
					 OV13B10_REG_VALUE_08BIT,
					 ctrl->val & OV13B10_GAIN_L_MASK);
		break;
	case V4L2_CID_VBLANK:
		ret = ov13b10_write_reg(ov13b10->client,
					OV13B10_REG_VTS,
					OV13B10_REG_VALUE_16BIT,
					ctrl->val + ov13b10->cur_mode->height);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = ov13b10_enable_test_pattern(ov13b10, ctrl->val);
		break;
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops ov13b10_ctrl_ops = {
	.s_ctrl = ov13b10_set_ctrl,
};

static int ov13b10_initialize_controls(struct ov13b10 *ov13b10)
{
	const struct ov13b10_mode *mode;
	struct v4l2_ctrl_handler *handler;
	struct v4l2_ctrl *ctrl;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;

	handler = &ov13b10->ctrl_handler;
	mode = ov13b10->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 8);
	if (ret)
		return ret;
	handler->lock = &ov13b10->mutex;

	ctrl = v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ,
				      0, 0, link_freq_menu_items);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
			  0, OV13B10_PIXEL_RATE, 1, OV13B10_PIXEL_RATE);

	h_blank = mode->hts_def - mode->width;
	ov13b10->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
				h_blank, h_blank, 1, h_blank);
	if (ov13b10->hblank)
		ov13b10->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = mode->vts_def - mode->height;
	ov13b10->vblank = v4l2_ctrl_new_std(handler, &ov13b10_ctrl_ops,
				V4L2_CID_VBLANK, vblank_def,
				OV13B10_VTS_MAX - mode->height,
				1, vblank_def);

	exposure_max = mode->vts_def - 16;
	ov13b10->exposure = v4l2_ctrl_new_std(handler, &ov13b10_ctrl_ops,
				V4L2_CID_EXPOSURE, OV13B10_EXPOSURE_MIN,
				exposure_max, OV13B10_EXPOSURE_STEP,
				mode->exp_def);

	ov13b10->anal_gain = v4l2_ctrl_new_std(handler, &ov13b10_ctrl_ops,
				V4L2_CID_ANALOGUE_GAIN, OV13B10_GAIN_MIN,
				OV13B10_GAIN_MAX, OV13B10_GAIN_STEP,
				OV13B10_GAIN_DEFAULT);

	ov13b10->test_pattern = v4l2_ctrl_new_std_menu_items(handler,
				&ov13b10_ctrl_ops, V4L2_CID_TEST_PATTERN,
				ARRAY_SIZE(ov13b10_test_pattern_menu) - 1,
				0, 0, ov13b10_test_pattern_menu);

	if (handler->error) {
		ret = handler->error;
		dev_err(&ov13b10->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	ov13b10->subdev.ctrl_handler = handler;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int ov13b10_check_sensor_id(struct ov13b10 *ov13b10,
				   struct i2c_client *client)
{
	struct device *dev = &ov13b10->client->dev;
	u32 id = 0;
	int ret;

	ret = ov13b10_read_reg(client, OV13B10_REG_CHIP_ID,
			       OV13B10_REG_VALUE_24BIT, &id);
	if (id != CHIP_ID) {
		dev_err(dev, "Unexpected sensor id(%06x), ret(%d)\n", id, ret);
		return -ENODEV;
	}

	dev_info(dev, "Detected OV%06x sensor\n", CHIP_ID);

	return 0;
}

static int ov13b10_configure_regulators(struct ov13b10 *ov13b10)
{
	unsigned int i;

	for (i = 0; i < OV13B10_NUM_SUPPLIES; i++)
		ov13b10->supplies[i].supply = ov13b10_supply_names[i];

	return devm_regulator_bulk_get(&ov13b10->client->dev,
				       OV13B10_NUM_SUPPLIES,
				       ov13b10->supplies);
}

static int ov13b10_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct ov13b10 *ov13b10;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		DRIVER_VERSION >> 16,
		(DRIVER_VERSION & 0xff00) >> 8,
		DRIVER_VERSION & 0x00ff);

	ov13b10 = devm_kzalloc(dev, sizeof(*ov13b10), GFP_KERNEL);
	if (!ov13b10)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &ov13b10->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &ov13b10->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &ov13b10->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &ov13b10->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	ov13b10->client = client;
	ov13b10->cur_mode = &supported_modes[0];

	ov13b10->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(ov13b10->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	ov13b10->power_gpio = devm_gpiod_get(dev, "power", GPIOD_OUT_LOW);
	if (IS_ERR(ov13b10->power_gpio))
		dev_warn(dev, "Failed to get power-gpios, maybe no use\n");

	ov13b10->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ov13b10->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");

	ov13b10->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
	if (IS_ERR(ov13b10->pwdn_gpio))
		dev_warn(dev, "Failed to get pwdn-gpios\n");

	ret = ov13b10_configure_regulators(ov13b10);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	ov13b10->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(ov13b10->pinctrl)) {
		ov13b10->pins_default =
			pinctrl_lookup_state(ov13b10->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(ov13b10->pins_default))
			dev_err(dev, "could not get default pinstate\n");

		ov13b10->pins_sleep =
			pinctrl_lookup_state(ov13b10->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(ov13b10->pins_sleep))
			dev_err(dev, "could not get sleep pinstate\n");
	}

	mutex_init(&ov13b10->mutex);

	sd = &ov13b10->subdev;
	v4l2_i2c_subdev_init(sd, client, &ov13b10_subdev_ops);
	ret = ov13b10_initialize_controls(ov13b10);
	if (ret)
		goto err_destroy_mutex;

	ret = __ov13b10_power_on(ov13b10);
	if (ret)
		goto err_free_handler;

	ret = ov13b10_check_sensor_id(ov13b10, client);
	if (ret)
		goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &ov13b10_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	ov13b10->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &ov13b10->pad);
	if (ret < 0)
		goto err_power_off;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(ov13b10->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 ov13b10->module_index, facing,
		 OV13B10_NAME, dev_name(sd->dev));
	ret = v4l2_async_register_subdev_sensor(sd);
	if (ret) {
		dev_err(dev, "v4l2 async register subdev failed\n");
		goto err_clean_entity;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	return 0;

err_clean_entity:
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
err_power_off:
	__ov13b10_power_off(ov13b10);
err_free_handler:
	v4l2_ctrl_handler_free(&ov13b10->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&ov13b10->mutex);

	return ret;
}

static void ov13b10_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov13b10 *ov13b10 = to_ov13b10(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&ov13b10->ctrl_handler);
	mutex_destroy(&ov13b10->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__ov13b10_power_off(ov13b10);
	pm_runtime_set_suspended(&client->dev);
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id ov13b10_of_match[] = {
	{ .compatible = "ovti,ov13b10" },
	{},
};
MODULE_DEVICE_TABLE(of, ov13b10_of_match);
#endif

static const struct i2c_device_id ov13b10_match_id[] = {
	{ "ovti,ov13b10", 0 },
	{ },
};

static struct i2c_driver ov13b10_i2c_driver = {
	.driver = {
		.name = OV13B10_NAME,
		.pm = &ov13b10_pm_ops,
		.of_match_table = of_match_ptr(ov13b10_of_match),
	},
	.probe		= &ov13b10_probe,
	.remove		= &ov13b10_remove,
	.id_table	= ov13b10_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&ov13b10_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&ov13b10_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("OmniVision ov13b10 sensor driver");
MODULE_LICENSE("GPL v2");