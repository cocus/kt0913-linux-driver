// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * driver/media/radio/radio-kt0913.c
 *
 * Driver for the KT0913 radio chip from KTMicro.
 * This driver provides a v4l2 interface to the tuner, using the I2C
 * protocol to communicate with the chip.
 * It exposes two bands, one for AM and another for FM. If the "campus
 * band" feature needs to be enabled, set the module parameter to 1.
 * Reference Clock and Audio DAC anti-pop configurations should be
 * carried out via a device tree node. Defaults will be used otherwise.
 * 
 * Audio output should be routed to a speaker or an audio capture
 * device.
 *
 * Based on radio-tea5764 by Fabio Belavenuto <belavenuto@gmail.com>
 *
 *  Copyright (c) 2020 Santiago Hormazabal <santiagohssl@gmail.com>
 *
 * TODO:
 *  use rd and wr support for the regmap instead of volatile regs.
 *  add support for the hardware assisted frequency seek.
 *  export FM SNR and AM/FM AFC deviation values as RO controls.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/pm_runtime.h>
#include <linux/of.h>
#include <linux/math64.h>
#include <linux/regmap.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-event.h>

#define PINFO(format, ...) \
	printk(KERN_INFO KBUILD_MODNAME ": " format "\n", ## __VA_ARGS__)
#define PWARN(format, ...) \
	printk(KERN_WARNING KBUILD_MODNAME ": " format "\n", ## __VA_ARGS__)
#define PERR(format, ...) \
	printk(KERN_ERR KBUILD_MODNAME ": " format "\n", ## __VA_ARGS__)
#define PDEBUG(format, ...) \
	printk(KERN_DEBUG KBUILD_MODNAME ": " format "\n", ## __VA_ARGS__)

/* ************************************************************************* */

/* registers of the kt0913 */
#define KT0913_REG_CHIP_ID      0x01
#define KT0913_REG_SEEK         0x02
#define KT0913_REG_TUNE         0x03
#define KT0913_REG_VOLUME       0x04
#define KT0913_REG_DSPCFGA      0x05
#define KT0913_REG_LOCFGA       0x0A
#define KT0913_REG_LOCFGC       0x0C
#define KT0913_REG_RXCFG        0x0F
#define KT0913_REG_STATUSA      0x12
#define KT0913_REG_STATUSB      0x13
#define KT0913_REG_STATUSC      0x14
#define KT0913_REG_AMSYSCFG     0x16
#define KT0913_REG_AMCHAN       0x17
#define KT0913_REG_AMCALI       0x18
#define KT0913_REG_GPIOCFG      0x1D
#define KT0913_REG_AMDSP        0x22
#define KT0913_REG_AMSTATUSA    0x24
#define KT0913_REG_AMSTATUSB    0x25
#define KT0913_REG_SOFTMUTE     0x2E
#define KT0913_REG_AMCFG        0x33
#define KT0913_REG_AMCFG2       0x34
#define KT0913_REG_AFC          0x3C

/* register symbols masks, values and shift count */
#define KT0913_TUNE_FMTUNE_MASK 0x8000 /* FM Tune enable */
#define KT0913_TUNE_FMTUNE_ON 0x8000 /* FM Tune enabled */
#define KT0913_TUNE_FMTUNE_OFF 0x0000 /* FM Tune disabled */
#define KT0913_TUNE_FMCHAN_MASK 0x0FFF /* frequency in kHz / 50kHz */

#define KT0913_VOLUME_DMUTE_MASK 0x2000
#define KT0913_VOLUME_DMUTE_ON 0x0000
#define KT0913_VOLUME_DMUTE_OFF 0x2000
#define KT0913_VOLUME_DE_MASK 0x0800 /* de-emphasis time constant */
#define KT0913_VOLUME_DE_75US 0x0000 /* 75us */
#define KT0913_VOLUME_DE_50US 0x0800 /* 50us */
#define KT0913_VOLUME_POP_MASK 0x30 /* audio dac anti-pop config */
#define KT0913_VOLUME_POP_SHIFT 4

#define KT0913_DSPCFGA_MONO_MASK 0x8000 /* mono select (0=stereo, 1=mono) */
#define KT0913_DSPCFGA_MONO_ON 0x8000 /* mono */
#define KT0913_DSPCFGA_MONO_OFF 0x0000 /* stereo */

#define KT0913_LOCFG_CAMPUSBAND_EN_MASK 0x0008 /* campus band fm enable */
#define KT0913_LOCFG_CAMPUSBAND_EN_ON 0x0008 /* FM range 64-110MHz */
#define KT0913_LOCFG_CAMPUSBAND_EN_OFF 0x0000 /* FM range 32-110MHz */

#define KT0913_RXCFGA_STDBY_MASK 0x1000 /* standby mode enable */
#define KT0913_RXCFGA_STDBY_ON 0x1000 /* standby mode enabled */
#define KT0913_RXCFGA_STDBY_OFF 0x0000 /* standby mode disabled */
#define KT0913_RXCFGA_VOLUME_MASK 0x001F /* volume control */

#define KT0913_STATUSA_XTAL_OK 0x8000 /* crystal ready indicator */
#define KT0913_STATUSA_STC 0x4000 /* seek/tune complete */

#define KT0913_STATUSA_PLL_LOCK_MASK 0x800 /* system pll ready indicator */
#define KT0913_STATUSA_PLL_LOCK_LOCKED 0x800 /* system pll ready */
#define KT0913_STATUSA_PLL_LOCK_UNLOCKED 0x000 /* not ready */
#define KT0913_STATUSA_LO_LOCK 0x400 /* LO synthesizer ready indicator */
#define KT0913_STATUSA_ST_MASK 0x300 /* stereo indicator (0x300=stereo, otherwise mono) */
#define KT0913_STATUSA_ST_STEREO 0x300 /* stereo */
#define KT0913_STATUSA_FMRSSI_MASK 0xF8 /* FM RSSI (-100dBm + FMRSSI*3dBm) */
#define KT0913_STATUSA_FMRSSI_SHIFT 3

#define KT0913_STATUSC_PWSTATUS 0x8000 /* power status indicator */
#define KT0913_STATUSC_CHIPRDY 0x2000 /* chip ready indicator */
#define KT0913_STATUSC_FMSNR 0x1FC0 /* FM SNR (unknown units) */

#define KT0913_AMCHAN_AMTUNE_MASK 0x8000 /* AM tune enable */
#define KT0913_AMCHAN_AMTUNE_ON 0x8000 /* AM tune enabled */
#define KT0913_AMCHAN_AMTUNE_OFF 0x0000 /* AM tune disabled */
#define KT0913_AMCHAN_AMCHAN_MASK 0x7FF /* am channel in kHz */

#define KT0913_AMSYSCFG_AM_FM_MASK 0x8000 /* am/fm mode control */
#define KT0913_AMSYSCFG_AM_FM_AM 0x8000 /* am mode */
#define KT0913_AMSYSCFG_AM_FM_FM 0x0000 /* fm mode (default) */
#define KT0913_AMSYSCFG_REFCLK_MASK 0x0F00 /* reference clock selection */
#define KT0913_AMSYSCFG_REFCLK_SHIFT 8
#define KT0913_AMSYSCFG_AU_GAIN_MASK 0x00C0 /* audio gain selection */
#define KT0913_AMSYSCFG_AU_GAIN_6DB 0x0040 /* 6dB audio gain */
#define KT0913_AMSYSCFG_AU_GAIN_3DB 0x0000 /* 3dB audio gain (default) */
#define KT0913_AMSYSCFG_AU_GAIN_0DB 0x00C0 /* 0dB audio gain */
#define KT0913_AMSYSCFG_AU_GAIN_MIN_3DB 0x0080 /* -3dB audio gain */

#define KT0913_AMSTATUSA_AMRSSI_MASK 0x1F00 /* am channel rssi */
#define KT0913_AMSTATUSA_AMRSSI_SHIFT 8

/* constants */
#define KT0913_CHIP_ID  0x544B /* ASCII of 'KT' */

#define V4L2_KHZ_FREQ_MUL 16U /* v4l2 uses 16x the kHz value as their freq */
#define KT0913_FMCHAN_MUL 50U /* kt0913 uses freqs with a 50kHz multiplier */
#define KT0913_FM_RANGE_LOW_NO_CAMPUS 64000U /* 64MHz lower bound for FM */
#define KT0913_FM_RANGE_LOW_CAMPUS 32000U /* 32MHz lower bound for campus FM */
#define KT0913_FM_RANGE_HIGH 110000U /* 110MHz upper bound for FM */
#define KT0913_AM_RANGE_LOW  500U /* 500kHz lower bound for AM */
#define KT0913_AM_RANGE_HIGH 1710U /* 1710kHz upper bound for AM */

/* ************************************************************************* */

/* v4l2 device number to use. -1 will assign the next free one */
static int kt0913_v4l2_radio_nr = -1;
/* use the extended range of FM down to 32MHz. disabled by default */
static int kt0913_use_campus_band = 0;

/* ************************************************************************* */

/* kt0913 status struct */
struct kt0913_device {
	struct v4l2_device v4l2_dev;		/* main v4l2 struct */
	struct i2c_client *client;			/* I2C client */
	struct video_device vdev;			/* vide_device struct */
	struct v4l2_ctrl_handler ctrl_handler; /* ctrl_handler struct */

	/* V4L2 Controls */
	struct v4l2_ctrl *ctrl_pll_lock;    /* PLL lock */
	struct v4l2_ctrl *ctrl_volume;      /* Overall volume */
	struct v4l2_ctrl *ctrl_au_gain;     /* Audio Gain */
	struct v4l2_ctrl *ctrl_mute;        /* Master mute */
	struct v4l2_ctrl *ctrl_deemphasis;  /* Deemphasis */

	/* current operation band (fm, fm_campus, am) */
	unsigned int band;

	/* audio dac anti-pop setting:
	 *  0 -> 100uF (default)
	 *  1 -> 60uF
	 *  2 -> 20uF
	 *  3 -> 10uF
	 */
	unsigned int audio_anti_pop;

	/*
	 * reference clock selection:
	 *  0 -> 32.768kHz (default)
	 *  1 -> 6.5MHz
	 *  2 -> 7.6MHz
	 *  3 -> 12MHz
	 *  4 -> 13MHz
	 *  5 -> 15.2MHz
	 *  6 -> 19.2MHz
	 *  7 -> 24MHz
	 *  8 -> 26MHz
	 *  9 -> 38kHz
	 */
	unsigned int refclock_val;

	/* Regmap */
	struct regmap* regmap;

	/* For core assisted locking */
	struct mutex mutex;
};

/* ************************************************************************* */

/* Regmap settings */
static const struct regmap_range kt0913_regmap_all_registers_range[] = {
	regmap_reg_range(0x01, 0x05),
	regmap_reg_range(0x0A, 0x0A),
	regmap_reg_range(0x0C, 0x0C),
	regmap_reg_range(0x0F, 0x0F),
	regmap_reg_range(0x12, 0x14),
	regmap_reg_range(0x16, 0x18),
	regmap_reg_range(0x1D, 0x1D),
	regmap_reg_range(0x22, 0x22),
	regmap_reg_range(0x24, 0x25),
	regmap_reg_range(0x2E, 0x2F),
	regmap_reg_range(0x30, 0x34),
	regmap_reg_range(0x3A, 0x3A),
	regmap_reg_range(0x3C, 0x3C),
};

static const struct regmap_access_table kt0913_all_registers_access_table = {
	.yes_ranges = kt0913_regmap_all_registers_range,
	.n_yes_ranges = ARRAY_SIZE(kt0913_regmap_all_registers_range),
};

static const struct reg_sequence kt0913_init_regs_to_defaults[] = {
	/* Standby disabled, volume 0dB */
	{ KT0913_REG_RXCFG, 0x881F },
	/* FM Channel spacing = 50kHz, Right & Left unmuted */
	{ KT0913_REG_SEEK, 0x000B },
	/* Stereo, High Stereo/Mono blend level, blend disabled */
	{ KT0913_REG_DSPCFGA, 0x1000 },
	/* FM AFC Enabled */
	{ KT0913_REG_LOCFGA, 0x0100 },
	/* Campus band disabled by default */
	{ KT0913_REG_LOCFGC, 0x0024 },
	/* 
	 * FM mode, internal defined bands, clock from XT, 32.768kHz
	 * 3dB audio gain, AM AFC Enabled
	 */
	{ KT0913_REG_AMSYSCFG, 0x0002 },
	/* Default AM freq = 504kHz */
	{ KT0913_REG_AMCHAN, 0x01F8},
	/* VOL and CH GPIOs set to HiZ */
	{ KT0913_REG_GPIOCFG, 0x0000 },
	/* AM Channel bandwidth = 6kHz, non-differential output */
	{ KT0913_REG_AMDSP, 0xAFC4 },
	/* 
	 * softmute is disabled on AM and FM, but set the defaults:
	 * strong softmute attn., slow softmute attack/recover, 
	 * lowest AM softumte start level, almost the minimum
	 * softmute target volume, RSSI mode for softmute, lowest 
	 * FM softmute start level
	 */
	{ KT0913_REG_SOFTMUTE, 0x0010 },
	/* 1kHz for AM channel space, working mode A for the keys */
	{ KT0913_REG_AMCFG, 0x1401 },
	/* TIME1 = shortest, TIME2 = fastest */
	{ KT0913_REG_AMCFG, 0x4050 },
	/* set 86MHz as the default frequency, and tune it */
	{ KT0913_REG_TUNE, 0x86B8 },
	/*
	 * FM&AM Softmute disabled, Mute disabled, 75us deemp.,
	 * no bass boost, 100uF anti pop cap
	 */
	{ KT0913_REG_VOLUME, 0xE080 },
};

static const struct regmap_config kt0913_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,
	.max_register = KT0913_REG_AFC,
	.volatile_table = &kt0913_all_registers_access_table,
	.cache_type = REGCACHE_RBTREE,
	.val_format_endian = REGMAP_ENDIAN_BIG,
};

/* ************************************************************************* */

/* bands where the kt0913 operates */
enum { BAND_FM, BAND_FM_CAMUS, BAND_AM };

static const struct v4l2_frequency_band kt0913_bands[] = {
	{
		/* BAND_FM */
		.type = V4L2_TUNER_RADIO,
		.index = 0, /* index provided to v4l2 */
		.capability = V4L2_TUNER_CAP_LOW | V4L2_TUNER_CAP_STEREO |
				  V4L2_TUNER_CAP_FREQ_BANDS,
		.rangelow = KT0913_FM_RANGE_LOW_NO_CAMPUS * V4L2_KHZ_FREQ_MUL,
		.rangehigh = KT0913_FM_RANGE_HIGH * V4L2_KHZ_FREQ_MUL,
		.modulation = V4L2_BAND_MODULATION_FM,
	},
	{
		/* BAND_FM_CAMUS */
		.type = V4L2_TUNER_RADIO,
		.index = 0, /* index provided to v4l2 */
		.capability = V4L2_TUNER_CAP_LOW | V4L2_TUNER_CAP_STEREO |
				  V4L2_TUNER_CAP_FREQ_BANDS,
		.rangelow = KT0913_FM_RANGE_LOW_CAMPUS * V4L2_KHZ_FREQ_MUL,
		.rangehigh = KT0913_FM_RANGE_HIGH * V4L2_KHZ_FREQ_MUL,
		.modulation = V4L2_BAND_MODULATION_FM,
	},
	{
		/* BAND_AM */
		.type = V4L2_TUNER_RADIO,
		.index = 1, /* index provided to v4l2 */
		.capability = V4L2_TUNER_CAP_LOW | V4L2_TUNER_CAP_FREQ_BANDS,
		.rangelow = KT0913_AM_RANGE_LOW * V4L2_KHZ_FREQ_MUL,
		.rangehigh = KT0913_AM_RANGE_HIGH * V4L2_KHZ_FREQ_MUL,
		.modulation = V4L2_BAND_MODULATION_AM,
	},
};

/* ************************************************************************* */

static inline struct kt0913_device *v4l2_device_to_device(
	struct v4l2_device *v4l2_dev)
{
	return container_of(v4l2_dev,
		struct kt0913_device, v4l2_dev);
}

static inline struct kt0913_device *v4l2_ctrl_to_device(
	struct v4l2_ctrl *ctrl_handler)
{
	return container_of(ctrl_handler->handler,
		struct kt0913_device, ctrl_handler);
}

/* ************************************************************************* */

static inline u32 khz_to_v4l2_freq(unsigned int freq)
{
	return freq * V4L2_KHZ_FREQ_MUL;
}

static inline unsigned int v4l2_freq_to_khz(u32 v4l2_freq)
{
	return v4l2_freq / V4L2_KHZ_FREQ_MUL;
}

/* ************************************************************************* */

static int __kt0913_get_fm_frequency(struct kt0913_device *radio,
	unsigned int *frequency)
{
	unsigned int tune_reg;
	int ret = regmap_read(radio->regmap, KT0913_REG_TUNE, &tune_reg);
	if (ret)
		return ret;

	*frequency = (tune_reg & KT0913_TUNE_FMCHAN_MASK) * KT0913_FMCHAN_MUL;

	return 0;
}

static int __kt0913_set_fm_frequency(struct kt0913_device *radio,
	unsigned int frequency)
{
	return regmap_write(radio->regmap, KT0913_REG_TUNE, 
		KT0913_TUNE_FMTUNE_ON | (frequency / KT0913_FMCHAN_MUL));
}

/* ************************************************************************* */

static int __kt0913_set_mute(struct kt0913_device *radio, int on)
{
	return regmap_update_bits(radio->regmap,
		KT0913_REG_VOLUME, KT0913_VOLUME_DMUTE_MASK,
		on ? KT0913_VOLUME_DMUTE_ON : KT0913_VOLUME_DMUTE_OFF);
}

/* ************************************************************************* */

static int __kt0913_set_deemphasis(struct kt0913_device *radio, s32 deemp)
{
	switch (deemp) {
	case V4L2_DEEMPHASIS_75_uS:
		return regmap_update_bits(radio->regmap,
			KT0913_REG_VOLUME, KT0913_VOLUME_DE_MASK,
			KT0913_VOLUME_DE_75US);
	case V4L2_DEEMPHASIS_50_uS:
		return regmap_update_bits(radio->regmap,
			KT0913_REG_VOLUME, KT0913_VOLUME_DE_MASK,
			KT0913_VOLUME_DE_50US);
	default:
		return -EINVAL;
	}
}

/* ************************************************************************* */

static int __kt0913_set_volume(struct kt0913_device *radio, s32 volume)
{
	/* map [-60, 0] to [1, 31] which is what the kt0913 expects */
	volume = (volume / 2) + 31;
	return regmap_update_bits(radio->regmap,
		KT0913_REG_RXCFG, KT0913_RXCFGA_VOLUME_MASK,
		volume);
}

/* ************************************************************************* */

static int __kt0913_set_standby(struct kt0913_device *radio, int standby)
{
	PDEBUG("__kt0913_set_standby -> %d", standby);

	return regmap_update_bits(radio->regmap,
		KT0913_REG_RXCFG, KT0913_RXCFGA_STDBY_MASK,
		standby ? KT0913_RXCFGA_STDBY_ON : KT0913_RXCFGA_STDBY_OFF);
}

/* ************************************************************************* */

static int __kt0913_get_pll_status(struct kt0913_device *radio, int *locked)
{
	unsigned int statusa_reg;
	int ret = regmap_read(radio->regmap, KT0913_REG_STATUSA, &statusa_reg);
	if (ret)
		return ret;

	*locked = (statusa_reg & KT0913_STATUSA_PLL_LOCK_MASK) ==
		KT0913_STATUSA_PLL_LOCK_LOCKED ? 1 : 0;

	return 0;
}

/* ************************************************************************* */

static int __kt0913_get_rx_stereo_or_mono(struct kt0913_device *radio,
	int *stereo)
{
	unsigned int statusa_reg;
	int ret = regmap_read(radio->regmap, KT0913_REG_STATUSA, &statusa_reg);
	if (ret)
		return ret;

	*stereo = (statusa_reg & KT0913_STATUSA_ST_MASK) ==
		KT0913_STATUSA_ST_STEREO ? 1 : 0;

	return 0;
}

/* ************************************************************************* */

static int __kt0913_get_fm_rssi(struct kt0913_device *radio, s32 *rssi)
{
	unsigned int statusa_reg;
	int ret = regmap_read(radio->regmap, KT0913_REG_STATUSA, &statusa_reg);
	if (ret)
		return ret;

	/* RSSI(dBm) = -100 + FMRSSI<4:0> * 3dBm */
	/* even tho we can get the value in dBm, we want a % */
	*rssi = (statusa_reg & KT0913_STATUSA_FMRSSI_MASK) >>
		KT0913_STATUSA_FMRSSI_SHIFT;
	/* map range 0-31 to 0-65535 */
	*rssi *= 65535;
	*rssi /= KT0913_STATUSA_FMRSSI_MASK >> KT0913_STATUSA_FMRSSI_SHIFT;

	return 0;
}

/* ************************************************************************* */

static int __kt0913_get_cfg_stereo_enabled(struct kt0913_device *radio,
	int *stereo)
{
	unsigned int dspcfga_reg;
	int ret = regmap_read(radio->regmap, KT0913_REG_DSPCFGA, &dspcfga_reg);
	if (ret)
		return ret;

	*stereo = (dspcfga_reg & KT0913_DSPCFGA_MONO_MASK) ==
		KT0913_DSPCFGA_MONO_OFF ? 1 : 0;

	return ret;
}

static int __kt0913_set_cfg_stereo_enabled(struct kt0913_device *radio,
	int stereo)
{
	return regmap_update_bits(radio->regmap,
		KT0913_REG_DSPCFGA, KT0913_DSPCFGA_MONO_MASK,
		stereo ? KT0913_DSPCFGA_MONO_OFF : KT0913_DSPCFGA_MONO_ON);
}

/* ************************************************************************* */

static int __kt0913_set_au_gain(struct kt0913_device *radio, s32 gain)
{
	switch (gain) {
	case 6:
		return regmap_update_bits(radio->regmap,
			KT0913_REG_AMSYSCFG, KT0913_AMSYSCFG_AU_GAIN_MASK,
			KT0913_AMSYSCFG_AU_GAIN_6DB);
	case 3:
		return regmap_update_bits(radio->regmap,
			KT0913_REG_AMSYSCFG, KT0913_AMSYSCFG_AU_GAIN_MASK,
			KT0913_AMSYSCFG_AU_GAIN_3DB);
	case 0:
		return regmap_update_bits(radio->regmap,
			KT0913_REG_AMSYSCFG, KT0913_AMSYSCFG_AU_GAIN_MASK,
			KT0913_AMSYSCFG_AU_GAIN_0DB);
	case -3:
		return regmap_update_bits(radio->regmap,
			KT0913_REG_AMSYSCFG, KT0913_AMSYSCFG_AU_GAIN_MASK,
			KT0913_AMSYSCFG_AU_GAIN_MIN_3DB);
	default:
		return -EINVAL;
	}
}

/* ************************************************************************* */

static int __kt0913_set_am_fm_band(struct kt0913_device *radio,
	unsigned int band)
{
	return regmap_update_bits(radio->regmap,
		KT0913_REG_AMSYSCFG, KT0913_AMSYSCFG_AM_FM_MASK,
		band == BAND_AM ?
		KT0913_AMSYSCFG_AM_FM_AM : KT0913_AMSYSCFG_AM_FM_FM);
}

/* ************************************************************************* */

static int __kt0913_get_am_frequency(struct kt0913_device *radio,
	unsigned int *frequency)
{
	unsigned int amchan_reg;
	int ret = regmap_read(radio->regmap, KT0913_REG_AMCHAN, &amchan_reg);
	if (ret)
		return ret;

	*frequency = (amchan_reg & KT0913_AMCHAN_AMCHAN_MASK);

	return 0;
}

static int __kt0913_set_am_frequency(struct kt0913_device *radio,
	unsigned int frequency)
{
	return regmap_write(radio->regmap, KT0913_REG_AMCHAN,
		KT0913_AMCHAN_AMTUNE_ON | frequency);
}

/* ************************************************************************* */

static int __kt0913_get_am_rssi(struct kt0913_device *radio, s32 *rssi)
{
	unsigned int amstatusa_reg;
	int ret = regmap_read(radio->regmap, KT0913_REG_AMSTATUSA, &amstatusa_reg);
	if (ret)
		return ret;

	/* AMRSSI(dBm) = -90 + AMRSSI<4:0> * 3dBm */
	/* even tho we can get the value in dBm, we want a % */
	*rssi = (amstatusa_reg & KT0913_AMSTATUSA_AMRSSI_MASK) >>
		KT0913_AMSTATUSA_AMRSSI_SHIFT;
	/* map range 0-31 to 0-65535 */
	*rssi *= 65535;
	*rssi /= KT0913_AMSTATUSA_AMRSSI_MASK >> KT0913_AMSTATUSA_AMRSSI_SHIFT;

	return 0;
}

/* ************************************************************************* */

static int __kt0913_init(struct kt0913_device *radio)
{
	int ret = 0;

	/* write the defaults */
	ret = regmap_multi_reg_write(radio->regmap,
		kt0913_init_regs_to_defaults,
		ARRAY_SIZE(kt0913_init_regs_to_defaults));
	if (ret) {
		PERR("regmap_multi_reg_write() failed! %d", ret);
		return ret;
	}

	/* set the audio dac anti-pop config */
	ret = regmap_update_bits(radio->regmap,
		KT0913_REG_VOLUME, KT0913_VOLUME_POP_MASK,
		radio->audio_anti_pop << KT0913_VOLUME_POP_SHIFT);
	if (ret) {
		PERR("regmap_update_bits() failed for anti-pop cfg! %d", ret);
		return ret;
	}

	/* set the reference clock config */
	ret = regmap_update_bits(radio->regmap,
		KT0913_REG_AMSYSCFG, KT0913_AMSYSCFG_REFCLK_MASK,
		radio->refclock_val << KT0913_AMSYSCFG_REFCLK_SHIFT);
	if (ret) {
		PERR("regmap_update_bits() failed for refclk cfg! %d", ret);
		return ret;
	}

	if (kt0913_use_campus_band) {
		PINFO("campus band is enabled!");
		/* set the campus band bit */
		ret = regmap_update_bits(radio->regmap,
			KT0913_REG_LOCFGC, KT0913_LOCFG_CAMPUSBAND_EN_MASK,
			KT0913_LOCFG_CAMPUSBAND_EN_ON);
		if (ret) {
			PERR("regmap_update_bits() failed for campus band! %d",
				ret);
			return ret;
		}
	}

	return __kt0913_set_mute(radio, true);
}

/* ************************************************************************* */

static int kt0913_ioctl_vidioc_g_frequency(struct file *file, void *priv,
	struct v4l2_frequency *f)
{
	struct kt0913_device *radio = video_drvdata(file);
	int ret; 

	if (f->tuner != 0)
		return -EINVAL;
	
	f->type = V4L2_TUNER_RADIO;

	if (radio->band == BAND_AM)
		ret = __kt0913_get_am_frequency(radio, &f->frequency);
	else
		ret = __kt0913_get_fm_frequency(radio, &f->frequency);

	if (ret)
		return ret;

	/* convert kHz freq into v4l2 freq */
	f->frequency = khz_to_v4l2_freq(f->frequency);

	return 0;
}

static int kt0913_ioctl_vidioc_s_frequency(struct file *file, void *priv,
	const struct v4l2_frequency *f)
{
	struct kt0913_device *radio = video_drvdata(file);
	unsigned freq = f->frequency;
	unsigned int new_band = BAND_FM;
	int ret;

	if (f->tuner != 0 || f->type != V4L2_TUNER_RADIO)
		return -EINVAL;

	if (freq == 0)
		return -EINVAL;

	/* check if the requested frequency is contained on the AM band */
	if ((freq >= kt0913_bands[BAND_AM].rangelow) &&
		(freq <=  kt0913_bands[BAND_AM].rangehigh)) {
		new_band = BAND_AM;
	}
	else {
		/* check if the requested frequency is contained on the FM band */
		if ((freq >= kt0913_bands[BAND_FM].rangelow) &&
			(freq <= kt0913_bands[BAND_FM].rangehigh)) {
			new_band = BAND_FM;
		}
		/* check if the requested frequency is contained on the campus 
		   FM band only if that feature was enabled */
		else if (kt0913_use_campus_band && 
				(freq >= kt0913_bands[BAND_FM_CAMUS].rangelow) &&
				(freq <= kt0913_bands[BAND_FM_CAMUS].rangehigh)) {
			new_band = BAND_FM_CAMUS;
		}
		else {
			PWARN("frequency out of allowed RF bands (%u kHz)",
				v4l2_freq_to_khz(freq));
			return -EINVAL;
		}
	}

	/* is the requested band different than the one currently set? */
	if (radio->band != new_band) {
		ret = __kt0913_set_am_fm_band(radio, new_band);
		if (ret)
			return ret;
		radio->band = new_band;
	}

	/* convert v4l2 freq to kHz */
	freq = v4l2_freq_to_khz(freq);

	if (radio->band == BAND_AM)
		return __kt0913_set_am_frequency(radio, freq);
	else
		return __kt0913_set_fm_frequency(radio, freq);
}

static int kt0913_ioctl_vidioc_enum_freq_bands(struct file *file, void *priv,
	struct v4l2_frequency_band *band)
{
	if (band->tuner != 0)
		return -EINVAL;

	switch (band->index) {
	case 0:
		if (kt0913_use_campus_band)
			*band = kt0913_bands[BAND_FM_CAMUS];
		else
			*band = kt0913_bands[BAND_FM];
		return 0;
	case 1:
		*band = kt0913_bands[BAND_AM];
		return 0;
	default:
		return -EINVAL;
	}
}

/* ************************************************************************* */

/* V4L2 vidioc */
static int kt0913_ioctl_vidioc_querycap(struct file *file, void  *priv,
	struct v4l2_capability *v)
{
	struct kt0913_device *radio = video_drvdata(file);
	struct video_device* dev;

	if (!radio)
		return -ENODEV;

	dev = &radio->vdev;

	if (!dev)
		return -ENODEV;

	if (dev->dev.driver)
		strscpy(v->driver, dev->dev.driver->name, sizeof(v->driver));

	strscpy(v->card, dev->name, sizeof(v->card));
	snprintf(v->bus_info, sizeof(v->bus_info),
		 "I2C:%s", dev_name(&dev->dev));
	return 0;
}

static int kt0913_ioctl_vidioc_g_tuner(struct file *file, void *priv,
	struct v4l2_tuner *v)
{
	struct kt0913_device *radio = video_drvdata(file);
	int ret;
	int stereo_enabled;
	int is_stereo;

	if (v->index > 0)
		return -EINVAL;

	strscpy(v->name, "FM/AM", sizeof(v->name));
	v->type = V4L2_TUNER_RADIO;
	
	v->capability = V4L2_TUNER_CAP_LOW | V4L2_TUNER_CAP_STEREO |
		V4L2_TUNER_CAP_FREQ_BANDS;

	v->rangelow = kt0913_bands[BAND_AM].rangelow;
	v->rangehigh = kt0913_bands[BAND_FM].rangehigh;

	if (radio->band == BAND_AM) {
		v->rxsubchans = V4L2_TUNER_SUB_MONO;
		v->audmode = V4L2_TUNER_MODE_MONO;

		ret = __kt0913_get_am_rssi(radio, &v->signal);
		if (ret)
			return ret;
	}
	else {
		ret = __kt0913_get_cfg_stereo_enabled(radio, &stereo_enabled);
		if (ret)
			return ret;

		v->rxsubchans = stereo_enabled ? 
			V4L2_TUNER_SUB_STEREO : V4L2_TUNER_SUB_MONO;

		ret = __kt0913_get_rx_stereo_or_mono(radio, &is_stereo);
		if (ret)
			return ret;

		v->audmode = is_stereo ?
			V4L2_TUNER_MODE_STEREO : V4L2_TUNER_MODE_MONO;

		ret = __kt0913_get_fm_rssi(radio, &v->signal);
		if (ret)
			return ret;
	}

	/* AFC is enabled and active by default */
	v->afc = 1;

	return 0;
}

static int kt0913_ioctl_vidioc_s_tuner(struct file *file, void *priv,
	const struct v4l2_tuner *v)
{
	struct kt0913_device *radio = video_drvdata(file);

	if (v->index > 0)
		return -EINVAL;

	PDEBUG("kt0913_ioctl_vidioc_s_tuner(v->audmode=%d)", v->audmode);
	
	/* only mono and stereo are supported */
	if (v->audmode != V4L2_TUNER_MODE_MONO &&
		v->audmode != V4L2_TUNER_MODE_STEREO)
		return -EINVAL;

	/* AM is mono only, so don't try to set it to stereo */
	if (radio->band == BAND_AM && v->audmode != V4L2_TUNER_MODE_MONO)
		return -EINVAL;

	/* set to stereo if specified, otherwise set to mono */
	return __kt0913_set_cfg_stereo_enabled(radio,
		v->audmode == V4L2_TUNER_MODE_STEREO);
}

static int kt0913_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct kt0913_device *radio = v4l2_ctrl_to_device(ctrl);

	switch (ctrl->id) {
	case V4L2_CID_AUDIO_MUTE:
		return __kt0913_set_mute(radio, ctrl->val);
	case V4L2_CID_AUDIO_VOLUME:
		return __kt0913_set_volume(radio, ctrl->val);
	case V4L2_CID_GAIN:
		return __kt0913_set_au_gain(radio, ctrl->val);
	case V4L2_CID_TUNE_DEEMPHASIS:
		return __kt0913_set_deemphasis(radio, ctrl->val);
	default:
		return -EINVAL;
	}
}

static int kt0913_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct kt0913_device *radio = v4l2_ctrl_to_device(ctrl);

	switch (ctrl->id) {
	case V4L2_CID_RF_TUNER_PLL_LOCK:
		return __kt0913_get_pll_status(radio, &ctrl->val);
	default:
		return -EINVAL;
	}
}

static const struct v4l2_ctrl_ops kt0913_ctrl_ops = {
	.s_ctrl = kt0913_s_ctrl,
	.g_volatile_ctrl = kt0913_g_volatile_ctrl,
};

/* ************************************************************************* */

/* File system interface (use the ancillary fops for v4l2) */
static const struct v4l2_file_operations kt0913_radio_fops = {
	.owner		= THIS_MODULE,
	.open		= v4l2_fh_open,
	.release	= v4l2_fh_release,
	.poll		= v4l2_ctrl_poll,
	.unlocked_ioctl	= video_ioctl2,
};

/* ioctl ops */
static const struct v4l2_ioctl_ops kt0913_ioctl_ops = {
	.vidioc_querycap    = kt0913_ioctl_vidioc_querycap,
	.vidioc_g_tuner     = kt0913_ioctl_vidioc_g_tuner,
	.vidioc_s_tuner     = kt0913_ioctl_vidioc_s_tuner,
	.vidioc_g_frequency = kt0913_ioctl_vidioc_g_frequency,
	.vidioc_s_frequency = kt0913_ioctl_vidioc_s_frequency,
	.vidioc_enum_freq_bands = kt0913_ioctl_vidioc_enum_freq_bands,
	/* use ancillary functions for these: */
	.vidioc_log_status  = v4l2_ctrl_log_status,
	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

/* V4L2 RADIO device structure */
static struct video_device kt0913_radio_template = {
	.name = "KT0913 FM/AM Radio",
	.fops = &kt0913_radio_fops,
	.ioctl_ops = &kt0913_ioctl_ops,
	.release = video_device_release_empty,
};

/* ************************************************************************* */

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id kt0913_of_match[] = {
	{ .compatible = "ktm,kt0913" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, kt0913_of_match);
#endif /* IS_ENABLED(CONFIG_OF) */

static void __kt0913_parse_dt(struct kt0913_device *radio)
{
	const void *ptr_anti_pop = of_get_property(radio->client->dev.of_node,
		"ktm,anti-pop", NULL);
	const void* ptr_refclk = of_get_property(radio->client->dev.of_node,
		"ktm,refclk", NULL);

	if (ptr_anti_pop) {
		radio->audio_anti_pop =
			clamp(be32_to_cpup(ptr_anti_pop), 0U, 3U);
	}
	else {
		radio->audio_anti_pop = 0;
		PWARN("No ktm,anti-pop on dt node, using default");
	}

	if (ptr_refclk) {
		radio->refclock_val =
			clamp(be32_to_cpup(ptr_refclk), 0U, 9U);
	}
	else {
		radio->refclock_val = 0;
		PWARN("No ktm,refclk on dt node, using default");
	}
}

/* ************************************************************************* */

static int kt0913_probe(struct i2c_client *client, 
	const struct i2c_device_id *id)
{
	struct kt0913_device *radio;
	struct v4l2_device *v4l2_dev;
	struct v4l2_ctrl_handler *hdl;
	struct regmap *regmap;
	int ret;

	PDEBUG("probe");

	/* this driver uses word R/W i2c operations, check if it's supported */
	if (!i2c_check_functionality(client->adapter,
	I2C_FUNC_SMBUS_READ_WORD_DATA | I2C_FUNC_SMBUS_WRITE_WORD_DATA)) {
		PERR("I2C adapter doesn't support word operations");
		return -EIO;
	}

	/* check if the device exist on the bus before initializing it */
	ret = i2c_smbus_read_word_data(client, KT0913_REG_CHIP_ID);
	if (ret < 0) {
		PERR("Error reading the CHIP ID of the kt0913 (%d)", ret);
		return ret;
	}

	/* check if the CHIP ID register value matches the expected value */
	if (ret != KT0913_CHIP_ID) {
		PERR("Invalid CHIP ID: 0x%x, expected 0x%x", ret, KT0913_CHIP_ID);
		return -ENODEV;
	}

	v4l_info(client, "kt0913 found @ 0x%x (%s)\n", 
		client->addr, client->adapter->name);

	/* alloc context for the kt0913 radio struct */
	radio = devm_kzalloc(&client->dev, sizeof(*radio), GFP_KERNEL);
	if (!radio)
		return -ENOMEM;

	v4l2_dev = &radio->v4l2_dev;
	ret = v4l2_device_register(&client->dev, v4l2_dev);
	if (ret < 0) {
		v4l2_err(v4l2_dev, "could not register v4l2_dev\n");
		goto errfr;
	}

	mutex_init(&radio->mutex);

	/* register the control handler from the context struct */
	hdl = &radio->ctrl_handler;
	v4l2_ctrl_handler_init(hdl, 5);

	/* add the control: Mute */
	radio->ctrl_mute = v4l2_ctrl_new_std(hdl, &kt0913_ctrl_ops,
		V4L2_CID_AUDIO_MUTE, 0, 1, 1, 0);
	if (hdl->error) {
		ret = hdl->error;
		v4l2_err(v4l2_dev, "Could not register control: mute\n");
		goto errunreg;
	}

	/* add the control: Volume */
	radio->ctrl_volume = v4l2_ctrl_new_std(hdl, &kt0913_ctrl_ops,
		V4L2_CID_AUDIO_VOLUME, -60, 0, 2, 0);
	if (hdl->error) {
		ret = hdl->error;
		v4l2_err(v4l2_dev, "Could not register control: Volume\n");
		goto errunreg;
	}

	/* add the control: audio gain */
	radio->ctrl_au_gain = v4l2_ctrl_new_std(hdl, &kt0913_ctrl_ops,
		V4L2_CID_GAIN, -3, 6, 3, 3);
	if (hdl->error) {
		ret = hdl->error;
		v4l2_err(v4l2_dev, "Could not register control: audio gain\n");
		goto errunreg;
	}
	radio->ctrl_au_gain->flags |= V4L2_CTRL_FLAG_SLIDER;

	/* add the control: PLL Lock */
	radio->ctrl_pll_lock = v4l2_ctrl_new_std(hdl, &kt0913_ctrl_ops,
		V4L2_CID_RF_TUNER_PLL_LOCK, 0, 1, 1, 0);
	if (hdl->error) {
		ret = hdl->error;
		v4l2_err(v4l2_dev, "Could not register control: pll lock\n");
		goto errunreg;
	}
	radio->ctrl_pll_lock->flags |= (V4L2_CTRL_FLAG_VOLATILE |
		V4L2_CTRL_FLAG_READ_ONLY);

	/* add the control: deemphasis */
	radio->ctrl_deemphasis = v4l2_ctrl_new_std_menu(hdl, &kt0913_ctrl_ops,
		V4L2_CID_TUNE_DEEMPHASIS, V4L2_DEEMPHASIS_75_uS,
		0, V4L2_DEEMPHASIS_75_uS);
	if (hdl->error) {
		ret = hdl->error;
		v4l2_err(v4l2_dev, "Could not register control: deemphasis\n");
		goto errunreg;
	}
	/* the control handler is ready to be used */
	v4l2_dev->ctrl_handler = hdl;

	radio->vdev = kt0913_radio_template;
	radio->vdev.lock = &radio->mutex;
	radio->vdev.v4l2_dev = v4l2_dev;
	radio->vdev.device_caps = V4L2_CAP_TUNER | V4L2_CAP_RADIO;
	video_set_drvdata(&radio->vdev, radio);

	radio->client = client;
	i2c_set_clientdata(client, radio);

	/* init the regmap of the kt0913 */
	regmap = devm_regmap_init_i2c(client, &kt0913_regmap_config);
	if (IS_ERR(regmap)) {
		ret = PTR_ERR(regmap);
		PERR("devm_regmap_init_i2c() failed! %d", ret);
		goto errunreg;
	}
	radio->regmap = regmap;

	__kt0913_parse_dt(radio);

	/* init the kt0913 into a known state */
	ret = __kt0913_init(radio);
	if (ret) {
		PERR("__kt0913_init() failed! %d", ret);
		goto errunreg;
	}

	pm_runtime_get_noresume(&client->dev);
	pm_runtime_set_active(&client->dev);
	pm_runtime_enable(&client->dev);
	pm_runtime_dont_use_autosuspend(&client->dev);

	ret = video_register_device(&radio->vdev,
		VFL_TYPE_RADIO, kt0913_v4l2_radio_nr);
	if (ret < 0) {
		PERR("Could not register video device!");
		goto error_pm_disable;
	}

	PINFO("registered.");
	return 0;
error_pm_disable:
	pm_runtime_disable(&client->dev);
	pm_runtime_set_suspended(&client->dev);
errunreg:
	v4l2_ctrl_handler_free(hdl);
	v4l2_device_unregister(v4l2_dev);
errfr:
	__kt0913_set_standby(radio, true);
	kfree(radio);
	return ret;
}

static int kt0913_remove(struct i2c_client *client)
{
	struct kt0913_device *radio = i2c_get_clientdata(client);

	PDEBUG("remove");
	if (!radio)
		return -EINVAL;

	__kt0913_set_standby(radio, true);

	pm_runtime_get_sync(&client->dev);
	pm_runtime_disable(&client->dev);
	pm_runtime_set_suspended(&client->dev);
	pm_runtime_put_noidle(&client->dev);

	video_unregister_device(&radio->vdev);
	v4l2_ctrl_handler_free(&radio->ctrl_handler);
	v4l2_device_unregister(&radio->v4l2_dev);
	
	PINFO("removed.");
	return 0;
}

/* ************************************************************************* */

#ifdef CONFIG_PM
static int kt0913_i2c_pm_runtime_suspend(struct device *dev)
{
	struct kt0913_device *radio = i2c_get_clientdata(to_i2c_client(dev));
	PDEBUG("kt0913_i2c_pm_runtime_suspend");
	if (!radio) {
		return 0;
	}
	return __kt0913_set_standby(radio, true);
}

static int kt0913_i2c_pm_runtime_resume(struct device *dev)
{
	struct kt0913_device *radio = i2c_get_clientdata(to_i2c_client(dev));
	PDEBUG("kt0913_i2c_pm_runtime_resume");
	if (!radio) {
		return 0;
	}
	return __kt0913_set_standby(radio, false);
}
#endif /* CONFIG_PM */

static const struct dev_pm_ops kt0913_i2c_pm_ops = {
	SET_RUNTIME_PM_OPS(kt0913_i2c_pm_runtime_suspend,
			   kt0913_i2c_pm_runtime_resume, NULL)
};

static const struct i2c_device_id kt0913_idtable[] = {
	{ "kt0913", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, kt0913_idtable);

static struct i2c_driver kt0913_driver = {
	.driver = {
		.name = "kt0913",
		.of_match_table = of_match_ptr(kt0913_of_match),
		.pm = &kt0913_i2c_pm_ops,
	},
	.probe = kt0913_probe,
	.remove = kt0913_remove,
	.id_table = kt0913_idtable,
};
module_i2c_driver(kt0913_driver);

MODULE_AUTHOR("Santiago Hormazabal <santiagohssl@gmail.com>");
MODULE_DESCRIPTION("KTMicro KT0913 AM/FM receiver");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.0.1");

module_param(kt0913_use_campus_band, int, 0);
MODULE_PARM_DESC(kt0913_use_campus_band, "Use the Campus Band feature (FM range 32MHz-110MHz)");
module_param(kt0913_v4l2_radio_nr, int, 0);
MODULE_PARM_DESC(kt0913_v4l2_radio_nr, "v4l2 device number to use (i.e. /dev/radioX)");
