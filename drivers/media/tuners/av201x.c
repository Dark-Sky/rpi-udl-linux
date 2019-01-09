/*
 * AV201x Airoha Technology silicon tuner driver
 *
 * Copyright (C) 2014 Luis Alves <ljalvs@gmail.com>
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License along
 *    with this program; if not, write to the Free Software Foundation, Inc.,
 *    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include "av201x.h"
#include "av201x_priv.h"

/* write multiple (continuous) registers */
static int av201x_wrm(struct av201x_priv *priv, u8 *buf, int len)
{
	int ret;
	struct i2c_msg msg = {
		.addr = priv->cfg->i2c_address,
		.flags = 0, .buf = buf, .len = len };

//	fprintk("buf = %*ph", len, buf);

	ret = i2c_transfer(priv->i2c, &msg, 1);
	if (ret < 0) {
		fprintk("ERROR: %d", ret);
		return ret;
	}
	return 0;
}

/* write one register */
static int av201x_wr(struct av201x_priv *priv, u8 addr, u8 data)
{
	u8 buf[] = { addr, data };
	return av201x_wrm(priv, buf, 2);
}

/* read multiple (continuous) registers starting at addr */
static int av201x_rdm(struct av201x_priv *priv, u8 addr, u8 *buf, int len)
{
	int ret;
	struct i2c_msg msg[] = {
		{ .addr = priv->cfg->i2c_address, .flags = 0,
			.buf = &addr, .len = 1 },
		{ .addr = priv->cfg->i2c_address, .flags = I2C_M_RD,
			.buf = buf, .len = len }
	};

	ret = i2c_transfer(priv->i2c, msg, 2);
	if (ret < 0) {
		fprintk("ERROR: %d", ret);
		return ret;
	}

//	fprintk("addr: %02x, buf = %*ph", addr, len, buf);
	return 0;
}

/* read one register */
static int av201x_rd(struct av201x_priv *priv, u8 addr, u8 *data)
{
	return av201x_rdm(priv, addr, data, 1);
}

/* read register, apply masks, write back */
static int av201x_regmask(struct av201x_priv *priv, u8 reg, u8 setmask, u8 clrmask)
{
	int ret;
	u8 b = 0;
	if (clrmask != 0xff) {
		ret = av201x_rd(priv, reg, &b);
		if (ret)
			return ret;
		b &= ~clrmask;
	}
	return av201x_wr(priv, reg, b | setmask);
}

static int av201x_wrtable(struct av201x_priv *priv, struct av201x_regtable *regtable, int len)
{
	int ret, i;

	for (i = 0; i < len; i++) {
		ret = av201x_regmask(priv, regtable[i].addr,
			regtable[i].setmask, regtable[i].clrmask);
		if (ret)
			return ret;
		if (regtable[i].sleep)
			msleep(regtable[i].sleep);
	}
	return 0;
}

static void av201x_release(struct dvb_frontend *fe)
{
	fprintk("");

	kfree(fe->tuner_priv);
	fe->tuner_priv = NULL;
}

static int av201x_init(struct dvb_frontend *fe)
{
	struct av201x_priv *priv = fe->tuner_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int ret;
	fprintk("");

	ret = av201x_wrtable(priv, av201x_inittuner0, ARRAY_SIZE(av201x_inittuner0));

	switch (priv->cfg->id) {
	case ID_AV2011:
		ret |= av201x_wrtable(priv, av201x_inittuner1a, ARRAY_SIZE(av201x_inittuner1a));
		break;
	case ID_AV2012:
	default:
		ret |= av201x_wrtable(priv, av201x_inittuner1b, ARRAY_SIZE(av201x_inittuner1b));
		break;
	}

	ret |= av201x_wrtable(priv, av201x_inittuner2, ARRAY_SIZE(av201x_inittuner2));

	ret |= av201x_wr(priv, REG_TUNER_CTRL, 0x96);

	msleep(120);
	/* initialize strength statistics, signal to apps that tuner is providing strength stats */
	c->strength.len = 1;
	c->strength.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	c->strength.stat[0].uvalue = 0;
	if (ret)
		fprintk("ERROR");
	return ret;
}

static int av201x_sleep(struct dvb_frontend *fe)
{
	struct av201x_priv *priv = fe->tuner_priv;
	int ret;
	fprintk("");

	ret = av201x_regmask(priv, REG_TUNER_CTRL, AV201X_SLEEP, 0);
	if (ret)
		fprintk("ERROR");
	return ret;
}

static int av201x_set_bandwidth(struct dvb_frontend *fe, u32 bw)
{
	struct av201x_priv *priv = fe->tuner_priv;
	u32 bf;
	int ret;

	/* check limits (4MHz < bw < 40MHz) */
	if (bw > 40000)
		bw = 40000;
	if (bw < 4000)
		bw = 4000;

	/* bandwidth step = 211kHz */
	bf = DIV_ROUND_CLOSEST(bw * 127, 21100);
	ret = av201x_wr(priv, REG_BWFILTER, (u8) bf);

	return ret;
}

static int av201x_set_params(struct dvb_frontend *fe)
{
	struct av201x_priv *priv = fe->tuner_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	u32 n, bw;
	u8 buf[5];
	int ret;

//	fprintk("");

	/*
	   ** PLL setup **
	   RF = (pll_N * ref_freq) / pll_M
	   pll_M = fixed 0x10000
	   PLL output is divided by 2
	   REG_FN = pll_M<24:0>
	*/
	buf[0] = REG_FN;
	n = DIV_ROUND_CLOSEST(c->frequency, priv->cfg->xtal_freq);
	buf[1] = (n > 0xff) ? 0xff : (u8) n;
	n = DIV_ROUND_CLOSEST((c->frequency / 1000) << 17, priv->cfg->xtal_freq / 1000);
	buf[2] = (u8) (n >> 9);
	buf[3] = (u8) (n >> 1);
	buf[4] = (u8) (((n << 7) & 0x80) | 0x50);
	ret = av201x_wrm(priv, buf, 5);
	if (ret)
		goto exit;

	msleep(20);

	/* set bandwidth */
	bw = (c->symbol_rate / 1000) * 135/200;
	if (c->symbol_rate < 6500000)
		bw += 6000;
	bw += 2000;
	bw *= 108/100;
	av201x_set_bandwidth(fe, bw);

	/* enable fine tune agc */
	ret |= av201x_wr(priv, REG_FT_CTRL, AV201X_FT_EN | AV201X_FT_BLK);
	ret |= av201x_wr(priv, REG_TUNER_CTRL, 0x96);
	msleep(20);
exit:
	if (ret)
		fprintk("ERROR");
	return ret;
}

static  int   AV201x_agc         [] = {     0,  82,   100,  116,  140,  162,  173,  187,  210,  223,  254,  255};
static  int   AV201x_level_dBm_10[] = {    90, -50,  -263, -361, -463, -563, -661, -761, -861, -891, -904, -910};

static int av201x_get_rf_strength(struct dvb_frontend *fe, u16 *st)
{
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int   if_agc, index, table_length, slope, *x, *y;

	if_agc = *st;
	x = AV201x_agc;
	y = AV201x_level_dBm_10;
	table_length = sizeof(AV201x_agc)/sizeof(int);

	/* Finding in which segment the if_agc value is */
	for (index = 0; index < table_length; index ++)
		if (x[index] > if_agc ) break;

	/* Computing segment slope */
	slope =  ((y[index]-y[index-1])*1000)/(x[index]-x[index-1]);
	/* Linear approximation of rssi value in segment (rssi values will be in 0.1dBm unit: '-523' means -52.3 dBm) */
	*st = 1000 + ((y[index-1] + ((if_agc - x[index-1])*slope + 500)/1000))/10;

	c->strength.len = 1;
	c->strength.stat[0].scale = FE_SCALE_DECIBEL;
	c->strength.stat[0].svalue = ((y[index-1] + ((if_agc - x[index-1])*slope + 500)/1000)) * 100;

	return 0;
}


static const struct dvb_tuner_ops av201x_tuner_ops = {
	.info = {
		.name           = "Airoha Technology AV201x",
		.frequency_min_hz = 850 * MHz,
		.frequency_max_hz = 2300 * MHz,
	},

	.release = av201x_release,

	.init = av201x_init,
	.sleep = av201x_sleep,
	.set_params = av201x_set_params,
	.set_bandwidth = av201x_set_bandwidth,
	.get_rf_strength = av201x_get_rf_strength,
};

struct dvb_frontend *av201x_attach(struct dvb_frontend *fe, struct av201x_config *cfg, struct i2c_adapter *i2c)
{
	struct av201x_priv *priv = NULL;

	priv = kzalloc(sizeof(struct av201x_priv), GFP_KERNEL);
	if (priv == NULL) {
		fprintk("attach ERROR");
		return NULL;
	}

	priv->cfg = cfg;
	priv->i2c = i2c;

	fprintk("Airoha Technology AV201x successfully attached");

	memcpy(&fe->ops.tuner_ops, &av201x_tuner_ops,
			sizeof(struct dvb_tuner_ops));

	fe->tuner_priv = priv;
	return fe;
}
EXPORT_SYMBOL(av201x_attach);

MODULE_DESCRIPTION("Airoha Technology AV201x silicon tuner driver");
MODULE_AUTHOR("Luis Alves <ljalvs@gmail.com>");
MODULE_LICENSE("GPL");
