/*
    Tmax TAS2101 - DVBS/S2 Satellite demodulator driver

    Copyright (C) 2014 Luis Alves <ljalvs@gmail.com>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/

#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/i2c-mux.h>

#include "media/dvb_frontend.h"

#include "tas2101.h"
#include "tas2101_priv.h"

#define dprintk(fmt, arg...)	printk(KERN_INFO "%s: " fmt "\n",  __func__, ##arg)

/* return i2c adapter */
/* bus = 0   master   */
/* bus = 1   demod    */
/* bus = 2   tuner    */
struct i2c_adapter *tas2101_get_i2c_adapter(struct dvb_frontend *fe, int bus)
{
	struct tas2101_priv *priv = fe->demodulator_priv;

	switch (bus) {
	case 0:
	default:
		return priv->i2c;
	case 1:
		return priv->i2c_demod;
	case 2:
		return priv->i2c_tuner;
	}
}
EXPORT_SYMBOL_GPL(tas2101_get_i2c_adapter);

/* write multiple (continuous) registers */
/* the first value is the starting address */
static int tas2101_wrm(struct tas2101_priv *priv, u8 *buf, int len)
{
	int ret;
	struct i2c_msg msg = {
		.addr = priv->cfg->i2c_address,
		.flags = 0, .buf = buf, .len = len };

//	dprintk("buf = %*ph", len, buf);

	ret = i2c_transfer(priv->i2c_demod, &msg, 1);
	if (ret < 0) {
		dprintk("err(%i) @0x%02x (len=%d)", ret, buf[0], len);
		return ret;
	}
	return 0;
}

/* write one register */
static int tas2101_wr(struct tas2101_priv *priv, u8 addr, u8 data)
{
	u8 buf[] = { addr, data };
	return tas2101_wrm(priv, buf, 2);
}

/* read multiple (continuous) registers starting at addr */
static int tas2101_rdm(struct tas2101_priv *priv, u8 addr, u8 *buf, int len)
{
	int ret;
	struct i2c_msg msg[] = {
		{ .addr = priv->cfg->i2c_address, .flags = 0,
			.buf = &addr, .len = 1 },
		{ .addr = priv->cfg->i2c_address, .flags = I2C_M_RD,
			.buf = buf, .len = len }
	};

	ret = i2c_transfer(priv->i2c_demod, msg, 2);
	if (ret < 0) {
		dprintk("err(%i) @0x%02x (len=%d)", ret, addr, len);
		return ret;
	}
//	dprintk("addr: %02x, buf = %*ph", addr, len, buf);
	return 0;
}

/* read one register */
static int tas2101_rd(struct tas2101_priv *priv, u8 addr, u8 *data)
{
	return tas2101_rdm(priv, addr, data, 1);
}

static int tas2101_regmask(struct tas2101_priv *priv, u8 reg, u8 setmask, u8 clrmask)
{
	int ret;
	u8 b = 0;
	if (clrmask != 0xff) {
		ret = tas2101_rd(priv, reg, &b);
		if (ret)
			return ret;
		b &= ~clrmask;
	}
	return tas2101_wr(priv, reg, b | setmask);
}

static int tas2101_wrtable(struct tas2101_priv *priv, struct tas2101_regtable *regtable, int len)
{
	int ret, i;

	for (i = 0; i < len; i++) {
		ret = tas2101_regmask(priv, regtable[i].addr,
			regtable[i].setmask, regtable[i].clrmask);
		if (ret)
			return ret;
		if (regtable[i].sleep)
			msleep(regtable[i].sleep);
	}
	return 0;
}

static int tas2101_read_ber(struct dvb_frontend *fe, u32 *ber)
{
	struct tas2101_priv *priv = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int ret;
	u8 buf[4];

	switch (c->delivery_system) {
		case SYS_DVBS:
			ret = tas2101_rdm(priv, S1_BER_0, buf, 4);
			if (ret)
				return ret;

			*ber = ((((u32) buf[3] & 3) << 24) | (((u32) buf[2]) << 16)
				| (((u32) buf[1]) << 8) | ((u32) buf[0]));
			break;

		case SYS_DVBS2:
			ret = tas2101_rdm(priv, S2_BER_0, buf, 2);
			if (ret)
				return ret;

			*ber = ((((u32) buf[1]) << 8) | ((u32) buf[0]));
			break;

		default:
			*ber = 0;
			break;
	}

//	dprintk("ber = %d", *ber);
	return 0;
}

static int tas2101_read_signal_strength(struct dvb_frontend *fe, u16 *signal_strength)
{
	struct tas2101_priv *priv = fe->demodulator_priv;
	int ret, i;
	long val, dbm_raw;
	u8 buf[2];

	ret = tas2101_rdm(priv, SIGSTR_0, buf, 2);
	if (ret)
		return ret;

	dbm_raw = (((u16)buf[1] & 0x0f) << 8) | buf[0];

	for (i = 0; i < ARRAY_SIZE(tas2101_dbmtable) - 1; i++)
		if (tas2101_dbmtable[i].raw < dbm_raw)
			break;

	if (i == 0)
		*signal_strength = tas2101_dbmtable[i].dbm;
	else
	{
		/* linear interpolation between two calibrated values */
		val = (dbm_raw - tas2101_dbmtable[i].raw) * tas2101_dbmtable[i-1].dbm;
		val += (tas2101_dbmtable[i-1].raw - dbm_raw) * tas2101_dbmtable[i].dbm;
		val /= (tas2101_dbmtable[i-1].raw - tas2101_dbmtable[i].raw);

		*signal_strength = (u16)val;
	}

//	dprintk("strength = 0x%04x", *signal_strength);
	return 0;
}

static int tas2101_read_snr(struct dvb_frontend *fe, u16 *snr)
{
	struct tas2101_priv *priv = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int ret, i;
	long val;
	u16 snr_raw;
	u8 buf[2];

	ret = tas2101_rdm(priv, SNR_0, buf, 2);
	if (ret)
		return ret;

	snr_raw = (((u16)buf[1] & 0x0f) << 8) | buf[0];

	for (i = 0; i < ARRAY_SIZE(tas2101_snrtable) - 1; i++)
		if (tas2101_snrtable[i].raw < snr_raw)
			break;

	if (i == 0)
		val = tas2101_snrtable[i].snr;
	else
	{
		/* linear interpolation between two calibrated values */
		val = (snr_raw - tas2101_snrtable[i].raw) * tas2101_snrtable[i-1].snr;
		val += (tas2101_snrtable[i-1].raw - snr_raw) * tas2101_snrtable[i].snr;
		val /= (tas2101_snrtable[i-1].raw - tas2101_snrtable[i].raw);
	}

	c->cnr.len = 1;
	c->cnr.stat[0].scale = FE_SCALE_DECIBEL;
	c->cnr.stat[0].uvalue = 100 * (s64) val;

	*snr = (u16) val * 328; /* 20dB = 100% */

//	dprintk("snr = 0x%04x", *snr);

	return 0;
}

/* unimplemented */
static int tas2101_read_ucblocks(struct dvb_frontend *fe, u32 *ucblocks)
{
	*ucblocks = 0;
	return 0;
}

static int tas2101_read_status(struct dvb_frontend *fe, enum fe_status *status)
{
	struct tas2101_priv *priv = fe->demodulator_priv;
	int ret;
	u8 reg;
	u16 snr;

	*status = 0;

	ret = tas2101_rd(priv, DEMOD_STATUS, &reg);
	if (ret)
		return ret;

	reg &= DEMOD_STATUS_MASK;
	if (reg == DEMOD_LOCKED) {
		*status = FE_HAS_SIGNAL | FE_HAS_CARRIER | FE_HAS_VITERBI | FE_HAS_SYNC | FE_HAS_LOCK;

		ret = tas2101_rd(priv, REG_04, &reg);
		if (ret)
			return ret;
		if (reg & 0x08)
			ret = tas2101_wr(priv, REG_04, reg & ~0x08);

		tas2101_read_snr(fe, &snr);
	}

	if (priv->algo == TAS2101_NOTUNE) {
		*status |= FE_TIMEDOUT;
	}

//	dprintk("status = 0x%02x", *status);
	return ret;
}

static void tas2101_spi_read(struct dvb_frontend *fe, struct ecp3_info *ecp3inf)
{

	struct tas2101_priv *priv = fe->demodulator_priv;
	struct i2c_adapter *adapter = priv->i2c;

	if (priv->cfg->read_properties)
		priv->cfg->read_properties(adapter,ecp3inf->reg, &(ecp3inf->data));
	return;
}
static void tas2101_spi_write(struct dvb_frontend *fe,struct ecp3_info *ecp3inf)
{
	struct tas2101_priv *priv = fe->demodulator_priv;
	struct i2c_adapter *adapter = priv->i2c;

	if (priv->cfg->write_properties)
		priv->cfg->write_properties(adapter,ecp3inf->reg, ecp3inf->data);
	return ;
}

static int tas2101_set_voltage(struct dvb_frontend *fe, enum fe_sec_voltage voltage)
{
	struct tas2101_priv *priv = fe->demodulator_priv;
	int ret = 0;

	switch (voltage) {
		case SEC_VOLTAGE_13:
			dprintk("SEC_VOLTAGE_13");
			if (priv->cfg->lnb_power)
				priv->cfg->lnb_power(fe, LNB_ON);
			ret = tas2101_regmask(priv, LNB_CTRL, 0, VSEL13_18);
			break;
		case SEC_VOLTAGE_18:
			dprintk("SEC_VOLTAGE_18");
			if (priv->cfg->lnb_power)
				priv->cfg->lnb_power(fe, LNB_ON);
			ret = tas2101_regmask(priv, LNB_CTRL, VSEL13_18, 0);
			break;
		default:
			dprintk("SEC_VOLTAGE_OFF");
			if (priv->cfg->lnb_power)
				priv->cfg->lnb_power(fe, LNB_OFF);
			break;
	}
	return ret;
}

static int tas2101_set_tone(struct dvb_frontend *fe, enum fe_sec_tone_mode tone)
{
	struct tas2101_priv *priv = fe->demodulator_priv;
	int ret = -EINVAL;

	switch (tone) {
	case SEC_TONE_ON:
		dprintk("SEC_TONE_ON");
		ret = tas2101_regmask(priv, LNB_CTRL, TONE_ON, DISEQC_CMD_MASK);
		break;
	case SEC_TONE_OFF:
		dprintk("SEC_TONE_OFF");
		ret = tas2101_regmask(priv, LNB_CTRL, TONE_OFF, DISEQC_CMD_MASK);
		break;
	default:
		dprintk("invalid tone (%d)", tone);
		break;
	}
	return ret;
}

static int tas2101_send_diseqc_msg(struct dvb_frontend *fe, struct dvb_diseqc_master_cmd *d)
{
	struct tas2101_priv *priv = fe->demodulator_priv;
	int ret, i;
	u8 bck, buf[9];

	/* backup LNB tone state */
	ret = tas2101_rd(priv, LNB_CTRL, &bck);
	if (ret)
		return ret;

	ret = tas2101_regmask(priv, REG_34, 0, 0x40);
	if (ret)
		goto exit;

	/* setup DISEqC message to demod */
	buf[0] = DISEQC_BUFFER;
	memcpy(&buf[1], d->msg, 8);
	ret = tas2101_wrm(priv, buf, d->msg_len + 1);
	if (ret)
		goto exit;

	/* send DISEqC send command */
	buf[0] = (bck & ~(DISEQC_CMD_LEN_MASK | DISEQC_CMD_MASK)) |
		DISEQC_SEND_MSG | ((d->msg_len - 1) << 3);
	ret = tas2101_wr(priv, LNB_CTRL, buf[0]);
	if (ret)
		goto exit;

	/* wait at least diseqc typical tx time */
	msleep(54);

	/* Wait for busy flag to clear */
	for (i = 0; i < 10; i++) {
		ret = tas2101_rd(priv, LNB_STATUS, &buf[0]);
		if (ret)
			break;
		if (buf[0] & DISEQC_BUSY)
			goto exit;
		msleep(20);
	}

	/* try to restore the tone setting but return a timeout error */
	ret = tas2101_wr(priv, LNB_CTRL, bck);
	dprintk("timeout sending burst");
	return -ETIMEDOUT;
exit:
	/* restore tone setting */
	return tas2101_wr(priv, LNB_CTRL, bck);
}

static int tas2101_diseqc_send_burst(struct dvb_frontend *fe, enum fe_sec_mini_cmd burst)
{
	struct tas2101_priv *priv = fe->demodulator_priv;
	int ret, i;
	u8 bck, r;

	if ((burst != SEC_MINI_A) && (burst != SEC_MINI_B)) {
		dprintk("invalid burst(%d)", burst);
		return -EINVAL;
	}

	/* backup LNB tone state */
	ret = tas2101_rd(priv, LNB_CTRL, &bck);
	if (ret)
		return ret;

	ret = tas2101_regmask(priv, REG_34, 0, 0x40);
	if (ret)
		goto exit;

	/* set tone burst cmd */
	r = (bck & ~DISEQC_CMD_MASK) | (burst == SEC_MINI_A) ? DISEQC_BURST_A : DISEQC_BURST_B;

	ret = tas2101_wr(priv, LNB_CTRL, r);
	if (ret)
		goto exit;

	/* spec = around 12.5 ms for the burst */
	for (i = 0; i < 10; i++) {
		ret = tas2101_rd(priv, LNB_STATUS, &r);
		if (ret)
			break;
		if (r & DISEQC_BUSY)
			goto exit;
		msleep(20);
	}

	/* try to restore the tone setting but return a timeout error */
	ret = tas2101_wr(priv, LNB_CTRL, bck);
	dprintk("timeout sending burst");
	return -ETIMEDOUT;
exit:
	/* restore tone setting */
	return tas2101_wr(priv, LNB_CTRL, bck);
}

static void tas2101_release(struct dvb_frontend *fe)
{
	struct tas2101_priv *priv = fe->demodulator_priv;

	dprintk("");
	i2c_mux_del_adapters(priv->muxc);
	kfree(priv);
}

/* channel 0: demod */
/* channel 1: tuner */
static int tas2101_i2c_select(struct i2c_mux_core *muxc, u32 chan_id)
{
	struct tas2101_priv *priv = i2c_mux_priv(muxc);
	struct i2c_adapter *adap = priv->i2c;
	int ret;
	u8 buf[2];
	struct i2c_msg msg_wr[] = {
		{ .addr = priv->cfg->i2c_address, .flags = 0, .buf = buf, .len = 2 }
	};
	struct i2c_msg msg_rd[] = {
		{ .addr = priv->cfg->i2c_address, .flags = 0, .buf = &buf[0], .len = 1 },
		{ .addr = priv->cfg->i2c_address, .flags = I2C_M_RD, .buf = &buf[1], .len = 1 }
	};

	if (priv->i2c_ch == chan_id)
		return 0;

	buf[0] = REG_06;
	ret = __i2c_transfer(adap, msg_rd, 2);
	if (ret != 2)
		goto err;

	if (chan_id == 0)
		buf[1] &= ~I2C_GATE;
	else
		buf[1] |= I2C_GATE;

	ret = __i2c_transfer(adap, msg_wr, 1);
	if (ret != 1)
		goto err;

	priv->i2c_ch = chan_id;

	return 0;
err:
	dprintk("failed=%d", ret);
	return -EREMOTEIO;
}

static struct dvb_frontend_ops tas2101_ops;

struct dvb_frontend *tas2101_attach(const struct tas2101_config *cfg, struct i2c_adapter *i2c)
{
	struct tas2101_priv *priv = NULL;
	int ret;
	u8 id[2];

	dprintk("");

	/* allocate memory for the priv data */
	priv = kzalloc(sizeof(struct tas2101_priv), GFP_KERNEL);
	if (priv == NULL)
		goto err;

	priv->cfg = cfg;
	priv->i2c = i2c;
	priv->i2c_ch = 0;

	/* create mux i2c adapter for tuner */
	priv->muxc = i2c_mux_alloc(i2c, &i2c->dev, 2, 0, I2C_MUX_LOCKED, tas2101_i2c_select, NULL);
	if (!priv->muxc) {
		ret = -ENOMEM;
		goto err1;
	}
	priv->muxc->priv = priv;
	ret = i2c_mux_add_adapter(priv->muxc, 0, 0, 0);
	if (ret)
		goto err1;
	ret = i2c_mux_add_adapter(priv->muxc, 0, 1, 0);
	if (ret)
		goto err1;
	priv->i2c_demod = priv->muxc->adapter[0];
	priv->i2c_tuner = priv->muxc->adapter[1];


	/* create dvb_frontend */
	memcpy(&priv->fe.ops, &tas2101_ops, sizeof(struct dvb_frontend_ops));
	priv->fe.demodulator_priv = priv;

	/* reset demod */
	if (cfg->reset_demod)
		cfg->reset_demod(&priv->fe);

	msleep(100);

	/* check if demod is alive */
	ret = tas2101_rdm(priv, ID_0, id, 2);
	if ((id[0] != 0x44) || (id[1] != 0x4c))
		ret |= -EIO;
	if (ret)
		goto err3;

	return &priv->fe;

err3:
	i2c_mux_del_adapters(priv->muxc);
err1:
	kfree(priv);
err:
	dev_err(&i2c->dev, "%s: Error attaching frontend", KBUILD_MODNAME);
	return NULL;
}
EXPORT_SYMBOL_GPL(tas2101_attach);

static int tas2101_initfe(struct dvb_frontend *fe)
{
	struct tas2101_priv *priv = fe->demodulator_priv;
	struct tas2101_regtable *t;
	u8 buf[7], size;
	int ret;

	dprintk("");

	priv->algo = TAS2101_NOTUNE;

	if (priv->cfg->id == ID_TAS2101) {
		t = tas2101_initfe0;
		size = ARRAY_SIZE(tas2101_initfe0);
	} else {
		t = tas2100_initfe0;
		size = ARRAY_SIZE(tas2100_initfe0);
	}
	ret = tas2101_wrtable(priv, t, size);
	if (ret)
		return ret;

	buf[0] = 0xe6;
	memcpy(&buf[1], priv->cfg->init, 6);
	ret = tas2101_wrm(priv, buf, 7);
	if (ret)
		return ret;

	ret = tas2101_regmask(priv, 0xe0, priv->cfg->init[6], 0xff);
	if (ret)
		return ret;

	if (priv->cfg->id == ID_TAS2101) {
		t = tas2101_initfe1;
		size = ARRAY_SIZE(tas2101_initfe1);
	} else {
		t = tas2100_initfe1;
		size = ARRAY_SIZE(tas2100_initfe1);
	}
	ret = tas2101_wrtable(priv, t, size);
	if (ret)
		return ret;

	if (priv->cfg->init2) {
		t = tas2101_initfe2;
		size = ARRAY_SIZE(tas2101_initfe2);
		ret = tas2101_wrtable(priv, t, size);
		if (ret)
			return ret;
	}

	return 0;
}

static int tas2101_sleep(struct dvb_frontend *fe)
{
	struct tas2101_priv *priv = fe->demodulator_priv;

	dprintk("");

	priv->algo = TAS2101_NOTUNE;

	return 0;
}

static int tas2101_set_frontend(struct dvb_frontend *fe)
{
	struct tas2101_priv *priv = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int ret;
	u32 s;
	u8 buf[3];

	dprintk("");

	/* do some basic parameter validation */
	switch (c->delivery_system) {
	case SYS_DVBS:
		dprintk("DVB-S");
		/* Only QPSK is supported for DVB-S */
		if (c->modulation != QPSK) {
			dprintk("unsupported modulation (%d)", c->modulation);
			return -EINVAL;
		}
		break;
	case SYS_DVBS2:
		dprintk("DVB-S2");
		break;
	default:
		dprintk("unsupported delivery system (%d)", c->delivery_system);
		return -EINVAL;
	}

	ret = tas2101_wrtable(priv, tas2101_setfe, ARRAY_SIZE(tas2101_setfe));
	if (ret)
		return ret;

	/* set symbol rate */
	s = c->symbol_rate / 1000;
	buf[0] = SET_SRATE0;
	buf[1] = (u8) s;
	buf[2] = (u8) (s >> 8);
	ret = tas2101_wrm(priv, buf, 3);
	if (ret)
		return ret;

	/* clear freq offset */
	buf[0] = FREQ_OS0;
	buf[1] = 0;
	buf[2] = 0;
	ret = tas2101_wrm(priv, buf, 3);
	if (ret)
		return ret;

	if (fe->ops.tuner_ops.set_params) {
		fe->ops.tuner_ops.set_params(fe);
	}

	ret = tas2101_regmask(priv, REG_30, 0x01, 0);
	if (ret)
		return ret;

	return 0;
}

static int tas2101_get_frontend(struct dvb_frontend *fe, struct dtv_frontend_properties *c)
{
	struct tas2101_priv *priv = fe->demodulator_priv;
	int ret;
	u8 reg, buf[2];

	ret = tas2101_rd(priv, MODFEC_0, &reg);
	if (ret)
		return ret;

	if ((reg >> 6) == 0) {
		/* DVB-S */
		reg &= 0x07;
	} else {
		/* DVB-S2 */
		ret = tas2101_rd(priv, MODFEC_1, &reg);
		if (ret)
			return ret;
		reg += 5;
	}

	if (reg > 33) {
		dprintk("Unable to get current delivery system and mode");
		reg = 0;
	}

	c->fec_inner = tas2101_modfec_modes[reg].fec;
	c->modulation = tas2101_modfec_modes[reg].modulation;
	c->delivery_system = tas2101_modfec_modes[reg].delivery_system;
	c->inversion = INVERSION_AUTO;

	/* symbol rate */
	ret = tas2101_rdm(priv, GET_SRATE0, buf, 2);
	if (ret)
		return ret;
	c->symbol_rate = ((buf[1] << 8) | buf[0]) * 1000;

	return 0;
}

static int tas2101_tune(struct dvb_frontend *fe, bool re_tune, unsigned int mode_flags, unsigned int *delay, enum fe_status *status)
{
	*delay = HZ / 5;
	if (re_tune) {
		int ret = tas2101_set_frontend(fe);
		if (ret)
			return ret;
	}
	return tas2101_read_status(fe, status);
}

static int tas2101_get_algo(struct dvb_frontend *fe)
{
	struct tas2101_priv *priv = fe->demodulator_priv;

	if (priv->algo == TAS2101_NOTUNE) {
		return DVBFE_ALGO_NOTUNE;
	} else {
		return DVBFE_ALGO_CUSTOM;
	}
}

static int tas2101_dtv_tune(struct dvb_frontend *fe)
{
	struct tas2101_priv *priv = fe->demodulator_priv;

	priv->algo = TAS2101_TUNE;

	return 0;
}

static int tas2101_search(struct dvb_frontend *fe)
{
	struct tas2101_priv *priv = fe->demodulator_priv;
	enum fe_status status = 0;
	int ret, i;

	priv->algo = TAS2101_TUNE;

	/* set frontend */
	ret = tas2101_set_frontend(fe);
	if (ret)
		goto error;

	/* wait frontend lock */
	for (i = 0; i < 5; i++) {
		dprintk("loop=%d", i);
		msleep(200);
		ret = tas2101_read_status(fe, &status);
		if (ret)
			goto error;

		if (status & FE_HAS_LOCK)
			break;
		if (status & FE_TIMEDOUT) {
			priv->algo = TAS2101_NOTUNE;
			return DVBFE_ALGO_SEARCH_FAILED;
		}
	}

	/* check if we have a valid signal */
	if (status & FE_HAS_LOCK) {
		dprintk("DVBFE_ALGO_SEARCH_SUCCESS");
		return DVBFE_ALGO_SEARCH_SUCCESS;
	} else {
		dprintk("DVBFE_ALGO_SEARCH_FAILED");
		priv->algo = TAS2101_NOTUNE;
		return DVBFE_ALGO_SEARCH_FAILED;
	}

error:
	dprintk("ERROR");
	priv->algo = TAS2101_NOTUNE;
	return DVBFE_ALGO_SEARCH_ERROR;
}

static int tas2101_get_spectrum_scan(struct dvb_frontend *fe, struct dvb_fe_spectrum_scan *s)
{
	struct tas2101_priv *priv = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	long dbm_raw;
	int ret, x;
	u32 sr;
	u8 buf[3];

	priv->algo = TAS2101_NOTUNE;

	ret = tas2101_wrtable(priv, tas2101_setfe, ARRAY_SIZE(tas2101_setfe));
	if (ret)
		return ret;

	c->symbol_rate		= 1000000;
	c->delivery_system	= SYS_DVBS;

	/* set symbol rate */
	sr = c->symbol_rate / 1000;
	buf[0] = SET_SRATE0;
	buf[1] = (u8) sr;
	buf[2] = (u8) (sr >> 8);
	ret = tas2101_wrm(priv, buf, 3);
	if (ret)
		return ret;

	/* clear freq offset */
	buf[0] = FREQ_OS0;
	buf[1] = 0;
	buf[2] = 0;
	ret = tas2101_wrm(priv, buf, 3);
	if (ret)
		return ret;

	for (x = 0 ; x < s->num_freq ; x++)
	{
		c->frequency = *(s->freq + x);

		if (fe->ops.tuner_ops.set_params) {
			fe->ops.tuner_ops.set_params(fe);

			msleep(100);

			ret = tas2101_rdm(priv, SIGSTR_0, buf, 2);
			if (ret)
				return ret;
			dbm_raw = (((u16)buf[1] & 0x0f) << 8) | buf[0];
			dbm_raw = 4095 - dbm_raw;
			*(s->rf_level + x) = dbm_raw;
		}
	}
	return 0;
}

static struct dvb_frontend_ops tas2101_ops = {
	.delsys = { SYS_DVBS, SYS_DVBS2 },
	.info = {
		.name = "Tmax TAS2101",
		.frequency_min = 950000,
		.frequency_max = 2150000,
		.frequency_stepsize = 1011, /* kHz for QPSK frontends */
		.frequency_tolerance = 5000,
		.symbol_rate_min = 100000,
		.symbol_rate_max = 67500000,
		.caps = FE_CAN_INVERSION_AUTO |
			FE_CAN_FEC_1_2 | FE_CAN_FEC_2_3 | FE_CAN_FEC_3_4 |
			FE_CAN_FEC_4_5 | FE_CAN_FEC_5_6 | FE_CAN_FEC_6_7 |
			FE_CAN_FEC_7_8 | FE_CAN_FEC_AUTO |
			FE_CAN_2G_MODULATION |
			FE_CAN_QPSK | FE_CAN_RECOVER |
			FE_HAS_EXTENDED_CAPS
	},
	.extended_info = {
		.extended_caps          = FE_CAN_SPECTRUMSCAN |
					  FE_CAN_BLINDSEARCH
	},
	.release = tas2101_release,

	.init = tas2101_initfe,
	.sleep = tas2101_sleep,
	.read_status = tas2101_read_status,
	.read_ber = tas2101_read_ber,
	.read_signal_strength = tas2101_read_signal_strength,
	.read_snr = tas2101_read_snr,
	.read_ucblocks = tas2101_read_ucblocks,

	.set_tone = tas2101_set_tone,
	.set_voltage = tas2101_set_voltage,
	.diseqc_send_master_cmd = tas2101_send_diseqc_msg,
	.diseqc_send_burst = tas2101_diseqc_send_burst,
	.get_frontend_algo = tas2101_get_algo,
	.tune = tas2101_tune,

	.set_frontend = tas2101_set_frontend,
	.get_frontend = tas2101_get_frontend,

	.spi_read	= tas2101_spi_read,
	.spi_write	= tas2101_spi_write,

	.search = tas2101_search,
	.dtv_tune = tas2101_dtv_tune,
	.get_spectrum_scan = tas2101_get_spectrum_scan,
};

MODULE_DESCRIPTION("DVB Frontend module for Tmax TAS2101");
MODULE_AUTHOR("Luis Alves (ljalvs@gmail.com)");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");

