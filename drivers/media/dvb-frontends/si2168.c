/*
 * Silicon Labs Si2168 DVB-T/T2/C demodulator driver
 *
 * Copyright (C) 2014 Antti Palosaari <crope@iki.fi>
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
 */

#include <linux/delay.h>

#include "si2168_priv.h"

static const struct dvb_frontend_ops si2168_ops;

/* execute firmware command */
static int si2168_cmd_execute(struct i2c_client *client, struct si2168_cmd *cmd)
{
	struct si2168_dev *dev = i2c_get_clientdata(client);
	int ret;
	unsigned long timeout;

//	fprintk("W: %*ph", cmd->wlen, cmd->args);

	mutex_lock(&dev->i2c_mutex);

	if (cmd->wlen) {
		memcpy(cmd->w_args, cmd->args, cmd->wlen);
		/* write cmd and args for firmware */
		ret = i2c_master_send(client, cmd->args, cmd->wlen);
		if (ret < 0) {
			goto w_err;
		} else if (ret != cmd->wlen) {
			ret = -EREMOTEIO;
			goto w_err;
		}
	}

	if (cmd->rlen) {
		/* wait cmd execution terminate */
		#define TIMEOUT 500
		timeout = jiffies + msecs_to_jiffies(TIMEOUT);
		while (!time_after(jiffies, timeout)) {
			ret = i2c_master_recv(client, cmd->args, cmd->rlen);
			if (ret < 0) {
				goto r_err;
			} else if (ret != cmd->rlen) {
				ret = -EREMOTEIO;
				goto r_err;
			}

			/* firmware ready? */
			if ((cmd->args[0] >> 7) & 0x01)
				break;
		}

		/* error bit set? */
		if ((cmd->args[0] >> 6) & 0x01) {
			ret = -EREMOTEIO;
			goto r_err;
		}

		if (!((cmd->args[0] >> 7) & 0x01)) {
			ret = -ETIMEDOUT;
			goto r_err;
		}
	}

	mutex_unlock(&dev->i2c_mutex);

//	fprintk("R: %*ph", cmd->rlen, cmd->args);

	return 0;
w_err:
	mutex_unlock(&dev->i2c_mutex);
	fprintk("Write: failed=%d", ret);
	fprintk("W: %*ph", cmd->wlen, cmd->args);
	return ret;
r_err:
	mutex_unlock(&dev->i2c_mutex);
	fprintk("Read: failed=%d", ret);
	fprintk("W: %*ph", cmd->wlen, cmd->w_args);
	fprintk("R: %*ph", cmd->rlen, cmd->args);
	return ret;
}

static int si2168_ts_bus_ctrl(struct dvb_frontend *fe, int acquire)
{
	struct i2c_client *client = fe->demodulator_priv;
	struct si2168_dev *dev = i2c_get_clientdata(client);
	struct si2168_cmd cmd;
	int ret = 0;

	fprintk("acquire: %d", acquire);

	/* set TS_MODE property */
	memcpy(cmd.args, "\x14\x00\x01\x10\x10\x00", 6);
	if (acquire)
		cmd.args[4] |= dev->ts_mode;
	else
		cmd.args[4] |= SI2168_TS_TRISTATE;
	if (dev->ts_clock_gapped)
		cmd.args[4] |= 0x40;
	cmd.wlen = 6;
	cmd.rlen = 4;
	ret = si2168_cmd_execute(client, &cmd);

	return ret;
}

static int si2168_read_status(struct dvb_frontend *fe, enum fe_status *status)
{
	struct i2c_client *client = fe->demodulator_priv;
	struct si2168_dev *dev = i2c_get_clientdata(client);
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int ret, i;
	unsigned int utmp, utmp1, utmp2;
	struct si2168_cmd cmd;

	*status = 0;

	if (!dev->active) {
		ret = -EAGAIN;
		goto err;
	}

	switch (c->delivery_system) {
	case SYS_DVBT:
		memcpy(cmd.args, "\xa0\x01", 2);
		cmd.wlen = 2;
		cmd.rlen = 13;
		break;
	case SYS_DVBC_ANNEX_A:
		memcpy(cmd.args, "\x90\x01", 2);
		cmd.wlen = 2;
		cmd.rlen = 9;
		break;
	case SYS_DVBC_ANNEX_B:
		memcpy(cmd.args, "\x98\x01", 2);
		cmd.wlen = 2;
		cmd.rlen = 10;
		break;
	case SYS_DVBT2:
		memcpy(cmd.args, "\x50\x01", 2);
		cmd.wlen = 2;
		cmd.rlen = 14;
		break;
	default:
		ret = -EINVAL;
		goto err;
	}

	ret = si2168_cmd_execute(client, &cmd);
	if (ret)
		goto err;

	switch ((cmd.args[2] >> 1) & 0x03) {
	case 0x01:
		*status = FE_HAS_SIGNAL | FE_HAS_CARRIER;
		break;
	case 0x03:
		*status = FE_HAS_SIGNAL | FE_HAS_CARRIER | FE_HAS_VITERBI |
				FE_HAS_SYNC | FE_HAS_LOCK;
		break;
	}

	if (dev->algo == SI2168_NOTUNE)
		*status |= FE_TIMEDOUT;
	dev->fe_status = *status;

// TODO pretty sure we can remove this if() and have it check SNR all the time
	if (*status & FE_HAS_LOCK) {
		c->cnr.len = 1;
		c->cnr.stat[0].scale = FE_SCALE_DECIBEL;
		c->cnr.stat[0].svalue = cmd.args[3] * 1000 / 4;
	} else {
		c->cnr.len = 1;
		c->cnr.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	}

	/* BER */
	if (*status & FE_HAS_VITERBI) {
		memcpy(cmd.args, "\x82\x00", 2);
		cmd.wlen = 2;
		cmd.rlen = 3;
		ret = si2168_cmd_execute(client, &cmd);
		if (ret)
			goto err;

		/*
		 * Firmware returns [0, 255] mantissa and [0, 8] exponent.
		 * Convert to DVB API: mantissa * 10^(8 - exponent) / 10^8
		 */
		utmp = clamp(8 - cmd.args[1], 0, 8);
		for (i = 0, utmp1 = 1; i < utmp; i++)
			utmp1 = utmp1 * 10;

		utmp1 = cmd.args[2] * utmp1;
		utmp2 = 100000000; /* 10^8 */

		fprintk(
			"post_bit_error=%u post_bit_count=%u ber=%u*10^-%u",
			utmp1, utmp2, cmd.args[2], cmd.args[1]);

		c->post_bit_error.stat[0].scale = FE_SCALE_COUNTER;
		c->post_bit_error.stat[0].uvalue += utmp1;
		c->post_bit_count.stat[0].scale = FE_SCALE_COUNTER;
		c->post_bit_count.stat[0].uvalue += utmp2;
	} else {
		c->post_bit_error.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
		c->post_bit_count.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	}

	/* UCB */
	if (*status & FE_HAS_SYNC) {
		memcpy(cmd.args, "\x84\x01", 2);
		cmd.wlen = 2;
		cmd.rlen = 3;
		ret = si2168_cmd_execute(client, &cmd);
		if (ret)
			goto err;

		utmp1 = cmd.args[2] << 8 | cmd.args[1] << 0;
		fprintk("block_error=%u", utmp1);

		/* Sometimes firmware returns bogus value */
		if (utmp1 == 0xffff)
			utmp1 = 0;

		c->block_error.stat[0].scale = FE_SCALE_COUNTER;
		c->block_error.stat[0].uvalue += utmp1;
	} else {
		c->block_error.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	}

	return 0;
err:
	fprintk("failed=%d", ret);
	return ret;
}

static int si2168_read_snr(struct dvb_frontend *fe, u16 *snr)
{
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;

	*snr = c->cnr.stat[0].scale == FE_SCALE_DECIBEL ? ((s32)c->cnr.stat[0].svalue / 250) *  328  : 0;

	return 0;
}

static int si2168_read_signal_strength(struct dvb_frontend *fe, u16 *strength)
{
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;

	*strength = c->strength.stat[0].scale == FE_SCALE_DECIBEL ? ((100000 + (s32)c->strength.stat[0].svalue) / 1000) * 656 : 0;

	return 0;
}

static int si2168_read_ber(struct dvb_frontend *fe, u32 *ber)
{
	struct i2c_client *client = fe->demodulator_priv;
	struct si2168_dev *dev = i2c_get_clientdata(client);
	struct si2168_cmd cmd;
	int ret;

	if (dev->fe_status & FE_HAS_LOCK) {
		memcpy(cmd.args, "\x82\x00", 2);
		cmd.wlen = 2;
		cmd.rlen = 3;
		ret = si2168_cmd_execute(client, &cmd);
		if (ret) {
			fprintk("read_ber fe%d cmd_exec failed=%d", fe->id, ret);
			goto err;
		}
		*ber = (u32)cmd.args[2] * cmd.args[1] & 0xf;
	} else *ber = 1;

	return 0;
err:
	fprintk("read_ber failed=%d", ret);
	return ret;
}

static int si2168_read_ucblocks(struct dvb_frontend *fe, u32 *ucblocks)
{
	struct i2c_client *client = fe->demodulator_priv;
	struct si2168_dev *dev = i2c_get_clientdata(client);
	struct si2168_cmd cmd;
	int ret;

	if (dev->stat_resp & 0x10) {
		memcpy(cmd.args, "\x84\x00", 2);
		cmd.wlen = 2;
		cmd.rlen = 3;
		ret = si2168_cmd_execute(client, &cmd);
		if (ret) {
		fprintk("read_ucblocks fe%d cmd_exec failed=%d", fe->id, ret);
			goto err;
		}

		*ucblocks = (u16)cmd.args[2] << 8 | cmd.args[1];
	} else 	*ucblocks = 0;

	return 0;
err:
	fprintk("read_ucblocks failed=%d", ret);
	return ret;
}

static enum dvbfe_algo si2168_get_frontend_algo(struct dvb_frontend *fe)
{
	struct i2c_client *client = fe->demodulator_priv;
	struct si2168_dev *dev = i2c_get_clientdata(client);

	if (dev->algo == SI2168_NOTUNE) {
		return DVBFE_ALGO_NOTUNE;
	} else {
		return DVBFE_ALGO_CUSTOM;
	}
}

static int si2168_dtv_tune(struct dvb_frontend *fe)
{
	struct i2c_client *client = fe->demodulator_priv;
	struct si2168_dev *dev = i2c_get_clientdata(client);

	fprintk("");

	dev->algo = SI2168_TUNE;

	return 0;
}

static int si2168_set_frontend(struct dvb_frontend *fe)
{
	struct i2c_client *client = fe->demodulator_priv;
	struct si2168_dev *dev = i2c_get_clientdata(client);
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int ret;
	struct si2168_cmd cmd;
	u8 bandwidth, delivery_system;

	fprintk("");

	if (!dev->active) {
		ret = -EAGAIN;
		goto err;
	}

	dev->algo = SI2168_TUNE;

	switch (c->delivery_system) {
	case SYS_DVBC_ANNEX_B:
		delivery_system = 0x10;
		break;
	case SYS_DVBT:
		delivery_system = 0x20;
		break;
	case SYS_DVBC_ANNEX_A:
		delivery_system = 0x30;
		if (c->symbol_rate < 6000000) {
			delivery_system = 0x10;
			c->delivery_system = SYS_DVBC_ANNEX_B;
			c->bandwidth_hz = 6000000;
		}
		break;
	case SYS_DVBT2:
		delivery_system = 0x70;
		break;
	default:
		ret = -EINVAL;
		goto err;
	}

	if (c->bandwidth_hz == 0) {
		ret = -EINVAL;
		goto err;
	} else if (c->bandwidth_hz <= 2000000)
		bandwidth = 0x02;
	else if (c->bandwidth_hz <= 5000000)
		bandwidth = 0x05;
	else if (c->bandwidth_hz <= 6000000)
		bandwidth = 0x06;
	else if (c->bandwidth_hz <= 7000000)
		bandwidth = 0x07;
	else if (c->bandwidth_hz <= 8000000)
		bandwidth = 0x08;
	else if (c->bandwidth_hz <= 9000000)
		bandwidth = 0x09;
	else if (c->bandwidth_hz <= 10000000)
		bandwidth = 0x0a;
	else
		bandwidth = 0x0f;

	/* program tuner */
	if (fe->ops.tuner_ops.set_params) {
		ret = fe->ops.tuner_ops.set_params(fe);
		if (ret)
			goto err;
	}

	memcpy(cmd.args, "\x88\x02\x02\x02\x02", 5);
	cmd.wlen = 5;
	cmd.rlen = 5;
	ret = si2168_cmd_execute(client, &cmd);
	if (ret)
		goto err;

	/* that has no big effect */
	if (c->delivery_system == SYS_DVBT)
		memcpy(cmd.args, "\x89\x21\x06\x11\xff\x98", 6);
	else if (c->delivery_system == SYS_DVBC_ANNEX_A)
		memcpy(cmd.args, "\x89\x21\x06\x11\x89\xf0", 6);
	else if (c->delivery_system == SYS_DVBT2)
		memcpy(cmd.args, "\x89\x21\x06\x11\x89\x20", 6);
	cmd.wlen = 6;
	cmd.rlen = 3;
	ret = si2168_cmd_execute(client, &cmd);
	if (ret)
		goto err;

	if (c->delivery_system == SYS_DVBT2) {
		/* select PLP */
		cmd.args[0] = 0x52;
		cmd.args[1] = c->stream_id & 0xff;
		cmd.args[2] = c->stream_id == NO_STREAM_ID_FILTER ? 0 : 1;
		cmd.wlen = 3;
		cmd.rlen = 1;
		ret = si2168_cmd_execute(client, &cmd);
		if (ret)
			goto err;
	}

	memcpy(cmd.args, "\x51\x03", 2);
	cmd.wlen = 2;
	cmd.rlen = 12;
	ret = si2168_cmd_execute(client, &cmd);
	if (ret)
		goto err;

	memcpy(cmd.args, "\x12\x08\x04", 3);
	cmd.wlen = 3;
	cmd.rlen = 3;
	ret = si2168_cmd_execute(client, &cmd);
	if (ret)
		goto err;

	memcpy(cmd.args, "\x14\x00\x0c\x10\x12\x00", 6);
	cmd.wlen = 6;
	cmd.rlen = 4;
	ret = si2168_cmd_execute(client, &cmd);
	if (ret)
		goto err;

	memcpy(cmd.args, "\x14\x00\x06\x10\x24\x00", 6);
	cmd.wlen = 6;
	cmd.rlen = 4;
	ret = si2168_cmd_execute(client, &cmd);
	if (ret)
		goto err;

	memcpy(cmd.args, "\x14\x00\x07\x10\x00\x24", 6);
	cmd.wlen = 6;
	cmd.rlen = 4;
	ret = si2168_cmd_execute(client, &cmd);
	if (ret)
		goto err;

	memcpy(cmd.args, "\x14\x00\x0a\x10\x00\x00", 6);
	cmd.args[4] = delivery_system | bandwidth;
	if (dev->spectral_inversion)
		cmd.args[5] |= 1;
	cmd.wlen = 6;
	cmd.rlen = 4;
	ret = si2168_cmd_execute(client, &cmd);
	if (ret)
		goto err;

	/* set DVB-C symbol rate */
	if (c->delivery_system == SYS_DVBC_ANNEX_A) {
		memcpy(cmd.args, "\x14\x00\x02\x11", 4);
		cmd.args[4] = ((c->symbol_rate / 1000) >> 0) & 0xff;
		cmd.args[5] = ((c->symbol_rate / 1000) >> 8) & 0xff;
		cmd.wlen = 6;
		cmd.rlen = 4;
		ret = si2168_cmd_execute(client, &cmd);
		if (ret)
			goto err;
	}
	else if (c->delivery_system == SYS_DVBC_ANNEX_B) {
		memcpy(cmd.args, "\x14\x00\x02\x16", 4);
		cmd.args[4] = ((c->symbol_rate / 1000) >> 0) & 0xff;
		cmd.args[5] = ((c->symbol_rate / 1000) >> 8) & 0xff;
		cmd.wlen = 6;
		cmd.rlen = 4;
		ret = si2168_cmd_execute(client, &cmd);
		if (ret)
			goto err;
	}

	memcpy(cmd.args, "\x14\x00\x0f\x10\x10\x00", 6);
	cmd.wlen = 6;
	cmd.rlen = 4;
	ret = si2168_cmd_execute(client, &cmd);
	if (ret)
		goto err;

	memcpy(cmd.args, "\x14\x00\x09\x10\xe3\x08", 6);
	cmd.args[5] |= dev->ts_clock_inv ? 0x00 : 0x10;
	cmd.wlen = 6;
	cmd.rlen = 4;
	ret = si2168_cmd_execute(client, &cmd);
	if (ret)
		goto err;

	memcpy(cmd.args, "\x14\x00\x08\x10\xdf\x17", 6);
	cmd.args[4] = dev->ts_clock_inv ? 0xdf : 0xcf;
	cmd.args[5] = dev->ts_clock_inv ? 0x17 : 0x33;
	cmd.wlen = 6;
	cmd.rlen = 4;
	ret = si2168_cmd_execute(client, &cmd);
	if (ret)
		goto err;

	memcpy(cmd.args, "\x14\x00\x01\x12\x00\x00", 6);
	cmd.wlen = 6;
	cmd.rlen = 4;
	ret = si2168_cmd_execute(client, &cmd);
	if (ret)
		goto err;

	memcpy(cmd.args, "\x14\x00\x01\x03\x0c\x00", 6);
	cmd.wlen = 6;
	cmd.rlen = 4;
	ret = si2168_cmd_execute(client, &cmd);
	if (ret)
		goto err;

	memcpy(cmd.args, "\x85", 1);
	cmd.wlen = 1;
	cmd.rlen = 1;
	ret = si2168_cmd_execute(client, &cmd);
	if (ret)
		goto err;

	dev->delivery_system = c->delivery_system;

	/* enable ts bus */
	ret = si2168_ts_bus_ctrl(fe, 1);
	if (ret)
		goto err;

	return 0;
err:
	fprintk("failed=%d", ret);
	return ret;
}

static int si2168_search(struct dvb_frontend *fe)
{
	struct i2c_client *client = fe->demodulator_priv;
	struct si2168_dev *dev = i2c_get_clientdata(client);
	enum fe_status status = 0;
	int ret, i;

	fprintk("");

	dev->algo = SI2168_TUNE;

	/* set frontend */
	ret = si2168_set_frontend(fe);
	if (ret)
		goto error;

	/* wait frontend lock */
	for (i = 0; i < 5; i++) {
		fprintk("loop=%d", i);
		msleep(100);
		ret = si2168_read_status(fe, &status);
		if (ret)
			goto error;

		if (status & FE_HAS_LOCK)
			break;
		if (status & FE_TIMEDOUT) {
			dev->algo = SI2168_NOTUNE;
			return DVBFE_ALGO_SEARCH_FAILED;
		}
	}

	/* check if we have a valid signal */
	if (status & FE_HAS_LOCK) {
		fprintk("DVBFE_ALGO_SEARCH_SUCCESS");
		return DVBFE_ALGO_SEARCH_SUCCESS;
	} else {
		fprintk("DVBFE_ALGO_SEARCH_FAILED");
		dev->algo = SI2168_NOTUNE;
		return DVBFE_ALGO_SEARCH_FAILED;
	}

error:
	fprintk("ERROR");
	dev->algo = SI2168_NOTUNE;
	return DVBFE_ALGO_SEARCH_ERROR;
}

static int si2168_get_spectrum_scan(struct dvb_frontend *fe, struct dvb_fe_spectrum_scan *s)
{
	struct i2c_client *client = fe->demodulator_priv;
	struct si2168_dev *dev = i2c_get_clientdata(client);
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	int x, ret;
	u16 lvl;

	if (!dev->active) {
		ret = -EAGAIN;
		return ret;
	}

	p->frequency		= 0;
	p->bandwidth_hz		= 0;
	p->delivery_system	= SYS_DVBT;
	p->modulation		= QAM_16;

	*s->type = SC_DBM;

	dev->algo = SI2168_NOTUNE;

	if (fe->ops.tuner_ops.set_params) {
		for (x = 0; x < s->num_freq; x++)
		{
			p->frequency = *(s->freq + x);
			ret = fe->ops.tuner_ops.set_params(fe);

			msleep(35);

			ret = fe->ops.tuner_ops.get_rf_strength(fe, &lvl);
			*(s->rf_level + x) = (s8)lvl * 1000;
		}
	}
	return 0;
}

static int si2168_init(struct dvb_frontend *fe)
{
	struct i2c_client *client = fe->demodulator_priv;
	struct si2168_dev *dev = i2c_get_clientdata(client);
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int ret, len, remaining;
	const struct firmware *fw;
	struct si2168_cmd cmd;

	fprintk("");

	dev->algo = SI2168_NOTUNE;

	/* initialize */
	memcpy(cmd.args, "\xc0\x12\x00\x0c\x00\x0d\x16\x00\x00\x00\x00\x00\x00", 13);
	cmd.wlen = 13;
	cmd.rlen = 0;
	ret = si2168_cmd_execute(client, &cmd);
	if (ret)
		goto err;

	if (dev->warm) {
		/* resume */
		memcpy(cmd.args, "\xc0\x06\x08\x0f\x00\x20\x21\x01", 8);
		cmd.wlen = 8;
		cmd.rlen = 1;
		ret = si2168_cmd_execute(client, &cmd);
		if (ret)
			goto err;

		udelay(100);

		memcpy(cmd.args, "\x85", 1);
		cmd.wlen = 1;
		cmd.rlen = 1;
		ret = si2168_cmd_execute(client, &cmd);
		if (ret)
			goto err;

		goto warm;
	}

	/* power up */
	memcpy(cmd.args, "\xc0\x06\x01\x0f\x00\x20\x20\x01", 8);
	cmd.wlen = 8;
	cmd.rlen = 1;
	ret = si2168_cmd_execute(client, &cmd);
	if (ret)
		goto err;

	/* request the firmware, this will block and timeout */
	ret = request_firmware(&fw, dev->firmware_name, &client->dev);
	if (ret) {
		/* fallback mechanism to handle old name for Si2168 B40 fw */
		if (dev->chip_id == SI2168_CHIP_ID_B40) {
			dev->firmware_name = SI2168_B40_FIRMWARE_FALLBACK;
			ret = request_firmware(&fw, dev->firmware_name,
					       &client->dev);
		}

		if (ret == 0) {
			dev_notice(&client->dev,
					"please install firmware file '%s'\n",
					SI2168_B40_FIRMWARE);
		} else {
			dev_err(&client->dev,
					"firmware file '%s' not found\n",
					dev->firmware_name);
			goto err_release_firmware;
		}
	}

	fprintk("downloading firmware from file '%s'",
			dev->firmware_name);

	if ((fw->size % 17 == 0) && (fw->data[0] > 5)) {
		/* firmware is in the new format */
		for (remaining = fw->size; remaining > 0; remaining -= 17) {
			len = fw->data[fw->size - remaining];
			if (len > SI2168_ARGLEN) {
				ret = -EINVAL;
				break;
			}
			memcpy(cmd.args, &fw->data[(fw->size - remaining) + 1], len);
			cmd.wlen = len;
			cmd.rlen = 1;
			ret = si2168_cmd_execute(client, &cmd);
			if (ret)
				break;
		}
	} else if (fw->size % 8 == 0) {
		/* firmware is in the old format */
		for (remaining = fw->size; remaining > 0; remaining -= 8) {
			len = 8;
			memcpy(cmd.args, &fw->data[fw->size - remaining], len);
			cmd.wlen = len;
			cmd.rlen = 1;
			ret = si2168_cmd_execute(client, &cmd);
			if (ret)
				break;
		}
	} else {
		/* bad or unknown firmware format */
		ret = -EINVAL;
	}

	if (ret) {
		fprintk("firmware download failed %d", ret);
		goto err_release_firmware;
	}

	release_firmware(fw);

	memcpy(cmd.args, "\x01\x01", 2);
	cmd.wlen = 2;
	cmd.rlen = 1;
	ret = si2168_cmd_execute(client, &cmd);
	if (ret)
		goto err;

	/* query firmware version */
	memcpy(cmd.args, "\x11", 1);
	cmd.wlen = 1;
	cmd.rlen = 10;
	ret = si2168_cmd_execute(client, &cmd);
	if (ret)
		goto err;

	dev->version = (cmd.args[9] + '@') << 24 | (cmd.args[6] - '0') << 16 |
		       (cmd.args[7] - '0') << 8 | (cmd.args[8]) << 0;
	fprintk("firmware version: %c %d.%d.%d",
		 dev->version >> 24 & 0xff, dev->version >> 16 & 0xff,
		 dev->version >> 8 & 0xff, dev->version >> 0 & 0xff);

	/* TER FEF */
	memcpy(cmd.args, "\x51\x00", 2);
	cmd.wlen = 2;
	cmd.rlen = 12;
	cmd.args[1] = (dev->fef_inv & 1) << 3 | (dev->fef_pin & 7);

	ret = si2168_cmd_execute(client, &cmd);
	if (ret) {
		fprintk("err set fef pip");
	}

	/* MP DEFAULTS */
	memcpy(cmd.args, "\x88\x01\x01\x01\x01", 5);
	cmd.wlen = 5;
	cmd.rlen = 2;
	switch (dev->fef_pin)
	{
	case SI2168_MP_A:
		cmd.args[1] = dev->fef_inv ? 3 : 2;
		break;
	case SI2168_MP_B:
		cmd.args[2] = dev->fef_inv ? 3 : 2;
		break;
	case SI2168_MP_C:
		cmd.args[3] = dev->fef_inv ? 3 : 2;
		break;
	case SI2168_MP_D:
		cmd.args[4] = dev->fef_inv ? 3 : 2;
		break;
	}

	ret = si2168_cmd_execute(client, &cmd);
	if (ret) {
		fprintk("err set mp defaults");
	}

	/* AGC */
	memcpy(cmd.args, "\x89\x01\x06\x12\x00\x00", 6);
	cmd.wlen = 6;
	cmd.rlen = 3;
	cmd.args[1] |= (dev->agc_inv & 1) << 7 | (dev->agc_pin & 7) << 4;

	ret = si2168_cmd_execute(client, &cmd);
	if (ret) {
		fprintk("err set ter agc");
	}

	/* set ts mode */
	ret = si2168_ts_bus_ctrl(fe, 1);
	if (ret)
		goto err;

	dev->warm = true;
warm:
	/* Init stats here to indicate which stats are supported */
	c->cnr.len = 1;
	c->cnr.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	c->post_bit_error.len = 1;
	c->post_bit_error.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	c->post_bit_count.len = 1;
	c->post_bit_count.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	c->block_error.len = 1;
	c->block_error.stat[0].scale = FE_SCALE_NOT_AVAILABLE;

	dev->active = true;

	return 0;
err_release_firmware:
	release_firmware(fw);
err:
	fprintk("failed=%d", ret);
	return ret;
}

static int si2168_sleep(struct dvb_frontend *fe)
{
	struct i2c_client *client = fe->demodulator_priv;
	struct si2168_dev *dev = i2c_get_clientdata(client);
//	int ret;
//	struct si2168_cmd cmd;

	fprintk("DISABLED");

	dev->algo = SI2168_NOTUNE;
	dev->active = false;

	/* tri-state data bus */
//	ret = si2168_ts_bus_ctrl(fe, 0);
//	if (ret)
//		goto err;

	/* Firmware later than B 4.0-11 loses warm state during sleep */
	if (dev->version > ('B' << 24 | 4 << 16 | 0 << 8 | 11 << 0))
		dev->warm = false;

//	memcpy(cmd.args, "\x13", 1);
//	cmd.wlen = 1;
//	cmd.rlen = 0;
//	ret = si2168_cmd_execute(client, &cmd);
//	if (ret)
//		goto err;

	return 0;
//err:
//	fprintk("failed=%d", ret);
//	return ret;
}

static int si2168_get_tune_settings(struct dvb_frontend *fe,
	struct dvb_frontend_tune_settings *s)
{
	s->min_delay_ms = 900;

	return 0;
}

static int si2168_select(struct i2c_mux_core *muxc, u32 chan)
{
	struct i2c_client *client = i2c_mux_priv(muxc);
	int ret;
	struct si2168_cmd cmd;

	/* open I2C gate */
	memcpy(cmd.args, "\xc0\x0d\x01", 3);
	cmd.wlen = 3;
	cmd.rlen = 0;
	ret = si2168_cmd_execute(client, &cmd);
	if (ret)
		goto err;

	return 0;
err:
	fprintk("failed=%d", ret);
	return ret;
}

static int si2168_deselect(struct i2c_mux_core *muxc, u32 chan)
{
	struct i2c_client *client = i2c_mux_priv(muxc);
	int ret;
	struct si2168_cmd cmd;

	/* close I2C gate */
	memcpy(cmd.args, "\xc0\x0d\x00", 3);
	cmd.wlen = 3;
	cmd.rlen = 0;
	ret = si2168_cmd_execute(client, &cmd);
	if (ret)
		goto err;

	return 0;
err:
	fprintk("failed=%d", ret);
	return ret;
}

static const struct dvb_frontend_ops si2168_ops = {
	.delsys = {SYS_DVBC_ANNEX_A, SYS_DVBC_ANNEX_B, SYS_DVBT2, SYS_DVBT},
	.info = {
		.name = "Silicon Labs Si2168",
		.symbol_rate_min = 1000000,
		.symbol_rate_max = 7200000,
		.caps =	FE_CAN_FEC_1_2 |
			FE_CAN_FEC_2_3 |
			FE_CAN_FEC_3_4 |
			FE_CAN_FEC_5_6 |
			FE_CAN_FEC_7_8 |
			FE_CAN_FEC_AUTO |
			FE_CAN_QPSK |
			FE_CAN_QAM_16 |
			FE_CAN_QAM_32 |
			FE_CAN_QAM_64 |
			FE_CAN_QAM_128 |
			FE_CAN_QAM_256 |
			FE_CAN_QAM_AUTO |
			FE_CAN_TRANSMISSION_MODE_AUTO |
			FE_CAN_GUARD_INTERVAL_AUTO |
			FE_CAN_HIERARCHY_AUTO |
			FE_CAN_MUTE_TS |
			FE_CAN_2G_MODULATION |
			FE_CAN_MULTISTREAM |
			FE_HAS_EXTENDED_CAPS
	},
	.extended_info = {
		.extended_caps          = FE_CAN_SPECTRUMSCAN
	},

	.get_tune_settings = si2168_get_tune_settings,

	.init			= si2168_init,
	.sleep			= si2168_sleep,

	.set_frontend		= si2168_set_frontend,
	.get_frontend_algo	= si2168_get_frontend_algo,
	.search			= si2168_search,
	.dtv_tune		= si2168_dtv_tune,
	.get_spectrum_scan	= si2168_get_spectrum_scan,

	.read_status		= si2168_read_status,
	.read_signal_strength	= si2168_read_signal_strength,
	.read_snr		= si2168_read_snr,
	.read_ber		= si2168_read_ber,
	.read_ucblocks		= si2168_read_ucblocks,
};

static int si2168_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct si2168_config *config = client->dev.platform_data;
	struct si2168_dev *dev;
	int ret;
	struct si2168_cmd cmd;

	fprintk("");

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		ret = -ENOMEM;
		goto err;
	}

	i2c_set_clientdata(client, dev);
	mutex_init(&dev->i2c_mutex);

	/* Initialize */
	memcpy(cmd.args, "\xc0\x12\x00\x0c\x00\x0d\x16\x00\x00\x00\x00\x00\x00", 13);
	cmd.wlen = 13;
	cmd.rlen = 0;
	ret = si2168_cmd_execute(client, &cmd);
	if (ret)
		goto err_kfree;

	/* Power up */
	memcpy(cmd.args, "\xc0\x06\x01\x0f\x00\x20\x20\x01", 8);
	cmd.wlen = 8;
	cmd.rlen = 1;
	ret = si2168_cmd_execute(client, &cmd);
	if (ret)
		goto err_kfree;

	/* Query chip revision */
	memcpy(cmd.args, "\x02", 1);
	cmd.wlen = 1;
	cmd.rlen = 13;
	ret = si2168_cmd_execute(client, &cmd);
	if (ret)
		goto err_kfree;

	dev->chip_id = cmd.args[1] << 24 | cmd.args[2] << 16 |
		       cmd.args[3] << 8 | cmd.args[4] << 0;

	switch (dev->chip_id) {
	case SI2168_CHIP_ID_A20:
		dev->firmware_name = SI2168_A20_FIRMWARE;
		break;
	case SI2168_CHIP_ID_A30:
		dev->firmware_name = SI2168_A30_FIRMWARE;
		break;
	case SI2168_CHIP_ID_B40:
		dev->firmware_name = SI2168_B40_FIRMWARE;
		break;
	case SI2168_CHIP_ID_D60:
		dev->firmware_name = SI2168_D60_FIRMWARE;
		break;
	default:
		fprintk("unknown chip version Si21%d-%c%c%c",
			cmd.args[2], cmd.args[1], cmd.args[3], cmd.args[4]);
		ret = -ENODEV;
		goto err_kfree;
	}

	dev->version = (cmd.args[1]) << 24 | (cmd.args[3] - '0') << 16 |
		       (cmd.args[4] - '0') << 8 | (cmd.args[5]) << 0;

	/* create mux i2c adapter for tuner */
	dev->muxc = i2c_mux_alloc(client->adapter, &client->dev,
				  1, 0, I2C_MUX_LOCKED,
				  si2168_select, si2168_deselect);
	if (!dev->muxc) {
		ret = -ENOMEM;
		goto err_kfree;
	}
	dev->muxc->priv = client;
	ret = i2c_mux_add_adapter(dev->muxc, 0, 0, 0);
	if (ret)
		goto err_kfree;

	/* create dvb_frontend */
	memcpy(&dev->fe.ops, &si2168_ops, sizeof(struct dvb_frontend_ops));
	dev->fe.demodulator_priv = client;
	*config->i2c_adapter = dev->muxc->adapter[0];
	*config->fe = &dev->fe;
	dev->ts_mode = config->ts_mode;
	dev->ts_clock_inv = config->ts_clock_inv;
	dev->ts_clock_gapped = config->ts_clock_gapped;
	dev->fef_pin = config->fef_pin;
	dev->fef_inv = config->fef_inv;
	dev->agc_pin = config->agc_pin;
	dev->agc_inv = config->agc_inv;
	dev->spectral_inversion = config->spectral_inversion;

	if (!dev->agc_pin) dev->agc_pin = SI2168_MP_A;
	if (!dev->fef_pin) dev->fef_pin = SI2168_MP_B;

	fprintk("Silicon Labs Si2168-%c%d%d successfully identified",
		 dev->version >> 24 & 0xff, dev->version >> 16 & 0xff,
		 dev->version >> 8 & 0xff);
	fprintk("firmware version: %c %d.%d.%d",
		 dev->version >> 24 & 0xff, dev->version >> 16 & 0xff,
		 dev->version >> 8 & 0xff, dev->version >> 0 & 0xff);

	return 0;
err_kfree:
	kfree(dev);
err:
	fprintk("failed=%d", ret);
	return ret;
}

static int si2168_remove(struct i2c_client *client)
{
	struct si2168_dev *dev = i2c_get_clientdata(client);

	fprintk("");

	i2c_mux_del_adapters(dev->muxc);

	dev->fe.ops.release = NULL;
	dev->fe.demodulator_priv = NULL;

	kfree(dev);

	return 0;
}

static const struct i2c_device_id si2168_id_table[] = {
	{"si2168", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, si2168_id_table);

static struct i2c_driver si2168_driver = {
	.driver = {
		.name                = "si2168",
		.suppress_bind_attrs = true,
	},
	.probe		= si2168_probe,
	.remove		= si2168_remove,
	.id_table	= si2168_id_table,
};

module_i2c_driver(si2168_driver);

MODULE_AUTHOR("Antti Palosaari <crope@iki.fi>");
MODULE_DESCRIPTION("Silicon Labs Si2168 DVB-T/T2/C demodulator driver");
MODULE_LICENSE("GPL");
MODULE_FIRMWARE(SI2168_A20_FIRMWARE);
MODULE_FIRMWARE(SI2168_A30_FIRMWARE);
MODULE_FIRMWARE(SI2168_B40_FIRMWARE);
MODULE_FIRMWARE(SI2168_D60_FIRMWARE);
