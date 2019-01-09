/*
 * Silicon Labs Si2183(2) DVB-T/T2/C/C2/S/S2 demodulator driver
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

#include "si2183.h"
#include "media/dvb_frontend.h"
#include <linux/firmware.h>
#include <linux/i2c-mux.h>
#include <linux/delay.h>
#include <linux/math64.h>

bool si2183_debug = true;

#define SI2183_B60_FIRMWARE "dvb-demod-si2183-b60-01.fw"

#define SI2183_ARGLEN      30

struct si2183_command {
	u8 args[SI2183_ARGLEN];
	unsigned wlen;
	unsigned rlen;

	int ret;
};

static const struct dvb_frontend_ops si2183_ops;

LIST_HEAD(silist);

struct si_base {
	struct i2c_mux_core *muxc;
	struct list_head     silist;

	u8                   adr;
	struct i2c_adapter  *i2c;
	u32                  count;

	struct i2c_adapter  *tuner_adapter;
};

/* state struct */
struct si2183_dev {
	struct mutex i2c_mutex;
	struct dvb_frontend fe;
	enum fe_delivery_system delivery_system;
	enum fe_status fe_status;
	u8 stat_resp;
	u16 snr;
	bool active;
	bool fw_loaded;
	u8 ts_mode;
	bool ts_clock_inv;
	bool ts_clock_gapped;
	int start_clk_mode;
	u8 agc_mode;
	struct si_base *base;
	void (*RF_switch)(struct i2c_adapter * i2c, u8 rf_in, u8 flag);
	u8 rf_in;
	u8 active_fe;
	void (*TS_switch)(struct i2c_adapter * i2c,u8 flag);
	void (*LED_switch)(struct i2c_adapter * i2c,u8 flag);

	void (*write_properties) (struct i2c_adapter *i2c,u8 reg, u32 buf);
	void (*read_properties) (struct i2c_adapter *i2c,u8 reg, u32 *buf);
};

static const struct si2183_command SI2183_GET_REVISION		= {{0x02}, 1, 13};
static const struct si2183_command SI2183_GET_FWVER		= {{0x11}, 1, 10};
static const struct si2183_command SI2183_SLEEP			= {{0x13}, 1, 0};
static const struct si2183_command SI2183_SET_TS_MODE		= {{0x14, 0x00, 0x01, 0x10, 0x10, 0x00}, 6, 4};
static const struct si2183_command SI2183_SET_DD_IEN		= {{0x14, 0x00, 0x06, 0x10, 0x00, 0x00}, 6, 4};
static const struct si2183_command SI2183_SET_INT_SENSE		= {{0x14, 0x00, 0x07, 0x10, 0x00, 0x20}, 6, 4};
static const struct si2183_command SI2183_SET_TS_SERIAL		= {{0x14, 0x00, 0x08, 0x10, 0x0c, 0x23}, 6, 4};
static const struct si2183_command SI2183_SET_TS_PARALLEL	= {{0x14, 0x00, 0x09, 0x10, 0x04, 0x01}, 6, 4};
static const struct si2183_command SI2183_SET_MCNS_SYSTEM	= {{0x14, 0x00, 0x0a, 0x10, 0x18, 0x00}, 6, 4};
static const struct si2183_command SI2183_SET_DVBT_SYSTEM	= {{0x14, 0x00, 0x0a, 0x10, 0x20, 0x00}, 6, 4};
static const struct si2183_command SI2183_SET_DVBC_SYSTEM	= {{0x14, 0x00, 0x0a, 0x10, 0x38, 0x00}, 6, 4};
static const struct si2183_command SI2183_SET_ISDBT_SYSTEM	= {{0x14, 0x00, 0x0a, 0x10, 0x40, 0x00}, 6, 4};
static const struct si2183_command SI2183_SET_DVBT2_SYSTEM	= {{0x14, 0x00, 0x0a, 0x10, 0x70, 0x00}, 6, 4};
static const struct si2183_command SI2183_SET_DVBS_SYSTEM	= {{0x14, 0x00, 0x0a, 0x10, 0x88, 0x00}, 6, 4};
static const struct si2183_command SI2183_SET_DVBS2_SYSTEM	= {{0x14, 0x00, 0x0a, 0x10, 0x98, 0x00}, 6, 4};
static const struct si2183_command SI2183_SET_DSS_SYSTEM	= {{0x14, 0x00, 0x0a, 0x10, 0xa8, 0x00}, 6, 4};
static const struct si2183_command SI2183_SET_AUTO_SYSTEM	= {{0x14, 0x00, 0x0a, 0x10, 0xf8, 0x04}, 6, 4};
static const struct si2183_command SI2183_SET_L2_CH_SEEK	= {{0x14, 0x00, 0x0a, 0x10, 0xf8, 0x06}, 6, 4};
static const struct si2183_command SI2183_SET_FER_RESOL		= {{0x14, 0x00, 0x0c, 0x10, 0x12, 0x00}, 6, 4};
static const struct si2183_command SI2183_SET_SQI_COMPUTATION	= {{0x14, 0x00, 0x0f, 0x10, 0x1e, 0x00}, 6, 4};
static const struct si2183_command SI2183_SET_TS_PARALLEL_2	= {{0x14, 0x00, 0x15, 0x10, 0xe3, 0x08}, 6, 4};
static const struct si2183_command SI2183_SET_TS_SERIAL_2	= {{0x14, 0x00, 0x16, 0x10, 0xc7, 0x01}, 6, 4};
static const struct si2183_command SI2183_SET_DVBC_MODULATION	= {{0x14, 0x00, 0x01, 0x11, 0x00, 0x00}, 6, 4};
static const struct si2183_command SI2183_SET_DVBC_SYMBOLRATE	= {{0x14, 0x00, 0x02, 0x11, 0x00, 0x00}, 6, 4};
static const struct si2183_command SI2183_SET_DVBT_HIERARCHY	= {{0x14, 0x00, 0x01, 0x12, 0x00, 0x00}, 6, 4};
static const struct si2183_command SI2183_SET_DVBT2_MODE	= {{0x14, 0x00, 0x04, 0x13, 0x00, 0x00}, 6, 4};
static const struct si2183_command SI2183_SET_DVBS2_SYMBOLRATE	= {{0x14, 0x00, 0x01, 0x14, 0x00, 0x00}, 6, 4};
static const struct si2183_command SI2183_SET_DVBS_SYMBOLRATE	= {{0x14, 0x00, 0x01, 0x15, 0x00, 0x00}, 6, 4};
static const struct si2183_command SI2183_SET_MCNS_MODULATION	= {{0x14, 0x00, 0x01, 0x16, 0x00, 0x00}, 6, 4};
static const struct si2183_command SI2183_SET_MCNS_SYMBOLRATE	= {{0x14, 0x00, 0x02, 0x16, 0x00, 0x00}, 6, 4};
static const struct si2183_command SI2183_GET_DVBT2_PARAMETERS	= {{0x50, 0x01}, 2, 14};
static const struct si2183_command SI2183_SET_DVBT2_STREAMID	= {{0x52, 0x00, 0x00}, 3, 1};
static const struct si2183_command SI2183_GET_DVBS_PARAMETERS	= {{0x60, 0x01}, 2, 10};
static const struct si2183_command SI2183_GET_DVBS2_PARAMETERS	= {{0x70, 0x01}, 2, 13};
static const struct si2183_command SI2183_SET_DVBS2_STREAMID	= {{0x71, 0x00, 0x00}, 3, 1};
static const struct si2183_command SI2183_SET_PLSCODE		= {{0x73, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00}, 8, 1};
static const struct si2183_command SI2183_GET_BER		= {{0x82, 0x00}, 2, 3};
static const struct si2183_command SI2183_GET_UCB		= {{0x84, 0x00}, 2, 3};
static const struct si2183_command SI2183_DSP_RESET		= {{0x85}, 1, 1};
static const struct si2183_command SI2183_GET_SYSTEM		= {{0x87, 0x00}, 2, 8};
static const struct si2183_command SI2183_SET_AGC_TER		= {{0x89, 0x41, 0x06, 0x12, 0x00, 0x00}, 6, 3};
static const struct si2183_command SI2183_GET_DVBC_A_PARAMETERS	= {{0x90, 0x01}, 2, 9};
static const struct si2183_command SI2183_GET_DVBC_B_PARAMETERS	= {{0x98, 0x01}, 2, 10};
static const struct si2183_command SI2183_GET_RF_STRENGTH	= {{0x8a, 0x00, 0x00, 0x00, 0x00, 0x00}, 6, 3};
static const struct si2183_command SI2183_SET_AGC_SAT		= {{0x8a, 0x1d, 0x12, 0x00, 0x00, 0x00}, 6, 3};
static const struct si2183_command SI2183_SET_AGC_SAT2		= {{0x8a, 0x08, 0x12, 0x00, 0x00, 0x00}, 6, 3};
static const struct si2183_command SI2183_DISEQC		= {{0x8c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 8, 1};
static const struct si2183_command SI2183_GET_IQ_SAMPLE 	= {{0x8f, 0x0f, 0xf8, 0x05}, 4, 5};
static const struct si2183_command SI2183_GET_DVBT_PARAMETERS	= {{0x0a, 0x01}, 2, 13};
static const struct si2183_command SI2183_GET_ISDBT_PARAMETERS	= {{0xa4, 0x01}, 2, 14};
static const struct si2183_command SI2183_INIT			= {{0xc0, 0x12, 0x00, 0x0c, 0x00, 0x0d, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 13, 0};
static const struct si2183_command SI2183_RESUME		= {{0xc0, 0x06, 0x08, 0x0f, 0x00, 0x20, 0x21, 0x01}, 8, 1};
static const struct si2183_command SI2183_POWER_ON		= {{0xc0, 0x06, 0x01, 0x0f, 0x00, 0x20, 0x20, 0x01}, 8, 1};
static const struct si2183_command SI2183_I2C_OPEN		= {{0xc0, 0x0d, 0x01}, 3, 0};
static const struct si2183_command SI2183_I2C_CLOSE		= {{0xc0, 0x0d, 0x00}, 3, 0};

/* Own I2C adapter locking is needed because of I2C gate logic. */
static int si2183_i2c_master_send_unlocked(const struct i2c_client *client,
					   const char *buf, int count)
{
	int ret;
	struct i2c_msg msg = {
		.addr = client->addr,
		.flags = 0,
		.len = count,
		.buf = (char *)buf,
	};

	ret = __i2c_transfer(client->adapter, &msg, 1);
	return (ret == 1) ? count : ret;
}

static int si2183_i2c_master_recv_unlocked(const struct i2c_client *client,
					   char *buf, int count)
{
	int ret;
	struct i2c_msg msg = {
		.addr = client->addr,
		.flags = I2C_M_RD,
		.len = count,
		.buf = buf,
	};

	ret = __i2c_transfer(client->adapter, &msg, 1);
	return (ret == 1) ? count : ret;
}

/* execute firmware command */
static int si2183_cmd_execute_unlocked(struct i2c_client *client,
				       struct si2183_command *cmd)
{
	struct si2183_command o_cmd = *cmd;
	int ret;
	unsigned long timeout;

	if (cmd->wlen) {
		if (si2183_debug)
			fprintk("W: %*ph", cmd->wlen, cmd->args);
		/* write cmd and args for firmware */
		ret = si2183_i2c_master_send_unlocked(client, cmd->args, cmd->wlen);
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
			ret = si2183_i2c_master_recv_unlocked(client, cmd->args, cmd->rlen);
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
		if (si2183_debug)
			fprintk("R: %*ph", cmd->rlen, cmd->args);
	}

	return 0;
w_err:
	fprintk("W: failed=%d", ret);
	fprintk("Write: failed=%d", ret);
	fprintk("W: %*ph", o_cmd.wlen, o_cmd.args);
	return ret;
r_err:
	fprintk("Read: failed=%d", ret);
	fprintk("W: %*ph", o_cmd.wlen, o_cmd.args);
	fprintk("R: %*ph", cmd->rlen, cmd->args);
	return ret;
}

static struct si2183_command si2183_cmd(struct i2c_client *client, char *command, u8 wlen, u8 rlen)
{
	struct si2183_dev *dev = i2c_get_clientdata(client);
	struct si2183_command cmd;

	memcpy(cmd.args, command, wlen);
	cmd.wlen = wlen;
	cmd.rlen = rlen;

	mutex_lock(&dev->i2c_mutex);

	cmd.ret = si2183_cmd_execute_unlocked(client, &cmd);

	mutex_unlock(&dev->i2c_mutex);

	return cmd;
}

static struct si2183_command si2183_CMD(struct i2c_client *client, struct si2183_command cmd)
{
	struct si2183_dev *dev = i2c_get_clientdata(client);

	mutex_lock(&dev->i2c_mutex);

	cmd.ret = si2183_cmd_execute_unlocked(client, &cmd);

	mutex_unlock(&dev->i2c_mutex);

	return cmd;
}

static int si2183_read_signal_strength(struct dvb_frontend *fe, u16 *strength)
{
	struct i2c_client *client = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	struct si2183_command cmd;

	if (fe->ops.tuner_ops.get_rf_strength)
	{
		cmd = si2183_CMD(client, SI2183_GET_RF_STRENGTH);
		if (cmd.ret) {
			fprintk("err getting RF strength");
		}

		*strength = cmd.args[1];
		fe->ops.tuner_ops.get_rf_strength(fe, strength);
	}

	*strength = c->strength.stat[0].scale == FE_SCALE_DECIBEL ? ((100000 + (s32)c->strength.stat[0].svalue) / 1000) * 656 : 0;

	return 0;
}

static int si2183_read_status(struct dvb_frontend *fe, enum fe_status *status)
{
	struct i2c_client *client = fe->demodulator_priv;
	struct si2183_config *config = client->dev.platform_data;
	struct si2183_dev *dev = i2c_get_clientdata(client);
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	struct si2183_command cmd;
	u16 agc;

	*status = 0;

	if (!dev->active) {
		return -EAGAIN;
	}

	cmd = si2183_CMD(client, SI2183_GET_SYSTEM);
	switch (cmd.args[3] & 0x0f) {
	case 0x08:
		c->delivery_system = SYS_DVBS;
		break;
	case 0x09:
		c->delivery_system = SYS_DVBS2;
		break;
	default:
		fprintk("delivery system not found");
		break;
	}

	switch (c->delivery_system) {
	case SYS_DVBT:
		cmd = si2183_CMD(client, SI2183_GET_DVBT_PARAMETERS);
		break;
	case SYS_DVBT2:
		cmd = si2183_CMD(client, SI2183_GET_DVBT2_PARAMETERS);
		break;
	case SYS_DVBC_ANNEX_A:
		cmd = si2183_CMD(client, SI2183_GET_DVBC_A_PARAMETERS);
		break;
	case SYS_DVBC_ANNEX_B:
		cmd = si2183_CMD(client, SI2183_GET_DVBC_B_PARAMETERS);
		break;
	case SYS_DVBS:
		cmd = si2183_CMD(client, SI2183_GET_DVBS_PARAMETERS);
		c->modulation = QPSK;
		c->rolloff = ROLLOFF_35;
		switch (cmd.args[9] & 0x0f) {
		case 0x01:
			c->fec_inner = FEC_1_2;
			break;
		case 0x02:
			c->fec_inner = FEC_2_3;
			break;
		case 0x03:
			c->fec_inner = FEC_3_4;
			break;
		case 0x05:
			c->fec_inner = FEC_5_6;
			break;
		case 0x07:
			c->fec_inner = FEC_7_8;
			break;
		default:
			fprintk("Unknown FEC");
			break;
		}
		break;
	case SYS_DVBS2:
		cmd = si2183_CMD(client, SI2183_GET_DVBS2_PARAMETERS);
		switch ((cmd.args[10] & 0x03)) {
		case 0:
			c->rolloff = ROLLOFF_35;
			break;
		case 1:
			c->rolloff = ROLLOFF_25;
			break;
		case 2:
			c->rolloff = ROLLOFF_20;
			break;
/* fixme - should we ever come across alternating RO bit pattern of 11,
   would be a sign we are on an S2x signal and the receiver
   should switch to the LO RO, so we should be able to indicate that. */
//		case 0:
//			c->rolloff = ROLLOFF_15;
//			break;
//		case 1:
//			c->rolloff = ROLLOFF_10;
//			break;
//		case 2:
//			c->rolloff = ROLLOFF_05;
//			break;
		default:
			fprintk("Unknown rolloff");
			break;
		}
		switch ((cmd.args[8] >> 7)) {
		case 0:
			c->pilot = PILOT_ON;
			break;
		case 1:
			c->pilot = PILOT_OFF;
			break;
		}
		switch ((cmd.args[8] >> 6) & 0x01) {
		case 0:
			c->inversion = INVERSION_ON;
			break;
		case 1:
			c->inversion = INVERSION_OFF;
			break;
		}
		switch (cmd.args[8] & 0x0f) {
		case 0x03:
			c->modulation = QPSK;
			break;
		case 0x0e:
			c->modulation = PSK_8;
			break;
		case 0x04:
			c->modulation = APSK_16;
			break;
		case 0x05:
			c->modulation = APSK_32;
			break;
		default:
			fprintk("Unkown modulation");
			break;
		}
		switch (cmd.args[9]) {
		case 0x01:
			c->fec_inner = FEC_1_2;
			break;
		case 0x02:
			c->fec_inner = FEC_2_3;
			break;
		case 0x03:
			c->fec_inner = FEC_3_4;
			break;
		case 0x04:
			c->fec_inner = FEC_4_5;
			break;
		case 0x05:
			c->fec_inner = FEC_5_6;
			break;
		case 0x07:
			c->fec_inner = FEC_7_8;
			break;
		case 0x08:
			c->fec_inner = FEC_8_9;
			break;
		case 0x09:
			c->fec_inner = FEC_9_10;
			break;
		case 0x0a:
			c->fec_inner = FEC_1_3;
			break;
		case 0x0b:
			c->fec_inner = FEC_1_4;
			break;
		case 0x0c:
			c->fec_inner = FEC_2_5;
			break;
		case 0x0d:
			c->fec_inner = FEC_3_5;
			break;
		default:
			fprintk("Unkown FEC");
			break;
		}
		break;
	default:
		break;
	}

	dev->stat_resp = cmd.args[2];
	switch ((dev->stat_resp >> 1) & 0x03) {
	case 0x01:
		*status = FE_HAS_SIGNAL | FE_HAS_CARRIER;
		break;
	case 0x03:
		*status = FE_HAS_SIGNAL | FE_HAS_CARRIER | FE_HAS_VITERBI |
				FE_HAS_SYNC | FE_HAS_LOCK;
		break;
	default:
		break;
	}

	if (config->algo == SI2183_FAILED) {
		*status |= FE_TIMEDOUT;
	}

	c->cnr.len = 1;
	c->cnr.stat[0].scale = FE_SCALE_DECIBEL;
	c->cnr.stat[0].svalue = (s64)cmd.args[3] * 250;
	c->strength.len = 1;
	c->strength.stat[0].scale = FE_SCALE_DECIBEL;
	c->strength.stat[0].svalue = (s64)cmd.args[6] * 250;

	dev->fe_status = *status;

	si2183_read_signal_strength(fe, &agc);

	return 0;
}

static int si2183_read_snr(struct dvb_frontend *fe, u16 *snr)
{
	return 0;
}

static int si2183_read_ber(struct dvb_frontend *fe, u32 *ber)
{
	struct i2c_client *client = fe->demodulator_priv;
	struct si2183_dev *dev = i2c_get_clientdata(client);
	struct si2183_command cmd;

	if (dev->fe_status & FE_HAS_LOCK) {
		cmd = si2183_CMD(client, SI2183_GET_BER);
		*ber = (u32)cmd.args[2] * cmd.args[1] & 0xf;
	} else
		*ber = 1;

	return 0;
}

static int si2183_read_ucblocks(struct dvb_frontend *fe, u32 *ucblocks)
{
	struct i2c_client *client = fe->demodulator_priv;
	struct si2183_dev *dev = i2c_get_clientdata(client);
	struct si2183_command cmd;

	if (dev->stat_resp & 0x10) {
		cmd = si2183_CMD(client, SI2183_GET_UCB);
		*ucblocks = (u16)cmd.args[2] << 8 | cmd.args[1];
	} else
		*ucblocks = 0;

	return 0;
}


static int si2183_set_dvbc(struct dvb_frontend *fe)
{
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	struct i2c_client *client = fe->demodulator_priv;
	struct si2183_command cmd;

	fprintk("");

	si2183_CMD(client, SI2183_SET_AGC_TER);

	si2183_CMD(client, SI2183_SET_DVBC_SYSTEM);

	cmd = SI2183_SET_DVBC_MODULATION;
	switch (c->modulation) {
	default:
	case QAM_AUTO:
		cmd.args[4] = 0x00;
		break;
	case QAM_16:
		cmd.args[4] = 0x07;
		break;
	case QAM_32:
		cmd.args[4] = 0x08;
		break;
	case QAM_64:
		cmd.args[4] = 0x09;
		break;
	case QAM_128:
		cmd.args[4] = 0x0a;
		break;
	case QAM_256:
		cmd.args[4] = 0x0b;
		break;
	}
	si2183_CMD(client, cmd);

	cmd = SI2183_SET_DVBC_SYMBOLRATE;
	cmd.args[4] = (c->symbol_rate / 1000) & 0xff;
	cmd.args[5] = (c->symbol_rate / 1000) >> 8;
	si2183_CMD(client, cmd);

	return 0;
}

static int si2183_set_mcns(struct dvb_frontend *fe)
{
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	struct i2c_client *client = fe->demodulator_priv;
	struct si2183_command cmd_sr, cmd_mod;

	fprintk("");

	si2183_CMD(client, SI2183_SET_AGC_TER);

	si2183_CMD(client, SI2183_SET_MCNS_SYSTEM);

	cmd_sr  = SI2183_SET_MCNS_SYMBOLRATE;
	cmd_mod = SI2183_SET_MCNS_MODULATION;
	switch (c->modulation) {
	default:
	case QAM_64:
		cmd_sr.args[4] = 0xdf; // 5087
		cmd_sr.args[5] = 0x13;
		cmd_mod.args[4] = 0x09;
		break;
	case QAM_256:
		cmd_sr.args[4] = 0xf1; // 5361
		cmd_sr.args[5] = 0x14;
		cmd_mod.args[4] = 0x0b;
		break;
	}
	si2183_CMD(client, cmd_sr);
	si2183_CMD(client, cmd_mod);

	return 0;
}

static int gold_code_index (int gold_sequence_index)
{
	unsigned int i, k , x_init;
	u8 GOLD_PRBS[19] = {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	for (k=0; k<gold_sequence_index; k++) {
		GOLD_PRBS[18] = (GOLD_PRBS[0] + GOLD_PRBS[7])%2;
		/* Shifting 18 first values */
		for (i=0; i<18; i++)
			GOLD_PRBS[i] = GOLD_PRBS[i+1];
	}
	x_init = 0;
	for (i=0; i<18; i++) { x_init = x_init + GOLD_PRBS[i]*(1<<i); }

	return x_init;
}

static int si2183_set_dvbs(struct dvb_frontend *fe)
{
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	struct i2c_client *client = fe->demodulator_priv;
	struct si2183_command cmd;
	u32 pls_mode, pls_code;

	fprintk("");

	si2183_CMD(client, SI2183_SET_AGC_SAT);

	switch (c->delivery_system) {
	default:
	case SYS_DVBS:
		cmd = SI2183_SET_DVBS_SYSTEM;
		break;
	case SYS_DVBS2:
		cmd = SI2183_SET_DVBS2_SYSTEM;
		break;
	case SYS_DSS:
		cmd = SI2183_SET_DSS_SYSTEM;
		break;
	}
	si2183_CMD(client, cmd);

	switch (c->delivery_system) {
	default:
	case SYS_DSS:
	case SYS_DVBS:
		cmd = SI2183_SET_DVBS_SYMBOLRATE;
		break;
	case SYS_DVBS2:
		cmd = SI2183_SET_DVBS2_SYMBOLRATE;
	}
	cmd.args[4] = (c->symbol_rate / 1000) & 0xff;
	cmd.args[5] = (c->symbol_rate / 1000) >> 8;
	si2183_CMD(client, cmd);

	if (c->delivery_system == SYS_DVBS2) {
		cmd = SI2183_SET_DVBS2_STREAMID;
		cmd.args[1] = c->stream_id && 0xff;
		cmd.args[2] = c->stream_id == NO_STREAM_ID_FILTER ? 0 : 1;
		si2183_CMD(client, cmd);

		pls_mode = c->stream_id == NO_STREAM_ID_FILTER ? 0 : (c->stream_id >> 26) & 3;
		pls_code = c->stream_id == NO_STREAM_ID_FILTER ? 0 : (c->stream_id >> 8) & 0x3FFFF;
		if (pls_mode)
			pls_code = gold_code_index(pls_code);
		cmd = SI2183_SET_PLSCODE;
		cmd.args[1] = pls_code > 0;
		cmd.args[2] = cmd.args[3] = 0;
		cmd.args[4] = (u8) pls_code;
		cmd.args[5] = (u8) (pls_code >> 8);
		cmd.args[6] = (u8) (pls_code >> 16);
		cmd.args[7] = (u8) (pls_code >> 24);
		si2183_CMD(client, cmd);
	}

	return 0;
}

static int si2183_set_dvbt(struct dvb_frontend *fe)
{
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	struct i2c_client *client = fe->demodulator_priv;
	struct si2183_command cmd;

	fprintk("");

	si2183_CMD(client, SI2183_SET_AGC_TER);

	switch (c->delivery_system) {
	default:
	case SYS_DVBT:
		cmd = SI2183_SET_DVBT_SYSTEM;
		break;
	case SYS_DVBT2:
		cmd = SI2183_SET_DVBT2_SYSTEM;
		break;
	}

	if (c->bandwidth_hz == 0)
		return -EINVAL;
	else if (c->bandwidth_hz <= 2000000)
		cmd.args[4] |= 0x02;
	else if (c->bandwidth_hz <= 5000000)
		cmd.args[4] |= 0x05;
	else if (c->bandwidth_hz <= 6000000)
		cmd.args[4] |= 0x06;
	else if (c->bandwidth_hz <= 7000000)
		cmd.args[4] |= 0x07;
	else if (c->bandwidth_hz <= 8000000)
		cmd.args[4] |= 0x08;
	else if (c->bandwidth_hz <= 9000000)
		cmd.args[4] |= 0x09;
	else if (c->bandwidth_hz <= 10000000)
		cmd.args[4] |= 0x0a;
	else
		cmd.args[4] |= 0x0f;
	si2183_CMD(client, cmd);

	/* hierarchy - HP = 0 / LP = 1 */
	cmd = SI2183_SET_DVBT_HIERARCHY;
	cmd.args[4] = c->hierarchy == HIERARCHY_1 ? 1 : 0;
	si2183_CMD(client, cmd);

	if (c->delivery_system == SYS_DVBT2) {
		cmd = SI2183_SET_DVBT2_STREAMID;
		cmd.args[1] = c->stream_id & 0xff;
		cmd.args[2] = c->stream_id == NO_STREAM_ID_FILTER ? 0 : 1;
		si2183_CMD(client, cmd);

		cmd = SI2183_SET_DVBT2_MODE;
		cmd.args[4] = 0x00; // 0x00 = Any, 0x01 = Base, 0x02 = Lite
		si2183_CMD(client, cmd);
	}

	return 0;
}

static int si2183_set_isdbt(struct dvb_frontend *fe)
{
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	struct i2c_client *client = fe->demodulator_priv;
	struct si2183_command cmd;

	fprintk("");

	si2183_CMD(client, SI2183_SET_AGC_TER);

	cmd = SI2183_SET_ISDBT_SYSTEM;
	if (c->bandwidth_hz == 0)
		return -EINVAL;
	else if (c->bandwidth_hz <= 2000000)
		cmd.args[4] |= 0x02;
	else if (c->bandwidth_hz <= 5000000)
		cmd.args[4] |= 0x05;
	else if (c->bandwidth_hz <= 6000000)
		cmd.args[4] |= 0x06;
	else if (c->bandwidth_hz <= 7000000)
		cmd.args[4] |= 0x07;
	else if (c->bandwidth_hz <= 8000000)
		cmd.args[4] |= 0x08;
	else if (c->bandwidth_hz <= 9000000)
		cmd.args[4] |= 0x09;
	else if (c->bandwidth_hz <= 10000000)
		cmd.args[4] |= 0x0a;
	else
		cmd.args[4] |= 0x0f;

	si2183_CMD(client, cmd);

	return 0;
}

static int si2183_set_frontend(struct dvb_frontend *fe)
{
	struct i2c_client *client = fe->demodulator_priv;
	struct si2183_dev *dev = i2c_get_clientdata(client);
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int ret;

	fprintk("");

	if (!dev->active) {
		ret = -EAGAIN;
		goto err;
	}

	if (dev->RF_switch) {
		switch (c->delivery_system) {
		case SYS_DVBT:
		case SYS_DVBT2:
		case SYS_DVBC_ANNEX_A:
		case SYS_DVBC_ANNEX_B:
		case SYS_ISDBT:
			dev->RF_switch(dev->base->i2c, dev->rf_in, 1);
			break;
		case SYS_DVBS:
		case SYS_DVBS2:
		case SYS_DSS:
		default:
			dev->RF_switch(dev->base->i2c, dev->rf_in, 0);
			break;

		}
	}

	if (fe->ops.tuner_ops.set_params) {
		ret = fe->ops.tuner_ops.set_params(fe);
		if (ret) {
			fprintk("err setting tuner params");
			goto err;
		}
	}

	switch (c->delivery_system) {
	case SYS_DVBT:
	case SYS_DVBT2:
		ret = si2183_set_dvbt(fe);
		break;
	case SYS_DVBC_ANNEX_A:
		ret = si2183_set_dvbc(fe);
		break;
	case SYS_DVBC_ANNEX_B:
		ret= si2183_set_mcns(fe);
		break;
	case SYS_ISDBT:
		ret = si2183_set_isdbt(fe);
		break;
	case SYS_DVBS:
	case SYS_DVBS2:
	case SYS_DSS:
		ret = si2183_set_dvbs(fe);
		break;
	default:
		ret = -EINVAL;
		goto err;
	}

	/* dsp restart */
	si2183_CMD(client, SI2183_DSP_RESET);

	dev->delivery_system = c->delivery_system;
	return 0;
err:
	fprintk("set_params failed=%d", ret);
	return ret;
}

static int si2183_init(struct dvb_frontend *fe)
{
	struct i2c_client *client = fe->demodulator_priv;
	struct si2183_dev *dev = i2c_get_clientdata(client);
	struct si2183_config *config = client->dev.platform_data;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int ret = 0, len, remaining;
	const struct firmware *fw;
	const char *fw_name;
	struct si2183_command cmd;
	unsigned int chip_id;

	fprintk("");

	config->algo = SI2183_NOTUNE;
	config->warm = false;

	if (dev->active_fe) {
		dev->active_fe |= (1 << fe->id);
		return 0;
	}

	c->cnr.len = 1;
	c->cnr.stat[0].scale = FE_SCALE_DECIBEL;

	cmd = si2183_CMD(client, SI2183_INIT);

	if(dev->start_clk_mode == 1){
	   cmd.args[3] = 0;
	   cmd.args[5] = 0x6;
	}

	cmd.wlen = 13;
	cmd.rlen = 0;
	cmd = si2183_CMD(client, cmd);
	if (cmd.ret)
		goto err;

	if (dev->fw_loaded) {
		cmd = SI2183_RESUME;

		if (dev->start_clk_mode == 1)
				cmd.args[6] = 0x31;	

		cmd = si2183_CMD(client, cmd);
		if (cmd.ret)
			goto err;

		udelay(100);

		cmd = si2183_CMD(client, SI2183_DSP_RESET);
		if (cmd.ret)
			goto err;

		goto warm;
	}

	cmd = si2183_CMD(client, SI2183_POWER_ON);
	if (cmd.ret)
		goto err;

	cmd = si2183_CMD(client, SI2183_GET_REVISION);
	if (cmd.ret)
		goto err;

	chip_id = cmd.args[1] << 24 | cmd.args[2] << 16 | cmd.args[3] << 8 |
			cmd.args[4] << 0;

	#define SI2183_B60 ('B' << 24 | 83 << 16 | '6' << 8 | '0' << 0)

	switch (chip_id) {
	case SI2183_B60:
		fw_name = SI2183_B60_FIRMWARE;
		break;
	default:
		fprintk("unknown chip version Si21%d-%c%c%c",
				cmd.args[2], cmd.args[1],
				cmd.args[3], cmd.args[4]);
		ret = -EINVAL;
		goto err;
	}

	fprintk("found a 'Silicon Labs Si21%d-%c%c%c'",
			cmd.args[2], cmd.args[1], cmd.args[3], cmd.args[4]);

	ret = request_firmware(&fw, fw_name, &client->dev);
	if (ret) {
		fprintk("firmware file '%s' not found", fw_name);
		goto err;
	}

	fprintk("downloading firmware from file '%s'",
			fw_name);

	for (remaining = fw->size; remaining > 0; remaining -= 17) {
		len = fw->data[fw->size - remaining];
		if (len > SI2183_ARGLEN) {
			ret = -EINVAL;
			break;
		}

		memcpy(cmd.args, &fw->data[(fw->size - remaining) + 1], len);
		cmd = si2183_cmd(client, cmd.args, len, 1);
		if (cmd.ret)
			break;
	}
	release_firmware(fw);

	if (cmd.ret) {
		fprintk("firmware download failed %d", ret);
		goto err;
	}

	cmd = si2183_cmd(client, "\x01\x01", 2, 1);
	if (cmd.ret)
		goto err;

	cmd = si2183_CMD(client, SI2183_GET_FWVER);
	if (cmd.ret)
		goto err;

	fprintk("firmware version: %c.%c.%d",
			cmd.args[6], cmd.args[7], cmd.args[8]);

	cmd = SI2183_SET_TS_MODE;
	cmd.args[4] |= dev->ts_mode | (dev->ts_clock_gapped ? 0x40 : 0);
	si2183_CMD(client, cmd);

	si2183_CMD(client, SI2183_SET_FER_RESOL);

	si2183_CMD(client, SI2183_SET_DD_IEN);

	si2183_CMD(client, SI2183_SET_INT_SENSE);

	si2183_CMD(client, SI2183_SET_SQI_COMPUTATION);

	cmd = SI2183_SET_TS_PARALLEL;
	cmd.args[5] |= (dev->ts_clock_inv ? 0x00 : 0x10);
	si2183_CMD(client, cmd);

	cmd = SI2183_SET_TS_SERIAL;
	cmd.args[5] |= (dev->ts_clock_inv ? 0x00 : 0x10);
	si2183_CMD(client, cmd);

	si2183_CMD(client, SI2183_SET_TS_PARALLEL_2);

	si2183_CMD(client, SI2183_SET_TS_SERIAL_2);

	dev->fw_loaded = true;
warm:
	dev->active = true;
	dev->active_fe |= (1 << fe->id);
	return 0;

err:
	fprintk("init failed=%d", ret);
	return ret;
}

static int si2183_sleep(struct dvb_frontend *fe)
{
	struct i2c_client *client = fe->demodulator_priv;
	struct si2183_dev *dev = i2c_get_clientdata(client);
	struct si2183_config *config = client->dev.platform_data;
	struct si2183_command cmd;

	fprintk("");

	config->algo = SI2183_NOTUNE;

	dev->active_fe &= ~(1 << fe->id);
	if (dev->active_fe)
		return 0;

	dev->active = false;

	cmd = si2183_CMD(client, SI2183_SLEEP);
	if (cmd.ret)
		return cmd.ret;

	return 0;
}

static int si2183_get_tune_settings(struct dvb_frontend *fe,
	struct dvb_frontend_tune_settings *s)
{
	s->min_delay_ms = 900;

	return 0;
}

static int si2183_select(struct i2c_mux_core *muxc, u32 chan)
{
	struct i2c_client *client = i2c_mux_priv(muxc);
	struct si2183_command cmd;
	bool debug_state = si2183_debug;

	// minimize noise
	si2183_debug = false;
	cmd = si2183_CMD(client, SI2183_I2C_OPEN);

	si2183_debug = debug_state;
	return cmd.ret;
}

static int si2183_deselect(struct i2c_mux_core *muxc, u32 chan)
{
	struct i2c_client *client = i2c_mux_priv(muxc);
	struct si2183_command cmd;
	bool debug_state = si2183_debug;

	// minimize noise
	si2183_debug = false;
	cmd = si2183_CMD(client, SI2183_I2C_CLOSE);

	si2183_debug = debug_state;
	return cmd.ret;
}

static int si2183_tune(struct dvb_frontend *fe, bool re_tune,
	unsigned int mode_flags, unsigned int *delay, enum fe_status *status)
{
	int ret;

	fprintk("");

	*delay = HZ / 5;
	if (re_tune) {
		ret = si2183_set_frontend(fe);
		if (ret)
			return ret;
	}
	return si2183_read_status(fe, status);
}

static int si2183_get_frontend_algo(struct dvb_frontend *fe)
{
	struct i2c_client *client = fe->demodulator_priv;
	struct si2183_config *config = client->dev.platform_data;

	if (config->algo == SI2183_TUNE) {
		return DVBFE_ALGO_CUSTOM;
	} else {
		return DVBFE_ALGO_NOTUNE;
	}
}

static int si2183_dtv_tune(struct dvb_frontend *fe)
{
	struct i2c_client *client = fe->demodulator_priv;
	struct si2183_config *config = client->dev.platform_data;

	fprintk("");

	config->algo = SI2183_TUNE;

	return 0;
}

static int si2183_search(struct dvb_frontend *fe)
{
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	struct i2c_client *client = fe->demodulator_priv;
	struct si2183_config *config = client->dev.platform_data;
	struct si2183_dev *dev = i2c_get_clientdata(client);
	struct si2183_command cmd;
	int ret, i;
	int max_sr = 65000000;
	int min_sr = 100000;
	int try_sr;
	int locked = 0;
	u8 tmp[20];
	enum fe_status status = 0;

	if (config->algo != SI2183_TUNE)
		return DVBFE_ALGO_SEARCH_FAILED;
	config->algo = SI2183_NOTUNE; // Prevents retuning while searching

	fprintk("");

	if (dev->RF_switch)
	{
		switch (c->delivery_system) {
		case SYS_DVBT:
		case SYS_DVBT2:
		case SYS_DVBC_ANNEX_A:
		case SYS_DVBC_ANNEX_B:
		case SYS_ISDBT:
			dev->RF_switch(dev->base->i2c, dev->rf_in, 1);
			break;
		case SYS_DVBS:
		case SYS_DVBS2:
		case SYS_DSS:
		default:
			dev->RF_switch(dev->base->i2c, dev->rf_in, 0);
			break;

		}
	}
    /* initialize symbol rate in demod to 18123 */
	cmd = SI2183_SET_DVBS2_SYMBOLRATE;
	cmd.args[4] = 0x46;
	cmd.args[5] = 0xCB;
	si2183_CMD(client, cmd);

	try_sr = max_sr;

	do {
	fprintk("si2183_search(): try_sr = %d, max_sr = %d", try_sr, max_sr);
	c->symbol_rate = try_sr;

	if (fe->ops.tuner_ops.set_params) {
		ret = fe->ops.tuner_ops.set_params(fe);
	}

	msleep(200);

	fprintk("Blindscan");

	si2183_CMD(client, SI2183_GET_SYSTEM);

	si2183_CMD(client, SI2183_SET_AGC_SAT2);

	si2183_cmd(client, "\x8b\xd3", 2, 3);

	si2183_CMD(client, SI2183_GET_SYSTEM);

	si2183_CMD(client, SI2183_SET_AGC_SAT2);

	si2183_cmd(client, "\x8b\xd3", 2, 3);

	cmd = SI2183_SET_DVBS2_STREAMID;
	cmd.args[1] = 0xff;
	si2183_CMD(client, cmd);

	si2183_CMD(client, SI2183_SET_PLSCODE);

	si2183_cmd(client, "\x14\x00\x0a\x10\xa8\x04", 6, 4);

	si2183_CMD(client, SI2183_DSP_RESET);

	si2183_cmd(client, "\x31\x03\x00\x00\x00\x00\x00\x00", 8, 1);
	si2183_CMD(client, SI2183_SET_DVBS2_STREAMID);

	si2183_cmd(client, "\x14\x00\x03\x03\xc6\x3f", 6, 4);

	si2183_cmd(client, "\x14\x00\x04\x03\x90\x41", 6, 4);

	if (!config->warm) {
		si2183_cmd(client, "\x14\x00\x05\x03\xE8\x03", 6, 4);
	}

	if (!config->warm) {
		si2183_cmd(client, "\x14\x00\x06\x03\x50\xC3", 6, 4);
	}

	si2183_cmd(client, "\x14\x00\x08\x03\x03\x00", 6, 4);

	si2183_CMD(client, SI2183_SET_L2_CH_SEEK);

	if (!config->warm) {
		si2183_cmd(client, "\x14\x00\x02\x03\x01\x03", 6, 4);
	}

	config->warm = true;

	/* dsp restart */
	si2183_CMD(client, SI2183_DSP_RESET);

	si2183_cmd(client, "\x30\x00", 2, 11);
	si2183_cmd(client, "\x30\x00", 2, 11);
	si2183_cmd(client, "\x31\x01\x00\x00\xB0\x53\x10\x00", 8, 1);

	msleep(200);

	for (i = 0; i < 10; i++) {
		fprintk("1: loop=%d", i);

		cmd = si2183_cmd(client, "\x30\x01", 2, 11);
		if (cmd.args[1] == 0x02 || cmd.args[1] == 0x00)
			break;

		msleep(200);
	}
	if (i == 10) {
		fprintk("DVBFE_ALGO_SEARCH_FAILED");
		config->algo = SI2183_FAILED;
	//	return DVBFE_ALGO_SEARCH_FAILED;
	}

	msleep(50);

	tmp[4] = cmd.args[4];
	tmp[5] = cmd.args[5];
	tmp[6] = cmd.args[6];

	memcpy(cmd.args, "\x31\x02\x00\x00\x00\x00\x00\x00", 8);
	cmd.args[4] = tmp[4];
	cmd.args[5] = tmp[5];
	cmd.args[6] = tmp[6];
	si2183_cmd(client, cmd.args, 8, 1);

	msleep(100);

	for (i = 0; i < 10; i++) {
		fprintk("2: loop=%d", i);

		cmd = si2183_cmd(client, "\x30\x01", 2, 11);
		if (cmd.args[1] == 0x00 && (cmd.args[10] >> 1) == 0x04)
			break;

		msleep(200);
	}
	if (i == 10) {
		fprintk("DVBFE_ALGO_SEARCH_FAILED");
		config->algo = SI2183_FAILED;
	//	return DVBFE_ALGO_SEARCH_FAILED;
	}

	fprintk("Fine Tune");

	tmp[8] = cmd.args[8];
	tmp[9] = cmd.args[9];

	si2183_cmd(client, "\x14\x00\x08\x03\x00\x00", 6, 4);

	si2183_CMD(client, SI2183_SET_DVBS2_STREAMID);

	c->symbol_rate = (tmp[9] << 8) | tmp[8];
	c->symbol_rate *= 1000;

	if (fe->ops.tuner_ops.set_bandwidth) {
		ret = fe->ops.tuner_ops.set_bandwidth(fe, c->symbol_rate + (((c->symbol_rate * 10) * 35) / 1000));
	}

	cmd = SI2183_SET_DVBS2_SYMBOLRATE;
	cmd.args[4] = tmp[8];
	cmd.args[5] = tmp[9];
	si2183_CMD(client, cmd);

	/* Si2183_L1_SetProperty: Setting Property 0x100a to 0x04f8(1272) */
	si2183_CMD(client, SI2183_SET_AUTO_SYSTEM);

	/* dsp restart */
	si2183_CMD(client, SI2183_DSP_RESET);

	for (i = 0; i < 10; i++) {
		fprintk("3: loop=%d", i);

		cmd = si2183_cmd(client, "\x87\x01", 2, 8);
		if (cmd.args[2] == 0x0e && cmd.args[4] != 0x00 && cmd.args[5] != 0x00)
			break;

		msleep(100);
	}
	if (i == 10) {
		fprintk("DVBFE_ALGO_SEARCH_FAILED");
		config->algo = SI2183_FAILED;
	//	return DVBFE_ALGO_SEARCH_FAILED;
	}

	for (i = 0; i < 10; i++) {
		fprintk("4: loop=%d", i);

		ret = si2183_read_status(fe, &status);
		if (status & FE_HAS_LOCK) {
			locked = 1;
			break;

		msleep(300);
		}
	}
	fprintk("locked = %d", locked);
	if (try_sr > 5000000) {
		try_sr -= 20000000;
	} else {
		try_sr -= 2000000;
	}
	} while ((try_sr > min_sr) && (!locked));

	/* check if we have a valid signal */
	if (status & FE_HAS_LOCK) {
		fprintk("DVBFE_ALGO_SEARCH_SUCCESS");
		return DVBFE_ALGO_SEARCH_SUCCESS;
	} else {
		fprintk("DVBFE_ALGO_SEARCH_FAILED");
		config->algo = SI2183_FAILED;
		return DVBFE_ALGO_SEARCH_FAILED;
	}
}

static int si2183_get_spectrum_scan(struct dvb_frontend *fe, struct dvb_fe_spectrum_scan *s)
{
	struct i2c_client *client = fe->demodulator_priv;
	struct si2183_config *config = client->dev.platform_data;
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	struct si2183_command cmd;
	int x, ret;
	u16 strength = 0;
	bool debug_state = si2183_debug;

	fprintk("");

	config->algo = SI2183_NOTUNE;
	config->warm = false;
	*s->type = SC_DBM;

	if (!fe->ops.tuner_ops.set_params || !fe->ops.tuner_ops.get_rf_strength) {
		fprintk("tuner does not support set_params() or get_rf_strength()");
		return 1;
	}

	if (fe->id == 0) { // Terrestrial
		p->frequency		= 0;
		p->bandwidth_hz		= 1000;
		p->symbol_rate		= 1000;
		p->delivery_system	= SYS_DVBT;
		p->modulation		= QAM_16;

		for (x = 0; x < s->num_freq; x++)
		{
			p->frequency = *(s->freq + x);
			ret = fe->ops.tuner_ops.set_params(fe);

			msleep(35); // same delay tested on 955Q

			ret = fe->ops.tuner_ops.get_rf_strength(fe, &strength);
			*(s->rf_level + x) = (s8)strength * 1000;
		}
	} else { // Satellite
		p->frequency		= 0;
		p->bandwidth_hz		= 4000;
		p->symbol_rate		= 4000;
		p->delivery_system	= SYS_DVBS;
		p->modulation		= QPSK;

		si2183_set_frontend(fe);

		// minimize noise
		si2183_debug = false;

		for (x = 0; x < s->num_freq; x++)
		{
			p->frequency = *(s->freq + x);

			ret = fe->ops.tuner_ops.set_params(fe);

			cmd = si2183_CMD(client, SI2183_GET_RF_STRENGTH);

			strength = cmd.args[1];
			fe->ops.tuner_ops.get_rf_strength(fe, &strength);
			*(s->rf_level + x) = p->strength.stat[0].svalue;
		}

		// return debug
		si2183_debug = debug_state;
	}

	return 0;
}

static int si2183_get_constellation_samples(struct dvb_frontend *fe, struct dvb_fe_constellation_samples *s)
{
	struct i2c_client *client = fe->demodulator_priv;
	struct si2183_command cmd;
	u32 x;
	bool debug_state = si2183_debug;

	// minimize noise
	si2183_debug = false;

	for (x = 0 ; x < s->num ; x++) {
		cmd = si2183_CMD(client, SI2183_GET_IQ_SAMPLE);
		s->samples[x].imaginary= cmd.args[1]; //just taking a guess, no docs
		s->samples[x].real = cmd.args[2];
	}

	// return debug
	si2183_debug = debug_state;

	return 0;
}

static int send_diseqc_cmd(struct dvb_frontend *fe,
	u8 cont_tone, u8 tone_burst, u8 burst_sel,
	u8 end_seq, u8 msg_len, u8 *msg)
{
	struct i2c_client *client = fe->demodulator_priv;
	struct si2183_command cmd;
	u8 enable = 1;

	cmd = SI2183_DISEQC;
	cmd.args[1] = enable | (cont_tone << 1)
		    | (tone_burst << 2) | (burst_sel << 3)
		    | (end_seq << 4) | (msg_len << 5);

	if (msg_len > 0)
		memcpy(&cmd.args[2], msg, msg_len);

	cmd = si2183_CMD(client, cmd);
	return cmd.ret;
}

static int si2183_set_tone(struct dvb_frontend *fe, enum fe_sec_tone_mode tone)
{
	int ret;
	u8 cont_tone;

	switch (tone) {
	case SEC_TONE_ON:
		cont_tone = 1;
		break;
	case SEC_TONE_OFF:
		cont_tone = 0;
		break;
	default:
		return -EINVAL;
	}

	ret = send_diseqc_cmd(fe, cont_tone, 0, 0, 1, 0, NULL);
	if (ret)
		goto err;

	return 0;
err:
	fprintk("set_tone failed=%d", ret);
	return ret;
}

static int si2183_diseqc_send_burst(struct dvb_frontend *fe,
	enum fe_sec_mini_cmd burst)
{
	int ret;
	u8 burst_sel;

	switch (burst) {
	case SEC_MINI_A:
		burst_sel = 0;
		break;
	case SEC_MINI_B:
		burst_sel = 1;
		break;
	default:
		return -EINVAL;
	}

	ret = send_diseqc_cmd(fe, 0, 1, burst_sel, 1, 0, NULL);
	if (ret)
		goto err;

	return 0;
err:
	fprintk("set_tone failed=%d", ret);
	return ret;
}

static int si2183_diseqc_send_msg(struct dvb_frontend *fe,
	struct dvb_diseqc_master_cmd *d)
{
	int ret;
	u8 remaining = d->msg_len;
	u8 *p = d->msg;
	u8 len = 0;

	while (remaining > 0) {
		p += len;
		len = (remaining > 6) ? 6 : remaining;
		remaining -= len;
		ret = send_diseqc_cmd(fe, 0, 0, 0,
			(remaining == 0) ? 1 : 0, len, p);
		if (ret)
			goto err;
		msleep(50);
	}

	return 0;
err:
	fprintk("set_tone failed=%d", ret);
	return ret;
}

static void si2183_spi_read(struct dvb_frontend *fe, struct ecp3_info *ecp3inf)
{
	struct i2c_client *client = fe->demodulator_priv;
	struct si2183_dev *dev = i2c_get_clientdata(client);


	if (dev->read_properties)
		dev->read_properties(client->adapter,ecp3inf->reg, &(ecp3inf->data));

	return ;
}

static void si2183_spi_write(struct dvb_frontend *fe, struct ecp3_info *ecp3inf)
{
	struct i2c_client *client = fe->demodulator_priv;
	struct si2183_dev *dev = i2c_get_clientdata(client);

	if (dev->write_properties)
		dev->write_properties(client->adapter,ecp3inf->reg, ecp3inf->data);
	return ;
}

static const struct dvb_frontend_ops si2183_ops = {
	.delsys = {SYS_DVBT, SYS_DVBT2,
		   SYS_ISDBT,
		   SYS_DVBC_ANNEX_A,SYS_DVBC_ANNEX_B,
		   SYS_DVBS2, SYS_DVBS, SYS_DSS},
	.info = {
		.name = "Silicon Labs Si2183",
		.symbol_rate_min = 100000,
		.symbol_rate_max = 67500000,
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
		.extended_caps	= FE_CAN_SPECTRUMSCAN |
				  FE_CAN_IQ|
				  FE_CAN_BLINDSEARCH
	},

	.get_tune_settings	= si2183_get_tune_settings,

	.init			= si2183_init,
	.sleep			= si2183_sleep,

	.set_frontend		= si2183_set_frontend,

	.read_status		= si2183_read_status,
	.read_signal_strength	= si2183_read_signal_strength,
	.read_snr		= si2183_read_snr,
	.read_ber		= si2183_read_ber,
	.read_ucblocks		= si2183_read_ucblocks,

	.get_frontend_algo	= si2183_get_frontend_algo,
	.tune			= si2183_tune,

	.search			= si2183_search,
	.dtv_tune		= si2183_dtv_tune,
	.get_spectrum_scan		= si2183_get_spectrum_scan,
	.get_constellation_samples	= si2183_get_constellation_samples,

	.set_tone		= si2183_set_tone,
	.diseqc_send_burst	= si2183_diseqc_send_burst,
	.diseqc_send_master_cmd	= si2183_diseqc_send_msg,

	.spi_read		= si2183_spi_read,
	.spi_write		= si2183_spi_write,
};


static struct si_base *match_base(struct i2c_adapter *i2c, u8 adr)
{
	struct si_base *p;

	list_for_each_entry(p, &silist, silist)
		if (p->i2c == i2c)// && p->adr == adr) lja: TO FIX
			return p;
	return NULL;
}

static int si2183_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct si2183_config *config = client->dev.platform_data;
	struct si2183_dev *dev;
	struct si_base *base;
	int ret = 0;

	fprintk("");

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		ret = -ENOMEM;
		fprintk("kzalloc() failed");
		goto err;
	}

	base = match_base(client->adapter, client->addr);
	if (base) {
		base->count++;
		dev->base = base;
	} else {
		base = kzalloc(sizeof(struct si_base), GFP_KERNEL);
		if (!base)
			goto err_kfree;
		base->i2c = client->adapter;
		base->adr = client->addr;
		base->count = 1;
		dev->base = base;
		list_add(&base->silist, &silist);

		/* create mux i2c adapter for tuner */
		base->muxc = i2c_mux_alloc(client->adapter, &client->adapter->dev,
					  1, 0, I2C_MUX_LOCKED,
					  si2183_select, si2183_deselect);
		if (!base->muxc) {
			ret = -ENOMEM;
			goto err_base_kfree;
		}
		base->muxc->priv = client;
		ret = i2c_mux_add_adapter(base->muxc, 0, 0, 0);
		if (ret)
			goto err_base_kfree;
		base->tuner_adapter = base->muxc->adapter[0];
	}

	/* create dvb_frontend */
	memcpy(&dev->fe.ops, &si2183_ops, sizeof(struct dvb_frontend_ops));
	dev->fe.demodulator_priv = client;
	*config->i2c_adapter = base->tuner_adapter;
	*config->fe = &dev->fe;
	dev->ts_mode = config->ts_mode;
	dev->ts_clock_inv = config->ts_clock_inv;
	dev->ts_clock_gapped = config->ts_clock_gapped;
	dev->agc_mode = config->agc_mode;
	dev->RF_switch = config->RF_switch;
	dev->rf_in  = config->rf_in;
	dev->fw_loaded = false;
	dev->snr = 0;
	dev->stat_resp = 0;

	dev->active_fe = 0;

	dev->write_properties = config->write_properties;
	dev->read_properties = config->read_properties;

	i2c_set_clientdata(client, dev);
	mutex_init(&dev->i2c_mutex);

	fprintk("Silicon Labs Si2183 successfully attached");
	return 0;
err_base_kfree:
	kfree(base);
err_kfree:
	kfree(dev);
err:
	fprintk("probe failed=%d", ret);
	return ret;
}

static int si2183_remove(struct i2c_client *client)
{
	struct si2183_dev *dev = i2c_get_clientdata(client);

	fprintk("");

	dev->base->count--;
	if (dev->base->count == 0) {
		i2c_mux_del_adapters(dev->base->muxc);
		list_del(&dev->base->silist);
		kfree(dev->base);
	}

	dev->fe.ops.release = NULL;
	dev->fe.demodulator_priv = NULL;

	kfree(dev);

	return 0;
}

static const struct i2c_device_id si2183_id_table[] = {
	{"si2183", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, si2183_id_table);

static struct i2c_driver si2183_driver = {
	.driver = {
		.name	= "si2183",
	},
	.probe		= si2183_probe,
	.remove		= si2183_remove,
	.id_table	= si2183_id_table,
};

module_i2c_driver(si2183_driver);

MODULE_AUTHOR("Luis Alves <ljalvs@gmail.com>");
MODULE_DESCRIPTION("Silicon Labs Si2183 DVB-T/T2/C/C2/S/S2 demodulator driver");
MODULE_LICENSE("GPL");
