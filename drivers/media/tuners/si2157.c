/*
 * Silicon Labs Si2146/2147/2148/2157/2158 silicon tuner driver
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

#include "si2157_priv.h"

static const struct dvb_tuner_ops si2157_ops;

/* execute firmware command */
static int si2157_cmd_execute(struct i2c_client *client, struct si2157_cmd *cmd)
{
	struct si2157_dev *dev = i2c_get_clientdata(client);
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

static int si2157_init(struct dvb_frontend *fe)
{
	struct i2c_client *client = fe->tuner_priv;
	struct si2157_dev *dev = i2c_get_clientdata(client);
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int ret, len, remaining;
	struct si2157_cmd cmd;
	const struct firmware *fw;
	const char *fw_name;
	unsigned int uitmp, chip_id;

	fprintk("");

	/* Returned IF frequency is garbage when firmware is not running */
	memcpy(cmd.args, "\x15\x00\x06\x07", 4);
	cmd.wlen = 4;
	cmd.rlen = 4;
	ret = si2157_cmd_execute(client, &cmd);
	if (ret)
		goto err;

	uitmp = cmd.args[2] << 0 | cmd.args[3] << 8;

	if (uitmp == dev->if_frequency / 1000)
		goto warm;

	/* power up */
	if (dev->chiptype == SI2157_CHIPTYPE_SI2146) {
		memcpy(cmd.args, "\xc0\x05\x01\x00\x00\x0b\x00\x00\x01", 9);
		cmd.wlen = 9;
	} else if (dev->chiptype == SI2157_CHIPTYPE_SI2141) {
		memcpy(cmd.args, "\xc0\x00\x0d\x0e\x00\x01\x01\x01\x01\x03", 10);
		cmd.wlen = 10;
	} else {
		memcpy(cmd.args, "\xc0\x00\x0c\x00\x00\x01\x01\x01\x01\x01\x01\x02\x00\x00\x01", 15);
		cmd.wlen = 15;
	}
	cmd.rlen = 1;
	ret = si2157_cmd_execute(client, &cmd);
	if (ret)
		goto err;

	/* Si2141 needs a second command before it answers the revision query */
	if (dev->chiptype == SI2157_CHIPTYPE_SI2141) {
		memcpy(cmd.args, "\xc0\x08\x01\x02\x00\x00\x01", 7);
		cmd.wlen = 7;
		ret = si2157_cmd_execute(client, &cmd);
		if (ret)
			goto err;
	}

	/* query chip revision */
	memcpy(cmd.args, "\x02", 1);
	cmd.wlen = 1;
	cmd.rlen = 13;
	ret = si2157_cmd_execute(client, &cmd);
	if (ret)
		goto err;

	chip_id = cmd.args[1] << 24 | cmd.args[2] << 16 | cmd.args[3] << 8 |
			cmd.args[4] << 0;

	#define SI2158_A20 ('A' << 24 | 58 << 16 | '2' << 8 | '0' << 0)
	#define SI2148_A20 ('A' << 24 | 48 << 16 | '2' << 8 | '0' << 0)
	#define SI2157_A30 ('A' << 24 | 57 << 16 | '3' << 8 | '0' << 0)
	#define SI2147_A30 ('A' << 24 | 47 << 16 | '3' << 8 | '0' << 0)
	#define SI2146_A10 ('A' << 24 | 46 << 16 | '1' << 8 | '0' << 0)
	#define SI2141_A10 ('A' << 24 | 41 << 16 | '1' << 8 | '0' << 0)

	switch (chip_id) {
	case SI2158_A20:
	case SI2148_A20:
		fw_name = SI2158_A20_FIRMWARE;
		break;
	case SI2141_A10:
		fw_name = SI2141_A10_FIRMWARE;
		break;
	case SI2157_A30:
	case SI2147_A30:
	case SI2146_A10:
		fw_name = NULL;
		break;
	default:
		fprintk("unknown chip version Si21%d-%c%c%c",
				cmd.args[2], cmd.args[1],
				cmd.args[3], cmd.args[4]);
		ret = -EINVAL;
		goto err;
	}

	fprintk("found a 'Silicon Labs Si21%d-%c%c%c'", cmd.args[2], cmd.args[1], cmd.args[3], cmd.args[4]);

	if (fw_name == NULL)
		goto skip_fw_download;

	/* request the firmware, this will block and timeout */
	ret = request_firmware(&fw, fw_name, &client->dev);
	if (ret) {
		fprintk("firmware file '%s' not found",
				fw_name);
		goto err;
	}

	/* firmware should be n chunks of 17 bytes */
	if (fw->size % 17 != 0) {
		fprintk("firmware file '%s' is invalid",
				fw_name);
		ret = -EINVAL;
		goto err_release_firmware;
	}

	fprintk("downloading firmware from file '%s'", fw_name);

	for (remaining = fw->size; remaining > 0; remaining -= 17) {
		len = fw->data[fw->size - remaining];
		if (len > SI2157_ARGLEN) {
			fprintk("Bad firmware length");
			ret = -EINVAL;
			goto err_release_firmware;
		}
		memcpy(cmd.args, &fw->data[(fw->size - remaining) + 1], len);
		cmd.wlen = len;
		cmd.rlen = 1;
		ret = si2157_cmd_execute(client, &cmd);
		if (ret) {
			fprintk("firmware download failed %d",
					ret);
			goto err_release_firmware;
		}
	}

	release_firmware(fw);

skip_fw_download:
	/* reboot the tuner with new firmware? */
	memcpy(cmd.args, "\x01\x01", 2);
	cmd.wlen = 2;
	cmd.rlen = 1;
	ret = si2157_cmd_execute(client, &cmd);
	if (ret)
		goto err;

	/* query firmware version */
	memcpy(cmd.args, "\x11", 1);
	cmd.wlen = 1;
	cmd.rlen = 10;
	ret = si2157_cmd_execute(client, &cmd);
	if (ret)
		goto err;

	fprintk("firmware version: %c.%c.%d",
			cmd.args[6], cmd.args[7], cmd.args[8]);
warm:
	/* init statistics in order signal app which are supported */
	c->strength.len = 1;
	c->strength.stat[0].scale = FE_SCALE_NOT_AVAILABLE;

	dev->active = true;
	return 0;
err_release_firmware:
	release_firmware(fw);
err:
	fprintk("failed=%d", ret);
	return ret;
}

static int si2157_sleep(struct dvb_frontend *fe)
{
	struct i2c_client *client = fe->tuner_priv;
	struct si2157_dev *dev = i2c_get_clientdata(client);
//	int ret;
//	struct si2157_cmd cmd;

//	fprintk("");

	dev->active = false;

//	/* standby */
//	memcpy(cmd.args, "\x16\x00", 2);
//	cmd.wlen = 2;
//	cmd.rlen = 1;
//	ret = si2157_cmd_execute(client, &cmd);
//	if (ret)
//		goto err;

	return 0;
//err:
//	fprintk("failed=%d", ret);
//	return ret;
}

static int si2157_set_params(struct dvb_frontend *fe)
{
	struct i2c_client *client = fe->tuner_priv;
	struct si2157_dev *dev = i2c_get_clientdata(client);
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int ret;
	struct si2157_cmd cmd;
	u8 bandwidth, delivery_system;
	u32 if_frequency = 5000000;

	if (!dev->active) {
		ret = -EAGAIN;
		goto err;
	}

	if (c->bandwidth_hz <= 6000000)
		bandwidth = 0x06;
	else if (c->bandwidth_hz <= 7000000)
		bandwidth = 0x07;
	else if (c->bandwidth_hz <= 8000000)
		bandwidth = 0x08;
	else
		bandwidth = 0x0f;

	switch (c->delivery_system) {
	case SYS_ATSC:
			delivery_system = 0x00;
			if_frequency = 3250000;
			break;
	case SYS_DVBC_ANNEX_B:
			delivery_system = 0x10;
			if_frequency = 4000000;
			break;
	case SYS_DVBT:
	case SYS_DVBT2: /* it seems DVB-T and DVB-T2 both are 0x20 here */
			delivery_system = 0x20;
			break;
	case SYS_DVBC_ANNEX_A:
			delivery_system = 0x30;
			break;
	case SYS_ISDBT:
			delivery_system = 0x40;
			break;
	case SYS_DTMB:
			delivery_system = 0x60;
			break;
	default:
			ret = -EINVAL;
			goto err;
	}

	memcpy(cmd.args, "\x14\x00\x03\x07\x00\x00", 6);
	cmd.args[4] = delivery_system | bandwidth;
	if (dev->inversion)
		cmd.args[5] = 0x01;
	cmd.wlen = 6;
	cmd.rlen = 4;
	ret = si2157_cmd_execute(client, &cmd);
	if (ret)
		goto err;

	if (dev->chiptype == SI2157_CHIPTYPE_SI2146)
		memcpy(cmd.args, "\x14\x00\x02\x07\x00\x01", 6);
	else
		memcpy(cmd.args, "\x14\x00\x02\x07\x00\x00", 6);
	cmd.args[4] = dev->if_port;
	cmd.wlen = 6;
	cmd.rlen = 4;
	ret = si2157_cmd_execute(client, &cmd);
	if (ret)
		goto err;

	/* set if frequency if needed */
	if (if_frequency != dev->if_frequency) {
		memcpy(cmd.args, "\x14\x00\x06\x07", 4);
		cmd.args[4] = (if_frequency / 1000) & 0xff;
		cmd.args[5] = ((if_frequency / 1000) >> 8) & 0xff;
		cmd.wlen = 6;
		cmd.rlen = 4;
		ret = si2157_cmd_execute(client, &cmd);
		if (ret)
			goto err;

		dev->if_frequency = if_frequency;
	}

	/* set frequency */
	memcpy(cmd.args, "\x41\x00\x00\x00\x00\x00\x00\x00", 8);
	cmd.args[4] = (c->frequency >>  0) & 0xff;
	cmd.args[5] = (c->frequency >>  8) & 0xff;
	cmd.args[6] = (c->frequency >> 16) & 0xff;
	cmd.args[7] = (c->frequency >> 24) & 0xff;
	cmd.wlen = 8;
	cmd.rlen = 1;
	ret = si2157_cmd_execute(client, &cmd);
	if (ret)
		goto err;

	return 0;
err:
	fprintk("failed=%d", ret);
	return ret;
}

static int si2157_get_if_frequency(struct dvb_frontend *fe, u32 *frequency)
{
	struct i2c_client *client = fe->tuner_priv;
	struct si2157_dev *dev = i2c_get_clientdata(client);

	*frequency = dev->if_frequency;
	return 0;
}

static int si2157_get_rf_strength(struct dvb_frontend *fe,
				       u16 *signal_strength)
{
	struct i2c_client *client = fe->tuner_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	struct si2157_cmd cmd;
	int ret;

	memcpy(cmd.args, "\x42\x00", 2);
	cmd.wlen = 2;
	cmd.rlen = 12;
	ret = si2157_cmd_execute(client, &cmd);
	if (ret) {
		fprintk("failed=%d", ret);
		return -1;
	}

	c->strength.stat[0].scale = FE_SCALE_DECIBEL;
	c->strength.stat[0].svalue = (s8)cmd.args[3] * 1000;
	*signal_strength = cmd.args[3];

	return 0;
}

static const struct dvb_tuner_ops si2157_ops = {
	.info = {
		.name           = "Silicon Labs Si2141/Si2146/2147/2148/2157/2158",
		.frequency_min  = 42000000,
		.frequency_max  = 870000000,
	},

	.init			= si2157_init,
	.sleep			= si2157_sleep,
	.set_params		= si2157_set_params,
	.get_if_frequency	= si2157_get_if_frequency,
	.get_rf_strength	= si2157_get_rf_strength,
};

static int si2157_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct si2157_config *cfg = client->dev.platform_data;
	struct dvb_frontend *fe = cfg->fe;
	struct si2157_dev *dev;
	struct si2157_cmd cmd;
	int ret;

	fprintk("");

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		ret = -ENOMEM;
		fprintk("kzalloc() failed");
		goto err;
	}

	i2c_set_clientdata(client, dev);
	dev->fe = cfg->fe;
	dev->inversion = cfg->inversion;
	dev->if_port = cfg->if_port;
	dev->chiptype = (u8)id->driver_data;
	dev->if_frequency = 5000000; /* default value of property 0x0706 */
	mutex_init(&dev->i2c_mutex);

	/* check if the tuner is there */
	cmd.wlen = 0;
	cmd.rlen = 1;
	ret = si2157_cmd_execute(client, &cmd);
	if (ret)
		goto err_kfree;

	memcpy(&fe->ops.tuner_ops, &si2157_ops, sizeof(struct dvb_tuner_ops));
	fe->tuner_priv = client;

#ifdef CONFIG_MEDIA_CONTROLLER
	if (cfg->mdev) {
		dev->mdev = cfg->mdev;

		dev->ent.name = KBUILD_MODNAME;
		dev->ent.function = MEDIA_ENT_F_TUNER;

		dev->pad[TUNER_PAD_RF_INPUT].flags = MEDIA_PAD_FL_SINK;
		dev->pad[TUNER_PAD_OUTPUT].flags = MEDIA_PAD_FL_SOURCE;
		dev->pad[TUNER_PAD_AUD_OUT].flags = MEDIA_PAD_FL_SOURCE;

		ret = media_entity_pads_init(&dev->ent, TUNER_NUM_PADS,
					     &dev->pad[0]);

		if (ret)
			goto err_kfree;

		ret = media_device_register_entity(cfg->mdev, &dev->ent);
		if (ret) {
			media_entity_cleanup(&dev->ent);
			goto err_kfree;
		}
	}
#endif

	fprintk("Silicon Labs %s successfully attached",
			dev->chiptype == SI2157_CHIPTYPE_SI2141 ?  "Si2141" :
			dev->chiptype == SI2157_CHIPTYPE_SI2146 ?
			"Si2146" : "Si2147/2148/2157/2158");

	return 0;

err_kfree:
	kfree(dev);
err:
	fprintk("failed=%d", ret);
	return ret;
}

static int si2157_remove(struct i2c_client *client)
{
	struct si2157_dev *dev = i2c_get_clientdata(client);
	struct dvb_frontend *fe = dev->fe;

	fprintk("");

	/* stop statistics polling */
	cancel_delayed_work_sync(&dev->stat_work);

#ifdef CONFIG_MEDIA_CONTROLLER_DVB
	if (dev->mdev)
		media_device_unregister_entity(&dev->ent);
#endif

	memset(&fe->ops.tuner_ops, 0, sizeof(struct dvb_tuner_ops));
	fe->tuner_priv = NULL;
	kfree(dev);

	return 0;
}

static const struct i2c_device_id si2157_id_table[] = {
	{"si2157", SI2157_CHIPTYPE_SI2157},
	{"si2146", SI2157_CHIPTYPE_SI2146},
	{"si2141", SI2157_CHIPTYPE_SI2141},
	{}
};
MODULE_DEVICE_TABLE(i2c, si2157_id_table);

static struct i2c_driver si2157_driver = {
	.driver = {
		.name		     = "si2157",
		.suppress_bind_attrs = true,
	},
	.probe		= si2157_probe,
	.remove		= si2157_remove,
	.id_table	= si2157_id_table,
};

module_i2c_driver(si2157_driver);

MODULE_DESCRIPTION("Silicon Labs Si2141/Si2146/2147/2148/2157/2158 silicon tuner driver");
MODULE_AUTHOR("Antti Palosaari <crope@iki.fi>");
MODULE_LICENSE("GPL");
MODULE_FIRMWARE(SI2158_A20_FIRMWARE);
MODULE_FIRMWARE(SI2141_A10_FIRMWARE);
