/*
 * STV6120 Silicon tuner driver
 *
 * Copyright (C) Chris Leee <updatelee@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/

#ifndef __STV6120_H
#define __STV6120_H

struct stv6120_config {
	u8	addr;
	u32	refclk;
	u8	clk_div; /* divisor value for the output clock */
	u8	tuner;
};

enum tuner_number {
	TUNER_1,
	TUNER_2,
};

struct stv6120_devctl {
	int (*tuner_init) (struct dvb_frontend *fe);
	int (*tuner_sleep) (struct dvb_frontend *fe);
	int (*tuner_set_mode) (struct dvb_frontend *fe, enum tuner_mode mode);
	int (*tuner_set_params) (struct dvb_frontend *fe);
	int (*tuner_set_frequency) (struct dvb_frontend *fe, u32 frequency);
	int (*tuner_get_frequency) (struct dvb_frontend *fe, u32 *frequency);
	int (*tuner_set_bandwidth) (struct dvb_frontend *fe, u32 bandwidth);
	int (*tuner_get_bandwidth) (struct dvb_frontend *fe, u32 *bandwidth);
	int (*tuner_set_bbgain) (struct dvb_frontend *fe, u32 gain);
	int (*tuner_get_bbgain) (struct dvb_frontend *fe, u32 *gain);
	int (*tuner_set_refclk)  (struct dvb_frontend *fe, u32 refclk);
	int (*tuner_get_status) (struct dvb_frontend *fe, u32 *status);
};


#if IS_REACHABLE(CONFIG_MEDIA_TUNER_STV6120)
extern struct stv6120_devctl *stv6120_attach(struct dvb_frontend *fe, const struct stv6120_config *config, struct i2c_adapter *i2c);
#else
static inline struct stv6120_devctl *stv6120_attach(struct dvb_frontend *fe, const struct stv6120_config *config, struct i2c_adapter *i2c)
{
	pr_info("%s: driver disabled by Kconfig\n", __func__);
	return NULL;
}
#endif /* CONFIG_DVB_STV6120 */
#endif /* __STV6120_H */
