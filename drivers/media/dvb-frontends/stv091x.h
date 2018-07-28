#ifndef _STV091X_H_
#define _STV091X_H_

#include <linux/types.h>
#include <linux/i2c.h>
#include "stv6110x.h"

enum stv091x_mode {
	STV091x_DUAL = 0,
	STV091x_SINGLE
};

struct stv091x_cfg {
	u32 clk;
	u8  adr;
	u8  parallel;
	u8  rptlvl;
	enum stv091x_mode demod_mode;

	char name[128];

	void (*set_lock_led) (struct dvb_frontend *fe, int offon);

	int (*tuner_init) (struct dvb_frontend *fe);
	int (*tuner_set_mode) (struct dvb_frontend *fe, enum tuner_mode mode);
	int (*tuner_set_params) (struct dvb_frontend *fe);
	int (*tuner_set_frequency) (struct dvb_frontend *fe, u32 frequency);
	int (*tuner_set_bandwidth) (struct dvb_frontend *fe, u32 bandwidth);

	void (*write_properties) (struct i2c_adapter *i2c,u8 reg, u32 buf);
	void (*read_properties) (struct i2c_adapter *i2c,u8 reg, u32 *buf);
};

enum stv091x_algo {
	STV091X_BLIND_SEARCH,
	STV091X_COLD_SEARCH,
	STV091X_WARM_SEARCH,
	STV091X_NOTUNE
};

#define MAKEWORD16(__a, __b)			(((__a) << 8) | (__b))

#if defined(CONFIG_DVB_STV091X) || \
	(defined(CONFIG_DVB_STV091X_MODULE) && defined(MODULE))

extern struct dvb_frontend *stv091x_attach(struct i2c_adapter *i2c,
					   struct stv091x_cfg *cfg, int nr);
#else

static inline struct dvb_frontend *stv091x_attach(struct i2c_adapter *i2c,
						  struct stv091x_cfg *cfg,
						  int nr)
{
	pr_warn("%s: driver disabled by Kconfig\n", __func__);
	return NULL;
}

#endif

#endif
