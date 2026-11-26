/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022, Linaro Ltd
 */
#ifndef __SOC_QCOM_PMIC_GLINK_H__
#define __SOC_QCOM_PMIC_GLINK_H__

#include <linux/atomic.h>
#include <linux/mutex.h>
#include <linux/usb/typec.h>

#define PMIC_GLINK_MAX_PORTS 3

struct pmic_glink;
struct pmic_glink_client;

/* Shared by UCSI (attach-time GPIO) and the authoritative altmode service. */
struct pmic_glink_orientation {
	struct mutex lock; /* serializes both sources and switch programming */
	enum typec_orientation gpio;
	enum typec_orientation altmode;
	bool warned;
	atomic_t epoch;
	unsigned int observed_epoch;
};

/* Caller holds lock. Leave PDR epochs alone so disconnect HPD is still delivered. */
static inline void pmic_glink_orientation_disconnect(struct pmic_glink_orientation *state)
{
	state->gpio = TYPEC_ORIENTATION_NONE;
	state->altmode = TYPEC_ORIENTATION_NONE;
	state->warned = false;
}

/* Caller holds lock; a queued notification from before PDR must not win. */
static inline bool
pmic_glink_orientation_sync(struct pmic_glink_orientation *state, unsigned int epoch)
{
	if (epoch != atomic_read(&state->epoch))
		return false;
	if (state->observed_epoch != epoch) {
		state->gpio = TYPEC_ORIENTATION_NONE;
		state->altmode = TYPEC_ORIENTATION_NONE;
		state->warned = false;
		state->observed_epoch = epoch;
	}
	return true;
}

static inline enum typec_orientation
pmic_glink_orientation_update(struct pmic_glink_orientation *state,
			      enum typec_orientation orientation, bool altmode)
{
	if (orientation == TYPEC_ORIENTATION_NONE) {
		state->gpio = TYPEC_ORIENTATION_NONE;
		state->altmode = TYPEC_ORIENTATION_NONE;
		state->warned = false;
	} else if (altmode) {
		state->altmode = orientation;
	} else {
		state->gpio = orientation;
	}

	return state->altmode != TYPEC_ORIENTATION_NONE ? state->altmode : state->gpio;
}

struct pmic_glink_orientation *
pmic_glink_get_orientation(struct pmic_glink_client *client, unsigned int port);

#define PMIC_GLINK_OWNER_BATTMGR	32778
#define PMIC_GLINK_OWNER_USBC		32779
#define PMIC_GLINK_OWNER_USBC_PAN	32780

#define PMIC_GLINK_REQ_RESP		1
#define PMIC_GLINK_NOTIFY		2

struct pmic_glink_hdr {
	__le32 owner;
	__le32 type;
	__le32 opcode;
};

int pmic_glink_send(struct pmic_glink_client *client, void *data, size_t len);

struct pmic_glink_client *devm_pmic_glink_client_alloc(struct device *dev,
						       unsigned int id,
						       void (*cb)(const void *, size_t, void *),
						       void (*pdr)(void *, int),
						       void *priv);
void pmic_glink_client_neuter(struct pmic_glink_client *client);


void pmic_glink_client_set_wakeup_filter(struct pmic_glink_client *client,
					 bool (*filter)(const void *, size_t, void *));
void pmic_glink_client_register(struct pmic_glink_client *client);

#endif
