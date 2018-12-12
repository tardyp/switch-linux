/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2018, NVIDIA CORPORATION.
 */

#ifndef __FIRMWARE_TEGRA_BPMP_PRIVATE_H
#define __FIRMWARE_TEGRA_BPMP_PRIVATE_H

#include <soc/tegra/bpmp.h>

struct tegra_bpmp_ops {
	int (*init)(struct tegra_bpmp *bpmp);
	void (*deinit)(struct tegra_bpmp *bpmp);
	bool (*is_resp_ready)(struct tegra_bpmp_channel *channel);
	bool (*is_req_ready)(struct tegra_bpmp_channel *channel);
	int (*ack_resp)(struct tegra_bpmp_channel *channel);
	int (*ack_req)(struct tegra_bpmp_channel *channel);
	bool (*is_resp_channel_free)(struct tegra_bpmp_channel *channel);
	bool (*is_req_channel_free)(struct tegra_bpmp_channel *channel);
	int (*post_resp)(struct tegra_bpmp_channel *channel);
	int (*post_req)(struct tegra_bpmp_channel *channel);
	int (*ring_doorbell)(struct tegra_bpmp *bpmp);
	int (*resume)(struct tegra_bpmp *bpmp);
};

#endif
