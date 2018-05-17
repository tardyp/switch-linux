/*
 * tegra_alt_pcm.c - Tegra PCM driver
 *
 * Author: Stephen Warren <swarren@nvidia.com>
 * Copyright (c) 2011-2014 NVIDIA CORPORATION.  All rights reserved.
 *
 * Based on code copyright/by:
 *
 * Copyright (c) 2009-2010, NVIDIA Corporation.
 * Scott Peterson <speterson@nvidia.com>
 * Vijay Mali <vmali@nvidia.com>
 *
 * Copyright (C) 2010 Google, Inc.
 * Iliyan Malchev <malchev@google.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/dmaengine_pcm.h>

#include "tegra_pcm_alt.h"

static const struct snd_pcm_hardware tegra_alt_pcm_hardware = {
	.info			= SNDRV_PCM_INFO_MMAP |
				  SNDRV_PCM_INFO_MMAP_VALID |
				  SNDRV_PCM_INFO_PAUSE |
				  SNDRV_PCM_INFO_RESUME |
				  SNDRV_PCM_INFO_INTERLEAVED,
	.formats		= SNDRV_PCM_FMTBIT_S8 |
				  SNDRV_PCM_FMTBIT_S16_LE |
				  SNDRV_PCM_FMTBIT_S24_LE |
				  SNDRV_PCM_FMTBIT_S20_3LE |
				  SNDRV_PCM_FMTBIT_S32_LE,
	.period_bytes_min	= 128,
	.period_bytes_max	= PAGE_SIZE * 4,
	.periods_min		= 1,
	.periods_max		= 8,
	.buffer_bytes_max	= PAGE_SIZE * 8,
	.fifo_size		= 4,
};

static const struct snd_dmaengine_pcm_config tegra_alt_dmaengine_pcm_config = {
	.pcm_hardware = &tegra_alt_pcm_hardware,
	.prepare_slave_config = snd_dmaengine_pcm_prepare_slave_config,
	.prealloc_buffer_size = PAGE_SIZE * 8,
};

int tegra_alt_pcm_platform_register(struct device *dev)
{
	return snd_dmaengine_pcm_register(dev, &tegra_alt_dmaengine_pcm_config, 0);
}
EXPORT_SYMBOL_GPL(tegra_alt_pcm_platform_register);

void tegra_alt_pcm_platform_unregister(struct device *dev)
{
	return snd_dmaengine_pcm_unregister(dev);
}
EXPORT_SYMBOL_GPL(tegra_alt_pcm_platform_unregister);

MODULE_AUTHOR("Stephen Warren <swarren@nvidia.com>");
MODULE_DESCRIPTION("Tegra Alt PCM ASoC driver");
MODULE_LICENSE("GPL");
