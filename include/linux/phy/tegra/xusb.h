/*
 * Copyright (c) 2016-2018, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#ifndef PHY_TEGRA_XUSB_H
#define PHY_TEGRA_XUSB_H

struct tegra_xusb_padctl;
struct device;

struct tegra_xusb_padctl *tegra_xusb_padctl_get(struct device *dev);
void tegra_xusb_padctl_put(struct tegra_xusb_padctl *padctl);

int tegra_xusb_padctl_usb3_save_context(struct tegra_xusb_padctl *padctl,
					unsigned int port);
int tegra_xusb_padctl_hsic_set_idle(struct tegra_xusb_padctl *padctl,
				    unsigned int port, bool idle);
int tegra_xusb_padctl_usb3_set_lfps_detect(struct tegra_xusb_padctl *padctl,
					   unsigned int port, bool enable);
int tegra_xusb_padctl_set_vbus_override(struct tegra_xusb_padctl *padctl);
int tegra_xusb_padctl_clear_vbus_override(struct tegra_xusb_padctl *padctl);
int tegra_xusb_padctl_set_id_override(struct tegra_xusb_padctl *padctl);
int tegra_xusb_padctl_clear_id_override(struct tegra_xusb_padctl *padctl);

void tegra_phy_xusb_utmi_pad_power_on(struct phy *phy);
void tegra_phy_xusb_utmi_pad_power_down(struct phy *phy);
int tegra_phy_xusb_utmi_port_reset_quirk(struct phy *phy);
#endif /* PHY_TEGRA_XUSB_H */
