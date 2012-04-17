/**
 * Copyright (c) 2011 Ineda sysems.  All rights reserved.
 *
 * INEDA SYSTEMS Pvt Ltd(ISPL) and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without license
 * agreement from INEDA SYSTEMS Pvt.Ltd(ISPL) is strictly prohibited.
 **/
#include "indus.h"
/******************************************************************************
 * ethtool_ops functionalities
 *****************************************************************************/
static int indus_get_settings(struct net_device *ndev,
		struct ethtool_cmd *eth_cmd)
{
	return 0;
}

static int indus_set_settings(struct net_device *ndev,
		struct ethtool_cmd *eth_cmd)
{
	return 0;
}

static void indus_get_drvinfo(struct net_device *ndev,
		struct ethtool_drvinfo *eth_drv_info)
{

}

static int indus_get_regs_len(struct net_device *ndev)
{
	return 0;
}

static void indus_get_regs(struct net_device *ndev,
		struct ethtool_regs *eth_reg, void *none)
{

}

static void indus_get_wol(struct net_device *ndev,
		struct ethtool_wolinfo *eth_wolinfo)
{

}

static int indus_set_wol(struct net_device *ndev,
		struct ethtool_wolinfo *eth_wolinfo)
{
	return 0;
}

static u32 indus_get_msglevel(struct net_device *ndev)
{
	return 0;
}

static void indus_set_msglevel(struct net_device *ndev, u32 level)
{

}

static int indus_nway_reset(struct net_device *ndev)
{
	return 0;
}

static u32 indus_get_link(struct net_device *ndev)
{
	return 0;
}

static int indus_get_eeprom_len(struct net_device *ndev)
{
	return 0;
}

static int indus_get_eeprom(struct net_device *ndev,
		struct ethtool_eeprom *eth_eeprom, u8 *data)
{
	return 0;
}

static int indus_set_eeprom(struct net_device *ndev,
		struct ethtool_eeprom *eth_eeprom, u8 *data)
{
	return 0;
}

static int indus_get_coalesce(struct net_device *ndev,
		struct ethtool_coalesce *eth_coalesce)
{
	return 0;
}

static int indus_set_coalesce(struct net_device *ndev,
		struct ethtool_coalesce *eth_coalesce)
{
	return 0;
}

static void indus_get_ringparam(struct net_device *ndev,
		struct ethtool_ringparam *eth_ringparam)
{

}

static int indus_set_ringparam(struct net_device *ndev,
		struct ethtool_ringparam *eth_ringparam)
{
	return 0;
}

static void indus_get_pauseparam(struct net_device *ndev,
		struct ethtool_pauseparam *eth_pauseparam)
{

}

static int indus_set_pauseparam(struct net_device *ndev,
		struct ethtool_pauseparam *eth_pauseparam)
{
	return 0;
}

static u32 indus_get_rx_csum(struct net_device *ndev)
{
	return 0;
}

static int indus_set_rx_csum(struct net_device *ndev, u32 csum)
{
	return 0;
}

static u32 indus_get_tx_csum(struct net_device *ndev)
{
	return 0;
}

static int indus_set_tx_csum(struct net_device *ndev, u32 csum)
{
	return 0;
}

static u32 indus_get_sg(struct net_device *ndev)
{
	return 0;
}

static int indus_set_sg(struct net_device *ndev, u32 sg)
{
	return 0;
}

static u32 indus_get_tso(struct net_device *ndev)
{
	return 0;
}

static int indus_set_tso(struct net_device *ndev, u32 tso)
{
	return 0;
}

static void indus_self_test(struct net_device *ndev,
		struct ethtool_test *eth_test, u64 *data)
{

}

static void indus_get_strings(struct net_device *ndev, u32 stringset, u8 *data)
{

}

static int indus_phys_id(struct net_device *ndev, u32 id)
{
	return 0;
}

static void indus_get_ethtool_stats(struct net_device *ndev,
		struct ethtool_stats *eth_status, u64 *data)
{

}

static int indus_begin(struct net_device *ndev)
{
	return 0;
}

static void indus_complete(struct net_device *ndev)
{

}

static u32 indus_get_ufo(struct net_device *ndev)
{
	return 0;
}

static int indus_set_ufo(struct net_device *ndev, u32 ufo)
{
	return 0;
}

static u32 indus_get_flags(struct net_device *ndev)
{
	return 0;
}

static int indus_set_flags(struct net_device *ndev, u32 ufo)
{
	return 0;
}

static u32 indus_get_priv_flags(struct net_device *ndev)
{
	return 0;
}

static int indus_set_priv_flags(struct net_device *ndev, u32 flags)
{
	return 0;
}

static int indus_get_sset_count(struct net_device *ndev, int count)
{
	return 0;
}

static int indus_get_rxnfc(struct net_device *ndev,
		struct ethtool_rxnfc *eth_rxnfc, void *data)
{
	return 0;
}

static int indus_set_rxnfc(struct net_device *ndev,
		struct ethtool_rxnfc *eth_rxnfc)
{
	return 0;
}

static int indus_flash_device(struct net_device *ndev,
		struct ethtool_flash *eth_flash)
{
	return 0;
}

static int indus_reset(struct net_device *ndev, u32 *data)
{
	return 0;
}

static int indus_set_rx_ntuple(struct net_device *ndev,
		struct ethtool_rx_ntuple *eth_rx_ntuple)
{
	return 0;
}

static int indus_get_rx_ntuple(struct net_device *ndev, u32 stringset,
		void *data)
{
	return 0;
}

const struct ethtool_ops indus_ethtool_ops =
{
	.get_settings		= indus_get_settings,
	.set_settings		= indus_set_settings,
	.get_drvinfo		= indus_get_drvinfo,
	.get_regs_len		= indus_get_regs_len,
	.get_regs		= indus_get_regs,
	.get_wol		= indus_get_wol,
	.set_wol		= indus_set_wol,
	.get_msglevel		= indus_get_msglevel,
	.set_msglevel		= indus_set_msglevel,
	.nway_reset		= indus_nway_reset,
	.get_link		= indus_get_link,
	.get_eeprom_len		= indus_get_eeprom_len,
	.get_eeprom		= indus_get_eeprom,
	.set_eeprom		= indus_set_eeprom,
	.get_coalesce		= indus_get_coalesce,
	.set_coalesce		= indus_set_coalesce,
	.get_ringparam		= indus_get_ringparam,
	.set_ringparam		= indus_set_ringparam,
	.get_pauseparam		= indus_get_pauseparam,
	.set_pauseparam		= indus_set_pauseparam,
	.get_rx_csum		= indus_get_rx_csum,
	.set_rx_csum		= indus_set_rx_csum,
	.get_tx_csum		= indus_get_tx_csum,
	.set_tx_csum		= indus_set_tx_csum,
	.get_sg			= indus_get_sg,
	.set_sg			= indus_set_sg,
	.get_tso		= indus_get_tso,
	.set_tso		= indus_set_tso,
	.self_test		= indus_self_test,
	.get_strings		= indus_get_strings,
	.phys_id		= indus_phys_id,
	.get_ethtool_stats	= indus_get_ethtool_stats,
	.begin			= indus_begin,
	.complete		= indus_complete,
	.get_ufo		= indus_get_ufo,
	.set_ufo		= indus_set_ufo,
	.get_flags		= indus_get_flags,
	.set_flags		= indus_set_flags,
	.get_priv_flags		= indus_get_priv_flags,
	.set_priv_flags		= indus_set_priv_flags,
	.get_sset_count		= indus_get_sset_count,
	.get_rxnfc		= indus_get_rxnfc,
	.set_rxnfc		= indus_set_rxnfc,
	.flash_device		= indus_flash_device,
	.reset			= indus_reset,
	.set_rx_ntuple		= indus_set_rx_ntuple,
	.get_rx_ntuple		= indus_get_rx_ntuple,
};
