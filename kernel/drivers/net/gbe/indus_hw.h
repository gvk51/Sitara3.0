/**
 * Copyright (c) 2011 Ineda sysems.  All rights reserved.
 *
 * INEDA SYSTEMS Pvt Ltd(ISPL) and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without license
 * agreement from INEDA SYSTEMS Pvt.Ltd(ISPL) is strictly prohibited.
 **/

#ifndef INDUS_HW_H_
#define INDUS_HW_H_

#include "gmac_dma.h"
#include "gmac_mac.h"
#include "gmac_pqi.h"
#include "gmac_sm.h"

/* TODO PHASE 1 Modify the vendor id based on the given */

#define INDUS_VENDOR_ID 	0x1c31
#define INDUS_DEVICE_ID 	0x0001
#define INDUS_QUEUE_SIZE 	0x400

/*******************************************************************************
 * indus_rx_ob accessing macros
 ******************************************************************************/
#define INDUS_RX_OB_HDR_LEN_SHIFT		0x0		/*dw0         */
#define INDUS_RX_OB_HDR_LEN_MASK		0x3fff

#define INDUS_RX_OB_STATUS_DD_SHIFT		0x0		/*dw2         */
#define INDUS_RX_OB_STATUS_DD_MASK		0x1

#define INDUS_RX_OB_STATUS_EOP_SHIFT		0x1
#define INDUS_RX_OB_STATUS_EOP_MASK		0x1

#define INDUS_RX_OB_STATUS_VP_SHIFT		0x2
#define INDUS_RX_OB_STATUS_VP_MASK		0x1

#define INDUS_RX_OB_STATUS_UDPCS_SHIFT		0x3
#define INDUS_RX_OB_STATUS_UDPCS_MASK		0x1

#define INDUS_RX_OB_STATUS_L4CS_SHIFT		0x4
#define INDUS_RX_OB_STATUS_L4CS_MASK		0x1

#define INDUS_RX_OB_STATUS_IPCS_SHIFT		0x5
#define INDUS_RX_OB_STATUS_IPCS_MASK		0x1

#define INDUS_RX_OB_STATUS_UDPV_SHIFT		0x6
#define INDUS_RX_OB_STATUS_UDPV_MASK		0x1

#define INDUS_RX_OB_STATUS_PUC_SHIFT		0x7
#define INDUS_RX_OB_STATUS_PUC_MASK		0x1

#define INDUS_RX_OB_STATUS_PMC_SHIFT		0x8
#define INDUS_RX_OB_STATUS_PMC_MASK		0x1

#define INDUS_RX_OB_STATUS_PBC_SHIFT		0x9
#define INDUS_RX_OB_STATUS_PBC_MASK		0x1

#define INDUS_RX_OB_STATUS_PME_SHIFT		0xA
#define INDUS_RX_OB_STATUS_PME_MASK		0xA

#define INDUS_RX_OB_STSTUS_HDR_SP_SHIFT		0xB
#define INDUS_RX_OB_STSTUS_HDR_SP_MASK		0x3
#define INDUS_RX_OB_STATUS_HDR_SP_TCP		0x2
#define INDUS_RX_OB_STATUS_HDR_SP_IP		0x1

#define INDUS_RX_OB_ERROR_MASK			0xfe000		/* dw2        */
#define INDUS_RX_OB_ERROR_SHIFT			0xD	/* d =>13 0-12 status */

#define INDUS_RX_OB_ERROR_RNT_SHIFT		0xE
#define INDUS_RX_OB_ERROR_RNT_MASK		0x1

#define INDUS_RX_OB_ERROR_NOC_SHIFT		0xF
#define INDUS_RX_OB_ERROR_NOC_MASK		0x1

#define INDUS_RX_OB_ERROR_LMS_SHIFT		0x10
#define INDUS_RX_OB_ERROR_LMS_MASK		0x1

#define INDUS_RX_OB_ERROR_L4E_SHIFT		0x11
#define INDUS_RX_OB_ERROR_L4E_MASK		0x1

#define INDUS_RX_OB_ERROR_IPE_SHIFT		0x12
#define INDUS_RX_OB_ERROR_IPE_MASK		0x1

#define INDUS_RX_OB_ERROR_RXE_SHIFT		0x13
#define INDUS_RX_OB_ERROR_RXE_MASK		0x1

#define INDUS_RX_OB_RSS_TYPE_SHIFT		0x14		/* 14 ==> 20  */
#define INDUS_RX_OB_RSS_TYPE_MASK		0xF

#define INDUS_RX_OB_RSS_TYPE_HASH_NO		0x0
#define INDUS_RX_OB_RSS_TYPE_HASH_TCP_IPV4	0x1
#define INDUS_RX_OB_RSS_TYPE_HASH_IPV4		0x2
#define INDUS_RX_OB_RSS_TYPE_HASH_TCP_IPV6	0x3
#define INDUS_RX_OB_RSS_TYPE_HASH_IPV6		0x4
#define INDUS_RX_OB_RSS_TYPE_HASH_IPV6_EX	0x5
#define INDUS_RX_OB_RSS_TYPE_HASH_TCP_IPV6_EX	0x6
#define INDUS_RX_OB_RSS_TYPE_HASH_UDP_IPV4	0x7
#define INDUS_RX_OB_RSS_TYPE_HASH_UDP_IPV6	0x8
#define INDUS_RX_OB_RSS_TYPE_HASH_UDP_IPV6_EX	0x9

#define INDUS_RX_OB_PKT_TYPE_SHIFT		0x18		/* 18 ==> 24  */
#define INDUS_RX_OB_PKT_TYPE_MASK		0x3f		/*  dw2       */

#define INDUS_RX_OB_PKT_TYPE_IPV4		0x0
#define INDUS_RX_OB_PKT_TYPE_IPV4_EX		0x1
#define INDUS_RX_OB_PKT_TYPE_IPV6		0x2
#define INDUS_RX_OB_PKT_TYPE_IPV6_EX		0x3
#define INDUS_RX_OB_PKT_TYPE_TCP		0x4
#define INDUS_RX_OB_PKT_TYPE_UDP		0x5

#define INDUS_RX_OB_INDEX_SHIFT			0x0
#define INDUS_RX_OB_INDEX_MASK			0x3FF		/*  dw3       */

#define INDUS_TX_DESC_L2LEN_SHIFT		0x0	/* dw0 in tx_desc     */
#define INDUS_TX_DESC_L2LEN_MASK		0x1F

#define INDUS_TX_DESC_IPLEN_SHIFT		0x5
#define INDUS_TX_DESC_IPLEN_MASK		0x1FF

#define INDUS_TX_DESC_L4LEN_SHIFT		0xE
#define INDUS_TX_DESC_L4LEN_MASK		0x3F

#define INDUS_TX_DESC_MSS_SHIFT			0x0	/* dw1 in tx_desc     */
#define INDUS_TX_DESC_MSS_MASK			0x3fff


#define INDUS_TX_DESC_CMD_SOP			0x1
#define INDUS_TX_DESC_CMD_EOP			0x2
#define INDUS_TX_DESC_CMD_CRC			0x4
#define INDUS_TX_DESC_CMD_L3P_IPV4		0x0
#define INDUS_TX_DESC_CMD_L3P_IPV6		0x8
#define INDUS_TX_DESC_CMD_L3P_IPV6_IN_IPV4	0x10
#define INDUS_TX_DESC_CMD_L3P_UR_PKT		0x18
#define INDUS_TX_DESC_CMD_L4PV_TCP_UDP		0x20
#define INDUS_TX_DESC_CMD_L4P_TCP		0x0;
#define INDUS_TX_DESC_CMD_L4P_UDP		0x40;
#define INDUS_TX_DESC_CMD_TSO			0x80
#define INDUS_TX_DESC_CMD_L3CSO			0x100
#define INDUS_TX_DESC_CMD_l4CSO			0x200
#define INDUS_TX_DESC_CMD_NOVLAN_NOINS		0x00
#define INDUS_TX_DESC_CMD_NOVLAN_INS		0x400
#define INDUS_TX_DESC_CMD_VLAN_NOINS		0x800
#define INDUS_TX_DESC_CMD_VLAN_INS		0xC00
#define INDUS_TX_DESC_CMD_SAR_SAME		0x0
#define INDUS_TX_DESC_CMD_SAR_MAC_1		0x4000
#define INDUS_TX_DESC_CMD_SAR_MAC_2		0x5000
#define INDUS_TX_DESC_CMD_SAR_MAC_3		0x6000
#define INDUS_TX_DESC_CMD_SAR_MAC_4		0x7000
#define INDUS_TX_DESC_CMD_PE			0x8000

#define INDUS_TX_CB_DESC_INDEX_SHIFT		0x0
#define INDUS_TX_CB_DESC_INDEX_MASK		0x3FF

#define INDUS_TX_CB_DESC_VALID_SHIFT		0xA
#define INDUS_TX_CB_DESC_VALID_MASK		0x1

/**
 * DCB Control register offsets
 */
#define DCB_CONTROL_ADDR_LOW_REG		0x0
#define DCB_CONTROL_ADDR_HIGH_REG		0x4
#define DCB_CONTROL_LEGTH			0x8
/**
 * Tx Empty DCB control register's offsets
 */
#define TX_EMPTY_DCB_ADDR_LOW_REG		0x0
#define TX_EMPTY_DCB_ADDR_HIGH_REG		0x4
#define TX_EMPTY_DCB_LEGTH			0x8
#define TX_EMPTY_DCB_HEAD			0xc

/**
 * RX Empty DCB Control register's offsets
 */
#define RX_EMPTY_DCB_ADDR_LOW_REG		0x0
#define RX_EMPTY_DCB_ADDR_HIGH_REG		0x4
#define RX_EMPTY_DCB_LEGTH			0x8
#define RX_EMPTY_DCB_HEAD			0xc

/**
 * RX occupied DCB Control register's offsets
 */
#define RX_OCCUPIED_DCB_ADDR_LOW_REG		0x0
#define RX_OCCUPIED_DCB_ADDR_HIGH_REG		0x4
#define RX_OCCUPIED_DCB_LEGTH			0x8
#define RX_OCCUPIED_DCB_TAIL			0x10

/**
 * TX occupied DCB Control register's offsets
 */
#define TX_OCCUPIED_DCB_ADDR_LOW_REG		0x0
#define TX_OCCUPIED_DCB_ADDR_HIGH_REG		0x4
#define TX_OCCUPIED_DCB_LEGTH			0x8
#define TX_OCCUPIED_DCB_TAIL			0x10

/*
 * MSIX and MSI ENABLE
 */
#define INDUS_PCICFG_MSI_ENABLE_OFFSET		0x40
#define INDUS_PCICFG_MSI_ENABLE_MASK		0x10000

#define INDUS_PCICFG_MSIX_ENABLE_OFFSET		0x58
#define INDUS_PCICFG_MSIX_ENABLE_MASK		0x80000000

#endif /* INDUS_HW_H_ */
