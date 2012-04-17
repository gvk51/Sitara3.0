/**
 * Copyright (c) 2011 Ineda sysems.  All rights reserved.
 *
 * INEDA SYSTEMS Pvt Ltd(ISPL) and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without license
 * agreement from INEDA SYSTEMS Pvt.Ltd(ISPL) is strictly prohibited.
 **/

#ifndef _INDUS_H_
#define _INDUS_H_

#include <linux/pci.h>
#include <linux/etherdevice.h>
#include <linux/version.h>
#include <linux/delay.h>
#include <net/tcp.h>
#include <net/ip.h>
#include <net/ipv6.h>
#include <linux/if_vlan.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include "indus_hw.h"


#define INDUS_SET_DEVICE_OPS(netdev,ops)	((netdev)->netdev_ops = (ops))
#define INDUS_SET_ETHTOOL_OPS(netdev,ops)	((netdev)->ethtool_ops = (ops))
#define INDUS_NAPI_WEIGHT			64
#define INDUS_MAX_TX_FRAG_COUNT			30
#define INDUS_DRV				"indus-gbe"
#define INDUS_RX_Q_LEN				1023
#define INDUS_TX_Q_LEN 				1023
#define INDUS_QUEUE_INDEX_HIGH			1023
#define INDUS_QUEUE_MAX_POST			1023
#define LINK_UP					true
#define LINK_DOWN				false
#define RX_FRAG_SIZE				2048
#define MAX_RX_POST 				INDUS_NAPI_WEIGHT
#define RX_FRAGS_REFILL_WM			(INDUS_RX_Q_LEN - MAX_RX_POST)
#define RX_QUEUE0_ENABLE			0x8
#define INDUS_IE1_ENABLE			INDUS_IE1_TX_Q_CMPLT | RX_QUEUE0_ENABLE
/*#define INDUS_IE1_ENABLE			INDUS_IE1_TX_Q_CMPLT | \
						INDUS_IE1_LSCD | \
						INDUS_IE1_MII | \
						INDUS_IE1_RX_QUEUE
*/

#define INDUS_IE1_DISABLE			INDUS_IE1_DEFAULT_VAL

#define INDUS_HDR_LEN 				64
#define INDUS_MAX_JUMBO_FRAME_SIZE		9018
extern int host;
#define GET_FIELD(x,shift,mask)			((x>>shift)&mask)


extern const struct ethtool_ops indus_ethtool_ops;

struct indus_dma_mem {
	void *va;	/* used to hold the virtual address of maped memory   */
	dma_addr_t dma;	/* physical address of the maped meory.               */
	u32 size;	/* size of the maped memory.                          */
};
struct indus_queue_info {
	struct indus_dma_mem dma_mem;	/* created memory for this queue      */
	u16 len;			/* length of this queue               */
	u16 entry_size;			/* Size of an descriptor in the queue */
	u16 id;				/* Reserved                           */
	u16 tail,head;			/* queue tail and head pointer info.  */
	bool created;			/* status of the queue.               */
	u32 reg_offset;			/* head register offset for desc_queue
					 * tail for completion queue          */
	atomic_t used;			/* Num of valid elements in the queue */
};
struct indus_rx_ob_queue {
	struct indus_queue_info q;
	struct napi_struct napi;
};
struct indus_rx_eb_desc {	/* Rx empty buffer descriptor                 */
	u32 pa_low;		/* packet address low                         */
	u32 pa_high;		/* packet address high                        */
	u16 buf_size;		/* size of the packet buf                     */
	u16 rsvd;		/* reserved                                   */
	u32 opaque;		/* opaque we will get the same values in rx_ob*/
};
struct indus_rx_ob_desc {	/* Rx occupied buffer descriptor              */
	u16 w3;			/* index */
	u16 rsvd;
	u32 opaque;
	u32 dw2;
	u16 buf_len;
	u16 vlan_tag;
	u32 dw1;		/* RSS hash frag */
	u32 dw0;		/* Header Length */
};
struct indus_tx_desc {	/* Tx data buffer descriptor                  */
	u32 pa_low;		/* packet address low                         */
	u32 pa_high;		/* packet address high                        */
	u16 cmd;
	u16 vlan_tag;
	u16 buf_len;
	u16 dw1;
	u32 dw0;
	u32 rsvd;
};
struct indus_tx_cb_desc {
	u32 dw0;
	u32 rsvd;
};
struct indus_rx_page_info {
	struct page *page;
	dma_addr_t bus;
	u16 page_offset;
	bool last_page_user;
};
struct indus_rx_obj {
	struct indus_queue_info eb_q;		/* empty buffer queue         */
	struct indus_rx_ob_queue ob_q;		/* occupied buffer queue      */
	struct indus_rx_page_info page_info[INDUS_RX_Q_LEN];
};
struct indus_tx_obj {
	struct indus_queue_info q;		/* data buffer queue          */
	struct indus_queue_info cq;		/* Completion buffer queue    */
	struct sk_buff *sent_skb_list[INDUS_TX_Q_LEN];
};
/******************************************************************************
 * Net device statistics
 *****************************************************************************/
struct indus_drvr_stats {
	u32 indus_tx_reqs ;	/* num Of TX requests initiated               */
	u32 indus_tx_stops;	/* num of times TX Q was stoped               */
	u32 indus_fwd_reqs;	/* num of send reqs through forwarding i/f    */
	u32 indus_tx_wrbs;	/* num of tx WRBs used                        */
	u32 indus_tx_events;	/* num of tx compleation events               */
	u32 indus_tx_compl;	/* num of tx completion entries processed     */
	ulong indus_tx_jiffies;	/* used for scaling interrupts                */

	u64 indus_tx_bytes;
	u64 indus_tx_bytes_prev;
	u64 indus_tx_pkts;
	u64 indus_tx_rate;

	u32 cache_barrier[16];

	u32 indus_ethrx_post_fail;	/* num of ethrx buffer alloc failures */
	u32 indus_rx_polls;	/* num of times NAPI called poll function     */
	u32 indus_rx_events;	/* num of ucast rx completion events          */
	u32 indus_rx_compl;	/* num of rx completion entries processed     */
	ulong indus_rx_jiffies;

	u64 indus_rx_bytes;
	u64 indus_rx_bytes_prev;
	u64 indus_rx_pkts;
	u32 indus_rx_rate;


	u32 indus_802_3_dropped_frames;	/* number of non ether type II frames
			dropped where frame len > length field of Mac Hdr     */

	u32 indus_802_3_malformed_frames;	/* number of non ether type II
		frames malformed where in frame len < length field of Mac Hdr */

	u32 indus_rxcp_err;	/* Num rx completion entries w/ err set       */
	ulong rx_fps_jiffies;	/* jiffies at last FPS calc                   */
	u32 indus_rx_frags;
	u32 indus_prev_rx_frags;
	u32 indus_rx_fps;	/* Rx frags per second                        */

	/* newly added */
	unsigned long indus_crc_error_cnt;
	unsigned long indus_rx_error_cnt;
	unsigned long indus_missing_packet_cnt;
	unsigned long indus_rx_len_err_cnt;
	unsigned long indus_rx_uni_drop_cnt;
	unsigned long indus_rx_bc_drop_cnt;
	unsigned long indus_rx_mc_drop_cnt;
	unsigned long indus_odd_byte_error_cnt;
	unsigned long indus_rx_mcast_pkt_cnt;

};
/*******************************************************************************
 * Indus gbe adapter structure
 ******************************************************************************/
struct indus_adapter {
	struct pci_dev *pdev;		/* Our device's pci_device structure. */
	struct net_device *netdev;	/* Our net_device structure           */
	struct indus_drvr_stats stat;	/* Our driver statistics              */
	struct delayed_work work;	/* This is used to update the statistics
					for every 1 sec                       */

	u8 __iomem *devcfg;		/* remaped pci configuration address  */

	/* Rx Handling */
	struct indus_rx_obj rx;
	u32 big_page_size;	/* Compounded page size shared by rx wrbs     */

	/* Tx Handling */
	struct indus_tx_obj tx;

	bool msix_enabled;		/*  msix interrupts status            */
	bool rx_fc;			/*  recive flow control               */
	bool tx_fc;			/*  transmit flow control             */
	bool link_status;		/*  UP or DOWN                        */
	bool linkchange_detected;	/*  Link change interrupt is detected */
	bool rx_post_starved;	/* Zero rx frags have been posted to indus gbe*/
	bool isr_registered;		/* isr handler registration status    */
	u32 ie1_status;			/* present interrupt enable reg status*/
	u32 tx_thld;
	u32 rx_thld;
	struct indus_rx_ob_desc tmp;
};

static inline u32 indus_dma_high(dma_addr_t dma_addr)
{
	return cpu_to_le32(upper_32_bits(dma_addr));
}
static inline u32 indus_dma_low(dma_addr_t dma_addr)
{
	return cpu_to_le32(dma_addr & 0xFFFFFFFF);
}
extern int indus_set_dcb_ctrl_reg(struct indus_adapter *adapter,int offset,
					struct indus_queue_info *q);
extern void indus_queue_head_update(struct indus_adapter *adapter,
						struct indus_queue_info *q);
extern void indus_queue_tail_update(struct indus_adapter *adapter,
						struct indus_queue_info *q);

extern void indus_set_queue_thlds(struct indus_adapter *adapter);

extern void indus_set_mac_addr(struct indus_adapter *adapter);

#define drvr_stats(adapter)		(&adapter->stat)

static inline u32 MODULO(u16 val, u16 limit)
{
	return val % limit;
}
static inline void index_adv(u16 *index, u16 val, u16 limit)
{
	*index = MODULO((*index + val), limit);
}

static inline void index_inc(u16 *index, u16 limit)
{
	*index = MODULO((*index + 1), limit);
}
static inline void *queue_head_node(struct indus_queue_info *q)
{
	return q->dma_mem.va + q->head * q->entry_size;
}

static inline void *queue_tail_node(struct indus_queue_info *q)
{
	return q->dma_mem.va + q->tail * q->entry_size;
}

static inline void queue_head_inc(struct indus_queue_info *q)
{
	index_inc(&q->head, q->len);
}

static inline void queue_tail_inc(struct indus_queue_info *q)
{
	index_inc(&q->tail, q->len);
}

#endif /* _INDUS_H_ */
