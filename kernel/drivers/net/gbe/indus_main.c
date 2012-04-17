/**
 * Copyright (c) 2011 Ineda sysems.  All rights reserved.
 *
 * INEDA SYSTEMS Pvt Ltd(ISPL) and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without license
 * agreement from INEDA SYSTEMS Pvt.Ltd(ISPL) is strictly prohibited.
 **/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include "indus.h"
#include "mdio.h"
#define GMAC_PROC_FS 1
int host;
#if GMAC_PROC_FS
struct proc_dir_entry *gmac;
struct proc_dir_entry *display_reg;
struct proc_dir_entry *trigger_poll;
#endif
module_param(host,int, 0774);
static DEFINE_PCI_DEVICE_TABLE(indus_dev_ids) = {
	{PCI_DEVICE(INDUS_VENDOR_ID,INDUS_DEVICE_ID)},
	{0}
};
MODULE_DEVICE_TABLE(pci,indus_dev_ids);
static struct indus_rx_ob_desc *indus_rx_ob_get(struct indus_queue_info *ob_q)
{
	struct indus_rx_ob_desc *ob_desc = queue_tail_node(ob_q);
	int valid =ob_desc->dw2 & INDUS_RX_OB_STATUS_DD_MASK;

	if(valid != INDUS_RX_OB_STATUS_DD_MASK)
		return NULL;

	queue_tail_inc(ob_q);
	return ob_desc;
}
static struct indus_rx_ob_desc *indus_rx_ob_get_test(struct indus_queue_info *ob_q)
{
	struct indus_rx_ob_desc *ob_desc = queue_tail_node(ob_q);
	int valid =ob_desc->dw2 & INDUS_RX_OB_STATUS_DD_MASK;

//	if(valid != INDUS_RX_OB_STATUS_DD_MASK)
//		return NULL;

	queue_tail_inc(ob_q);
	return ob_desc;
}

static inline struct page *indus_alloc_pages(u32 size)
{
	gfp_t alloc_flags = GFP_ATOMIC;
	u32 order = get_order(size);
	if (order > 0)
		alloc_flags |= __GFP_COMP;
	return alloc_pages(alloc_flags, order);
}
/**
 * Determine number of data buffer descriptor entries needed to xmit data
 * in an skb
 * */
static u32 tx_desc_count(struct sk_buff *skb)
{
	u32 cnt = (skb->len > skb->data_len);

	cnt += skb_shinfo(skb)->nr_frags;

	return cnt;
}

/**
 * Allocate a page, split it to fragments of size RX_FRAG_SIZE and post as
 * receive buffers to indusgb
 */
static void indus_post_rx_frags(struct indus_adapter *adapter)
{
	struct indus_rx_page_info *page_info_tbl = adapter->rx.page_info;
	struct indus_queue_info *rx_q = &adapter->rx.eb_q;

	struct indus_rx_page_info *page_info = NULL, *prev_page_info = NULL;
	struct page *pagep = NULL;
	struct indus_rx_eb_desc *rx_eb = NULL;
	dma_addr_t page_dmaaddr = 0, frag_dmaaddr = 0;
	u32 posted, page_offset = 0;

	page_info = &page_info_tbl[rx_q->head];
	for (posted = 0; (posted < MAX_RX_POST || pagep) && !page_info->page;
								posted++) {
		if(!pagep){
			pagep = indus_alloc_pages(adapter->big_page_size);
			if (unlikely(!pagep)) {
				drvr_stats(adapter)->indus_ethrx_post_fail++;
				break;
			}
			page_dmaaddr = pci_map_page(adapter->pdev, pagep, 0,
						adapter->big_page_size,
						PCI_DMA_FROMDEVICE);
			page_info->page_offset = 0;
		}
		else{
			get_page(pagep);
			page_info->page_offset = page_offset + RX_FRAG_SIZE;
		}
		page_offset = page_info->page_offset;
		page_info->page = pagep;
		page_info->bus = page_dmaaddr;
		frag_dmaaddr = page_dmaaddr + page_info->page_offset;

		/* Filling info in rx empty buff descriptor                   */
		rx_eb = queue_head_node(rx_q);
		rx_eb->pa_high = indus_dma_high(frag_dmaaddr);
		rx_eb->pa_low = indus_dma_low(frag_dmaaddr);
		rx_eb->buf_size = RX_FRAG_SIZE;

		/* Any space left in the current big page for another frag?   */
		if ((page_offset + RX_FRAG_SIZE + RX_FRAG_SIZE) >
					adapter->big_page_size) {
			pagep = NULL;
			page_info->last_page_user = true;
		}
		prev_page_info = page_info;
		queue_head_inc(rx_q);
		page_info = &page_info_tbl[rx_q->head];
		atomic_inc(&rx_q->used);
		if(!(atomic_read(&rx_q->used) < INDUS_QUEUE_MAX_POST))
			break;
	}
	if (pagep)
		prev_page_info->last_page_user = true;
	if (posted) {
		indus_queue_head_update(adapter,rx_q);
	} else if (atomic_read(&rx_q->used) == 0) {
		/* Let indus_worker replenish when memory is available */
		adapter->rx_post_starved = true;
	}
}

static void unmap_tx_frag(struct pci_dev *pdev,
			struct indus_tx_desc *tx_d, bool map_single)
{
	dma_addr_t dma;
	dma = (u64)tx_d->pa_high << 32 | (u64)tx_d->pa_low;
	if (tx_d->buf_len){
		if(map_single)
			pci_unmap_single(pdev,dma,tx_d->buf_len,
							PCI_DMA_TODEVICE);
		else
			pci_unmap_page(pdev,dma,tx_d->buf_len,
							PCI_DMA_TODEVICE);
	}
}
static void tx_compl_clean(struct indus_adapter *adapter, u16 index)
{
	struct indus_queue_info *tx_q = &adapter->tx.q;
	bool unmap_skb_hdr = true;
	struct sk_buff *sent_skb;
	u16 cur_index,cnt = 0;
	struct indus_tx_desc *tx_d;
	printk("ENTERED INTO TX COMPL CLEAN");

	sent_skb = adapter->tx.sent_skb_list[tx_q->tail];
	BUG_ON(!sent_skb);
	adapter->tx.sent_skb_list[tx_q->tail] = NULL;

	do {
		cur_index = tx_q->tail;
		tx_d = queue_tail_node(tx_q);
		unmap_tx_frag(adapter->pdev,tx_d,(unmap_skb_hdr&
					skb_headlen(sent_skb)));
		unmap_skb_hdr = false;
		cnt++;
		queue_tail_inc(tx_q);
	}while(cur_index != index);
	atomic_sub(cnt,&tx_q->used);
	dev_kfree_skb_any(sent_skb);
	printk("EXITED FROM TX COMPL CLEAN");
}

static void indus_tx_q_clean(struct indus_adapter *adapter)
{
	struct indus_queue_info *tx_q = &adapter->tx.q;
	struct sk_buff *sent_skb;
	u16 end_idx = 0;
	while (atomic_read(&tx_q->used)) {
		sent_skb = adapter->tx.sent_skb_list[tx_q->tail];
		end_idx = tx_q->tail;
		index_adv(&end_idx,tx_desc_count(sent_skb) - 1, tx_q->len);
		tx_compl_clean(adapter, end_idx);
	}

}
/**
 * getting phy info link speed and configuring
 */
static int indus_link_status_query(struct indus_adapter *adapter,
			bool *link_up, u8 *mac_speed,u16 *link_speed)
{
	return 0;
}
static void indus_link_status_update(struct indus_adapter *adapter,
					bool link_up)
{
	struct net_device *netdev = adapter->netdev;
	if (adapter->link_status != link_up) {
		if (link_up) {
			netif_start_queue(netdev);
			netif_carrier_on(netdev);
			printk(KERN_INFO "%s: Link up\n", netdev->name);
		} else {
			netif_stop_queue(netdev);
			netif_carrier_off(netdev);
			printk(KERN_INFO "%s: Link down\n", netdev->name);
		}
		adapter->link_status = link_up;
	}
}
static void indus_mii_event(struct indus_adapter *adapter)
{

}
static void indus_rx_event(struct indus_adapter *adapter, int q_id)
{
	struct indus_rx_obj *rx = &adapter->rx;
	adapter->ie1_status = adapter->ie1_status&(0xFFFFFFFF^(1<<(q_id+3)));
	rx->ob_q.q.id = q_id;
	printk("KERN_INFO DISABLING RX INTERRUPTS %x\n",adapter->ie1_status);
	iowrite32(adapter->ie1_status, adapter->devcfg+INDUS_IE1_OFFSET);
	iowrite32((1<<(q_id+3)),adapter->devcfg+INDUS_INT_CLR1_OFFSET);
	napi_schedule(&rx->ob_q.napi);
}
static struct indus_tx_cb_desc *indus_tx_cb_get(struct indus_queue_info
									*tx_cq)
{
	struct indus_tx_cb_desc * cb_desc = queue_tail_node(tx_cq);
	bool valid = false;
	valid =  GET_FIELD(cb_desc->dw0, INDUS_TX_CB_DESC_VALID_SHIFT,
						INDUS_TX_CB_DESC_VALID_MASK);
	if(!valid)
		return NULL;
	queue_tail_inc(tx_cq);
	return cb_desc;

}

static void indus_dump_cb_q(struct indus_queue_info *tx_cq)
{
	struct indus_tx_cb_desc * cb_desc = queue_tail_node(tx_cq);
	int i;
	bool valid = false;
	u32 index;
	for (i=0; i<1023 ;i++) {
		cb_desc =  (struct indus_tx_cb_desc *) 
			tx_cq->dma_mem.va + i * tx_cq->entry_size;
		valid =  GET_FIELD(cb_desc->dw0, INDUS_TX_CB_DESC_VALID_SHIFT,
						INDUS_TX_CB_DESC_VALID_MASK);
		if(valid) {
			index = GET_FIELD(cb_desc->dw0, INDUS_TX_CB_DESC_VALID_SHIFT,
				INDUS_TX_CB_DESC_INDEX_MASK);
			printk("Valid = %d index = %d\n",
				valid, index);
		}
		valid = false;
	}
	return;
}

static void indus_dump_tx_q(struct indus_queue_info *tx_q)
{
	u8 *tx_desc = tx_q->dma_mem.va;
	int i;
	printk("TX DESC DUMP........ \n");
	for (i = 0; (void *) (tx_desc  + i)
			< tx_q->dma_mem.va + sizeof(struct indus_tx_desc) ; i++)
		printk( " %2x ", *(tx_desc+i));
	printk("\n");
	return;
}
static void indus_dump_rx_q(struct indus_adapter *adapter)
{
	int regval,i=0;
	struct indus_rx_ob_desc *ob_desc = NULL;
	regval = ioread32(adapter->devcfg + INDUS_RX_OB0_TAIL_OFFSET);
	struct indus_queue_info *rx_q = &adapter->rx.ob_q.q;



	u8 *rx_desc = &adapter->tmp;

	printk("\n");
	printk(" Rx tail in software %x\n",rx_q->tail);
	printk("\n");
	for(i=0;i<24;i++)
		printk(" %2x ", *(rx_desc + i));
	
	printk("\n");


	printk(" Rx tail in software %x\n",rx_q->tail);
	ob_desc = indus_rx_ob_get_test(rx_q);

	rx_desc = ob_desc;
	printk("\n");
	for(i=0;i<24;i++)
		printk(" %2x ", *(rx_desc + i));
	
	printk("\n");
	printk("Rx tail in software %x\n",rx_q->tail);
	ob_desc = indus_rx_ob_get_test(rx_q);
	rx_desc = ob_desc;
	printk("\n");
	for(i=0;i<24;i++)
		printk(" %2x ", *(rx_desc + i));
	
	printk("\n");

	return;
}

static void indus_tx_cb_reset(struct indus_tx_cb_desc *ob_desc)
{
	memset(ob_desc, 0, sizeof(*ob_desc));
}

static void indus_tx_event(struct indus_adapter *adapter, int q_id)
{

	struct indus_queue_info *cq = &adapter->tx.cq;
	struct indus_queue_info *txq = &adapter->tx.q;
	struct indus_tx_cb_desc *cb_desc = NULL;
	u16 index = -1,tx_cnt = 0;

	while (1) {
		cb_desc = indus_tx_cb_get(cq);
		if (!cb_desc)
			break;
		index = GET_FIELD(cb_desc->dw0, INDUS_TX_CB_DESC_INDEX_SHIFT,
				INDUS_TX_CB_DESC_INDEX_MASK);
		tx_compl_clean(adapter, index);
		indus_tx_cb_reset(cb_desc);
		tx_cnt++;
	}
	if(tx_cnt){
		indus_queue_tail_update(adapter, cq);
		drvr_stats(adapter)->indus_tx_events++;
		drvr_stats(adapter)->indus_tx_compl += tx_cnt;

		/* As Tx wrbs have been freed up, wake up netdev queue if
		 * it was stopped due to lack of tx wrbs.
		 */
		if (netif_queue_stopped(adapter->netdev) &&
			atomic_read(&txq->used) < txq->len / 2) {
			netif_wake_queue(adapter->netdev);
		}


	}

}
static irqreturn_t indus_intx(int irq, void *dev)
{
	struct indus_adapter *adapter = dev;
	u32 isr = 0, q_id = 0;
	//u32 regval;
	static int val = 0;
	static int val_rx = 0;
	isr = ioread32(adapter->devcfg + INDUS_INT_STS1_OFFSET);
	printk("Interrupt received %x \n", isr);
	if(isr == 0xffffffff)
	{
		return IRQ_HANDLED;
	}
	//regval = ioread32(adapter->devcfg + INDUS_TX_EB_TAIL_OFFSET);
	//printk("TX QUEUE TAIL = %x \n", regval);
	//regval = ioread32(adapter->devcfg + INDUS_TX_EB_HEAD_OFFSET);
	//printk("TX QUEUE HEAD = %x \n", regval);
	//regval = ioread32(adapter->devcfg + INDUS_TX_OB_HEAD_OFFSET);
	//printk("TX COMPL QUEUE HEAD = %x \n", regval);
	//regval = ioread32(adapter->devcfg + INDUS_TX_OB_TAIL_OFFSET);
	//printk("TX COMPL QUEUE TAIL = %x \n", regval);

	//regval = ioread32(adapter->devcfg + INDUS_RIS1_OFFSET);
	//printk(" RAW Interrupt received %x \n",regval);

	if (!isr)
		return IRQ_NONE;
	if (isr &  INDUS_INT_STS1_RX_QUEUE) {
		q_id = 0;
		printk(" Rx Interrupt received Enter\n");
		printk(" RX INT Count %d\n",val_rx++);		
		iowrite32(8,adapter->devcfg + INDUS_INT_CLR1_OFFSET);
		/*
		 * FIXME chage the q_id appropriatly
		 */
		indus_rx_event(adapter,q_id);
		printk(" Rx Interrupt received Exit\n");
	} else if (isr & INDUS_INT_STS1_TX_Q_CMPLT) {
		//printk(" tx Interrupt received ENtered\n");
		printk(" INT Count %d\n",val++);
		iowrite32(1,adapter->devcfg + INDUS_INT_CLR1_OFFSET);
		indus_tx_event(adapter, 0);
		//printk(" tx Interrupt received exited\n");
		//regval = ioread32(adapter->devcfg + INDUS_RIS1_OFFSET);
		//printk(" RAW Interrupt received %x \n",regval);
		//isr = ioread32(adapter->devcfg + INDUS_INT_STS1_OFFSET);
		printk("Interrupt received %x \n", isr);
	} else if (isr& INDUS_INT_STS1_LSCD){
		/*
		 * TODO PHASE 1 Link state Detection interrupt
		 */
		printk(" LSCD Interrupt received \n");
	} else if (isr & INDUS_INT_STS1_MII){
		//indus_mii_event(adapter);
		printk(" MII Interrupt received \n");
	}
	return IRQ_HANDLED;
}
/**
 * 1) Disabling/Enable interrupts using interrupt Enable Registers
 *
 */
static void indus_intr_set(struct indus_adapter *adapter, bool enable)
{
	int reg_val = 0;
	if(enable){
		printk("Enabling Interrupts %x\n",INDUS_IE1_ENABLE);
		iowrite32(INDUS_IE1_ENABLE,
					adapter->devcfg+INDUS_IE1_OFFSET);
		adapter->ie1_status = INDUS_IE1_ENABLE;
		reg_val = ioread32(adapter->devcfg+INDUS_IE1_OFFSET);
		printk("Enable reg val = %x\n",reg_val);
		
	}else{
		iowrite32(INDUS_IE1_DISABLE,
					adapter->devcfg+INDUS_IE1_OFFSET);
		adapter->ie1_status = INDUS_IE1_DISABLE;
	}
}
/**
 * TODO PHASE 2 MSI registration
 */
static int indus_msix_register(struct indus_adapter *adapter)
{
	return -1;
}
static void indus_msix_unregister(struct indus_adapter *adapter)
{
}

static int indus_irq_register(struct indus_adapter *adapter)
{
	struct net_device *netdev = adapter->netdev;
	int status;

	if (adapter->msix_enabled) {
		status = indus_msix_register(adapter);
		if (status == 0)
			goto done;
	}
	netdev->irq = adapter->pdev->irq;
	status = request_irq(netdev->irq, indus_intx, IRQF_SHARED, netdev->name,
			adapter);
	if (status) {
		dev_err(&adapter->pdev->dev,
			"INTx request IRQ failed - err %d\n", status);
		return status;
	}
done:
	adapter->isr_registered = true;
	return 0;
}
static void indus_irq_unregister(struct indus_adapter *adapter)
{
	struct net_device *netdev = adapter->netdev;

	if (!adapter->isr_registered)
		return;

	/* INTx */
	if (!adapter->msix_enabled) {
		free_irq(netdev->irq, adapter);
		goto done;
	}

	/* MSIx */
	indus_msix_unregister(adapter);

done:
	adapter->isr_registered = false;

}
/******************************************************************************
 * net_device_ops functionalities
 *****************************************************************************/

static void db_fill(struct indus_tx_desc *tx_d, dma_addr_t busaddr,
			u16 len, u16 cmd)
{

	tx_d->pa_high = indus_dma_high(busaddr);
	tx_d->pa_low = indus_dma_low(busaddr);
	tx_d->cmd = cmd;
	tx_d->buf_len = len;
	printk("TX     :     tx_d->pa_high   :%x \n", tx_d->pa_high);
	printk("TX     :     tx_d->pa_low    :%x \n", tx_d->pa_low);
	printk("TX     :     tx_d->cmd       :%x \n", tx_d->cmd);
	printk("TX     :     tx_d->buf_len   :%x \n", tx_d->buf_len);
}
static u32 make_tx_dbs(struct indus_adapter *adapter, struct sk_buff *skb,
					u32 db_cnt)
{
	dma_addr_t busaddr;
	int i, copied = 0;
	struct pci_dev *pdev = adapter->pdev;
	struct indus_queue_info *tx_q = &adapter->tx.q;
	struct indus_tx_desc *tx_d;
	bool map_single = false;
	u16 map_head = tx_q->head;
	u16 cmd = 0;
	if (skb->len > skb->data_len) {
		u16 len = skb_headlen(skb);
		if(db_cnt == 1)
			cmd |= INDUS_TX_DESC_CMD_EOP;
		cmd |= INDUS_TX_DESC_CMD_SOP;
		cmd |= INDUS_TX_DESC_CMD_CRC;
		cmd |= INDUS_TX_DESC_CMD_PE;
		/**
		 * CMD info filling
		 */
		busaddr = pci_map_single(pdev, skb->data, len,
					 PCI_DMA_TODEVICE);
		if (pci_dma_mapping_error(pdev, busaddr))
			goto dma_err;
		map_single = true;
		tx_d = queue_head_node(tx_q);
		db_fill(tx_d, busaddr,len,cmd);
		queue_head_inc(tx_q);
		copied += len;
		cmd = 0;
	}
	for (i = 0; i < skb_shinfo(skb)->nr_frags; i++) {
		struct skb_frag_struct *frag = &skb_shinfo(skb)->frags[i];
		busaddr = pci_map_page(pdev, frag->page,
					frag->page_offset,
					frag->size, PCI_DMA_TODEVICE);
		if (pci_dma_mapping_error(pdev, busaddr))
			goto dma_err;
		if(!((i+1) < skb_shinfo(skb)->nr_frags))
			cmd |= INDUS_TX_DESC_CMD_EOP;
		cmd |= INDUS_TX_DESC_CMD_CRC;
		/**
		 * Configure command descriptor;
		 */
		tx_d = queue_head_node(tx_q);
		db_fill(tx_d, busaddr,frag->size,cmd);
		queue_head_inc(tx_q);
		copied += frag->size;
		cmd = 0;
	}
	return copied;
dma_err:
	tx_q->head = map_head;
	while (copied) {
		tx_d = queue_head_node(tx_q);
		unmap_tx_frag(pdev, tx_d, map_single);
		map_single = false;
		copied -= tx_d->buf_len;
		queue_head_inc(tx_q);
	}
	return copied;
}
static void indus_tx_stats_update(struct indus_adapter *adapter,
			u32 db_cnt, u32 copied, u32 gso_segs, bool stopped)
{
	struct indus_drvr_stats *stats = drvr_stats(adapter);
	struct net_device_stats *n_stats = &adapter->netdev->stats;
	stats->indus_tx_reqs++;
	stats->indus_tx_wrbs += db_cnt;
	stats->indus_tx_bytes += copied;
	stats->indus_tx_pkts += (gso_segs ? gso_segs : 1);
	if (stopped)
		stats->indus_tx_stops++;

	n_stats->tx_bytes = stats->indus_tx_bytes;
	n_stats->tx_packets = stats->indus_tx_pkts;
}
static void print_skb(struct sk_buff *skb)
{
	int i = 0;
	printk("Printing the packet info \n");
	for (i = 0; i < skb->len; i++)
	{
		printk("%8x ",*(skb->data+i));
	}
	printk("\nEnd of packet \n");
}
static netdev_tx_t indus_start_xmit(struct sk_buff *skb,
					struct net_device *dev)
{
	struct indus_adapter *adapter = netdev_priv(dev);
	struct indus_tx_obj *tx = &adapter->tx;
	struct indus_queue_info *tx_q = &tx->q;

	u32 db_cnt = 0, copied = 0;
	u32 start = tx_q->head;
	bool stopped = false;
	printk("XMIT INVOKED ...\n");

//	static int val = 0;
//	if(val > 0)
//		return NETDEV_TX_OK;
//	val++;

	db_cnt = tx_desc_count(skb);

	copied = make_tx_dbs(adapter, skb, db_cnt);
	if (copied) {
		/* record the sent skb in the sent_skb table */
		BUG_ON(tx->sent_skb_list[start]);
		tx->sent_skb_list[start] = skb;

		/* Ensure tx_q has space for the next skb; Else stop the queue
		 * so that we serialze the
		 * tx compls of the current transmit which'll wake up the queue
		 */
		atomic_add(db_cnt, &tx_q->used);
		if ((INDUS_MAX_TX_FRAG_COUNT + atomic_read(&tx_q->used)) >=
								tx_q->len) {
			netif_stop_queue(dev);
			stopped = true;
		}
		indus_queue_head_update(adapter,tx_q);
		indus_tx_stats_update(adapter, db_cnt, copied,
				skb_shinfo(skb)->gso_segs, stopped);
	}else{
		tx_q->head = start;
		dev_kfree_skb_any(skb);
	}
	//indus_dump_cb_q(&tx->cq);
	//indus_dump_tx_q(&tx->q);
	return NETDEV_TX_OK;
}
static void indus_rx_stats_update(struct indus_adapter *adapter,
					u32 pktsize, u16 numfrags)
{
	struct indus_drvr_stats *stats = drvr_stats(adapter);
	struct net_device_stats *n_stats = &adapter->netdev->stats;

	stats->indus_rx_compl++;
	stats->indus_rx_frags += numfrags;
	stats->indus_rx_bytes += pktsize;
	stats->indus_rx_pkts++;

	n_stats->rx_packets = stats->indus_rx_pkts;
	n_stats->rx_bytes = stats->indus_rx_bytes;
}


static void indus_rx_ob_reset(struct indus_adapter *adapter,struct indus_rx_ob_desc * ob_desc)
{
	printk("Reseting index is %x \n",((ulong)ob_desc - (ulong)adapter->rx.ob_q.q.dma_mem.va)/24);
	memset(ob_desc, 0, sizeof(*ob_desc));
}
static struct indus_rx_page_info *get_rx_page_info(
						struct indus_adapter *adapter,
						int index)
{
	struct indus_rx_page_info *rx_page_info;
	struct indus_queue_info *rx_q = &adapter->rx.eb_q;

	rx_page_info = &adapter->rx.page_info[index];
	BUG_ON(!rx_page_info->page);

	if (rx_page_info->last_page_user) {
		pci_unmap_page(adapter->pdev, rx_page_info->bus,
			adapter->big_page_size, PCI_DMA_FROMDEVICE);
		rx_page_info->last_page_user = false;
	}
	atomic_dec(&rx_q->used);
	return rx_page_info;
}
static void indus_fill_skb(struct indus_adapter *adapter, struct sk_buff *skb,
						struct indus_queue_info *q)
{
	bool end_pkt = false;
	int j = 0;
	struct indus_rx_ob_desc *ob_desc = NULL;
	struct indus_rx_page_info *page_info;

	while(!end_pkt){
		ob_desc = indus_rx_ob_get(q);
		BUG_ON(!ob_desc);;
		end_pkt = (ob_desc->dw2 >> INDUS_RX_OB_STATUS_EOP_SHIFT)&
						INDUS_RX_OB_STATUS_EOP_MASK;
		page_info = get_rx_page_info(adapter,
					(ob_desc->w3&INDUS_RX_OB_INDEX_MASK));
		if (page_info->page_offset == 0) {
			/* Fresh page                                         */
			j++;
			skb_shinfo(skb)->frags[j].page = page_info->page;
			skb_shinfo(skb)->frags[j].page_offset =
							page_info->page_offset;
			skb_shinfo(skb)->frags[j].size = 0;
			skb_shinfo(skb)->nr_frags++;
		} else {
			put_page(page_info->page);
		}

		skb_shinfo(skb)->frags[j].size += ob_desc->buf_len;
		skb->len += ob_desc->buf_len;
		skb->data_len += ob_desc->buf_len;
		BUG_ON(skb->len > INDUS_MAX_JUMBO_FRAME_SIZE);
		page_info->page = NULL;
		adapter->tmp = *ob_desc;
		indus_rx_ob_reset(adapter,ob_desc);
	}
}
static void process_header(struct indus_adapter *adapter, struct sk_buff *skb,
					struct indus_rx_ob_desc *ob_desc)
{
	u32 hdr_len;
	u8 *start;
	struct indus_rx_page_info *page_info = NULL;

	page_info = get_rx_page_info(adapter,
					(ob_desc->w3&INDUS_RX_OB_INDEX_MASK));
	start = page_address(page_info->page) + page_info->page_offset;
	prefetch(start);

	hdr_len = min((u16)INDUS_HDR_LEN, ob_desc->buf_len);
	memcpy(skb->data, start, hdr_len);
	skb->len = ob_desc->buf_len;

	if (ob_desc->buf_len <= INDUS_HDR_LEN) { /* tiny packet */
		/* Complete packet has now been moved to data */
		put_page(page_info->page);
		skb->data_len = 0;
		skb->tail += ob_desc->buf_len;
	} else {
		skb_shinfo(skb)->nr_frags = 1;
		skb_shinfo(skb)->frags[0].page = page_info->page;
		skb_shinfo(skb)->frags[0].page_offset =
					page_info->page_offset + hdr_len;
		skb_shinfo(skb)->frags[0].size = ob_desc->buf_len - hdr_len;
		skb->data_len = ob_desc->buf_len - hdr_len;
		skb->tail += hdr_len;
	}
	page_info->page = NULL;
	adapter->tmp = *ob_desc;
	indus_rx_ob_reset(adapter,ob_desc);
}

static void indus_rx_pkt_discard(struct indus_adapter *adapter,
					struct indus_rx_ob_desc *ob_desc,
					struct indus_queue_info *q)
{
	struct indus_rx_page_info *page_info;
	int pkt_len = 0;
	bool end_pkt = (ob_desc->dw2 >> INDUS_RX_OB_STATUS_EOP_SHIFT)&
			INDUS_RX_OB_STATUS_EOP_MASK;
	int index = ob_desc->w3&INDUS_RX_OB_INDEX_MASK;
	pkt_len = ob_desc->buf_len;
	printk("packet discard\n");
	if(end_pkt){
		page_info = get_rx_page_info(adapter, index);
		put_page(page_info->page);
		memset(page_info, 0, sizeof(*page_info));
		goto done;
	}
	while(!end_pkt){
		ob_desc = indus_rx_ob_get(q);
		if(!ob_desc)
			break;
		end_pkt = (ob_desc->dw2 >> INDUS_RX_OB_STATUS_EOP_SHIFT)&
						INDUS_RX_OB_STATUS_EOP_MASK;
		pkt_len += ob_desc->buf_len;
		BUG_ON(pkt_len > INDUS_MAX_JUMBO_FRAME_SIZE);
		index = ob_desc->w3&INDUS_RX_OB_INDEX_MASK;
		page_info = get_rx_page_info(adapter, index);
		put_page(page_info->page);
		memset(page_info, 0, sizeof(*page_info));
		adapter->tmp = *ob_desc;
		indus_rx_ob_reset(adapter,ob_desc);
	}
done:
	indus_queue_tail_update(adapter,q);
	return;
}
static int is_pkt_complete(struct indus_adapter *adapter,
					struct indus_queue_info *q)
{
	struct indus_rx_ob_desc *ob_desc = NULL;
	int cnt = 0,tail = q->tail;
	int valid = 0;
	bool end_pkt = false;
	int pkt_len = 0;
	while(!end_pkt){
		ob_desc = queue_tail_node(q);
		if(!ob_desc)
			goto err;
		valid = ob_desc->dw2 & INDUS_RX_OB_STATUS_DD_MASK;
		if(valid != INDUS_RX_OB_STATUS_DD_MASK)
			goto err;
		end_pkt = (ob_desc->dw2 >> INDUS_RX_OB_STATUS_EOP_SHIFT)&
					INDUS_RX_OB_STATUS_EOP_MASK;
		queue_tail_inc(q);
		pkt_len += ob_desc->buf_len;
		BUG_ON(pkt_len > INDUS_MAX_JUMBO_FRAME_SIZE);
		cnt++;
	}
	q->tail = tail;
	return cnt;
err:
	q->tail = tail;
	return 0;
}
/**
 * return 0  success
 *   err no  failure
 */
static int indus_process_rx_pkt(struct indus_adapter *adapter,
						struct indus_queue_info *q)
{
	struct indus_rx_ob_desc *ob_desc = NULL;
	struct sk_buff *skb = NULL;
	int rcvd_frags;

	rcvd_frags = is_pkt_complete(adapter,q);
	if(!rcvd_frags)
		return -ENOSPC;
	ob_desc = indus_rx_ob_get(q);
	BUG_ON(!ob_desc);

	skb = netdev_alloc_skb_ip_align(adapter->netdev, INDUS_HDR_LEN);
	if (unlikely(!skb)) {
		if (net_ratelimit())
			dev_warn(&adapter->pdev->dev,"skb alloc failed\n");
		//indus_rx_pkt_discard(adapter, ob_desc,q);
	}
	/* first buffer contains the header */
	process_header(adapter,skb,ob_desc);
	if(rcvd_frags > 1){
		printk(" RX more than one frag \n");
		indus_fill_skb(adapter,skb,q);
	}

	skb->ip_summed = CHECKSUM_NONE;
	skb->truesize = skb->len + sizeof(struct sk_buff);
	skb->protocol = eth_type_trans(skb, adapter->netdev);
	indus_queue_tail_update(adapter,q);
	netif_receive_skb(skb);
	indus_rx_stats_update(adapter, skb->len, rcvd_frags);
	return 0;
}
int indus_rx_poll(struct napi_struct *napi, int weight)
{
	struct net_device *dev = napi->dev;
	struct indus_adapter *adapter = netdev_priv(dev);
	struct indus_rx_ob_queue *rx_q =
			container_of(napi,struct indus_rx_ob_queue, napi);
	struct indus_queue_info *ob_q = &rx_q->q;
	int work_done,status;

	printk("Entered into poll \n");

	adapter->stat.indus_rx_polls++;
	for(work_done = 0; work_done < weight; work_done++){
		status = indus_process_rx_pkt(adapter,ob_q);
		if(status){
			dev_err(&adapter->pdev->dev,"failed to process pkt");
			break;
		}
	}
	/* Refill the queue */
	if(atomic_read(&adapter->rx.eb_q.used) < RX_FRAGS_REFILL_WM)
		indus_post_rx_frags(adapter);
	/* All consumed     */
	if (work_done < weight) {
		napi_complete(napi);
		adapter->ie1_status = adapter->ie1_status | (1<<(ob_q->id+3));
		/* unmasking the interrupt source */
		printk("KERN_INFO ENABLING RX INTERRUPTS %x\n",adapter->ie1_status);
		iowrite32(adapter->ie1_status,adapter->devcfg+INDUS_IE1_OFFSET);
	}
	printk("Exiting From poll \n");
	return work_done;
}
static struct net_device_stats* indus_get_stats(struct net_device *dev)
{
	return &dev->stats;
}
static int indus_open(struct net_device *netdev)
{
	struct indus_adapter *adapter = netdev_priv(netdev);
	struct indus_rx_obj *rx = &adapter->rx;

	bool link_up;
	int status;
	u8 mac_speed;
	u16 link_speed;


	/* First time posting */
	indus_post_rx_frags(adapter);

	napi_enable(&rx->ob_q.napi);

	indus_irq_register(adapter);

	indus_intr_set(adapter, true);

	status = indus_link_status_query(adapter, &link_up, &mac_speed,
					&link_speed);
	if (status)
		goto ret_sts;

	/**
	 * TODO Chage the after demo
	 */
	link_up = LINK_UP;
	indus_link_status_update(adapter, link_up);
	schedule_delayed_work(&adapter->work, msecs_to_jiffies(100));
	return 0;
ret_sts:
	return status;
}
static int indus_stop(struct net_device *netdev)
{
	struct indus_adapter *adapter = netdev_priv(netdev);
	struct indus_rx_obj *rx = &adapter->rx;
	struct indus_queue_info *tx_q = &adapter->tx.q;
	u16 timeo = 0;

	cancel_delayed_work_sync(&adapter->work);

	indus_link_status_update(adapter,LINK_DOWN);

	/* Wait for all pending tx completions to arrive so that
	 * all tx skbs are freed.
	 */
	do {
		if (atomic_read(&tx_q->used) == 0 || ++timeo > 200)
			break;
		mdelay(1);

	}while(true);

	if (atomic_read(&tx_q->used))
		dev_err(&adapter->pdev->dev, "%d pending tx-completions\n",
			atomic_read(&tx_q->used));

	/**
	 * TODO PHASE 1 Stop the controller dmaing data.
	 */
	indus_intr_set(adapter,false);

	if(adapter->msix_enabled){
		/**
		 * TODO  PHASE 2 synchronize the msix vector isrs.
		 */
	}else{
		synchronize_irq(netdev->irq);
	}
	indus_irq_unregister(adapter);

	napi_disable(&rx->ob_q.napi);

	/*free posted tx for which compls will never arrive */
	indus_tx_q_clean(adapter);
	return 0;
}
struct net_device_ops indus_netdev_ops = {
	.ndo_open	= indus_open,
	.ndo_stop	= indus_stop,
	.ndo_start_xmit	= indus_start_xmit,
	.ndo_get_stats	= indus_get_stats,
};

static u32 indus_calc_rate(u64 bytes, unsigned long ticks)
{
	u64 rate = bytes;

	do_div(rate, ticks / HZ);
	rate <<= 3;			/* bytes/sec -> bits/sec */
	do_div(rate, 1000000ul);	/* MB/Sec */

	return rate;
}
static void indus_tx_rate_update(struct indus_adapter *adapter)
{
	struct indus_drvr_stats *stats = drvr_stats(adapter);
	ulong now = jiffies;

	/* Wrapped around? */
	if (time_before(now, stats->indus_tx_jiffies)) {
		stats->indus_tx_jiffies = now;
		return;
	}
	/* Update tx rate once in two seconds */
	if ((now - stats->indus_tx_jiffies) > 2 * HZ) {
		stats->indus_tx_rate = indus_calc_rate((stats->indus_tx_bytes
						  - stats->indus_tx_bytes_prev),
					       (now - stats->indus_tx_jiffies));
		stats->indus_tx_jiffies = now;
		stats->indus_tx_bytes_prev = stats->indus_tx_bytes;
	}
}
static void indus_rx_rate_update(struct indus_adapter *adapter)
{
	struct indus_drvr_stats *stats = drvr_stats(adapter);
	ulong now = jiffies;

	/* Wrapped around */
	if (time_before(now, stats->indus_rx_jiffies)) {
		stats->indus_rx_jiffies = now;
		return;
	}

	/* Update the rate once in two seconds */
	if ((now - stats->indus_rx_jiffies) < 2 * HZ)
		return;

	stats->indus_rx_rate = indus_calc_rate(stats->indus_rx_bytes
					  - stats->indus_rx_bytes_prev,
					 now - stats->indus_rx_jiffies);
	stats->indus_rx_jiffies = now;
	stats->indus_rx_bytes_prev = stats->indus_rx_bytes;
}
static void indus_read_stats(struct indus_adapter *adapter)
{
	struct indus_drvr_stats *stats = drvr_stats(adapter);

	stats->indus_crc_error_cnt = ioread32(adapter->devcfg+
						INDUS_CRC_ERROR_CNT_OFFSET);

	stats->indus_rx_error_cnt =  ioread32(adapter->devcfg+
						INDUS_RX_ERROR_CNT_OFFSET);

	stats->indus_missing_packet_cnt = ioread32(adapter->devcfg+
					INDUS_MISSING_PACKET_CNT_OFFSET);

	stats->indus_rx_len_err_cnt = ioread32(adapter->devcfg+
					INDUS_RX_LEN_ERR_CNT_OFFSET);

	stats->indus_rx_uni_drop_cnt = ioread32(adapter->devcfg+
					INDUS_RX_UNI_DROP_CNT_OFFSET);

	stats->indus_rx_bc_drop_cnt = ioread32(adapter->devcfg+
					INDUS_RX_BC_DROP_CNT_OFFSET);

	stats->indus_rx_mc_drop_cnt = ioread32(adapter->devcfg+
					INDUS_RX_MC_DROP_CNT_OFFSET);

	stats->indus_odd_byte_error_cnt = ioread32(adapter->devcfg+
					INDUS_ODD_BYTE_ERROR_CNT_OFFSET);

	stats->indus_rx_mcast_pkt_cnt =  ioread32(adapter->devcfg+
					INDUS_RX_MCAST_PKT_CNT_OFFSET);
}
static void netdev_stats_update(struct indus_adapter *adapter)
{
	struct indus_drvr_stats *stats = drvr_stats(adapter);
	struct net_device_stats *n_stats = &adapter->netdev->stats;

	n_stats->rx_compressed 		= 0;
	n_stats->tx_compressed 		= 0;
	n_stats->rx_errors 		= stats->indus_crc_error_cnt +
					stats->indus_rx_error_cnt +
					stats->indus_missing_packet_cnt +
					stats->indus_rx_len_err_cnt ;
	n_stats->tx_errors		= 0;
	n_stats->rx_dropped		= stats->indus_rx_uni_drop_cnt +
						stats->indus_rx_bc_drop_cnt +
						stats->indus_rx_mc_drop_cnt +
						stats->indus_odd_byte_error_cnt;
	n_stats->tx_dropped		= 0;
	n_stats->multicast		= stats->indus_rx_mcast_pkt_cnt;
	n_stats->collisions		= 0;
	n_stats->rx_length_errors	= stats->indus_rx_len_err_cnt;
	n_stats->rx_crc_errors		= stats->indus_crc_error_cnt;
	n_stats->rx_fifo_errors 	= stats->indus_missing_packet_cnt;
	n_stats->tx_aborted_errors	= 0;
	n_stats->tx_carrier_errors	= 0;
	n_stats->tx_fifo_errors		= 0;
	n_stats->tx_heartbeat_errors 	= 0;
	n_stats->tx_window_errors	= 0;

}
void indus_worker (struct work_struct *work)
{
	struct indus_adapter *adapter =
			container_of(work, struct indus_adapter, work.work);

	//indus_read_stats(adapter);
	indus_tx_rate_update(adapter);
	indus_rx_rate_update(adapter);
	netdev_stats_update(adapter);
	if (adapter->rx_post_starved) {
		adapter->rx_post_starved = false;
		indus_post_rx_frags(adapter);
	}
	schedule_delayed_work(&adapter->work, msecs_to_jiffies(1000));
}

static int indus_queue_alloc (struct indus_adapter *adapter,
			struct indus_queue_info *q, u16 len, u16 entry_size)
{
	struct indus_dma_mem *mem = &q->dma_mem;

	memset(q, 0, sizeof(*q));
	q->len = len;
	q->entry_size = entry_size;
	mem->size = len * entry_size;
	mem->va = pci_alloc_consistent(adapter->pdev, mem->size, &mem->dma);
	if (!mem->va)
		return -ENOMEM;
	memset(mem->va, 0, mem->size);
	q->created = true;
	return 0;
}
static void indus_queue_free(struct indus_adapter *adapter,
			     struct indus_queue_info *q)
{
	struct indus_dma_mem *mem = &q->dma_mem;
	if (mem->va)
		pci_free_consistent(adapter->pdev, mem->size,
			mem->va, mem->dma);
	q->created = false;
}
static int setup_tx_queues(struct indus_adapter *adapter)
{
	struct indus_queue_info *db_q, *cb_q;
	int status ;
	u32 reg_val;
	db_q = &adapter->tx.q;
	cb_q = &adapter->tx.cq;
	status = indus_queue_alloc(adapter,db_q,INDUS_TX_Q_LEN,
					sizeof(struct indus_tx_desc));
	if(status)
		goto do_none;
	db_q->reg_offset = INDUS_TX_EB_HEAD_OFFSET;
	printk("TX Q offset = %p \n", &adapter->tx.q.reg_offset);
	printk("TX WRITE dma mem high = %x \n", indus_dma_high(db_q->dma_mem.dma));
	printk("TX WRITE dma mem low = %x \n", indus_dma_low(db_q->dma_mem.dma));
	status = indus_set_dcb_ctrl_reg(adapter,INDUS_TX_EB_ADDR_LOW_OFFSET,
									db_q);
	if(status)
		goto free_eb;
	reg_val = ioread32(adapter->devcfg + INDUS_TX_EB_ADDR_HIGH_OFFSET);
	printk("TX  READ dma mem high = %x \n", reg_val);
	reg_val = ioread32(adapter->devcfg + INDUS_TX_EB_ADDR_LOW_OFFSET);
	printk("TX  READ dma mem low = %x \n", reg_val);

	status = indus_queue_alloc(adapter,cb_q,INDUS_TX_Q_LEN,
					sizeof(struct indus_tx_cb_desc));
	if(status)
		goto free_eb;
	//indus_dump_cb_q(&adapter->tx.cq);
	cb_q->reg_offset = INDUS_TX_OB_TAIL_OFFSET;
	status = indus_set_dcb_ctrl_reg(adapter,INDUS_TX_OB_ADDR_LOW_OFFSET,
									cb_q);
	if(status)
		goto free_ob;
	return status;
free_ob:
	indus_queue_free(adapter,cb_q);
free_eb:
	indus_queue_free(adapter,db_q);
do_none:
	return status;
}
static int setup_rx_queues(struct indus_adapter *adapter)
{
	struct indus_queue_info *eb, *ob;
	int status;
	eb = &adapter->rx.eb_q;
	ob = &adapter->rx.ob_q.q;


	status = indus_queue_alloc(adapter,eb,INDUS_RX_Q_LEN,
					sizeof(struct indus_rx_eb_desc));
	if(status)
		goto do_none;
	eb->reg_offset = INDUS_RX_EB_HEAD_OFFSET;
	status = indus_set_dcb_ctrl_reg(adapter,INDUS_RX_EB_ADDR_LOW_OFFSET,
					eb);
	if(status)
		goto free_eb;
	status = indus_queue_alloc(adapter,ob,INDUS_RX_Q_LEN,
					sizeof(struct indus_rx_ob_desc));
	if(status)
		goto free_eb;
	ob->reg_offset = INDUS_RX_OB0_TAIL_OFFSET;
	status = indus_set_dcb_ctrl_reg(adapter,INDUS_RX_OB0_ADDR_LOW_OFFSET,
					ob);
	if(status)
		goto free_ob;
	return status;
free_ob:
	indus_queue_free(adapter,ob);
free_eb:
	indus_queue_free(adapter,eb);
do_none:
	return status;
}
static void clear_tx_queues(struct indus_adapter *adapter)
{
	struct indus_queue_info *db_q, *cb_q;
	db_q = &adapter->tx.q;
	cb_q = &adapter->tx.cq;
	if(db_q->created)
		indus_queue_free(adapter,db_q);
	if(cb_q->created)
		indus_queue_free(adapter,cb_q);
}
static void clear_rx_queues(struct indus_adapter *adapter)
{
	struct indus_queue_info *eb_q, *ob_q;
	eb_q = &adapter->rx.eb_q;
	ob_q = &adapter->rx.ob_q.q;
	if(eb_q->created)
		indus_queue_free(adapter,eb_q);
	if(ob_q->created)
		indus_queue_free(adapter,ob_q);
}
static int indus_setup(struct indus_adapter *adapter)
{
	int status;
	status = setup_tx_queues(adapter);
	if (status != 0)
		goto do_none;

	status = setup_rx_queues(adapter);
	if (status != 0)
		goto tx_q_destroy;

	return status;
tx_q_destroy:
	clear_tx_queues(adapter);
do_none:
	return status;
}
static void indus_clear(struct indus_adapter *adapter)
{
	clear_tx_queues(adapter);
	clear_rx_queues(adapter);
}

u32 mdio_read(struct indus_adapter *adapter, u16 reg_addr)
{
	u32 rdata,cmd = 0x60020000;
	cmd |= ((0x1F & MDIO_PHY_ADDR)<<23);
	cmd |= ((0x1F & reg_addr)<<18);

	iowrite32(cmd, adapter->devcfg + INDUS_MGMT_FRM_OFFSET);
	msleep(5000);
	rdata = ioread32(adapter->devcfg + INDUS_MGMT_FRM_OFFSET);
	msleep(1000);
	return (0x0000FFFF & rdata);
}

void mdio_write(struct indus_adapter *adapter, u16 reg_addr, u16 wdata)
{
	
	u32 cmd = 0x50020000;
	u32 val = 0;
	val = mdio_read(adapter,reg_addr);
	cmd |= ((0x1F & MDIO_PHY_ADDR)<<23);
	cmd |= ((0x1F & reg_addr)<<18);
	cmd |= val;
	cmd |= wdata;
	iowrite32(cmd, adapter->devcfg + INDUS_MGMT_FRM_OFFSET);
	msleep(5000);
}

void phy_config(struct indus_adapter *adapter)
{
	u16 data = 0;
	u16 mdio_rd = 0;
	
	printk(KERN_INFO "entered into the phy_config\n");

	data = MDIO_PHY_FULL_DPLX;
	data |= MDIO_PHY_100MBPS;
	
	mdio_write(adapter, MDIO_CTRL_REG, data);
	
	while (!(0x0004 & mdio_rd)) {
		if (net_ratelimit())
			printk(" mdio_rd %x\n", mdio_rd);
		mdio_rd = mdio_read(adapter, MDIO_STATUS_REG);
		msleep(2000);
	}
		
	msleep(5000);
	printk(KERN_INFO " stats value %x\n",mdio_read(adapter, MDIO_STATUS_REG));
	printk("\nPHY configured successfully %x \n\n",data);

	printk(KERN_INFO "exited from the phy_config\n");
}

static int indus_ctrl_init(struct indus_adapter *adapter)
{
	int status = 0;
	u32 gmac_cfg;
	u32 receive_ctrl = 0;
	adapter->devcfg = ioremap_nocache(pci_resource_start(adapter->pdev, 0),
					pci_resource_len(adapter->pdev, 0));
	if(adapter->devcfg == NULL){
		status = -ENOMEM;
		goto none;
	}
	/** receive control initi ************************/
	//receive_ctrl = ioread32(adapter->devcfg + INDUS_RECV_CTRL_OFFSET);
	//receive_ctrl |= (INDUS_RECV_CTRL_SBP | INDUS_RECV_CTRL_UPE |INDUS_RECV_CTRL_MPE | INDUS_RECV_CTRL_BCAST_EN);
	//iowrite32(receive_ctrl,adapter->devcfg + INDUS_RECV_CTRL_OFFSET);
	
	//L2 switch enabling

	// YLN FIXME doing Host0 only
        iowrite32(0x21, adapter->devcfg + INDUS_L2_CTRL_OFFSET);
	/******** configuring phy *********/
	if(!host)
	{
		iowrite32(8,adapter->devcfg + INDUS_MGMT_SPD_OFFSET);

		iowrite32(1, adapter->devcfg + INDUS_PHY_RESET_N_OFFSET);

		msleep(5000);
		//while(!(0x0004 & mdio_read(adapter, MDIO_STATUS_REG)));
	
		printk(KERN_INFO "before phy PHY control reg info after config %x\n",mdio_read(adapter, MDIO_CTRL_REG));
		printk(KERN_INFO " stats value %x\n",mdio_read(adapter, MDIO_STATUS_REG));

		//resetting the phy
		//mdio_write(adapter, MDIO_CTRL_REG, MDIO_PHY_RESET);
		//while(!(0x0004 & mdio_read(adapter, MDIO_STATUS_REG)));

		//printk(KERN_INFO "PHY control reg info before config %x\n",mdio_read(adapter, MDIO_CTRL_REG));
		//printk(KERN_INFO "PHY status reg info %x\n", mdio_read(adapter,  MDIO_STATUS_REG));
		//printk(KERN_INFO "PHY IDO reg info %x\n", mdio_read(adapter,  MDIO_PHY_ID0_REG));
		//printk(KERN_INFO "PHY ID1 reg info %x\n", mdio_read(adapter,  MDIO_PHY_ID1_REG));

		//phy_config
		//phy_config(adapter);

		//printk(KERN_INFO "PHY control reg info after config %x\n",mdio_read(adapter, MDIO_CTRL_REG));
		//resetting the phy
		//mdio_write(adapter, MDIO_CTRL_REG, MDIO_PHY_RESET);
		//while(!(0x0004 & mdio_read(adapter, MDIO_STATUS_REG)));
 	
		//printk(KERN_INFO " stats value %x\n",mdio_read(adapter, MDIO_STATUS_REG));
		//printk(KERN_INFO "After PHY control reg info after config %x\n",mdio_read(adapter, MDIO_CTRL_REG));
		//gmac config
		gmac_cfg = ioread32(adapter->devcfg + INDUS_GMAC_CTRL_OFFSET);

		printk(KERN_INFO " GMAC CTRL reg info %x\n",gmac_cfg);
		gmac_cfg |= (INDUS_GMAC_CTRL_FDEN | INDUS_GMAC_CTRL_MODE_MODE_MII |INDUS_GMAC_CTRL_SPEED_MODE_100MBPS);
		iowrite32(gmac_cfg, adapter->devcfg + INDUS_GMAC_CTRL_OFFSET);

		gmac_cfg = ioread32(adapter->devcfg + INDUS_GMAC_CTRL_OFFSET);
		printk(KERN_INFO " GMAC CTRL reg info %x\n",gmac_cfg);

		msleep(5000);
	
		//setting Go_MAC
		iowrite32(1, adapter->devcfg + INDUS_GO_MAC_OFFSET);
		msleep(5000);
	}
			
	/********************/	
	/**
	 * TODO PHASE 2 Enabling the msix to use further and
	 * set msix_enabled= true
	 */
	indus_set_mac_addr(adapter);
	indus_set_queue_thlds(adapter);
	pci_save_state(adapter->pdev);
	return status;
none:
	return status;
}
static void indus_ctrl_cleanup(struct indus_adapter *adapter)
{
	if(adapter->devcfg)
		iounmap(adapter->devcfg);

	/**
	 * TODO PHASE 1 Controler reset.
	 */
}
static void indus_netdev_init(struct net_device *netdev)
{
	struct indus_adapter *adapter = netdev_priv(netdev);
	netdev->features |= NETIF_F_SG;
	netdev->flags |= IFF_MULTICAST;

	adapter->big_page_size = (1 << get_order(RX_FRAG_SIZE)) * PAGE_SIZE;
	adapter->tx_thld = 1;
	adapter->rx_thld = 1;

	adapter->rx_fc = false;
	adapter->tx_fc = false;
	adapter->link_status = LINK_DOWN;
	adapter->linkchange_detected = false;

	adapter->rx_post_starved = false;

	adapter->ie1_status = 0;

	adapter->msix_enabled = false;
	adapter->isr_registered = false;

	INDUS_SET_DEVICE_OPS(netdev,&indus_netdev_ops);
	INDUS_SET_ETHTOOL_OPS(netdev,&indus_ethtool_ops);

	netif_napi_add(netdev,&adapter->rx.ob_q.napi,indus_rx_poll,
			INDUS_NAPI_WEIGHT);

	netif_carrier_off(netdev);
	netif_stop_queue(netdev);
}
#if GMAC_PROC_FS
int display_regs_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	struct indus_adapter *adapter = (struct indus_adapter *)data;
	u32 regval;
	regval = ioread32(adapter->devcfg + INDUS_RX_EB_HEAD_OFFSET);
	printk("RX EB HEAD = %x \n",regval);
	regval = ioread32(adapter->devcfg + INDUS_RX_EB_TAIL_OFFSET);
	printk("RX EB TAIL = %x \n",regval);
	regval = ioread32(adapter->devcfg + INDUS_RX_OB0_HEAD_OFFSET);
	printk("RX OB HEAD = %x \n", regval);
	regval = ioread32(adapter->devcfg + INDUS_RX_OB0_TAIL_OFFSET);
	printk("RX OB TAIL = %x \n", regval);

	regval = ioread32(adapter->devcfg + INDUS_TX_EB_HEAD_OFFSET);
	printk("TX EB HEAD = %x \n",regval);
	regval = ioread32(adapter->devcfg + INDUS_TX_EB_TAIL_OFFSET);
	printk("TX EB TAIL = %x \n",regval);
	regval = ioread32(adapter->devcfg + INDUS_TX_OB_HEAD_OFFSET);
	printk("TX OB HEAD = %x \n", regval);
	regval = ioread32(adapter->devcfg + INDUS_TX_OB_TAIL_OFFSET);
	printk("TX OB TAIL = %x \n", regval);

	regval = ioread32(adapter->devcfg + INDUS_RIS1_OFFSET);
	printk("After RIS = %x \n", regval);

	regval = ioread32(adapter->devcfg + INDUS_INT_STS1_OFFSET);
	printk("After STS = %x \n", regval);

	return 0;
}
int trigger_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	struct indus_adapter *adapter = (struct indus_adapter *)data;
/*	adapter->ie1_status = adapter->ie1_status&(0xFFFFFFFF^(1<<3));
	printk("KERN_INFO DISABLING RX INTERRUPTS %x\n",adapter->ie1_status);
	iowrite32(adapter->ie1_status, adapter->devcfg+INDUS_IE1_OFFSET);

	indus_rx_poll(&adapter->rx.ob_q.napi,1000);
*/
	//indus_dump_rx_q(adapter);
	napi_schedule(&adapter->rx.ob_q.napi);

	return 0;
}

int display_regs_write (struct file *file, const char __user *buffer,unsigned long count, void *data)
{
	return count;
}
int trigger_write (struct file *file, const char __user *buffer,unsigned long count, void *data)
{
	return count;
}

#endif

static int __devinit indus_probe(struct pci_dev *pdev,
				const struct pci_device_id *id)
{
	int status = 0;
	struct indus_adapter *adapter;
	struct net_device *netdev;
	u32 gmac_cfg;

	status = pci_enable_device(pdev);
	if (status)
		goto do_none;
	status = pci_request_regions(pdev, INDUS_DRV);
	if (status)
		goto disable_dev;
	pci_set_master(pdev);


	netdev = alloc_etherdev(sizeof(struct indus_adapter));
	if (netdev == NULL) {
		status = -ENOMEM;
		goto rel_reg;
	}
	adapter = netdev_priv(netdev);

	adapter->pdev = pdev;
	pci_set_drvdata(pdev, adapter);
	adapter->netdev = netdev;

	INIT_DELAYED_WORK(&adapter->work, indus_worker);

	indus_netdev_init(netdev);
	SET_NETDEV_DEV(netdev, &pdev->dev);

	status = pci_set_dma_mask(pdev, DMA_BIT_MASK(64));
	if (!status) {
		netdev->features |= NETIF_F_HIGHDMA;
	} else {
		status = pci_set_dma_mask(pdev, DMA_BIT_MASK(32));
		if (status) {
			dev_err(&pdev->dev, "Could not set PCI DMA Mask\n");
			goto free_netdev;
		}
	}

	if (status)
		goto free_netdev;
	{
		u32 regval;
		pci_read_config_dword(adapter->pdev, INDUS_PCICFG_MSI_ENABLE_OFFSET, &regval);
		if (regval & INDUS_PCICFG_MSI_ENABLE_MASK)
			printk("MSI BIT SET \n");
		else
			printk("MSI BIT NOT SET\n");
		pci_read_config_dword(adapter->pdev, INDUS_PCICFG_MSIX_ENABLE_OFFSET, &regval);
		if (regval & INDUS_PCICFG_MSIX_ENABLE_MASK)
			printk("MSIX BIT SET \n");
		else
			printk("MSIX BIT NOT SET\n");
	}
	//indus_dump_cb_q(&adapter->tx.cq);

	status = indus_ctrl_init(adapter);
	if(status)
		goto ctrl_clean;

	status = indus_setup(adapter);
	if(status)
		goto ctrl_clean;

	gmac_cfg = ioread32(adapter->devcfg + INDUS_GMAC_CTRL_OFFSET);
	//gmac cfg after setting dma circular buffers parameters
	gmac_cfg |= (INDUS_GMAC_CTRL_ETH_EN | INDUS_GMAC_CTRL_TXE | INDUS_GMAC_CTRL_RXE);
	iowrite32(gmac_cfg, adapter->devcfg + INDUS_GMAC_CTRL_OFFSET);

	gmac_cfg = ioread32(adapter->devcfg + INDUS_GMAC_CTRL_OFFSET);
	printk(KERN_INFO " GMAC CTRL reg info %x\n",gmac_cfg);


	/** receive control initi ************************/
	gmac_cfg = ioread32(adapter->devcfg + INDUS_RECV_CTRL_OFFSET);
	gmac_cfg |= (INDUS_RECV_CTRL_SBP | INDUS_RECV_CTRL_UPE |INDUS_RECV_CTRL_MPE | INDUS_RECV_CTRL_BCAST_EN);
	iowrite32(gmac_cfg,adapter->devcfg + INDUS_RECV_CTRL_OFFSET);

	msleep(5000);

	status = register_netdev(netdev);
	if (status != 0)
		goto unsetup;
	

//	gmac_cfg = mdio_read(adapter, MDIO_PHY_ID0_REG);
//	printk(" MDIO REG VALUE = %x \n", gmac_cfg);

	dev_info(&pdev->dev, "Driver prbing done");
#if GMAC_PROC_FS
	gmac = proc_mkdir("gmac", NULL);
	if(!gmac)
		printk(KERN_ERR "failed to create the gmac directory\n");

	display_reg = create_proc_entry("display_reg",0777,gmac);
	trigger_poll= create_proc_entry("trigger_poll",0777,gmac);
	display_reg->read_proc = display_regs_read;
	display_reg->write_proc = display_regs_write;
	display_reg->data = adapter;


	trigger_poll->read_proc = trigger_read;
	trigger_poll->write_proc = trigger_write;
	trigger_poll->data = adapter;

#endif

	return 0;

unsetup:
	indus_clear(adapter);
ctrl_clean:
	indus_ctrl_cleanup(adapter);
free_netdev:
	free_netdev(adapter->netdev);
	pci_set_drvdata(pdev, NULL);
rel_reg:
	pci_release_regions(pdev);
disable_dev:
	pci_disable_device(pdev);
do_none:
	dev_err(&pdev->dev, "%s initialization failed\n", INDUS_DRV);
	return status;
}


static void __devexit indus_remove(struct pci_dev *pdev)
{
	struct indus_adapter *adapter = pci_get_drvdata(pdev);

	if (!adapter)
		return;

	unregister_netdev(adapter->netdev);

	indus_clear(adapter);
	indus_ctrl_cleanup(adapter);
	pci_set_drvdata(pdev, NULL);
	pci_release_regions(pdev);
	pci_disable_device(pdev);
	free_netdev(adapter->netdev);
}

static struct pci_driver indus_driver = {
	.name		= INDUS_DRV,
	.id_table	= indus_dev_ids,
	.probe		= indus_probe,
	.remove		= indus_remove,
};

static int __init indus_module_init(void)
{
	int ret;
	ret = pci_register_driver(&indus_driver);
	if (ret)
		printk(KERN_ERR "failed register the indus_driver"
		" to pci subsystem");

	return ret;
}
static void __exit indus_module_exit(void)
{
	pci_unregister_driver(&indus_driver);
}

MODULE_AUTHOR("Ineda Systems Pvt Ltd");
MODULE_DESCRIPTION("Linux network driver for Indus GBE");
MODULE_LICENSE("GPL");

module_init(indus_module_init);
module_exit(indus_module_exit);
