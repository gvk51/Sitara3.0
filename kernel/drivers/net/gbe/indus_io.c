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
/**
 * This function is used to configure Descriptor circular buffer registers
 */
int indus_set_dcb_ctrl_reg(struct indus_adapter *adapter,int offset,
					struct indus_queue_info *q)
{
	int status = 0;
	if(!q->created){
		status = -EINVAL;
		goto out;
	}
	iowrite32(indus_dma_low(q->dma_mem.dma),
			adapter->devcfg+offset+DCB_CONTROL_ADDR_LOW_REG);
	iowrite32(indus_dma_high(q->dma_mem.dma),
			adapter->devcfg+(offset+DCB_CONTROL_ADDR_HIGH_REG));
	iowrite32((q->len),
			adapter->devcfg+ (offset+DCB_CONTROL_LEGTH));

out:
	return status;

}

void indus_queue_head_update(struct indus_adapter *adapter,
					struct indus_queue_info *q)
{
	u32 regval;
	if(!q->created){
		return;
	}
	if(q->head == 0){
		printk("Updating Head %d Offset: %x (High)\n",
			INDUS_QUEUE_INDEX_HIGH,q->reg_offset);
		iowrite32(INDUS_QUEUE_INDEX_HIGH,
			(adapter->devcfg+q->reg_offset));
		regval = ioread32(adapter->devcfg + INDUS_RX_EB_HEAD_OFFSET);
		printk("RX EB HEAD = %x \n",regval);
		regval = ioread32(adapter->devcfg + INDUS_RX_EB_TAIL_OFFSET);
		printk("RX EB TAIL = %x \n",regval);
		regval = ioread32(adapter->devcfg + INDUS_RX_OB0_HEAD_OFFSET);
		printk("RX OB HEAD = %x \n", regval);
		regval = ioread32(adapter->devcfg + INDUS_RX_OB0_TAIL_OFFSET);
		printk("RX OB TAIL = %x \n", regval);
		regval = ioread32(adapter->devcfg + INDUS_RIS1_OFFSET);
		printk("After RIS = %x \n", regval);

                printk("TX EB HEAD = %x \n",ioread32(adapter->devcfg + INDUS_TX_EB_HEAD_OFFSET));
                printk("TX EB TAIL = %x \n",ioread32(adapter->devcfg + INDUS_TX_EB_TAIL_OFFSET));
                printk("TX OB HEAD = %x \n",ioread32(adapter->devcfg + INDUS_TX_OB_HEAD_OFFSET));
                printk("TX OB TAIL = %x \n",ioread32(adapter->devcfg + INDUS_TX_OB_TAIL_OFFSET));

	}else{
		printk("Updating Head %d Offset: %x \n",
			q->head,q->reg_offset);
		iowrite32(q->head,(adapter->devcfg+q->reg_offset));
		regval = ioread32(adapter->devcfg + INDUS_RX_EB_HEAD_OFFSET);
		printk("RX EB HEAD = %x \n",regval);
		regval = ioread32(adapter->devcfg + INDUS_RX_EB_TAIL_OFFSET);
		printk("RX EB TAIL = %x \n",regval);
		regval = ioread32(adapter->devcfg + INDUS_RX_OB0_HEAD_OFFSET);
		printk("RX OB HEAD = %x \n", regval);
		regval = ioread32(adapter->devcfg + INDUS_RX_OB0_TAIL_OFFSET);
		printk("RX OB TAIL = %x \n", regval);
		regval = ioread32(adapter->devcfg + INDUS_RIS1_OFFSET);
		printk("After RIS = %x \n", regval);

		printk("TX EB HEAD = %x \n",ioread32(adapter->devcfg + INDUS_TX_EB_HEAD_OFFSET));
		printk("TX EB TAIL = %x \n",ioread32(adapter->devcfg + INDUS_TX_EB_TAIL_OFFSET));
		printk("TX OB HEAD = %x \n",ioread32(adapter->devcfg + INDUS_TX_OB_HEAD_OFFSET));
		printk("TX OB TAIL = %x \n",ioread32(adapter->devcfg + INDUS_TX_OB_TAIL_OFFSET));


	}

}
void indus_queue_tail_update(struct indus_adapter *adapter,struct indus_queue_info *q)
{
	u32 regval;
	if(!q->created){
		return;
	}
	if(q->tail == 0){
		iowrite32(INDUS_QUEUE_INDEX_HIGH,
				(adapter->devcfg+q->reg_offset));
/*		printk("RX OB HEAD = %x \n", regval);
		regval = ioread32(adapter->devcfg + INDUS_RX_OB0_TAIL_OFFSET);
		printk("RX OB TAIL = %x \n", regval);
		regval = ioread32(adapter->devcfg + INDUS_RIS1_OFFSET);
*/

	}else{
		iowrite32(q->tail,(adapter->devcfg+q->reg_offset));
/*		regval = ioread32(adapter->devcfg + INDUS_TX_OB_TAIL_OFFSET);
		printk("RX OB HEAD = %x \n", regval);
		regval = ioread32(adapter->devcfg + INDUS_RX_OB0_TAIL_OFFSET);
		printk("RX OB TAIL = %x \n", regval);
		regval = ioread32(adapter->devcfg + INDUS_RIS1_OFFSET);
*/

	}
}

void indus_set_queue_thlds(struct indus_adapter *adapter)
{
	iowrite32(adapter->tx_thld,adapter->devcfg+INDUS_TX_BUFF_THLD_OFFSET);
	iowrite32(adapter->rx_thld,adapter->devcfg+INDUS_RX_Q0_INTR_CNT_OFFSET);
}
void indus_set_mac_addr(struct indus_adapter *adapter)
{
	/**
	 * TODO cross check the mac address with hw team
	 */
	char mac[6];
	int reg_val = 0;

	if(host==0){
		reg_val = 0xaabb;
	}else{
		reg_val = 0x0011;
	}

	printk("Writing Mac addr high %x\n", reg_val);
	iowrite32(reg_val, adapter->devcfg + INDUS_UNI_HIGH0_OFFSET);
	reg_val = ioread32(adapter->devcfg + INDUS_UNI_HIGH0_OFFSET);
	printk("Read mac addr high %x\n", reg_val);
	mac[0] = (reg_val & 0xFF00) >> 8;
	mac[1] = reg_val & 0x00FF;

	if(host==0){
		reg_val = 0xccddeeff;
	}else{
		reg_val = 0x22334455;
	}
	iowrite32(reg_val, adapter->devcfg + INDUS_UNI_LOW0_OFFSET);
	reg_val = ioread32(adapter->devcfg + INDUS_UNI_LOW0_OFFSET);
	printk("Read mac addr low %x\n", reg_val);
	mac[2] = (reg_val & 0xFF000000) >> 24;
	mac[3] = (reg_val & 0x00FF0000) >> 16;
	mac[4] = (reg_val & 0x0000FF00) >> 8;
	mac[5] = (reg_val & 0x000000FF);
	memcpy(adapter->netdev->dev_addr, mac, adapter->netdev->addr_len);
	printk(" Mac address %x %x %x %x %x %x \n",
			adapter->netdev->dev_addr[0],
			adapter->netdev->dev_addr[1],
			adapter->netdev->dev_addr[2],
			adapter->netdev->dev_addr[3],
			adapter->netdev->dev_addr[4],
			adapter->netdev->dev_addr[5]);
	memcpy(adapter->netdev->perm_addr, mac, adapter->netdev->addr_len);

}
