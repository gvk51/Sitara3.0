#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/kdev_t.h>
#include <linux/interrupt.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <asm/signal.h>
#include <asm/siginfo.h>
#include <linux/kfifo.h>
#include <linux/slab.h>
#include <asm/atomic.h>

#include "ipc_hwinfo.h"
#include "ipc_packet.h"
#include "ipc_sts_header.h"

static struct class *ipc_mem_class;
static unsigned int ipc_mem_major;

/* Pointer to table of device ID's this driver supports. */
static DEFINE_PCI_DEVICE_TABLE(ipc_pci_ids) = {
        {PCI_DEVICE(INDUS_PCI_VENDOR_ID, INDUS_PCI_DEVICE1_ID),},
        { 0, }
};

/* Driver exports pci_device_id table using MODULE_DEVICE_TABLE */
MODULE_DEVICE_TABLE(pci, ipc_pci_ids);


static unsigned char ipc_pci_get_revision(struct pci_dev *dev)
{
	u8 revision;

	/* Read a byte from the PCI Configuration address space of the 
	device identified by dev. */
	pci_read_config_byte(dev, PCI_REVISION_ID, &revision);
	return revision;
}



static int __devinit ipc_map_device(struct pci_dev *pdev, struct ipc_hwinfo *hw)
{
        int error = -ENOMEM;
	unsigned int verify = 0;
	// int i = 0;

        /* Map the memory mapped base address to virtual address
	   A9 Buffers are mapped to 1st 4K of PCI BAR0 start and CFG/STATUS
	   registers to the subsequent 4K region */
        hw->mmio_a9_wr_buff_x1 = pci_iomap(pdev, INDUS_IPC_BAR0, 0);
        if (hw->mmio_a9_wr_buff_x1 == NULL) {
		dev_err(&pdev->dev, "Error mapping mmio for BAR0\n");
		return error;
        }

	/* BAR0 has mapping for all the three sections:
	   CONFIG/STATUS registers, Host Input Buffers and the Host
	   Read Circular Buffer (Input Buffer and Circular Buffers
	   have same address)
	*/

	hw->mmio_a9_wr_buff_pvsmc = hw->mmio_a9_wr_buff_x1 + IPC_PVSMC_BUFF_OFFSET;
	hw->mmio_a9_rd_buff = hw->mmio_a9_wr_buff_x1 + IPC_A9_BUFF_OFFSET;
	hw->mmio_a9_cfg_status = hw->mmio_a9_wr_buff_x1 + IPC_A9_CFG_STS_OFFSET;

#if 0
	printk("IPC Driver: BAR0 Addr CFG 0x%x \nIPC Driver: BAR0 Addr X1 Buf: 0x%x\n", hw->mmio_a9_cfg_status, hw->mmio_a9_wr_buff_x1);

	verify = ioread32(hw->mmio_a9_cfg_status);
	printk("IPC Driver: CFG/STS 1st Reg Addr: 0x%x\n", (unsigned int*)hw->mmio_a9_cfg_status);
	printk("IPC Driver: CFG/STS Contents: 0x%x\n", verify);

	verify = ioread32(hw->mmio_a9_cfg_status + 4);
	printk("IPC Driver: CFG/STS 2nd Reg Addr: 0x%x\n", (unsigned int*)(hw->mmio_a9_cfg_status + 4));
	printk("IPC Driver: CFG/STS Contents: 0x%x\n", verify);

	verify = ioread32(hw->mmio_a9_cfg_status + 8);
	printk("IPC Driver: CFG/STS 3rd Reg Addr: 0x%x\n", (unsigned int*)(hw->mmio_a9_cfg_status + 8));
	printk("IPC Driver: CFG/STS Contents: 0x%x\n", verify);

	verify = ioread32(hw->mmio_a9_cfg_status + 12);
	printk("IPC Driver: CFG/STS 4th Reg Addr: 0x%x\n", (unsigned int*)(hw->mmio_a9_cfg_status + 12));
	printk("IPC Driver: CFG/STS Contents: 0x%x\n", verify);


	verify = ioread32(hw->mmio_a9_cfg_status + 16);
	printk("IPC Driver: CFG/STS 5th Reg Addr: 0x%x\n", (unsigned int*)(hw->mmio_a9_cfg_status + 16));
	printk("IPC Driver: CFG/STS Contents: 0x%x\n", verify);

	verify = ioread32(hw->mmio_a9_cfg_status + 20);
	printk("IPC Driver: CFG/STS 6th Reg Addr: 0x%x\n", (unsigned int*)(hw->mmio_a9_cfg_status + 20));
	printk("IPC Driver: CFG/STS Contents: 0x%x\n", verify);
#endif



#if 0
	printk("IPC Driver: BAR0 CFG/STS Register Contents:\n");
	for(;i < 16; i++) {
		printk("0x%x\t", *(hw->mmio_a9_cfg_status + i*4));
	}
 
	printk("IPC Driver: BAR0 Buffer Contents:\n");
	for(;i < 16; i++) {
		printk("0x%x\t", *(hw->mmio_a9_wr_buff_x1 + i*4));
	}
#endif

	return 0;
}

static void ipc_unmap_device(struct pci_dev *pdev, struct ipc_hwinfo *hw)
{
	/* PCI BAR0 has a mapped address.
	   Unmap the config/status and circular buffer */
	pci_iounmap(pdev, hw->mmio_a9_wr_buff_x1);
}


/* IPC Buffer\Interrupt Configuration and Status Routines */

static inline void ipc_pci_is_buf_empty(struct ipc_hwinfo *hw, unsigned int *status)
{

	*status = ioread32(hw->mmio_a9_cfg_status + INDUS_BUF_STATUS_OFFSET); 
	*status = (*status & (INDUS_EMPTY_MASK << 1));

	return;

#if 0
	int ret;

	ret = pci_read_config_dword(hw->ipc_pci_dev, 
		INDUS_IPC_CONTROL_STATUS_BASE + INDUS_BUF_STATUS_OFFSET,
		status);


	if (ret) {
		printk(KERN_DEBUG IPC_DRIVER "Error read buf empty status: \
			%s:%i\n", __FILE__, __LINE__);
	}
	else {
		*status = (*status & (INDUS_EMPTY_MASK << 1));
	}

	return ret;
#endif
}

static inline int ipc_pci_is_buf_full(struct ipc_hwinfo *hw, unsigned int *status)
{
	int ret;

	ret = pci_read_config_dword(hw->ipc_pci_dev, 
		INDUS_IPC_CONTROL_STATUS_BASE + INDUS_BUF_STATUS_OFFSET,
		status);

	if (ret) {
		printk(KERN_DEBUG IPC_DRIVER "Error read buf full status: \
			%s:%i\n", __FILE__, __LINE__);
	}
	else {
		*status = (*status & INDUS_FULL_MASK);
	}

	return ret;
}

static inline void ipc_pci_get_interrupt_status(struct ipc_hwinfo *hw, unsigned int *pending)
{	
	*pending = ioread32(hw->mmio_a9_cfg_status + INDUS_INTR_RAW_STATUS_OFFSET); 
	*pending = (*pending & INDUS_RSTS_MASK);

	return;
}

static inline int ipc_pci_get_interrupt_mask_status(struct ipc_hwinfo *hw, unsigned int *status)
{
	int ret;

	ret = pci_read_config_dword(hw->ipc_pci_dev, 
		INDUS_IPC_CONTROL_STATUS_BASE + INDUS_INTR_MASK_STATUS_OFFSET,
		status);

	if (ret) {
		printk(KERN_DEBUG IPC_DRIVER "Error get intr mask status: \
			%s:%i\n", __FILE__, __LINE__);

	}
	else {
		*status = (*status & (INDUS_RSTS_MASK << 1));
	}

	return ret;
}

static inline void ipc_enable_interrupt(struct ipc_hwinfo *hw)
{
	int status, intr_status = 0;
	
	printk("IPC_Driver: ipc_enable_intr++\n");
	iowrite32(intr_status, hw->mmio_a9_cfg_status + INDUS_INTR_DISABLE_OFFSET); 
	printk("IPC_Driver: ipc_enable_intr--\n");
	return;

#if 0
	// Enable Interrupts
	status = pci_write_config_dword(hw->ipc_pci_dev, 
		INDUS_IPC_CONTROL_STATUS_BASE + INDUS_INTR_DISABLE_OFFSET,
		intr_status);

	if (status) {
		printk(KERN_DEBUG IPC_DRIVER "Err enabling interrupt: \
			%s:%i\n", __FILE__, __LINE__);
	}

	return status;
#endif
}

static inline void ipc_disable_interrupt(struct ipc_hwinfo *hw)
{
#if 1
	int status, intr_status = 1;

	printk("IPC_Driver: ipc_disable_intr++\n");
	iowrite32(intr_status, hw->mmio_a9_cfg_status + INDUS_INTR_DISABLE_OFFSET); 
	printk("IPC_Driver: ipc_disable_intr--\n");
#endif
	return;


#if 0
	// Disable Interrupts
	status = pci_write_config_dword(hw->ipc_pci_dev, 
		INDUS_IPC_CONTROL_STATUS_BASE + INDUS_INTR_DISABLE_OFFSET,
		intr_status);

	if (status) {
		printk(KERN_DEBUG IPC_DRIVER "Error disabling interrupt: \
			%s:%i\n", __FILE__, __LINE__);
	}

	return status;
#endif
}

static inline void ipc_clear_interrupt(struct ipc_hwinfo *hw)
{
	int clear = 1;
	printk("IPC Driver: In ISR, Clearing Intr\n");

	iowrite32(clear, hw->mmio_a9_cfg_status + INDUS_INTR_CLEAR_OFFSET); 

	return;


#if 0
	int status, clear = 1;

	// Clear Interrupt
	status = pci_write_config_dword(hw->ipc_pci_dev, 
		INDUS_IPC_CONTROL_STATUS_BASE + INDUS_INTR_CLEAR_OFFSET,
		clear);

	if (status) {
		printk(KERN_DEBUG IPC_DRIVER "Error clearing interrupt: \
			%s:%i\n", __FILE__, __LINE__);
	}

	return status;
#endif
}

static inline int ipc_flush_buffers(struct ipc_hwinfo *hw)
{
	int status, clear = 1;

	printk("IPC_Driver: flush buffers++\n");
	iowrite32(clear, hw->mmio_a9_cfg_status + INDUS_BUF_CONTROL); 
	printk("IPC_Driver: flush buffers--\n");
	return 0;

#if 0
	// Clear Interrupt
	status = pci_write_config_dword(hw->ipc_pci_dev, 
		INDUS_IPC_CONTROL_STATUS_BASE + 0 /* TBD Get from HW Team */,
		clear);

	if (status) {
		printk(KERN_DEBUG IPC_DRIVER "Err flushing buffers: \
			%s:%i\n", __FILE__, __LINE__);
	}

	return status;
#endif
}
/* End of IPC Buffer\Interrupt Configuration and Status Routines */


static irqreturn_t ipc_isr(int irq, void *data)
{
        struct ipc_hwinfo *hw = (struct ipc_hwinfo *)data;
	int pending = 0;
	int status = 0;
	unsigned int loop;
	unsigned int tmp[16] = {0};

#if 0	
        /* make sure it is our interrupt */
	if(ipc_pci_get_interrupt_status(hw, &pending)) {
		/* Error reading interrupt status */
		return IRQ_NONE;
	}
#endif
	ipc_pci_get_interrupt_status(hw, &pending);
	
	if (!pending) {
		// printk("IPC Driver: In ISR, Intr Status not Set, Not Handling\n");
		/* This is not our interrupt */
		return IRQ_NONE;
	}
	
#if 0
	if(ipc_pci_is_buf_empty(hw, &status)) {
		/* Error reading buffer empty status */
		return IRQ_NONE;
	}
#endif


	ipc_pci_is_buf_empty(hw, &status);

	if (status) {
		// printk("IPC Driver: In ISR, Buff Empty, Not Handling\n");
		/* Buffer is empty, nothing to process */
		printk("IPC_Driver: In ISR: Buffer is empty\n");
		return IRQ_HANDLED;
	}


	/* Read the IPC data from the IPC Circular Buffer */
	printk("IPC Driver: In ISR, reading Circ buffer contents\n");
	for (loop = 0; loop < 16; loop++) {
		// tmp[loop] = ioread32((unsigned int *)hw->mmio_a9_rd_buff + loop);
                   tmp[loop] = ioread32((unsigned int *)hw->mmio_a9_wr_buff_x1 + loop);
		printk("0x%x ", tmp[loop]);
	}

	memcpy((unsigned char*) hw->ipc_recv_msg, (unsigned char*) tmp, 64);
	
	/* If Buffer not empty, kick the tasklet for Rx */
	tasklet_schedule(&hw->rx_tasklet);

	/* Acknowledge Read Buffer Interrupt */
	ipc_clear_interrupt(hw);

        return IRQ_HANDLED;
}

static ssize_t ipc_read(struct file *fp, char __user *buf,
                        size_t len, loff_t *off)
{
        int ret, i;
        unsigned int copied;
	struct ipc_hwinfo *ipc_hw = fp->private_data;
	printk("ipc_read: +ipc_read\n");
	
	printk("ipc_read : len : %d\n", len);

	/* 
	   Note that with only one concurrent reader and one concurrent
	   writer, you don't need extra locking to use kfifo macro. 
	*/

        ret = kfifo_to_user(&ipc_hw->rd_fifo, buf, 64, &copied);
	if (unlikely(ret)) {
		printk(KERN_ERR IPC_DRIVER "Error reading from kifo: \
			%s:%i\n", __FILE__, __LINE__);
		return -EFAULT;
	}
	
	printk(KERN_INFO "ipc_read : received buffer : \n");
	for (i=0 ; i < 16 ; i++) {
		printk("0x%x ", *((unsigned int*)buf + i));
	}
	printk("\n");
	
	printk("IPC Driver: -ipc_read\n");
        return ret ? ret : copied;
}




static ssize_t ipc_write(struct file *fp, const char __user *buf,
                         size_t len, loff_t *off)
{
	// struct ipc_message ipc_packet;
	int i,j;
	char ipc_packet[68] = {};
	struct ipc_hwinfo *ipc_hw = fp->private_data;
	unsigned long flags;

	printk("IPC Driver: +ipc_write\n");

	spin_lock_irqsave(&ipc_hw->ipc_data_lock, flags);

	printk(KERN_INFO "buffer to send : \n");
	for (i=0 ; i<64 ; i++) {
		printk("%x ", *(buf+i));
	}

        if(copy_from_user(ipc_packet, buf, sizeof(ipc_packet)))
                return -EFAULT;

	if ( ( (*(int *)ipc_packet + 0) ) == IPC_HOST_PVSMC) {
		// TBD: Before Write, Check if the Host Buffer Status is OK
		// Get from HW Team
		/* Write to Input Host Buffer of PVSMC */
	       // memcpy_toio(&ipc_hw->mmio_a9_wr_buff_pvsmc[0 /* TBD Get from HW Team */], 
		//		((int *)ipc_packet + 1), 64); // Dont hard code it

		for(j=0;j<16;j++)
		{
			iowrite32(*((int *)ipc_packet +1+j),&ipc_hw->mmio_a9_wr_buff_pvsmc[j*4]);
		}
	}
	else if ( ( (*(int *)ipc_packet + 0) ) == IPC_HOST_DOMX) {

		// TBD: Before Write, Check if the Host Buffer Status is OK
		// Get from HW Team
		/* Write to Input Host Buffer of DOMX */
	        //memcpy_toio(&ipc_hw->mmio_a9_wr_buff_x1[0 /* TBD Get from HW Team */], 
		//		((int *)ipc_packet + 1), 64); // Dont hard code it

		for(j=0;j<16;j++)
		{
			iowrite32(*((int *)ipc_packet +1+j),&ipc_hw->mmio_a9_wr_buff_x1[j*4]);
		}

	}

	spin_unlock_irqrestore(&ipc_hw->ipc_data_lock, flags);

	printk("IPC Driver: -ipc_write\n");

	return len;
}

static int ipc_open(struct inode *ip, struct file *fp)
{
	struct ipc_hwinfo *ipc_hw = container_of(ip->i_cdev,
					struct ipc_hwinfo, cdev);

	printk("IPC Driver: +ipc_open\n");

	printk("ipc_cdev_count : %d\n", ipc_hw->ipc_cdev_count);

	/* Restrict access to single Client IPC Driver, device
	   file can't be opened more than one at a time */
	if (! atomic_dec_and_test (&ipc_hw->ipc_cdev_count)) {
		/* Device already open */
		printk("IPC Already Opened\n");
		// atomic_inc(&ipc_hw->ipc_cdev_count);
		return -EBUSY;
	}

	atomic_inc(&ipc_hw->ipc_cdev_count);

	/* mutex may not be needed, we allow only single IPC
	   Driver client to open the device file */
	mutex_lock(&ipc_hw->ipc_open_mutex);
        fp->private_data = ipc_hw;
	ipc_hw->ipc_async_queue = NULL;
	mutex_unlock(&ipc_hw->ipc_open_mutex);

	printk("IPC Driver: -ipc_open\n");
	return 0;
}

static int ipc_close(struct inode *ip, struct file *fp)
{
	int ret = 0;

	struct ipc_hwinfo *ipc_hw;
	printk("IPC Driver: +ipc_close\n");


	/* mutex may not be needed, we allow only single IPC
	   Driver client to open the device file */
	// mutex_lock(&ipc_hw->ipc_open_mutex);
	ipc_hw = fp->private_data;
	// mutex_unlock(&ipc_hw->ipc_open_mutex);
	printk("ipc_cdev_count : %d\n", ipc_hw->ipc_cdev_count);

	/* Release the device */
	atomic_dec(&ipc_hw->ipc_cdev_count); 

	mutex_lock(&ipc_hw->ipc_open_mutex);
	/* remove this fp from the asynchronously notified fp's */
        ret = fasync_helper(-1, fp, 0, &ipc_hw->ipc_async_queue);
	mutex_unlock(&ipc_hw->ipc_open_mutex);

	if (ret < 0) {
		printk(KERN_ERR IPC_DRIVER "Error from fasync_helper: \
			%s:%i\n", __FILE__, __LINE__);
	}

	printk("IPC Driver: -ipc_close\n");
	return ret;
}


static int ipc_fasync(int fd, struct file *filp, int on)
{
	int retval;
	struct ipc_hwinfo *ipc_hw = filp->private_data;

	/* This could race against open */
	mutex_lock(&ipc_hw->ipc_open_mutex);
	retval = fasync_helper(fd, filp, on, &ipc_hw->ipc_async_queue);
	mutex_unlock(&ipc_hw->ipc_open_mutex);

	if (retval < 0)
		return retval;

	return 0;
}

static long ipc_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	/* We may need to have few IOCTLs later in future */
        return 0;
}


static const struct file_operations ipc_fops = {
        .owner          = THIS_MODULE,
        .read           = ipc_read,
        .write          = ipc_write,
        .open           = ipc_open,
        .release        = ipc_close,
        .fasync		= ipc_fasync,
	.unlocked_ioctl	= ipc_ioctl,
};

static void ipc_rx_tasklet(unsigned long data)
{
	struct pci_dev *pdev = (struct pci_dev *)data;
	struct ipc_hwinfo *ipc_hw = pci_get_drvdata(pdev);
	unsigned long flags, loop;

#if 0
        unsigned int ipc_data[IPC_BUFFER_SIZE];
#endif

	/* Take the lock */
	spin_lock_irqsave(&ipc_hw->ipc_data_lock, flags);

#if 0
	/* Read the IPC data from the IPC Circular Buffer */
        memcpy_fromio((char *)ipc_data, (char *)&ipc_hw->mmio_a9_rd_buff[0/* TBD */], IPC_BUFFER_SIZE);

	printk("IPC Driver: Message Contents: after reading from Circular Buffer and data before adding to fifo\n");
	for (loop = 0; loop < 16; loop++) {
		printk("0x%x ", ipc_data[loop]);
	}
	printk("\n");
#endif

	/* Put the IPC Packet in the kfifo */
        if (!kfifo_in(&ipc_hw->rd_fifo, ipc_hw->ipc_recv_msg, 64 /* TBD Is this the size of each element?*/)) {
		printk(KERN_ERR IPC_DRIVER "Err putting to kfifo: \
			%s:%i\n", __FILE__, __LINE__);
		return;
	}

	// Inform the user to perform read
	if (ipc_hw->ipc_async_queue) {
		kill_fasync(&ipc_hw->ipc_async_queue, SIGIO, POLL_IN);
	}

	/* Release the lock */
	spin_unlock_irqrestore(&ipc_hw->ipc_data_lock, flags);
}


/*
This function gets called by the PCI core (during execution of 
pci_register_driver() for already existing devices or later if a new device 
gets inserted) for all PCI devices which match the ID table and are not 
"owned" by the other drivers yet. 

This function gets passed a "struct pci_dev *" for each device whose entry
in the ID table matches the device. 

The function returns zero when the driver take "ownership" of the device or
else an error code (negative number).

The function is called from process context, so it can sleep.
*/
static int __devinit ipc_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct device *device;
	int minor, error;
	unsigned int verify = 0;
	struct ipc_hwinfo *ipc_hw;

	printk("IPC_Driver Probe++\n");	

	error = -ENOMEM;

	// Check the PCI config space Revision ID 
	/*
	if (ipc_pci_get_revision(pdev) == IPC_REV_ID)
		return -ENODEV;
	*/

	/* track global allocations for this device */
	ipc_hw = kzalloc(sizeof(*ipc_hw), GFP_KERNEL);
	if (!ipc_hw)
		return error;

	/* Initialize the Queue to store the circular buffer contents */
	error = kfifo_alloc(&ipc_hw->rd_fifo, INDUS_IPC_FIFO_SIZE, GFP_KERNEL);
	if (error) {
		printk(KERN_ERR IPC_DRIVER "Error initializing kfifo: \
			%s:%i\n", __FILE__, __LINE__);

		return -ENOMEM;
	}

	/* Store the pci_dev structure in the hw structure */
	ipc_hw->ipc_pci_dev = pdev;

	/* 
	   Before the driver can access any device resource 
	   (I/O region or interrupt) of the PCI device, the 
	   driver must call the pci_enable_device function
	*/
	error = pci_enable_device(pdev);
	if (error)
		goto free;

	error = pci_request_regions(pdev, IPC_DEVICE_CLASS);
	if (error)
		goto disable;

	error = ipc_map_device(pdev, ipc_hw);
	if (error)
		goto free_regions;

	/* Associates the given data with the given pci_driver 
	structure which you can retrieve later (remove) */
	pci_set_drvdata(pdev, ipc_hw);

	error = request_irq(pdev->irq, ipc_isr, IRQF_SHARED, IPC_DEVICE_CLASS, ipc_hw);
	if (error) {
		printk(KERN_ERR IPC_DRIVER "Err registring interrupt %d\n: \
			%s:%i\n", pdev->irq, __FILE__, __LINE__);
		goto unmap;
	}

        /* Init tasklet for bottom half processing */
        tasklet_init(&ipc_hw->rx_tasklet, ipc_rx_tasklet,
		(unsigned long)pdev);

	spin_lock_init(&ipc_hw->ipc_data_lock);
        mutex_init(&ipc_hw->ipc_open_mutex);


	/* initialize a cdev structure */
	cdev_init(&ipc_hw->cdev, &ipc_fops);
	ipc_hw->cdev.owner = THIS_MODULE;

	minor = 1;

	/* Add a character device to the system */
	error = cdev_add(&ipc_hw->cdev, MKDEV(ipc_mem_major, minor), MAX_MINOR_COUNT);
	if (error) {
		dev_err(&pdev->dev, "Could not add cdev\n");
		goto remove_isr;
	}

	device = device_create(ipc_mem_class, &pdev->dev,
		    MKDEV(ipc_mem_major, minor), NULL,
				    IPC_DEVICE_NAME);
	if (IS_ERR(device))
		dev_err(&pdev->dev, "Could not create files\n");


        atomic_set(&ipc_hw->ipc_cdev_count, 1);

#if 0
        /* Reset and initialize the IPC H/W */
	/* Flush the host read buffers */
	error = ipc_flush_buffers(ipc_hw);
	if (error) {
		dev_err(&pdev->dev, "Could not flush circular buffers\n");
		goto remove_isr;
	}
	printk("IPC_Driver Probe: Flushed CBuffers\n");	
#endif
	/* Device should be ready to accept interrupts now */
	ipc_enable_interrupt(ipc_hw);

	printk(KERN_INFO IPC_DRIVER "IPC driver initialized: \
		%s:%i\n", __FILE__, __LINE__);


#if 0
	// Printing the contents just before probe
	printk("IPC Driver: BAR0 Addr CFG 0x%x \nIPC Driver: BAR0 Addr X1 Buf: 0x%x\n", ipc_hw->mmio_a9_cfg_status, ipc_hw->mmio_a9_wr_buff_x1);

	verify = ioread32(ipc_hw->mmio_a9_cfg_status);
	printk("IPC Driver: CFG/STS 1st Reg Addr: 0x%x\n", (unsigned int*)ipc_hw->mmio_a9_cfg_status);
	printk("IPC Driver: CFG/STS Contents: 0x%x\n", verify);

	verify = ioread32(ipc_hw->mmio_a9_cfg_status + 4);
	printk("IPC Driver: CFG/STS 2nd Reg Addr: 0x%x\n", (unsigned int*)(ipc_hw->mmio_a9_cfg_status + 4));
	printk("IPC Driver: CFG/STS Contents: 0x%x\n", verify);

	verify = ioread32(ipc_hw->mmio_a9_cfg_status + 8);
	printk("IPC Driver: CFG/STS 3rd Reg Addr: 0x%x\n", (unsigned int*)(ipc_hw->mmio_a9_cfg_status + 8));
	printk("IPC Driver: CFG/STS Contents: 0x%x\n", verify);

	verify = ioread32(ipc_hw->mmio_a9_cfg_status + 12);
	printk("IPC Driver: CFG/STS 4th Reg Addr: 0x%x\n", (unsigned int*)(ipc_hw->mmio_a9_cfg_status + 12));
	printk("IPC Driver: CFG/STS Contents: 0x%x\n", verify);


	verify = ioread32(ipc_hw->mmio_a9_cfg_status + 16);
	printk("IPC Driver: CFG/STS 5th Reg Addr: 0x%x\n", (unsigned int*)(ipc_hw->mmio_a9_cfg_status + 16));
	printk("IPC Driver: CFG/STS Contents: 0x%x\n", verify);

	verify = ioread32(ipc_hw->mmio_a9_cfg_status + 20);
	printk("IPC Driver: CFG/STS 6th Reg Addr: 0x%x\n", (unsigned int*)(ipc_hw->mmio_a9_cfg_status + 20));
	printk("IPC Driver: CFG/STS Contents: 0x%x\n", verify);
 #endif 


	printk("IPC_Driver Probe--\n");	

	return 0;

remove_isr:
	ipc_disable_interrupt(ipc_hw);
	free_irq(pdev->irq, ipc_hw);
	tasklet_kill(&ipc_hw->rx_tasklet);
	printk("IPC_Driver Probe: Remove ISR goto\n");	
unmap:
	pci_set_drvdata(pdev, NULL);
	ipc_unmap_device(pdev, ipc_hw);
	printk("IPC_Driver Probe: Unmap goto\n");	

free_regions:
	pci_release_regions(pdev);
	printk("IPC_Driver Probe: free regions goto\n");	

disable:
	/* Do not access device regs after calling pci_disable_device() */
	pci_disable_device(pdev);
	printk("IPC_Driver Probe: disable dev goto\n");	

free:
	kfifo_free(&ipc_hw->rd_fifo);
	kfree(ipc_hw);
	printk("IPC_Driver Probe: kfifo free goto\n");	

	return error;
}

/* 
Pointer to the function that the PCI core calls when the struct pci_dev is
removed from the system, or when the PCI driver is unloaded from the kernel
*/
static void __devexit ipc_remove(struct pci_dev *pdev)
{
	/* 
	   Clean up any allocated resources and stuff here.
	   Like call release_region();
	*/
        int minor;
        struct ipc_hwinfo *ipc_hw = pci_get_drvdata(pdev);

	printk("IPC_Driver Remove++\n");	

        minor = MINOR(ipc_hw->cdev.dev);
        device_destroy(ipc_mem_class, MKDEV(ipc_mem_major, minor));

        cdev_del(&ipc_hw->cdev);
	/* Disable Interrupts */
        ipc_disable_interrupt(ipc_hw);
        free_irq(pdev->irq, ipc_hw);
        tasklet_kill(&ipc_hw->rx_tasklet);

        ipc_unmap_device(pdev, ipc_hw);
        pci_release_regions(pdev);

	/* Do not access device regs after calling pci_disable_device() */
        pci_disable_device(pdev);

	kfifo_free(&ipc_hw->rd_fifo);
        kfree(ipc_hw);
	printk("IPC_Driver Remove--\n");	
}

#ifdef CONFIG_PM
/* 
Pointer to the function that the PCI core calls when the struct
pci_dev is being suspended. This function is optional
*/
static int ipc_suspend(struct pci_dev *dev, pm_message_t state)
{
	return 0;
}

/*
Pointer to the function that the PCI core calls when the struct
pci_dev is being resumed. It is always called after suspend has
been called. This function is optional.
*/
static int ipc_resume(struct pci_dev *dev)
{
	return 0;
}
#endif /* CONFIG_PM */



static struct pci_driver ipc_pci_driver = {
	/*
	The name of the driver must be unique among all 
	PCI drivers in the kernel and is normally set to
	the same name as the module name of the driver. 
	It shows up in sysfs under /sys/bus/pci/drivers/
	*/
	.name = 	IPC_DEVICE_CLASS,
	.id_table = 	ipc_pci_ids,
	.probe = 	ipc_probe,
#ifdef CONFIG_PM
	.suspend = 	ipc_suspend,
	.resume = 	ipc_resume,
#endif /* CONFIG_PM */
	.remove = 	__devexit_p(ipc_remove),
};


static int __init ipc_pci_init(void)
{
        int error;
        dev_t dev;

	printk("IPC_Driver Init++\n");	
	/* 
	Populate sysfs entries
	This is used to create a struct class pointer that can then be used
	in calls to device_create() (in probe function)
	*/
        ipc_mem_class = class_create(THIS_MODULE, IPC_DEVICE_CLASS);
        if (IS_ERR(ipc_mem_class)) {
                error = PTR_ERR(ipc_mem_class);
		printk(KERN_ERR IPC_DRIVER "Can't populate sysfs entry: \
			%s:%i\n", __FILE__, __LINE__);

                return error;
        }

	/* 
	Request allocation of a char device number(s). The major number
	is allocated dynamically and returned in dev (dev_t *dev) (with
	the first minor number 
	*/
        error = alloc_chrdev_region(&dev, BASE_MINOR_NUM, MAX_MINOR_COUNT, IPC_DEVICE_NAME);
        if (error) {
		printk(KERN_ERR IPC_DRIVER "Can't register char device: \
			%s:%i\n", __FILE__, __LINE__);

	        class_destroy(ipc_mem_class);
		return error;
	}

	/* Save the major number of the device */
        ipc_mem_major = MAJOR(dev);

	/*
	Register new driver struct pci_driver with the PCI core.
	This is traditionally done in the module initialization
	for the PCI drivers
	*/
        error = pci_register_driver(&ipc_pci_driver);
        if (error) {
	        unregister_chrdev_region(dev, MAX_MINOR_COUNT);
	        class_destroy(ipc_mem_class);
		printk(KERN_ERR IPC_DRIVER "Can't Can't register pci driver: \
			%s:%i\n", __FILE__, __LINE__);

	        return error;
	}

	printk("IPC_Driver Init--\n");	

//        atomic_set(&ipc_hw->ipc_cdev_count, 1);
        return 0;
}

static void __exit ipc_pci_exit(void)
{
		
	printk("IPC_Driver Exit++\n");	

	/* PCI calls the remove hook for device(s) handled by this driver */
	pci_unregister_driver(&ipc_pci_driver);

	/* Unregisters a range of MAX_MINOR_COUNT device numbers,
	   starting with MKDEV(ipc_mem_major, 0) */
        unregister_chrdev_region(MKDEV(ipc_mem_major, 0), MAX_MINOR_COUNT);

	/* Destroy a struct class structure - remove class attributes and
	decrement/remove refcount for kobject */
        class_destroy(ipc_mem_class);
	printk("IPC_Driver Exit--\n");	
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Kapil Hali <kapil.hali@inedasystems.com>");

module_init(ipc_pci_init);
module_exit(ipc_pci_exit);
