#ifndef __IPC_HWINFO_H
#define __IPC_HWINFO_H


#define IPC_DEVICE_CLASS	"ineda_ipc_mem_dev"
#define IPC_DRIVER		"IPC_ANDROID_DRIVER"
#define IPC_DEVICE_NAME 	"a9indusipc"

#define MAX_MINOR_COUNT	1
#define BASE_MINOR_NUM	0

#define INDUS_IPC_CONTROL_STATUS_BASE	0x90


#define IPC_REV_ID	0x0001

enum {
	/* */
	INDUS_IPC_BAR0 = 0,
};

enum {
	/* TBD Hw Team has to provide the values */
	INDUS_PCI_VENDOR_ID = 	0x1C31,
	INDUS_PCI_DEVICE1_ID =	0x0100
};

enum {
	/* PAGE_SIZE is atleast 4K (64 messages from the
	   circular read IPC Buffer will be queued) */

	/* fifo size in elements (bytes) */
	INDUS_IPC_FIFO_SIZE = PAGE_SIZE
};

struct ipc_hwinfo {

#define IPC_BUFFER_SIZE	64
        /* mmio registers and buffers on device */
	/* A9 Host Config/Status Register */
        char __iomem 		*mmio_a9_cfg_status;

	/* This is a hardware bug and will have to 
	be resolved in the next release. Currently
	we have a common host input buffer, we need
	input buffer specific to each host
	*/

#define IPC_X1_BUFF_OFFSET	(1024 * 0)
#define IPC_PVSMC_BUFF_OFFSET	(1024 * 1)
#define IPC_A9_BUFF_OFFSET	(1024 * 2)
#define IPC_A9_CFG_STS_OFFSET	(1024 * 4)

	/* A9 Host Input Buffer */
        char __iomem 		*mmio_a9_wr_buff_x1;
        char __iomem 		*mmio_a9_wr_buff_pvsmc;
	/* x2 is not needed yet */
        // char __iomem 	*mmio_a9_wr_buff_x2;

	/* A9 Host Circular Buffer */
        char __iomem 		*mmio_a9_rd_buff;

	/* Hold the reference of the PCI Device */
        struct pci_dev 		*ipc_pci_dev;

	/* Queue the pids of the user processes */
        struct fasync_struct 	*ipc_async_queue;

	/* Abstraction representing char device */
        struct cdev 		cdev;

	/* Restrict device open one at a time */
	atomic_t		ipc_cdev_count;
	
	unsigned char ipc_recv_msg[64];		

	/* Queu holding the read buffers */
	struct kfifo		rd_fifo;

	struct tasklet_struct	rx_tasklet;

	// How many do we need, to be determined?
        spinlock_t 		ipc_data_lock;
	struct mutex		ipc_open_mutex;

};

#endif /* __IPC_HWINFO_H */
