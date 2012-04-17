#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/stat.h>
#include <linux/moduleparam.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/time.h>
#include <linux/sysfs.h>
#include <linux/kdev_t.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <linux/sched.h>
#include <linux/threads.h>
#include <linux/input.h>

struct input_dev* vkpd_dev;

static const unsigned char virtual_kbd_keycode[256] = {
	  0,  0,  0,  0, 30, 48, 46, 32, 18, 33, 34, 35, 23, 36, 37, 38,
	 50, 49, 24, 25, 16, 19, 31, 20, 22, 47, 17, 45, 21, 44,  2,  3,
	  4,  5,  6,  7,  8,  9, 10, 11, 28,  1, 14, 15, 57, 12, 13, 26,
	 27, 43, 43, 39, 40, 41, 51, 52, 53, 58, 59, 60, 61, 62, 63, 64,
	 65, 66, 67, 68, 87, 88, 99, 70,119,110,102,104,111,107,109,106,
	105,108,103, 69, 98, 55, 74, 78, 96, 79, 80, 81, 75, 76, 77, 71,
	 72, 73, 82, 83, 86,127,116,117,183,184,185,186,187,188,189,190,
	191,192,193,194,134,138,130,132,128,129,131,137,133,135,136,113,
	115,114,  0,  0,  0,121,  0, 89, 93,124, 92, 94, 95,  0,  0,  0,
	122,123, 90, 91, 85,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	 29, 42, 56,125, 97, 54,100,126,164,166,165,163,161,115,114,113,
	150,158,159,128,136,177,178,176,142,152,173,140
};

int __init vmodule_init(void)
{
	int i = 0;
	printk("vmodule_init called entered");
	/* extra safe initialization */
	vkpd_dev = input_allocate_device();

	/* set up descriptive labels */
	vkpd_dev->name = "Ineda Virtual Keyboard";
	/* phys is unique on a running system */
	vkpd_dev->phys = "No Phical Path";
	vkpd_dev->id.bustype = BUS_HOST;
	vkpd_dev->id.vendor = 0x0001;

	vkpd_dev->id.product = 0x0001;
	vkpd_dev->id.version = 0x0100;

	/* this device has two keys (A and B) */
	set_bit(EV_KEY, vkpd_dev->evbit);
	for (i = 0; i < 255; i++)
		set_bit(virtual_kbd_keycode[i], vkpd_dev->keybit);
	clear_bit(0, vkpd_dev->keybit);


	/* and finally register with the input core */
	i = input_register_device(vkpd_dev);
	printk("vmodule_init called exited %d",i);
	return 0;
}

void __exit vmodule_exit(void)
{
	printk("vmodule_exit called entered");
	input_unregister_device(vkpd_dev);
	printk("vmodule_exit called exited");
}
module_init(vmodule_init);
module_exit(vmodule_exit);
MODULE_AUTHOR("Prakash V");
MODULE_DESCRIPTION("Virtual keyboard Driver");
MODULE_SUPPORTED_DEVICE("Keyboard");
MODULE_LICENSE("GPL");

