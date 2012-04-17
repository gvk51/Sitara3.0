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
#include "debug.h"

struct input_dev* vtouch_dev;

int __init vtmodule_init(void)
{
	int i = 0;
	debug_i("vmodule_init called entered");
	/* extra safe initialization */
	vtouch_dev = input_allocate_device();

	/* set up descriptive labels */
	vtouch_dev->name = "Ineda Virtual Touch";
	/* phys is unique on a running system */
	vtouch_dev->phys = "No Physical Path";
	vtouch_dev->id.bustype = BUS_HOST;
	vtouch_dev->id.vendor = 0x0001;

	vtouch_dev->id.product = 0x0001;
	vtouch_dev->id.version = 0x0100;


	vtouch_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	vtouch_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	input_set_abs_params(vtouch_dev, ABS_X, 0, 1280, 0, 0);
	input_set_abs_params(vtouch_dev, ABS_Y, 0, 1024, 0, 0);

//	input_set_abs_params(vtouch_dev, ABS_PRESSURE, 1,
//	                     0, 0, 0);

	/* and finally register with the input core */

	i = input_register_device(vtouch_dev);
	debug_i("vmodule_init called exited %d",i);
	return 0;
}

void __exit vtmodule_exit(void)
{
	debug_i("vmodule_exit called entered");
	input_unregister_device(vtouch_dev);
	debug_i("vmodule_exit called exited");
}
module_init(vtmodule_init);
module_exit(vtmodule_exit);
MODULE_AUTHOR("Valeswara Rao D");
MODULE_DESCRIPTION("Virtual Touch Driver");
MODULE_SUPPORTED_DEVICE("Touch");
MODULE_LICENSE("GPL");

