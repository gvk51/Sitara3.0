#include "debug.h"
#include <linux/kernel.h>
#if DEBUG
void debug_i(char *fmt,...)
{
	va_list args;
	char buf[500];
	va_start(args, fmt);
	vsnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);
	printk(KERN_INFO "INFO   :   %s",buf);
}
void debug_w(char *fmt,...)
{
	va_list args;
	char buf[500];
	va_start(args, fmt);
	vsnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);
	printk(KERN_INFO "WARN   :   %s",buf);
}
void debug_e(char *fmt,...)
{
	va_list args;
	char buf[500];
	va_start(args, fmt);
	vsnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);
	printk(KERN_INFO "ERROR  :   %s",buf);
}
void debug_d(char *fmt,...)
{
	va_list args;
	char buf[500];
	va_start(args, fmt);
	vsnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);
	printk(KERN_INFO "DEBUG  :   %s",buf);
}
#endif
