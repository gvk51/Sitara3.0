/*
 * Hardware specific definitions for Sharp SL-C6000x series of PDAs
 *
 * Copyright (c) 2005 Dirk Opfer
 *
 * Based on Sharp's 2.4 kernel patches
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#ifndef _ASM_ARCH_TOSA_H_
#define _ASM_ARCH_TOSA_H_ 1

/*  TOSA Chip selects  */
#define TOSA_LCDC_PHYS		PXA_CS4_PHYS
/* Internel Scoop */
#define TOSA_CF_PHYS		(PXA_CS2_PHYS + 0x00800000)
/* Jacket Scoop */
#define TOSA_SCOOP_PHYS  	(PXA_CS5_PHYS + 0x00800000)

#define TOSA_NR_IRQS		(IRQ_BOARD_START + TC6393XB_NR_IRQS)
/*
 * SCOOP2 internal GPIOs
 */
#define TOSA_SCOOP_GPIO_BASE		NR_BUILTIN_GPIO
#define TOSA_SCOOP_PXA_VCORE1		SCOOP_GPCR_PA11
#define TOSA_GPIO_TC6393XB_REST_IN	(TOSA_SCOOP_GPIO_BASE + 1)
#define TOSA_GPIO_IR_POWERDWN		(TOSA_SCOOP_GPIO_BASE + 2)
#define TOSA_GPIO_SD_WP			(TOSA_SCOOP_GPIO_BASE + 3)
#define TOSA_GPIO_PWR_ON		(TOSA_SCOOP_GPIO_BASE + 4)
#define TOSA_SCOOP_AUD_PWR_ON		SCOOP_GPCR_PA16
#define TOSA_GPIO_BT_RESET		(TOSA_SCOOP_GPIO_BASE + 6)
#define TOSA_GPIO_BT_PWR_EN		(TOSA_SCOOP_GPIO_BASE + 7)
#define TOSA_SCOOP_AC_IN_OL		SCOOP_GPCR_PA19

/* GPIO Direction   1 : output mode / 0:input mode */
#define TOSA_SCOOP_IO_DIR     (TOSA_SCOOP_PXA_VCORE1 | \
		TOSA_SCOOP_AUD_PWR_ON)

/*
 * SCOOP2 jacket GPIOs
 */
#define TOSA_SCOOP_JC_GPIO_BASE		(NR_BUILTIN_GPIO + 12)
#define TOSA_GPIO_BT_LED		(TOSA_SCOOP_JC_GPIO_BASE + 0)
#define TOSA_GPIO_NOTE_LED		(TOSA_SCOOP_JC_GPIO_BASE + 1)
#define TOSA_GPIO_CHRG_ERR_LED		(TOSA_SCOOP_JC_GPIO_BASE + 2)
#define TOSA_GPIO_USB_PULLUP		(TOSA_SCOOP_JC_GPIO_BASE + 3)
#define TOSA_GPIO_TC6393XB_SUSPEND	(TOSA_SCOOP_JC_GPIO_BASE + 4)
#define TOSA_GPIO_TC6393XB_L3V_ON	(TOSA_SCOOP_JC_GPIO_BASE + 5)
#define TOSA_SCOOP_JC_WLAN_DETECT	SCOOP_GPCR_PA17
#define TOSA_GPIO_WLAN_LED		(TOSA_SCOOP_JC_GPIO_BASE + 7)
#define TOSA_SCOOP_JC_CARD_LIMIT_SEL	SCOOP_GPCR_PA19

/* GPIO Direction   1 : output mode / 0:input mode */
#define TOSA_SCOOP_JC_IO_DIR (TOSA_SCOOP_JC_CARD_LIMIT_SEL)

/*
 * TC6393XB GPIOs
 */
#define TOSA_TC6393XB_GPIO_BASE		(NR_BUILTIN_GPIO + 2 * 12)

#define TOSA_GPIO_TG_ON			(TOSA_TC6393XB_GPIO_BASE + 0)
#define TOSA_GPIO_L_MUTE		(TOSA_TC6393XB_GPIO_BASE + 1)
#define TOSA_GPIO_BL_C20MA		(TOSA_TC6393XB_GPIO_BASE + 3)
#define TOSA_GPIO_CARD_VCC_ON		(TOSA_TC6393XB_GPIO_BASE + 4)
#define TOSA_GPIO_CHARGE_OFF		(TOSA_TC6393XB_GPIO_BASE + 6)
#define TOSA_GPIO_CHARGE_OFF_JC		(TOSA_TC6393XB_GPIO_BASE + 7)
#define TOSA_GPIO_BAT0_V_ON		(TOSA_TC6393XB_GPIO_BASE + 9)
#define TOSA_GPIO_BAT1_V_ON		(TOSA_TC6393XB_GPIO_BASE + 10)
#define TOSA_GPIO_BU_CHRG_ON		(TOSA_TC6393XB_GPIO_BASE + 11)
#define TOSA_GPIO_BAT_SW_ON		(TOSA_TC6393XB_GPIO_BASE + 12)
#define TOSA_GPIO_BAT0_TH_ON		(TOSA_TC6393XB_GPIO_BASE + 14)
#define TOSA_GPIO_BAT1_TH_ON		(TOSA_TC6393XB_GPIO_BASE + 15)

/*
 * Timing Generator
 */
#define TG_PNLCTL 			0x00
#define TG_TPOSCTL 			0x01
#define TG_DUTYCTL 			0x02
#define TG_GPOSR 			0x03
#define TG_GPODR1 			0x04
#define TG_GPODR2 			0x05
#define TG_PINICTL 			0x06
#define TG_HPOSCTL 			0x07

/*
 * PXA GPIOs
 */
#define TOSA_GPIO_POWERON		(0)
#define TOSA_GPIO_RESET			(1)
#define TOSA_GPIO_AC_IN			(2)
#define TOSA_GPIO_RECORD_BTN		(3)
#define TOSA_GPIO_SYNC			(4)	/* Cradle SYNC Button */
#define TOSA_GPIO_USB_IN		(5)
#define TOSA_GPIO_JACKET_DETECT		(7)
#define TOSA_GPIO_nSD_DETECT		(9)
#define TOSA_GPIO_nSD_INT		(10)
#define TOSA_GPIO_TC6393XB_CLK		(11)
#define TOSA_GPIO_BAT1_CRG		(12)
#define TOSA_GPIO_CF_CD			(13)
#define TOSA_GPIO_BAT0_CRG		(14)
#define TOSA_GPIO_TC6393XB_INT		(15)
#define TOSA_GPIO_BAT0_LOW		(17)
#define TOSA_GPIO_TC6393XB_RDY		(18)
#define TOSA_GPIO_ON_RESET		(19)
#define TOSA_GPIO_EAR_IN		(20)
#define TOSA_GPIO_CF_IRQ		(21)	/* CF slot0 Ready */
#define TOSA_GPIO_ON_KEY		(22)
#define TOSA_GPIO_VGA_LINE		(27)
#define TOSA_GPIO_TP_INT		(32)	/* Touch Panel pen down interrupt */
#define TOSA_GPIO_JC_CF_IRQ		(36)	/* CF slot1 Ready */
#define TOSA_GPIO_BAT_LOCKED		(38)	/* Battery locked */
#define TOSA_GPIO_IRDA_TX		(47)
#define TOSA_GPIO_TG_SPI_SCLK		(81)
#define TOSA_GPIO_TG_SPI_CS		(82)
#define TOSA_GPIO_TG_SPI_MOSI		(83)
#define TOSA_GPIO_BAT1_LOW		(84)

#define TOSA_GPIO_HP_IN			GPIO_EAR_IN

#define TOSA_GPIO_MAIN_BAT_LOW		GPIO_BAT0_LOW

#define TOSA_KEY_STROBE_NUM		(11)
#define TOSA_KEY_SENSE_NUM		(7)

#define TOSA_GPIO_HIGH_STROBE_BIT	(0xfc000000)
#define TOSA_GPIO_LOW_STROBE_BIT	(0x0000001f)
#define TOSA_GPIO_ALL_SENSE_BIT		(0x00000fe0)
#define TOSA_GPIO_ALL_SENSE_RSHIFT	(5)
#define TOSA_GPIO_STROBE_BIT(a)		GPIO_bit(58+(a))
#define TOSA_GPIO_SENSE_BIT(a)		GPIO_bit(69+(a))
#define TOSA_GAFR_HIGH_STROBE_BIT	(0xfff00000)
#define TOSA_GAFR_LOW_STROBE_BIT	(0x000003ff)
#define TOSA_GAFR_ALL_SENSE_BIT		(0x00fffc00)
#define TOSA_GPIO_KEY_SENSE(a) 		(69+(a))
#define TOSA_GPIO_KEY_STROBE(a)		(58+(a))

/*
 * Interrupts
 */
#define TOSA_IRQ_GPIO_WAKEUP        	IRQ_GPIO(TOSA_GPIO_WAKEUP)
#define TOSA_IRQ_GPIO_AC_IN         	IRQ_GPIO(TOSA_GPIO_AC_IN)
#define TOSA_IRQ_GPIO_RECORD_BTN    	IRQ_GPIO(TOSA_GPIO_RECORD_BTN)
#define TOSA_IRQ_GPIO_SYNC          	IRQ_GPIO(TOSA_GPIO_SYNC)
#define TOSA_IRQ_GPIO_USB_IN        	IRQ_GPIO(TOSA_GPIO_USB_IN)
#define TOSA_IRQ_GPIO_JACKET_DETECT 	IRQ_GPIO(TOSA_GPIO_JACKET_DETECT)
#define TOSA_IRQ_GPIO_nSD_INT       	IRQ_GPIO(TOSA_GPIO_nSD_INT)
#define TOSA_IRQ_GPIO_nSD_DETECT    	IRQ_GPIO(TOSA_GPIO_nSD_DETECT)
#define TOSA_IRQ_GPIO_BAT1_CRG      	IRQ_GPIO(TOSA_GPIO_BAT1_CRG)
#define TOSA_IRQ_GPIO_CF_CD         	IRQ_GPIO(TOSA_GPIO_CF_CD)
#define TOSA_IRQ_GPIO_BAT0_CRG      	IRQ_GPIO(TOSA_GPIO_BAT0_CRG)
#define TOSA_IRQ_GPIO_TC6393XB_INT    	IRQ_GPIO(TOSA_GPIO_TC6393XB_INT)
#define TOSA_IRQ_GPIO_BAT0_LOW      	IRQ_GPIO(TOSA_GPIO_BAT0_LOW)
#define TOSA_IRQ_GPIO_EAR_IN        	IRQ_GPIO(TOSA_GPIO_EAR_IN)
#define TOSA_IRQ_GPIO_CF_IRQ        	IRQ_GPIO(TOSA_GPIO_CF_IRQ)
#define TOSA_IRQ_GPIO_ON_KEY        	IRQ_GPIO(TOSA_GPIO_ON_KEY)
#define TOSA_IRQ_GPIO_VGA_LINE      	IRQ_GPIO(TOSA_GPIO_VGA_LINE)
#define TOSA_IRQ_GPIO_TP_INT        	IRQ_GPIO(TOSA_GPIO_TP_INT)
#define TOSA_IRQ_GPIO_JC_CF_IRQ     	IRQ_GPIO(TOSA_GPIO_JC_CF_IRQ)
#define TOSA_IRQ_GPIO_BAT_LOCKED    	IRQ_GPIO(TOSA_GPIO_BAT_LOCKED)
#define TOSA_IRQ_GPIO_BAT1_LOW      	IRQ_GPIO(TOSA_GPIO_BAT1_LOW)
#define TOSA_IRQ_GPIO_KEY_SENSE(a)  	IRQ_GPIO(69+(a))

#define TOSA_IRQ_GPIO_MAIN_BAT_LOW 	IRQ_GPIO(TOSA_GPIO_MAIN_BAT_LOW)

#define TOSA_KEY_SYNC		KEY_102ND /* ??? */

#ifndef CONFIG_TOSA_USE_EXT_KEYCODES
#define TOSA_KEY_RECORD		KEY_YEN
#define TOSA_KEY_ADDRESSBOOK	KEY_KATAKANA
#define TOSA_KEY_CANCEL		KEY_ESC
#define TOSA_KEY_CENTER		KEY_HIRAGANA
#define TOSA_KEY_OK		KEY_HENKAN
#define TOSA_KEY_CALENDAR	KEY_KATAKANAHIRAGANA
#define TOSA_KEY_HOMEPAGE	KEY_HANGEUL
#define TOSA_KEY_LIGHT		KEY_MUHENKAN
#define TOSA_KEY_MENU		KEY_HANJA
#define TOSA_KEY_FN		KEY_RIGHTALT
#define TOSA_KEY_MAIL		KEY_ZENKAKUHANKAKU
#else
#define TOSA_KEY_RECORD		KEY_RECORD
#define TOSA_KEY_ADDRESSBOOK	KEY_ADDRESSBOOK
#define TOSA_KEY_CANCEL		KEY_CANCEL
#define TOSA_KEY_CENTER		KEY_SELECT /* ??? */
#define TOSA_KEY_OK		KEY_OK
#define TOSA_KEY_CALENDAR	KEY_CALENDAR
#define TOSA_KEY_HOMEPAGE	KEY_HOMEPAGE
#define TOSA_KEY_LIGHT		KEY_KBDILLUMTOGGLE
#define TOSA_KEY_MENU		KEY_MENU
#define TOSA_KEY_FN		KEY_FN
#define TOSA_KEY_MAIL		KEY_MAIL
#endif

struct spi_device;
extern int tosa_bl_enable(struct spi_device *spi, int enable);

#endif /* _ASM_ARCH_TOSA_H_ */
