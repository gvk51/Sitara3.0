
#ifndef	_MDIO_H_
#define _MDIO_H_	

//#include <platform/platform_api.h>
//#include "gmac_hal_export.h"

/*------------------------------------------------------------------------------*/
/*))))))))))))))))))))))))))))))))))PHY ADDRESS(((((((((((((((((((((((((((((((((*/		
/*------------------------------------------------------------------------------*/
#define MDIO_PHY_ADDR								0x07

/*------------------------------------------------------------------------------*/
/*)))))))))))))))))))))))))))))))PHY REGISTER SET(((((((((((((((((((((((((((((((*/		
/*------------------------------------------------------------------------------*/
#define MDIO_CTRL_REG								0x00
#define MDIO_STATUS_REG								0x01
#define MDIO_PHY_ID0_REG							0x02
#define MDIO_PHY_ID1_REG							0x03
#define MDIO_AUTONEG_ADVERT_REG						0x04
#define MDIO_LINK_PARTNER_ABILITY					0x05
#define MDIO_AUTONEG_EXP_REG						0x06
#define MDIO_NXT_PAGE_TX_REG						0x07
#define MDIO_LINK_PARTNER_NXT_PG_REG				0x08
#define MDIO_1000_BASET_CTRL_REG					0x09
#define MDIO_1000_BASET_STATUS_REG					0x0A
#define	MDIO_EXTND_STATUS_REG						0x0F
#define	MDIO_PHY_SPEC_CTRL_REG						0x10
#define	MDIO_PHY_SPEC_STATUS_REG					0x11
#define	MDIO_INTR_EN_REG							0x12
#define	MDIO_INTR_STATUS_REG						0x13
#define	MDIO_EXTND_PHY_SPEC_CTRL_REG				0x14
#define	MDIO_RECV_ERR_CNT_REG						0x15
#define	MDIO_EXTND_ADDR_REG							0x16
#define	MDIO_GLOBAL_STATUS_REG						0x17
#define	MDIO_LED_CTRL_REG							0x18
#define	MDIO_MANUAL_LED_CTRL_REG					0x19
#define	MDIO_EXTND_PHY_SPEC_CTRL_REG2				0x1A
#define	MDIO_EXTND_PHY_SPEC_STATUS_REG				0x1B
#define	MDIO_TEST_REG								0x1C
#define	MDIO_EXTND_ADDR								0x1D
#define	MDIO_MISC_REG								0x1E


/*------------------------------------------------------------------------------*/
/*)))))))))))))))))))))))))))))))CTRL REG CONFIG((((((((((((((((((((((((((((((((*/		
/*------------------------------------------------------------------------------*/

//Control Register Configuration
#define MDIO_PHY_RESET								0x8000
#define MDIO_PHY_LOOPBACK							0x4000
#define MDIO_PHY_10MBPS								0X0000
#define MDIO_PHY_100MBPS							0X2000
#define MDIO_PHY_1000MBPS							0X0040
#define MDIO_PHY_FULL_DPLX							0x0100

#define MDIO_AUTO_DISABLE							0x0100

/*------------------------------------------------------------------------------*/
/*)))))))))))))))))))))))))))))))FUNC PROTOTYPEs((((((((((((((((((((((((((((((((*/		
/*------------------------------------------------------------------------------*/
/*DllImportExport_GEMAC_HWAPI void phy_soft_reset(void *handle);
DllImportExport_GEMAC_HWAPI void mdio_write(void*, uint16, uint16);
DllImportExport_GEMAC_HWAPI uint32 mdio_read(void*, uint16);
DllImportExport_GEMAC_HWAPI void phy_reg_read(void*);
DllImportExport_GEMAC_HWAPI void phy_config(void *);
*/

#endif
