#ifndef __IPC_CFG_HEADER_H
#define __IPC_CFG_HEADER_H



#define INDUS_CF1_OFFSET		0x0
#define INDUS_DEV_ID_SHIFT		0x10
#define INDUS_DEV_ID_MASK    	   	0x00000003
#define INDUS_VEN_ID_SHIFT    	   	0x00
#define INDUS_VEN_ID_MASK    	   	0x00000003
#define INDUS_RESERVED___SHIFT    	0x00
#define INDUS_RESERVED___MASK    	0x00000001

#define INDUS_CF2_OFFSET  		0x4
#define INDUS_PERR_S_SHIFT    	   	0x1F
#define INDUS_PERR_S_MASK    	   	0x00000001
#define INDUS_SERR_S_SHIFT    	   	0x1E
#define INDUS_SERR_S_MASK    	   	0x00000001
#define INDUS_M_AB_S_SHIFT    	   	0x1D
#define INDUS_M_AB_S_MASK    	   	0x00000001
#define INDUS_M_TA_S_SHIFT    	   	0x1C
#define INDUS_M_TA_S_MASK    	   	0x00000001
#define INDUS_S_TA_S_SHIFT    	   	0x1B
#define INDUS_S_TA_S_MASK    	   	0x00000001
#define INDUS_DS_TIM_SHIFT    	   	0x19
#define INDUS_DS_TIM_MASK    	   	0x00000003
#define INDUS_ERR_RSP_SHIFT    	   	0x18
#define INDUS_ERR_RSP_MASK    	   	0x00000001
#define INDUS_B2B_CP_SHIFT    	   	0x17
#define INDUS_B2B_CP_MASK    	   	0x00000001
#define INDUS_RSV3_SHIFT    	   	0x16
#define INDUS_RSV3_MASK    	   	0x00000001
#define INDUS_66M_CP_SHIFT    	   	0x15
#define INDUS_66M_CP_MASK    	   	0x00000001
#define INDUS_CPTR_PR_SHIFT    	   	0x14
#define INDUS_CPTR_PR_MASK    	   	0x00000001
#define INDUS_INTR_S_SHIFT    	   	0x13
#define INDUS_INTR_S_MASK    	   	0x00000001
#define INDUS_RSV2_SHIFT    	   	0x0B
#define INDUS_RSV2_MASK    	   	0x00000003
#define INDUS_INTR_D_SHIFT    	   	0x0A
#define INDUS_INTR_D_MASK    	   	0x00000001
#define INDUS_B2B_EN_SHIFT    	   	0x09
#define INDUS_B2B_EN_MASK    	   	0x00000001
#define INDUS_SER_EN_SHIFT    	   	0x08
#define INDUS_SER_EN_MASK    	   	0x00000001
#define INDUS_RSV1_SHIFT    	   	0x07
#define INDUS_RSV1_MASK    	   	0x00000001
#define INDUS_PER_RP_SHIFT    	   	0x06
#define INDUS_PER_RP_MASK    	   	0x00000001
#define INDUS_PAT_EN_SHIFT    	   	0x05
#define INDUS_PAT_EN_MASK    	   	0x00000001
#define INDUS_MWI_EN_SHIFT    	   	0x04
#define INDUS_MWI_EN_MASK    	   	0x00000001
#define INDUS_SP_CYC_SHIFT    	   	0x03
#define INDUS_SP_CYC_MASK    	   	0x00000001
#define INDUS_BUS_M_SHIFT    	   	0x02
#define INDUS_BUS_M_MASK    	   	0x00000001
#define INDUS_MMS_EN_SHIFT    	   	0x01
#define INDUS_MMS_EN_MASK    	   	0x00000001
#define INDUS_IOS_EN_SHIFT    	  	0x00
#define INDUS_IOS_EN_MASK    	   	0x00000001
#define INDUS_RESERVED___SHIFT    	0x00
#define INDUS_RESERVED___MASK    	0x00000001

#define INDUS_CF3_OFFSET  		0x8
#define INDUS_BAS_CLAS_SHIFT    	0x18
#define INDUS_BAS_CLAS_MASK    	   	0x00000003
#define INDUS_SUB_CLAS_SHIFT    	0x10
#define INDUS_SUB_CLAS_MASK    	   	0x00000003
#define INDUS_CLAS_PIF_SHIFT    	0x08
#define INDUS_CLAS_PIF_MASK    	   	0x00000003
#define INDUS_REV_ID_SHIFT    	   	0x00
#define INDUS_REV_ID_MASK    	   	0x00000003
#define INDUS_RESERVED___SHIFT    	0x00
#define INDUS_RESERVED___MASK    	0x00000001

#define INDUS_CF4_OFFSET  		0xC
#define INDUS_BIST_SHIFT    	   	0x18
#define INDUS_BIST_MASK    	   	0x00000003
#define INDUS_HEADER_T_SHIFT    	0x10
#define INDUS_HEADER_T_MASK    	   	0x00000003
#define INDUS_LAT_TIM_SHIFT    	   	0x08
#define INDUS_LAT_TIM_MASK    	   	0x00000003
#define INDUS_CLINE_SZ_SHIFT    	0x00
#define INDUS_CLINE_SZ_MASK    	   	0x00000003
#define INDUS_RESERVED___SHIFT    	0x00
#define INDUS_RESERVED___MASK    	0x00000001

#define INDUS_BAR0_OFFSET  		0x10
#define INDUS_BA_MSB_SHIFT    	   	0x0A
#define INDUS_BA_MSB_MASK    	   	0x00000003
#define INDUS_BA_RO_SHIFT    	   	0x04
#define INDUS_BA_RO_MASK    	   	0x00000003
#define INDUS_PRF_SHIFT    	   	0x03
#define INDUS_PRF_MASK    	   	0x00000001
#define INDUS_ADDR_TY_SHIFT    	   	0x01
#define INDUS_ADDR_TY_MASK    	   	0x00000003
#define INDUS_RESERVED___SHIFT    	0x00
#define INDUS_RESERVED___MASK    	0x00000001
#define INDUS_MMS_IN_SHIFT    	   	0x00
#define INDUS_MMS_IN_MASK    	   	0x00000001

#define INDUS_BAR1_OFFSET  		0x14
#define INDUS_BA_MSB_SHIFT    	   	0x0A
#define INDUS_BA_MSB_MASK    	   	0x00000003
#define INDUS_BA_RO_SHIFT    	   	0x04
#define INDUS_BA_RO_MASK    	   	0x00000003
#define INDUS_PRF_SHIFT    	   	0x03
#define INDUS_PRF_MASK    	   	0x00000001
#define INDUS_ADDR_TY_SHIFT    	   	0x01
#define INDUS_ADDR_TY_MASK    	   	0x00000003
#define INDUS_RESERVED___SHIFT    	0x00
#define INDUS_RESERVED___MASK    	0x00000001
#define INDUS_MMS_IN_SHIFT    	   	0x00
#define INDUS_MMS_IN_MASK    	   	0x00000001

#define INDUS_BAR2_OFFSET  		0x18
#define INDUS_BA_MSB_SHIFT    	   	0x0A
#define INDUS_BA_MSB_MASK    	   	0x00000003
#define INDUS_BA_RO_SHIFT    	   	0x04
#define INDUS_BA_RO_MASK    	   	0x00000003
#define INDUS_PRF_SHIFT    	   	0x03
#define INDUS_PRF_MASK    	   	0x00000001
#define INDUS_ADDR_TY_SHIFT    	   	0x01
#define INDUS_ADDR_TY_MASK    	   	0x00000003
#define INDUS_RESERVED___SHIFT    	0x00
#define INDUS_RESERVED___MASK    	0x00000001
#define INDUS_MMS_IN_SHIFT    	   	0x00
#define INDUS_MMS_IN_MASK    	   	0x00000001

#define INDUS_BAR3_OFFSET  		0x1C
#define INDUS_BA_MSB_SHIFT    	   	0x0A
#define INDUS_BA_MSB_MASK    	   	0x00000003
#define INDUS_BA_RO_SHIFT    	   	0x04
#define INDUS_BA_RO_MASK    	   	0x00000003
#define INDUS_PRF_SHIFT    	   	0x03
#define INDUS_PRF_MASK    	   	0x00000001
#define INDUS_ADDR_TY_SHIFT    	   	0x01
#define INDUS_ADDR_TY_MASK    	   	0x00000003
#define INDUS_RESERVED___SHIFT    	0x00
#define INDUS_RESERVED___MASK    	0x00000001
#define INDUS_MMS_IN_SHIFT    	   	0x00
#define INDUS_MMS_IN_MASK    	   	0x00000001

#define INDUS_BAR4_OFFSET  		0x20
#define INDUS_BA_MSB_SHIFT    	   	0x0A
#define INDUS_BA_MSB_MASK    	   	0x00000003
#define INDUS_BA_RO_SHIFT    	   	0x04
#define INDUS_BA_RO_MASK    	   	0x00000003
#define INDUS_PRF_SHIFT    	   	0x03
#define INDUS_PRF_MASK    	   	0x00000001
#define INDUS_ADDR_TY_SHIFT    	   	0x01
#define INDUS_ADDR_TY_MASK    	   	0x00000003
#define INDUS_RESERVED___SHIFT    	0x00
#define INDUS_RESERVED___MASK    	0x00000001
#define INDUS_MMS_IN_SHIFT    	   	0x00
#define INDUS_MMS_IN_MASK    	   	0x00000001

#define INDUS_BAR5_OFFSET  		0x24
#define INDUS_BA_MSB_SHIFT    	   	0x0A
#define INDUS_BA_MSB_MASK    	   	0x00000003
#define INDUS_BA_RO_SHIFT    	   	0x04
#define INDUS_BA_RO_MASK    	   	0x00000003
#define INDUS_PRF_SHIFT    	   	0x03
#define INDUS_PRF_MASK    	   	0x00000001
#define INDUS_ADDR_TY_SHIFT    	   	0x01
#define INDUS_ADDR_TY_MASK    	   	0x00000003
#define INDUS_MMS_IN_SHIFT    	   	0x00
#define INDUS_MMS_IN_MASK    	   	0x00000001
#define INDUS_RESERVED___SHIFT    	0x00
#define INDUS_RESERVED___MASK    	0x00000001

#define INDUS_CF11_OFFSET  		0x28
#define INDUS_CIS_PTR_SHIFT    	   	0x00
#define INDUS_CIS_PTR_MASK    	   	0x00000003
#define INDUS_RESERVED___SHIFT    	0x00
#define INDUS_RESERVED___MASK    	0x00000001

#define INDUS_CF12_OFFSET  		0x2C
#define INDUS_SUBS_ID_SHIFT    	   	0x10
#define INDUS_SUBS_ID_MASK    	   	0x00000003
#define INDUS_S_VENDID_SHIFT    	0x00
#define INDUS_S_VENDID_MASK    	   	0x00000003
#define INDUS_RESERVED___SHIFT    	0x00
#define INDUS_RESERVED___MASK    	0x00000001

#define INDUS_CF13_OFFSET  		0x30
#define INDUS_EROM_BA_SHIFT    	   	0x0B
#define INDUS_EROM_BA_MASK    	   	0x00000003
#define INDUS_RSV_SHIFT    	   	0x01
#define INDUS_RSV_MASK    	   	0x00000003
#define INDUS_EROM_EN_SHIFT    	   	0x00
#define INDUS_EROM_EN_MASK    	   	0x00000001
#define INDUS_RESERVED___SHIFT    	0x00
#define INDUS_RESERVED___MASK    	0x00000001

#define INDUS_CF14_OFFSET  		0x34
#define INDUS_CAP_PTR_SHIFT    	   	0x02
#define INDUS_CAP_PTR_MASK    	   	0x00000003
#define INDUS_CPTR_LS_SHIFT    	   	0x00
#define INDUS_CPTR_LS_MASK    	   	0x00000003
#define INDUS_RESERVED___SHIFT    	0x00
#define INDUS_RESERVED___MASK    	0x00000001

#define INDUS_CF15_OFFSET  		0x38
#define INDUS_RSV_SHIFT    	   	0x00
#define INDUS_RSV_MASK    	   	0x00000003
#define INDUS_RESERVED___SHIFT    	0x00
#define INDUS_RESERVED___MASK    	0x00000001

#define INDUS_CF16_OFFSET  		0x3C
#define INDUS_MAX_LAT_SHIFT    	   	0x18
#define INDUS_MAX_LAT_MASK    	   	0x00000003
#define INDUS_MIN_GNT_SHIFT    	   	0x10
#define INDUS_MIN_GNT_MASK    	   	0x00000003
#define INDUS_INT_PIN_SHIFT    	   	0x08
#define INDUS_INT_PIN_MASK    	   	0x00000003
#define INDUS_INT_LIN_SHIFT    	   	0x00
#define INDUS_INT_LIN_MASK    	   	0x00000003
#define INDUS_RESERVED___SHIFT    	0x00
#define INDUS_RESERVED___MASK    	0x00000001

#define INDUS_MSI1_OFFSET  		0x40
#define INDUS_RSV_SHIFT    	   	0x19
#define INDUS_RSV_MASK    	   	0x00000003
#define INDUS_PVEC_MEN_SHIFT    	0x18
#define INDUS_PVEC_MEN_MASK    	   	0x00000001
#define INDUS_64B_A_CAP_SHIFT    	0x17
#define INDUS_64B_A_CAP_MASK    	0x00000001
#define INDUS_MMSG_EN_SHIFT    	   	0x14
#define INDUS_MMSG_EN_MASK    	   	0x00000003
#define INDUS_MMSG_CP_SHIFT    	   	0x11
#define INDUS_MMSG_CP_MASK    	   	0x00000003
#define INDUS_MSI_EN_SHIFT    	   	0x10
#define INDUS_MSI_EN_MASK    	   	0x00000001
#define INDUS_NEXT_ID_SHIFT    	   	0x08
#define INDUS_NEXT_ID_MASK    	   	0x00000003
#define INDUS_CAP_ID_SHIFT    	   	0x00
#define INDUS_CAP_ID_MASK    	   	0x00000003
#define INDUS_RESERVED___SHIFT    	0x00
#define INDUS_RESERVED___MASK    	0x00000001

#define INDUS_MSI2_OFFSET  0x44
#define INDUS_MSG_A_SHIFT    	   	0x02
#define INDUS_MSG_A_MASK    	   	0x00000003
#define INDUS_MSG_AL_SHIFT    	   	0x00
#define INDUS_MSG_AL_MASK    	   	0x00000003
#define INDUS_RESERVED___SHIFT    	0x00
#define INDUS_RESERVED___MASK    	0x00000001

#define INDUS_MSI3_OFFSET  		0x48
#define INDUS_MSG_UA_SHIFT    	   	0x00
#define INDUS_MSG_UA_MASK    	   	0x00000003
#define INDUS_RESERVED___SHIFT    	0x00
#define INDUS_RESERVED___MASK    	0x00000001

#define INDUS_MSI4_OFFSET  		0x4C
#define INDUS_RSV_SHIFT    	  	0x10
#define INDUS_RSV_MASK    	   	0x00000003
#define INDUS_DATA_SHIFT    	   	0x00
#define INDUS_DATA_MASK    	   	0x00000003
#define INDUS_RESERVED___SHIFT    	0x00
#define INDUS_RESERVED___MASK    	0x00000001

#define INDUS_MSI5_OFFSET  		0x50
#define INDUS_MSKBITS_SHIFT    	   	0x00
#define INDUS_MSKBITS_MASK    	   	0x00000003
#define INDUS_RESERVED___SHIFT    	0x00
#define INDUS_RESERVED___MASK    	0x00000001

#define INDUS_MSI6_OFFSET  		0x54
#define INDUS_PENBITS_SHIFT    	   	0x00
#define INDUS_PENBITS_MASK    	   	0x00000003
#define INDUS_RESERVED___SHIFT    	0x00
#define INDUS_RESERVED___MASK    	0x00000001

#define INDUS_PWR_MAN_CAP_OFFSET  	0x60
#define INDUS_PME_SUP_SHIFT    	   	0x1B
#define INDUS_PME_SUP_MASK    	   	0x00000003
#define INDUS_D2_SUP_SHIFT    	   	0x1A
#define INDUS_D2_SUP_MASK    	   	0x00000001
#define INDUS_D1_SUP_SHIFT    	   	0x19
#define INDUS_D1_SUP_MASK    	   	0x00000001
#define INDUS_AUX_CUR_SHIFT    	   	0x16
#define INDUS_AUX_CUR_MASK    	   	0x00000003
#define INDUS_DSI_SHIFT    	   	0x15
#define INDUS_DSI_MASK    	   	0x00000001
#define INDUS_RESERVED_20_SHIFT    	0x14
#define INDUS_RESERVED_20_MASK    	0x00000001
#define INDUS_PME_CLK_SHIFT    	   	0x13
#define INDUS_PME_CLK_MASK    	   	0x00000001
#define INDUS_VER_SHIFT    	   	0x10
#define INDUS_VER_MASK    	   	0x00000003
#define INDUS_NXT_ITEM_PTR_SHIFT    	0x08
#define INDUS_NXT_ITEM_PTR_MASK    	0x00000003
#define INDUS_CAP_ID_SHIFT    	   	0x00
#define INDUS_CAP_ID_MASK    	   	0x00000003
#define INDUS_RESERVED___SHIFT    	0x00
#define INDUS_RESERVED___MASK    	0x00000001

#define INDUS_PWR_MANG_OFFSET  		0x64
#define INDUS_RESERVED_15_8_SHIFT    	0x08
#define INDUS_RESERVED_15_8_MASK    	0x000000FF
#define INDUS_PME_STS_SHIFT    	   	0x07
#define INDUS_PME_STS_MASK    	   	0x00000001
#define INDUS_RESERVED_6_2_SHIFT    	0x02
#define INDUS_RESERVED_6_2_MASK    	0x0000001F
#define INDUS_POW_STATE_SHIFT    	0x00
#define INDUS_POW_STATE_MASK    	0x00000003
#define INDUS_PME_EN_SHIFT    	   	0x00
#define INDUS_PME_EN_MASK    	   	0x00000001

#define INDUS_PCIE_CAP_LIST_OFFSET  	0x80
#define INDUS_RESERVED_31_30_SHIFT    	0x1E
#define INDUS_RESERVED_31_30_MASK    	0x00000003
#define INDUS_INTR_MSG_NUM_SHIFT    	0x19
#define INDUS_INTR_MSG_NUM_MASK    	0x00000003
#define INDUS_RESERVED_24_SHIFT    	0x18
#define INDUS_RESERVED_24_MASK    	0x00000001
#define INDUS_DEV_PRT_TYPE_SHIFT    	0x14
#define INDUS_DEV_PRT_TYPE_MASK    	0x00000003
#define INDUS_CAP_VER_SHIFT    	   	0x10
#define INDUS_CAP_VER_MASK    	   	0x00000003
#define INDUS_NXT_CAP_PTR_SHIFT    	0x08
#define INDUS_NXT_CAP_PTR_MASK    	0x00000003
#define INDUS_CAP_ID_SHIFT    	   	0x00
#define INDUS_CAP_ID_MASK    	   	0x00000003

#define INDUS_PCIE_CAP_OFFSET  		0x84
#define INDUS_RESERVED_31_29_SHIFT    	0x1D
#define INDUS_RESERVED_31_29_MASK    	0x00000007
#define INDUS_FN_LVL_RST_CAP_SHIFT    	0x1C
#define INDUS_FN_LVL_RST_CAP_MASK    	0x00000001
#define INDUS_RESERVED_27_16_SHIFT    	0x10
#define INDUS_RESERVED_27_16_MASK    	0x00000FFF
#define INDUS_ROLE_BSD_ERR_REP_SHIFT    0x0F
#define INDUS_ROLE_BSD_ERR_REP_MASK    	0x00000001
#define INDUS_RESERVED_14_6_SHIFT    	0x06
#define INDUS_RESERVED_14_6_MASK    	0x000001FF
#define INDUS_EXTD_TAG_FLD_SUP_SHIFT    0x05
#define INDUS_EXTD_TAG_FLD_SUP_MASK    	0x00000001
#define INDUS_PHANT_FN_SUP_SHIFT    	0x03
#define INDUS_PHANT_FN_SUP_MASK    	0x00000003
#define INDUS_MAX_PAYLD_SIZE_SUP_SHIFT  0x00
#define INDUS_MAX_PAYLD_SIZE_SUP_MASK   0x00000003

#define INDUS_PCIE_CNTRL_OFFSET  	0x88
#define INDUS_RESERVED_31_16_SHIFT    	0x10
#define INDUS_RESERVED_31_16_MASK    	0x0000FFFF
#define INDUS_INIT_FN_LVL_RST_SHIFT    	0x0F
#define INDUS_INIT_FN_LVL_RST_MASK    	0x00000001
#define INDUS_RESERVED_14_4_SHIFT    	0x04
#define INDUS_RESERVED_14_4_MASK    	0x000007FF
#define INDUS_UR_REP_EN_SHIFT    	0x03
#define INDUS_UR_REP_EN_MASK    	0x00000001
#define INDUS_RESERVED_2_SHIFT    	0x02
#define INDUS_RESERVED_2_MASK    	0x00000001
#define INDUS_NF_ERR_REP_EN_SHIFT    	0x01
#define INDUS_NF_ERR_REP_EN_MASK    	0x00000001

#define INDUS_PCIE_STS_OFFSET  		0x8A
#define INDUS_RESERVED_31_20_SHIFT    	0x14
#define INDUS_RESERVED_31_20_MASK    	0x00000FFF
#define INDUS_UNSUP_REQ_SHIFT    	0x13
#define INDUS_UNSUP_REQ_MASK    	0x00000001
#define INDUS_RESERVED_18_SHIFT    	0x12
#define INDUS_RESERVED_18_MASK    	0x00000001
#define INDUS_NON_FATAL_ERR_DET_SHIFT   0x11
#define INDUS_NON_FATAL_ERR_DET_MASK    0x00000001

#endif  /* __IPC_CFG_HEADER_H */
