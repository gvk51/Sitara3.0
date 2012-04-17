#define INDUS_RECV_CTRL_OFFSET  0x400
#define INDUS_RECV_CTRL_DEFAULT_VAL  0x00000000
#define INDUS_RECV_CTRL_VALID_MASK  0x000FFFFF
#define INDUS_RECV_CTRL_WRITE_MASK  0x000FFFFF
#define INDUS_RECV_CTRL_READ_MASK  0x000FFFFF
#define INDUS_RECV_CTRL_RESERVED_31_20_MASK    	   0xFFF00000
#define INDUS_RECV_CTRL_CSE    	   0x00080000
#define INDUS_RECV_CTRL_VFB    	   0x00070000
#define INDUS_RECV_CTRL_MB    	   0x0000E000
#define INDUS_RECV_CTRL_RPA    	   0x00001000
#define INDUS_RECV_CTRL_VSE    	   0x00000800
#define INDUS_RECV_CTRL_SYNQFP    	   0x00000400
#define INDUS_RECV_CTRL_FLEP    	   0x00000200
#define INDUS_RECV_CTRL_PMCF    	   0x00000100
#define INDUS_RECV_CTRL_DPF    	   0x00000080
#define INDUS_RECV_CTRL_CFI    	   0x00000040
#define INDUS_RECV_CTRL_CFI_EN    	   0x00000020
#define INDUS_RECV_CTRL_VFE    	   0x00000010
#define INDUS_RECV_CTRL_BCAST_EN    	   0x00000008
#define INDUS_RECV_CTRL_MPE    	   0x00000004
#define INDUS_RECV_CTRL_UPE    	   0x00000002
#define INDUS_RECV_CTRL_SBP    	   0x00000001

#define INDUS_VLAN_HASH0_OFFSET  0x404
#define INDUS_VLAN_HASH0_DEFAULT_VAL  0x00000000
#define INDUS_VLAN_HASH0_VALID_MASK  0xFFFFFFFF
#define INDUS_VLAN_HASH0_WRITE_MASK  0xFFFFFFFF
#define INDUS_VLAN_HASH0_READ_MASK  0xFFFFFFFF

#define INDUS_VLAN_HASH1_OFFSET  0x408
#define INDUS_VLAN_HASH1_DEFAULT_VAL  0x00000000
#define INDUS_VLAN_HASH1_VALID_MASK  0xFFFFFFFF
#define INDUS_VLAN_HASH1_WRITE_MASK  0xFFFFFFFF
#define INDUS_VLAN_HASH1_READ_MASK  0xFFFFFFFF

#define INDUS_VLAN_HASH2_OFFSET  0x40c
#define INDUS_VLAN_HASH2_DEFAULT_VAL  0x00000000
#define INDUS_VLAN_HASH2_VALID_MASK  0xFFFFFFFF
#define INDUS_VLAN_HASH2_WRITE_MASK  0xFFFFFFFF
#define INDUS_VLAN_HASH2_READ_MASK  0xFFFFFFFF

#define INDUS_VLAN_HASH3_OFFSET  0x410
#define INDUS_VLAN_HASH3_DEFAULT_VAL  0x00000000
#define INDUS_VLAN_HASH3_VALID_MASK  0xFFFFFFFF
#define INDUS_VLAN_HASH3_WRITE_MASK  0xFFFFFFFF
#define INDUS_VLAN_HASH3_READ_MASK  0xFFFFFFFF

#define INDUS_VLAN_HASH4_OFFSET  0x414
#define INDUS_VLAN_HASH4_DEFAULT_VAL  0x00000000
#define INDUS_VLAN_HASH4_VALID_MASK  0xFFFFFFFF
#define INDUS_VLAN_HASH4_WRITE_MASK  0xFFFFFFFF
#define INDUS_VLAN_HASH4_READ_MASK  0xFFFFFFFF

#define INDUS_VLAN_HASH5_OFFSET  0x418
#define INDUS_VLAN_HASH5_DEFAULT_VAL  0x00000000
#define INDUS_VLAN_HASH5_VALID_MASK  0xFFFFFFFF
#define INDUS_VLAN_HASH5_WRITE_MASK  0xFFFFFFFF
#define INDUS_VLAN_HASH5_READ_MASK  0xFFFFFFFF

#define INDUS_VLAN_HASH6_OFFSET  0x41c
#define INDUS_VLAN_HASH6_DEFAULT_VAL  0x00000000
#define INDUS_VLAN_HASH6_VALID_MASK  0xFFFFFFFF
#define INDUS_VLAN_HASH6_WRITE_MASK  0xFFFFFFFF
#define INDUS_VLAN_HASH6_READ_MASK  0xFFFFFFFF

#define INDUS_VLAN_HASH7_OFFSET  0x420
#define INDUS_VLAN_HASH7_DEFAULT_VAL  0x00000000
#define INDUS_VLAN_HASH7_VALID_MASK  0xFFFFFFFF
#define INDUS_VLAN_HASH7_WRITE_MASK  0xFFFFFFFF
#define INDUS_VLAN_HASH7_READ_MASK  0xFFFFFFFF

#define INDUS_VLAN_PRIORITY_OFFSET  0x424
#define INDUS_VLAN_PRIORITY_DEFAULT_VAL  0x00000000
#define INDUS_VLAN_PRIORITY_VALID_MASK  0x00000007
#define INDUS_VLAN_PRIORITY_WRITE_MASK  0x00000007
#define INDUS_VLAN_PRIORITY_READ_MASK  0x00000007
#define INDUS_VLAN_PRIORITY_RESERVED_31_3_MASK    	   0xFFFFFFF8

#define INDUS_RCV_CKSM_CTRL_OFFSET  0x428
#define INDUS_RCV_CKSM_CTRL_DEFAULT_VAL  0x00000000
#define INDUS_RCV_CKSM_CTRL_VALID_MASK  0x00001FFF
#define INDUS_RCV_CKSM_CTRL_WRITE_MASK  0x00001FFF
#define INDUS_RCV_CKSM_CTRL_READ_MASK  0x00001FFF
#define INDUS_RCV_CKSM_CTRL_RESERVED_31_13_MASK    	   0xFFFFE000
#define INDUS_RCV_CKSM_CTRL_PCSD    	   0x00001000
#define INDUS_RCV_CKSM_CTRL_IPPLCSEN    	   0x00000800
#define INDUS_RCV_CKSM_CTRL_CRCOLEN    	   0x00000400
#define INDUS_RCV_CKSM_CTRL_TUOLEN    	   0x00000200
#define INDUS_RCV_CKSM_CTRL_IPCSOLEN    	   0x00000100
#define INDUS_RCV_CKSM_CTRL_PCSB    	   0x000000FF

#define INDUS_RSS_EN0_OFFSET  0x42c
#define INDUS_RSS_EN0_DEFAULT_VAL  0x00000000
#define INDUS_RSS_EN0_VALID_MASK  0x00003FFF
#define INDUS_RSS_EN0_WRITE_MASK  0x00003FFF
#define INDUS_RSS_EN0_READ_MASK  0x00003FFF
#define INDUS_RSS_EN0_RESERVED_31_14_MASK    	   0xFFFFC000
#define INDUS_RSS_EN0_RSS_FIELD_EN    	   0x00003FE0
#define INDUS_RSS_EN0_RSVD    	   0x00000018
#define INDUS_RSS_EN0_DEF_QUEUE    	   0x00000006
#define INDUS_RSS_EN0_RSS_Q_EN    	   0x00000001

#define INDUS_RSS_KEY0_OFFSET  0x430
#define INDUS_RSS_KEY0_DEFAULT_VAL  0x00000000
#define INDUS_RSS_KEY0_VALID_MASK  0xFFFFFFFF
#define INDUS_RSS_KEY0_WRITE_MASK  0xFFFFFFFF
#define INDUS_RSS_KEY0_READ_MASK  0xFFFFFFFF
#define INDUS_RSS_KEY0_K3    	   0xFF000000
#define INDUS_RSS_KEY0_K2    	   0x00FF0000
#define INDUS_RSS_KEY0_K1    	   0x0000FF00
#define INDUS_RSS_KEY0_K0    	   0x000000FF

#define INDUS_RSS_KEY1_OFFSET  0x434
#define INDUS_RSS_KEY1_DEFAULT_VAL  0x00000000
#define INDUS_RSS_KEY1_VALID_MASK  0xFFFFFFFF
#define INDUS_RSS_KEY1_WRITE_MASK  0xFFFFFFFF
#define INDUS_RSS_KEY1_READ_MASK  0xFFFFFFFF
#define INDUS_RSS_KEY1_K3    	   0xFF000000
#define INDUS_RSS_KEY1_K2    	   0x00FF0000
#define INDUS_RSS_KEY1_K1    	   0x0000FF00
#define INDUS_RSS_KEY1_K0    	   0x000000FF

#define INDUS_RSS_KEY2_OFFSET  0x438
#define INDUS_RSS_KEY2_DEFAULT_VAL  0x00000000
#define INDUS_RSS_KEY2_VALID_MASK  0xFFFFFFFF
#define INDUS_RSS_KEY2_WRITE_MASK  0xFFFFFFFF
#define INDUS_RSS_KEY2_READ_MASK  0xFFFFFFFF
#define INDUS_RSS_KEY2_K3    	   0xFF000000
#define INDUS_RSS_KEY2_K2    	   0x00FF0000
#define INDUS_RSS_KEY2_K1    	   0x0000FF00
#define INDUS_RSS_KEY2_K0    	   0x000000FF

#define INDUS_RSS_KEY3_OFFSET  0x43c
#define INDUS_RSS_KEY3_DEFAULT_VAL  0x00000000
#define INDUS_RSS_KEY3_VALID_MASK  0xFFFFFFFF
#define INDUS_RSS_KEY3_WRITE_MASK  0xFFFFFFFF
#define INDUS_RSS_KEY3_READ_MASK  0xFFFFFFFF
#define INDUS_RSS_KEY3_K3    	   0xFF000000
#define INDUS_RSS_KEY3_K2    	   0x00FF0000
#define INDUS_RSS_KEY3_K1    	   0x0000FF00
#define INDUS_RSS_KEY3_K0    	   0x000000FF

#define INDUS_RSS_KEY4_OFFSET  0x440
#define INDUS_RSS_KEY4_DEFAULT_VAL  0x00000000
#define INDUS_RSS_KEY4_VALID_MASK  0xFFFFFFFF
#define INDUS_RSS_KEY4_WRITE_MASK  0xFFFFFFFF
#define INDUS_RSS_KEY4_READ_MASK  0xFFFFFFFF
#define INDUS_RSS_KEY4_K3    	   0xFF000000
#define INDUS_RSS_KEY4_K2    	   0x00FF0000
#define INDUS_RSS_KEY4_K1    	   0x0000FF00
#define INDUS_RSS_KEY4_K0    	   0x000000FF

#define INDUS_RSS_KEY5_OFFSET  0x444
#define INDUS_RSS_KEY5_DEFAULT_VAL  0x00000000
#define INDUS_RSS_KEY5_VALID_MASK  0xFFFFFFFF
#define INDUS_RSS_KEY5_WRITE_MASK  0xFFFFFFFF
#define INDUS_RSS_KEY5_READ_MASK  0xFFFFFFFF
#define INDUS_RSS_KEY5_K3    	   0xFF000000
#define INDUS_RSS_KEY5_K2    	   0x00FF0000
#define INDUS_RSS_KEY5_K1    	   0x0000FF00
#define INDUS_RSS_KEY5_K0    	   0x000000FF

#define INDUS_RSS_KEY6_OFFSET  0x448
#define INDUS_RSS_KEY6_DEFAULT_VAL  0x00000000
#define INDUS_RSS_KEY6_VALID_MASK  0xFFFFFFFF
#define INDUS_RSS_KEY6_WRITE_MASK  0xFFFFFFFF
#define INDUS_RSS_KEY6_READ_MASK  0xFFFFFFFF
#define INDUS_RSS_KEY6_K3    	   0xFF000000
#define INDUS_RSS_KEY6_K2    	   0x00FF0000
#define INDUS_RSS_KEY6_K1    	   0x0000FF00
#define INDUS_RSS_KEY6_K0    	   0x000000FF

#define INDUS_RSS_KEY7_OFFSET  0x44c
#define INDUS_RSS_KEY7_DEFAULT_VAL  0x00000000
#define INDUS_RSS_KEY7_VALID_MASK  0xFFFFFFFF
#define INDUS_RSS_KEY7_WRITE_MASK  0xFFFFFFFF
#define INDUS_RSS_KEY7_READ_MASK  0xFFFFFFFF
#define INDUS_RSS_KEY7_K3    	   0xFF000000
#define INDUS_RSS_KEY7_K2    	   0x00FF0000
#define INDUS_RSS_KEY7_K1    	   0x0000FF00
#define INDUS_RSS_KEY7_K0    	   0x000000FF

#define INDUS_RSS_KEY8_OFFSET  0x450
#define INDUS_RSS_KEY8_DEFAULT_VAL  0x00000000
#define INDUS_RSS_KEY8_VALID_MASK  0xFFFFFFFF
#define INDUS_RSS_KEY8_WRITE_MASK  0xFFFFFFFF
#define INDUS_RSS_KEY8_READ_MASK  0xFFFFFFFF
#define INDUS_RSS_KEY8_K3    	   0xFF000000
#define INDUS_RSS_KEY8_K2    	   0x00FF0000
#define INDUS_RSS_KEY8_K1    	   0x0000FF00
#define INDUS_RSS_KEY8_K0    	   0x000000FF

#define INDUS_RSS_KEY9_OFFSET  0x454
#define INDUS_RSS_KEY9_DEFAULT_VAL  0x00000000
#define INDUS_RSS_KEY9_VALID_MASK  0xFFFFFFFF
#define INDUS_RSS_KEY9_WRITE_MASK  0xFFFFFFFF
#define INDUS_RSS_KEY9_READ_MASK  0xFFFFFFFF
#define INDUS_RSS_KEY9_K3    	   0xFF000000
#define INDUS_RSS_KEY9_K2    	   0x00FF0000
#define INDUS_RSS_KEY9_K1    	   0x0000FF00
#define INDUS_RSS_KEY9_K0    	   0x000000FF

#define INDUS_RSS_IT0_OFFSET  0x458
#define INDUS_RSS_IT0_DEFAULT_VAL  0x00000000
#define INDUS_RSS_IT0_VALID_MASK  0xFFFFFFFF
#define INDUS_RSS_IT0_WRITE_MASK  0xFFFFFFFF
#define INDUS_RSS_IT0_READ_MASK  0xFFFFFFFF
#define INDUS_RSS_IT0_ENTRY15    	   0xC0000000
#define INDUS_RSS_IT0_ENTRY14    	   0x30000000
#define INDUS_RSS_IT0_ENTRY13    	   0x0C000000
#define INDUS_RSS_IT0_ENTRY12    	   0x03000000
#define INDUS_RSS_IT0_ENTRY11    	   0x00C00000
#define INDUS_RSS_IT0_ENTRY10    	   0x00300000
#define INDUS_RSS_IT0_ENTRY9    	   0x000C0000
#define INDUS_RSS_IT0_ENTRY8    	   0x00030000
#define INDUS_RSS_IT0_ENTRY7    	   0x0000C000
#define INDUS_RSS_IT0_ENTRY6    	   0x00003000
#define INDUS_RSS_IT0_ENTRY5    	   0x00000C00
#define INDUS_RSS_IT0_ENTRY4    	   0x00000300
#define INDUS_RSS_IT0_ENTRY3    	   0x000000C0
#define INDUS_RSS_IT0_ENTRY2    	   0x00000030
#define INDUS_RSS_IT0_ENTRY1    	   0x0000000C
#define INDUS_RSS_IT0_ENTRY0    	   0x00000003

#define INDUS_RSS_IT1_OFFSET  0x45c
#define INDUS_RSS_IT1_DEFAULT_VAL  0x00000000
#define INDUS_RSS_IT1_VALID_MASK  0xFFFFFFFF
#define INDUS_RSS_IT1_WRITE_MASK  0xFFFFFFFF
#define INDUS_RSS_IT1_READ_MASK  0xFFFFFFFF
#define INDUS_RSS_IT1_ENTRY31    	   0xC0000000
#define INDUS_RSS_IT1_ENTRY30    	   0x30000000
#define INDUS_RSS_IT1_ENTRY29    	   0x0C000000
#define INDUS_RSS_IT1_ENTRY28    	   0x03000000
#define INDUS_RSS_IT1_ENTRY27    	   0x00C00000
#define INDUS_RSS_IT1_ENTRY26    	   0x00300000
#define INDUS_RSS_IT1_ENTRY25    	   0x000C0000
#define INDUS_RSS_IT1_ENTRY24    	   0x00030000
#define INDUS_RSS_IT1_ENTRY23    	   0x0000C000
#define INDUS_RSS_IT1_ENTRY22    	   0x00003000
#define INDUS_RSS_IT1_ENTRY21    	   0x00000C00
#define INDUS_RSS_IT1_ENTRY20    	   0x00000300
#define INDUS_RSS_IT1_ENTRY19    	   0x000000C0
#define INDUS_RSS_IT1_ENTRY18    	   0x00000030
#define INDUS_RSS_IT1_ENTRY17    	   0x0000000C
#define INDUS_RSS_IT1_ENTRY16    	   0x00000003

#define INDUS_RSS_IT2_OFFSET  0x460
#define INDUS_RSS_IT2_DEFAULT_VAL  0x00000000
#define INDUS_RSS_IT2_VALID_MASK  0xFFFFFFFF
#define INDUS_RSS_IT2_WRITE_MASK  0xFFFFFFFF
#define INDUS_RSS_IT2_READ_MASK  0xFFFFFFFF
#define INDUS_RSS_IT2_ENTRY47    	   0xC0000000
#define INDUS_RSS_IT2_ENTRY46    	   0x30000000
#define INDUS_RSS_IT2_ENTRY45    	   0x0C000000
#define INDUS_RSS_IT2_ENTRY44    	   0x03000000
#define INDUS_RSS_IT2_ENTRY43    	   0x00C00000
#define INDUS_RSS_IT2_ENTRY42    	   0x00300000
#define INDUS_RSS_IT2_ENTRY41    	   0x000C0000
#define INDUS_RSS_IT2_ENTRY40    	   0x00030000
#define INDUS_RSS_IT2_ENTRY39    	   0x0000C000
#define INDUS_RSS_IT2_ENTRY38    	   0x00003000
#define INDUS_RSS_IT2_ENTRY37    	   0x00000C00
#define INDUS_RSS_IT2_ENTRY36    	   0x00000300
#define INDUS_RSS_IT2_ENTRY35    	   0x000000C0
#define INDUS_RSS_IT2_ENTRY34    	   0x00000030
#define INDUS_RSS_IT2_ENTRY33    	   0x0000000C
#define INDUS_RSS_IT2_ENTRY32    	   0x00000003

#define INDUS_RSS_IT3_OFFSET  0x464
#define INDUS_RSS_IT3_DEFAULT_VAL  0x00000000
#define INDUS_RSS_IT3_VALID_MASK  0xFFFFFFFF
#define INDUS_RSS_IT3_WRITE_MASK  0xFFFFFFFF
#define INDUS_RSS_IT3_READ_MASK  0xFFFFFFFF
#define INDUS_RSS_IT3_ENTRY63    	   0xC0000000
#define INDUS_RSS_IT3_ENTRY62    	   0x30000000
#define INDUS_RSS_IT3_ENTRY61    	   0x0C000000
#define INDUS_RSS_IT3_ENTRY60    	   0x03000000
#define INDUS_RSS_IT3_ENTRY59    	   0x00C00000
#define INDUS_RSS_IT3_ENTRY58    	   0x00300000
#define INDUS_RSS_IT3_ENTRY57    	   0x000C0000
#define INDUS_RSS_IT3_ENTRY56    	   0x00030000
#define INDUS_RSS_IT3_ENTRY55    	   0x0000C000
#define INDUS_RSS_IT3_ENTRY54    	   0x00003000
#define INDUS_RSS_IT3_ENTRY53    	   0x00000C00
#define INDUS_RSS_IT3_ENTRY52    	   0x00000300
#define INDUS_RSS_IT3_ENTRY51    	   0x000000C0
#define INDUS_RSS_IT3_ENTRY50    	   0x00000030
#define INDUS_RSS_IT3_ENTRY49    	   0x0000000C
#define INDUS_RSS_IT3_ENTRY48    	   0x00000003

#define INDUS_RSS_IT4_OFFSET  0x468
#define INDUS_RSS_IT4_DEFAULT_VAL  0x00000000
#define INDUS_RSS_IT4_VALID_MASK  0xFFFFFFFF
#define INDUS_RSS_IT4_WRITE_MASK  0xFFFFFFFF
#define INDUS_RSS_IT4_READ_MASK  0xFFFFFFFF
#define INDUS_RSS_IT4_ENTRY79    	   0xC0000000
#define INDUS_RSS_IT4_ENTRY78    	   0x30000000
#define INDUS_RSS_IT4_ENTRY77    	   0x0C000000
#define INDUS_RSS_IT4_ENTRY76    	   0x03000000
#define INDUS_RSS_IT4_ENTRY75    	   0x00C00000
#define INDUS_RSS_IT4_ENTRY74    	   0x00300000
#define INDUS_RSS_IT4_ENTRY73    	   0x000C0000
#define INDUS_RSS_IT4_ENTRY72    	   0x00030000
#define INDUS_RSS_IT4_ENTRY71    	   0x0000C000
#define INDUS_RSS_IT4_ENTRY70    	   0x00003000
#define INDUS_RSS_IT4_ENTRY69    	   0x00000C00
#define INDUS_RSS_IT4_ENTRY68    	   0x00000300
#define INDUS_RSS_IT4_ENTRY67    	   0x000000C0
#define INDUS_RSS_IT4_ENTRY66    	   0x00000030
#define INDUS_RSS_IT4_ENTRY65    	   0x0000000C
#define INDUS_RSS_IT4_ENTRY64    	   0x00000003

#define INDUS_RSS_IT5_OFFSET  0x46c
#define INDUS_RSS_IT5_DEFAULT_VAL  0x00000000
#define INDUS_RSS_IT5_VALID_MASK  0xFFFFFFFF
#define INDUS_RSS_IT5_WRITE_MASK  0xFFFFFFFF
#define INDUS_RSS_IT5_READ_MASK  0xFFFFFFFF
#define INDUS_RSS_IT5_ENTRY95    	   0xC0000000
#define INDUS_RSS_IT5_ENTRY94    	   0x30000000
#define INDUS_RSS_IT5_ENTRY93    	   0x0C000000
#define INDUS_RSS_IT5_ENTRY92    	   0x03000000
#define INDUS_RSS_IT5_ENTRY91    	   0x00C00000
#define INDUS_RSS_IT5_ENTRY90    	   0x00300000
#define INDUS_RSS_IT5_ENTRY89    	   0x000C0000
#define INDUS_RSS_IT5_ENTRY88    	   0x00030000
#define INDUS_RSS_IT5_ENTRY87    	   0x0000C000
#define INDUS_RSS_IT5_ENTRY86    	   0x00003000
#define INDUS_RSS_IT5_ENTRY85    	   0x00000C00
#define INDUS_RSS_IT5_ENTRY84    	   0x00000300
#define INDUS_RSS_IT5_ENTRY83    	   0x000000C0
#define INDUS_RSS_IT5_ENTRY82    	   0x00000030
#define INDUS_RSS_IT5_ENTRY81    	   0x0000000C
#define INDUS_RSS_IT5_ENTRY80    	   0x00000003

#define INDUS_RSS_IT6_OFFSET  0x470
#define INDUS_RSS_IT6_DEFAULT_VAL  0x00000000
#define INDUS_RSS_IT6_VALID_MASK  0xFFFFFFFF
#define INDUS_RSS_IT6_WRITE_MASK  0xFFFFFFFF
#define INDUS_RSS_IT6_READ_MASK  0xFFFFFFFF
#define INDUS_RSS_IT6_ENTRY111    	   0xC0000000
#define INDUS_RSS_IT6_ENTRY110    	   0x30000000
#define INDUS_RSS_IT6_ENTRY109    	   0x0C000000
#define INDUS_RSS_IT6_ENTRY108    	   0x03000000
#define INDUS_RSS_IT6_ENTRY107    	   0x00C00000
#define INDUS_RSS_IT6_ENTRY106    	   0x00300000
#define INDUS_RSS_IT6_ENTRY105    	   0x000C0000
#define INDUS_RSS_IT6_ENTRY104    	   0x00030000
#define INDUS_RSS_IT6_ENTRY103    	   0x0000C000
#define INDUS_RSS_IT6_ENTRY102    	   0x00003000
#define INDUS_RSS_IT6_ENTRY101    	   0x00000C00
#define INDUS_RSS_IT6_ENTRY100    	   0x00000300
#define INDUS_RSS_IT6_ENTRY99    	   0x000000C0
#define INDUS_RSS_IT6_ENTRY98    	   0x00000030
#define INDUS_RSS_IT6_ENTRY97    	   0x0000000C
#define INDUS_RSS_IT6_ENTRY96    	   0x00000003

#define INDUS_RSS_IT7_OFFSET  0x474
#define INDUS_RSS_IT7_DEFAULT_VAL  0x00000000
#define INDUS_RSS_IT7_VALID_MASK  0xFFFFFFFF
#define INDUS_RSS_IT7_WRITE_MASK  0xFFFFFFFF
#define INDUS_RSS_IT7_READ_MASK  0xFFFFFFFF
#define INDUS_RSS_IT7_ENTRY127    	   0xC0000000
#define INDUS_RSS_IT7_ENTRY126    	   0x30000000
#define INDUS_RSS_IT7_ENTRY125    	   0x0C000000
#define INDUS_RSS_IT7_ENTRY124    	   0x03000000
#define INDUS_RSS_IT7_ENTRY123    	   0x00C00000
#define INDUS_RSS_IT7_ENTRY122    	   0x00300000
#define INDUS_RSS_IT7_ENTRY121    	   0x000C0000
#define INDUS_RSS_IT7_ENTRY120    	   0x00030000
#define INDUS_RSS_IT7_ENTRY119    	   0x0000C000
#define INDUS_RSS_IT7_ENTRY118    	   0x00003000
#define INDUS_RSS_IT7_ENTRY117    	   0x00000C00
#define INDUS_RSS_IT7_ENTRY116    	   0x00000300
#define INDUS_RSS_IT7_ENTRY115    	   0x000000C0
#define INDUS_RSS_IT7_ENTRY114    	   0x00000030
#define INDUS_RSS_IT7_ENTRY113    	   0x0000000C
#define INDUS_RSS_IT7_ENTRY112    	   0x00000003

#define INDUS_RSS_WR_COMP_OFFSET  0x478
#define INDUS_RSS_WR_COMP_DEFAULT_VAL  0x00000000
#define INDUS_RSS_WR_COMP_VALID_MASK  0x00000001
#define INDUS_RSS_WR_COMP_WRITE_MASK  0x00000001
#define INDUS_RSS_WR_COMP_READ_MASK  0x00000000
#define INDUS_RSS_WR_COMP_RESERVED_31_1_MASK    	   0xFFFFFFFE

#define INDUS_MAX_HDR_SIZE_OFFSET  0x47c
#define INDUS_MAX_HDR_SIZE_DEFAULT_VAL  0x00000000
#define INDUS_MAX_HDR_SIZE_VALID_MASK  0x0001FFFF
#define INDUS_MAX_HDR_SIZE_WRITE_MASK  0x0001FFFF
#define INDUS_MAX_HDR_SIZE_READ_MASK  0x0001FFFF
#define INDUS_MAX_HDR_SIZE_RESERVED_31_17_MASK    	   0xFFFE0000
#define INDUS_MAX_HDR_SIZE_HDBFR_FL_INT    	   0x00010000
#define INDUS_MAX_HDR_SIZE_HDRSP_OVERRD    	   0x00008000
#define INDUS_MAX_HDR_SIZE_HSEN    	   0x00004000
#define INDUS_MAX_HDR_SIZE_MHS    	   0x00003FFF

#define INDUS_LNG_PKT_OFFSET  0x480
#define INDUS_LNG_PKT_DEFAULT_VAL  0x00002600
#define INDUS_LNG_PKT_VALID_MASK  0x00007FFF
#define INDUS_LNG_PKT_WRITE_MASK  0x00007FFF
#define INDUS_LNG_PKT_READ_MASK  0x00007FFF
#define INDUS_LNG_PKT_RESERVED_31_15_MASK    	   0xFFFF8000
#define INDUS_LNG_PKT_EN    	   0x00004000
#define INDUS_LNG_PKT_LEN    	   0x00003FFF

#define INDUS_RIS1_OFFSET  0x484
#define INDUS_RIS1_DEFAULT_VAL  0x00000000
#define INDUS_RIS1_VALID_MASK  0x000000FF
#define INDUS_RIS1_WRITE_MASK  0x00000000
#define INDUS_RIS1_READ_MASK  0x000000FF
#define INDUS_RIS1_RESERVED_31_8_MASK    	   0xFFFFFF00
#define INDUS_RIS1_DMA_RX_ERROR    	   0x00000080
#define INDUS_RIS1_RX_QUEUE    	   0x00000078
#define INDUS_RIS1_MII    	   0x00000004
#define INDUS_RIS1_LSCD    	   0x00000002
#define INDUS_RIS1_TX_Q_CMPLT    	   0x00000001

#define INDUS_IE1_OFFSET  0x488
#define INDUS_IE1_DEFAULT_VAL  0x00000000
#define INDUS_IE1_VALID_MASK  0x000000FF
#define INDUS_IE1_WRITE_MASK  0x000000FF
#define INDUS_IE1_READ_MASK  0x000000FF
#define INDUS_IE1_RESERVED_31_8_MASK    	   0xFFFFFF00
#define INDUS_IE1_DMA_RX_ERROR    	   0x00000080
#define INDUS_IE1_RX_QUEUE    	   0x00000078
#define INDUS_IE1_MII    	   0x00000004
#define INDUS_IE1_LSCD    	   0x00000002
#define INDUS_IE1_TX_Q_CMPLT    	   0x00000001

#define INDUS_INT_STS1_OFFSET  0x48c
#define INDUS_INT_STS1_DEFAULT_VAL  0x00000000
#define INDUS_INT_STS1_VALID_MASK  0x000000FF
#define INDUS_INT_STS1_WRITE_MASK  0x00000000
#define INDUS_INT_STS1_READ_MASK  0x000000FF
#define INDUS_INT_STS1_RESERVED_31_8_MASK    	   0xFFFFFF00
#define INDUS_INT_STS1_DMA_RX_ERROR    	   0x00000080
#define INDUS_INT_STS1_RX_QUEUE    	   0x00000078
#define INDUS_INT_STS1_MII    	   0x00000004
#define INDUS_INT_STS1_LSCD    	   0x00000002
#define INDUS_INT_STS1_TX_Q_CMPLT    	   0x00000001

#define INDUS_INT_CLR1_OFFSET  0x490
#define INDUS_INT_CLR1_DEFAULT_VAL  0x00000000
#define INDUS_INT_CLR1_VALID_MASK  0x000000FF
#define INDUS_INT_CLR1_WRITE_MASK  0x000000FF
#define INDUS_INT_CLR1_READ_MASK  0x00000000
#define INDUS_INT_CLR1_RESERVED_31_8_MASK    	   0xFFFFFF00
#define INDUS_INT_CLR1_DMA_RX_ERROR    	   0x00000080
#define INDUS_INT_CLR1_RX_QUEUE    	   0x00000078
#define INDUS_INT_CLR1_MII    	   0x00000004
#define INDUS_INT_CLR1_LSCD    	   0x00000002
#define INDUS_INT_CLR1_TX_Q_CMPLT    	   0x00000001

#define INDUS_MOD_CLK_CNT_OFFSET  0x494
#define INDUS_MOD_CLK_CNT_DEFAULT_VAL  0x00000000
#define INDUS_MOD_CLK_CNT_VALID_MASK  0xFFFFFFFF
#define INDUS_MOD_CLK_CNT_WRITE_MASK  0xFFFFFFFF
#define INDUS_MOD_CLK_CNT_READ_MASK  0xFFFFFFFF

#define INDUS_INT_THROTTLE_OFFSET  0x498
#define INDUS_INT_THROTTLE_DEFAULT_VAL  0x00000000
#define INDUS_INT_THROTTLE_VALID_MASK  0x000007FF
#define INDUS_INT_THROTTLE_WRITE_MASK  0x000007FF
#define INDUS_INT_THROTTLE_READ_MASK  0x000007FF
#define INDUS_INT_THROTTLE_RESERVED_31_11_MASK    	   0xFFFFF800
#define INDUS_INT_THROTTLE_MOD_CNTR    	   0x000007FE
#define INDUS_INT_THROTTLE_THROTTLE_EN    	   0x00000001

#define INDUS_VECT_ALLOC0_OFFSET  0x49c
#define INDUS_VECT_ALLOC0_DEFAULT_VAL  0x00000000
#define INDUS_VECT_ALLOC0_VALID_MASK  0x00FFFFFF
#define INDUS_VECT_ALLOC0_WRITE_MASK  0x00FFFFFF
#define INDUS_VECT_ALLOC0_READ_MASK  0x00FFFFFF
#define INDUS_VECT_ALLOC0_RESERVED_31_24_MASK    	   0xFF000000
#define INDUS_VECT_ALLOC0_INT_ALLOC5    	   0x00E00000
#define INDUS_VECT_ALLOC0_INT_ALLOC_EN5    	   0x00100000
#define INDUS_VECT_ALLOC0_INT_ALLOC4    	   0x000E0000
#define INDUS_VECT_ALLOC0_INT_ALLOC_EN4    	   0x00010000
#define INDUS_VECT_ALLOC0_INT_ALLOC3    	   0x0000E000
#define INDUS_VECT_ALLOC0_INT_ALLOC_EN3    	   0x00001000
#define INDUS_VECT_ALLOC0_INT_ALLOC2    	   0x00000E00
#define INDUS_VECT_ALLOC0_INT_ALLOC_EN2    	   0x00000100
#define INDUS_VECT_ALLOC0_INT_ALLOC1    	   0x000000E0
#define INDUS_VECT_ALLOC0_INT_ALLOC_EN1    	   0x00000010
#define INDUS_VECT_ALLOC0_INT_ALLOC0    	   0x0000000E
#define INDUS_VECT_ALLOC0_INT_ALLOC_EN0    	   0x00000001

#define INDUS_SRAM_LOW_THLD_OFFSET  0x4a0
#define INDUS_SRAM_LOW_THLD_DEFAULT_VAL  0x000003ff
#define INDUS_SRAM_LOW_THLD_VALID_MASK  0x000003FF
#define INDUS_SRAM_LOW_THLD_WRITE_MASK  0x000003FF
#define INDUS_SRAM_LOW_THLD_READ_MASK  0x000003FF
#define INDUS_SRAM_LOW_THLD_RESERVED_31_10_MASK    	   0xFFFFFC00

#define INDUS_PAUSE_PKT_THRESH_OFFSET  0x4a4
#define INDUS_PAUSE_PKT_THRESH_DEFAULT_VAL  0x00000010
#define INDUS_PAUSE_PKT_THRESH_VALID_MASK  0x000003FF
#define INDUS_PAUSE_PKT_THRESH_WRITE_MASK  0x000003FF
#define INDUS_PAUSE_PKT_THRESH_READ_MASK  0x000003FF
#define INDUS_PAUSE_PKT_THRESH_RESERVED_31_10_MASK    	   0xFFFFFC00
