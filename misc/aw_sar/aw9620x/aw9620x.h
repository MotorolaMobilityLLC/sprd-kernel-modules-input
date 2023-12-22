/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef AW9620X_H_
#define AW9620X_H_
#include "../comm/aw_sar_type.h"

#define AW9620X_CHANNEL_NUM_MAX				(8)
#define AW9620X_FINE_ADJUST_STEP0			(165)
#define AW9620X_COARSE_ADJUST_STEP0			(9900)
#define AW9620X_FINE_ADJUST_STEP1			(330)
#define AW9620X_COARSE_ADJUST_STEP1			(19800)
#define AW9620X_CFG_REG_STEP				(0x8c)
#define AW9620X_PARA_TIMES				(10000)
#define AW9620X_PROT_STOP_WAIT_IRQ_CYCLES		(100)
#define AW9620X_REG_UPDATA_DIS_MASK			(0x00ffffff)
#define AW9620X_AFECFG3_CVMULTUALMOD			(11)
#define AW9620X_AFECFG3_CVOFF2X				(10)
#define AW9620X_REMOVE_FLOAT_COEF			(1024)
#define AW9620X_REG_MCFG_DELAY_MS			(20)

#define AW9620X_REG_RSTNALL_VAL				(0x3c)
#define AW9620X_POWER_ON_DELAY_MS			(25)
#define AW9620X_REG_STEP				(0x8c)
#define AW9620X_REMOVE_FLOAT_COEF			(1024)

#define AW9620X_SAR_VCC_MIN_UV			(1700000)
#define AW9620X_SAR_VCC_MAX_UV			(3600000)
#define AW9320X_SRAM_ERROR_CODE			(0x1c00)

struct aw9620x {
	uint32_t last_blfilta[AW9620X_CHANNEL_NUM_MAX];
	uint32_t last_irq_en;
};


enum aw_sar_irq_trigger_position {
	AW9620X_TRIGGER_FAR,
	AW9620X_TRIGGER_TH0,
	AW9620X_TRIGGER_TH1 = 0x03,
};

enum AW9620X_CHIP_ID_VALUE {
	AW96203CSR_CHIP_ID = 0x20226203,
	AW96205DNR_CHIP_ID = 0x20226205,
	AW96208CSR_CHIP_ID = 0x20226208,
};

enum AW9620X_OPERAION_MODE {
	AW9620X_ACTIVE_MODE = 1,
	AW9620X_SLEEP_MODE,
	AW9620X_DEEPSLEEP_MODE,
};

/********************************************
 * Register List
 ********************************************/
#define AFE_BASE_ADDR					(0x1A00)
#define DSP_BASE_ADDR					(0x1A00)
#define STAT_BASE_ADDR					(0x1A00)
#define DATA_BASE_ADDR					(0x1A00)
#define SFR_BASE_ADDR					(0x0000)
/* registers list */
//96208_0917_AFE MAP
#define REG_SCANCTRL0					((0x0000) + AFE_BASE_ADDR)
#define REG_SCANCTRL1					((0x0004) + AFE_BASE_ADDR)
#define REG_AFECFG0_CH0					((0x0040) + AFE_BASE_ADDR)
#define REG_AFECFG1_CH0					((0x0044) + AFE_BASE_ADDR)
#define REG_AFECFG3_CH0					((0x004C) + AFE_BASE_ADDR)
#define REG_AFECFG0_CH1					((0x00CC) + AFE_BASE_ADDR)
#define REG_AFECFG1_CH1					((0x00D0) + AFE_BASE_ADDR)
#define REG_AFECFG3_CH1					((0x00D8) + AFE_BASE_ADDR)
#define REG_AFECFG0_CH2					((0x0158) + AFE_BASE_ADDR)
#define REG_AFECFG1_CH2					((0x015C) + AFE_BASE_ADDR)
#define REG_AFECFG3_CH2					((0x0164) + AFE_BASE_ADDR)
#define REG_AFECFG0_CH3					((0x01E4) + AFE_BASE_ADDR)
#define REG_AFECFG1_CH3					((0x01E8) + AFE_BASE_ADDR)
#define REG_AFECFG3_CH3					((0x01F0) + AFE_BASE_ADDR)
#define REG_AFECFG0_CH4					((0x0270) + AFE_BASE_ADDR)
#define REG_AFECFG1_CH4					((0x0274) + AFE_BASE_ADDR)
#define REG_AFECFG3_CH4					((0x027C) + AFE_BASE_ADDR)
#define REG_AFECFG0_CH5					((0x02FC) + AFE_BASE_ADDR)
#define REG_AFECFG1_CH5					((0x0300) + AFE_BASE_ADDR)
#define REG_AFECFG3_CH5					((0x0308) + AFE_BASE_ADDR)
#define REG_AFECFG0_CH6					((0x0388) + AFE_BASE_ADDR)
#define REG_AFECFG1_CH6					((0x038C) + AFE_BASE_ADDR)
#define REG_AFECFG3_CH6					((0x0394) + AFE_BASE_ADDR)
#define REG_AFECFG0_CH7					((0x0414) + AFE_BASE_ADDR)
#define REG_AFECFG1_CH7					((0x0418) + AFE_BASE_ADDR)
#define REG_AFECFG3_CH7					((0x0420) + AFE_BASE_ADDR)

/* registers list */
//96208_0917_DSP MAP
#define REG_FILTCTRL0_CH0				((0x0050) + DSP_BASE_ADDR)
#define REG_BLFILT_CH0					((0x0078) + DSP_BASE_ADDR)
#define REG_BLERRAOT_CH0				((0x008C) + DSP_BASE_ADDR)
#define REG_PROXCTRL_CH0				((0x0090) + DSP_BASE_ADDR)
#define REG_PROXTH0_CH0					((0x0094) + DSP_BASE_ADDR)
#define REG_PROXTH1_CH0					((0x0098) + DSP_BASE_ADDR)
#define REG_AOTTAR_CH0					((0x009C) + DSP_BASE_ADDR)
#define REG_FILTCTRL0_CH1				((0x00DC) + DSP_BASE_ADDR)
#define REG_BLFILT_CH1					((0x0104) + DSP_BASE_ADDR)
#define REG_BLERRAOT_CH1				((0x0118) + DSP_BASE_ADDR)
#define REG_PROXCTRL_CH1				((0x011C) + DSP_BASE_ADDR)
#define REG_PROXTH0_CH1					((0x0120) + DSP_BASE_ADDR)
#define REG_PROXTH1_CH1					((0x0124) + DSP_BASE_ADDR)
#define REG_AOTTAR_CH1					((0x0128) + DSP_BASE_ADDR)
#define REG_FILTCTRL0_CH2				((0x0168) + DSP_BASE_ADDR)
#define REG_BLFILT_CH2					((0x0190) + DSP_BASE_ADDR)
#define REG_BLERRAOT_CH2				((0x01A4) + DSP_BASE_ADDR)
#define REG_PROXCTRL_CH2				((0x01A8) + DSP_BASE_ADDR)
#define REG_PROXTH0_CH2					((0x01AC) + DSP_BASE_ADDR)
#define REG_PROXTH1_CH2					((0x01B0) + DSP_BASE_ADDR)
#define REG_AOTTAR_CH2					((0x01B4) + DSP_BASE_ADDR)
#define REG_FILTCTRL0_CH3				((0x01F4) + DSP_BASE_ADDR)
#define REG_BLFILT_CH3					((0x021C) + DSP_BASE_ADDR)
#define REG_BLERRAOT_CH3				((0x0230) + DSP_BASE_ADDR)
#define REG_PROXCTRL_CH3				((0x0234) + DSP_BASE_ADDR)
#define REG_PROXTH0_CH3					((0x0238) + DSP_BASE_ADDR)
#define REG_PROXTH1_CH3					((0x023C) + DSP_BASE_ADDR)
#define REG_AOTTAR_CH3					((0x0240) + DSP_BASE_ADDR)
#define REG_FILTCTRL0_CH4				((0x0280) + DSP_BASE_ADDR)
#define REG_BLFILT_CH4					((0x02A8) + DSP_BASE_ADDR)
#define REG_BLERRAOT_CH4				((0x02BC) + DSP_BASE_ADDR)
#define REG_PROXCTRL_CH4				((0x02C0) + DSP_BASE_ADDR)
#define REG_PROXTH0_CH4					((0x02C4) + DSP_BASE_ADDR)
#define REG_PROXTH1_CH4					((0x02C8) + DSP_BASE_ADDR)
#define REG_AOTTAR_CH4					((0x02CC) + DSP_BASE_ADDR)
#define REG_FILTCTRL0_CH5				((0x030C) + DSP_BASE_ADDR)
#define REG_BLFILT_CH5					((0x0334) + DSP_BASE_ADDR)
#define REG_BLERRAOT_CH5				((0x0348) + DSP_BASE_ADDR)
#define REG_PROXCTRL_CH5				((0x034C) + DSP_BASE_ADDR)
#define REG_PROXTH0_CH5					((0x0350) + DSP_BASE_ADDR)
#define REG_PROXTH1_CH5					((0x0354) + DSP_BASE_ADDR)
#define REG_AOTTAR_CH5					((0x0358) + DSP_BASE_ADDR)
#define REG_FILTCTRL0_CH6				((0x0398) + DSP_BASE_ADDR)
#define REG_BLFILT_CH6					((0x03C0) + DSP_BASE_ADDR)
#define REG_BLERRAOT_CH6				((0x03D4) + DSP_BASE_ADDR)
#define REG_PROXCTRL_CH6				((0x03D8) + DSP_BASE_ADDR)
#define REG_PROXTH0_CH6					((0x03DC) + DSP_BASE_ADDR)
#define REG_PROXTH1_CH6					((0x03E0) + DSP_BASE_ADDR)
#define REG_AOTTAR_CH6					((0x03E4) + DSP_BASE_ADDR)
#define REG_FILTCTRL0_CH7				((0x0424) + DSP_BASE_ADDR)
#define REG_BLFILT_CH7					((0x044C) + DSP_BASE_ADDR)
#define REG_BLERRAOT_CH7				((0x0460) + DSP_BASE_ADDR)
#define REG_PROXCTRL_CH7				((0x0464) + DSP_BASE_ADDR)
#define REG_PROXTH0_CH7					((0x0468) + DSP_BASE_ADDR)
#define REG_PROXTH1_CH7					((0x046C) + DSP_BASE_ADDR)
#define REG_AOTTAR_CH7					((0x0470) + DSP_BASE_ADDR)

/* registers list */
//96208_0917_STAT MAP
#define REG_FWVER					((0x000C) + STAT_BASE_ADDR)
#define REG_WST						((0x0010) + STAT_BASE_ADDR)
#define REG_STAT0					((0x0014) + STAT_BASE_ADDR)
#define REG_STAT1					((0x0018) + STAT_BASE_ADDR)

/* registers list */
//96208_0917_DATA MAP
#define REG_TEMPERATURE					((0x001C) + DATA_BASE_ADDR)
#define REG_BASELINE_CH0				((0x00AC) + DATA_BASE_ADDR)
#define REG_DIFF_CH0					((0x00B0) + DATA_BASE_ADDR)
#define REG_BASELINE_CH1				((0x0138) + DATA_BASE_ADDR)
#define REG_DIFF_CH1					((0x013C) + DATA_BASE_ADDR)
#define REG_BASELINE_CH2				((0x01C4) + DATA_BASE_ADDR)
#define REG_DIFF_CH2					((0x01C8) + DATA_BASE_ADDR)
#define REG_BASELINE_CH3				((0x0250) + DATA_BASE_ADDR)
#define REG_DIFF_CH3					((0x0254) + DATA_BASE_ADDR)
#define REG_BASELINE_CH4				((0x02DC) + DATA_BASE_ADDR)
#define REG_DIFF_CH4					((0x02E0) + DATA_BASE_ADDR)
#define REG_BASELINE_CH5				((0x0368) + DATA_BASE_ADDR)
#define REG_DIFF_CH5					((0x036C) + DATA_BASE_ADDR)
#define REG_BASELINE_CH6				((0x03F4) + DATA_BASE_ADDR)
#define REG_DIFF_CH6					((0x03F8) + DATA_BASE_ADDR)
#define REG_BASELINE_CH7				((0x0480) + DATA_BASE_ADDR)
#define REG_DIFF_CH7					((0x0484) + DATA_BASE_ADDR)

/* registers list */
//96208_0917_SFR MAP
#define REG_IRQSRC					((0x4410) + SFR_BASE_ADDR)
#define REG_IRQEN					((0x4414) + SFR_BASE_ADDR)
#define REG_I2CDEVID					((0x4440) + SFR_BASE_ADDR)
#define REG_CHIP_ID					((0xFF00) + SFR_BASE_ADDR)
#define REG_CHIPSTAT					((0xFF04) + SFR_BASE_ADDR)
#define REG_HOSTCTRL					((0xFF10) + SFR_BASE_ADDR)
#define REG_RSTNALL					((0xFF18) + SFR_BASE_ADDR)

//driver user
#define REG_ACCESSEN					((0xFF20) + SFR_BASE_ADDR)
#define REG_MCFG					((0x4444) + SFR_BASE_ADDR)
#define REG_ISPGO					((0x4714) + SFR_BASE_ADDR)
#define REG_PMU_CFG					((0x4820) + SFR_BASE_ADDR)
#define REG_ARRAY2_EW_EN				((0x4794) + SFR_BASE_ADDR)
#define REG_ISPADR					((0x4704) + SFR_BASE_ADDR)
#define REG_ISPWDATA					((0x4708) + SFR_BASE_ADDR)
#define REG_ISPRDATA					((0x470C) + SFR_BASE_ADDR)
#define REG_ISPCMD					((0x4710) + SFR_BASE_ADDR)
#define REG_T_RCV					((0x472C) + SFR_BASE_ADDR)
#define REG_CMD						((0x4408) + SFR_BASE_ADDR)
#define REG_BOOT_LOADER_ACTIVE				((0x4748) + SFR_BASE_ADDR)

#define REG_OPEN_APB_ACCESS_EN				(0x3c00f091)
#define REG_SET_PMU_CFG					(0x6)
#define REG_SET_BTROM_EW_EN				(0x5a637955)
#define REG_ISP_CMD_CONFIG				(0x0c)
#define REG_SET_ISP_GO					(0x1)
#define REG_ENSET_PMU_CFG				(0x4)
#define REG_ACCESS_MAIN_ARR				(0x05)
#define REG_SET_T_RCV					(0xf0)
#define REG_SET_T_RCV_EN				(0x16)
#define REG_OPEN_REG_ACCESS_EN				(0x3c00ffff)
#define REG_OPEN_MCFG_EN				(0x10000)
#define REG_ENSET_BTROM_EW_EN				(0x0)
#define REG_SET_MCFG00					(0x0)
#define REG_ISP_CMD_MAIN_ARR				(0X03)
#define REG_HOSTCTRL_EN					(0x1)
#define REG_ACCESSEN_OPEN				(0x3c00F091)
#define REG_ACCESSEN_CLOSE				(0x3c00f011)
#define AW_REG_FLASH_WAKE_UP				(0x4700)
#define REG_UPDATA_DIS					(0x4744)
#define AW_REG_FLASH_WAKE_UP_ON				(0x110)
#define AW_REG_FLASH_WAKE_UP_OFF			(0x102)

#define AW_BT_HOST2CPU_TRIG_ONLINE_UPGRADE_CMD		(0x14)

#define AW_BT_STATE_INFO_ERR				(0x1c00)
#define AW_BT_STATE_INFO_PST				(0x1C04)
#define AW_BT_STATE_INFO_FLAG				(0x1C08)
#define AW_BT_PROT_CMD_PACK_ADDR			(0x1C10)
#define AW_BT_PROT_ACK_PACK_ADDR			(0x1C14)
#define AW_BT_MESSAGE_MSG0				(0x1C20)
#define AW_BT_MESSAGE_MSG1				(0x1C24)
#define AW_BT_MESSAGE_MSG_FLAG				(0x1C28)
#define	AW_BT_VER_INF_VERSION				(0x1C30)
#define	AW_BT_VER_INF_DATE				(0x1C34)
#define	AW_BT_HOST2CPU_TRIG				(0x4408)
#define AW_BTCPU2HOST_TRIG				(0x4410)

#define AW_SRAM_FIRST_DETECT				(0x20000800)

struct aw_reg_data {
	unsigned char rw;
	unsigned short reg;
};
/********************************************
 * Register Access
 *******************************************/
#define REG_NONE_ACCESS					(0)
#define REG_RD_ACCESS					(1 << 0)
#define REG_WR_ACCESS					(1 << 1)

static const struct aw_reg_data g_aw9620x_reg_access[] = {
	//AFE MAP
	{ .reg = REG_SCANCTRL0,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_SCANCTRL1,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AFECFG0_CH0,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AFECFG1_CH0,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AFECFG3_CH0,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AFECFG0_CH1,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AFECFG1_CH1,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AFECFG3_CH1,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AFECFG0_CH2,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AFECFG1_CH2,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AFECFG3_CH2,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AFECFG0_CH3,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AFECFG1_CH3,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AFECFG3_CH3,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AFECFG0_CH4,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AFECFG1_CH4,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AFECFG3_CH4,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AFECFG0_CH5,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AFECFG1_CH5,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AFECFG3_CH5,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AFECFG0_CH6,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AFECFG1_CH6,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AFECFG3_CH6,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AFECFG0_CH7,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AFECFG1_CH7,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AFECFG3_CH7,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },

	//DSP MAP
	{ .reg = REG_FILTCTRL0_CH0,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_BLFILT_CH0,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_BLERRAOT_CH0,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_PROXCTRL_CH0,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_PROXTH0_CH0,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_PROXTH1_CH0,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AOTTAR_CH0,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_FILTCTRL0_CH1,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_BLFILT_CH1,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_BLERRAOT_CH1,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_PROXCTRL_CH1,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_PROXTH0_CH1,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_PROXTH1_CH1,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AOTTAR_CH1,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_FILTCTRL0_CH2,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_BLFILT_CH2,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_BLERRAOT_CH2,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_PROXCTRL_CH2,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_PROXTH0_CH2,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_PROXTH1_CH2,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AOTTAR_CH2,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_FILTCTRL0_CH3,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_BLFILT_CH3,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_BLERRAOT_CH3,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_PROXCTRL_CH3,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_PROXTH0_CH3,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_PROXTH1_CH3,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AOTTAR_CH3,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_FILTCTRL0_CH4,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_BLFILT_CH4,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_BLERRAOT_CH4,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_PROXCTRL_CH4,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_PROXTH0_CH4,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_PROXTH1_CH4,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AOTTAR_CH4,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_FILTCTRL0_CH5,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_BLFILT_CH5,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_BLERRAOT_CH5,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_PROXCTRL_CH5,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_PROXTH0_CH5,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_PROXTH1_CH5,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AOTTAR_CH5,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_FILTCTRL0_CH6,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_BLFILT_CH6,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_BLERRAOT_CH6,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_PROXCTRL_CH6,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_PROXTH0_CH6,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_PROXTH1_CH6,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AOTTAR_CH6,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_FILTCTRL0_CH7,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_BLFILT_CH7,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_BLERRAOT_CH7,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_PROXCTRL_CH7,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_PROXTH0_CH7,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_PROXTH1_CH7,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AOTTAR_CH7,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },

	//STAT MAP
	{ .reg = REG_FWVER,			.rw = REG_RD_ACCESS, },
	{ .reg = REG_WST,			.rw = REG_RD_ACCESS, },
	{ .reg = REG_STAT0,			.rw = REG_RD_ACCESS, },
	{ .reg = REG_STAT1,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },

	//DATA MAP
	{ .reg = REG_TEMPERATURE,		.rw = REG_RD_ACCESS, },
	{ .reg = REG_BASELINE_CH0,		.rw = REG_RD_ACCESS, },
	{ .reg = REG_DIFF_CH0,			.rw = REG_RD_ACCESS, },
	{ .reg = REG_BASELINE_CH1,		.rw = REG_RD_ACCESS, },
	{ .reg = REG_DIFF_CH1,			.rw = REG_RD_ACCESS, },
	{ .reg = REG_BASELINE_CH2,		.rw = REG_RD_ACCESS, },
	{ .reg = REG_DIFF_CH2,			.rw = REG_RD_ACCESS, },
	{ .reg = REG_BASELINE_CH3,		.rw = REG_RD_ACCESS, },
	{ .reg = REG_DIFF_CH3,			.rw = REG_RD_ACCESS, },
	{ .reg = REG_BASELINE_CH4,		.rw = REG_RD_ACCESS, },
	{ .reg = REG_DIFF_CH4,			.rw = REG_RD_ACCESS, },
	{ .reg = REG_BASELINE_CH5,		.rw = REG_RD_ACCESS, },
	{ .reg = REG_DIFF_CH5,			.rw = REG_RD_ACCESS, },
	{ .reg = REG_BASELINE_CH6,		.rw = REG_RD_ACCESS, },
	{ .reg = REG_DIFF_CH6,			.rw = REG_RD_ACCESS, },
	{ .reg = REG_BASELINE_CH7,		.rw = REG_RD_ACCESS, },
	{ .reg = REG_DIFF_CH7,			.rw = REG_RD_ACCESS, },

	//SFR MAP
	{ .reg = REG_IRQSRC,			.rw = REG_RD_ACCESS, },
	{ .reg = REG_IRQEN,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_I2CDEVID,			.rw = REG_RD_ACCESS, },
	{ .reg = REG_CHIP_ID,			.rw = REG_RD_ACCESS, },
	{ .reg = REG_CHIPSTAT,			.rw = REG_RD_ACCESS, },
	{ .reg = REG_HOSTCTRL,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_RSTNALL,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
};

//aw96208 reg config
static const uint32_t aw9620x_reg_default[] = {
	0x1A00, 0x003FFFFF,
	0x1A04, 0x00000032,
	0x1A40, 0x09500001,
	0x1A44, 0x3F000000,
	0x1A4C, 0x00000000,
	0x1ACC, 0x09500004,
	0x1AD0, 0x3F000000,
	0x1AD8, 0x00000000,
	0x1B58, 0x09500010,
	0x1B5C, 0x3F000000,
	0x1B64, 0x00000000,
	0x1BE4, 0x09500040,
	0x1BE8, 0x3F000000,
	0x1BF0, 0x00000000,
	0x1C70, 0x09500100,
	0x1C74, 0x3F000000,
	0x1C7C, 0x00000000,
	0x1CFC, 0x09500400,
	0x1D00, 0x3F000000,
	0x1D08, 0x00000000,
	0x1D88, 0x09501000,
	0x1D8C, 0x3F000000,
	0x1D94, 0x00000000,
	0x1E14, 0x09504000,
	0x1E18, 0x3F000000,
	0x1E20, 0x00000000,
	0x1A50, 0x00000004,
	0x1A78, 0x00640000,
	0x1A8C, 0xFF000001,
	0x1A90, 0x00000000,
	0x1A94, 0x00002710,
	0x1A98, 0x00004E20,
	0x1A9C, 0x00000000,
	0x1ADC, 0x00000004,
	0x1B04, 0x00640000,
	0x1B18, 0xFF000001,
	0x1B1C, 0x00000000,
	0x1B20, 0x00002710,
	0x1B24, 0x00004E20,
	0x1B28, 0x00000000,
	0x1B68, 0x00000004,
	0x1B90, 0x00640000,
	0x1BA4, 0xFF000001,
	0x1BA8, 0x00000000,
	0x1BAC, 0x00002710,
	0x1BB0, 0x00004E20,
	0x1BB4, 0x00000000,
	0x1BF4, 0x00000004,
	0x1C1C, 0x00640000,
	0x1C30, 0xFF000001,
	0x1C34, 0x00000000,
	0x1C38, 0x00002710,
	0x1C3C, 0x00004E20,
	0x1C40, 0x00000000,
	0x1C80, 0x00000004,
	0x1CA8, 0x00640000,
	0x1CBC, 0xFF000001,
	0x1CC0, 0x00000000,
	0x1CC4, 0x00002710,
	0x1CC8, 0x00004E20,
	0x1CCC, 0x00000000,
	0x1D0C, 0x00000004,
	0x1D34, 0x00640000,
	0x1D48, 0xFF000001,
	0x1D4C, 0x00000000,
	0x1D50, 0x00002710,
	0x1D54, 0x00004E20,
	0x1D58, 0x00000000,
	0x1D98, 0x00000004,
	0x1DC0, 0x00640000,
	0x1DD4, 0xFF000001,
	0x1DD8, 0x00000000,
	0x1DDC, 0x00002710,
	0x1DE0, 0x00004E20,
	0x1DE4, 0x00000000,
	0x1E24, 0x00000004,
	0x1E4C, 0x00640000,
	0x1E60, 0xFF000001,
	0x1E64, 0x00000000,
	0x1E68, 0x00002710,
	0x1E6C, 0x00004E20,
	0x1E70, 0x00000000,
	0x4414, 0x0000081F,
	0x1A18, 0x00FFFFFF,
};

#endif
