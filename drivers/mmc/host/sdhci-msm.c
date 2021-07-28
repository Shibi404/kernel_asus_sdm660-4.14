/*
 * drivers/mmc/host/sdhci-msm.c - Qualcomm Technologies, Inc. MSM SDHCI Platform
 * driver source file
 *
 * Copyright (c) 2012-2020, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/sdio_func.h>
#include <linux/gfp.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/wait.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/mmc/slot-gpio.h>
#include <linux/dma-mapping.h>
#include <linux/iopoll.h>
#include <linux/pinctrl/consumer.h>
#include <linux/iopoll.h>
#include <linux/msm-bus.h>
#include <linux/pm_runtime.h>
#include <trace/events/mmc.h>

#include "sdhci-msm.h"
#include "sdhci-msm-ice.h"
#include "cmdq_hci.h"

#define QOS_REMOVE_DELAY_MS	10
#define CORE_POWER		0x0
#define CORE_SW_RST		(1 << 7)

#define SDHCI_VER_100		0x2B

#define CORE_VERSION_STEP_MASK		0x0000FFFF
#define CORE_VERSION_MINOR_MASK		0x0FFF0000
#define CORE_VERSION_MINOR_SHIFT	16
#define CORE_VERSION_MAJOR_MASK		0xF0000000
#define CORE_VERSION_MAJOR_SHIFT	28
#define CORE_VERSION_TARGET_MASK	0x000000FF
#define SDHCI_MSM_VER_420               0x49

#define SWITCHABLE_SIGNALLING_VOL	(1 << 29)

#define CORE_VERSION_MAJOR_MASK		0xF0000000
#define CORE_VERSION_MAJOR_SHIFT	28

#define CORE_HC_MODE		0x78
#define HC_MODE_EN		0x1
#define FF_CLK_SW_RST_DIS	(1 << 13)

#define CORE_PWRCTL_BUS_OFF	0x01
#define CORE_PWRCTL_BUS_ON	(1 << 1)
#define CORE_PWRCTL_IO_LOW	(1 << 2)
#define CORE_PWRCTL_IO_HIGH	(1 << 3)

#define CORE_PWRCTL_BUS_SUCCESS	0x01
#define CORE_PWRCTL_BUS_FAIL	(1 << 1)
#define CORE_PWRCTL_IO_SUCCESS	(1 << 2)
#define CORE_PWRCTL_IO_FAIL	(1 << 3)

#define INT_MASK		0xF
#define MAX_PHASES		16

#define CORE_CMD_DAT_TRACK_SEL	(1 << 0)
#define CORE_DLL_EN		(1 << 16)
#define CORE_CDR_EN		(1 << 17)
#define CORE_CK_OUT_EN		(1 << 18)
#define CORE_CDR_EXT_EN		(1 << 19)
#define CORE_DLL_PDN		(1 << 29)
#define CORE_DLL_RST		(1 << 30)

#define CORE_DLL_LOCK		(1 << 7)
#define CORE_DDR_DLL_LOCK	(1 << 11)

#define CORE_CLK_PWRSAVE		(1 << 1)
#define CORE_VNDR_SPEC_ADMA_ERR_SIZE_EN	(1 << 7)
#define CORE_HC_MCLK_SEL_DFLT		(2 << 8)
#define CORE_HC_MCLK_SEL_HS400		(3 << 8)
#define CORE_HC_MCLK_SEL_MASK		(3 << 8)
#define CORE_HC_AUTO_CMD21_EN		(1 << 6)
#define CORE_IO_PAD_PWR_SWITCH_EN	(1 << 15)
#define CORE_IO_PAD_PWR_SWITCH	(1 << 16)
#define CORE_HC_SELECT_IN_EN	(1 << 18)
#define CORE_HC_SELECT_IN_HS400	(6 << 19)
#define CORE_HC_SELECT_IN_MASK	(7 << 19)
#define CORE_VENDOR_SPEC_POR_VAL	0xA1C

#define HC_SW_RST_WAIT_IDLE_DIS	(1 << 20)
#define HC_SW_RST_REQ (1 << 21)
#define CORE_ONE_MID_EN     (1 << 25)

#define CORE_8_BIT_SUPPORT		(1 << 18)
#define CORE_3_3V_SUPPORT		(1 << 24)
#define CORE_3_0V_SUPPORT		(1 << 25)
#define CORE_1_8V_SUPPORT		(1 << 26)
#define CORE_SYS_BUS_SUPPORT_64_BIT	BIT(28)

#define CORE_CSR_CDC_CTLR_CFG0		0x130
#define CORE_SW_TRIG_FULL_CALIB		(1 << 16)
#define CORE_HW_AUTOCAL_ENA		(1 << 17)

#define CORE_CSR_CDC_CTLR_CFG1		0x134
#define CORE_CSR_CDC_CAL_TIMER_CFG0	0x138
#define CORE_TIMER_ENA			(1 << 16)

#define CORE_CSR_CDC_CAL_TIMER_CFG1	0x13C
#define CORE_CSR_CDC_REFCOUNT_CFG	0x140
#define CORE_CSR_CDC_COARSE_CAL_CFG	0x144
#define CORE_CDC_OFFSET_CFG		0x14C
#define CORE_CSR_CDC_DELAY_CFG		0x150
#define CORE_CDC_SLAVE_DDA_CFG		0x160
#define CORE_CSR_CDC_STATUS0		0x164
#define CORE_CALIBRATION_DONE		(1 << 0)

#define CORE_CDC_ERROR_CODE_MASK	0x7000000

#define CQ_CMD_DBG_RAM	                0x110
#define CQ_CMD_DBG_RAM_WA               0x150
#define CQ_CMD_DBG_RAM_OL               0x154

#define CORE_CSR_CDC_GEN_CFG		0x178
#define CORE_CDC_SWITCH_BYPASS_OFF	(1 << 0)
#define CORE_CDC_SWITCH_RC_EN		(1 << 1)

#define CORE_CDC_T4_DLY_SEL		(1 << 0)
#define CORE_CMDIN_RCLK_EN		(1 << 1)
#define CORE_START_CDC_TRAFFIC		(1 << 6)

#define CORE_PWRSAVE_DLL	(1 << 3)
#define CORE_FIFO_ALT_EN	(1 << 10)
#define CORE_CMDEN_HS400_INPUT_MASK_CNT (1 << 13)

#define CORE_DDR_CAL_EN		(1 << 0)
#define CORE_FLL_CYCLE_CNT	(1 << 18)
#define CORE_DLL_CLOCK_DISABLE	(1 << 21)

#define DDR_CONFIG_POR_VAL		0x80040873
#define DLL_USR_CTL_POR_VAL		0x10800
#define ENABLE_DLL_LOCK_STATUS		(1 << 26)
#define FINE_TUNE_MODE_EN		(1 << 27)
#define BIAS_OK_SIGNAL			(1 << 29)
#define DLL_CONFIG_3_POR_VAL		0x10

/* 512 descriptors */
#define SDHCI_MSM_MAX_SEGMENTS  (1 << 9)
#define SDHCI_MSM_MMC_CLK_GATE_DELAY	200 /* msecs */

#define CORE_FREQ_100MHZ	(100 * 1000 * 1000)
#define TCXO_FREQ		19200000

#define INVALID_TUNING_PHASE	-1
#define sdhci_is_valid_gpio_wakeup_int(_h) ((_h)->pdata->sdiowakeup_irq >= 0)

#define NUM_TUNING_PHASES		16
#define MAX_DRV_TYPES_SUPPORTED_HS200	4
#define MSM_AUTOSUSPEND_DELAY_MS 100

#define RCLK_TOGGLE 0x2

struct sdhci_msm_offset {
	u32 CORE_MCI_DATA_CNT;
	u32 CORE_MCI_STATUS;
	u32 CORE_MCI_FIFO_CNT;
	u32 CORE_MCI_VERSION;
	u32 CORE_GENERICS;
	u32 CORE_TESTBUS_CONFIG;
	u32 CORE_TESTBUS_SEL2_BIT;
	u32 CORE_TESTBUS_ENA;
	u32 CORE_TESTBUS_SEL2;
	u32 CORE_PWRCTL_STATUS;
	u32 CORE_PWRCTL_MASK;
	u32 CORE_PWRCTL_CLEAR;
	u32 CORE_PWRCTL_CTL;
	u32 CORE_SDCC_DEBUG_REG;
	u32 CORE_DLL_CONFIG;
	u32 CORE_DLL_STATUS;
	u32 CORE_VENDOR_SPEC;
	u32 CORE_VENDOR_SPEC_ADMA_ERR_ADDR0;
	u32 CORE_VENDOR_SPEC_ADMA_ERR_ADDR1;
	u32 CORE_VENDOR_SPEC_FUNC2;
	u32 CORE_VENDOR_SPEC_CAPABILITIES0;
	u32 CORE_DDR_200_CFG;
	u32 CORE_VENDOR_SPEC3;
	u32 CORE_DLL_CONFIG_2;
	u32 CORE_DLL_CONFIG_3;
	u32 CORE_DDR_CONFIG;
	u32 CORE_DDR_CONFIG_OLD; /* Applcable to sddcc minor ver < 0x49 only */
	u32 CORE_DLL_USR_CTL; /* Present on SDCC5.1 onwards */
};

struct sdhci_msm_offset sdhci_msm_offset_mci_removed = {
	.CORE_MCI_DATA_CNT = 0x35C,
	.CORE_MCI_STATUS = 0x324,
	.CORE_MCI_FIFO_CNT = 0x308,
	.CORE_MCI_VERSION = 0x318,
	.CORE_GENERICS = 0x320,
	.CORE_TESTBUS_CONFIG = 0x32C,
	.CORE_TESTBUS_SEL2_BIT = 3,
	.CORE_TESTBUS_ENA = (1 << 31),
	.CORE_TESTBUS_SEL2 = (1 << 3),
	.CORE_PWRCTL_STATUS = 0x240,
	.CORE_PWRCTL_MASK = 0x244,
	.CORE_PWRCTL_CLEAR = 0x248,
	.CORE_PWRCTL_CTL = 0x24C,
	.CORE_SDCC_DEBUG_REG = 0x358,
	.CORE_DLL_CONFIG = 0x200,
	.CORE_DLL_STATUS = 0x208,
	.CORE_VENDOR_SPEC = 0x20C,
	.CORE_VENDOR_SPEC_ADMA_ERR_ADDR0 = 0x214,
	.CORE_VENDOR_SPEC_ADMA_ERR_ADDR1 = 0x218,
	.CORE_VENDOR_SPEC_FUNC2 = 0x210,
	.CORE_VENDOR_SPEC_CAPABILITIES0 = 0x21C,
	.CORE_DDR_200_CFG = 0x224,
	.CORE_VENDOR_SPEC3 = 0x250,
	.CORE_DLL_CONFIG_2 = 0x254,
	.CORE_DLL_CONFIG_3 = 0x258,
	.CORE_DDR_CONFIG = 0x25C,
	.CORE_DLL_USR_CTL = 0x388,
};

struct sdhci_msm_offset sdhci_msm_offset_mci_present = {
	.CORE_MCI_DATA_CNT = 0x30,
	.CORE_MCI_STATUS = 0x34,
	.CORE_MCI_FIFO_CNT = 0x44,
	.CORE_MCI_VERSION = 0x050,
	.CORE_GENERICS = 0x70,
	.CORE_TESTBUS_CONFIG = 0x0CC,
	.CORE_TESTBUS_SEL2_BIT = 4,
	.CORE_TESTBUS_ENA = (1 << 3),
	.CORE_TESTBUS_SEL2 = (1 << 4),
	.CORE_PWRCTL_STATUS = 0xDC,
	.CORE_PWRCTL_MASK = 0xE0,
	.CORE_PWRCTL_CLEAR = 0xE4,
	.CORE_PWRCTL_CTL = 0xE8,
	.CORE_SDCC_DEBUG_REG = 0x124,
	.CORE_DLL_CONFIG = 0x100,
	.CORE_DLL_STATUS = 0x108,
	.CORE_VENDOR_SPEC = 0x10C,
	.CORE_VENDOR_SPEC_ADMA_ERR_ADDR0 = 0x114,
	.CORE_VENDOR_SPEC_ADMA_ERR_ADDR1 = 0x118,
	.CORE_VENDOR_SPEC_FUNC2 = 0x110,
	.CORE_VENDOR_SPEC_CAPABILITIES0 = 0x11C,
	.CORE_DDR_200_CFG = 0x184,
	.CORE_VENDOR_SPEC3 = 0x1B0,
	.CORE_DLL_CONFIG_2 = 0x1B4,
	.CORE_DLL_CONFIG_3 = 0x1B8,
	.CORE_DDR_CONFIG_OLD = 0x1B8, /* Applicable to sdcc minor ver < 0x49 */
	.CORE_DDR_CONFIG = 0x1BC,
};

u8 sdhci_msm_readb_relaxed(struct sdhci_host *host, u32 offset)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;
	void __iomem *base_addr;

	if (msm_host->mci_removed)
		base_addr = host->ioaddr;
	else
		base_addr = msm_host->core_mem;

	return readb_relaxed(base_addr + offset);
}

u32 sdhci_msm_readl_relaxed(struct sdhci_host *host, u32 offset)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;
	void __iomem *base_addr;

	if (msm_host->mci_removed)
		base_addr = host->ioaddr;
	else
		base_addr = msm_host->core_mem;

	return readl_relaxed(base_addr + offset);
}

void sdhci_msm_writeb_relaxed(u8 val, struct sdhci_host *host, u32 offset)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;
	void __iomem *base_addr;

	if (msm_host->mci_removed)
		base_addr = host->ioaddr;
	else
		base_addr = msm_host->core_mem;

	writeb_relaxed(val, base_addr + offset);
}

void sdhci_msm_writel_relaxed(u32 val, struct sdhci_host *host, u32 offset)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;
	void __iomem *base_addr;

	if (msm_host->mci_removed)
		base_addr = host->ioaddr;
	else
		base_addr = msm_host->core_mem;

	writel_relaxed(val, base_addr + offset);
}

/* Timeout value to avoid infinite waiting for pwr_irq */
#define MSM_PWR_IRQ_TIMEOUT_MS 5000

static const u32 tuning_block_64[] = {
	0x00FF0FFF, 0xCCC3CCFF, 0xFFCC3CC3, 0xEFFEFFFE,
	0xDDFFDFFF, 0xFBFFFBFF, 0xFF7FFFBF, 0xEFBDF777,
	0xF0FFF0FF, 0x3CCCFC0F, 0xCFCC33CC, 0xEEFFEFFF,
	0xFDFFFDFF, 0xFFBFFFDF, 0xFFF7FFBB, 0xDE7B7FF7
};

static const u32 tuning_block_128[] = {
	0xFF00FFFF, 0x0000FFFF, 0xCCCCFFFF, 0xCCCC33CC,
	0xCC3333CC, 0xFFFFCCCC, 0xFFFFEEFF, 0xFFEEEEFF,
	0xFFDDFFFF, 0xDDDDFFFF, 0xBBFFFFFF, 0xBBFFFFFF,
	0xFFFFFFBB, 0xFFFFFF77, 0x77FF7777, 0xFFEEDDBB,
	0x00FFFFFF, 0x00FFFFFF, 0xCCFFFF00, 0xCC33CCCC,
	0x3333CCCC, 0xFFCCCCCC, 0xFFEEFFFF, 0xEEEEFFFF,
	0xDDFFFFFF, 0xDDFFFFFF, 0xFFFFFFDD, 0xFFFFFFBB,
	0xFFFFBBBB, 0xFFFF77FF, 0xFF7777FF, 0xEEDDBB77
};

/* global to hold each slot instance for debug */
static struct sdhci_msm_host *sdhci_slot[2];

static int disable_slots;
/* root can write, others read */
module_param(disable_slots, int, 0644);

static bool nocmdq;
module_param(nocmdq, bool, 0644);

enum vdd_io_level {
	/* set vdd_io_data->low_vol_level */
	VDD_IO_LOW,
	/* set vdd_io_data->high_vol_level */
	VDD_IO_HIGH,
	/*
	 * set whatever there in voltage_level (third argument) of
	 * sdhci_msm_set_vdd_io_vol() function.
	 */
	VDD_IO_SET_LEVEL,
};

enum dll_init_context {
	DLL_INIT_NORMAL = 0,
	DLL_INIT_FROM_CX_COLLAPSE_EXIT,
};

static unsigned int sdhci_msm_get_sup_clk_rate(struct sdhci_host *host,
						u32 req_clk);

/* MSM platform specific tuning */
static inline int msm_dll_poll_ck_out_en(struct sdhci_host *host,
						u8 poll)
{
	int rc = 0;
	u32 wait_cnt = 50;
	u8 ck_out_en = 0;
	struct mmc_host *mmc = host->mmc;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;
	const struct sdhci_msm_offset *msm_host_offset =
					msm_host->offset;

	/* poll for CK_OUT_EN bit.  max. poll time = 50us */
	ck_out_en = !!(readl_relaxed(host->ioaddr +
		msm_host_offset->CORE_DLL_CONFIG) & CORE_CK_OUT_EN);

	while (ck_out_en != poll) {
		if (--wait_cnt == 0) {
			pr_err("%s: %s: CK_OUT_EN bit is not %d\n",
				mmc_hostname(mmc), __func__, poll);
			rc = -ETIMEDOUT;
			goto out;
		}
		udelay(1);

		ck_out_en = !!(readl_relaxed(host->ioaddr +
			msm_host_offset->CORE_DLL_CONFIG) & CORE_CK_OUT_EN);
	}
out:
	return rc;
}

/*
 * Enable CDR to track changes of DAT lines and adjust sampling
 * point according to voltage/temperature variations
 */
static int msm_enable_cdr_cm_sdc4_dll(struct sdhci_host *host)
{
	int rc = 0;
	u32 config;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;
	const struct sdhci_msm_offset *msm_host_offset =
					msm_host->offset;

	config = readl_relaxed(host->ioaddr +
		msm_host_offset->CORE_DLL_CONFIG);
	config |= CORE_CDR_EN;
	config &= ~(CORE_CDR_EXT_EN | CORE_CK_OUT_EN);
	writel_relaxed(config, host->ioaddr +
		msm_host_offset->CORE_DLL_CONFIG);

	rc = msm_dll_poll_ck_out_en(host, 0);
	if (rc)
		goto err;

	writel_relaxed((readl_relaxed(host->ioaddr +
		msm_host_offset->CORE_DLL_CONFIG) | CORE_CK_OUT_EN),
		host->ioaddr + msm_host_offset->CORE_DLL_CONFIG);

	rc = msm_dll_poll_ck_out_en(host, 1);
	if (rc)
		goto err;
	goto out;
err:
	pr_err("%s: %s: failed\n", mmc_hostname(host->mmc), __func__);
out:
	return rc;
}

static ssize_t store_auto_cmd21(struct device *dev, struct device_attribute
				*attr, const char *buf, size_t count)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;
	u32 tmp;
	unsigned long flags;

	if (!kstrtou32(buf, 0, &tmp)) {
		spin_lock_irqsave(&host->lock, flags);
		msm_host->en_auto_cmd21 = !!tmp;
		spin_unlock_irqrestore(&host->lock, flags);
	}
	return count;
}

static ssize_t show_auto_cmd21(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;

	return snprintf(buf, PAGE_SIZE, "%d\n", msm_host->en_auto_cmd21);
}

/* MSM auto-tuning handler */
static int sdhci_msm_config_auto_tuning_cmd(struct sdhci_host *host,
					    bool enable,
					    u32 type)
{
	int rc = 0;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;
	const struct sdhci_msm_offset *msm_host_offset =
					msm_host->offset;
	u32 val = 0;

	if (!msm_host->en_auto_cmd21)
		return 0;

	if (type == MMC_SEND_TUNING_BLOCK_HS200)
		val = CORE_HC_AUTO_CMD21_EN;
	else
		return 0;

	if (enable) {
		rc = msm_enable_cdr_cm_sdc4_dll(host);
		writel_relaxed(readl_relaxed(host->ioaddr +
			msm_host_offset->CORE_VENDOR_SPEC) | val,
			host->ioaddr + msm_host_offset->CORE_VENDOR_SPEC);
	} else {
		writel_relaxed(readl_relaxed(host->ioaddr +
			msm_host_offset->CORE_VENDOR_SPEC) & ~val,
			host->ioaddr + msm_host_offset->CORE_VENDOR_SPEC);
	}
	return rc;
}

static int msm_config_cm_dll_phase(struct sdhci_host *host, u8 phase)
{
	int rc = 0;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;
	const struct sdhci_msm_offset *msm_host_offset =
					msm_host->offset;
	u8 grey_coded_phase_table[] = {0x0, 0x1, 0x3, 0x2, 0x6, 0x7, 0x5, 0x4,
					0xC, 0xD, 0xF, 0xE, 0xA, 0xB, 0x9,
					0x8};
	unsigned long flags;
	u32 config;
	struct mmc_host *mmc = host->mmc;

	pr_debug("%s: Enter %s\n", mmc_hostname(mmc), __func__);
	spin_lock_irqsave(&host->lock, flags);

	config = readl_relaxed(host->ioaddr +
		msm_host_offset->CORE_DLL_CONFIG);
	config &= ~(CORE_CDR_EN | CORE_CK_OUT_EN);
	config |= (CORE_CDR_EXT_EN | CORE_DLL_EN);
	writel_relaxed(config, host->ioaddr +
		msm_host_offset->CORE_DLL_CONFIG);

	/* Wait until CK_OUT_EN bit of DLL_CONFIG register becomes '0' */
	rc = msm_dll_poll_ck_out_en(host, 0);
	if (rc)
		goto err_out;

	/*
	 * Write the selected DLL clock output phase (0 ... 15)
	 * to CDR_SELEXT bit field of DLL_CONFIG register.
	 */
	writel_relaxed(((readl_relaxed(host->ioaddr +
			msm_host_offset->CORE_DLL_CONFIG)
			& ~(0xF << 20))
			| (grey_coded_phase_table[phase] << 20)),
			host->ioaddr + msm_host_offset->CORE_DLL_CONFIG);

	/* Set CK_OUT_EN bit of DLL_CONFIG register to 1. */
	writel_relaxed((readl_relaxed(host->ioaddr +
		msm_host_offset->CORE_DLL_CONFIG) | CORE_CK_OUT_EN),
		host->ioaddr + msm_host_offset->CORE_DLL_CONFIG);

	/* Wait until CK_OUT_EN bit of DLL_CONFIG register becomes '1' */
	rc = msm_dll_poll_ck_out_en(host, 1);
	if (rc)
		goto err_out;

	config = readl_relaxed(host->ioaddr +
		msm_host_offset->CORE_DLL_CONFIG);
	config |= CORE_CDR_EN;
	config &= ~CORE_CDR_EXT_EN;
	writel_relaxed(config, host->ioaddr +
		msm_host_offset->CORE_DLL_CONFIG);
	goto out;

err_out:
	pr_err("%s: %s: Failed to set DLL phase: %d\n",
		mmc_hostname(mmc), __func__, phase);
out:
	spin_unlock_irqrestore(&host->lock, flags);
	pr_debug("%s: Exit %s\n", mmc_hostname(mmc), __func__);
	return rc;
}

/*
 * Find out the greatest range of consecuitive selected
 * DLL clock output phases that can be used as sampling
 * setting for SD3.0 UHS-I card read operation (in SDR104
 * timing mode) or for eMMC4.5 card read operation (in
 * HS400/HS200 timing mode).
 * Select the 3/4 of the range and configure the DLL with the
 * selected DLL clock output phase.
 */

static int msm_find_most_appropriate_phase(struct sdhci_host *host,
				u8 *phase_table, u8 total_phases)
{
	int ret;
	u8 ranges[MAX_PHASES][MAX_PHASES] = { {0}, {0} };
	u8 phases_per_row[MAX_PHASES] = {0};
	int row_index = 0, col_index = 0, selected_row_index = 0, curr_max = 0;
	int i, cnt, phase_0_raw_index = 0, phase_15_raw_index = 0;
	bool phase_0_found = false, phase_15_found = false;
	struct mmc_host *mmc = host->mmc;

	pr_debug("%s: Enter %s\n", mmc_hostname(mmc), __func__);
	if (!total_phases || (total_phases > MAX_PHASES)) {
		pr_err("%s: %s: invalid argument: total_phases=%d\n",
			mmc_hostname(mmc), __func__, total_phases);
		return -EINVAL;
	}

	for (cnt = 0; cnt < total_phases; cnt++) {
		ranges[row_index][col_index] = phase_table[cnt];
		phases_per_row[row_index] += 1;
		col_index++;

		if ((cnt + 1) == total_phases) {
			continue;
		/* check if next phase in phase_table is consecutive or not */
		} else if ((phase_table[cnt] + 1) != phase_table[cnt + 1]) {
			row_index++;
			col_index = 0;
		}
	}

	if (row_index >= MAX_PHASES)
		return -EINVAL;

	/* Check if phase-0 is present in first valid window? */
	if (!ranges[0][0]) {
		phase_0_found = true;
		phase_0_raw_index = 0;
		/* Check if cycle exist between 2 valid windows */
		for (cnt = 1; cnt <= row_index; cnt++) {
			if (phases_per_row[cnt]) {
				for (i = 0; i < phases_per_row[cnt]; i++) {
					if (ranges[cnt][i] == 15) {
						phase_15_found = true;
						phase_15_raw_index = cnt;
						break;
					}
				}
			}
		}
	}

	/* If 2 valid windows form cycle then merge them as single window */
	if (phase_0_found && phase_15_found) {
		/* number of phases in raw where phase 0 is present */
		u8 phases_0 = phases_per_row[phase_0_raw_index];
		/* number of phases in raw where phase 15 is present */
		u8 phases_15 = phases_per_row[phase_15_raw_index];

		if (phases_0 + phases_15 >= MAX_PHASES)
			/*
			 * If there are more than 1 phase windows then total
			 * number of phases in both the windows should not be
			 * more than or equal to MAX_PHASES.
			 */
			return -EINVAL;

		/* Merge 2 cyclic windows */
		i = phases_15;
		for (cnt = 0; cnt < phases_0; cnt++) {
			ranges[phase_15_raw_index][i] =
				ranges[phase_0_raw_index][cnt];
			if (++i >= MAX_PHASES)
				break;
		}

		phases_per_row[phase_0_raw_index] = 0;
		phases_per_row[phase_15_raw_index] = phases_15 + phases_0;
	}

	for (cnt = 0; cnt <= row_index; cnt++) {
		if (phases_per_row[cnt] > curr_max) {
			curr_max = phases_per_row[cnt];
			selected_row_index = cnt;
		}
	}

	i = ((curr_max * 3) / 4);
	if (i)
		i--;

	ret = (int)ranges[selected_row_index][i];

	if (ret >= MAX_PHASES) {
		ret = -EINVAL;
		pr_err("%s: %s: invalid phase selected=%d\n",
			mmc_hostname(mmc), __func__, ret);
	}

	pr_debug("%s: Exit %s\n", mmc_hostname(mmc), __func__);
	return ret;
}

static inline void msm_cm_dll_set_freq(struct sdhci_host *host)
{
	u32 mclk_freq = 0;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;
	const struct sdhci_msm_offset *msm_host_offset =
					msm_host->offset;

	/* Program the MCLK value to MCLK_FREQ bit field */
	if (host->clock <= 112000000)
		mclk_freq = 0;
	else if (host->clock <= 125000000)
		mclk_freq = 1;
	else if (host->clock <= 137000000)
		mclk_freq = 2;
	else if (host->clock <= 150000000)
		mclk_freq = 3;
	else if (host->clock <= 162000000)
		mclk_freq = 4;
	else if (host->clock <= 175000000)
		mclk_freq = 5;
	else if (host->clock <= 187000000)
		mclk_freq = 6;
	else if (host->clock <= 208000000)
		mclk_freq = 7;

	writel_relaxed(((readl_relaxed(host->ioaddr +
			msm_host_offset->CORE_DLL_CONFIG)
			& ~(7 << 24)) | (mclk_freq << 24)),
			host->ioaddr + msm_host_offset->CORE_DLL_CONFIG);
}

/* Initialize the DLL (Programmable Delay Line ) */
static int msm_init_cm_dll(struct sdhci_host *host,
				enum dll_init_context init_context)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;
	const struct sdhci_msm_offset *msm_host_offset =
					msm_host->offset;
	struct mmc_host *mmc = host->mmc;
	int rc = 0;
	unsigned long flags;
	u32 wait_cnt;
	bool prev_pwrsave, curr_pwrsave;

	pr_debug("%s: Enter %s\n", mmc_hostname(mmc), __func__);
	spin_lock_irqsave(&host->lock, flags);
	prev_pwrsave = !!(readl_relaxed(host->ioaddr +
		msm_host_offset->CORE_VENDOR_SPEC) & CORE_CLK_PWRSAVE);
	curr_pwrsave = prev_pwrsave;
	/*
	 * Make sure that clock is always enabled when DLL
	 * tuning is in progress. Keeping PWRSAVE ON may
	 * turn off the clock. So let's disable the PWRSAVE
	 * here and re-enable it once tuning is completed.
	 */
	if (prev_pwrsave) {
		writel_relaxed((readl_relaxed(host->ioaddr +
			msm_host_offset->CORE_VENDOR_SPEC)
			& ~CORE_CLK_PWRSAVE), host->ioaddr +
			msm_host_offset->CORE_VENDOR_SPEC);
		curr_pwrsave = false;
	}

	if (msm_host->use_updated_dll_reset) {
		/* Disable the DLL clock */
		writel_relaxed((readl_relaxed(host->ioaddr +
				msm_host_offset->CORE_DLL_CONFIG)
				& ~CORE_CK_OUT_EN), host->ioaddr +
				msm_host_offset->CORE_DLL_CONFIG);

		writel_relaxed((readl_relaxed(host->ioaddr +
				msm_host_offset->CORE_DLL_CONFIG_2)
				| CORE_DLL_CLOCK_DISABLE), host->ioaddr +
				msm_host_offset->CORE_DLL_CONFIG_2);
	}

	/* Write 1 to DLL_RST bit of DLL_CONFIG register */
	writel_relaxed((readl_relaxed(host->ioaddr +
		msm_host_offset->CORE_DLL_CONFIG) | CORE_DLL_RST),
		host->ioaddr + msm_host_offset->CORE_DLL_CONFIG);

	/* Write 1 to DLL_PDN bit of DLL_CONFIG register */
	writel_relaxed((readl_relaxed(host->ioaddr +
		msm_host_offset->CORE_DLL_CONFIG) | CORE_DLL_PDN),
		host->ioaddr + msm_host_offset->CORE_DLL_CONFIG);

	if (msm_host->use_updated_dll_reset) {
		u32 mclk_freq = 0;
		u32 actual_clk = sdhci_msm_get_sup_clk_rate(host, host->clock);

		/*
		 * Only configure the mclk_freq in normal DLL init
		 * context. If the DLL init is coming from
		 * CX Collapse Exit context, the host->clock may be zero.
		 * The DLL_CONFIG_2 register has already been restored to
		 * proper value prior to getting here.
		 */
		if (init_context == DLL_INIT_NORMAL) {
			switch (actual_clk) {
			case 208000000:
			case 202000000:
			case 201500000:
			case 200000000:
				mclk_freq = 42;
				break;
			case 192000000:
				mclk_freq = 40;
				break;
			default:
				mclk_freq = (u32)((actual_clk / TCXO_FREQ) * 4);
				pr_info_once("%s: %s: Non standard clk freq =%u\n",
				mmc_hostname(mmc), __func__, actual_clk);
			}

			if ((readl_relaxed(host->ioaddr +
				msm_host_offset->CORE_DLL_CONFIG_2)
				& CORE_FLL_CYCLE_CNT))
				mclk_freq *= 2;

			writel_relaxed(((readl_relaxed(host->ioaddr +
			   msm_host_offset->CORE_DLL_CONFIG_2)
			   & ~(0xFF << 10)) | (mclk_freq << 10)),
			   host->ioaddr + msm_host_offset->CORE_DLL_CONFIG_2);
		}
		/* wait for 5us before enabling DLL clock */
		udelay(5);
	}

	/* Write 0 to DLL_RST bit of DLL_CONFIG register */
	writel_relaxed((readl_relaxed(host->ioaddr +
			msm_host_offset->CORE_DLL_CONFIG) & ~CORE_DLL_RST),
			host->ioaddr + msm_host_offset->CORE_DLL_CONFIG);

	/* Write 0 to DLL_PDN bit of DLL_CONFIG register */
	writel_relaxed((readl_relaxed(host->ioaddr +
			msm_host_offset->CORE_DLL_CONFIG) & ~CORE_DLL_PDN),
			host->ioaddr + msm_host_offset->CORE_DLL_CONFIG);

	if (msm_host->use_updated_dll_reset) {
		/* Enable the DLL clock */
		writel_relaxed((readl_relaxed(host->ioaddr +
				msm_host_offset->CORE_DLL_CONFIG_2)
				& ~CORE_DLL_CLOCK_DISABLE), host->ioaddr +
				msm_host_offset->CORE_DLL_CONFIG_2);
	}

	/* Configure Tassadar DLL (Only applicable for 7FF projects) */
	if (msm_host->use_7nm_dll) {
		if (msm_host->dll_hsr) {
			writel_relaxed(msm_host->dll_hsr->dll_usr_ctl,
					host->ioaddr +
					msm_host_offset->CORE_DLL_USR_CTL);
			writel_relaxed(msm_host->dll_hsr->dll_config_3,
					host->ioaddr +
					msm_host_offset->CORE_DLL_CONFIG_3);
		} else {
			writel_relaxed(DLL_USR_CTL_POR_VAL | FINE_TUNE_MODE_EN |
					ENABLE_DLL_LOCK_STATUS | BIAS_OK_SIGNAL,
					host->ioaddr +
					msm_host_offset->CORE_DLL_USR_CTL);

			writel_relaxed(DLL_CONFIG_3_POR_VAL, host->ioaddr +
				msm_host_offset->CORE_DLL_CONFIG_3);
		}
	}

	/*
	 * Update the lower two bytes of DLL_CONFIG only with HSR values.
	 * Since these are the static settings.
	 */
	if (msm_host->dll_hsr) {
		writel_relaxed((readl_relaxed(host->ioaddr +
			msm_host_offset->CORE_DLL_CONFIG) |
			(msm_host->dll_hsr->dll_config & 0xffff)),
			host->ioaddr + msm_host_offset->CORE_DLL_CONFIG);
	}

	/* Set DLL_EN bit to 1. */
	writel_relaxed((readl_relaxed(host->ioaddr +
			msm_host_offset->CORE_DLL_CONFIG) | CORE_DLL_EN),
			host->ioaddr + msm_host_offset->CORE_DLL_CONFIG);

	/* Set CK_OUT_EN bit to 1. */
	writel_relaxed((readl_relaxed(host->ioaddr +
			msm_host_offset->CORE_DLL_CONFIG)
			| CORE_CK_OUT_EN), host->ioaddr +
			msm_host_offset->CORE_DLL_CONFIG);

	wait_cnt = 50;
	/* Wait until DLL_LOCK bit of DLL_STATUS register becomes '1' */
	while (!(readl_relaxed(host->ioaddr +
		msm_host_offset->CORE_DLL_STATUS) & CORE_DLL_LOCK)) {
		/* max. wait for 50us sec for LOCK bit to be set */
		if (--wait_cnt == 0) {
			pr_err("%s: %s: DLL failed to LOCK\n",
				mmc_hostname(mmc), __func__);
			rc = -ETIMEDOUT;
			goto out;
		}
		/* wait for 1us before polling again */
		udelay(1);
	}

out:
	/* Restore the correct PWRSAVE state */
	if (prev_pwrsave ^ curr_pwrsave) {
		u32 reg = readl_relaxed(host->ioaddr +
			msm_host_offset->CORE_VENDOR_SPEC);

		if (prev_pwrsave)
			reg |= CORE_CLK_PWRSAVE;
		else
			reg &= ~CORE_CLK_PWRSAVE;

		writel_relaxed(reg, host->ioaddr +
			msm_host_offset->CORE_VENDOR_SPEC);
	}

	spin_unlock_irqrestore(&host->lock, flags);
	pr_debug("%s: Exit %s\n", mmc_hostname(mmc), __func__);
	return rc;
}

static int sdhci_msm_cdclp533_calibration(struct sdhci_host *host)
{
	u32 calib_done;
	int ret = 0;
	int cdc_err = 0;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;
	const struct sdhci_msm_offset *msm_host_offset =
					msm_host->offset;

	pr_debug("%s: Enter %s\n", mmc_hostname(host->mmc), __func__);

	/* Write 0 to CDC_T4_DLY_SEL field in VENDOR_SPEC_DDR200_CFG */
	writel_relaxed((readl_relaxed(host->ioaddr +
			msm_host_offset->CORE_DDR_200_CFG)
			& ~CORE_CDC_T4_DLY_SEL),
			host->ioaddr + msm_host_offset->CORE_DDR_200_CFG);

	/* Write 0 to CDC_SWITCH_BYPASS_OFF field in CORE_CSR_CDC_GEN_CFG */
	writel_relaxed((readl_relaxed(host->ioaddr + CORE_CSR_CDC_GEN_CFG)
			& ~CORE_CDC_SWITCH_BYPASS_OFF),
			host->ioaddr + CORE_CSR_CDC_GEN_CFG);

	/* Write 1 to CDC_SWITCH_RC_EN field in CORE_CSR_CDC_GEN_CFG */
	writel_relaxed((readl_relaxed(host->ioaddr + CORE_CSR_CDC_GEN_CFG)
			| CORE_CDC_SWITCH_RC_EN),
			host->ioaddr + CORE_CSR_CDC_GEN_CFG);

	/* Write 0 to START_CDC_TRAFFIC field in CORE_DDR200_CFG */
	writel_relaxed((readl_relaxed(host->ioaddr +
			msm_host_offset->CORE_DDR_200_CFG)
			& ~CORE_START_CDC_TRAFFIC),
			host->ioaddr + msm_host_offset->CORE_DDR_200_CFG);

	/*
	 * Perform CDC Register Initialization Sequence
	 *
	 * CORE_CSR_CDC_CTLR_CFG0	0x11800EC
	 * CORE_CSR_CDC_CTLR_CFG1	0x3011111
	 * CORE_CSR_CDC_CAL_TIMER_CFG0	0x1201000
	 * CORE_CSR_CDC_CAL_TIMER_CFG1	0x4
	 * CORE_CSR_CDC_REFCOUNT_CFG	0xCB732020
	 * CORE_CSR_CDC_COARSE_CAL_CFG	0xB19
	 * CORE_CSR_CDC_DELAY_CFG	0x3AC
	 * CORE_CDC_OFFSET_CFG		0x0
	 * CORE_CDC_SLAVE_DDA_CFG	0x16334
	 */

	writel_relaxed(0x11800EC, host->ioaddr + CORE_CSR_CDC_CTLR_CFG0);
	writel_relaxed(0x3011111, host->ioaddr + CORE_CSR_CDC_CTLR_CFG1);
	writel_relaxed(0x1201000, host->ioaddr + CORE_CSR_CDC_CAL_TIMER_CFG0);
	writel_relaxed(0x4, host->ioaddr + CORE_CSR_CDC_CAL_TIMER_CFG1);
	writel_relaxed(0xCB732020, host->ioaddr + CORE_CSR_CDC_REFCOUNT_CFG);
	writel_relaxed(0xB19, host->ioaddr + CORE_CSR_CDC_COARSE_CAL_CFG);
	writel_relaxed(0x4E2, host->ioaddr + CORE_CSR_CDC_DELAY_CFG);
	writel_relaxed(0x0, host->ioaddr + CORE_CDC_OFFSET_CFG);
	writel_relaxed(0x16334, host->ioaddr + CORE_CDC_SLAVE_DDA_CFG);

	/* CDC HW Calibration */

	/* Write 1 to SW_TRIG_FULL_CALIB field in CORE_CSR_CDC_CTLR_CFG0 */
	writel_relaxed((readl_relaxed(host->ioaddr + CORE_CSR_CDC_CTLR_CFG0)
			| CORE_SW_TRIG_FULL_CALIB),
			host->ioaddr + CORE_CSR_CDC_CTLR_CFG0);

	/* Write 0 to SW_TRIG_FULL_CALIB field in CORE_CSR_CDC_CTLR_CFG0 */
	writel_relaxed((readl_relaxed(host->ioaddr + CORE_CSR_CDC_CTLR_CFG0)
			& ~CORE_SW_TRIG_FULL_CALIB),
			host->ioaddr + CORE_CSR_CDC_CTLR_CFG0);

	/* Write 1 to HW_AUTOCAL_ENA field in CORE_CSR_CDC_CTLR_CFG0 */
	writel_relaxed((readl_relaxed(host->ioaddr + CORE_CSR_CDC_CTLR_CFG0)
			| CORE_HW_AUTOCAL_ENA),
			host->ioaddr + CORE_CSR_CDC_CTLR_CFG0);

	/* Write 1 to TIMER_ENA field in CORE_CSR_CDC_CAL_TIMER_CFG0 */
	writel_relaxed((readl_relaxed(host->ioaddr +
			CORE_CSR_CDC_CAL_TIMER_CFG0) | CORE_TIMER_ENA),
			host->ioaddr + CORE_CSR_CDC_CAL_TIMER_CFG0);

	mb();

	/* Poll on CALIBRATION_DONE field in CORE_CSR_CDC_STATUS0 to be 1 */
	ret = readl_poll_timeout(host->ioaddr + CORE_CSR_CDC_STATUS0,
		 calib_done, (calib_done & CORE_CALIBRATION_DONE), 1, 50);

	if (ret == -ETIMEDOUT) {
		pr_err("%s: %s: CDC Calibration was not completed\n",
				mmc_hostname(host->mmc), __func__);
		goto out;
	}

	/* Verify CDC_ERROR_CODE field in CORE_CSR_CDC_STATUS0 is 0 */
	cdc_err = readl_relaxed(host->ioaddr + CORE_CSR_CDC_STATUS0)
			& CORE_CDC_ERROR_CODE_MASK;
	if (cdc_err) {
		pr_err("%s: %s: CDC Error Code %d\n",
			mmc_hostname(host->mmc), __func__, cdc_err);
		ret = -EINVAL;
		goto out;
	}

	/* Write 1 to START_CDC_TRAFFIC field in CORE_DDR200_CFG */
	writel_relaxed((readl_relaxed(host->ioaddr +
			msm_host_offset->CORE_DDR_200_CFG)
			| CORE_START_CDC_TRAFFIC),
			host->ioaddr + msm_host_offset->CORE_DDR_200_CFG);
out:
	pr_debug("%s: Exit %s, ret:%d\n", mmc_hostname(host->mmc),
			__func__, ret);
	return ret;
}

static int sdhci_msm_cm_dll_sdc4_calibration(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;
	const struct sdhci_msm_offset *msm_host_offset =
					msm_host->offset;
	u32 dll_status;
	int ret = 0;

	pr_debug("%s: Enter %s\n", mmc_hostname(host->mmc), __func__);

	/*
	 * Reprogramming the value in case it might have been modified by
	 * bootloaders.
	 */
	if (msm_host->dll_hsr && msm_host->dll_hsr->ddr_config) {
		writel_relaxed(msm_host->dll_hsr->ddr_config, host->ioaddr +
			msm_host_offset->CORE_DDR_CONFIG);
	} else if (msm_host->rclk_delay_fix) {
		writel_relaxed(DDR_CONFIG_POR_VAL, host->ioaddr +
			msm_host_offset->CORE_DDR_CONFIG);
	} else {
		writel_relaxed(DDR_CONFIG_POR_VAL, host->ioaddr +
			msm_host_offset->CORE_DDR_CONFIG_OLD);
	}

	if (msm_host->enhanced_strobe && mmc_card_strobe(msm_host->mmc->card))
		writel_relaxed((readl_relaxed(host->ioaddr +
				msm_host_offset->CORE_DDR_200_CFG)
				| CORE_CMDIN_RCLK_EN), host->ioaddr +
				msm_host_offset->CORE_DDR_200_CFG);

	/* Write 1 to DDR_CAL_EN field in CORE_DLL_CONFIG_2 */
	writel_relaxed((readl_relaxed(host->ioaddr +
			msm_host_offset->CORE_DLL_CONFIG_2)
			| CORE_DDR_CAL_EN),
			host->ioaddr + msm_host_offset->CORE_DLL_CONFIG_2);

	/* Poll on DDR_DLL_LOCK bit in CORE_DLL_STATUS to be set */
	ret = readl_poll_timeout(host->ioaddr +
		 msm_host_offset->CORE_DLL_STATUS,
		 dll_status, (dll_status & CORE_DDR_DLL_LOCK), 10, 1000);

	if (ret == -ETIMEDOUT) {
		pr_err("%s: %s: CM_DLL_SDC4 Calibration was not completed\n",
				mmc_hostname(host->mmc), __func__);
		goto out;
	}

	/*
	 * set CORE_PWRSAVE_DLL bit in CORE_VENDOR_SPEC3.
	 * when MCLK is gated OFF, it is not gated for less than 0.5us
	 * and MCLK must be switched on for at-least 1us before DATA
	 * starts coming. Controllers with 14lpp tech DLL cannot
	 * guarantee above requirement. So PWRSAVE_DLL should not be
	 * turned on for host controllers using this DLL.
	 */
	if (!msm_host->use_14lpp_dll)
		writel_relaxed((readl_relaxed(host->ioaddr +
				msm_host_offset->CORE_VENDOR_SPEC3)
				| CORE_PWRSAVE_DLL), host->ioaddr +
				msm_host_offset->CORE_VENDOR_SPEC3);
	mb();
out:
	pr_debug("%s: Exit %s, ret:%d\n", mmc_hostname(host->mmc),
			__func__, ret);
	return ret;
}

static int sdhci_msm_enhanced_strobe(struct sdhci_host *host)
{
	int ret = 0;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;
	struct mmc_host *mmc = host->mmc;

	pr_debug("%s: Enter %s\n", mmc_hostname(host->mmc), __func__);

	if (!msm_host->enhanced_strobe || !mmc_card_strobe(mmc->card)) {
		pr_debug("%s: host/card does not support hs400 enhanced strobe\n",
				mmc_hostname(mmc));
		return -EINVAL;
	}

	if (msm_host->calibration_done ||
		!(mmc->ios.timing == MMC_TIMING_MMC_HS400)) {
		return 0;
	}

	/*
	 * Reset the tuning block.
	 */
	ret = msm_init_cm_dll(host, DLL_INIT_NORMAL);
	if (ret)
		goto out;

	ret = sdhci_msm_cm_dll_sdc4_calibration(host);
out:
	if (!ret)
		msm_host->calibration_done = true;
	pr_debug("%s: Exit %s, ret:%d\n", mmc_hostname(host->mmc),
			__func__, ret);
	return ret;
}

static int sdhci_msm_hs400_dll_calibration(struct sdhci_host *host)
{
	int ret = 0;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;
	const struct sdhci_msm_offset *msm_host_offset =
					msm_host->offset;

	pr_debug("%s: Enter %s\n", mmc_hostname(host->mmc), __func__);

	/*
	 * Retuning in HS400 (DDR mode) will fail, just reset the
	 * tuning block and restore the saved tuning phase.
	 */
	ret = msm_init_cm_dll(host, DLL_INIT_NORMAL);
	if (ret)
		goto out;

	/* Set the selected phase in delay line hw block */
	ret = msm_config_cm_dll_phase(host, msm_host->saved_tuning_phase);
	if (ret)
		goto out;

	/* Write 1 to CMD_DAT_TRACK_SEL field in DLL_CONFIG */
	writel_relaxed((readl_relaxed(host->ioaddr +
				msm_host_offset->CORE_DLL_CONFIG)
				| CORE_CMD_DAT_TRACK_SEL), host->ioaddr +
				msm_host_offset->CORE_DLL_CONFIG);

	if (msm_host->use_cdclp533)
		/* Calibrate CDCLP533 DLL HW */
		ret = sdhci_msm_cdclp533_calibration(host);
	else
		/* Calibrate CM_DLL_SDC4 HW */
		ret = sdhci_msm_cm_dll_sdc4_calibration(host);
out:
	pr_debug("%s: Exit %s, ret:%d\n", mmc_hostname(host->mmc),
			__func__, ret);
	return ret;
}

static void sdhci_msm_set_mmc_drv_type(struct sdhci_host *host, u32 opcode,
		u8 drv_type)
{
	struct mmc_command cmd = {0};
	struct mmc_request mrq = {NULL};
	struct mmc_host *mmc = host->mmc;
	u8 val = ((drv_type << 4) | 2);

	cmd.opcode = MMC_SWITCH;
	cmd.arg = (MMC_SWITCH_MODE_WRITE_BYTE << 24) |
		(EXT_CSD_HS_TIMING << 16) |
		(val << 8) |
		EXT_CSD_CMD_SET_NORMAL;
	cmd.flags = MMC_CMD_AC | MMC_RSP_R1B;
	/* 1 sec */
	cmd.busy_timeout = 1000 * 1000;

	memset(cmd.resp, 0, sizeof(cmd.resp));
	cmd.retries = 3;

	mrq.cmd = &cmd;
	cmd.data = NULL;

	mmc_wait_for_req(mmc, &mrq);
	pr_debug("%s: %s: set card drive type to %d\n",
			mmc_hostname(mmc), __func__,
			drv_type);
}

int sdhci_msm_execute_tuning(struct sdhci_host *host, u32 opcode)
{
	struct sdhci_host *host = mmc_priv(mmc);
	int tuning_seq_cnt = 10;
	u8 phase, tuned_phases[16], tuned_phase_cnt = 0;
	int rc;
	struct mmc_host *mmc = host->mmc;
	struct mmc_ios	ios = host->mmc->ios;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;
	u8 drv_type = 0;
	bool drv_type_changed = false;
	struct mmc_card *card = host->mmc->card;
	int sts_retry;
	u8 last_good_phase = 0;

	/*
	 * Tuning is required for SDR104, HS200 and HS400 cards and
	 * if clock frequency is greater than 100MHz in these modes.
	 */
	if (host->clock <= CORE_FREQ_100MHZ ||
		!((ios.timing == MMC_TIMING_MMC_HS400) ||
		(ios.timing == MMC_TIMING_MMC_HS200) ||
		(ios.timing == MMC_TIMING_UHS_SDR104)))
		return 0;

	/*
	 * Clear tuning_done flag before tuning to ensure proper
	 * HS400 settings.
	 */
	msm_host->tuning_done = 0;

	/*
	 * For HS400 tuning in HS200 timing requires:
	 * - select MCLK/2 in VENDOR_SPEC
	 * - program MCLK to 400MHz (or nearest supported) in GCC
	 */
	if (msm_host->tuning_in_progress)
		return 0;
	msm_host->tuning_in_progress = true;
	pr_debug("%s: Enter %s\n", mmc_hostname(mmc), __func__);

	/* CDC/SDC4 DLL HW calibration is only required for HS400 mode*/
	if (msm_host->tuning_done && !msm_host->calibration_done &&
		(mmc->ios.timing == MMC_TIMING_MMC_HS400)) {
		rc = sdhci_msm_hs400_dll_calibration(host);
		spin_lock_irqsave(&host->lock, flags);
		if (!rc)
			msm_host->calibration_done = true;
		spin_unlock_irqrestore(&host->lock, flags);
		goto out;
	}

	spin_lock_irqsave(&host->lock, flags);

	if ((opcode == MMC_SEND_TUNING_BLOCK_HS200) &&
		(mmc->ios.bus_width == MMC_BUS_WIDTH_8)) {
		tuning_block_pattern = tuning_block_128;
		size = sizeof(tuning_block_128);
	}
	spin_unlock_irqrestore(&host->lock, flags);

	data_buf = kmalloc(size, GFP_KERNEL);
	if (!data_buf) {
		rc = -ENOMEM;
		goto out;
	}

retry:
	tuned_phase_cnt = 0;

	/* first of all reset the tuning block */
	rc = msm_init_cm_dll(host, DLL_INIT_NORMAL);
	if (rc)
		goto kfree;

	phase = 0;
	do {
		struct mmc_command cmd = {0};
		struct mmc_data data = {0};
		struct mmc_request mrq = {
			.cmd = &cmd,
			.data = &data
		};
		struct scatterlist sg;
		struct mmc_command sts_cmd = {0};

		/* set the phase in delay line hw block */
		rc = msm_config_cm_dll_phase(host, phase);
		if (rc)
			goto kfree;

		cmd.opcode = opcode;
		cmd.flags = MMC_RSP_R1 | MMC_CMD_ADTC;

		data.blksz = size;
		data.blocks = 1;
		data.flags = MMC_DATA_READ;
		data.timeout_ns = 1000 * 1000 * 1000; /* 1 sec */

		data.sg = &sg;
		data.sg_len = 1;
		sg_init_one(&sg, data_buf, size);
		memset(data_buf, 0, size);
		mmc_wait_for_req(mmc, &mrq);

		if (card && (cmd.error || data.error)) {
			/*
			 * Set the dll to last known good phase while sending
			 * status command to ensure that status command won't
			 * fail due to bad phase.
			 */
			if (tuned_phase_cnt)
				last_good_phase =
					tuned_phases[tuned_phase_cnt-1];
			else if (msm_host->saved_tuning_phase !=
					INVALID_TUNING_PHASE)
				last_good_phase = msm_host->saved_tuning_phase;

			rc = msm_config_cm_dll_phase(host, last_good_phase);
			if (rc)
				goto kfree;

			sts_cmd.opcode = MMC_SEND_STATUS;
			sts_cmd.arg = card->rca << 16;
			sts_cmd.flags = MMC_RSP_R1 | MMC_CMD_AC;
			sts_retry = 5;
			while (sts_retry) {
				mmc_wait_for_cmd(mmc, &sts_cmd, 0);

				if (sts_cmd.error ||
				   (R1_CURRENT_STATE(sts_cmd.resp[0])
				   != R1_STATE_TRAN)) {
					sts_retry--;
					/*
					 * wait for at least 146 MCLK cycles for
					 * the card to move to TRANS state. As
					 * the MCLK would be min 200MHz for
					 * tuning, we need max 0.73us delay. To
					 * be on safer side 1ms delay is given.
					 */
					usleep_range(1000, 1200);
					pr_debug("%s: phase %d sts cmd err %d resp 0x%x\n",
						mmc_hostname(mmc), phase,
						sts_cmd.error, sts_cmd.resp[0]);
					continue;
				}
				break;
			};
		}

		if (!cmd.error && !data.error &&
			!memcmp(data_buf, tuning_block_pattern, size)) {
			/* tuning is successful at this tuning point */
			tuned_phases[tuned_phase_cnt++] = phase;
			pr_debug("%s: %s: found *** good *** phase = %d\n",
				mmc_hostname(mmc), __func__, phase);
		} else {
			/* Ignore crc errors occurred during tuning */
			if (cmd.error)
				mmc->err_stats[MMC_ERR_CMD_CRC]--;
			else if (data.error)
				mmc->err_stats[MMC_ERR_DAT_CRC]--;
			pr_debug("%s: %s: found ## bad ## phase = %d\n",
				mmc_hostname(mmc), __func__, phase);
		}
	} while (++phase < 16);

	if ((tuned_phase_cnt == NUM_TUNING_PHASES) &&
			card && mmc_card_mmc(card)) {
		/*
		 * If all phases pass then its a problem. So change the card's
		 * drive type to a different value, if supported and repeat
		 * tuning until at least one phase fails. Then set the original
		 * drive type back.
		 *
		 * If all the phases still pass after trying all possible
		 * drive types, then one of those 16 phases will be picked.
		 * This is no different from what was going on before the
		 * modification to change drive type and retune.
		 */
		pr_debug("%s: tuned phases count: %d\n", mmc_hostname(mmc),
				tuned_phase_cnt);

		/* set drive type to other value . default setting is 0x0 */
		while (++drv_type <= MAX_DRV_TYPES_SUPPORTED_HS200) {
			pr_debug("%s: trying different drive strength (%d)\n",
				mmc_hostname(mmc), drv_type);
			if (card->ext_csd.raw_driver_strength &
					(1 << drv_type)) {
				sdhci_msm_set_mmc_drv_type(host, opcode,
						drv_type);
				if (!drv_type_changed)
					drv_type_changed = true;
				goto retry;
			}
		}
	}

	/* reset drive type to default (50 ohm) if changed */
	if (drv_type_changed)
		sdhci_msm_set_mmc_drv_type(host, opcode, 0);

	if (tuned_phase_cnt) {
		if (tuned_phase_cnt == ARRAY_SIZE(tuned_phases)) {
			/*
			 * All phases valid is _almost_ as bad as no phases
			 * valid.  Probably all phases are not really reliable
			 * but we didn't detect where the unreliable place is.
			 * That means we'll essentially be guessing and hoping
			 * we get a good phase.  Better to try a few times.
			 */
			dev_dbg(mmc_dev(mmc), "%s: All phases valid; try again\n",
				mmc_hostname(mmc));
			if (--tuning_seq_cnt) {
				tuned_phase_cnt = 0;
				goto retry;
			}
		}

		rc = msm_find_most_appropriate_phase(host, tuned_phases,
							tuned_phase_cnt);
		if (rc < 0)
			goto kfree;
		else
			phase = (u8)rc;

		/*
		 * Finally set the selected phase in delay
		 * line hw block.
		 */
		rc = msm_config_cm_dll_phase(host, phase);
		if (rc)
			goto kfree;
		msm_host->saved_tuning_phase = phase;
		pr_debug("%s: %s: finally setting the tuning phase to %d\n",
				mmc_hostname(mmc), __func__, phase);
	} else {
		if (--tuning_seq_cnt)
			goto retry;
		/* tuning failed */
		pr_err("%s: %s: no tuning point found\n",
			mmc_hostname(mmc), __func__);
		rc = -EIO;
	}

kfree:
	kfree(data_buf);
out:
	spin_lock_irqsave(&host->lock, flags);
	if (!rc)
		msm_host->tuning_done = true;
	spin_unlock_irqrestore(&host->lock, flags);
	msm_host->tuning_in_progress = false;
	pr_debug("%s: Exit %s, err(%d)\n", mmc_hostname(mmc), __func__, rc);
	return rc;
}

static int sdhci_msm_setup_gpio(struct sdhci_msm_pltfm_data *pdata, bool enable)
{
	struct sdhci_msm_gpio_data *curr;
	int i, ret = 0;

	curr = pdata->pin_data->gpio_data;
	for (i = 0; i < curr->size; i++) {
		if (!gpio_is_valid(curr->gpio[i].no)) {
			ret = -EINVAL;
			pr_err("%s: Invalid gpio = %d\n", __func__,
					curr->gpio[i].no);
			goto free_gpios;
		}
		if (enable) {
			ret = gpio_request(curr->gpio[i].no,
						curr->gpio[i].name);
			if (ret) {
				pr_err("%s: gpio_request(%d, %s) failed %d\n",
					__func__, curr->gpio[i].no,
					curr->gpio[i].name, ret);
				goto free_gpios;
			}
			curr->gpio[i].is_enabled = true;
		} else {
			gpio_free(curr->gpio[i].no);
			curr->gpio[i].is_enabled = false;
		}
	}
	return ret;

free_gpios:
	for (i--; i >= 0; i--) {
		gpio_free(curr->gpio[i].no);
		curr->gpio[i].is_enabled = false;
	}
	return ret;
}

static int sdhci_msm_config_pinctrl_drv_type(struct sdhci_msm_pltfm_data *pdata,
		unsigned int clock)
{
	int ret = 0;

	if (clock > 150000000) {
		if (pdata->pctrl_data->pins_drv_type_200MHz)
			ret = pinctrl_select_state(pdata->pctrl_data->pctrl,
				pdata->pctrl_data->pins_drv_type_200MHz);
	} else if (clock > 75000000) {
		if (pdata->pctrl_data->pins_drv_type_100MHz)
			ret = pinctrl_select_state(pdata->pctrl_data->pctrl,
				pdata->pctrl_data->pins_drv_type_100MHz);
	} else if (clock > 400000) {
		if (pdata->pctrl_data->pins_drv_type_50MHz)
			ret = pinctrl_select_state(pdata->pctrl_data->pctrl,
				pdata->pctrl_data->pins_drv_type_50MHz);
	} else {
		if (pdata->pctrl_data->pins_drv_type_400KHz)
			ret = pinctrl_select_state(pdata->pctrl_data->pctrl,
				pdata->pctrl_data->pins_drv_type_400KHz);
	}

	return ret;
}

static int sdhci_msm_setup_pinctrl(struct sdhci_msm_pltfm_data *pdata,
		bool enable)
{
	int ret = 0;

	if (enable)
		ret = pinctrl_select_state(pdata->pctrl_data->pctrl,
			pdata->pctrl_data->pins_active);
	else
		ret = pinctrl_select_state(pdata->pctrl_data->pctrl,
			pdata->pctrl_data->pins_sleep);

	if (ret < 0)
		pr_err("%s state for pinctrl failed with %d\n",
			enable ? "Enabling" : "Disabling", ret);

	return ret;
}

static int sdhci_msm_setup_pins(struct sdhci_msm_pltfm_data *pdata, bool enable)
{
	int ret = 0;

	if  (pdata->pin_cfg_sts == enable) {
		return 0;
	} else if (pdata->pctrl_data) {
		ret = sdhci_msm_setup_pinctrl(pdata, enable);
		goto out;
	} else if (!pdata->pin_data) {
		return 0;
	}

	if (pdata->pin_data->is_gpio)
		ret = sdhci_msm_setup_gpio(pdata, enable);
out:
	if (!ret)
		pdata->pin_cfg_sts = enable;

	return ret;
}

static int sdhci_msm_dt_get_array(struct device *dev, const char *prop_name,
				 u32 **out, int *len, u32 size)
{
	int ret = 0;
	struct device_node *np = dev->of_node;
	size_t sz;
	u32 *arr = NULL;

	if (!of_get_property(np, prop_name, len)) {
		ret = -EINVAL;
		goto out;
	}
	sz = *len = *len / sizeof(*arr);
	if (sz <= 0 || (size > 0 && (sz > size))) {
		dev_err(dev, "%s invalid size\n", prop_name);
		ret = -EINVAL;
		goto out;
	}

	arr = devm_kzalloc(dev, sz * sizeof(*arr), GFP_KERNEL);
	if (!arr) {
		ret = -ENOMEM;
		goto out;
	}

	ret = of_property_read_u32_array(np, prop_name, arr, sz);
	if (ret < 0) {
		dev_err(dev, "%s failed reading array %d\n", prop_name, ret);
		goto out;
	}
	*out = arr;
out:
	if (ret)
		*len = 0;
	return ret;
}

#define MAX_PROP_SIZE 32
static int sdhci_msm_dt_parse_vreg_info(struct device *dev,
		struct sdhci_msm_reg_data **vreg_data, const char *vreg_name)
{
	int len, ret = 0;
	const __be32 *prop;
	char prop_name[MAX_PROP_SIZE];
	struct sdhci_msm_reg_data *vreg;
	struct device_node *np = dev->of_node;

	snprintf(prop_name, MAX_PROP_SIZE, "%s-supply", vreg_name);
	if (!of_parse_phandle(np, prop_name, 0)) {
		dev_info(dev, "No vreg data found for %s\n", vreg_name);
		return ret;
	}

	vreg = devm_kzalloc(dev, sizeof(*vreg), GFP_KERNEL);
	if (!vreg) {
		ret = -ENOMEM;
		return ret;
	}

	vreg->name = vreg_name;

	snprintf(prop_name, MAX_PROP_SIZE,
			"qcom,%s-always-on", vreg_name);
	if (of_get_property(np, prop_name, NULL))
		vreg->is_always_on = true;

	snprintf(prop_name, MAX_PROP_SIZE,
			"qcom,%s-lpm-sup", vreg_name);
	if (of_get_property(np, prop_name, NULL))
		vreg->lpm_sup = true;

	snprintf(prop_name, MAX_PROP_SIZE,
			"qcom,%s-voltage-level", vreg_name);
	prop = of_get_property(np, prop_name, &len);
	if (!prop || (len != (2 * sizeof(__be32)))) {
		dev_warn(dev, "%s %s property\n",
			prop ? "invalid format" : "no", prop_name);
	} else {
		vreg->low_vol_level = be32_to_cpup(&prop[0]);
		vreg->high_vol_level = be32_to_cpup(&prop[1]);
	}

	snprintf(prop_name, MAX_PROP_SIZE,
			"qcom,%s-current-level", vreg_name);
	prop = of_get_property(np, prop_name, &len);
	if (!prop || (len != (2 * sizeof(__be32)))) {
		dev_warn(dev, "%s %s property\n",
			prop ? "invalid format" : "no", prop_name);
	} else {
		vreg->lpm_uA = be32_to_cpup(&prop[0]);
		vreg->hpm_uA = be32_to_cpup(&prop[1]);
	}

	*vreg_data = vreg;
	dev_dbg(dev, "%s: %s %s vol=[%d %d]uV, curr=[%d %d]uA\n",
		vreg->name, vreg->is_always_on ? "always_on," : "",
		vreg->lpm_sup ? "lpm_sup," : "", vreg->low_vol_level,
		vreg->high_vol_level, vreg->lpm_uA, vreg->hpm_uA);

	return ret;
}

static int sdhci_msm_parse_pinctrl_info(struct device *dev,
		struct sdhci_msm_pltfm_data *pdata)
{
	struct sdhci_pinctrl_data *pctrl_data;
	struct pinctrl *pctrl;
	int ret = 0;

	/* Try to obtain pinctrl handle */
	pctrl = devm_pinctrl_get(dev);
	if (IS_ERR(pctrl)) {
		ret = PTR_ERR(pctrl);
		goto out;
	}
	pctrl_data = devm_kzalloc(dev, sizeof(*pctrl_data), GFP_KERNEL);
	if (!pctrl_data) {
		ret = -ENOMEM;
		goto out;
	}
	pctrl_data->pctrl = pctrl;
	/* Look-up and keep the states handy to be used later */
	pctrl_data->pins_active = pinctrl_lookup_state(
			pctrl_data->pctrl, "active");
	if (IS_ERR(pctrl_data->pins_active)) {
		ret = PTR_ERR(pctrl_data->pins_active);
		dev_err(dev, "Could not get active pinstates, err:%d\n", ret);
		goto out;
	}
	pctrl_data->pins_sleep = pinctrl_lookup_state(
			pctrl_data->pctrl, "sleep");
	if (IS_ERR(pctrl_data->pins_sleep)) {
		ret = PTR_ERR(pctrl_data->pins_sleep);
		dev_err(dev, "Could not get sleep pinstates, err:%d\n", ret);
		goto out;
	}

	pctrl_data->pins_drv_type_400KHz = pinctrl_lookup_state(
			pctrl_data->pctrl, "ds_400KHz");
	if (IS_ERR(pctrl_data->pins_drv_type_400KHz)) {
		dev_dbg(dev, "Could not get 400K pinstates, err:%d\n", ret);
		pctrl_data->pins_drv_type_400KHz = NULL;
	}

	pctrl_data->pins_drv_type_50MHz = pinctrl_lookup_state(
			pctrl_data->pctrl, "ds_50MHz");
	if (IS_ERR(pctrl_data->pins_drv_type_50MHz)) {
		dev_dbg(dev, "Could not get 50M pinstates, err:%d\n", ret);
		pctrl_data->pins_drv_type_50MHz = NULL;
	}

	pctrl_data->pins_drv_type_100MHz = pinctrl_lookup_state(
			pctrl_data->pctrl, "ds_100MHz");
	if (IS_ERR(pctrl_data->pins_drv_type_100MHz)) {
		dev_dbg(dev, "Could not get 100M pinstates, err:%d\n", ret);
		pctrl_data->pins_drv_type_100MHz = NULL;
	}

	pctrl_data->pins_drv_type_200MHz = pinctrl_lookup_state(
			pctrl_data->pctrl, "ds_200MHz");
	if (IS_ERR(pctrl_data->pins_drv_type_200MHz)) {
		dev_dbg(dev, "Could not get 200M pinstates, err:%d\n", ret);
		pctrl_data->pins_drv_type_200MHz = NULL;
	}

	pdata->pctrl_data = pctrl_data;
out:
	return ret;
}

#define GPIO_NAME_MAX_LEN 32
static int sdhci_msm_dt_parse_gpio_info(struct device *dev,
		struct sdhci_msm_pltfm_data *pdata)
{
	int ret = 0, cnt, i;
	struct sdhci_msm_pin_data *pin_data;
	struct device_node *np = dev->of_node;

	ret = sdhci_msm_parse_pinctrl_info(dev, pdata);
	if (!ret) {
		goto out;
	} else if (ret == -EPROBE_DEFER) {
		dev_err(dev, "Pinctrl framework not registered, err:%d\n", ret);
		goto out;
	} else {
		dev_err(dev, "Parsing Pinctrl failed with %d, falling back on GPIO lib\n",
			ret);
		ret = 0;
	}
	pin_data = devm_kzalloc(dev, sizeof(*pin_data), GFP_KERNEL);
	if (!pin_data) {
		ret = -ENOMEM;
		goto out;
	}

	cnt = of_gpio_count(np);
	if (cnt > 0) {
		pin_data->gpio_data = devm_kzalloc(dev,
				sizeof(struct sdhci_msm_gpio_data), GFP_KERNEL);
		if (!pin_data->gpio_data) {
			ret = -ENOMEM;
			goto out;
		}
		pin_data->gpio_data->size = cnt;
		pin_data->gpio_data->gpio = devm_kzalloc(dev, cnt *
				sizeof(struct sdhci_msm_gpio), GFP_KERNEL);

		if (!pin_data->gpio_data->gpio) {
			ret = -ENOMEM;
			goto out;
		}

		for (i = 0; i < cnt; i++) {
			const char *name = NULL;
			char result[GPIO_NAME_MAX_LEN];

			pin_data->gpio_data->gpio[i].no = of_get_gpio(np, i);
			of_property_read_string_index(np,
					"qcom,gpio-names", i, &name);

			snprintf(result, GPIO_NAME_MAX_LEN, "%s-%s",
					dev_name(dev), name ? name : "?");
			pin_data->gpio_data->gpio[i].name = result;
			dev_dbg(dev, "%s: gpio[%s] = %d\n", __func__,
					pin_data->gpio_data->gpio[i].name,
					pin_data->gpio_data->gpio[i].no);
		}
	}
	pdata->pin_data = pin_data;
out:
	if (ret)
		dev_err(dev, "%s failed with err %d\n", __func__, ret);
	return ret;
}

#ifdef CONFIG_SMP
static inline void parse_affine_irq(struct sdhci_msm_pltfm_data *pdata)
{
	pdata->pm_qos_data.irq_req_type = PM_QOS_REQ_AFFINE_IRQ;
}
#else
static inline void parse_affine_irq(struct sdhci_msm_pltfm_data *pdata) { }
#endif

static int sdhci_msm_pm_qos_parse_irq(struct device *dev,
		struct sdhci_msm_pltfm_data *pdata)
{
	struct device_node *np = dev->of_node;
	const char *str;
	u32 cpu;
	int ret = 0;
	int i;

	pdata->pm_qos_data.irq_valid = false;
	pdata->pm_qos_data.irq_req_type = PM_QOS_REQ_AFFINE_CORES;
	if (!of_property_read_string(np, "qcom,pm-qos-irq-type", &str) &&
		!strcmp(str, "affine_irq")) {
		parse_affine_irq(pdata);
	}

	/* must specify cpu for "affine_cores" type */
	if (pdata->pm_qos_data.irq_req_type == PM_QOS_REQ_AFFINE_CORES) {
		pdata->pm_qos_data.irq_cpu = -1;
		ret = of_property_read_u32(np, "qcom,pm-qos-irq-cpu", &cpu);
		if (ret) {
			dev_err(dev, "%s: error %d reading irq cpu\n", __func__,
				ret);
			goto out;
		}
		if (cpu < 0 || cpu >= num_possible_cpus()) {
			dev_err(dev, "%s: invalid irq cpu %d (NR_CPUS=%d)\n",
				__func__, cpu, num_possible_cpus());
			ret = -EINVAL;
			goto out;
		}
		pdata->pm_qos_data.irq_cpu = cpu;
	}

	if (of_property_count_u32_elems(np, "qcom,pm-qos-irq-latency") !=
		SDHCI_POWER_POLICY_NUM) {
		dev_err(dev, "%s: could not read %d values for 'qcom,pm-qos-irq-latency'\n",
			__func__, SDHCI_POWER_POLICY_NUM);
		ret = -EINVAL;
		goto out;
	}

	for (i = 0; i < SDHCI_POWER_POLICY_NUM; i++)
		of_property_read_u32_index(np, "qcom,pm-qos-irq-latency", i,
			&pdata->pm_qos_data.irq_latency.latency[i]);

	pdata->pm_qos_data.irq_valid = true;
out:
	return ret;
}

static int sdhci_msm_pm_qos_parse_cpu_groups(struct device *dev,
		struct sdhci_msm_pltfm_data *pdata)
{
	struct device_node *np = dev->of_node;
	u32 mask;
	int nr_groups;
	int ret;
	int i;

	/* Read cpu group mapping */
	nr_groups = of_property_count_u32_elems(np, "qcom,pm-qos-cpu-groups");
	if (nr_groups <= 0) {
		ret = -EINVAL;
		goto out;
	}
	pdata->pm_qos_data.cpu_group_map.nr_groups = nr_groups;
	pdata->pm_qos_data.cpu_group_map.mask =
		kcalloc(nr_groups, sizeof(cpumask_t), GFP_KERNEL);
	if (!pdata->pm_qos_data.cpu_group_map.mask) {
		ret = -ENOMEM;
		goto out;
	}

	for (i = 0; i < nr_groups; i++) {
		of_property_read_u32_index(np, "qcom,pm-qos-cpu-groups",
			i, &mask);

		pdata->pm_qos_data.cpu_group_map.mask[i].bits[0] = mask;
		if (!cpumask_subset(&pdata->pm_qos_data.cpu_group_map.mask[i],
			cpu_possible_mask)) {
			dev_err(dev, "%s: invalid mask 0x%x of cpu group #%d\n",
				__func__, mask, i);
			ret = -EINVAL;
			goto free_res;
		}
	}
	return 0;

free_res:
	kfree(pdata->pm_qos_data.cpu_group_map.mask);
out:
	return ret;
}

static int sdhci_msm_pm_qos_parse_latency(struct device *dev, const char *name,
		int nr_groups, struct sdhci_msm_pm_qos_latency **latency)
{
	struct device_node *np = dev->of_node;
	struct sdhci_msm_pm_qos_latency *values;
	int ret;
	int i;
	int group;
	int cfg;

	ret = of_property_count_u32_elems(np, name);
	if (ret > 0 && ret != SDHCI_POWER_POLICY_NUM * nr_groups) {
		dev_err(dev, "%s: invalid number of values for property %s: expected=%d actual=%d\n",
			__func__, name,	SDHCI_POWER_POLICY_NUM * nr_groups,
			ret);
		return -EINVAL;
	} else if (ret < 0) {
		return ret;
	}

	values = kcalloc(nr_groups, sizeof(struct sdhci_msm_pm_qos_latency),
			GFP_KERNEL);
	if (!values)
		return -ENOMEM;

	for (i = 0; i < SDHCI_POWER_POLICY_NUM * nr_groups; i++) {
		group = i / SDHCI_POWER_POLICY_NUM;
		cfg = i % SDHCI_POWER_POLICY_NUM;
		of_property_read_u32_index(np, name, i,
				&(values[group].latency[cfg]));
	}

	*latency = values;
	return 0;
}

static void sdhci_msm_pm_qos_parse(struct device *dev,
				struct sdhci_msm_pltfm_data *pdata)
{
	if (sdhci_msm_pm_qos_parse_irq(dev, pdata))
		dev_notice(dev, "%s: PM QoS voting for IRQ will be disabled\n",
			__func__);

	if (!sdhci_msm_pm_qos_parse_cpu_groups(dev, pdata)) {
		pdata->pm_qos_data.cmdq_valid =
			!sdhci_msm_pm_qos_parse_latency(dev,
				"qcom,pm-qos-cmdq-latency-us",
				pdata->pm_qos_data.cpu_group_map.nr_groups,
				&pdata->pm_qos_data.cmdq_latency);
		pdata->pm_qos_data.legacy_valid =
			!sdhci_msm_pm_qos_parse_latency(dev,
				"qcom,pm-qos-legacy-latency-us",
				pdata->pm_qos_data.cpu_group_map.nr_groups,
				&pdata->pm_qos_data.latency);
		if (!pdata->pm_qos_data.cmdq_valid &&
			!pdata->pm_qos_data.legacy_valid) {
			/* clean-up previously allocated arrays */
			kfree(pdata->pm_qos_data.latency);
			kfree(pdata->pm_qos_data.cmdq_latency);
			dev_err(dev, "%s: invalid PM QoS latency values. Voting for cpu group will be disabled\n",
				__func__);
		}
	} else {
		dev_notice(dev, "%s: PM QoS voting for cpu group will be disabled\n",
			__func__);
	}
}

static int sdhci_msm_dt_parse_hsr_info(struct device *dev,
		struct sdhci_msm_host *msm_host)

{
	u32 *dll_hsr_table = NULL;
	int dll_hsr_table_len, dll_hsr_reg_count;
	int ret = 0;

static const struct sdhci_pltfm_data sdhci_msm_pdata = {
	.quirks = SDHCI_QUIRK_BROKEN_CARD_DETECTION |
		  SDHCI_QUIRK_NO_CARD_NO_RESET |
		  SDHCI_QUIRK_SINGLE_POWER_WRITE |
		  SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN |
		  SDHCI_QUIRK_MULTIBLOCK_READ_ACMD12,

	.quirks2 = SDHCI_QUIRK2_PRESET_VALUE_BROKEN,
	.ops = &sdhci_msm_ops,
};

static int sdhci_msm_probe(struct platform_device *pdev)
{
	const struct sdhci_msm_offset *msm_host_offset;
	struct sdhci_host *host;
	struct sdhci_pltfm_host *pltfm_host;
	struct sdhci_msm_host *msm_host;
	struct resource *core_memres = NULL;
	int ret = 0, dead = 0, tlmm_cfg = 0;
	u16 host_version;
	u32 irq_status, irq_ctl;
	struct resource *tlmm_memres = NULL;
	void __iomem *tlmm_mem;
	unsigned long flags;
	bool force_probe;

	pr_debug("%s: Enter %s\n", dev_name(&pdev->dev), __func__);
	msm_host = devm_kzalloc(&pdev->dev, sizeof(struct sdhci_msm_host),
				GFP_KERNEL);
	if (!msm_host) {
		ret = -ENOMEM;
		goto out;
	}

	if (of_find_compatible_node(NULL, NULL, "qcom,sdhci-msm-v5")) {
		msm_host->mci_removed = true;
		msm_host->offset = &sdhci_msm_offset_mci_removed;
	} else {
		msm_host->mci_removed = false;
		msm_host->offset = &sdhci_msm_offset_mci_present;
	}
	msm_host_offset = msm_host->offset;
	msm_host->sdhci_msm_pdata.ops = &sdhci_msm_ops;
	host = sdhci_pltfm_init(pdev, &msm_host->sdhci_msm_pdata, 0);
	if (IS_ERR(host)) {
		ret = PTR_ERR(host);
		goto out_host_free;
	}

	pltfm_host = sdhci_priv(host);
	pltfm_host->priv = msm_host;
	msm_host->mmc = host->mmc;
	msm_host->pdev = pdev;

	/* get the ice device vops if present */
	ret = sdhci_msm_ice_get_dev(host);
	if (ret == -EPROBE_DEFER) {
		/*
		 * SDHCI driver might be probed before ICE driver does.
		 * In that case we would like to return EPROBE_DEFER code
		 * in order to delay its probing.
		 */
		dev_err(&pdev->dev, "%s: required ICE device not probed yet err = %d\n",
			__func__, ret);
		goto pltfm_free;

	} else if (ret == -ENODEV) {
		/*
		 * ICE device is not enabled in DTS file. No need for further
		 * initialization of ICE driver.
		 */
		dev_warn(&pdev->dev, "%s: ICE device is not enabled",
			__func__);
	} else if (ret) {
		dev_err(&pdev->dev, "%s: sdhci_msm_ice_get_dev failed %d\n",
			__func__, ret);
		goto pltfm_free;
	}

	/* Extract platform data */
	if (pdev->dev.of_node) {
		ret = of_alias_get_id(pdev->dev.of_node, "sdhc");
		if (ret <= 0) {
			dev_err(&pdev->dev, "Failed to get slot index %d\n",
				ret);
			goto pltfm_free;
		}

		/* Read property to determine if the probe is forced */
		force_probe = of_find_property(pdev->dev.of_node,
			"qcom,force-sdhc1-probe", NULL);

		/* skip the probe if eMMC isn't a boot device */
		if ((ret == 1) && !sdhci_msm_is_bootdevice(&pdev->dev)
		    && !force_probe) {
			ret = -ENODEV;
			goto pltfm_free;
		}

		if (disable_slots & (1 << (ret - 1))) {
			dev_info(&pdev->dev, "%s: Slot %d disabled\n", __func__,
				ret);
			ret = -ENODEV;
			goto pltfm_free;
		}

		if (ret <= 2)
			sdhci_slot[ret-1] = msm_host;

		msm_host->pdata = sdhci_msm_populate_pdata(&pdev->dev,
							   msm_host);
		if (!msm_host->pdata) {
			dev_err(&pdev->dev, "DT parsing error\n");
			goto pltfm_free;
		}
	} else {
		dev_err(&pdev->dev, "No device tree node\n");
		goto pltfm_free;
	}

	/* Setup Clocks */

	/* Setup SDCC bus voter clock. */
	msm_host->bus_clk = devm_clk_get(&pdev->dev, "bus_clk");
	if (!IS_ERR_OR_NULL(msm_host->bus_clk)) {
		/* Vote for max. clk rate for max. performance */
		ret = clk_set_rate(msm_host->bus_clk, INT_MAX);
		if (ret)
			goto pltfm_free;
		ret = clk_prepare_enable(msm_host->bus_clk);
		if (ret)
			goto pltfm_free;
	}

	/* Setup main peripheral bus clock */
	msm_host->pclk = devm_clk_get(&pdev->dev, "iface_clk");
	if (!IS_ERR(msm_host->pclk)) {
		ret = clk_prepare_enable(msm_host->pclk);
		if (ret)
			goto bus_clk_disable;
	}
	atomic_set(&msm_host->controller_clock, 1);

	/* Setup SDC ufs bus aggr clock */
	msm_host->bus_aggr_clk = devm_clk_get(&pdev->dev, "bus_aggr_clk");
	if (!IS_ERR(msm_host->bus_aggr_clk)) {
		ret = clk_prepare_enable(msm_host->bus_aggr_clk);
		if (ret) {
			dev_err(&pdev->dev, "Bus aggregate clk not enabled\n");
			goto pclk_disable;
		}
	}

	if (msm_host->ice.pdev) {
		/* Setup SDC ICE clock */
		msm_host->ice_clk = devm_clk_get(&pdev->dev, "ice_core_clk");
		if (!IS_ERR(msm_host->ice_clk)) {
			/* ICE core has only one clock frequency for now */
			ret = clk_set_rate(msm_host->ice_clk,
					msm_host->pdata->ice_clk_max);
			if (ret) {
				dev_err(&pdev->dev, "ICE_CLK rate set failed (%d) for %u\n",
					ret,
					msm_host->pdata->ice_clk_max);
				goto bus_aggr_clk_disable;
			}
			ret = clk_prepare_enable(msm_host->ice_clk);
			if (ret)
				goto bus_aggr_clk_disable;

			msm_host->ice_clk_rate =
				msm_host->pdata->ice_clk_max;
		}
	}

	/* Setup SDC MMC clock */
	msm_host->clk = devm_clk_get(&pdev->dev, "core_clk");
	if (IS_ERR(msm_host->clk)) {
		ret = PTR_ERR(msm_host->clk);
		goto bus_aggr_clk_disable;
	}

	/* Set to the minimum supported clock frequency */
	ret = clk_set_rate(msm_host->clk, sdhci_msm_get_min_clock(host));
	if (ret) {
		dev_err(&pdev->dev, "MClk rate set failed (%d)\n", ret);
		goto bus_aggr_clk_disable;
	}
	ret = clk_prepare_enable(msm_host->clk);
	if (ret)
		goto bus_aggr_clk_disable;

	msm_host->clk_rate = sdhci_msm_get_min_clock(host);
	atomic_set(&msm_host->clks_on, 1);

	/* Setup CDC calibration fixed feedback clock */
	msm_host->ff_clk = devm_clk_get(&pdev->dev, "cal_clk");
	if (!IS_ERR(msm_host->ff_clk)) {
		ret = clk_prepare_enable(msm_host->ff_clk);
		if (ret)
			goto clk_disable;
	}

	/* Setup CDC calibration sleep clock */
	msm_host->sleep_clk = devm_clk_get(&pdev->dev, "sleep_clk");
	if (!IS_ERR(msm_host->sleep_clk)) {
		ret = clk_prepare_enable(msm_host->sleep_clk);
		if (ret)
			goto ff_clk_disable;
	}

	msm_host->saved_tuning_phase = INVALID_TUNING_PHASE;

	ret = sdhci_msm_bus_register(msm_host, pdev);
	if (ret)
		goto sleep_clk_disable;

	if (msm_host->msm_bus_vote.client_handle)
		INIT_DELAYED_WORK(&msm_host->msm_bus_vote.vote_work,
				  sdhci_msm_bus_work);
	sdhci_msm_bus_voting(host, 1);

	/* Setup regulators */
	ret = sdhci_msm_vreg_init(&pdev->dev, msm_host->pdata, true);
	if (ret) {
		dev_err(&pdev->dev, "Regulator setup failed (%d)\n", ret);
		goto bus_unregister;
	}

	/* Reset the core and Enable SDHC mode */
	core_memres = platform_get_resource_byname(pdev,
				IORESOURCE_MEM, "core_mem");
	if (!msm_host->mci_removed) {
		if (!core_memres) {
			dev_err(&pdev->dev, "Failed to get iomem resource\n");
			goto vreg_deinit;
		}
		msm_host->core_mem = devm_ioremap(&pdev->dev,
			core_memres->start, resource_size(core_memres));

		if (!msm_host->core_mem) {
			dev_err(&pdev->dev, "Failed to remap registers\n");
			ret = -ENOMEM;
			goto vreg_deinit;
		}
	}

	tlmm_memres = platform_get_resource_byname(pdev,
				IORESOURCE_MEM, "tlmm_mem");
	if (tlmm_memres) {
		tlmm_mem = devm_ioremap(&pdev->dev, tlmm_memres->start,
						resource_size(tlmm_memres));

		if (!tlmm_mem) {
			dev_err(&pdev->dev, "Failed to remap tlmm registers\n");
			ret = -ENOMEM;
			goto vreg_deinit;
		}

		ret = of_property_read_u32(pdev->dev.of_node,
					   "tlmm_cfg",
					   &tlmm_cfg);
		if (ret)
			writel_relaxed(readl_relaxed(tlmm_mem) | 0x2, tlmm_mem);
		else
			writel_relaxed(readl_relaxed(tlmm_mem) | tlmm_cfg,
						 tlmm_mem);
	}

	/*
	 * Reset the vendor spec register to power on reset state.
	 */
	writel_relaxed(CORE_VENDOR_SPEC_POR_VAL,
	host->ioaddr + msm_host_offset->CORE_VENDOR_SPEC);

	/* This enable ADMA error interrupt in case of length mismatch */
	writel_relaxed((readl_relaxed(host->ioaddr +
			msm_host_offset->CORE_VENDOR_SPEC) |
			CORE_VNDR_SPEC_ADMA_ERR_SIZE_EN),
			host->ioaddr + msm_host_offset->CORE_VENDOR_SPEC);

	/*
	 * Ensure SDHCI FIFO is enabled by disabling alternative FIFO
	 */
	writel_relaxed((readl_relaxed(host->ioaddr +
			msm_host_offset->CORE_VENDOR_SPEC3) &
			~CORE_FIFO_ALT_EN), host->ioaddr +
			msm_host_offset->CORE_VENDOR_SPEC3);

	if (!msm_host->mci_removed) {
		/* Set HC_MODE_EN bit in HC_MODE register */
		writel_relaxed(HC_MODE_EN, (msm_host->core_mem + CORE_HC_MODE));

		/* Set FF_CLK_SW_RST_DIS bit in HC_MODE register */
		writel_relaxed(readl_relaxed(msm_host->core_mem +
				CORE_HC_MODE) | FF_CLK_SW_RST_DIS,
				msm_host->core_mem + CORE_HC_MODE);
	}
	sdhci_set_default_hw_caps(msm_host, host);

	/*
	 * Set the PAD_PWR_SWITCH_EN bit so that the PAD_PWR_SWITCH bit can
	 * be used as required later on.
	 */
	writel_relaxed((readl_relaxed(host->ioaddr +
			msm_host_offset->CORE_VENDOR_SPEC) |
			CORE_IO_PAD_PWR_SWITCH_EN), host->ioaddr +
			msm_host_offset->CORE_VENDOR_SPEC);
	/*
	 * CORE_SW_RST above may trigger power irq if previous status of PWRCTL
	 * was either BUS_ON or IO_HIGH_V. So before we enable the power irq
	 * interrupt in GIC (by registering the interrupt handler), we need to
	 * ensure that any pending power irq interrupt status is acknowledged
	 * otherwise power irq interrupt handler would be fired prematurely.
	 */
	irq_status = sdhci_msm_readl_relaxed(host,
		msm_host_offset->CORE_PWRCTL_STATUS);
	sdhci_msm_writel_relaxed(irq_status, host,
		msm_host_offset->CORE_PWRCTL_CLEAR);
	irq_ctl = sdhci_msm_readl_relaxed(host,
		msm_host_offset->CORE_PWRCTL_CTL);

	if (irq_status & (CORE_PWRCTL_BUS_ON | CORE_PWRCTL_BUS_OFF))
		irq_ctl |= CORE_PWRCTL_BUS_SUCCESS;
	if (irq_status & (CORE_PWRCTL_IO_HIGH | CORE_PWRCTL_IO_LOW))
		irq_ctl |= CORE_PWRCTL_IO_SUCCESS;
	sdhci_msm_writel_relaxed(irq_ctl, host,
		msm_host_offset->CORE_PWRCTL_CTL);

	/*
	 * Ensure that above writes are propogated before interrupt enablement
	 * in GIC.
	 */
	mb();

	/*
	 * Following are the deviations from SDHC spec v3.0 -
	 * 1. Card detection is handled using separate GPIO.
	 * 2. Bus power control is handled by interacting with PMIC.
	 */
	host->quirks |= SDHCI_QUIRK_BROKEN_CARD_DETECTION;
	host->quirks |= SDHCI_QUIRK_SINGLE_POWER_WRITE;
	host->quirks |= SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN;
	host->quirks |= SDHCI_QUIRK_NO_ENDATTR_IN_NOPDESC;
	host->quirks2 |= SDHCI_QUIRK2_ALWAYS_USE_BASE_CLOCK;
	host->quirks2 |= SDHCI_QUIRK2_IGNORE_DATATOUT_FOR_R1BCMD;
	host->quirks2 |= SDHCI_QUIRK2_BROKEN_PRESET_VALUE;
	host->quirks2 |= SDHCI_QUIRK2_USE_RESERVED_MAX_TIMEOUT;
	host->quirks2 |= SDHCI_QUIRK2_NON_STANDARD_TUNING;
	host->quirks2 |= SDHCI_QUIRK2_USE_PIO_FOR_EMMC_TUNING;

	if (host->quirks2 & SDHCI_QUIRK2_ALWAYS_USE_BASE_CLOCK)
		host->quirks2 |= SDHCI_QUIRK2_DIVIDE_TOUT_BY_4;

	host_version = readw_relaxed((host->ioaddr + SDHCI_HOST_VERSION));
	dev_dbg(&pdev->dev, "Host Version: 0x%x Vendor Version 0x%x\n",
		host_version, ((host_version & SDHCI_VENDOR_VER_MASK) >>
		  SDHCI_VENDOR_VER_SHIFT));
	if (((host_version & SDHCI_VENDOR_VER_MASK) >>
		SDHCI_VENDOR_VER_SHIFT) == SDHCI_VER_100) {
		/*
		 * Add 40us delay in interrupt handler when
		 * operating at initialization frequency(400KHz).
		 */
		host->quirks2 |= SDHCI_QUIRK2_SLOW_INT_CLR;
		/*
		 * Set Software Reset for DAT line in Software
		 * Reset Register (Bit 2).
		 */
		host->quirks2 |= SDHCI_QUIRK2_RDWR_TX_ACTIVE_EOT;
	}

	host->quirks2 |= SDHCI_QUIRK2_IGN_DATA_END_BIT_ERROR;

	/* Setup PWRCTL irq */
	msm_host->pwr_irq = platform_get_irq_byname(pdev, "pwr_irq");
	if (msm_host->pwr_irq < 0) {
		dev_err(&pdev->dev, "Failed to get pwr_irq by name (%d)\n",
				msm_host->pwr_irq);
		goto vreg_deinit;
	}
	ret = devm_request_threaded_irq(&pdev->dev, msm_host->pwr_irq, NULL,
					sdhci_msm_pwr_irq, IRQF_ONESHOT,
					dev_name(&pdev->dev), host);
	if (ret) {
		dev_err(&pdev->dev, "Request threaded irq(%d) failed (%d)\n",
				msm_host->pwr_irq, ret);
		goto vreg_deinit;
	}

	/* Enable pwr irq interrupts */
	sdhci_msm_writel_relaxed(INT_MASK, host,
		msm_host_offset->CORE_PWRCTL_MASK);

#ifdef CONFIG_MMC_CLKGATE
	/* Set clock gating delay to be used when CONFIG_MMC_CLKGATE is set */
	msm_host->mmc->clkgate_delay = SDHCI_MSM_MMC_CLK_GATE_DELAY;
#endif

	/* Set host capabilities */
	msm_host->mmc->caps |= msm_host->pdata->mmc_bus_width;
	msm_host->mmc->caps |= msm_host->pdata->caps;
	msm_host->mmc->caps |= MMC_CAP_AGGRESSIVE_PM;
	msm_host->mmc->caps |= MMC_CAP_WAIT_WHILE_BUSY;
	msm_host->mmc->caps2 |= msm_host->pdata->caps2;
	msm_host->mmc->caps2 |= MMC_CAP2_BOOTPART_NOACC;
	msm_host->mmc->caps2 |= MMC_CAP2_HS400_POST_TUNING;
	msm_host->mmc->caps2 |= MMC_CAP2_CLK_SCALE;
	msm_host->mmc->caps2 |= MMC_CAP2_SANITIZE;
	msm_host->mmc->caps2 |= MMC_CAP2_MAX_DISCARD_SIZE;
	msm_host->mmc->caps2 |= MMC_CAP2_SLEEP_AWAKE;
	msm_host->mmc->pm_caps |= MMC_PM_KEEP_POWER | MMC_PM_WAKE_SDIO_IRQ;

	if (msm_host->pdata->nonremovable)
		msm_host->mmc->caps |= MMC_CAP_NONREMOVABLE;

	if (msm_host->pdata->nonhotplug)
		msm_host->mmc->caps2 |= MMC_CAP2_NONHOTPLUG;

	msm_host->mmc->sdr104_wa = msm_host->pdata->sdr104_wa;

	/* Initialize ICE if present */
	if (msm_host->ice.pdev) {
		ret = sdhci_msm_ice_init(host);
		if (ret) {
			dev_err(&pdev->dev, "%s: SDHCi ICE init failed (%d)\n",
					mmc_hostname(host->mmc), ret);
			ret = -EINVAL;
			goto vreg_deinit;
		}
		host->is_crypto_en = true;
		msm_host->mmc->inlinecrypt_support = true;
		/* Packed commands cannot be encrypted/decrypted using ICE */
		msm_host->mmc->caps2 &= ~(MMC_CAP2_PACKED_WR |
				MMC_CAP2_PACKED_WR_CONTROL);
	}

	init_completion(&msm_host->pwr_irq_completion);

	if (gpio_is_valid(msm_host->pdata->status_gpio)) {
		/*
		 * Set up the card detect GPIO in active configuration before
		 * configuring it as an IRQ. Otherwise, it can be in some
		 * weird/inconsistent state resulting in flood of interrupts.
		 */
		sdhci_msm_setup_pins(msm_host->pdata, true);

		/*
		 * This delay is needed for stabilizing the card detect GPIO
		 * line after changing the pull configs.
		 */
		usleep_range(10000, 10500);
		ret = mmc_gpio_request_cd(msm_host->mmc,
				msm_host->pdata->status_gpio, 0);
		if (ret) {
			dev_err(&pdev->dev, "%s: Failed to request card detection IRQ %d\n",
					__func__, ret);
			goto vreg_deinit;
		}
	}

	if ((sdhci_readl(host, SDHCI_CAPABILITIES) & SDHCI_CAN_64BIT) &&
		(dma_supported(mmc_dev(host->mmc), DMA_BIT_MASK(64)))) {
		host->dma_mask = DMA_BIT_MASK(64);
		mmc_dev(host->mmc)->dma_mask = &host->dma_mask;
		mmc_dev(host->mmc)->coherent_dma_mask  = host->dma_mask;
	} else if (dma_supported(mmc_dev(host->mmc), DMA_BIT_MASK(32))) {
		host->dma_mask = DMA_BIT_MASK(32);
		mmc_dev(host->mmc)->dma_mask = &host->dma_mask;
		mmc_dev(host->mmc)->coherent_dma_mask  = host->dma_mask;
	} else {
		dev_err(&pdev->dev, "%s: Failed to set dma mask\n", __func__);
	}

	msm_host->pdata->sdiowakeup_irq = platform_get_irq_byname(pdev,
							  "sdiowakeup_irq");
	if (sdhci_is_valid_gpio_wakeup_int(msm_host)) {
		dev_info(&pdev->dev, "%s: sdiowakeup_irq = %d\n", __func__,
				msm_host->pdata->sdiowakeup_irq);
		msm_host->is_sdiowakeup_enabled = true;
		ret = request_irq(msm_host->pdata->sdiowakeup_irq,
				  sdhci_msm_sdiowakeup_irq,
				  IRQF_SHARED | IRQF_TRIGGER_HIGH,
				  "sdhci-msm sdiowakeup", host);
		if (ret) {
			dev_err(&pdev->dev, "%s: request sdiowakeup IRQ %d: failed: %d\n",
				__func__, msm_host->pdata->sdiowakeup_irq, ret);
			msm_host->pdata->sdiowakeup_irq = -1;
			msm_host->is_sdiowakeup_enabled = false;
			goto vreg_deinit;
		} else {
			spin_lock_irqsave(&host->lock, flags);
			sdhci_msm_cfg_sdiowakeup_gpio_irq(host, false);
			msm_host->sdio_pending_processing = false;
			spin_unlock_irqrestore(&host->lock, flags);
		}
	}

	sdhci_msm_cmdq_init(host, pdev);
	ret = sdhci_add_host(host);
	if (ret) {
		dev_err(&pdev->dev, "Add host failed (%d)\n", ret);
		goto vreg_deinit;
	}

	/*
	 * To avoid polling and to avoid this R1b command conversion
	 * to R1 command if the requested busy timeout > host's max
	 * busy timeout in case of sanitize, erase or any R1b command
	 */
	host->mmc->max_busy_timeout = 0;

	msm_host->pltfm_init_done = true;

	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);
	pm_runtime_set_autosuspend_delay(&pdev->dev, MSM_AUTOSUSPEND_DELAY_MS);
	pm_runtime_use_autosuspend(&pdev->dev);

	msm_host->msm_bus_vote.max_bus_bw.show = show_sdhci_max_bus_bw;
	msm_host->msm_bus_vote.max_bus_bw.store = store_sdhci_max_bus_bw;
	sysfs_attr_init(&msm_host->msm_bus_vote.max_bus_bw.attr);
	msm_host->msm_bus_vote.max_bus_bw.attr.name = "max_bus_bw";
	msm_host->msm_bus_vote.max_bus_bw.attr.mode = 0644;
	ret = device_create_file(&pdev->dev,
			&msm_host->msm_bus_vote.max_bus_bw);
	if (ret)
		goto remove_host;

	if (!gpio_is_valid(msm_host->pdata->status_gpio)) {
		msm_host->polling.show = show_polling;
		msm_host->polling.store = store_polling;
		sysfs_attr_init(&msm_host->polling.attr);
		msm_host->polling.attr.name = "polling";
		msm_host->polling.attr.mode = 0644;
		ret = device_create_file(&pdev->dev, &msm_host->polling);
		if (ret)
			goto remove_max_bus_bw_file;
	}

	msm_host->auto_cmd21_attr.show = show_auto_cmd21;
	msm_host->auto_cmd21_attr.store = store_auto_cmd21;
	sysfs_attr_init(&msm_host->auto_cmd21_attr.attr);
	msm_host->auto_cmd21_attr.attr.name = "enable_auto_cmd21";
	msm_host->auto_cmd21_attr.attr.mode = 0644;
	ret = device_create_file(&pdev->dev, &msm_host->auto_cmd21_attr);
	if (ret) {
		pr_err("%s: %s: failed creating auto-cmd21 attr: %d\n",
		       mmc_hostname(host->mmc), __func__, ret);
		device_remove_file(&pdev->dev, &msm_host->auto_cmd21_attr);
	}
	if (sdhci_msm_is_bootdevice(&pdev->dev))
		mmc_flush_detect_work(host->mmc);

	/* Successful initialization */
	goto out;

remove_max_bus_bw_file:
	device_remove_file(&pdev->dev, &msm_host->msm_bus_vote.max_bus_bw);
remove_host:
	dead = (readl_relaxed(host->ioaddr + SDHCI_INT_STATUS) == 0xffffffff);
	pm_runtime_disable(&pdev->dev);
	sdhci_remove_host(host, dead);
vreg_deinit:
	sdhci_msm_vreg_init(&pdev->dev, msm_host->pdata, false);
bus_unregister:
	if (msm_host->msm_bus_vote.client_handle)
		sdhci_msm_bus_cancel_work_and_set_vote(host, 0);
	sdhci_msm_bus_unregister(msm_host);
sleep_clk_disable:
	if (!IS_ERR(msm_host->sleep_clk))
		clk_disable_unprepare(msm_host->sleep_clk);
ff_clk_disable:
	if (!IS_ERR(msm_host->ff_clk))
		clk_disable_unprepare(msm_host->ff_clk);
clk_disable:
	if (!IS_ERR(msm_host->clk))
		clk_disable_unprepare(msm_host->clk);
bus_aggr_clk_disable:
	if (!IS_ERR(msm_host->bus_aggr_clk))
		clk_disable_unprepare(msm_host->bus_aggr_clk);
pclk_disable:
	if (!IS_ERR(msm_host->pclk))
		clk_disable_unprepare(msm_host->pclk);
bus_clk_disable:
	if (!IS_ERR_OR_NULL(msm_host->bus_clk))
		clk_disable_unprepare(msm_host->bus_clk);
pltfm_free:
	sdhci_pltfm_free(pdev);
out_host_free:
	devm_kfree(&pdev->dev, msm_host);
out:
	pr_debug("%s: Exit %s\n", dev_name(&pdev->dev), __func__);
	return ret;
}

static int sdhci_msm_remove(struct platform_device *pdev)
{
	struct sdhci_host *host = platform_get_drvdata(pdev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;
	struct sdhci_msm_pltfm_data *pdata = msm_host->pdata;
	int nr_groups = msm_host->pdata->pm_qos_data.cpu_group_map.nr_groups;
	int i;
	int dead = (readl_relaxed(host->ioaddr + SDHCI_INT_STATUS) ==
			0xffffffff);

	pr_debug("%s: %s Enter\n", dev_name(&pdev->dev), __func__);
	if (!gpio_is_valid(msm_host->pdata->status_gpio))
		device_remove_file(&pdev->dev, &msm_host->polling);

	device_remove_file(&pdev->dev, &msm_host->auto_cmd21_attr);
	device_remove_file(&pdev->dev, &msm_host->msm_bus_vote.max_bus_bw);
	pm_runtime_disable(&pdev->dev);

	if (msm_host->pm_qos_group_enable) {
		struct sdhci_msm_pm_qos_group *group;

		for (i = 0; i < nr_groups; i++)
			cancel_delayed_work_sync(
					&msm_host->pm_qos[i].unvote_work);

		device_remove_file(&msm_host->pdev->dev,
			&msm_host->pm_qos_group_enable_attr);
		device_remove_file(&msm_host->pdev->dev,
			&msm_host->pm_qos_group_status_attr);

		for (i = 0; i < nr_groups; i++) {
			group = &msm_host->pm_qos[i];
			pm_qos_remove_request(&group->req);
		}
	}

	if (msm_host->pm_qos_irq.enabled) {
		cancel_delayed_work_sync(&msm_host->pm_qos_irq.unvote_work);
		device_remove_file(&pdev->dev,
				&msm_host->pm_qos_irq.enable_attr);
		device_remove_file(&pdev->dev,
				&msm_host->pm_qos_irq.status_attr);
		pm_qos_remove_request(&msm_host->pm_qos_irq.req);
	}

	if (msm_host->pm_qos_wq)
		destroy_workqueue(msm_host->pm_qos_wq);

	sdhci_remove_host(host, dead);

	sdhci_msm_vreg_init(&pdev->dev, msm_host->pdata, false);

	sdhci_msm_setup_pins(pdata, true);
	sdhci_msm_setup_pins(pdata, false);

	if (msm_host->msm_bus_vote.client_handle) {
		sdhci_msm_bus_cancel_work_and_set_vote(host, 0);
		sdhci_msm_bus_unregister(msm_host);
	}

	sdhci_pltfm_free(pdev);

	return 0;
}

#ifdef CONFIG_PM
static int sdhci_msm_cfg_sdio_wakeup(struct sdhci_host *host, bool enable)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;
	unsigned long flags;
	int ret = 0;

	if (!(host->mmc->card && mmc_card_sdio(host->mmc->card) &&
	      sdhci_is_valid_gpio_wakeup_int(msm_host) &&
	      mmc_card_wake_sdio_irq(host->mmc))) {
		msm_host->sdio_pending_processing = false;
		return 1;
	}

	spin_lock_irqsave(&host->lock, flags);
	if (enable) {
		/* configure DAT1 gpio if applicable */
		if (sdhci_is_valid_gpio_wakeup_int(msm_host)) {
			msm_host->sdio_pending_processing = false;
			ret = enable_irq_wake(msm_host->pdata->sdiowakeup_irq);
			if (!ret)
				sdhci_msm_cfg_sdiowakeup_gpio_irq(host, true);
			goto out;
		} else {
			pr_err("%s: sdiowakeup_irq(%d) invalid\n",
					mmc_hostname(host->mmc), enable);
		}
	} else {
		if (sdhci_is_valid_gpio_wakeup_int(msm_host)) {
			ret = disable_irq_wake(msm_host->pdata->sdiowakeup_irq);
			sdhci_msm_cfg_sdiowakeup_gpio_irq(host, false);
			msm_host->sdio_pending_processing = false;
		} else {
			pr_err("%s: sdiowakeup_irq(%d)invalid\n",
					mmc_hostname(host->mmc), enable);

		}
	}
out:
	if (ret)
		pr_err("%s: %s: %sable wakeup: failed: %d gpio: %d\n",
		       mmc_hostname(host->mmc), __func__, enable ? "en" : "dis",
		       ret, msm_host->pdata->sdiowakeup_irq);
	spin_unlock_irqrestore(&host->lock, flags);
	return ret;
}


static int sdhci_msm_runtime_suspend(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;
	ktime_t start = ktime_get();
	int ret;

	if (host->mmc->card && mmc_card_sdio(host->mmc->card))
		goto defer_disable_host_irq;

	sdhci_cfg_irq(host, false, true);

defer_disable_host_irq:
	disable_irq(msm_host->pwr_irq);

	/*
	 * Remove the vote immediately only if clocks are off in which
	 * case we might have queued work to remove vote but it may not
	 * be completed before runtime suspend or system suspend.
	 */
	if (!atomic_read(&msm_host->clks_on)) {
		if (msm_host->msm_bus_vote.client_handle)
			sdhci_msm_bus_cancel_work_and_set_vote(host, 0);
	}

	if (host->is_crypto_en) {
		ret = sdhci_msm_ice_suspend(host);
		if (ret < 0)
			pr_err("%s: failed to suspend crypto engine %d\n",
					mmc_hostname(host->mmc), ret);
	}
	trace_sdhci_msm_runtime_suspend(mmc_hostname(host->mmc), 0,
			ktime_to_us(ktime_sub(ktime_get(), start)));
	return 0;
}

static int sdhci_msm_runtime_resume(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;
	ktime_t start = ktime_get();
	int ret;

	if (host->is_crypto_en) {
		ret = sdhci_msm_enable_controller_clock(host);
		if (ret) {
			pr_err("%s: Failed to enable reqd clocks\n",
					mmc_hostname(host->mmc));
			goto skip_ice_resume;
		}
		ret = sdhci_msm_ice_resume(host);
		if (ret)
			pr_err("%s: failed to resume crypto engine %d\n",
					mmc_hostname(host->mmc), ret);
	}
skip_ice_resume:

	if (host->mmc->card && mmc_card_sdio(host->mmc->card))
		goto defer_enable_host_irq;

	sdhci_cfg_irq(host, true, true);

defer_enable_host_irq:
	enable_irq(msm_host->pwr_irq);

	trace_sdhci_msm_runtime_resume(mmc_hostname(host->mmc), 0,
			ktime_to_us(ktime_sub(ktime_get(), start)));
	return 0;
}

static int sdhci_msm_suspend(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;
	int ret = 0;
	int sdio_cfg = 0;
	ktime_t start = ktime_get();

	if (gpio_is_valid(msm_host->pdata->status_gpio) &&
			 (msm_host->mmc->slot.cd_irq >= 0))
		disable_irq(msm_host->mmc->slot.cd_irq);

	if (pm_runtime_suspended(dev)) {
		pr_debug("%s: %s: already runtime suspended\n",
		mmc_hostname(host->mmc), __func__);
		goto out;
	}
	ret = sdhci_msm_runtime_suspend(dev);
out:
	sdhci_msm_disable_controller_clock(host);
	if (host->mmc->card && mmc_card_sdio(host->mmc->card)) {
		sdio_cfg = sdhci_msm_cfg_sdio_wakeup(host, true);
		if (sdio_cfg)
			sdhci_cfg_irq(host, false, true);
	}

	trace_sdhci_msm_suspend(mmc_hostname(host->mmc), ret,
			ktime_to_us(ktime_sub(ktime_get(), start)));
	return ret;
}

static int sdhci_msm_resume(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;
	int ret = 0;
	int sdio_cfg = 0;
	ktime_t start = ktime_get();

	if (gpio_is_valid(msm_host->pdata->status_gpio) &&
			 (msm_host->mmc->slot.cd_irq >= 0))
		enable_irq(msm_host->mmc->slot.cd_irq);

	if (pm_runtime_suspended(dev)) {
		pr_debug("%s: %s: runtime suspended, defer system resume\n",
		mmc_hostname(host->mmc), __func__);
		goto out;
	}

	ret = sdhci_msm_runtime_resume(dev);
out:
	if (host->mmc->card && mmc_card_sdio(host->mmc->card)) {
		sdio_cfg = sdhci_msm_cfg_sdio_wakeup(host, false);
		if (sdio_cfg)
			sdhci_cfg_irq(host, true, true);
	}

	trace_sdhci_msm_resume(mmc_hostname(host->mmc), ret,
			ktime_to_us(ktime_sub(ktime_get(), start)));
	return ret;
}

static int sdhci_msm_suspend_noirq(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;
	int ret = 0;

	/*
	 * ksdioirqd may be running, hence retry
	 * suspend in case the clocks are ON
	 */
	if (atomic_read(&msm_host->clks_on)) {
		pr_warn("%s: %s: clock ON after suspend, aborting suspend\n",
			mmc_hostname(host->mmc), __func__);
		ret = -EAGAIN;
	}

	if (host->mmc->card && mmc_card_sdio(host->mmc->card))
		if (msm_host->sdio_pending_processing)
			ret = -EBUSY;

	return ret;
}

static int sdhci_msm_freeze(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct mmc_host *mmc = host->mmc;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;

	if (gpio_is_valid(msm_host->pdata->status_gpio) &&
			(msm_host->mmc->slot.cd_irq >= 0))
		mmc_gpiod_free_cd_irq(mmc);
	return 0;
}

static int sdhci_msm_restore(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct mmc_host *mmc = host->mmc;

	if (mmc->inlinecrypt_support)
		mmc->inlinecrypt_reset_needed = true;

	mmc_gpiod_restore_cd_irq(mmc);

	return 0;
}

static const struct dev_pm_ops sdhci_msm_pmops = {
	.suspend_late		= sdhci_msm_suspend,
	.resume_early		= sdhci_msm_resume,
	.runtime_suspend	= sdhci_msm_runtime_suspend,
	.runtime_resume		= sdhci_msm_runtime_resume,
	.suspend_noirq		= sdhci_msm_suspend_noirq,
	.freeze			= sdhci_msm_freeze,
	.restore		= sdhci_msm_restore,
	.thaw			= sdhci_msm_restore,
};

#define SDHCI_MSM_PMOPS (&sdhci_msm_pmops)

#else
#define SDHCI_MSM_PMOPS NULL
#endif
static const struct of_device_id sdhci_msm_dt_match[] = {
	{.compatible = "qcom,sdhci-msm"},
	{.compatible = "qcom,sdhci-msm-v5"},
	{},
};
MODULE_DEVICE_TABLE(of, sdhci_msm_dt_match);

static struct platform_driver sdhci_msm_driver = {
	.probe		= sdhci_msm_probe,
	.remove		= sdhci_msm_remove,
	.driver		= {
		.name	= "sdhci_msm",
		.owner	= THIS_MODULE,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		.of_match_table = sdhci_msm_dt_match,
		.pm	= SDHCI_MSM_PMOPS,
	},
};

module_platform_driver(sdhci_msm_driver);

MODULE_DESCRIPTION("Qualcomm Technologies, Inc. Secure Digital Host Controller Interface driver");
MODULE_LICENSE("GPL v2");
