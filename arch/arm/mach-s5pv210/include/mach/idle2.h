/* arch/arm/mach-s5pv210/include/mach/idle2.h
 *
 * Copyright (c) Samsung Electronics Co. Ltd
 * Copyright (c) 2012 Will Tisdale - <willtisdale@gmail.com>
 *
 * S5PV210 - IDLE2 functions
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/


#include <mach/regs-gpio.h>
#include <mach/cpuidle.h>
#include <mach/power-domain.h>
#include <mach/gpio.h>
#include <mach/gpio-herring.h>
#include <linux/cpufreq.h>
#include <linux/interrupt.h>

#define MAX_CHK_DEV			0xf
#define IDLE2_FREQ			(800 * 1000) /* Use 800MHz when entering idle2 */
#define DISABLE_FURTHER_CPUFREQ 	0x10
#define ENABLE_FURTHER_CPUFREQ 		0x20

/*
 * For saving & restoring VIC register before entering
 * idle2 mode
 */
static dma_addr_t phy_regs_save;
static unsigned long vic_regs[4];
static unsigned long *regs_save;

/* Specific device list for checking before entering
 * idle2 mode
 **/
struct check_device_op {
	void __iomem		*base;
	struct platform_device	*pdev;
};

/* Array of checking devices list */
struct check_device_op chk_dev_op[] = {
#if defined(CONFIG_S3C_DEV_HSMMC)
	{.base = 0, .pdev = &s3c_device_hsmmc0},
#endif
#if defined(CONFIG_S3C_DEV_HSMMC1)
	{.base = 0, .pdev = &s3c_device_hsmmc1},
#endif
#if defined(CONFIG_S3C_DEV_HSMMC2)
	{.base = 0, .pdev = &s3c_device_hsmmc2},
#endif
#if defined(CONFIG_S3C_DEV_HSMMC3)
	{.base = 0, .pdev = &s3c_device_hsmmc3},
#endif
	{.base = 0, .pdev = &s5p_device_onenand},
	{.base = 0, .pdev = NULL},
};

#define S3C_HSMMC_PRNSTS	(0x24)
#define S3C_HSMMC_CLKCON	(0x2c)
#define S3C_HSMMC_CMD_INHIBIT	0x00000001
#define S3C_HSMMC_DATA_INHIBIT	0x00000002
#define S3C_HSMMC_CLOCK_CARD_EN	0x0004


/* If SD/MMC interface is working: return = true */
inline static bool check_sdmmc_op(unsigned int ch)
{
	unsigned int reg1, reg2;
	void __iomem *base_addr;

	if (unlikely(ch > 2)) {
		printk(KERN_ERR "Invalid ch[%d] for SD/MMC \n", ch);
		return false;
	}

	base_addr = chk_dev_op[ch].base;
	/* Check CMDINHDAT[1] and CMDINHCMD [0] */
	reg1 = readl(base_addr + S3C_HSMMC_PRNSTS);
	/* Check CLKCON [2]: ENSDCLK */
	reg2 = readl(base_addr + S3C_HSMMC_CLKCON);

	if (unlikely((reg1 & (S3C_HSMMC_CMD_INHIBIT | S3C_HSMMC_DATA_INHIBIT)) ||
			(reg2 & (S3C_HSMMC_CLOCK_CARD_EN))))
		return true;
	else
		return false;
}

/* Check all sdmmc controller */
inline static bool loop_sdmmc_check(void)
{
	unsigned int iter;

	for (iter = 0; iter < 3; iter++) {
		if (unlikely(check_sdmmc_op(iter))) {
#ifdef CONFIG_S5P_IDLE2_DEBUG
			printk(KERN_INFO "%s: %d returns true\n", __func__, iter);
#endif
			return true;
		}
	}
	return false;
}

/* Check onenand is working or not */

/* ONENAND_IF_STATUS(0xB060010C)
 * ORWB[0] = 	1b : busy
 * 		0b : Not busy
 **/
inline static bool check_onenand_op(void)
{
	unsigned int val;
	void __iomem *base_addr;

	base_addr = chk_dev_op[3].base;

	val = __raw_readl(base_addr + 0x0000010c);

	if (unlikely(val & 0x1)) {
#ifdef CONFIG_S5P_IDLE2_DEBUG
		printk(KERN_INFO "%s: check_onenand_op() returns true\n", __func__);
#endif
		return true;
	}
	return false;
}

inline static bool check_clock_gating(void)
{
	unsigned long val;

	/* check clock gating */
	val = __raw_readl(S5P_CLKGATE_IP0);
	if (unlikely(val & (S5P_CLKGATE_IP0_MDMA | S5P_CLKGATE_IP0_PDMA0
					| S5P_CLKGATE_IP0_G3D | S5P_CLKGATE_IP0_PDMA1))) {
#ifdef CONFIG_S5P_IDLE2_DEBUG
		printk(KERN_INFO "%s: S5P_CLKGATE_IP0 - DMA/3D active\n", __func__);
#endif
		return true;
	}

	val = __raw_readl(S5P_CLKGATE_IP3);
	if (unlikely(val & (S5P_CLKGATE_IP3_I2C0 | S5P_CLKGATE_IP3_I2C_HDMI_DDC
					| S5P_CLKGATE_IP3_I2C2))) {
#ifdef CONFIG_S5P_IDLE2_DEBUG
		printk(KERN_INFO "%s: S5P_CLKGATE_IP3 - i2c / HDMI active\n", __func__);
#endif
		return true;
	}

	return false;
}

extern void i2sdma_getpos(dma_addr_t *src);
inline static bool check_idmapos(void)
{
	dma_addr_t src;
	i2sdma_getpos(&src);
	src = src & 0x3FFF;
	src = 0x4000 - src;
	if (unlikely(src < 0x150)) {
#ifdef CONFIG_S5P_IDLE2_DEBUG
		printk(KERN_INFO "%s: returns true\n", __func__);
#endif
		return true;
	}
	else
		return false;
}

extern unsigned int get_rtc_cnt(void);
inline static bool check_rtcint(void)
{
	unsigned int current_cnt = get_rtc_cnt();
	if (unlikely(current_cnt < 0x40)){
#ifdef CONFIG_S5P_IDLE2_DEBUG
		printk(KERN_INFO "%s: returns true\n", __func__);
#endif
		return true;
	}
	else
		return false;
}

inline static bool enter_idle2_check(void)
{
	if (unlikely(loop_sdmmc_check() || check_clock_gating() || check_idmapos() || check_rtcint() || check_onenand_op()))
		return true;
	else
		return false;

}

inline static void idle2_set_cpufreq_lock(bool flag)
{
	int ret;
	if (flag) {
		preempt_enable();
		local_irq_enable();
		ret = cpufreq_driver_target(cpufreq_cpu_get(0), IDLE2_FREQ,
				DISABLE_FURTHER_CPUFREQ);
		if (ret < 0)
			printk(KERN_WARNING "%s: Error %d locking CPUfreq\n", __func__, ret);
		else
			printk(KERN_INFO "%s: CPUfreq locked to 800MHz\n", __func__);
		local_irq_disable();
		preempt_disable();
	} else {
		preempt_enable();
		local_irq_enable();
		ret = cpufreq_driver_target(cpufreq_cpu_get(0), IDLE2_FREQ,
				ENABLE_FURTHER_CPUFREQ);
		if (ret < 0)
			printk(KERN_WARNING "%s: Error %d unlocking CPUfreq\n", __func__, ret);
		else
			printk(KERN_INFO "%s: CPUfreq unlocked from 800MHz\n", __func__);
		local_irq_disable();
		preempt_disable();
	}
}

/*
 * Before entering, idle2 mode GPIO Power Down Mode
 * Configuration register has to be set with same state
 * in Normal Mode
 */
#define GPIO_OFFSET		0x20
#define GPIO_CON_PDN_OFFSET	0x10
#define GPIO_PUD_PDN_OFFSET	0x14
#define GPIO_PUD_OFFSET		0x08

inline static void s5p_gpio_pdn_conf(void)
{
	void __iomem *gpio_base = S5PV210_GPA0_BASE;
	unsigned int val;

	do {
		/* Keep the previous state in idle2 mode */
		__raw_writel(0xffff, gpio_base + GPIO_CON_PDN_OFFSET);

		/* Pull up-down state in idle2 is same as normal */
		val = __raw_readl(gpio_base + GPIO_PUD_OFFSET);
		__raw_writel(val, gpio_base + GPIO_PUD_PDN_OFFSET);

		gpio_base += GPIO_OFFSET;

	} while (gpio_base <= S5PV210_MP28_BASE);
}

inline static void s5p_enter_idle2(void)
{
	unsigned long tmp;
	unsigned long save_eint_mask;

	/* store the physical address of the register recovery block */
	__raw_writel(phy_regs_save, S5P_INFORM2);

	/* ensure at least INFORM0 has the resume address */
	__raw_writel(virt_to_phys(s5p_idle2_resume), S5P_INFORM0);

	/* Save current VIC_INT_ENABLE register*/
	vic_regs[0] = __raw_readl(S5P_VIC0REG(VIC_INT_ENABLE));
	vic_regs[1] = __raw_readl(S5P_VIC1REG(VIC_INT_ENABLE));
	vic_regs[2] = __raw_readl(S5P_VIC2REG(VIC_INT_ENABLE));
	vic_regs[3] = __raw_readl(S5P_VIC3REG(VIC_INT_ENABLE));

	/* Disable all interrupt through VIC */
	__raw_writel(0xffffffff, S5P_VIC0REG(VIC_INT_ENABLE_CLEAR));
	__raw_writel(0xffffffff, S5P_VIC1REG(VIC_INT_ENABLE_CLEAR));
	__raw_writel(0xffffffff, S5P_VIC2REG(VIC_INT_ENABLE_CLEAR));
	__raw_writel(0xffffffff, S5P_VIC3REG(VIC_INT_ENABLE_CLEAR));

	/* GPIO Power Down Control */
	s5p_gpio_pdn_conf();
        save_eint_mask = __raw_readl(S5P_EINT_WAKEUP_MASK);
        __raw_writel(0xFFFFFFFF, S5P_EINT_WAKEUP_MASK);

	/* Wakeup source configuration for idle2 */
	tmp = __raw_readl(S5P_WAKEUP_MASK);

	tmp |= 0xffff;
        // Key interrupt mask idma & RTC tic only
        tmp &= ~((1<<2) | (1<<13));

	__raw_writel(tmp, S5P_WAKEUP_MASK);

	/* Clear wakeup status register */
	tmp = __raw_readl(S5P_WAKEUP_STAT);
	__raw_writel(tmp, S5P_WAKEUP_STAT);

	/* IDLE config register set */
	/* TOP Memory retention off */
	/* TOP Memory LP mode       */
	/* ARM_L2_Cacheret on       */
	tmp = __raw_readl(S5P_IDLE_CFG);
	tmp &= ~(0x3f << 26);
	tmp |= ((1<<30) | (1<<28) | (1<<26) | (1<<0));
	__raw_writel(tmp, S5P_IDLE_CFG);

	/* Power mode Config setting */
	tmp = __raw_readl(S5P_PWR_CFG);
	tmp &= S5P_CFG_WFI_CLEAN;
	tmp |= S5P_CFG_WFI_IDLE;
	__raw_writel(tmp, S5P_PWR_CFG);

	/* To check VIC Status register before enter idle2 mode */
	if (unlikely(__raw_readl(S5P_VIC2REG(VIC_RAW_STATUS)) & 0x10000)) {
#ifdef CONFIG_S5P_IDLE2_DEBUG
		printk(KERN_WARNING "%s: VIC interrupt active, bailing!\n", __func__);
#endif
		goto skipped_idle2;
	}

	/* SYSCON_INT_DISABLE */
	tmp = __raw_readl(S5P_OTHERS);
	tmp |= S5P_OTHER_SYSC_INTOFF;
	__raw_writel(tmp, S5P_OTHERS);

	/* Entering idle2 mode with WFI instruction */
	if (likely(s5p_idle2_save(regs_save) == 0)) {
#ifdef CONFIG_S5P_IDLE2_DEBUG
		printk(KERN_INFO "*** Entering IDLE2 TOP OFF mode\n");
#endif
		flush_cache_all();
		cpu_do_idle();
	}
skipped_idle2:
	__raw_writel(save_eint_mask, S5P_EINT_WAKEUP_MASK);

	tmp = __raw_readl(S5P_IDLE_CFG);
	tmp &= ~((3<<30)|(3<<28)|(3<<26)|(1<<0));
	tmp |= ((2<<30)|(2<<28));
	__raw_writel(tmp, S5P_IDLE_CFG);

	/* Power mode Config setting */
	tmp = __raw_readl(S5P_PWR_CFG);
	tmp &= S5P_CFG_WFI_CLEAN;
	__raw_writel(tmp, S5P_PWR_CFG);

	/* Release retention GPIO/MMC/UART IO */
	tmp = __raw_readl(S5P_OTHERS);
	tmp |= ((1<<31) | (1<<30) | (1<<29) | (1<<28));
	__raw_writel(tmp, S5P_OTHERS);

	__raw_writel(vic_regs[0], S5P_VIC0REG(VIC_INT_ENABLE));
	__raw_writel(vic_regs[1], S5P_VIC1REG(VIC_INT_ENABLE));
	__raw_writel(vic_regs[2], S5P_VIC2REG(VIC_INT_ENABLE));
	__raw_writel(vic_regs[3], S5P_VIC3REG(VIC_INT_ENABLE));
}

inline static void s5p_enter_idle2_topon(void)
{
	unsigned long tmp;
	unsigned long save_eint_mask;

	/* store the physical address of the register recovery block */
	__raw_writel(phy_regs_save, S5P_INFORM2);

	/* ensure at least INFORM0 has the resume address */
	__raw_writel(virt_to_phys(s5p_idle2_resume), S5P_INFORM0);

	/* Save current VIC_INT_ENABLE register*/
	vic_regs[0] = __raw_readl(S5P_VIC0REG(VIC_INT_ENABLE));
	vic_regs[1] = __raw_readl(S5P_VIC1REG(VIC_INT_ENABLE));
	vic_regs[2] = __raw_readl(S5P_VIC2REG(VIC_INT_ENABLE));
	vic_regs[3] = __raw_readl(S5P_VIC3REG(VIC_INT_ENABLE));

	/* Disable all interrupt through VIC */
	__raw_writel(0xffffffff, S5P_VIC0REG(VIC_INT_ENABLE_CLEAR));
	__raw_writel(0xffffffff, S5P_VIC1REG(VIC_INT_ENABLE_CLEAR));
	__raw_writel(0xffffffff, S5P_VIC2REG(VIC_INT_ENABLE_CLEAR));
	__raw_writel(0xffffffff, S5P_VIC3REG(VIC_INT_ENABLE_CLEAR));

	/* GPIO Power Down Control */
        save_eint_mask = __raw_readl(S5P_EINT_WAKEUP_MASK);
        __raw_writel(0xFFFFFFFF, S5P_EINT_WAKEUP_MASK);

	/* Wakeup source configuration for idle2 */
	tmp = __raw_readl(S5P_WAKEUP_MASK);

	tmp |= 0xffff;
        // Key interrupt mask idma & RTC tic only
        tmp &= ~((1<<2) | (1<<13));

	__raw_writel(tmp, S5P_WAKEUP_MASK);

	/* Clear wakeup status register */
	tmp = __raw_readl(S5P_WAKEUP_STAT);
	__raw_writel(tmp, S5P_WAKEUP_STAT);

	/* IDLE config register set */
	/* TOP Memory retention on */
	/* TOP Memory LP mode off */
	/* ARM_L2_Cacheret on */
	tmp = __raw_readl(S5P_IDLE_CFG);
	tmp &= ~(0x3f << 26);
	tmp |= ((2<<30) | (2<<28) | (1<<26) | (1<<0));
	__raw_writel(tmp, S5P_IDLE_CFG);

	/* Power mode Config setting */
	tmp = __raw_readl(S5P_PWR_CFG);
	tmp &= S5P_CFG_WFI_CLEAN;
	tmp |= S5P_CFG_WFI_IDLE;
	__raw_writel(tmp, S5P_PWR_CFG);

	/* To check VIC Status register before enter idle2 mode */
	if (unlikely(__raw_readl(S5P_VIC2REG(VIC_RAW_STATUS)) & 0x10000)) {
#ifdef CONFIG_S5P_IDLE2_DEBUG
		printk(KERN_WARNING "%s: VIC interrupt active, bailing!\n", __func__);
#endif
		goto skipped_idle2;
	}

	/* SYSCON_INT_DISABLE */
	tmp = __raw_readl(S5P_OTHERS);
	tmp |= S5P_OTHER_SYSC_INTOFF;
	__raw_writel(tmp, S5P_OTHERS);

	/* Entering idle2 mode with WFI instruction */
	if (likely(s5p_idle2_save(regs_save)) == 0) {
#ifdef CONFIG_S5P_IDLE2_DEBUG
		printk(KERN_INFO "*** Entering IDLE2 TOP ON mode\n");
#endif
		flush_cache_all();
		cpu_do_idle();
	}
skipped_idle2:
	__raw_writel(save_eint_mask, S5P_EINT_WAKEUP_MASK);

	tmp = __raw_readl(S5P_IDLE_CFG);
	tmp &= ~((3<<30)|(3<<28)|(3<<26)|(1<<0));
	tmp |= ((2<<30)|(2<<28));
	__raw_writel(tmp, S5P_IDLE_CFG);

	/* Power mode Config setting */
	tmp = __raw_readl(S5P_PWR_CFG);
	tmp &= S5P_CFG_WFI_CLEAN;
	__raw_writel(tmp, S5P_PWR_CFG);

	__raw_writel(vic_regs[0], S5P_VIC0REG(VIC_INT_ENABLE));
	__raw_writel(vic_regs[1], S5P_VIC1REG(VIC_INT_ENABLE));
	__raw_writel(vic_regs[2], S5P_VIC2REG(VIC_INT_ENABLE));
	__raw_writel(vic_regs[3], S5P_VIC3REG(VIC_INT_ENABLE));
}

int s5p_init_remap(void)
{
	int i = 0;
	struct platform_device *pdev;
	struct resource *res;
	/* Allocate memory region to access IP's directly */
	for (i = 0 ; i < MAX_CHK_DEV ; i++) {

		pdev = chk_dev_op[i].pdev;

		if (pdev == NULL)
			break;

		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		if (!res) {
			printk(KERN_ERR "failed to get io memory region\n");
			return -EINVAL;
		}
		/* ioremap for register block */
		if(pdev == &s5p_device_onenand)
			chk_dev_op[i].base = ioremap(res->start+0x00600000, 4096);
		else
			chk_dev_op[i].base = ioremap(res->start, 4096);

		if (!chk_dev_op[i].base) {
			printk(KERN_ERR "failed to remap io region\n");
			return -EINVAL;
		}
	}
	return 0;
}
