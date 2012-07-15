/*
 * arch/arm/mach-s5pv210/cpuidle.c
 *
 * Copyright (c) Samsung Electronics Co. Ltd
 * Copyright (c) 2012 - Will Tisdale <willtisdale@gmail.com>
 *
 * CPU idle driver for S5PV210
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <asm/proc-fns.h>
#include <asm/cacheflush.h>
#include <linux/dma-mapping.h>

#include <mach/map.h>
#include <mach/regs-irq.h>
#include <mach/regs-clock.h>
#include <plat/pm.h>
#include <plat/devs.h>
#include <linux/cpuidle.h>

#ifdef CONFIG_S5P_IDLE2
#include <mach/idle2.h>
#include <linux/suspend.h>
#include <linux/workqueue.h>
#endif /* CONFIG_S5P_IDLE2 */


#ifdef CONFIG_S5P_IDLE2
static bool idle2_disabled __read_mostly = false;
static bool idle2_disabled_by_suspend __read_mostly = false;
static bool work_initialised __read_mostly = false;
static bool idle2_requested __read_mostly = false;
static bool external_active __read_mostly;
static bool inactive_pending;
static bool enable_pending;
static bool earlysuspend_active __read_mostly = false;
static bool idle2_cpufreq_lock = false;
static bool needs_topon = false;
static bool topon_cancel_pending;
#endif /* CONFIG_S5P_IDLE2 */


inline static void s5p_enter_idle(void)
{
	unsigned long tmp;

	tmp = __raw_readl(S5P_IDLE_CFG);
	tmp &= ~((3<<30)|(3<<28)|(1<<0));
	tmp |= ((2<<30)|(2<<28));
	__raw_writel(tmp, S5P_IDLE_CFG);

	tmp = __raw_readl(S5P_PWR_CFG);
	tmp &= S5P_CFG_WFI_CLEAN;
	__raw_writel(tmp, S5P_PWR_CFG);

	cpu_do_idle();
}

/* Actual code that puts the SoC in different idle states */
inline static int s5p_enter_idle_normal(struct cpuidle_device *device,
				struct cpuidle_state *state)
{
	struct timeval before, after;
	int idle_time;

	local_irq_disable();
	do_gettimeofday(&before);

	s5p_enter_idle();

	do_gettimeofday(&after);
	local_irq_enable();
	idle_time = (after.tv_sec - before.tv_sec) * USEC_PER_SEC +
			(after.tv_usec - before.tv_usec);
	return idle_time;
}

static DEFINE_PER_CPU(struct cpuidle_device, s5p_cpuidle_device);

static struct cpuidle_driver s5p_idle_driver = {
	.name =         "s5p_idle",
	.owner =        THIS_MODULE,
};

#ifdef CONFIG_S5P_IDLE2
/* Actual code that puts the SoC in different idle states */
inline static int s5p_enter_idle_idle2_topoff(struct cpuidle_device *device,
				struct cpuidle_state *state)
{
	struct timeval before, after;
	int idle_time;

	if (enter_idle2_check()) {
#ifdef CONFIG_S5P_IDLE2_DEBUG
		printk("%s: Falling back to IDLE state\n", __func__);
#endif
		s5p_enter_idle_normal(device, state);
		return 0;
	}

	local_irq_disable();
	do_gettimeofday(&before);

	s5p_enter_idle2();
	do_gettimeofday(&after);
	local_irq_enable();
	idle_time = (after.tv_sec - before.tv_sec) * USEC_PER_SEC +
			(after.tv_usec - before.tv_usec);
	return idle_time;
}

inline static int s5p_enter_idle_idle2_topon(struct cpuidle_device *device,
				struct cpuidle_state *state)
{
	struct timeval before, after;
	int idle_time;

	if (enter_idle2_check()) {
#ifdef CONFIG_S5P_IDLE2_DEBUG
		printk("%s: Falling back to IDLE state\n", __func__);
#endif
		s5p_enter_idle_normal(device, state);
		return 0;
	}

	local_irq_disable();
	do_gettimeofday(&before);

	s5p_enter_idle2_topon();
	do_gettimeofday(&after);
	local_irq_enable();
	idle_time = (after.tv_sec - before.tv_sec) * USEC_PER_SEC +
			(after.tv_usec - before.tv_usec);
	return idle_time;
}

void earlysuspend_active_fn(bool flag)
{
	if (flag)
		earlysuspend_active = true;
	else
		earlysuspend_active = false;
	printk(KERN_DEBUG "earlysuspend_active: %d\n", earlysuspend_active);
}

static void idle2_requested_fn(bool flag)
{
	if (flag)
		idle2_requested = true;
	else
		idle2_requested = false;
	printk(KERN_DEBUG "idle2_requested: %d\n", idle2_requested);
}

static void external_active_fn(bool flag)
{
	if (flag)
		external_active = true;
	else
		external_active = false;
	printk(KERN_DEBUG "external_active: %d\n", external_active);
}

static void needs_topon_fn(bool flag)
{
	if (flag)
		needs_topon = true;
	else
		needs_topon = false;
	printk(KERN_DEBUG "needs_topon: %d\n", needs_topon);
}

inline static int s5p_enter_idle_deep_topoff(struct cpuidle_device *device,
				struct cpuidle_state *state)
{
	if (unlikely(idle2_disabled || idle2_disabled_by_suspend))
		return s5p_enter_idle_normal(device, state);
	if (unlikely(needs_topon)) {
		return s5p_enter_idle_idle2_topon(device, state);
		printk(KERN_WARNING "%s: we shouldn't be here\n", __func__);
	}
	if (likely(idle2_requested && !external_active))
		return s5p_enter_idle_idle2_topoff(device, state);
	return s5p_enter_idle_normal(device, state);
}

inline static int s5p_enter_idle_deep_topon(struct cpuidle_device *device,
				struct cpuidle_state *state)
{
	if (unlikely(idle2_disabled || idle2_disabled_by_suspend))
		return s5p_enter_idle_normal(device, state);
	if (unlikely(!needs_topon)) {
		return s5p_enter_idle_idle2_topoff(device, state);
		printk(KERN_WARNING "%s: we shouldn't be here\n", __func__);
	}
	if (likely(idle2_requested && !external_active))
		return s5p_enter_idle_idle2_topon(device, state);
	return s5p_enter_idle_normal(device, state);
}

static struct workqueue_struct *idle2_wq;
struct work_struct idle2_disable_work;
struct delayed_work idle2_enable_work;
struct work_struct idle2_external_active_work;
struct delayed_work idle2_external_inactive_work;
struct work_struct idle2_enable_topon_work;
struct delayed_work idle2_cancel_topon_work;

static void idle2_enable_work_fn(struct work_struct *work)
{
	idle2_requested_fn(true);
}

static void idle2_disable_work_fn(struct work_struct *work)
{
	cancel_delayed_work_sync(&idle2_enable_work);
	idle2_requested_fn(false);
}

static void idle2_external_active_work_fn(struct work_struct *work)
{
	cancel_delayed_work_sync(&idle2_external_inactive_work);
	external_active_fn(true);
}

static void idle2_external_inactive_work_fn(struct work_struct *work)
{
	external_active_fn(false);
}

static void idle2_enable_topon_work_fn(struct work_struct *work)
{
	cancel_delayed_work_sync(&idle2_cancel_topon_work);
	needs_topon_fn(true);
}

static void idle2_cancel_topon_work_fn(struct work_struct *work)
{
	needs_topon_fn(false);
}

void idle2_enable(unsigned long delay)
{
	if (work_initialised && !enable_pending) {
		enable_pending = true;
		queue_delayed_work(idle2_wq, &idle2_enable_work, delay);
		printk(KERN_DEBUG "enable_pending: %d\n", enable_pending);
	}
}

void idle2_disable(void)
{
	if (work_initialised && (idle2_requested || enable_pending)) {
		queue_work(idle2_wq, &idle2_disable_work);
		enable_pending = false;
	}
}

void idle2_external_active(void)
{
	if (work_initialised && (!external_active || inactive_pending)) {
		queue_work(idle2_wq, &idle2_external_active_work);
		inactive_pending = false;
	}
}

void idle2_external_inactive(unsigned long delay)
{
	if (work_initialised && external_active && !inactive_pending) {
		inactive_pending = true;
		queue_delayed_work(idle2_wq, &idle2_external_inactive_work, delay);
		printk(KERN_DEBUG "inactive_pending: %d\n", inactive_pending);
	}
}

void idle2_needs_topon(void)
{
	if (work_initialised && (!needs_topon || topon_cancel_pending)) {
		queue_work(idle2_wq, &idle2_enable_topon_work);
		topon_cancel_pending = false;
	}
}

void idle2_cancel_topon(unsigned long delay)
{
	if (work_initialised && needs_topon && !topon_cancel_pending) {
		topon_cancel_pending = true;
		queue_delayed_work(idle2_wq, &idle2_cancel_topon_work, delay);
		printk(KERN_DEBUG "topon_cancel_pending: %d\n", topon_cancel_pending);
	}
}

static int idle2_disabled_set(const char *arg, const struct kernel_param *kp)
{
	int ret;
	printk(KERN_INFO "%s: %s\n", __func__, arg);
	return ret = param_set_bool(arg, kp);
}

static int idle2_disabled_get(char *buffer, const struct kernel_param *kp)
{
	return param_get_bool(buffer, kp);
}

static struct kernel_param_ops idle2_disabled_ops = {
	.set = idle2_disabled_set,
	.get = idle2_disabled_get,
};
module_param_cb(idle2_disabled, &idle2_disabled_ops, &idle2_disabled, 0644);

static int s5p_idle_prepare(struct cpuidle_device *device)
{
	if (!idle2_disabled && !external_active && idle2_requested && earlysuspend_active) {
		if (unlikely(needs_topon)) {
			device->states[2].flags &= ~CPUIDLE_FLAG_IGNORE;
			device->states[1].flags |= CPUIDLE_FLAG_IGNORE;
		} else {
			device->states[1].flags &= ~CPUIDLE_FLAG_IGNORE;
			device->states[2].flags |= CPUIDLE_FLAG_IGNORE;
		}
		if (unlikely(!idle2_cpufreq_lock)) {
			idle2_set_cpufreq_lock(true);
			idle2_cpufreq_lock = true;
		}
	} else {
		device->states[1].flags |= CPUIDLE_FLAG_IGNORE;
		device->states[2].flags |= CPUIDLE_FLAG_IGNORE;
		if (unlikely(idle2_cpufreq_lock)) {
			idle2_set_cpufreq_lock(false);
			idle2_cpufreq_lock = false;
		}
	}

	return 0;
}

static int idle2_pm_notify(struct notifier_block *nb,
	unsigned long event, void *dummy)
{
	if (event == PM_SUSPEND_PREPARE) {
		idle2_disabled_by_suspend = true;
		printk(KERN_INFO "%s: IDLE2 disabled\n", __func__);
	}
	else if (event == PM_POST_SUSPEND) {
		idle2_disabled_by_suspend = false;
		printk(KERN_INFO "%s: IDLE2 enabled\n", __func__);
	}
	return NOTIFY_OK;
}

static struct notifier_block idle2_pm_notifier = {
	.notifier_call = idle2_pm_notify,
};
#endif /* CONFIG_S5P_IDLE2 */


/* Initialize CPU idle by registering the idle states */
static int s5p_init_cpuidle(void)
{
	struct cpuidle_device *device;

#ifdef CONFIG_S5P_IDLE2
	idle2_wq = create_singlethread_workqueue("idle2_workqueue");
	BUG_ON(!idle2_wq);
	INIT_WORK(&idle2_disable_work, idle2_disable_work_fn);
	INIT_DELAYED_WORK(&idle2_enable_work, idle2_enable_work_fn);
	INIT_WORK(&idle2_external_active_work, idle2_external_active_work_fn);
	INIT_DELAYED_WORK(&idle2_external_inactive_work, idle2_external_inactive_work_fn);
	INIT_WORK(&idle2_enable_topon_work, idle2_enable_topon_work_fn);
	INIT_DELAYED_WORK(&idle2_cancel_topon_work, idle2_cancel_topon_work_fn);
	work_initialised = true;
#endif /* CONFIG_S5P_IDLE2 */

	cpuidle_register_driver(&s5p_idle_driver);

	device = &per_cpu(s5p_cpuidle_device, smp_processor_id());
	device->state_count = 0;

	/* Wait for interrupt state */
	device->states[0].enter = s5p_enter_idle_normal;
	device->states[0].exit_latency = 1;	/* uS */
	device->states[0].target_residency = 10000;
	device->states[0].flags = CPUIDLE_FLAG_TIME_VALID;
	strcpy(device->states[0].name, "IDLE");
	strcpy(device->states[0].desc, "ARM clock gating - WFI");
	device->state_count++;
	
#ifdef CONFIG_S5P_IDLE2
	/* Deep-Idle top OFF Wait for interrupt state */
	device->states[1].enter = s5p_enter_idle_deep_topoff;
	device->states[1].exit_latency = 400;	/* uS */
	device->states[1].target_residency = 2000;
	device->states[1].flags = CPUIDLE_FLAG_TIME_VALID |
					CPUIDLE_FLAG_CHECK_BM;
	strcpy(device->states[1].name, "IDLE2-TOPOFF");
	strcpy(device->states[1].desc, "ARM/TOP/SUB Power gating - WFI");
	device->state_count++;

	/* Deep-Idle top ON Wait for interrupt state */
	device->states[2].enter = s5p_enter_idle_deep_topon;
	device->states[2].exit_latency = 1;	/* uS */
	device->states[2].target_residency = 2000;
	device->states[2].flags = CPUIDLE_FLAG_TIME_VALID;
	strcpy(device->states[2].name, "IDLE2-TOPON");
	strcpy(device->states[2].desc, "ARM Power gating - WFI");
	device->state_count++;

	/*
	 * Device prepare isn't required when we are building with
	 * CONFIG_S5P_IDLE2 disabled as there is only one active state
	 * so there is nothing to prepare.
	 */
	device->prepare = s5p_idle_prepare;
#endif

	if (cpuidle_register_device(device)) {
		printk(KERN_ERR "s5p_init_cpuidle: Failed registering\n");
		BUG();
		return -EIO;
	}
#ifdef CONFIG_S5P_IDLE2
	regs_save = dma_alloc_coherent(NULL, 4096, &phy_regs_save, GFP_KERNEL);
	if (regs_save == NULL) {
		printk(KERN_ERR "DMA alloc error\n");
		BUG();
		return -ENOMEM;
	}
	printk(KERN_INFO "cpuidle: IDLE2 support enabled - version 0.210 by <willtisdale@gmail.com>\n");

	register_pm_notifier(&idle2_pm_notifier);

	return s5p_init_remap();
#else /* CONFIG_S5P_IDLE2 */
	return 0;
#endif
}

device_initcall(s5p_init_cpuidle);
