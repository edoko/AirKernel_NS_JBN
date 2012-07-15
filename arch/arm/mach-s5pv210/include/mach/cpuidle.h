/* arch/arm/mach-s5pv210/include/mach/cpuidle.h
 *
 * Copyright (c) 2010 Samsung Electronics - Jaecheol Lee <jc.lee@samsung>
 * Copyright (c) 2012 Will Tisdale - <willtisdale@gmail.com>
 *
 * S5PV210 - CPUIDLE support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifdef CONFIG_S5P_IDLE2
extern int s5p_idle2_save(unsigned long *saveblk);
extern void s5p_idle2_resume(void);
extern void s5p_idle2(void);
extern void idle2_enable(unsigned long delay);
extern void idle2_disable(void);
extern void idle2_external_active(void);
extern void idle2_external_inactive(unsigned long delay);
extern void earlysuspend_active_fn(bool flag);
extern void idle2_cancel_topon(unsigned long delay);
extern void idle2_needs_topon(void);
#endif