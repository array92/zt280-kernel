/**
 * linux/include/i2c/touch_common.h
 *
 * Copyright (C) 2007-2008 Avionic Design Development GmbH
 * Copyright (C) 2008-2009 Avionic Design GmbH
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file COPYING in the main directory of this archive for
 * more details.
 *
 * Written by x
 */

#ifndef _LINUX_TOUCH_H_
#define _LINUX_TOUCH_H_

/**
 * struct touch_platform_data - platform-specific TOUCH data
 * @init_irq:          initialize interrupt
 * @get_irq_level:     obtain current interrupt level
 */
struct touch_platform_data {
       int (*init_irq)(void);
       int (*get_irq_level)(void);
       int (*enable)(int en);
       int abs_xmin;
       int abs_xmax;
       int abs_ymin;
       int abs_ymax;
       int abs_pmin;
       int abs_pmax;

#if defined(CONFIG_TOUCHSCREEN_TS_KEY)
       struct ts_key *key;
       int key_num;
       int (*to_key)(const struct ts_key *key,int key_num,int x,int y);
       int (*report_key)(struct input_dev *input,const struct ts_key *key,int key_num,int last_status,int status);       
#endif
       int (*convert)(int x, int y);
       void (*convert2)(int *in_out_x, int *in_out_y);
       int (*is_ac_online)(void);
};

#endif /* !_LINUX_TOUCH_H_*/
