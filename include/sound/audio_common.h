/*
 * Copyright 2005 Openedhand Ltd.
 *
 * Author: Richard Purdie <richard@openedhand.com>
 *
 * Based on WM8753.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef _AUDIO_COMMON_H_
#define _AUDIO_COMMON_H_

struct audio_platform_data {
    int (*is_hp_pluged)(void);
    void (*codec_power_control)(int enable);
    void (*amp_power_control)(int enable);
};

#endif

