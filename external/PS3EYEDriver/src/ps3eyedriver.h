/*
	This file is part of PS3EyeDriver, a modified version of the 
	original PS3EYEDriver by Chadwick Boulay: https://github.com/inspirit/PS3EYEDriver.
	
	Copyright (C) 2015 by Brendan Walker (brendan@millerwalker.net)

	Because large portions of this code came from the Linux kernel driver code,
	it must also be GPL'd as it is a derivative work:	
	
    PS3EyeDriver is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    PS3EyeDriver is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with PS3EyeDriver.  If not, see <http://www.gnu.org/licenses/>.
	
	@license GPL-3.0+ <http://spdx.org/licenses/GPL-3.0+>
*/

#ifndef PS3EYEDRIVER_H
#define PS3EYEDRIVER_H
// -- includes -----
#include <stdbool.h>

// -- pre-declarations -----
struct _ps3eye_camera;
typedef struct _ps3eye_camera ps3eye_t;

/* stdint.h is not available on older MSVC */
#if defined(_MSC_VER) && (_MSC_VER < 1600) && (!defined(_STDINT)) && (!defined(_STDINT_H))
typedef unsigned __int8   uint8_t;
typedef unsigned __int16  uint16_t;
typedef unsigned __int32  uint32_t;
#else
#include <stdint.h>
#endif

// -- interface -----
#ifdef __cplusplus
extern "C" {
#endif

void ps3eye_init();
void ps3eye_uninit();

int ps3eye_count_connected();

ps3eye_t *ps3eye_open(uint32_t cameraID, uint32_t width, uint32_t height, uint8_t fps);
void ps3eye_close(ps3eye_t *eye);

uint8_t *ps3eye_grab_frame(ps3eye_t *camera, int *out_stride);

bool ps3eye_get_auto_gain(ps3eye_t *camera);
void ps3eye_set_auto_gain(ps3eye_t *camera, bool val);

bool ps3eye_get_auto_white_balance(ps3eye_t *camera);
void ps3eye_set_auto_white_balance(ps3eye_t *camera, bool val);

uint8_t ps3eye_get_gain(ps3eye_t *camera);
void ps3eye_set_gain(ps3eye_t *camera, uint8_t val);

uint8_t ps3eye_get_exposure(ps3eye_t *camera);
void ps3eye_set_exposure(ps3eye_t *camera, uint8_t val);

uint8_t ps3eye_get_sharpness(ps3eye_t *camera);
void ps3eye_set_sharpness(ps3eye_t *camera, uint8_t val);

uint8_t ps3eye_get_contrast(ps3eye_t *camera);
void ps3eye_set_contrast(ps3eye_t *camera, uint8_t val);

uint8_t ps3eye_get_brightness(ps3eye_t *camera);
void ps3eye_set_brightness(ps3eye_t *camera, uint8_t val);

uint8_t ps3eye_get_hue(ps3eye_t *camera);
void ps3eye_set_hue(ps3eye_t *camera, uint8_t val);

uint8_t ps3eye_get_red_balance(ps3eye_t *camera);
void ps3eye_set_red_balance(ps3eye_t *camera, uint8_t val);

uint8_t ps3eye_get_blue_balance(ps3eye_t *camera);
void ps3eye_set_blue_balance(ps3eye_t *camera, uint8_t val);

void ps3eye_set_flip(ps3eye_t *camera, bool horizontal, bool vertical);

#ifdef __cplusplus
}
#endif

#endif //PS3EYEDRIVER_H