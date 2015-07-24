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
// -- includes -----
#include <stdbool.h>
#include <assert.h>

#include "ps3eyedriver.h"
#include "libusb.h"

// -- macros -----
#if defined WIN32 || defined _WIN32 || defined WINCE
	#define USE_WINDOWS_THREADS
	#include <windows.h>
	#include <process.h>
#else
	#include <sys/time.h>
	#include <time.h>
	#if defined __MACH__ && defined __APPLE__
		#include <mach/mach.h>
		#include <mach/mach_time.h>
	#endif
#endif

#if defined(DEBUG)
#define debug(...) fprintf(stdout,__VA_ARGS__)
#else
#define debug(...) 
#endif

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(_A) (sizeof(_A) / sizeof((_A)[0]))
#endif

#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef MAX
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

//-- constants -----
const uint16_t PS3EYE_VENDOR_ID = 0x1415;
const uint16_t PS3EYE_PRODUCT_ID = 0x2000;

#define OV534_REG_ADDRESS	0xf1	/* sensor address */
#define OV534_REG_SUBADDR	0xf2
#define OV534_REG_WRITE		0xf3
#define OV534_REG_READ		0xf4
#define OV534_REG_OPERATION	0xf5
#define OV534_REG_STATUS	0xf6

#define OV534_OP_WRITE_3	0x37
#define OV534_OP_WRITE_2	0x33
#define OV534_OP_READ_2		0xf9

#define CTRL_TIMEOUT 500
#define VGA	 0
#define QVGA 1

static const uint8_t ov534_reg_initdata[][2] = {
	{ 0xe7, 0x3a },

	{ OV534_REG_ADDRESS, 0x42 }, /* select OV772x sensor */

	{ 0xc2, 0x0c },
	{ 0x88, 0xf8 },
	{ 0xc3, 0x69 },
	{ 0x89, 0xff },
	{ 0x76, 0x03 },
	{ 0x92, 0x01 },
	{ 0x93, 0x18 },
	{ 0x94, 0x10 },
	{ 0x95, 0x10 },
	{ 0xe2, 0x00 },
	{ 0xe7, 0x3e },

	{ 0x96, 0x00 },

	{ 0x97, 0x20 },
	{ 0x97, 0x20 },
	{ 0x97, 0x20 },
	{ 0x97, 0x0a },
	{ 0x97, 0x3f },
	{ 0x97, 0x4a },
	{ 0x97, 0x20 },
	{ 0x97, 0x15 },
	{ 0x97, 0x0b },

	{ 0x8e, 0x40 },
	{ 0x1f, 0x81 },
	{ 0x34, 0x05 },
	{ 0xe3, 0x04 },
	{ 0x88, 0x00 },
	{ 0x89, 0x00 },
	{ 0x76, 0x00 },
	{ 0xe7, 0x2e },
	{ 0x31, 0xf9 },
	{ 0x25, 0x42 },
	{ 0x21, 0xf0 },

	{ 0x1c, 0x00 },
	{ 0x1d, 0x40 },
	{ 0x1d, 0x02 }, /* payload size 0x0200 * 4 = 2048 bytes */
	{ 0x1d, 0x00 }, /* payload size */

// -------------

//	{ 0x1d, 0x01 },/* frame size */		// kwasy
//	{ 0x1d, 0x4b },/* frame size */
//	{ 0x1d, 0x00 }, /* frame size */


//	{ 0x1d, 0x02 },/* frame size */		// macam
//	{ 0x1d, 0x57 },/* frame size */
//	{ 0x1d, 0xff }, /* frame size */

	{ 0x1d, 0x02 },/* frame size */		// jfrancois / linuxtv.org/hg/v4l-dvb
	{ 0x1d, 0x58 },/* frame size */
	{ 0x1d, 0x00 }, /* frame size */

// ---------

	{ 0x1c, 0x0a },
	{ 0x1d, 0x08 }, /* turn on UVC header */
	{ 0x1d, 0x0e }, /* .. */

	{ 0x8d, 0x1c },
	{ 0x8e, 0x80 },
	{ 0xe5, 0x04 },

// ----------------
//	{ 0xc0, 0x28 },//	kwasy / macam
//	{ 0xc1, 0x1e },//

	{ 0xc0, 0x50 },		// jfrancois
	{ 0xc1, 0x3c },
	{ 0xc2, 0x0c }, 


	
};

static const uint8_t ov772x_reg_initdata[][2] = {

	{0x12, 0x80 },
	{0x11, 0x01 },
	{0x11, 0x01 },
	{0x11, 0x01 },
	{0x11, 0x01 },
	{0x11, 0x01 },
	{0x11, 0x01 },
	{0x11, 0x01 },
	{0x11, 0x01 },
	{0x11, 0x01 },
	{0x11, 0x01 },
	{0x11, 0x01 },

	{0x3d, 0x03 },
	{0x17, 0x26 },
	{0x18, 0xa0 },
	{0x19, 0x07 },
	{0x1a, 0xf0 },
	{0x32, 0x00 },
	{0x29, 0xa0 },
	{0x2c, 0xf0 },
	{0x65, 0x20 },
	{0x11, 0x01 },
	{0x42, 0x7f },
	{0x63, 0xAA }, 	// AWB
	{0x64, 0xff },
	{0x66, 0x00 },
	{0x13, 0xf0 },	// COM8  - jfrancois 0xf0	orig x0f7
	{0x0d, 0x41 },
	{0x0f, 0xc5 },
	{0x14, 0x11 },

	{0x22, 0x7f },
	{0x23, 0x03 },
	{0x24, 0x40 },
	{0x25, 0x30 },
	{0x26, 0xa1 },
	{0x2a, 0x00 },
	{0x2b, 0x00 }, 
	{0x6b, 0xaa },
	{0x13, 0xff },	// COM8 - jfrancois 0xff orig 0xf7

	{0x90, 0x05 },
	{0x91, 0x01 },
	{0x92, 0x03 },
	{0x93, 0x00 },
	{0x94, 0x60 },
	{0x95, 0x3c },
	{0x96, 0x24 },
	{0x97, 0x1e },
	{0x98, 0x62 },
	{0x99, 0x80 },
	{0x9a, 0x1e },
	{0x9b, 0x08 },
	{0x9c, 0x20 },
	{0x9e, 0x81 },

	{0xa6, 0x04 },
	{0x7e, 0x0c },
	{0x7f, 0x16 },
	{0x80, 0x2a },
	{0x81, 0x4e },
    {0x82, 0x61 },
	{0x83, 0x6f },
	{0x84, 0x7b },
	{0x85, 0x86 },
	{0x86, 0x8e },
	{0x87, 0x97 },
	{0x88, 0xa4 },
	{0x89, 0xaf },
	{0x8a, 0xc5 },
	{0x8b, 0xd7 },
	{0x8c, 0xe8 },
	{0x8d, 0x20 },

	{0x0c, 0x90 },

	{0x2b, 0x00 }, 
	{0x22, 0x7f },
	{0x23, 0x03 },
	{0x11, 0x01 },
	{0x0c, 0xd0 },
	{0x64, 0xff },
	{0x0d, 0x41 },

	{0x14, 0x41 },
	{0x0e, 0xcd },
	{0xac, 0xbf },
	{0x8e, 0x00 },	// De-noise threshold - jfrancois 0x00 - orig 0x04
	{0x0c, 0xd0 }

};

static const uint8_t bridge_start_vga[][2] = {
	{0x1c, 0x00},
	{0x1d, 0x40},
	{0x1d, 0x02},
	{0x1d, 0x00},
	{0x1d, 0x02},
	{0x1d, 0x58},
	{0x1d, 0x00},
	{0xc0, 0x50},
	{0xc1, 0x3c},
};
static const uint8_t sensor_start_vga[][2] = {
	{0x12, 0x00},
	{0x17, 0x26},
	{0x18, 0xa0},
	{0x19, 0x07},
	{0x1a, 0xf0},
	{0x29, 0xa0},
	{0x2c, 0xf0},
	{0x65, 0x20},
};
static const uint8_t bridge_start_qvga[][2] = {
	{0x1c, 0x00},
	{0x1d, 0x40},
	{0x1d, 0x02},
	{0x1d, 0x00},
	{0x1d, 0x01},
	{0x1d, 0x4b},
	{0x1d, 0x00},
	{0xc0, 0x28},
	{0xc1, 0x1e},
};
static const uint8_t sensor_start_qvga[][2] = {
	{0x12, 0x40},
	{0x17, 0x3f},
	{0x18, 0x50},
	{0x19, 0x03},
	{0x1a, 0x78},
	{0x29, 0x50},
	{0x2c, 0x78},
	{0x65, 0x2f},
};

/* Values for bmHeaderInfo (Video and Still Image Payload Headers, 2.4.3.3) */
#define UVC_STREAM_EOH	(1 << 7)
#define UVC_STREAM_ERR	(1 << 6)
#define UVC_STREAM_STI	(1 << 5)
#define UVC_STREAM_RES	(1 << 4)
#define UVC_STREAM_SCR	(1 << 3)
#define UVC_STREAM_PTS	(1 << 2)
#define UVC_STREAM_EOF	(1 << 1)
#define UVC_STREAM_FID	(1 << 0)

/* packet types when moving from iso buf to frame buf */
enum gspca_packet_type {
    DISCARD_PACKET,
    FIRST_PACKET,
    INTER_PACKET,
    LAST_PACKET
};

//-- structures -----
struct _ps3eye_yuv422_buffer 
{
	uint8_t *pixels;
	size_t size;
	uint32_t stride;
	uint32_t width;
	uint32_t height;
};
typedef struct _ps3eye_yuv422_buffer ps3eye_yuv422_buffer_t;

struct _ps3eye_yuv422_buffer_concurrent_queue 
{
	void *mutex;
	uint32_t start_buffer_index;
	uint32_t end_buffer_index;
	uint32_t buffer_count;
	ps3eye_yuv422_buffer_t *buffers;
};
typedef struct _ps3eye_yuv422_buffer_concurrent_queue ps3eye_yuv422_buffer_concurrent_queue_t;

struct _ps3eye_usb_request_block
{
	bool allow_new_xfrs;

	int num_transfers;
	enum gspca_packet_type last_packet_type;
	uint32_t last_pts;
	uint16_t last_fid;
	struct libusb_transfer *xfr[2];

	uint8_t *frame_buffer;
    uint8_t *frame_buffer_end;
    uint8_t *frame_data_start;
	uint32_t frame_data_len;
	uint32_t frame_size;
	uint8_t frame_complete_ind;
	uint8_t frame_work_ind;

	int64_t last_frame_time;	
};
typedef struct _ps3eye_usb_request_block ps3eye_urb_t;

struct _ps3eye_thread_state
{
	uint32_t thread_id;
	uintptr_t thread_handle;
	void *start_event;
	volatile int32_t stop_flag;
};
typedef struct _ps3eye_thread_state ps3eye_thread_state_t;

struct _ps3eye_camera
{
	bool autogain; 
	uint8_t gain; // 0 <-> 63
	uint8_t exposure; // 0 <-> 255
	uint8_t sharpness; // 0 <-> 63
	uint8_t hue; // 0 <-> 255
	bool awb;
	uint8_t brightness; // 0 <-> 255
	uint8_t contrast; // 0 <-> 255
	uint8_t blueblc; // 0 <-> 255
	uint8_t redblc; // 0 <-> 255
    bool flip_h;
    bool flip_v;
    bool is_streaming;

	uint32_t frame_width;
	uint32_t frame_height;
	uint32_t frame_stride;
	uint8_t frame_rate;

	int64_t last_qued_frame_time;

	//usb stuff
	libusb_device *device;
	libusb_device_handle *handle;
	int interface_index;
	uint8_t usb_buf[64];

	ps3eye_urb_t urb;
	ps3eye_thread_state_t thread_state;
	ps3eye_yuv422_buffer_concurrent_queue_t buffer_queue;
};
typedef struct _ps3eye_camera ps3eye_t;

//-- globals -----
libusb_context* g_usb_context= NULL;

//-- prototypes -----

// camera setup/teardown methods
static ps3eye_t *_ps3eye_camera_allocate();
static void _ps3eye_camera_dispose(ps3eye_t *camera);
static bool _ps3eye_camera_setup(
	ps3eye_t *camera, int camera_id, uint32_t width, uint32_t height, uint8_t desiredFrameRate);
static bool _ps3eye_camera_start(ps3eye_t *camera);
static void _ps3eye_camera_stop(ps3eye_t *camera);	
static bool _ps3eye_camera_usb_open(ps3eye_t *camera, int camera_id);
static void _ps3eye_camera_usb_close(ps3eye_t *camera);

// threading helper methods
static bool _ps3eye_camera_thread_start(ps3eye_t *camera);
static uint32_t __stdcall _ps3eye_camera_thread_run(void *user_data);
static void _ps3eye_camera_thread_stop(ps3eye_t *camera);
static bool _ps3eye_thread_event_set(void *event);
static void *_ps3eye_mutex_allocate();
static void _ps3eye_mutex_dispose(void *mutex);
static bool _ps3eye_mutex_lock(void *mutex);
static void _ps3eye_mutex_unlock(void *mutex);

// frame buffer queue methods
static void _ps3eye_frame_buffer_queue_init(
	ps3eye_yuv422_buffer_concurrent_queue_t *queue, int buffer_count, uint32_t buffer_stride, uint32_t buffer_width, uint32_t buffer_height);
static void _ps3eye_frame_buffer_queue_write_frame(ps3eye_yuv422_buffer_concurrent_queue_t *queue, uint32_t buffer_stride, uint8_t *pixels);
static ps3eye_yuv422_buffer_t *_ps3eye_frame_buffer_queue_read_frame(ps3eye_yuv422_buffer_concurrent_queue_t *queue);
static void _ps3eye_frame_buffer_queue_dispose(ps3eye_yuv422_buffer_concurrent_queue_t *queue);

// usb command helpers
static void ov534_set_led(ps3eye_t *camera, int status);
static void ov534_set_frame_rate(ps3eye_t *camera, uint8_t frame_rate);
static void ov534_reg_write(ps3eye_t *camera, uint16_t reg, uint8_t val);
static uint8_t ov534_reg_read(ps3eye_t *camera, uint16_t reg);
static int sccb_check_status(ps3eye_t *camera);
static void sccb_reg_write(ps3eye_t *camera, uint8_t reg, uint8_t val);
static uint8_t sccb_reg_read(ps3eye_t *camera, uint16_t reg);
static void reg_w_array(ps3eye_t *camera, const uint8_t (*data)[2], int len);
static void sccb_w_array(ps3eye_t *camera, const uint8_t (*data)[2], int len);

// USB request block helper methods
static void _ps3eye_urb_init(ps3eye_urb_t *urb);
static void _ps3eye_urb_dispose(ps3eye_urb_t *urb);
static bool _ps3eye_urb_start_transfers(ps3eye_urb_t *urb, libusb_device_handle *handle, uint32_t curr_frame_size);
static void _ps3eye_urb_close_transfers(ps3eye_urb_t *urb);
static void _ps3eye_urb_frame_add(ps3eye_urb_t *urb, enum gspca_packet_type packet_type, const uint8_t *data, int len);
static void _ps3eye_urb_pkt_scan(ps3eye_urb_t *urb, uint8_t *data, int len);
static void LIBUSB_CALL _ps3eye_urb_transfer_callback(struct libusb_transfer *xfr);

// usb helper methods
static uint8_t _pseye_usb_find_endpoint(struct libusb_device *device);
static void _pseye_usb_error(int error_code);

// time helper methods
static int64_t _pseye_get_tick_count();

//-- public methods -----
void 
ps3eye_init()
{
	if (g_usb_context == NULL)
	{
		debug("ps3eye: initializing up libusb\n");
		libusb_init(&g_usb_context);
		libusb_set_debug(g_usb_context, 1);			
	}
}

void 
ps3eye_uninit()
{
	if (g_usb_context != NULL)
	{
		debug("ps3eye: cleaning up libusb\n");
		libusb_exit(g_usb_context);		
		g_usb_context= NULL;
	}
}

int 
ps3eye_count_connected()
{
	int camera_count= 0;

	libusb_device *dev;
	libusb_device **usb_devices;
	int i = 0;

	int usb_result = (int)libusb_get_device_list(g_usb_context, &usb_devices);

	if (usb_result >= 0)
	{
		while ((dev = usb_devices[i++]) != NULL) 
		{
			struct libusb_device_descriptor desc;
			
			libusb_get_device_descriptor(dev, &desc);
			if(desc.idVendor == PS3EYE_VENDOR_ID && desc.idProduct == PS3EYE_PRODUCT_ID)
			{
				camera_count++;
			}
		}		
	}
	else
	{
		debug("ps3eye_count_connected: Error Device scan:\n");		
		_pseye_usb_error(usb_result);
		camera_count = 0;
	}

	libusb_free_device_list(usb_devices, 1);

	return camera_count;
}

ps3eye_t *
ps3eye_open(uint32_t cameraID, uint32_t frame_width, uint32_t frame_height, uint8_t frame_rate)
{
	bool success= true;
	ps3eye_t *camera= NULL;
	
	if ((camera = _ps3eye_camera_allocate()) == NULL)
	{
		debug("ps3eye_open: Can't open camera id %d (unable to allocate)\n", cameraID);
		success= false;		
	}
	
	if (success && 
		!_ps3eye_camera_setup(camera, cameraID, frame_width, frame_height, frame_rate))
	{
		debug("ps3eye_open: Can't open camera id %d (unable to setup camera)\n", cameraID);
		success= false;				
	}
	
	if (success && !_ps3eye_camera_start(camera))
	{
		debug("ps3eye_open: Can't open camera id %d (unable to start streaming)\n", cameraID);
		success= false;				
	}

	if (success && !_ps3eye_camera_thread_start(camera))
	{
		debug("ps3eye_open: Can't open camera id %d (unable to start thread)\n", cameraID);
		success = false;
	}
	
	if (!success)
	{
		ps3eye_close(camera);
	}
	
	return camera;
}

void 
ps3eye_close(ps3eye_t *camera)
{
	if (camera)
	{
		_ps3eye_camera_thread_stop(camera);
		_ps3eye_camera_stop(camera);
		_ps3eye_camera_usb_close(camera);		
		_ps3eye_camera_dispose(camera);
	}
}

uint8_t *
ps3eye_grab_frame(ps3eye_t *camera, int *out_stride)
{
	ps3eye_yuv422_buffer_t *buffer= _ps3eye_frame_buffer_queue_read_frame(&camera->buffer_queue);
	uint8_t *frame_data = (buffer != NULL) ? buffer->pixels : NULL;

	*out_stride = camera->frame_stride;
	
	return frame_data;
}

bool 
ps3eye_get_auto_gain(ps3eye_t *camera) 
{ 
	return camera->autogain; 
}

void 
ps3eye_set_auto_gain(ps3eye_t *camera, bool val) 
{
	camera->autogain = val;
	
	if (val)
	{
		sccb_reg_write(camera, 0x13, 0xf7); //AGC,AEC,AWB ON
		sccb_reg_write(camera, 0x64, sccb_reg_read(camera, 0x64)|0x03);
	}
	else 
	{
		sccb_reg_write(camera, 0x13, 0xf0); //AGC,AEC,AWB OFF
		sccb_reg_write(camera, 0x64, sccb_reg_read(camera, 0x64)&0xFC);

		ps3eye_set_gain(camera, camera->gain);
		ps3eye_set_exposure(camera, camera->exposure);
	}
}

bool 
ps3eye_get_auto_white_balance(ps3eye_t *camera) 
{ 
	return camera->awb; 
}

void 
ps3eye_set_auto_white_balance(ps3eye_t *camera, bool val) 
{
	camera->awb = val;
	
	if (val) 
	{
		sccb_reg_write(camera, 0x63, 0xe0); //AWB ON
	}
	else
	{
		sccb_reg_write(camera, 0x63, 0xAA); //AWB OFF
	}
}

uint8_t 
ps3eye_get_gain(ps3eye_t *camera)
{
	return camera->gain; 
}

void 
ps3eye_set_gain(ps3eye_t *camera, uint8_t val) 
{
	camera->gain = val;
	
	switch(val & 0x30){
	case 0x00:
		val &=0x0F;
		break;
	case 0x10:
		val &=0x0F;
		val |=0x30;
		break;
	case 0x20:
		val &=0x0F;
		val |=0x70;
		break;
	case 0x30:
		val &=0x0F;
		val |=0xF0;
		break;
	}
	
	sccb_reg_write(camera, 0x00, val);
}

uint8_t 
ps3eye_get_exposure(ps3eye_t *camera)
{ 
	return camera->exposure; 
}

void 
ps3eye_set_exposure(ps3eye_t *camera, uint8_t val) 
{
	camera->exposure = val;
	sccb_reg_write(camera, 0x08, val>>7);
	sccb_reg_write(camera, 0x10, val<<1);
}

uint8_t 
ps3eye_get_sharpness(ps3eye_t *camera)
{
	return camera->sharpness; 
}

void 
ps3eye_set_sharpness(ps3eye_t *camera, uint8_t val) 
{
	camera->sharpness = val;
	sccb_reg_write(camera, 0x91, val); //vga noise
	sccb_reg_write(camera, 0x8E, val); //qvga noise
}

uint8_t 
ps3eye_get_contrast(ps3eye_t *camera)
{
	return camera->contrast; 
}

void 
ps3eye_set_contrast(ps3eye_t *camera, uint8_t val) 
{
	camera->contrast = val;
	sccb_reg_write(camera, 0x9C, val);
}

uint8_t 
ps3eye_get_brightness(ps3eye_t *camera)
{ 
	return camera->brightness; 
}

void 
ps3eye_set_brightness(ps3eye_t *camera, uint8_t val) 
{
	camera->brightness = val;
	sccb_reg_write(camera, 0x9B, val);
}

uint8_t 
ps3eye_get_hue(ps3eye_t *camera)
{ 
	return camera->hue; 
}

void 
ps3eye_set_hue(ps3eye_t *camera, uint8_t val) 
{
	camera->hue = val;
	sccb_reg_write(camera, 0x01, val);
}

uint8_t 
ps3eye_get_red_balance(ps3eye_t *camera) 
{ 
	return camera->redblc; 
}

void 
ps3eye_set_red_balance(ps3eye_t *camera, uint8_t val) 
{
	camera->redblc = val;
	sccb_reg_write(camera, 0x43, val);
}

uint8_t 
ps3eye_get_blue_balance(ps3eye_t *camera) 
{ 
	return camera->blueblc; 
}

void 
ps3eye_set_blue_balance(ps3eye_t *camera, uint8_t val) 
{
	camera->blueblc = val;
	sccb_reg_write(camera, 0x42, val);
}

void 
ps3eye_set_flip(ps3eye_t *camera, bool horizontal, bool vertical) 
{
	camera->flip_h = horizontal;
	camera->flip_v = vertical;
	uint8_t val = sccb_reg_read(camera, 0x0c);
	val &= ~0xc0;
	if (!horizontal) val |= 0x40;
	if (!vertical) val |= 0x80;
	sccb_reg_write(camera, 0x0c, val);
}

//-- private methods -----

// -- thread methods --
static bool _ps3eye_camera_thread_start(
	ps3eye_t *camera)
{
#ifdef USE_WINDOWS_THREADS
	ps3eye_thread_state_t *thread_state= &camera->thread_state;

	// Create manual reset events for starting and stopping the thread
	thread_state->start_event = CreateEvent(NULL, FALSE, FALSE, NULL);
	if (thread_state->start_event == NULL)
	{
		debug("_ps3eye_thread_start: Failed to allocate start signal(%d)\n", GetLastError());
		_ps3eye_camera_thread_stop(camera);
		return false;
	}

	// This is raised with an atomic increment later
	thread_state->stop_flag = 0;

	// Create and start the thread
	thread_state->thread_handle =
		_beginthreadex(
			NULL, // security == NULL means thread cannot be inherited by a child process
			0, // stacksize = 0 means use the same stack sized used for the main thread
			&_ps3eye_camera_thread_run,
			camera, // argument to the thread function
			0, // initflag == 0 means run thread immediately
			&thread_state->thread_id);

	if (thread_state->thread_handle != 0)
	{
		if (WaitForSingleObject(thread_state->start_event, INFINITE) != WAIT_OBJECT_0)
		{
			debug("_ps3eye_thread_start: Timeout on thread start(%d)\n", GetLastError());
			_ps3eye_camera_thread_stop(camera);
			return false;
		}
	}
	else
	{
		debug("_ps3eye_thread_start: Failed to start thread(%d)\n", GetLastError());
		_ps3eye_camera_thread_stop(camera);
		return false;
	}

	return true;
#else
	// NOT YET IMPLEMENTED
	return false;
#endif
}

static uint32_t
__stdcall _ps3eye_camera_thread_run(void *user_data)
{
	ps3eye_t *camera = (ps3eye_t *)user_data;
	ps3eye_thread_state_t *thread_state = &camera->thread_state;
	ps3eye_yuv422_buffer_concurrent_queue_t *buffer_queue = &camera->buffer_queue;

	// Tell the thread that created us that we've started
	_ps3eye_thread_event_set(thread_state->start_event);

	while (thread_state->stop_flag == 0)
	{
		bool is_new_frame = false;
		struct timeval ZERO_TIMEVAL = { 0, 0 };

		// Poll the URB transfers have completed to grab.
		// This will force the URB transfer callback to get called.
		int result = libusb_handle_events(g_usb_context);
		if (result < 0)
		{
			debug("_ps3eye_thread_run: Failed to poll usb events:\n");
			_pseye_usb_error(result);
		}

		// See if the transfer callback completed the frame transfer
		if (camera->last_qued_frame_time < camera->urb.last_frame_time)
		{
			camera->last_qued_frame_time = camera->urb.last_frame_time;
			is_new_frame = true;
		}

		if (is_new_frame) 
		{
			uint8_t *frame_data = (uint8_t *)(
				camera->urb.frame_buffer
				+ camera->urb.frame_complete_ind * camera->urb.frame_size);

			_ps3eye_frame_buffer_queue_write_frame(buffer_queue, camera->frame_stride, frame_data);
		}
	}

	#ifdef USE_WINDOWS_THREADS
	_endthreadex(EXIT_SUCCESS);
	#else
	// NOT YET IMPLEMENTED
	#endif

	return EXIT_SUCCESS;
}

static void _ps3eye_camera_thread_stop(
	ps3eye_t *camera)
{
	#ifdef USE_WINDOWS_THREADS
	ps3eye_thread_state_t *thread_state = &camera->thread_state;

	if (thread_state->thread_handle != 0)
	{
		// Signal the camera thread to stop
		InterlockedIncrement(&thread_state->stop_flag);

		// Give control to other threads
		Sleep(0); 

		// Wait until thread has stopped
		WaitForSingleObject((void *)thread_state->thread_handle, INFINITE); 

		thread_state->thread_handle = 0;
		thread_state->thread_id = -1;
		thread_state->stop_flag = 0;
	}

	if (thread_state->start_event != NULL)
	{
		CloseHandle(thread_state->start_event);
		thread_state->start_event = NULL;
	}
	#else
	// NOT YET IMPLEMENTED
	#endif
}

static bool
_ps3eye_thread_event_set(void *event)
{
	bool success = false;

#ifdef USE_WINDOWS_THREADS
	success = (SetEvent(event) != 0);
#else
	// NOT YET IMPLEMENTED
#endif

	return success;
}

static void *
_ps3eye_mutex_allocate()
{
	void *mutex = NULL;

#ifdef USE_WINDOWS_THREADS
	mutex = CreateMutex(NULL, FALSE, NULL);
	if (mutex == NULL)
	{
		debug("_ps3eye_mutex_allocate: Failed to allocate lock(%d)\n", GetLastError());
	}
#else
	// NOT YET IMPLEMENTED
#endif

	return mutex;
}

static void 
_ps3eye_mutex_dispose(void *mutex)
{
	if (mutex != NULL)
	{
#ifdef USE_WINDOWS_THREADS
		CloseHandle(mutex);
#else
		// NOT YET IMPLEMENTED
#endif
	}
}

static bool
_ps3eye_mutex_lock(void *mutex)
{
	bool has_lock = false;

	if (mutex != NULL)
	{
#ifdef USE_WINDOWS_THREADS
		uint32_t result = WaitForSingleObject(mutex, INFINITE);

		if (result == WAIT_OBJECT_0 || result == WAIT_ABANDONED)
		{
			has_lock = true;
		}
		else
		{
			debug("_ps3eye_frame_buffer_queue_read_frame: Failed to lock mutex(%d)\n", GetLastError());
		}
#else
	// NOT YET IMPLEMENTED
#endif
	}

	return has_lock;
}

static void
_ps3eye_mutex_unlock(void *mutex)
{
	if (mutex != NULL)
	{
#ifdef USE_WINDOWS_THREADS
		ReleaseMutex(mutex);
#else
		// NOT YET IMPLEMENTED
#endif
	}
}

// -- frame buffer queue methods --
static void
_ps3eye_frame_buffer_queue_init(
	ps3eye_yuv422_buffer_concurrent_queue_t *queue, int buffer_count,
	uint32_t buffer_stride, uint32_t buffer_width, uint32_t buffer_height)
{
	queue->start_buffer_index = 0;
	queue->end_buffer_index = 0;
	queue->buffer_count = buffer_count;
	queue->buffers = (ps3eye_yuv422_buffer_t *)calloc(sizeof(ps3eye_yuv422_buffer_t), buffer_count);

	int i;
	for (i = 0; i < buffer_count; ++i)
	{
		ps3eye_yuv422_buffer_t *buffer = &queue->buffers[i];
		size_t size = buffer_stride * buffer_height;

		buffer->pixels = (uint8_t *)calloc(sizeof(uint8_t), size);
		buffer->size = size;
		buffer->width = buffer_width;
		buffer->height = buffer_height;
		buffer->stride = buffer_stride;
	}

	queue->mutex = _ps3eye_mutex_allocate();
}

static void
_ps3eye_frame_buffer_queue_write_frame(ps3eye_yuv422_buffer_concurrent_queue_t *queue, uint32_t buffer_stride, uint8_t *pixels)
{
	ps3eye_yuv422_buffer_t *buffer;

	bool has_lock = _ps3eye_mutex_lock(queue->mutex);

	// The the newest frame to the end of the queue
	buffer = queue->buffers + queue->end_buffer_index;
	assert(buffer->stride == buffer_stride);
	memcpy(buffer->pixels, pixels, buffer->size);

	// Advance the end buffer index
	queue->end_buffer_index = (queue->end_buffer_index + 1) % queue->buffer_count;
	
	// If the end of the queue overlapped the start, then advance the start
	queue->start_buffer_index = 
		(queue->start_buffer_index == queue->end_buffer_index)
		? (queue->start_buffer_index + 1) % queue->buffer_count 
		: queue->start_buffer_index;

	if (has_lock)
	{
		_ps3eye_mutex_unlock(queue->mutex);
	}
}

static ps3eye_yuv422_buffer_t *
_ps3eye_frame_buffer_queue_read_frame(ps3eye_yuv422_buffer_concurrent_queue_t *queue)
{
	bool has_lock = _ps3eye_mutex_lock(queue->mutex);

	ps3eye_yuv422_buffer_t *buffer = NULL;

	// Only return a frame if there is data in the queue
	if (queue->start_buffer_index != queue->end_buffer_index)
	{
		buffer = queue->buffers + queue->start_buffer_index;
		queue->start_buffer_index = (queue->start_buffer_index + 1) % queue->buffer_count;
	}

	if (has_lock)
	{
		_ps3eye_mutex_unlock(queue->mutex);
	}

	return buffer;
}

static void
_ps3eye_frame_buffer_queue_dispose(ps3eye_yuv422_buffer_concurrent_queue_t *queue)
{
	uint32_t i;

	for (i = 0; i < queue->buffer_count; i++)
	{
		if (queue->buffers[i].pixels != NULL)
		{
			free(queue->buffers[i].pixels);
		}
	}

	free(queue->buffers);

	_ps3eye_mutex_dispose(queue->mutex);
	queue->mutex = NULL;
}

// -- camera setup/teardown methods --
static ps3eye_t *
_ps3eye_camera_allocate()
{
	ps3eye_t *camera= (ps3eye_t *)malloc(sizeof(ps3eye_t));	
	memset(camera, 0, sizeof(ps3eye_t));
	
	// default controls
	camera->autogain = false;
	camera->gain = 20;
	camera->exposure = 120;
	camera->sharpness = 0;
	camera->hue = 143;
	camera->awb = false;
	camera->brightness = 20;
	camera->contrast =  37;
	camera->blueblc = 128;
	camera->redblc = 128;
    camera->flip_h = false;
    camera->flip_v = false;
	
	// USB state
	camera->device= NULL;
	camera->handle= NULL;
	camera->interface_index= -1;	
	
	_ps3eye_urb_init(&camera->urb);
	camera->thread_state.thread_id = -1;

	return camera;
}

static void
_ps3eye_camera_dispose(ps3eye_t *camera)
{
	_ps3eye_urb_dispose(&camera->urb);
	_ps3eye_frame_buffer_queue_dispose(&camera->buffer_queue);
	free(camera);
}

static bool 
_ps3eye_camera_setup(
	ps3eye_t *camera, int camera_id, uint32_t width, uint32_t height, uint8_t desiredFrameRate)
{
	uint16_t sensor_id;

	// open usb device so we can setup and go
	if(camera->handle == NULL) 
	{
		if( !_ps3eye_camera_usb_open(camera, camera_id) )
		{
			return false;
		}
	}

	// find best cam mode
	if((width == 0 && height == 0) || width > 320 || height > 240)
	{
		camera->frame_width = 640;
		camera->frame_height = 480;
		camera->frame_rate = MAX(desiredFrameRate, (uint8_t)15);
	} 
	else 
	{
		camera->frame_width = 320;
		camera->frame_height = 240;
		camera->frame_rate = MAX(desiredFrameRate, (uint8_t)30);
	}
    camera->frame_stride = camera->frame_width * 2;
	//

	// Allocate 3 frames worth of buffer
	_ps3eye_frame_buffer_queue_init(
		&camera->buffer_queue, 
		3, 
		camera->frame_stride, 
		camera->frame_width,
		camera->frame_height);

	/* reset bridge */
	ov534_reg_write(camera, 0xe7, 0x3a);
	ov534_reg_write(camera, 0xe0, 0x08);

#ifdef _MSC_VER
	Sleep(100);
#else
    nanosleep((struct timespec[]){{0, 100000000}}, NULL);
#endif

	/* initialize the sensor address */
	ov534_reg_write(camera, OV534_REG_ADDRESS, 0x42);

	/* reset sensor */
	sccb_reg_write(camera, 0x12, 0x80);
#ifdef _MSC_VER
	Sleep(10);
#else    
    nanosleep((struct timespec[]){{0, 10000000}}, NULL);
#endif

	/* probe the sensor */
	sccb_reg_read(camera, 0x0a);
	sensor_id = sccb_reg_read(camera, 0x0a) << 8;
	sccb_reg_read(camera, 0x0b);
	sensor_id |= sccb_reg_read(camera, 0x0b);
	debug("Sensor ID: %04x\n", sensor_id);

	/* initialize */
	reg_w_array(camera, ov534_reg_initdata, ARRAY_SIZE(ov534_reg_initdata));
	ov534_set_led(camera, 1);
	sccb_w_array(camera, ov772x_reg_initdata, ARRAY_SIZE(ov772x_reg_initdata));
	ov534_reg_write(camera, 0xe0, 0x09);
	ov534_set_led(camera, 0);

	return true;
}

static bool 
_ps3eye_camera_start(ps3eye_t *camera)
{
	bool success= true;
	
    if(!camera->is_streaming)
	{
		if (camera->frame_width == 320) 
		{	/* 320x240 */
			reg_w_array(camera, bridge_start_qvga, ARRAY_SIZE(bridge_start_qvga));
			sccb_w_array(camera, sensor_start_qvga, ARRAY_SIZE(sensor_start_qvga));
		} 
		else 
		{	/* 640x480 */
			reg_w_array(camera, bridge_start_vga, ARRAY_SIZE(bridge_start_vga));
			sccb_w_array(camera, sensor_start_vga, ARRAY_SIZE(sensor_start_vga));
		}

		ov534_set_frame_rate(camera, camera->frame_rate);

		ps3eye_set_auto_gain(camera, camera->autogain);
		ps3eye_set_auto_white_balance(camera, camera->awb);
		ps3eye_set_gain(camera, camera->gain);
		ps3eye_set_hue(camera, camera->hue);
		ps3eye_set_exposure(camera, camera->exposure);
		ps3eye_set_brightness(camera, camera->brightness);
		ps3eye_set_contrast(camera, camera->contrast);
		ps3eye_set_sharpness(camera, camera->sharpness);
		ps3eye_set_red_balance(camera, camera->redblc);
		ps3eye_set_blue_balance(camera, camera->blueblc);
		ps3eye_set_flip(camera, camera->flip_h, camera->flip_v);

		ov534_set_led(camera, 1);
		ov534_reg_write(camera, 0xe0, 0x00); // start stream

		// init and start urb
		success= 
			_ps3eye_urb_start_transfers(
				&camera->urb, 
				camera->handle, 
				camera->frame_stride*camera->frame_height);
		camera->last_qued_frame_time = 0;
		camera->is_streaming = true;
	}
    
	return success;
}

static void 
_ps3eye_camera_stop(ps3eye_t *camera)
{
    if(!camera->is_streaming)
	{
		return;	
	}
    
	/* stop streaming data */
	ov534_reg_write(camera, 0xe0, 0x09);
	ov534_set_led(camera, 0);

	// close urb
	_ps3eye_urb_close_transfers(&camera->urb);

    camera->is_streaming = false;
}

static bool 
_ps3eye_camera_usb_open(ps3eye_t *camera, int camera_id)
{
	bool success= false;
	libusb_device **usb_devices= NULL;
	
	camera->device= NULL;
	camera->handle= NULL;
	
	// Find the usb camera that matched the camera id
	{
		int device_index = 0;
		int camera_index= 0;

		int result = (int)libusb_get_device_list(g_usb_context, &usb_devices);
		if (result >= 0)
		{
			libusb_device *test_usb_device= NULL;
			
			while ((test_usb_device = usb_devices[device_index++]) != NULL) 
			{
				struct libusb_device_descriptor desc;
				
				libusb_get_device_descriptor(test_usb_device, &desc);
				if(desc.idVendor == PS3EYE_VENDOR_ID && desc.idProduct == PS3EYE_PRODUCT_ID)
				{
					if (camera_index == camera_id)
					{
						camera->device= test_usb_device;
						break;
					}
					else
					{
						camera_index++;						
					}					
				}
			}		
		}
		else
		{
			debug("_ps3eye_camera_usb_open: Error Device scan:\n");		
			_pseye_usb_error(result);
		}		
	}
	
	// Attempt to open the device and claim the first interface
	if (camera->device != NULL)
	{
		int result = libusb_open(camera->device, &camera->handle);
		if(result >= 0) 
		{
			//libusb_set_configuration(handle_, 0);

			result = libusb_claim_interface(camera->handle, 0);
			if(result >= 0) 
			{
				camera->interface_index= 0;
				success= true;
			}
			else
			{
				debug("_ps3eye_camera_usb_open: device claim interface error: \n");
				_pseye_usb_error(result);
			}					
		}
		else
		{
			debug("_ps3eye_camera_usb_open: device open error: \n");
			_pseye_usb_error(result);
		}
	}
	
	if (usb_devices != NULL)
	{
		libusb_free_device_list(usb_devices, 1);		
	}	
	
	if (!success)
	{
		_ps3eye_camera_usb_close(camera);
	}

	return success;
}

static void 
_ps3eye_camera_usb_close(ps3eye_t *camera)
{
	if (camera->interface_index != -1)
	{
		debug("_ps3eye_camera_usb_close: freeing interface\n");
		libusb_release_interface(camera->handle, camera->interface_index);				
		
		camera->interface_index= -1;
	}

	if (camera->handle != NULL)
	{
		debug("_ps3eye_camera_usb_close: closing device\n");
		libusb_release_interface(camera->handle, 0);				
		libusb_close(camera->handle);
		
		camera->handle= NULL;
	}
	
	camera->device = NULL;
	debug("_ps3eye_camera_usb_closedevice closed\n");
}

// -- camera usb commands methods --
/* Two bits control LED: 0x21 bit 7 and 0x23 bit 7.
 * (direction and output)? */
static void 
ov534_set_led(ps3eye_t *camera, int status)
{
	uint8_t data;

	debug("led status: %d\n", status);

	data = ov534_reg_read(camera, 0x21);
	data |= 0x80;
	ov534_reg_write(camera, 0x21, data);

	data = ov534_reg_read(camera, 0x23);
	if (status)
		data |= 0x80;
	else
		data &= ~0x80;

	ov534_reg_write(camera, 0x23, data);
	
	if (!status) {
		data = ov534_reg_read(camera, 0x21);
		data &= ~0x80;
		ov534_reg_write(camera, 0x21, data);
	}
}

/* set framerate */
static void 
ov534_set_frame_rate(ps3eye_t *camera, uint8_t frame_rate)
{
	int i;
	struct rate_s 
	{
		 uint8_t fps;
		 uint8_t r11;
		 uint8_t r0d;
		 uint8_t re5;
	};
	const struct rate_s *r;
	static const struct rate_s rate_0[] = { /* 640x480 */
		 {60, 0x01, 0xc1, 0x04},
		 {50, 0x01, 0x41, 0x02},
		 {40, 0x02, 0xc1, 0x04},
		 {30, 0x04, 0x81, 0x02},
		 {15, 0x03, 0x41, 0x04},
	};
	static const struct rate_s rate_1[] = { /* 320x240 */
		 {125, 0x02, 0x81, 0x02},
		 {100, 0x02, 0xc1, 0x04},
		 {75, 0x03, 0xc1, 0x04},
		 {60, 0x04, 0xc1, 0x04},
		 {50, 0x02, 0x41, 0x04},
		 {40, 0x03, 0x41, 0x04},
		 {30, 0x04, 0x41, 0x04},
	};

	if (camera->frame_width == 640) 
	{
		 r = rate_0;
		 i = ARRAY_SIZE(rate_0);
	} 
	else 
	{
		 r = rate_1;
		 i = ARRAY_SIZE(rate_1);
	}

	while (--i > 0) 
	{
		if (frame_rate >= r->fps)
		{
			break;			
		}

		r++;
	}
 
	sccb_reg_write(camera, 0x11, r->r11);
	sccb_reg_write(camera, 0x0d, r->r0d);
	ov534_reg_write(camera, 0xe5, r->re5);

	debug("ov534_set_frame_rate: frame_rate: %d\n", r->fps);
}

static void 
ov534_reg_write(ps3eye_t *camera, uint16_t reg, uint8_t val)
{
	int ret;

	//debug("reg=0x%04x, val=0%02x", reg, val);
	camera->usb_buf[0] = val;

  	ret = libusb_control_transfer(camera->handle,
							LIBUSB_ENDPOINT_OUT | 
							LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE, 
							0x01, 0x00, reg,
							camera->usb_buf, 1, 500);
	if (ret < 0) 
	{
		debug("ov534_reg_write: write failed\n");
	}
}

static uint8_t 
ov534_reg_read(ps3eye_t *camera, uint16_t reg)
{
	int ret = 
		libusb_control_transfer(
			camera->handle,
			LIBUSB_ENDPOINT_IN|LIBUSB_REQUEST_TYPE_VENDOR|LIBUSB_RECIPIENT_DEVICE, 
			0x01, 0x00, reg,
			camera->usb_buf, 1, 500);

	//debug("reg=0x%04x, data=0x%02x", reg, usb_buf[0]);
	if (ret < 0) 
	{
		debug("ov534_reg_read: read failed\n");	
	}
	
	return camera->usb_buf[0];
}

static int 
sccb_check_status(ps3eye_t *camera)
{
	uint8_t data;
	int i;

	for (i = 0; i < 5; i++) 
	{
		data = ov534_reg_read(camera, OV534_REG_STATUS);

		switch (data) 
		{
		case 0x00:
			return 1;
		case 0x04:
			return 0;
		case 0x03:
			break;
		default:
			debug("sccb_check_status: status 0x%02x, attempt %d/5\n", data, i + 1);
		}
	}
	
	return 0;
}

static void 
sccb_reg_write(ps3eye_t *camera, uint8_t reg, uint8_t val)
{
	//debug("reg: 0x%02x, val: 0x%02x", reg, val);
	ov534_reg_write(camera, OV534_REG_SUBADDR, reg);
	ov534_reg_write(camera, OV534_REG_WRITE, val);
	ov534_reg_write(camera, OV534_REG_OPERATION, OV534_OP_WRITE_3);

	if (!sccb_check_status(camera))
	{
		debug("sccb_reg_write: failed\n");		
	}
}

static uint8_t 
sccb_reg_read(ps3eye_t *camera, uint16_t reg)
{
	ov534_reg_write(camera, OV534_REG_SUBADDR, (uint8_t)reg);
	ov534_reg_write(camera, OV534_REG_OPERATION, OV534_OP_WRITE_2);
	
	if (!sccb_check_status(camera))
	{
		debug("sccb_reg_read: failed 1\n");		
	}

	ov534_reg_write(camera, OV534_REG_OPERATION, OV534_OP_READ_2);
	if (!sccb_check_status(camera))
	{
		debug( "sccb_reg_read: failed 2\n");		
	}

	return ov534_reg_read(camera, OV534_REG_READ);
}

/* output a bridge sequence (reg - val) */
static void 
reg_w_array(ps3eye_t *camera, const uint8_t (*data)[2], int len)
{
	while (--len >= 0) 
	{
		ov534_reg_write(camera, (*data)[0], (*data)[1]);
		data++;
	}
}

/* output a sensor sequence (reg - val) */
static void 
sccb_w_array(ps3eye_t *camera, const uint8_t (*data)[2], int len)
{
	while (--len >= 0) 
	{
		if ((*data)[0] != 0xff) 
		{
			sccb_reg_write(camera, (*data)[0], (*data)[1]);
		} 
		else 
		{
			sccb_reg_read(camera, (*data)[1]);
			sccb_reg_write(camera, 0xff, 0x00);
		}
		
		data++;
	}
}

// -- USB transfer request helper methods --
static void 
_ps3eye_urb_init(ps3eye_urb_t *urb)
{
	// we allocate max possible size
	// 16 frames 
	size_t stride = 640*2;
	const size_t fsz = stride*480;
	urb->frame_buffer = (uint8_t*)malloc(fsz * 16 + 16384*2);
	urb->frame_buffer_end = urb->frame_buffer + fsz * 16;

	urb->frame_data_start = urb->frame_buffer;
	urb->frame_data_len = 0;
	urb->frame_complete_ind = 0;
	urb->frame_work_ind = 0;
	urb->frame_size = (uint32_t)fsz;
	
	urb->num_transfers= 0; 
	urb->last_packet_type= DISCARD_PACKET; 
	urb->last_pts= 0; 
	urb->last_fid= 0;	
}

static void 
_ps3eye_urb_dispose(ps3eye_urb_t *urb)
{
	debug("ps3eye_urb_dispose: Free urb data\n");
	
	if (urb->num_transfers)
	{
		_ps3eye_urb_close_transfers(urb);
	}
	
	if (urb->frame_buffer != NULL)
	{
		free(urb->frame_buffer);		
		urb->frame_buffer = NULL;		
	}	
}

static bool 
_ps3eye_urb_start_transfers(
	ps3eye_urb_t *urb, libusb_device_handle *handle, uint32_t curr_frame_size)
{
	struct libusb_transfer *xfr0,*xfr1;
	uint8_t* buff, *buff1;
	uint8_t ep_addr;
	int bsize = 16384;
	
	urb->frame_size = curr_frame_size;

	// bulk transfers
	xfr0 = libusb_alloc_transfer(0);
	xfr1 = libusb_alloc_transfer(0);

	buff = urb->frame_buffer_end;
	buff1 = buff + bsize;
	memset(urb->frame_buffer_end, 0, bsize*2);

	urb->allow_new_xfrs = true;

	urb->xfr[0] = xfr0;
	urb->xfr[1] = xfr1;

	ep_addr = _pseye_usb_find_endpoint(libusb_get_device(handle));
	//debug("found ep: %d\n", ep_addr);

	libusb_clear_halt(handle, ep_addr);

	libusb_fill_bulk_transfer(xfr0, handle, ep_addr, buff, bsize, _ps3eye_urb_transfer_callback, urb, 0);
	libusb_fill_bulk_transfer(xfr1, handle, ep_addr, buff1, bsize, _ps3eye_urb_transfer_callback, urb, 0);

	int res = libusb_submit_transfer(xfr0);
	res |= libusb_submit_transfer(xfr1);

	urb->num_transfers = 2;
	urb->frame_complete_ind = 0;
	urb->frame_work_ind = 0;
	urb->last_pts = 0;
	urb->last_fid = 0;
	urb->last_frame_time = 0;
	
	if (res < 0)
	{
		debug("_ps3eye_urb_start_transfers: failed to start transfer: \n");
		_pseye_usb_error(res);
	}

	return res == 0;
}

static void 
_ps3eye_urb_close_transfers(ps3eye_urb_t *urb)
{
	int cancel_attempts_remaining = 3;

	urb->allow_new_xfrs = false;

	if (urb->xfr[0] != NULL)
	{
		libusb_cancel_transfer(urb->xfr[0]);
	}

	if (urb->xfr[1] != NULL)
	{
		libusb_cancel_transfer(urb->xfr[1]);
	}
	
	while(urb->num_transfers > 0)
	{
		//int result= libusb_handle_events_completed(g_usb_context, &urb->num_transfers);
		struct timeval tv;
		tv.tv_sec = 0;
		tv.tv_usec = 50 * 1000; // ms

		int result = libusb_handle_events_timeout_completed(g_usb_context, &tv, NULL);

		--cancel_attempts_remaining;
		
		if( result != 0)
		{
			debug("_ps3eye_urb_close_transfers: failed to wait for transfer to complete: \n");
			_pseye_usb_error(result);	

			// Bail on waiting for the transfers to complete
			urb->num_transfers= 0;
		}
		else if (cancel_attempts_remaining <= 0)
		{
			debug("_ps3eye_urb_close_transfers: timed out waiting for transfers to complete during close \n");

			// Bail on waiting for the transfers to complete
			urb->num_transfers = 0;
		}
	}
}

static void 
_ps3eye_urb_frame_add(
	ps3eye_urb_t *urb, enum gspca_packet_type packet_type, const uint8_t *data, int len)
{
	int i;
	
	if (packet_type == FIRST_PACKET) 
	{
		urb->frame_data_start = urb->frame_buffer + urb->frame_work_ind*urb->frame_size;
		urb->frame_data_len = 0;
	} 
	else
	{
		switch(urb->last_packet_type)
		{
			case FIRST_PACKET:
			case INTER_PACKET:
				// do nothing
				break;
			case DISCARD_PACKET:
				if (packet_type == LAST_PACKET) {
					urb->last_packet_type = packet_type;
					urb->frame_data_len = 0;
				}
				return;
			case LAST_PACKET:
				return;
		}
	}

	/* append the packet to the frame buffer */
	if (len > 0)
	{
		if(urb->frame_data_len + len > urb->frame_size)
		{
			packet_type = DISCARD_PACKET;
			urb->frame_data_len = 0;
		} else {
			memcpy(urb->frame_data_start + urb->frame_data_len, data, len);
			urb->frame_data_len += len;
		}
	}

	urb->last_packet_type = packet_type;

	if (packet_type == LAST_PACKET) 
	{        
		urb->last_frame_time = _pseye_get_tick_count();
		urb->frame_complete_ind = urb->frame_work_ind;
		i = (urb->frame_work_ind + 1) & 15;
		urb->frame_work_ind = i;            
		urb->frame_data_len = 0;
		//debug("frame completed %d\n", frame_complete_ind);
	}
}

static void 
_ps3eye_urb_pkt_scan(ps3eye_urb_t *urb, uint8_t *data, int len)
{
	uint32_t this_pts;
	uint16_t this_fid;
	int remaining_len = len;
	int payload_len;

	payload_len = 2048; // bulk type
	do {
		len = MIN(remaining_len, payload_len);

		/* Payloads are prefixed with a UVC-style header.  We
		   consider a frame to start when the FID toggles, or the PTS
		   changes.  A frame ends when EOF is set, and we've received
		   the correct number of bytes. */

		/* Verify UVC header.  Header length is always 12 */
		if (data[0] != 12 || len < 12) {
			debug("bad header\n");
			goto discard;
		}

		/* Check errors */
		if (data[1] & UVC_STREAM_ERR) {
			debug("payload error\n");
			goto discard;
		}

		/* Extract PTS and FID */
		if (!(data[1] & UVC_STREAM_PTS)) {
			debug("PTS not present\n");
			goto discard;
		}

		this_pts = (data[5] << 24) | (data[4] << 16) | (data[3] << 8) | data[2];
		this_fid = (data[1] & UVC_STREAM_FID) ? 1 : 0;

		/* If PTS or FID has changed, start a new frame. */
		if (this_pts != urb->last_pts || this_fid != urb->last_fid) 
		{
			if (urb->last_packet_type == INTER_PACKET)
			{
				_ps3eye_urb_frame_add(urb, LAST_PACKET, NULL, 0);
			}
			
			urb->last_pts = this_pts;
			urb->last_fid = this_fid;
			
			_ps3eye_urb_frame_add(urb, FIRST_PACKET, data + 12, len - 12);
		} /* If this packet is marked as EOF, end the frame */
		else if (data[1] & UVC_STREAM_EOF) 
		{
			urb->last_pts = 0;
			
			if(urb->frame_data_len + len - 12 != urb->frame_size)
			{
				goto discard;
			}
			
			_ps3eye_urb_frame_add(urb, LAST_PACKET, data + 12, len - 12);
		} 
		else 
		{
			/* Add the data from this payload */
			_ps3eye_urb_frame_add(urb, INTER_PACKET, data + 12, len - 12);
		}


		/* Done this payload */
		goto scan_next;

discard:
		/* Discard data until a new frame starts. */
		_ps3eye_urb_frame_add(urb, DISCARD_PACKET, NULL, 0);
scan_next:
		remaining_len -= len;
		data += len;
	} while (remaining_len > 0);
}

static void LIBUSB_CALL 
_ps3eye_urb_transfer_callback(struct libusb_transfer *xfr)
{
    ps3eye_urb_t *urb = (ps3eye_urb_t *)xfr->user_data;
    enum libusb_transfer_status status = xfr->status;

    if (status != LIBUSB_TRANSFER_COMPLETED) 
    {
        debug("transfer status %d\n", status);

		if (urb->xfr[0] == xfr)
		{
			urb->xfr[0] = NULL;
		}
		else if (urb->xfr[1] == xfr)
		{
			urb->xfr[1] = NULL;
		}

		libusb_free_transfer(xfr);
        urb->num_transfers--;
        
        if(status != LIBUSB_TRANSFER_CANCELLED)
        {
			_ps3eye_urb_close_transfers(urb);
        }


        return;
    }

    //debug("length:%u, actual_length:%u\n", xfr->length, xfr->actual_length);

	_ps3eye_urb_pkt_scan(urb, xfr->buffer, xfr->actual_length);

	if (urb->allow_new_xfrs)
	{
		if (libusb_submit_transfer(xfr) < 0)
		{
			debug("error re-submitting URB\n");
			_ps3eye_urb_close_transfers(urb);
		}
	}
}

// -- usb helper methods --
/*
 * look for an input transfer endpoint in an alternate setting
 * libusb_endpoint_descriptor
 */
static uint8_t 
_pseye_usb_find_endpoint(struct libusb_device *device)
{
	const struct libusb_interface_descriptor *altsetting= NULL;
    const struct libusb_endpoint_descriptor *ep= NULL;
	struct libusb_config_descriptor *config= NULL;
    int i;
    uint8_t ep_addr = 0;

    libusb_get_active_config_descriptor(device, &config);

    if (!config) return 0;

    for (i = 0; i < config->bNumInterfaces; i++) 
	{
        altsetting = config->interface[i].altsetting;
		
        if (altsetting[0].bInterfaceNumber == 0) 
		{
            break;
        }
    }

	if (altsetting != NULL)
	{
		for (i = 0; i < altsetting->bNumEndpoints; i++) 
		{
			ep = &altsetting->endpoint[i];
			
			if ((ep->bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) == LIBUSB_TRANSFER_TYPE_BULK 
				&& ep->wMaxPacketSize != 0) 
			{
				ep_addr = ep->bEndpointAddress;
				break;
			}
		}		
	}

    libusb_free_config_descriptor(config);

    return ep_addr;
}

static void 
_pseye_usb_error(int error_code)
{
	switch (error_code){
	case LIBUSB_ERROR_IO:
		debug("LIBUSB_ERROR_IO\nInput/output error.\n");
		break;
	case LIBUSB_ERROR_INVALID_PARAM:
		debug("LIBUSB_ERROR_INVALID_PARAM\nInvalid parameter.\n");
		break;
	case LIBUSB_ERROR_ACCESS:
		debug("LIBUSB_ERROR_ACCESS\nAccess denied (insufficient permissions).\n");
		break;
	case LIBUSB_ERROR_NO_DEVICE:
		debug("LIBUSB_ERROR_NO_DEVICE\nNo such device (it may have been disconnected).\n");
		break;
	case LIBUSB_ERROR_NOT_FOUND:
		debug("LIBUSB_ERROR_NOT_FOUND\nEntity not found.\n");
		break;
	case LIBUSB_ERROR_BUSY:
		debug("LIBUSB_ERROR_BUSY\nResource busy.\n");
		break;
	case LIBUSB_ERROR_TIMEOUT:
		debug("LIBUSB_ERROR_TIMEOUT\nOperation timed out.\n");
		break;
	case LIBUSB_ERROR_OVERFLOW:
		debug("LIBUSB_ERROR_OVERFLOW\nOverflow.\n");
		break;
	case LIBUSB_ERROR_PIPE:
		debug("LIBUSB_ERROR_PIPE\nPipe error.\n");
		break;
	case LIBUSB_ERROR_INTERRUPTED:
		debug("LIBUSB_ERROR_INTERRUPTED\nSystem call interrupted (perhaps due to signal).\n");
		break;
	case LIBUSB_ERROR_NO_MEM:
		debug("LIBUSB_ERROR_NO_MEM\nInsufficient memory.\n");
		break;
	case LIBUSB_ERROR_NOT_SUPPORTED:
		debug("LIBUSB_ERROR_NOT_SUPPORTED\nOperation not supported or unimplemented on this platform.\n");
		break;
	case LIBUSB_ERROR_OTHER:
		debug("LIBUSB_ERROR_OTHER\nOther error.\n");
		break;
	default:
		debug("unkown error(%d)\n", error_code);
	}
}

// -- time helper methods --
// timestamps
// WIN and MAC only
static int64_t 
_pseye_get_tick_count()
{
#if defined WIN32 || defined _WIN32 || defined WINCE
    LARGE_INTEGER counter;
    QueryPerformanceCounter( &counter );
    return (int64_t)counter.QuadPart;
#else
    return (int64_t)mach_absolute_time();
#endif
}