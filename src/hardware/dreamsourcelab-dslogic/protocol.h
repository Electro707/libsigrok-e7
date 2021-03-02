/*
 * This file is part of the libsigrok project.
 *
 * Copyright (C) 2013 Bert Vermeulen <bert@biot.com>
 * Copyright (C) 2012 Joel Holdsworth <joel@airwebreathe.org.uk>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef LIBSIGROK_HARDWARE_DREAMSOURCELAB_DSLOGIC_PROTOCOL_H
#define LIBSIGROK_HARDWARE_DREAMSOURCELAB_DSLOGIC_PROTOCOL_H

#include <glib.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <libusb.h>
#include <libsigrok/libsigrok.h>
#include "libsigrok-internal.h"

#define LOG_PREFIX "dreamsourcelab-dslogic"

#define USB_INTERFACE		0
#define USB_CONFIGURATION	1

#define MAX_RENUM_DELAY_MS	3000
#define NUM_SIMUL_TRANSFERS	32
#define MAX_EMPTY_TRANSFERS	(NUM_SIMUL_TRANSFERS * 2)

#define NUM_CHANNELS		16
#define NUM_TRIGGER_STAGES	16

#define DSLOGIC_REQUIRED_VERSION_MAJOR	2
#define DSL_REQUIRED_VERSION_MINOR	 0
#define DSL_HDL_VERSION             0x0D

/* 6 delay states of up to 256 clock ticks */
#define MAX_SAMPLE_DELAY	(6 * 256)

#define VTH_ADDR 0x78
#define CTR1_ADDR 0x71
#define CTR0_ADDR 0x70
#define COMB_ADDR 0x68
#define EI2C_ADDR 0x60
#define ADCC_ADDR 0x48
#define HDL_VERSION_ADDR 0x04

/* hardware Capabilities */
#define CAPS_MODE_LOGIC (1 << 0)
#define CAPS_MODE_ANALOG (1 << 1)
#define CAPS_MODE_DSO (1 << 2)

#define CAPS_FEATURE_NONE 0
// voltage threshold
#define CAPS_FEATURE_VTH (1 << 0)
// with external buffer
#define CAPS_FEATURE_BUF (1 << 1)
// pre offset control
#define CAPS_FEATURE_PREOFF (1 << 2)
// small startup eemprom
#define CAPS_FEATURE_SEEP (1 << 3)
// zero calibration ability
#define CAPS_FEATURE_ZERO (1 << 4)
// use HMCAD1511 adc chip
#define CAPS_FEATURE_HMCAD1511 (1 << 5)
// usb 3.0
#define CAPS_FEATURE_USB30 (1 << 6)
// pogopin panel
#define CAPS_FEATURE_POGOPIN (1 << 7)
// use ADF4360-7 vco chip
#define CAPS_FEATURE_ADF4360 (1 << 8)
// 20M bandwidth limitation
#define CAPS_FEATURE_20M (1 << 9)
// use startup flash (fx3)
#define CAPS_FEATURE_FLASH (1 << 10)
// 32 channels
#define CAPS_FEATURE_LA_CH32 (1 << 11)
// auto tunning vgain
#define CAPS_FEATURE_AUTO_VGAIN (1 << 12)
/* end */

#define DSLOGIC_FPGA_FIRMWARE_5V "dreamsourcelab-dslogic-fpga-5v.fw"
#define DSLOGIC_FPGA_FIRMWARE_3V3 "dreamsourcelab-dslogic-fpga-3v3.fw"
#define DSCOPE_FPGA_FIRMWARE "dreamsourcelab-dscope-fpga.fw"
#define DSLOGIC_PRO_FPGA_FIRMWARE "dreamsourcelab-dslogic-pro-fpga.fw"
#define DSLOGIC_PLUS_FPGA_FIRMWARE "dreamsourcelab-dslogic-plus-fpga.fw"
#define DSLOGIC_BASIC_FPGA_FIRMWARE "dreamsourcelab-dslogic-basic-fpga.fw"
#define DSLOGIC_U3PRO16_FPGA_FIRMWARE "dreamsourcelab-dslogic-u3pro16-fpga.fw"

// read only
#define bmGPIF_DONE     (1 << 7)
#define bmFPGA_DONE     (1 << 6)
#define bmFPGA_INIT_B   (1 << 5)
// write only
#define bmCH_CH0        (1 << 7)
#define bmCH_COM        (1 << 6)
#define bmCH_CH1        (1 << 5)
// read/write
#define bmSYS_OVERFLOW  (1 << 4)
#define bmSYS_CLR       (1 << 3)
#define bmSYS_EN        (1 << 2)
#define bmLED_RED       (1 << 1)
#define bmLED_GREEN     (1 << 0)

#define bmNONE          0
#define bmEEWP          (1 << 0)
#define bmFORCE_RDY     (1 << 1)
#define bmFORCE_STOP    (1 << 2)
#define bmSCOPE_SET     (1 << 3)
#define bmSCOPE_CLR     (1 << 4)
#define bmBW20M_SET     (1 << 5)
#define bmBW20M_CLR     (1 << 6)

enum CHANNEL_ID {
    DSL_STREAM20x16 = 0,
    DSL_STREAM25x12,
    DSL_STREAM50x6,
    DSL_STREAM100x3,

    DSL_STREAM20x16_3DN2,
    DSL_STREAM25x12_3DN2,
    DSL_STREAM50x6_3DN2,
    DSL_STREAM100x3_3DN2,

    DSL_STREAM10x32_32_3DN2,
    DSL_STREAM20x16_32_3DN2,
    DSL_STREAM25x12_32_3DN2,
    DSL_STREAM50x6_32_3DN2,
    DSL_STREAM100x3_32_3DN2,

    DSL_STREAM50x32,
    DSL_STREAM100x30,
    DSL_STREAM250x12,
    DSL_STREAM125x16_16,
    DSL_STREAM250x12_16,
    DSL_STREAM500x6,
    DSL_STREAM1000x3,

    DSL_BUFFER100x16,
    DSL_BUFFER200x8,
    DSL_BUFFER400x4,

    DSL_BUFFER250x32,
    DSL_BUFFER500x16,
    DSL_BUFFER1000x8,

    DSL_ANALOG10x2,
    DSL_ANALOG10x2_500,

    DSL_DSO200x2,
    DSL_DSO1000x2,
};

enum OPERATION_MODE {
    LOGIC = 0,
    DSO = 1,
    ANALOG = 2,
};

static const uint64_t vdivs10to2000[] = {
    SR_mV(10),
    SR_mV(20),
    SR_mV(50),
    SR_mV(100),
    SR_mV(200),
    SR_mV(500),
    SR_V(1),
    SR_V(2),
    0,
};

static const uint64_t samplerates100[] = {
    SR_HZ(10),
    SR_HZ(20),
    SR_HZ(50),
    SR_HZ(100),
    SR_HZ(200),
    SR_HZ(500),
    SR_KHZ(1),
    SR_KHZ(2),
    SR_KHZ(5),
    SR_KHZ(10),
    SR_KHZ(20),
    SR_KHZ(40),
    SR_KHZ(50),
    SR_KHZ(100),
    SR_KHZ(200),
    SR_KHZ(400),
    SR_KHZ(500),
    SR_MHZ(1),
    SR_MHZ(2),
    SR_MHZ(4),
    SR_MHZ(5),
    SR_MHZ(10),
    SR_MHZ(20),
    SR_MHZ(25),
    SR_MHZ(50),
    SR_MHZ(100),
    0,
};

static const uint64_t samplerates400[] = {
    SR_HZ(10),
    SR_HZ(20),
    SR_HZ(50),
    SR_HZ(100),
    SR_HZ(200),
    SR_HZ(500),
    SR_KHZ(1),
    SR_KHZ(2),
    SR_KHZ(5),
    SR_KHZ(10),
    SR_KHZ(20),
    SR_KHZ(40),
    SR_KHZ(50),
    SR_KHZ(100),
    SR_KHZ(200),
    SR_KHZ(400),
    SR_KHZ(500),
    SR_MHZ(1),
    SR_MHZ(2),
    SR_MHZ(4),
    SR_MHZ(5),
    SR_MHZ(10),
    SR_MHZ(20),
    SR_MHZ(25),
    SR_MHZ(50),
    SR_MHZ(100),
    SR_MHZ(200),
    SR_MHZ(400),
    0,
};

static const uint64_t samplerates1000[] = {
    SR_HZ(10),
    SR_HZ(20),
    SR_HZ(50),
    SR_HZ(100),
    SR_HZ(200),
    SR_HZ(500),
    SR_KHZ(1),
    SR_KHZ(2),
    SR_KHZ(5),
    SR_KHZ(10),
    SR_KHZ(20),
    SR_KHZ(40),
    SR_KHZ(50),
    SR_KHZ(100),
    SR_KHZ(200),
    SR_KHZ(400),
    SR_KHZ(500),
    SR_MHZ(1),
    SR_MHZ(2),
    SR_MHZ(4),
    SR_MHZ(5),
    SR_MHZ(10),
    SR_MHZ(20),
    SR_MHZ(25),
    SR_MHZ(50),
    SR_MHZ(100),
    SR_MHZ(125),
    SR_MHZ(250),
    SR_MHZ(500),
    SR_GHZ(1),
    0,
};

enum {
    DSL_ERROR = -1,
    DSL_INIT = 0,
    DSL_START = 1,
    DSL_READY = 2,
    DSL_TRIGGERED = 3,
    DSL_DATA = 4,
    DSL_STOP = 5,
    DSL_FINISH = 7,
    DSL_ABORT = 8,
};

enum dslogic_operation_modes {
	DS_OP_NORMAL,
	DS_OP_INTERNAL_TEST,
	DS_OP_EXTERNAL_TEST,
	DS_OP_LOOPBACK_TEST,
};

enum dslogic_edge_modes {
	DS_EDGE_RISING,
	DS_EDGE_FALLING,
};

struct dslogic_version {
	uint8_t major;
	uint8_t minor;
};

struct dslogic_mode {
	uint8_t flags;
	uint8_t sample_delay_h;
	uint8_t sample_delay_l;
};

struct dslogic_trigger_pos {
	uint32_t check_id;
	uint32_t real_pos;
	uint32_t ram_saddr;
	uint32_t remain_cnt_l;
	uint32_t remain_cnt_h;
	uint32_t status;
	//uint8_t first_block[488];
};

struct dslogic_channels {
    enum CHANNEL_ID id;
    enum OPERATION_MODE mode;
    enum sr_channeltype type;
    gboolean stream;
    uint16_t num;
    uint16_t vld_num;
    uint8_t unit_bits;
    uint64_t min_samplerate;
    uint64_t max_samplerate;
    uint64_t hw_min_samplerate;
    uint64_t hw_max_samplerate;
    uint8_t pre_div;
    const char *descr;
};

struct dslogic_caps {
    uint64_t mode_caps;
    uint64_t feature_caps;
    uint64_t channels;
    uint64_t total_ch_num;
    uint64_t hw_depth;
    uint64_t dso_depth;
    uint8_t intest_channel;
    const uint64_t *vdivs;
    const uint64_t *samplerates;
    uint8_t vga_id;
    uint16_t default_channelmode;
    uint64_t default_samplerate;
    uint64_t default_samplelimit;
    uint16_t default_pwmtrans;
    uint16_t default_pwmmargin;
    uint32_t ref_min;
    uint32_t ref_max;
    uint16_t default_comb_comp;
    uint64_t half_samplerate;
    uint64_t quarter_samplerate;
};

struct dslogic_profile {
	uint16_t vid;
    uint16_t pid;
    enum libusb_speed usb_speed;

    const char *vendor;
    const char *model;
    const char *model_version;

    const char *firmware;

    struct dslogic_caps dev_caps;
};

struct dev_context {
	const struct dslogic_profile *profile;
	/*
	 * Since we can't keep track of a DSLogic device after upgrading
	 * the firmware (it renumerates into a different device address
	 * after the upgrade) this is like a global lock. No device will open
	 * until a proper delay after the last device was upgraded.
	 */
	int64_t fw_updated;

	const uint64_t *samplerates;
	int num_samplerates;

	uint64_t cur_samplerate;
	uint64_t limit_samples;
	uint64_t capture_ratio;
    uint64_t actual_bytes;

	gboolean acq_aborted;

	unsigned int sent_samples;
	int submitted_transfers;
	int empty_transfer_count;

	struct sr_context *ctx;

	uint16_t *deinterleave_buffer;

	uint16_t mode;
	uint32_t trigger_pos;
	gboolean external_clock;
	gboolean continuous_mode;
	int clock_edge;
	double cur_threshold;
    
    gboolean rle_mode;
    gboolean clock_type;
    uint16_t filter;
    gboolean stream;
    uint8_t  test_mode;
    enum CHANNEL_ID ch_mode;
    gboolean instant;
    uint64_t actual_samples;
    uint16_t unit_pitch;
    uint16_t th_level;
    
    void *cb_data;
	unsigned int num_transfers;
	struct libusb_transfer **transfers;
	int *usbfd;
    
    int trf_completed;
    int empty_poll_count;
    
    gboolean abort;
    int status;
};

struct dslogic_adc_config {
    uint8_t dest;
    uint8_t cnt;
    uint8_t delay;
    uint8_t byte[4];
};

static const struct dslogic_adc_config adc_clk_init_500m[] = {
    {ADCC_ADDR+2, 1,   0,   {0x01, 0x00, 0x00, 0x00}}, // power up
    {ADCC_ADDR,   4,   0,   {0x01, 0x61, 0x00, 0x30}}, //
    {ADCC_ADDR,   4,   0,   {0x01, 0x40, 0xF1, 0x46}}, //
    {ADCC_ADDR,   4,   10,  {0x01, 0x62, 0x3D, 0x40}}, //
    {0, 0, 0, {0, 0, 0, 0}}
};

static const struct dslogic_channels channel_modes[] = {
    // LA Stream
    {DSL_STREAM20x16,  LOGIC,  SR_CHANNEL_LOGIC,  TRUE,  16, 16, 1, SR_KHZ(10), SR_MHZ(20),
     SR_KHZ(10), SR_MHZ(100), 1, "Use 16 Channels (Max 20MHz)"},
    {DSL_STREAM25x12,  LOGIC,  SR_CHANNEL_LOGIC,  TRUE,  16, 12, 1, SR_KHZ(10), SR_MHZ(25),
     SR_KHZ(10), SR_MHZ(100), 1, "Use 12 Channels (Max 25MHz)"},
    {DSL_STREAM50x6,   LOGIC,  SR_CHANNEL_LOGIC,  TRUE,  16, 6,  1, SR_KHZ(10), SR_MHZ(50),
     SR_KHZ(10), SR_MHZ(100), 1, "Use 6 Channels (Max 50MHz)"},
    {DSL_STREAM100x3,  LOGIC,  SR_CHANNEL_LOGIC,  TRUE,  16, 3,  1, SR_KHZ(10), SR_MHZ(100),
     SR_KHZ(10), SR_MHZ(100), 1, "Use 3 Channels (Max 100MHz)"},

    {DSL_STREAM20x16_3DN2,  LOGIC,  SR_CHANNEL_LOGIC,  TRUE, 16, 16, 1, SR_KHZ(10), SR_MHZ(20),
     SR_KHZ(10), SR_MHZ(500), 5, "Use 16 Channels (Max 20MHz)"},
    {DSL_STREAM25x12_3DN2,  LOGIC,  SR_CHANNEL_LOGIC,  TRUE, 16, 12, 1, SR_KHZ(10), SR_MHZ(25),
     SR_KHZ(10), SR_MHZ(500), 5, "Use 12 Channels (Max 25MHz)"},
    {DSL_STREAM50x6_3DN2,  LOGIC,  SR_CHANNEL_LOGIC,  TRUE, 16, 6, 1, SR_KHZ(10), SR_MHZ(50),
     SR_KHZ(10), SR_MHZ(500), 5, "Use 6 Channels (Max 50MHz)"},
    {DSL_STREAM100x3_3DN2,  LOGIC,  SR_CHANNEL_LOGIC,  TRUE, 16, 3, 1, SR_KHZ(10), SR_MHZ(100),
     SR_KHZ(10), SR_MHZ(500), 5, "Use 3 Channels (Max 100MHz)"},

    {DSL_STREAM10x32_32_3DN2,  LOGIC,  SR_CHANNEL_LOGIC,  TRUE, 32, 32, 1, SR_KHZ(10), SR_MHZ(10),
     SR_KHZ(10), SR_MHZ(500), 5, "Use 32 Channels (Max 10MHz)"},
    {DSL_STREAM20x16_32_3DN2,  LOGIC,  SR_CHANNEL_LOGIC,  TRUE, 32, 16, 1, SR_KHZ(10), SR_MHZ(20),
     SR_KHZ(10), SR_MHZ(500), 5, "Use 16 Channels (Max 20MHz)"},
    {DSL_STREAM25x12_32_3DN2,  LOGIC,  SR_CHANNEL_LOGIC,  TRUE, 32, 12, 1, SR_KHZ(10), SR_MHZ(25),
     SR_KHZ(10), SR_MHZ(500), 5, "Use 12 Channels (Max 25MHz)"},
    {DSL_STREAM50x6_32_3DN2,  LOGIC,  SR_CHANNEL_LOGIC,  TRUE, 32, 6, 1, SR_KHZ(10), SR_MHZ(50),
     SR_KHZ(10), SR_MHZ(500), 5, "Use 6 Channels (Max 50MHz)"},
    {DSL_STREAM100x3_32_3DN2,  LOGIC,  SR_CHANNEL_LOGIC,  TRUE, 32, 3, 1, SR_KHZ(10), SR_MHZ(100),
     SR_KHZ(10), SR_MHZ(500), 5, "Use 3 Channels (Max 100MHz)"},

    {DSL_STREAM50x32,  LOGIC,  SR_CHANNEL_LOGIC,  TRUE, 32, 32, 1, SR_KHZ(10), SR_MHZ(50),
     SR_KHZ(10), SR_MHZ(500), 5, "Use 32 Channels (Max 50MHz)"},
    {DSL_STREAM100x30,  LOGIC,  SR_CHANNEL_LOGIC,  TRUE, 32, 30, 1, SR_KHZ(10), SR_MHZ(100),
     SR_KHZ(10), SR_MHZ(500), 5, "Use 30 Channels (Max 100MHz)"},
    {DSL_STREAM250x12,  LOGIC,  SR_CHANNEL_LOGIC,  TRUE, 32, 12, 1, SR_KHZ(10), SR_MHZ(250),
     SR_KHZ(10), SR_MHZ(500), 5, "Use 12 Channels (Max 250MHz)"},
    {DSL_STREAM125x16_16,  LOGIC,  SR_CHANNEL_LOGIC,  TRUE, 16, 16, 1, SR_KHZ(10), SR_MHZ(125),
     SR_KHZ(10), SR_MHZ(500), 5, "Use 16 Channels (Max 125MHz)"},
    {DSL_STREAM250x12_16,  LOGIC,  SR_CHANNEL_LOGIC,  TRUE, 16, 12, 1, SR_KHZ(10), SR_MHZ(250),
     SR_KHZ(10), SR_MHZ(500), 5, "Use 12 Channels (Max 250MHz)"},
    {DSL_STREAM500x6,  LOGIC,  SR_CHANNEL_LOGIC,  TRUE,  16, 6,  1, SR_KHZ(10), SR_MHZ(500),
     SR_KHZ(10), SR_MHZ(500), 5, "Use 6 Channels (Max 500MHz)"},
    {DSL_STREAM1000x3,  LOGIC,  SR_CHANNEL_LOGIC,  TRUE, 8, 3,  1, SR_KHZ(10), SR_GHZ(1),
     SR_KHZ(10), SR_MHZ(500), 5, "Use 3 Channels (Max 1GHz)"},

    // LA Buffer
    {DSL_BUFFER100x16, LOGIC,  SR_CHANNEL_LOGIC,  FALSE, 16, 16, 1, SR_KHZ(10), SR_MHZ(100),
     SR_KHZ(10), SR_MHZ(100), 1, "Use Channels 0~15 (Max 100MHz)"},
    {DSL_BUFFER200x8,  LOGIC,  SR_CHANNEL_LOGIC,  FALSE, 8, 8,  1, SR_KHZ(10), SR_MHZ(200),
     SR_KHZ(10), SR_MHZ(100), 1, "Use Channels 0~7 (Max 200MHz)"},
    {DSL_BUFFER400x4,  LOGIC,  SR_CHANNEL_LOGIC,  FALSE, 4, 4,  1, SR_KHZ(10), SR_MHZ(400),
     SR_KHZ(10), SR_MHZ(100), 1, "Use Channels 0~3 (Max 400MHz)"},

    {DSL_BUFFER250x32,  LOGIC,  SR_CHANNEL_LOGIC,  FALSE, 32, 32,  1, SR_KHZ(10), SR_MHZ(250),
     SR_KHZ(10), SR_MHZ(500), 5, "Use Channels 0~31 (Max 250MHz)"},
    {DSL_BUFFER500x16,  LOGIC,  SR_CHANNEL_LOGIC,  FALSE, 16, 16,  1, SR_KHZ(10), SR_MHZ(500),
     SR_KHZ(10), SR_MHZ(500), 5, "Use Channels 0~15 (Max 500MHz)"},
    {DSL_BUFFER1000x8,  LOGIC,  SR_CHANNEL_LOGIC,  FALSE, 8, 8,  1, SR_KHZ(10), SR_GHZ(1),
     SR_KHZ(10), SR_MHZ(500), 5, "Use Channels 0~7 (Max 1GHz)"},

//     DAQ
//     {DSL_ANALOG10x2,   ANALOG, SR_CHANNEL_ANALOG, TRUE,  2, 2,  8, SR_HZ(10),  SR_MHZ(10),
//      SR_KHZ(10), SR_MHZ(100), 1, "Use Channels 0~1 (Max 10MHz)"},
//     {DSL_ANALOG10x2_500,   ANALOG, SR_CHANNEL_ANALOG, TRUE,  2, 2,  8, SR_HZ(10),  SR_MHZ(10),
//      SR_KHZ(10), SR_MHZ(500), 1, "Use Channels 0~1 (Max 10MHz)"},
// 
//     OSC
//     {DSL_DSO200x2,     DSO,    SR_CHANNEL_DSO,    FALSE, 2, 2,  8, SR_KHZ(10), SR_MHZ(200),
//      SR_KHZ(10), SR_MHZ(100), 1, "Use Channels 0~1 (Max 200MHz)"},
//     {DSL_DSO1000x2,    DSO,    SR_CHANNEL_DSO,    FALSE, 2, 2,  8, SR_KHZ(10), SR_GHZ(1),
//      SR_KHZ(10), SR_MHZ(500), 1, "Use Channels 0~1 (Max 1GHz)"}
};

/** Device threshold level. */
enum {
    /** 1.8/2.5/3.3 level */
    SR_TH_3V3 = 0,
    /** 5.0 level */
    SR_TH_5V0 = 1,
};

enum {
    DSL_CTL_FW_VERSION		= 0,
    DSL_CTL_REVID_VERSION	= 1,
    DSL_CTL_HW_STATUS		= 2,
    DSL_CTL_PROG_B			= 3,
    DSL_CTL_SYS				= 4,
    DSL_CTL_LED				= 5,
    DSL_CTL_INTRDY			= 6,
    DSL_CTL_WORDWIDE		= 7,

    DSL_CTL_START			= 8,
    DSL_CTL_STOP			= 9,
    DSL_CTL_BULK_WR			= 10,
    DSL_CTL_REG				= 11,
    DSL_CTL_NVM				= 12,

    DSL_CTL_I2C_DSO			= 13,
    DSL_CTL_I2C_REG			= 14,
    DSL_CTL_I2C_STATUS		= 15,

    DSL_CTL_DSO_EN0			= 16,
    DSL_CTL_DSO_DC0			= 17,
    DSL_CTL_DSO_ATT0		= 18,
    DSL_CTL_DSO_EN1			= 19,
    DSL_CTL_DSO_DC1			= 20,
    DSL_CTL_DSO_ATT1		= 21,

    DSL_CTL_AWG_WR			= 22,
    DSL_CTL_I2C_PROBE		= 23,
    DSL_CTL_I2C_EXT         = 24,
};

SR_PRIV int dslogic_fpga_firmware_upload(const struct sr_dev_inst *sdi);
SR_PRIV int dslogic_set_voltage_threshold(const struct sr_dev_inst *sdi, double threshold);
SR_PRIV int dslogic_config_adc(const struct sr_dev_inst *sdi, const struct dslogic_adc_config *config);
SR_PRIV int dslogic_dev_open(struct sr_dev_inst *sdi, struct sr_dev_driver *di);
SR_PRIV struct dev_context *dslogic_dev_new(void);
SR_PRIV int dslogic_acquisition_start(const struct sr_dev_inst *sdi);
SR_PRIV int dslogic_acquisition_stop(struct sr_dev_inst *sdi);

SR_PRIV int dslogic_hdl_version(const struct sr_dev_inst *sdi, uint8_t *value);
SR_PRIV int dslogic_get_hardware_status(libusb_device_handle *devhdl, uint8_t *hw_info);
SR_PRIV int dslogic_wr_reg(const struct sr_dev_inst *sdi, uint8_t addr, uint8_t value);

#endif
