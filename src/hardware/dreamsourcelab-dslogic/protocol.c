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

#include <config.h>
#include <math.h>
#include <stdbool.h>
#include <glib.h>
#include <glib/gstdio.h>
#include <assert.h>
#include "protocol.h"

#define DS_CMD_GET_FW_VERSION		0xb0
#define DS_CMD_GET_REVID_VERSION	0xb1
#define DS_CMD_START			0xb2
#define DS_CMD_CONFIG			0xb3
#define DS_CMD_SETTING			0xb4
#define DS_CMD_CONTROL			0xb5
#define DS_CMD_STATUS			0xb6
#define DS_CMD_STATUS_INFO		0xb7
#define DS_CMD_WR_REG			0xb8
#define DS_CMD_WR_NVM			0xb9
#define DS_CMD_RD_NVM			0xba
#define DS_CMD_RD_NVM_PRE		0xbb
#define DS_CMD_GET_HW_INFO		0xbc

#define DS_START_FLAGS_STOP		(1 << 7)
#define DS_START_FLAGS_CLK_48MHZ	(1 << 6)
#define DS_START_FLAGS_SAMPLE_WIDE	(1 << 5)
#define DS_START_FLAGS_MODE_LA		(1 << 4)

#define DS_ADDR_COMB			0x68
#define DS_ADDR_EEWP			0x70
#define DS_ADDR_VTH			0x78

#define DS_MAX_LOGIC_DEPTH		SR_MHZ(16)
#define DS_MAX_LOGIC_SAMPLERATE		SR_MHZ(100)
#define DS_MAX_TRIG_PERCENT		90

#define DS_MODE_TRIG_EN			(1 << 0)
#define DS_MODE_CLK_TYPE		(1 << 1)
#define DS_MODE_CLK_EDGE		(1 << 2)
#define DS_MODE_RLE_MODE		(1 << 3)
#define DS_MODE_DSO_MODE		(1 << 4)
#define DS_MODE_HALF_MODE		(1 << 5)
#define DS_MODE_QUAR_MODE		(1 << 6)
#define DS_MODE_ANALOG_MODE		(1 << 7)
#define DS_MODE_FILTER			(1 << 8)
#define DS_MODE_INSTANT			(1 << 9)
#define DS_MODE_STRIG_MODE		(1 << 11)
#define DS_MODE_STREAM_MODE		(1 << 12)
#define DS_MODE_LPB_TEST		(1 << 13)
#define DS_MODE_EXT_TEST		(1 << 14)
#define DS_MODE_INT_TEST		(1 << 15)

// #define DSLOGIC_ATOMIC_SAMPLES		(sizeof(uint64_t) * 8)
#define DSLOGIC_ATOMIC_BYTES		sizeof(uint64_t)

#define DSLOGIC_ATOMIC_BITS 6
#define DSLOGIC_ATOMIC_SAMPLES (1 << DSLOGIC_ATOMIC_BITS)
#define DSLOGIC_ATOMIC_SIZE (1 << (DSLOGIC_ATOMIC_BITS - 3))
#define DSLOGIC_ATOMIC_MASK (0xFFFF << DSLOGIC_ATOMIC_BITS)

/*
 * The FPGA is configured with TLV tuples. Length is specified as the
 * number of 16-bit words.
 */
#define _DS_CFG(variable, wordcnt) ((variable << 8) | wordcnt)
#define DS_CFG_START			0xf5a5f5a5
#define DS_CFG_MODE			_DS_CFG(0, 1)
#define DS_CFG_DIVIDER			_DS_CFG(1, 2)
#define DS_CFG_COUNT			_DS_CFG(3, 2)
#define DS_CFG_TRIG_POS			_DS_CFG(5, 2)
#define DS_CFG_TRIG_GLB			_DS_CFG(7, 1)
#define DS_CFG_CH_EN			_DS_CFG(8, 1)
#define DS_CFG_TRIG			_DS_CFG(64, 160)
#define DS_CFG_END			0xfa5afa5a

//////////////////////////////////////////////////////////////////////
// New stuff
/* Protocol commands */
#define CMD_CTL_WR              0xb0
#define CMD_CTL_RD_PRE          0xb1
#define CMD_CTL_RD              0xb2

#define bmWR_PROG_B        (1 << 2)
#define bmWR_INTRDY        (1 << 7)
#define bmWR_WORDWIDE      (1 << 0)

#define EI2C_CTR_OFF   0x2
#define EI2C_RXR_OFF   0x3
#define EI2C_SR_OFF    0x4
#define EI2C_TXR_OFF   0x3
#define EI2C_CR_OFF    0x4
#define EI2C_SEL_OFF   0x7

#define bmEI2C_EN (1 << 7)

#define bmEI2C_STA (1 << 7)
#define bmEI2C_STO (1 << 6)
#define bmEI2C_RD (1 << 5)
#define bmEI2C_WR (1 << 4)
#define bmEI2C_NACK (1 << 3)

#define bmEI2C_RXNACK (1 << 7)
#define bmEI2C_TIP (1 << 1)

#define EI2C_AWR 0x82
#define EI2C_ARD 0x83

#define TRIG_EN_BIT 0
#define CLK_TYPE_BIT 1
#define CLK_EDGE_BIT 2
#define RLE_MODE_BIT 3
#define DSO_MODE_BIT 4
#define HALF_MODE_BIT 5
#define QUAR_MODE_BIT 6
#define ANALOG_MODE_BIT 7
#define FILTER_BIT 8
#define INSTANT_BIT 9
#define STRIG_MODE_BIT 11
#define STREAM_MODE_BIT 12
#define LPB_TEST_BIT 13
#define EXT_TEST_BIT 14
#define INT_TEST_BIT 15

/*
 * packet content check
 */
#define TRIG_CHECKID 0x55555555
#define DSO_PKTID 0xa500

#define SAMPLES_ALIGN 1023ULL

/** Device test modes. */
enum {
    /** No test mode */
    SR_TEST_NONE,
    /** Internal pattern test mode */
    SR_TEST_INTERNAL,
    /** External pattern test mode */
    SR_TEST_EXTERNAL,
    /** SDRAM loopback test mode */
    SR_TEST_LOOPBACK,
};

/** Device buffer mode */
enum {
    /** Stop immediately */
    SR_BUF_STOP = 0,
    /** Upload captured data */
    SR_BUF_UPLOAD = 1,
};



/** Device input filter. */
enum {
    /** None */
    SR_FILTER_NONE = 0,
    /** One clock cycle */
    SR_FILTER_1T = 1,
};

// Not needed as only simple trigger will be available for now
// enum {
//     SIMPLE_TRIGGER = 0,
//     ADV_TRIGGER,
//     SERIAL_TRIGGER,
// };

enum {
    SR_PKT_OK,
    SR_PKT_SOURCE_ERROR,
    SR_PKT_DATA_ERROR,
};

////////////////////////////////////

#pragma pack(push, 1)

struct version_info {
	uint8_t major;
	uint8_t minor;
};

struct cmd_start_acquisition {
	uint8_t flags;
	uint8_t sample_delay_h;
	uint8_t sample_delay_l;
};

struct fpga_config {
	uint32_t sync;

    uint16_t mode_header;                   // 0
    uint16_t mode;
    uint16_t divider_header;                // 1-2
    uint16_t div_l;
    uint16_t div_h;
    uint16_t count_header;                  // 3-4
    uint16_t cnt_l;
    uint16_t cnt_h;
    uint16_t trig_pos_header;               // 5-6
    uint16_t tpos_l;
    uint16_t tpos_h;
    uint16_t trig_glb_header;               // 7
    uint16_t trig_glb;
    uint16_t dso_count_header;              // 8-9
    uint16_t dso_cnt_l;
    uint16_t dso_cnt_h;
    uint16_t ch_en_header;                  // 10-11
    uint16_t ch_en_l;
    uint16_t ch_en_h;
    uint16_t fgain_header;                  // 12
    uint16_t fgain;

    uint16_t trig_header;                   // 64
    uint16_t trig_mask0[NUM_TRIGGER_STAGES];
    uint16_t trig_mask1[NUM_TRIGGER_STAGES];
    uint16_t trig_value0[NUM_TRIGGER_STAGES];
    uint16_t trig_value1[NUM_TRIGGER_STAGES];
    uint16_t trig_edge0[NUM_TRIGGER_STAGES];
    uint16_t trig_edge1[NUM_TRIGGER_STAGES];
    uint16_t trig_logic0[NUM_TRIGGER_STAGES];
    uint16_t trig_logic1[NUM_TRIGGER_STAGES];
    uint32_t trig_count[NUM_TRIGGER_STAGES];

    uint32_t end_sync;
};

struct DSL_setting_ext32 {
    uint32_t sync;

    uint16_t trig_header;                   // 96
    uint16_t trig_mask0[NUM_TRIGGER_STAGES];
    uint16_t trig_mask1[NUM_TRIGGER_STAGES];
    uint16_t trig_value0[NUM_TRIGGER_STAGES];
    uint16_t trig_value1[NUM_TRIGGER_STAGES];
    uint16_t trig_edge0[NUM_TRIGGER_STAGES];
    uint16_t trig_edge1[NUM_TRIGGER_STAGES];

    uint16_t align_bytes;
    uint32_t end_sync;
};

struct ctl_header {
    uint8_t dest;
    uint16_t offset;
    uint8_t size;
};
struct ctl_wr_cmd {
    struct ctl_header header;
    uint8_t data[60];
};
struct ctl_rd_cmd {
    struct ctl_header header;
    uint8_t *data;
};
#pragma pack(pop)

static void LIBUSB_CALL trigger_receive(struct libusb_transfer *transfer);

/*
 * This should be larger than the FPGA bitstream image so that it'll get
 * uploaded in one big operation. There seem to be issues when uploading
 * it in chunks.
 */
#define FW_BUFSIZE (1024 * 1024)

#define FPGA_UPLOAD_DELAY (10 * 1000)

#define USB_TIMEOUT (3 * 1000)


static int command_ctl_wr(libusb_device_handle *devhdl, struct ctl_wr_cmd cmd)
{
    int ret;

    /* Send the control command. */
    ret = libusb_control_transfer(devhdl, LIBUSB_REQUEST_TYPE_VENDOR |
            LIBUSB_ENDPOINT_OUT, CMD_CTL_WR, 0x0000, 0x0000,
            (unsigned char *)&cmd, cmd.header.size+sizeof(struct ctl_header), 3000);
    if (ret < 0) {
        sr_err("Unable to send CMD_CTL_WR command(dest:%d/offset:%d/size:%d): %s.",
               cmd.header.dest, cmd.header.offset, cmd.header.size,
               libusb_error_name(ret));
        return SR_ERR;
    }

    return SR_OK;
}

static int command_ctl_rd(libusb_device_handle *devhdl, struct ctl_rd_cmd cmd)
{
    int ret;

    /* Send the control message. */
    ret = libusb_control_transfer(devhdl, LIBUSB_REQUEST_TYPE_VENDOR |
            LIBUSB_ENDPOINT_OUT, CMD_CTL_RD_PRE, 0x0000, 0x0000,
            (unsigned char *)&cmd, sizeof(struct ctl_header), 3000);
    if (ret < 0) {
        sr_err("Unable to send CMD_CTL_RD_PRE command(dest:%d/offset:%d/size:%d): %s.",
               cmd.header.dest, cmd.header.offset, cmd.header.size,
               libusb_error_name(ret));
        return SR_ERR;
    }

    g_usleep(10*1000);

    /* Send the control message. */
    ret = libusb_control_transfer(devhdl, LIBUSB_REQUEST_TYPE_VENDOR |
        LIBUSB_ENDPOINT_IN, CMD_CTL_RD, 0x0000, 0x0000,
        (unsigned char *)cmd.data, cmd.header.size, 3000);

    if (ret < 0) {
        sr_err("Unable to send CMD_CTL_RD command: %s.",
               libusb_error_name(ret));
        return SR_ERR;
    }

    return SR_OK;
}

SR_PRIV int dslogic_hdl_version(const struct sr_dev_inst *sdi, uint8_t *value)
{
    struct sr_usb_dev_inst *usb;
    struct libusb_device_handle *hdl;
    struct ctl_rd_cmd rd_cmd;
    int ret;
    uint8_t rdata[HDL_VERSION_ADDR+1];

    usb = sdi->conn;
    hdl = usb->devhdl;

    rd_cmd.header.dest = DSL_CTL_I2C_STATUS;
    rd_cmd.header.offset = 0;
    rd_cmd.header.size = HDL_VERSION_ADDR+1;
    rd_cmd.data = rdata;
    if ((ret = command_ctl_rd(hdl, rd_cmd)) != SR_OK) {
        sr_err("Sent DSL_CTL_I2C_STATUS command failed.");
        return SR_ERR;
    }

    *value = rdata[HDL_VERSION_ADDR];
    return SR_OK;
}

SR_PRIV int dslogic_wr_reg(const struct sr_dev_inst *sdi, uint8_t addr, uint8_t value)
{
    struct sr_usb_dev_inst *usb;
    struct libusb_device_handle *hdl;
    struct ctl_wr_cmd wr_cmd;
    int ret;

    usb = sdi->conn;
    hdl = usb->devhdl;

    wr_cmd.header.dest = DSL_CTL_I2C_REG;
    wr_cmd.header.offset = addr;
    wr_cmd.header.size = 1;
    wr_cmd.data[0] = value;
    if ((ret = command_ctl_wr(hdl, wr_cmd)) != SR_OK) {
        sr_err("Sent DSL_CTL_I2C_REG command failed.");
        return SR_ERR;
    }

    return SR_OK;
}

static int dslogic_rd_reg(const struct sr_dev_inst *sdi, uint8_t addr, uint8_t *value)
{
    struct sr_usb_dev_inst *usb;
    struct ctl_rd_cmd rd_cmd;
    int ret;

    usb = sdi->conn;

    rd_cmd.header.dest = DSL_CTL_I2C_STATUS;
    rd_cmd.header.offset = addr;
    rd_cmd.header.size = 1;
    rd_cmd.data = value;
    if ((ret = command_ctl_rd(usb->devhdl, rd_cmd)) != SR_OK) {
        sr_err("Sent DSL_CTL_I2C_STATUS read command failed.");
        return SR_ERR;
    }

    return SR_OK;
}

SR_PRIV int command_get_fw_version(libusb_device_handle *devhdl,
				  struct version_info *vi)
{
    struct ctl_rd_cmd rd_cmd;
    uint8_t rd_cmd_data[2];
    int ret;
    
    rd_cmd.header.dest = DSL_CTL_FW_VERSION;
    rd_cmd.header.size = 2;
    rd_cmd.data = rd_cmd_data;
    if ((ret = command_ctl_rd(devhdl, rd_cmd)) != SR_OK) {
        sr_err("Failed to get firmware version.");
        return SR_ERR;
    }
    sr_info("Got %d.%d", rd_cmd_data[0], rd_cmd_data[1]);
    vi->major = rd_cmd_data[0];
    vi->minor = rd_cmd_data[1];
    
    return SR_OK;
}

static unsigned int enabled_channel_count(const struct sr_dev_inst *sdi)
{
	unsigned int count = 0;
	for (const GSList *l = sdi->channels; l; l = l->next) {
		const struct sr_channel *const probe = (struct sr_channel *)l->data;
		if (probe->enabled)
			count++;
	}
	return count;
}

static uint16_t enabled_channel_mask(const struct sr_dev_inst *sdi)
{
	unsigned int mask = 0;
	for (const GSList *l = sdi->channels; l; l = l->next) {
		const struct sr_channel *const probe = (struct sr_channel *)l->data;
		if (probe->enabled)
			mask |= 1 << probe->index;
	}
	return mask;
}

static size_t to_bytes_per_ms(const struct sr_dev_inst *sdi)
{
	const struct dev_context *const devc = sdi->priv;
	const size_t ch_count = enabled_channel_count(sdi);
	if (devc->continuous_mode)
		return (devc->cur_samplerate * ch_count) / (1000 * 8);


	/* If we're in buffered mode, the transfer rate is not so important,
	 * but we expect to get at least 10% of the high-speed USB bandwidth.
	 */
	return 35000000 / (1000 * 10);
}

static size_t get_buffer_size(const struct sr_dev_inst *sdi)
{
	/*
	 * The buffer should be large enough to hold 10ms of data and
	 * a multiple of the size of a data atom.
	 */
	const size_t block_size = enabled_channel_count(sdi) * 512;
	const size_t s = 10 * to_bytes_per_ms(sdi);
	if (!block_size)
		return s;
	return ((s + block_size - 1) / block_size) * block_size;
}

static int dsl_header_size(const struct dev_context *devc)
{
    int size;

    if (devc->profile->dev_caps.feature_caps & CAPS_FEATURE_USB30)
        size = SR_KB(1);
    else
        size = SR_B(512);
    return size;
}

static unsigned int get_total_buffer_time(const struct dev_context *devc)
{
    if (devc->profile->usb_speed == LIBUSB_SPEED_SUPER)
        return 40;
    else
        return 100;
}

static unsigned int get_number_of_transfers(const struct sr_dev_inst *sdi)
{
// 	/* Total buffer size should be able to hold about 100ms of data. */
// 	const unsigned int s = get_buffer_size(sdi);
// 	const unsigned int n = (100 * to_bytes_per_ms(sdi) + s - 1) / s;
// 	return (n > NUM_SIMUL_TRANSFERS) ? NUM_SIMUL_TRANSFERS : n;
    
    unsigned int n;
    struct dev_context *devc;
    devc = sdi->priv;

    #ifndef _WIN32
    /* Total buffer size should be able to hold about 100ms of data. */
    n = (devc->stream) ? ceil(get_total_buffer_time(devc) * 1.0f * to_bytes_per_ms(sdi) / get_buffer_size(sdi)) : 1;
    #else
    n = (devc->stream) ? ceil(get_total_buffer_time(devc) * 1.0f * to_bytes_per_ms(sdi) / get_buffer_size(sdi)) :
        (devc->profile->usb_speed == LIBUSB_SPEED_SUPER) ? 16 : 4;
    #endif

    if (n > NUM_SIMUL_TRANSFERS)
        return NUM_SIMUL_TRANSFERS;

    return n;
}

static unsigned int get_timeout(const struct sr_dev_inst *sdi)
{
	const size_t total_size = get_buffer_size(sdi) *
		get_number_of_transfers(sdi);
	const unsigned int timeout = total_size / to_bytes_per_ms(sdi);
	return timeout + timeout / 4; /* Leave a headroom of 25% percent. */
}

SR_PRIV int dslogic_get_hardware_status(libusb_device_handle *devhdl, uint8_t *hw_info){
    struct ctl_rd_cmd rd_cmd;
    int ret;
    
    rd_cmd.header.dest = DSL_CTL_HW_STATUS;
    rd_cmd.header.size = 1;
    *hw_info = 0;
    rd_cmd.data = hw_info;
    if ((ret = command_ctl_rd(devhdl, rd_cmd)) != SR_OK) {
        sr_err("Failed to get hardware infos.");
        return SR_ERR;
    }
    
    return SR_OK;
}

SR_PRIV int dslogic_fpga_firmware_upload(const struct sr_dev_inst *sdi)
{
	const char *name = NULL;
	uint64_t sum, filesize;
	struct sr_resource bitstream;
	struct drv_context *drvc;
	struct dev_context *devc;
	struct sr_usb_dev_inst *usb;
    struct ctl_wr_cmd wr_cmd;
    struct ctl_rd_cmd rd_cmd;
	unsigned char *buf;
	ssize_t chunksize;
	int transferred;
	int result, ret;
    uint8_t rd_cmd_data;

	drvc = sdi->driver->context;
	devc = sdi->priv;
	usb = sdi->conn;

	if (!strcmp(devc->profile->model, "DSLogic")) {
		if (devc->cur_threshold < 1.40)
			name = DSLOGIC_FPGA_FIRMWARE_3V3;
		else
			name = DSLOGIC_FPGA_FIRMWARE_5V;
	} else if (!strcmp(devc->profile->model, "DSLogic Pro")){
		name = DSLOGIC_PRO_FPGA_FIRMWARE;
	} else if (!strcmp(devc->profile->model, "DSLogic Plus")){
		name = DSLOGIC_PLUS_FPGA_FIRMWARE;
	} else if (!strcmp(devc->profile->model, "DSLogic Basic")){
		name = DSLOGIC_BASIC_FPGA_FIRMWARE;
	} else if (!strcmp(devc->profile->model, "DSCope")) {
		name = DSCOPE_FPGA_FIRMWARE;
    } else if (!strcmp(devc->profile->model, "DSLogic U3Pro16")) {
        name = DSLOGIC_U3PRO16_FPGA_FIRMWARE;
	} else {
		sr_err("Failed to select FPGA firmware.");
		return SR_ERR;
	}

	sr_dbg("Uploading FPGA firmware '%s'.", name);

	result = sr_resource_open(drvc->sr_ctx, &bitstream,
			SR_RESOURCE_FIRMWARE, name);
	if (result != SR_OK){
		return result;
    }
    
    filesize = (uint64_t)bitstream.size;

	// step0: assert PROG_B low
    wr_cmd.header.dest = DSL_CTL_PROG_B;
    wr_cmd.header.size = 1;
    wr_cmd.data[0] = ~bmWR_PROG_B;
    if ((ret = command_ctl_wr(usb->devhdl, wr_cmd)) != SR_OK){
        sr_resource_close(drvc->sr_ctx, &bitstream);
		return SR_ERR;
    }

	// step1: turn off GREEN/RED led
    wr_cmd.header.dest = DSL_CTL_LED;
    wr_cmd.header.size = 1;
    wr_cmd.data[0] = ~bmLED_GREEN & ~bmLED_RED;
    if ((ret = command_ctl_wr(usb->devhdl, wr_cmd)) != SR_OK){
        sr_resource_close(drvc->sr_ctx, &bitstream);
		return SR_ERR;
    }

	// step2: assert PORG_B high
    wr_cmd.header.dest = DSL_CTL_PROG_B;
    wr_cmd.header.size = 1;
    wr_cmd.data[0] = bmWR_PROG_B;
    if ((ret = command_ctl_wr(usb->devhdl, wr_cmd)) != SR_OK){
        sr_resource_close(drvc->sr_ctx, &bitstream);
		return SR_ERR;
    }

	// step3: wait INIT_B go high
    rd_cmd.header.dest = DSL_CTL_HW_STATUS;
    rd_cmd.header.size = 1;
	rd_cmd_data = 0;
    rd_cmd.data = &rd_cmd_data;
    while(1) {
        if ((ret = command_ctl_rd(usb->devhdl, rd_cmd)) != SR_OK){
            sr_resource_close(drvc->sr_ctx, &bitstream);
			return SR_ERR;
        }
        if (rd_cmd_data & bmFPGA_INIT_B)
            break;
    }

	// step4: send config ctl command
    wr_cmd.header.dest = DSL_CTL_INTRDY;
    wr_cmd.header.size = 1;
    wr_cmd.data[0] = (uint8_t)~bmWR_INTRDY;
    if ((ret = command_ctl_wr(usb->devhdl, wr_cmd)) != SR_OK){
        sr_resource_close(drvc->sr_ctx, &bitstream);
        return SR_ERR;
    }

    wr_cmd.header.dest = DSL_CTL_BULK_WR;
    wr_cmd.header.size = 3;
    wr_cmd.data[0] = (uint8_t)filesize;
    wr_cmd.data[1] = (uint8_t)(filesize >> 8);
    wr_cmd.data[2] = (uint8_t)(filesize >> 16);
    if ((ret = command_ctl_wr(usb->devhdl, wr_cmd)) != SR_OK) {
        sr_err("Configure FPGA error: send command fpga_config failed.");
        sr_resource_close(drvc->sr_ctx, &bitstream);
		return SR_ERR;
    }
    
	buf = g_malloc(FW_BUFSIZE);
	sum = 0;
	result = SR_OK;
	while (1) {
		chunksize = sr_resource_read(drvc->sr_ctx, &bitstream,
				buf, FW_BUFSIZE);
		if (chunksize < 0)
			result = SR_ERR;
		if (chunksize <= 0)
			break;

		if ((ret = libusb_bulk_transfer(usb->devhdl, 2 | LIBUSB_ENDPOINT_OUT,
				buf, chunksize, &transferred, USB_TIMEOUT)) < 0) {
			sr_err("Unable to configure FPGA firmware: %s.",
					libusb_error_name(ret));
			result = SR_ERR;
			break;
		}
		sum += transferred;
		sr_spew("Uploaded %" PRIu64 "/%" PRIu64 " bytes.",
			sum, bitstream.size);

		if (transferred != chunksize) {
			sr_err("Short transfer while uploading FPGA firmware.");
			result = SR_ERR;
			break;
		}
	}
	g_free(buf);
	sr_resource_close(drvc->sr_ctx, &bitstream);

	if (result == SR_OK)
		sr_dbg("FPGA firmware upload done.");

	// step6: assert INTRDY high (indicate data end)
    wr_cmd.header.dest = DSL_CTL_INTRDY;
    wr_cmd.header.size = 1;
    wr_cmd.data[0] = bmWR_INTRDY;
    if ((ret = command_ctl_wr(usb->devhdl, wr_cmd)) != SR_OK)
		return SR_ERR;

    // step7: check GPIF_DONE
    rd_cmd.header.dest = DSL_CTL_HW_STATUS;
    rd_cmd.header.size = 1;
    rd_cmd_data = 0;
    rd_cmd.data = &rd_cmd_data;
    while ((ret = command_ctl_rd(usb->devhdl, rd_cmd)) == SR_OK) {
        if (rd_cmd_data & bmGPIF_DONE) {
            break;
        }
    }

    // step8: assert INTRDY low
    wr_cmd.header.dest = DSL_CTL_INTRDY;
    wr_cmd.header.size = 1;
    wr_cmd.data[0] = (uint8_t)~bmWR_INTRDY;
    if ((ret = command_ctl_wr(usb->devhdl, wr_cmd)) != SR_OK)
        return SR_ERR;

    // step9: check FPGA_DONE bit
    rd_cmd.header.dest = DSL_CTL_HW_STATUS;
    rd_cmd.header.size = 1;
	rd_cmd_data = 0;
    rd_cmd.data = &rd_cmd_data;
    while ((ret = command_ctl_rd(usb->devhdl, rd_cmd)) == SR_OK) {
        if (rd_cmd_data & bmFPGA_DONE) {
            // step10: turn on GREEN led
            wr_cmd.header.dest = DSL_CTL_LED;
            wr_cmd.data[0] = bmLED_GREEN;
            if ((ret = command_ctl_wr(usb->devhdl, wr_cmd)) == SR_OK)
                break;
        }
    }

    // step10: recover GPIF to be wordwide
    wr_cmd.header.dest = DSL_CTL_WORDWIDE;
    wr_cmd.header.size = 1;
    wr_cmd.data[0] = bmWR_WORDWIDE;
    if ((ret = command_ctl_wr(usb->devhdl, wr_cmd)) != SR_OK) {
        sr_err("Sent DSL_CTL_WORDWIDE command failed.");
        return SR_ERR;
    }

    sr_dbg("FPGA configure done");
    return SR_OK;
}

/*
 * Get the session trigger and configure the FPGA structure
 * accordingly.
 * @return @c true if any triggers are enabled, @c false otherwise.
 */
// static bool set_trigger(const struct sr_dev_inst *sdi, struct fpga_config *cfg)
// {
// 	struct sr_trigger *trigger;
// 	struct sr_trigger_stage *stage;
// 	struct sr_trigger_match *match;
// 	struct dev_context *devc;
// 	const GSList *l, *m;
// 	const unsigned int num_enabled_channels = enabled_channel_count(sdi);
// 	int num_trigger_stages = 0;
// 
// 	int channelbit, i = 0;
// 	uint32_t trigger_point;
// 
// 	devc = sdi->priv;
// 
// 	cfg->ch_en = enabled_channel_mask(sdi);
// 
// 	for (i = 0; i < NUM_TRIGGER_STAGES; i++) {
// 		cfg->trig_mask0[i] = 0xffff;
// 		cfg->trig_mask1[i] = 0xffff;
// 		cfg->trig_value0[i] = 0;
// 		cfg->trig_value1[i] = 0;
// 		cfg->trig_edge0[i] = 0;
// 		cfg->trig_edge1[i] = 0;
// 		cfg->trig_logic0[i] = 2;
// 		cfg->trig_logic1[i] = 2;
// 		cfg->trig_count[i] = 0;
// 	}
// 
// 	trigger_point = (devc->capture_ratio * devc->limit_samples) / 100;
// 	if (trigger_point < DSLOGIC_ATOMIC_SAMPLES)
// 		trigger_point = DSLOGIC_ATOMIC_SAMPLES;
//     const uint32_t mem_depth = devc->profile->dev_caps.dso_depth;
// 	const uint32_t max_trigger_point = devc->continuous_mode ? ((mem_depth * 10) / 100) :
// 		((mem_depth * DS_MAX_TRIG_PERCENT) / 100);
// 	if (trigger_point > max_trigger_point)
// 		trigger_point = max_trigger_point;
// 	cfg->trig_pos = trigger_point & ~(DSLOGIC_ATOMIC_SAMPLES - 1);
// 
// 	if (!(trigger = sr_session_trigger_get(sdi->session))) {
// 		sr_dbg("No session trigger found");
// 		return false;
// 	}
// 
// 	for (l = trigger->stages; l; l = l->next) {
// 		stage = l->data;
// 		num_trigger_stages++;
// 		for (m = stage->matches; m; m = m->next) {
// 			match = m->data;
// 			if (!match->channel->enabled)
// 				/* Ignore disabled channels with a trigger. */
// 				continue;
// 			channelbit = 1 << (match->channel->index);
// 			/* Simple trigger support (event). */
// 			if (match->match == SR_TRIGGER_ONE) {
// 				cfg->trig_mask0[0] &= ~channelbit;
// 				cfg->trig_mask1[0] &= ~channelbit;
// 				cfg->trig_value0[0] |= channelbit;
// 				cfg->trig_value1[0] |= channelbit;
// 			} else if (match->match == SR_TRIGGER_ZERO) {
// 				cfg->trig_mask0[0] &= ~channelbit;
// 				cfg->trig_mask1[0] &= ~channelbit;
// 			} else if (match->match == SR_TRIGGER_FALLING) {
// 				cfg->trig_mask0[0] &= ~channelbit;
// 				cfg->trig_mask1[0] &= ~channelbit;
// 				cfg->trig_edge0[0] |= channelbit;
// 				cfg->trig_edge1[0] |= channelbit;
// 			} else if (match->match == SR_TRIGGER_RISING) {
// 				cfg->trig_mask0[0] &= ~channelbit;
// 				cfg->trig_mask1[0] &= ~channelbit;
// 				cfg->trig_value0[0] |= channelbit;
// 				cfg->trig_value1[0] |= channelbit;
// 				cfg->trig_edge0[0] |= channelbit;
// 				cfg->trig_edge1[0] |= channelbit;
// 			} else if (match->match == SR_TRIGGER_EDGE) {
// 				cfg->trig_edge0[0] |= channelbit;
// 				cfg->trig_edge1[0] |= channelbit;
// 			}
// 		}
// 	}
// 
// 	cfg->trig_glb = (num_enabled_channels << 4) | (num_trigger_stages - 1);
// 
// 	return num_trigger_stages != 0;
// }

static int fpga_configure(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;
	const struct sr_usb_dev_inst *const usb = sdi->conn;
    struct libusb_device_handle *hdl = usb->devhdl;
	uint8_t c[3];
	struct fpga_config cfg;
	uint16_t mode = 0;
	uint32_t divider;
	int transferred, len, ret;
    struct DSL_setting_ext32 setting_ext32;
    struct ctl_wr_cmd wr_cmd;
    struct ctl_rd_cmd rd_cmd;
    struct sr_trigger *trigger;
    uint32_t tmp_u32;
    uint64_t tmp_u64;
    gboolean qutr_trig;
    gboolean half_trig;
    uint32_t trigger_point;
    uint32_t arm_size;
    uint8_t rd_cmd_data;
    const unsigned int num_enabled_channels = enabled_channel_count(sdi);
    int num_trigger_stages = 0;
    const GSList *l;
    int i;

	sr_dbg("Configuring FPGA.");
    
//     if (!(trigger = sr_session_trigger_get(sdi->session))) {
// 		sr_dbg("No session trigger found");
// 		return false;
// 	}

    cfg.sync = 0xf5a5f5a5;
    cfg.mode_header = 0x0001;
    cfg.divider_header = 0x0102;
    cfg.count_header = 0x0302;
    cfg.trig_pos_header = 0x0502;
    cfg.trig_glb_header = 0x0701;
    cfg.dso_count_header = 0x0802;
    cfg.ch_en_header = 0x0a02;
    cfg.fgain_header = 0x0c01;
    cfg.trig_header = 0x40a0;
    cfg.end_sync = 0xfa5afa5a;
    
    setting_ext32.sync = 0xf5a5f5a5;
    setting_ext32.trig_header = 0x6060;
    setting_ext32.align_bytes = 0xffff;
    setting_ext32.end_sync = 0xfa5afa5a;

	// basic configuration
    cfg.mode =    
//                    (trigger->trigger_en << TRIG_EN_BIT) +
                   (devc->clock_type << CLK_TYPE_BIT) +
                   (devc->clock_edge << CLK_EDGE_BIT) +
                   (devc->rle_mode << RLE_MODE_BIT) +
                   ((devc->cur_samplerate == devc->profile->dev_caps.half_samplerate) << HALF_MODE_BIT) +
                   ((devc->cur_samplerate == devc->profile->dev_caps.quarter_samplerate) << QUAR_MODE_BIT) +
                   ((devc->filter == SR_FILTER_1T) << FILTER_BIT) +
                   (devc->instant << INSTANT_BIT) +
//                    ((trigger->trigger_mode == SERIAL_TRIGGER) << STRIG_MODE_BIT) +
                   ((devc->stream) << STREAM_MODE_BIT) +
                   ((devc->test_mode == SR_TEST_LOOPBACK) << LPB_TEST_BIT) +
                   ((devc->test_mode == SR_TEST_EXTERNAL) << EXT_TEST_BIT) +
                   ((devc->test_mode == SR_TEST_INTERNAL) << INT_TEST_BIT);

    // sample rate divider
    tmp_u32  = (uint32_t)ceil(channel_modes[devc->ch_mode].hw_max_samplerate * 1.0 / devc->cur_samplerate);
    devc->unit_pitch = ceil(channel_modes[devc->ch_mode].hw_min_samplerate * 1.0 / devc->cur_samplerate);
    cfg.div_h = ((tmp_u32 >= channel_modes[devc->ch_mode].pre_div) ? channel_modes[devc->ch_mode].pre_div - 1U : tmp_u32 - 1U) << 8;
    tmp_u32 = (uint32_t)ceil(tmp_u32 * 1.0 / channel_modes[devc->ch_mode].pre_div);
    cfg.div_l = tmp_u32 & 0x0000ffff;
    cfg.div_h += tmp_u32 >> 16;

    // capture counter
    tmp_u64 = devc->actual_samples;
    tmp_u64 >>= 4; // hardware minimum unit 64
    cfg.cnt_l = tmp_u64 & 0x0000ffff;
    cfg.cnt_h = tmp_u64 >> 16;
    tmp_u64 = devc->actual_samples;
    cfg.dso_cnt_l = tmp_u64 & 0x0000ffff;
    cfg.dso_cnt_h = tmp_u64 >> 16;

    // trigger position
    // must be align to minimum parallel bits
    trigger_point = (devc->capture_ratio * devc->limit_samples) / 100;
    tmp_u32 = max(trigger_point, DSLOGIC_ATOMIC_SAMPLES);
    if (devc->stream)
        tmp_u32 = min(tmp_u32, dsl_channel_depth(sdi) * 10 / 100);
    else
        tmp_u32 = min(tmp_u32, dsl_channel_depth(sdi) * DS_MAX_TRIG_PERCENT / 100);
    cfg.tpos_l = tmp_u32 & DSLOGIC_ATOMIC_MASK;
    cfg.tpos_h = tmp_u32 >> 16;
    
    trigger_point = (devc->capture_ratio * devc->limit_samples) / 100;
	if (trigger_point < DSLOGIC_ATOMIC_SAMPLES) {
		trigger_point = DSLOGIC_ATOMIC_SAMPLES;
    }
    const uint32_t mem_depth = devc->profile->dev_caps.dso_depth;
	const uint32_t max_trigger_point = devc->continuous_mode ? ((mem_depth * 10) / 100) :
		((mem_depth * DS_MAX_TRIG_PERCENT) / 100);
	if (trigger_point > max_trigger_point){
		trigger_point = max_trigger_point;
    }
// 	cfg->trig_pos = trigger_point & ~(DSLOGIC_ATOMIC_SAMPLES - 1);
    cfg.tpos_l = trigger_point & DSLOGIC_ATOMIC_MASK;
    cfg.tpos_h = trigger_point >> 16;

    for (l = trigger->stages; l; l = l->next) {
        num_trigger_stages++;
    }
    // trigger global settings
    cfg.trig_glb = ((num_enabled_channels & 0x1f) << 8) + (num_trigger_stages & 0x00ff);

    // channel enable mapping
    cfg.ch_en_l = 0;
    cfg.ch_en_h = 0;
    for (l = sdi->channels; l; l = l->next) {
        struct sr_channel *probe = (struct sr_channel *)l->data;
        if (probe->index < 16)
            cfg.ch_en_l += probe->enabled << probe->index;
        else
            cfg.ch_en_h += probe->enabled << (probe->index - 16);
    }

    // digital fgain
    for (l = sdi->channels; l; l = l->next) {
        struct sr_channel *probe = (struct sr_channel *)l->data;
        // Look into this some more. Might not be needed for logic analyzer??
//         cfg.fgain = probe->digi_fgain;
        break;
    }

    // trigger advanced configuration
    if (1) {
            //TODO: What the fuck is this shit. I shit you not this is directly from dsview
        qutr_trig = !(devc->profile->dev_caps.feature_caps & CAPS_FEATURE_USB30) && (cfg.mode & (1 << QUAR_MODE_BIT));
        half_trig = (!(devc->profile->dev_caps.feature_caps & CAPS_FEATURE_USB30) && cfg.mode & (1 << HALF_MODE_BIT)) ||
                    ((devc->profile->dev_caps.feature_caps & CAPS_FEATURE_USB30) && cfg.mode & (1 << QUAR_MODE_BIT));

        cfg.trig_mask0[0] = ds_trigger_get_mask0(TriggerStages, TriggerProbes-1, 0, qutr_trig, half_trig);
        cfg.trig_mask1[0] = ds_trigger_get_mask1(TriggerStages, TriggerProbes-1, 0, qutr_trig, half_trig);
        cfg.trig_value0[0] = ds_trigger_get_value0(TriggerStages, TriggerProbes-1, 0, qutr_trig, half_trig);
        cfg.trig_value1[0] = ds_trigger_get_value1(TriggerStages, TriggerProbes-1, 0, qutr_trig, half_trig);
        cfg.trig_edge0[0] = ds_trigger_get_edge0(TriggerStages, TriggerProbes-1, 0, qutr_trig, half_trig);
        cfg.trig_edge1[0] = ds_trigger_get_edge1(TriggerStages, TriggerProbes-1, 0, qutr_trig, half_trig);
        
        setting_ext32.trig_mask0[0] = ds_trigger_get_mask0(TriggerStages, 2*TriggerProbes-1, TriggerProbes, qutr_trig, half_trig);
        setting_ext32.trig_mask1[0] = ds_trigger_get_mask1(TriggerStages, 2*TriggerProbes-1, TriggerProbes, qutr_trig, half_trig);
        setting_ext32.trig_value0[0] = ds_trigger_get_value0(TriggerStages, 2*TriggerProbes-1, TriggerProbes, qutr_trig, half_trig);
        setting_ext32.trig_value1[0] = ds_trigger_get_value1(TriggerStages, 2*TriggerProbes-1, TriggerProbes, qutr_trig, half_trig);
        setting_ext32.trig_edge0[0] = ds_trigger_get_edge0(TriggerStages, 2*TriggerProbes-1, TriggerProbes, qutr_trig, half_trig);
        setting_ext32.trig_edge1[0] = ds_trigger_get_edge1(TriggerStages, 2*TriggerProbes-1, TriggerProbes, qutr_trig, half_trig);

        cfg.trig_logic0[0] = (trigger->trigger_logic[TriggerStages] << 1) + trigger->trigger0_inv[TriggerStages];
        cfg.trig_logic1[0] = (trigger->trigger_logic[TriggerStages] << 1) + trigger->trigger1_inv[TriggerStages];

        cfg.trig_count[0] = trigger->trigger0_count[TriggerStages];

        for (i = 1; i < NUM_TRIGGER_STAGES; i++) {
            cfg.trig_mask0[i] = 0xffff;
            cfg.trig_mask1[i] = 0xffff;
            cfg.trig_value0[i] = 0;
            cfg.trig_value1[i] = 0;
            cfg.trig_edge0[i] = 0;
            cfg.trig_edge1[i] = 0;

            setting_ext32.trig_mask0[i] = 0xffff;
            setting_ext32.trig_mask1[i] = 0xffff;
            setting_ext32.trig_value0[i] = 0;
            setting_ext32.trig_value1[i] = 0;
            setting_ext32.trig_edge0[i] = 0;
            setting_ext32.trig_edge1[i] = 0;

            cfg.trig_logic0[i] = 2;
            cfg.trig_logic1[i] = 2;

            cfg.trig_count[i] = 0;
        }
    } else {
        // This will never excute. Advanced triggering code was down here
    }

    if (!(devc->profile->usb_speed == LIBUSB_SPEED_SUPER)) {
        // set GPIF to be wordwide
        wr_cmd.header.dest = DSL_CTL_WORDWIDE;
        wr_cmd.header.size = 1;
        wr_cmd.data[0] = bmWR_WORDWIDE;
        if ((ret = command_ctl_wr(hdl, wr_cmd)) != SR_OK) {
            sr_err("Sent DSL_CTL_WORDWIDE command failed.");
            return SR_ERR;
        }
    }

    // send bulk write control command
    arm_size = sizeof(struct fpga_config) / sizeof(uint16_t);
    wr_cmd.header.dest = DSL_CTL_BULK_WR;
    wr_cmd.header.size = 3;
    wr_cmd.data[0] = (uint8_t)arm_size;
    wr_cmd.data[1] = (uint8_t)(arm_size >> 8);
    wr_cmd.data[2] = (uint8_t)(arm_size >> 16);
    if ((ret = command_ctl_wr(hdl, wr_cmd)) != SR_OK) {
        sr_err("Sent bulk write command of arm FPGA failed.");
        return SR_ERR;
    }
    // check sys_clr dessert
    rd_cmd.header.dest = DSL_CTL_HW_STATUS;
    rd_cmd.header.size = 1;
    rd_cmd_data = 0;
    rd_cmd.data = &rd_cmd_data;
    while(1) {
        if ((ret = command_ctl_rd(hdl, rd_cmd)) != SR_OK)
            return SR_ERR;
        if (rd_cmd_data & bmSYS_CLR)
            break;
    }

    // send bulk data
    // setting
    ret = libusb_bulk_transfer(hdl, 2 | LIBUSB_ENDPOINT_OUT,
                               (unsigned char *)&cfg,
                               sizeof(struct fpga_config),
                               &transferred, 1000);
    if (ret < 0) {
        sr_err("Unable to arm FPGA of dsl device: %s.",
                libusb_error_name(ret));
        return SR_ERR;
    } else if (transferred != sizeof(struct fpga_config)) {
        sr_err("Arm FPGA error: expacted transfer size %d; actually %d",
                sizeof(struct fpga_config), transferred);
        return SR_ERR;
    }
    // setting_ext32
    if (devc->profile->dev_caps.feature_caps & CAPS_FEATURE_LA_CH32) {
        ret = libusb_bulk_transfer(hdl, 2 | LIBUSB_ENDPOINT_OUT,
                                   (unsigned char *)&setting_ext32,
                                   sizeof(struct DSL_setting_ext32),
                                   &transferred, 1000);
        if (ret < 0) {
            sr_err("Unable to arm FPGA(setting_ext32) of dsl device: %s.",
                    libusb_error_name(ret));
            return SR_ERR;
        } else if (transferred != sizeof(struct DSL_setting_ext32)) {
            sr_err("Arm FPGA(setting_ext32) error: expacted transfer size %d; actually %d",
                    sizeof(struct DSL_setting_ext32), transferred);
            return SR_ERR;
        }
    }

    // assert INTRDY high (indicate data end)
    wr_cmd.header.dest = DSL_CTL_INTRDY;
    wr_cmd.header.size = 1;
    wr_cmd.data[0] = bmWR_INTRDY;
    if ((ret = command_ctl_wr(hdl, wr_cmd)) != SR_OK)
        return SR_ERR;


    // check FPGA_DONE bit
    rd_cmd.header.dest = DSL_CTL_HW_STATUS;
    rd_cmd.header.size = 1;
    rd_cmd_data = 0;
    rd_cmd.data = &rd_cmd_data;
    if ((ret = command_ctl_rd(hdl, rd_cmd)) != SR_OK)
        return SR_ERR;
    if (rd_cmd_data & bmGPIF_DONE) {
        sr_info("Arm FPGA done");
        return SR_OK;
    } else {
        return SR_ERR;
    }
}

SR_PRIV int dslogic_config_adc(const struct sr_dev_inst *sdi, const struct dslogic_adc_config *config)
{
    while(config->dest) {
        assert((config->cnt > 0) && (config->cnt <= 4));
        if (config->delay >0)
            g_usleep(config->delay*1000);
        for (int i = 0; i < config->cnt; i++) {
            dslogic_wr_reg(sdi, config->dest, config->byte[i]);
        }
        config++;
    }
    return SR_OK;
}

SR_PRIV int dslogic_set_voltage_threshold(const struct sr_dev_inst *sdi, double threshold)
{
	int ret;
	struct dev_context *const devc = sdi->priv;
	const struct sr_usb_dev_inst *const usb = sdi->conn;
	const uint8_t value = (threshold / 5.0) * 255;

	ret = dslogic_wr_reg(sdi, VTH_ADDR, value);
    if (devc->profile->dev_caps.feature_caps & CAPS_FEATURE_ADF4360) {
        dslogic_config_adc(sdi, adc_clk_init_500m);
    }

	devc->cur_threshold = threshold;

	return SR_OK;
}

SR_PRIV int dslogic_dev_open(struct sr_dev_inst *sdi, struct sr_dev_driver *di)
{
	libusb_device **devlist;
	struct sr_usb_dev_inst *usb;
	struct libusb_device_descriptor des;
	struct dev_context *devc;
	struct drv_context *drvc;
	struct version_info vi;
	int ret = SR_ERR, i, device_count;
// 	uint8_t revid;
	char connection_id[64];

	drvc = di->context;
	devc = sdi->priv;
	usb = sdi->conn;

	device_count = libusb_get_device_list(drvc->sr_ctx->libusb_ctx, &devlist);
	if (device_count < 0) {
		sr_err("Failed to get device list: %s.",
		       libusb_error_name(device_count));
		return SR_ERR;
	}

	for (i = 0; i < device_count; i++) {
		libusb_get_device_descriptor(devlist[i], &des);

		if (des.idVendor != devc->profile->vid
		    || des.idProduct != devc->profile->pid)
			continue;

		if ((sdi->status == SR_ST_INITIALIZING) ||
				(sdi->status == SR_ST_INACTIVE)) {
			/* Check device by its physical USB bus/port address. */
			if (usb_get_port_path(devlist[i], connection_id, sizeof(connection_id)) < 0)
				continue;

			if (strcmp(sdi->connection_id, connection_id))
				/* This is not the one. */
				continue;
		}

		if (!(ret = libusb_open(devlist[i], &usb->devhdl))) {
			if (usb->address == 0xff)
				/*
				 * First time we touch this device after FW
				 * upload, so we don't know the address yet.
				 */
				usb->address = libusb_get_device_address(devlist[i]);
		} else {
			sr_err("Failed to open device: %s.",
			       libusb_error_name(ret));
			ret = SR_ERR;
			break;
		}

		if (libusb_has_capability(LIBUSB_CAP_SUPPORTS_DETACH_KERNEL_DRIVER)) {
			if (libusb_kernel_driver_active(usb->devhdl, USB_INTERFACE) == 1) {
				if ((ret = libusb_detach_kernel_driver(usb->devhdl, USB_INTERFACE)) < 0) {
					sr_err("Failed to detach kernel driver: %s.",
						libusb_error_name(ret));
					ret = SR_ERR;
					break;
				}
			}
		}

		ret = command_get_fw_version(usb->devhdl, &vi);
		if (ret != SR_OK) {
			sr_err("Failed to get firmware version.");
			break;
		}

// 		ret = command_get_revid_version(sdi, &revid);
// 		if (ret != SR_OK) {
// 			sr_err("Failed to get REVID.");
// 			break;
// 		}

		/*
		 * Changes in major version mean incompatible/API changes, so
		 * bail out if we encounter an incompatible version.
		 * Different minor versions are OK, they should be compatible.
		 */
		if (vi.major != DSLOGIC_REQUIRED_VERSION_MAJOR) {
			sr_err("Expected firmware version %d.x, "
			       "got %d.%d.", DSLOGIC_REQUIRED_VERSION_MAJOR,
			       vi.major, vi.minor);
			ret = SR_ERR;
			break;
		}

		sr_info("Opened device on %d.%d (logical) / %s (physical), "
			"interface %d, firmware %d.%d.",
			usb->bus, usb->address, connection_id,
			USB_INTERFACE, vi.major, vi.minor);

// 		sr_info("Detected REVID=%d, it's a Cypress CY7C68013%s.",
// 			revid, (revid != 1) ? " (FX2)" : "A (FX2LP)");

		ret = SR_OK;

		break;
	}

	libusb_free_device_list(devlist, 1);

	return ret;
}

SR_PRIV struct dev_context *dslogic_dev_new(void)
{
	struct dev_context *devc;
    unsigned int i;

	devc = g_malloc0(sizeof(struct dev_context));
    
    for (i = 0; i < ARRAY_SIZE(channel_modes); i++)
        assert(channel_modes[i].id == i);
	
//     devc->channel = NULL;
    devc->profile = NULL;
	devc->fw_updated = 0;
    devc->cur_samplerate = 0;
    devc->limit_samples = 0;
//     devc->clock_type = FALSE;
    devc->clock_edge = FALSE;
//     devc->rle_mode = FALSE;
//     devc->instant = FALSE;
// 	devc->op_mode = OP_STREAM;
//     devc->test_mode = SR_TEST_NONE;
// 	devc->ch_mode = devc->profile->dev_caps.default_channelmode;
//     devc->stream = (devc->op_mode == OP_STREAM);
//     devc->buf_options = SR_BUF_UPLOAD;
//     devc->th_level = SR_TH_3V3;
//     devc->vth = 1.0;
//     devc->filter = SR_FILTER_NONE;
//     devc->timebase = 10000;
//     devc->trigger_slope = DSO_TRIGGER_RISING;
//     devc->trigger_source = DSO_TRIGGER_AUTO;
//     devc->trigger_hpos = 0x0;
//     devc->trigger_hrate = 0;
//     devc->trigger_holdoff = 0;
//     devc->zero = FALSE;
//     devc->zero_branch = FALSE;
//     devc->zero_comb_fgain = FALSE;
//     devc->zero_comb = FALSE;
//     devc->status = DSL_FINISH;

//     devc->mstatus_valid = FALSE;
//     devc->data_lock = FALSE;
//     devc->max_height = 0;
//     devc->trigger_margin = 8;
//     devc->trigger_channel = 0;

//     dsl_adjust_samplerate(devc);

	return devc;
}

static void abort_acquisition(struct dev_context *devc)
{
	int i;

	devc->acq_aborted = TRUE;

	for (i = devc->num_transfers - 1; i >= 0; i--) {
		if (devc->transfers[i])
			libusb_cancel_transfer(devc->transfers[i]);
	}
}

static void finish_acquisition(struct sr_dev_inst *sdi)
{
	struct dev_context *devc;

	devc = sdi->priv;

	std_session_send_df_end(sdi);

	usb_source_remove(sdi->session, devc->ctx);

	devc->num_transfers = 0;
	g_free(devc->transfers);
	g_free(devc->deinterleave_buffer);
}

static void free_transfer(struct libusb_transfer *transfer)
{
	struct sr_dev_inst *sdi;
	struct dev_context *devc;
	unsigned int i;

	sdi = transfer->user_data;
	devc = sdi->priv;

	g_free(transfer->buffer);
	transfer->buffer = NULL;
	libusb_free_transfer(transfer);

	for (i = 0; i < devc->num_transfers; i++) {
		if (devc->transfers[i] == transfer) {
			devc->transfers[i] = NULL;
			break;
		}
	}

	devc->submitted_transfers--;
	if (devc->submitted_transfers == 0)
		finish_acquisition(sdi);
}

static void resubmit_transfer(struct libusb_transfer *transfer)
{
	int ret;

	if ((ret = libusb_submit_transfer(transfer)) == LIBUSB_SUCCESS)
		return;

	sr_err("%s: %s", __func__, libusb_error_name(ret));
	free_transfer(transfer);

}

static void deinterleave_buffer(const uint8_t *src, size_t length,
	uint16_t *dst_ptr, size_t channel_count, uint16_t channel_mask)
{
	uint16_t sample;

	for (const uint64_t *src_ptr = (uint64_t*)src;
		src_ptr < (uint64_t*)(src + length);
		src_ptr += channel_count) {
		for (int bit = 0; bit != 64; bit++) {
			const uint64_t *word_ptr = src_ptr;
			sample = 0;
			for (unsigned int channel = 0; channel != 16;
				channel++) {
				const uint16_t m = channel_mask >> channel;
				if (!m)
					break;
				if ((m & 1) && ((*word_ptr++ >> bit) & UINT64_C(1)))
					sample |= 1 << channel;
			}
			*dst_ptr++ = sample;
		}
	}
}

static void send_data(struct sr_dev_inst *sdi,
	uint16_t *data, size_t sample_count)
{
	const struct sr_datafeed_logic logic = {
		.length = sample_count * sizeof(uint16_t),
		.unitsize = sizeof(uint16_t),
		.data = data
	};

	const struct sr_datafeed_packet packet = {
		.type = SR_DF_LOGIC,
		.payload = &logic
	};

	sr_session_send(sdi, &packet);
}

static void LIBUSB_CALL receive_transfer(struct libusb_transfer *transfer)
{
    struct sr_dev_inst *const sdi = transfer->user_data;
	struct dev_context *const devc = sdi->priv;
    
//     struct dev_context *devc = transfer->user_data;
//     struct sr_dev_inst *sdi = devc->cb_data;
    
	const size_t channel_count = enabled_channel_count(sdi);
	const uint16_t channel_mask = enabled_channel_mask(sdi);
	const unsigned int cur_sample_count = DSLOGIC_ATOMIC_SAMPLES *
		transfer->actual_length /
		(DSLOGIC_ATOMIC_BYTES * channel_count);

	gboolean packet_has_error = FALSE;
	unsigned int num_samples;
	int trigger_offset;

	/*
	 * If acquisition has already ended, just free any queued up
	 * transfer that come in.
	 */
	if (devc->acq_aborted) {
		free_transfer(transfer);
		return;
	}

	sr_dbg("receive_transfer(): status %s received %d bytes.",
		libusb_error_name(transfer->status), transfer->actual_length);

	/* Save incoming transfer before reusing the transfer struct. */

	switch (transfer->status) {
	case LIBUSB_TRANSFER_NO_DEVICE:
		abort_acquisition(devc);
		free_transfer(transfer);
		return;
	case LIBUSB_TRANSFER_COMPLETED:
	case LIBUSB_TRANSFER_TIMED_OUT: /* We may have received some data though. */
		break;
	default:
		packet_has_error = TRUE;
		break;
	}

	if (transfer->actual_length == 0 || packet_has_error) {
		devc->empty_transfer_count++;
		if (devc->empty_transfer_count > MAX_EMPTY_TRANSFERS) {
			/*
			 * The FX2 gave up. End the acquisition, the frontend
			 * will work out that the samplecount is short.
			 */
			abort_acquisition(devc);
			free_transfer(transfer);
		} else {
			resubmit_transfer(transfer);
		}
		return;
	} else {
		devc->empty_transfer_count = 0;
	}

	if (!devc->limit_samples || devc->sent_samples < devc->limit_samples) {
		if (devc->limit_samples && devc->sent_samples + cur_sample_count > devc->limit_samples)
			num_samples = devc->limit_samples - devc->sent_samples;
		else
			num_samples = cur_sample_count;

		/**
		 * The DSLogic emits sample data as sequences of 64-bit sample words
		 * in a round-robin i.e. 64-bits from channel 0, 64-bits from channel 1
		 * etc. for each of the enabled channels, then looping back to the
		 * channel.
		 *
		 * Because sigrok's internal representation is bit-interleaved channels
		 * we must recast the data.
		 *
		 * Hopefully in future it will be possible to pass the data on as-is.
		 */
		if (transfer->actual_length % (DSLOGIC_ATOMIC_BYTES * channel_count) != 0)
			sr_err("Invalid transfer length!");
		deinterleave_buffer(transfer->buffer, transfer->actual_length,
			devc->deinterleave_buffer, channel_count, channel_mask);

		/* Send the incoming transfer to the session bus. */
		if (devc->trigger_pos > devc->sent_samples
			&& devc->trigger_pos <= devc->sent_samples + num_samples) {
			/* DSLogic trigger in this block. Send trigger position. */
			trigger_offset = devc->trigger_pos - devc->sent_samples;
			/* Pre-trigger samples. */
			send_data(sdi, devc->deinterleave_buffer, trigger_offset);
			devc->sent_samples += trigger_offset;
			/* Trigger position. */
			devc->trigger_pos = 0;
			std_session_send_df_trigger(sdi);
			/* Post trigger samples. */
			num_samples -= trigger_offset;
			send_data(sdi, devc->deinterleave_buffer
				+ trigger_offset, num_samples);
			devc->sent_samples += num_samples;
		} else {
			send_data(sdi, devc->deinterleave_buffer, num_samples);
			devc->sent_samples += num_samples;
		}
	}

	if (devc->limit_samples && devc->sent_samples >= devc->limit_samples) {
		abort_acquisition(devc);
		free_transfer(transfer);
	} else
		resubmit_transfer(transfer);
}

static int receive_header(struct libusb_transfer *transfer)
{
// 	struct timeval tv;
// 	struct drv_context *drvc;
// 
// 	(void)fd;
// 	(void)revents;
// 
// 	drvc = (struct drv_context *)cb_data;
// 
// 	tv.tv_sec = tv.tv_usec = 0;
// 	libusb_handle_events_timeout(drvc->sr_ctx->libusb_ctx, &tv);
// 
// 	return TRUE;
    
    struct dev_context *devc;
    struct sr_datafeed_packet packet;
    struct dslogic_trigger_pos *trigger_pos;
    const struct sr_dev_inst *sdi;
    uint64_t remain_cnt;

//     packet.status = SR_PKT_OK;
    devc = transfer->user_data;
    sdi = devc->cb_data;
    trigger_pos = (struct ds_trigger_pos *)transfer->buffer;

    if (devc->status != DSL_ABORT)
        devc->status = DSL_ERROR;
    if (!devc->abort && transfer->status == LIBUSB_TRANSFER_COMPLETED &&
        trigger_pos->check_id == TRIG_CHECKID) {
        sr_info("%" PRIu64 ": receive_trigger_pos(): status %d; timeout %d; received %d bytes.",
            g_get_monotonic_time(), transfer->status, transfer->timeout, transfer->actual_length);
        remain_cnt = trigger_pos->remain_cnt_h;
        remain_cnt = (remain_cnt << 32) + trigger_pos->remain_cnt_l;
        if (transfer->actual_length == dsl_header_size(devc)) {
            if (devc->stream || remain_cnt < devc->limit_samples) {
                if ((!devc->stream || (devc->status == DSL_ABORT))) {
                    devc->actual_samples = (devc->limit_samples - remain_cnt) & ~SAMPLES_ALIGN;
                    devc->actual_bytes = devc->actual_samples / DSLOGIC_ATOMIC_SAMPLES * enabled_channel_count(sdi) * DSLOGIC_ATOMIC_SIZE;
                    devc->actual_samples = devc->actual_bytes / enabled_channel_count(sdi) * 8;
                }

                packet.type = SR_DF_TRIGGER;
                packet.payload = trigger_pos;
                sr_session_send(sdi, &packet);

                devc->status = DSL_DATA;
            }
        }
    } else if (!devc->abort) {
        sr_err("%s: trigger packet data error.", __func__);
        packet.type = SR_DF_TRIGGER;
        packet.payload = trigger_pos;
//         packet.status = SR_PKT_DATA_ERROR;
        sr_session_send(sdi, &packet);
    }

    free_transfer(transfer);
}

static int command_start_acquisition(const struct sr_dev_inst *sdi)
{
//     struct sr_usb_dev_inst *usb;
// 	struct dslogic_mode mode;
// 	int ret;
    
    struct dev_context *devc;
    struct sr_usb_dev_inst *usb;
    struct libusb_transfer *transfer;
    unsigned int i, num_transfers;
    int ret;
    unsigned char *buf;
    size_t size;
    struct dslogic_trigger_pos *trigger_pos;

    devc = sdi->priv;
    usb = sdi->conn;

// 	mode.flags = DS_START_FLAGS_MODE_LA | DS_START_FLAGS_SAMPLE_WIDE;
// 	mode.sample_delay_h = mode.sample_delay_l = 0;
// 
// 	usb = sdi->conn;
// 	ret = libusb_control_transfer(usb->devhdl, LIBUSB_REQUEST_TYPE_VENDOR |
// 			LIBUSB_ENDPOINT_OUT, DS_CMD_START, 0x0000, 0x0000,
// 			(unsigned char *)&mode, sizeof(mode), USB_TIMEOUT);
// 	if (ret < 0) {
// 		sr_err("Failed to send start command: %s.", libusb_error_name(ret));
// 		return SR_ERR;
// 	}
// 
// 	return SR_OK;
    
    sr_dbg("Starting Transfer");
    
    num_transfers = get_number_of_transfers(sdi);
    size = get_buffer_size(sdi);

    /* trigger packet transfer */
    if (!(trigger_pos = g_try_malloc0(dsl_header_size(devc)))) {
        sr_err("%s: USB trigger_pos buffer malloc failed.", __func__);
        return SR_ERR_MALLOC;
    }

    devc->transfers = g_try_malloc0(sizeof(*devc->transfers) * (num_transfers + 1));
    if (!devc->transfers) {
        sr_err("%s: USB transfer malloc failed.", __func__);
        return SR_ERR_MALLOC;
    }
    transfer = libusb_alloc_transfer(0);
    libusb_fill_bulk_transfer(transfer, usb->devhdl,
            6 | LIBUSB_ENDPOINT_IN, (unsigned char *)trigger_pos, dsl_header_size(devc),
            (libusb_transfer_cb_fn)receive_header, devc, 0);
    if ((ret = libusb_submit_transfer(transfer)) != 0) {
        sr_err("%s: Failed to submit trigger_pos transfer: %s.",
               __func__, libusb_error_name(ret));
        libusb_free_transfer(transfer);
        g_free(trigger_pos);
        devc->status = DSL_ERROR;
        return SR_ERR;
    } else {
        devc->num_transfers++;
        devc->transfers[0] = transfer;
        devc->submitted_transfers++;
    }

    /* data packet transfer */
    for (i = 1; i <= num_transfers; i++) {
        if (!(buf = g_try_malloc(size))) {
            sr_err("%s: USB transfer buffer malloc failed.", __func__);
            return SR_ERR_MALLOC;
        }
        transfer = libusb_alloc_transfer(0);
        libusb_fill_bulk_transfer(transfer, usb->devhdl,
                6 | LIBUSB_ENDPOINT_IN, buf, size,
                (libusb_transfer_cb_fn)receive_transfer, devc, 0);
        if ((ret = libusb_submit_transfer(transfer)) != 0) {
            sr_err("%s: Failed to submit transfer: %s.",
                   __func__, libusb_error_name(ret));
            libusb_free_transfer(transfer);
            g_free(buf);
            devc->status = DSL_ERROR;
            devc->abort = TRUE;
            return SR_ERR;
        }
        devc->transfers[i] = transfer;
        devc->submitted_transfers++;
        devc->num_transfers++;
    }
    
    return SR_OK;

}

static int command_stop_acquisition(const struct sr_dev_inst *sdi)
{
	struct sr_usb_dev_inst *usb;
    struct ctl_wr_cmd wr_cmd;
    struct dev_context *devc;
	int ret;

	devc = sdi->priv;
    usb = sdi->conn;

    if (!devc->abort) {
        devc->abort = TRUE;
        dslogic_wr_reg(sdi, CTR0_ADDR, bmFORCE_RDY);
    } else if (devc->status == DSL_FINISH) {
        /* Stop GPIF acquisition */
        wr_cmd.header.dest = DSL_CTL_STOP;
        wr_cmd.header.size = 0;
        if ((ret = command_ctl_wr(usb->devhdl, wr_cmd)) != SR_OK)
            sr_err("%s: Sent acquisition stop command failed!", __func__);
        else
            sr_info("%s: Sent acquisition stop command!", __func__);
    }

	return SR_OK;
}

static int start_transfers(const struct sr_dev_inst *sdi)
{
	const size_t channel_count = enabled_channel_count(sdi);
	const size_t size = get_buffer_size(sdi);
	const unsigned int num_transfers = get_number_of_transfers(sdi);
	const unsigned int timeout = get_timeout(sdi);

	struct dev_context *devc;
	struct sr_usb_dev_inst *usb;
	struct libusb_transfer *transfer;
	unsigned int i;
	int ret;
	unsigned char *buf;

	devc = sdi->priv;
	usb = sdi->conn;

	devc->sent_samples = 0;
	devc->acq_aborted = FALSE;
	devc->empty_transfer_count = 0;
	devc->submitted_transfers = 0;

	g_free(devc->transfers);
	devc->transfers = g_try_malloc0(sizeof(*devc->transfers) * num_transfers);
	if (!devc->transfers) {
		sr_err("USB transfers malloc failed.");
		return SR_ERR_MALLOC;
	}

	devc->deinterleave_buffer = g_try_malloc(DSLOGIC_ATOMIC_SAMPLES *
		(size / (channel_count * DSLOGIC_ATOMIC_BYTES)) * sizeof(uint16_t));
	if (!devc->deinterleave_buffer) {
		sr_err("Deinterleave buffer malloc failed.");
		g_free(devc->deinterleave_buffer);
		return SR_ERR_MALLOC;
	}

	devc->num_transfers = num_transfers;
	for (i = 0; i < num_transfers; i++) {
		if (!(buf = g_try_malloc(size))) {
			sr_err("USB transfer buffer malloc failed.");
			return SR_ERR_MALLOC;
		}
		transfer = libusb_alloc_transfer(0);
		libusb_fill_bulk_transfer(transfer, usb->devhdl,
				6 | LIBUSB_ENDPOINT_IN, buf, size,
				trigger_receive, (void *)sdi, timeout);
		sr_info("submitting transfer: %d", i);
		if ((ret = libusb_submit_transfer(transfer)) != 0) {
			sr_err("Failed to submit transfer: %s.",
			       libusb_error_name(ret));
			libusb_free_transfer(transfer);
			g_free(buf);
			abort_acquisition(devc);
			return SR_ERR;
		}
		devc->transfers[i] = transfer;
		devc->submitted_transfers++;
	}

// 	std_session_send_df_header(sdi);

	return SR_OK;
}

static void LIBUSB_CALL trigger_receive(struct libusb_transfer *transfer)
{
	const struct sr_dev_inst *sdi;
	struct dslogic_trigger_pos *tpos;
	struct dev_context *devc;

	sdi = transfer->user_data;
	devc = sdi->priv;
	if (transfer->status == LIBUSB_TRANSFER_CANCELLED) {
		sr_dbg("Trigger transfer canceled.");
		/* Terminate session. */
		std_session_send_df_end(sdi);
		usb_source_remove(sdi->session, devc->ctx);
		devc->num_transfers = 0;
		g_free(devc->transfers);
	} else if (transfer->status == LIBUSB_TRANSFER_COMPLETED
			&& transfer->actual_length == sizeof(struct dslogic_trigger_pos)) {
		tpos = (struct dslogic_trigger_pos *)transfer->buffer;
		sr_info("tpos real_pos %d ram_saddr %d cnt_h %d cnt_l %d", tpos->real_pos,
			tpos->ram_saddr, tpos->remain_cnt_h, tpos->remain_cnt_l);
		devc->trigger_pos = tpos->real_pos;
		g_free(tpos);
		start_transfers(sdi);
	}
	libusb_free_transfer(transfer);    
    
//     const struct sr_dev_inst *sdi;
//     struct dslogic_trigger_pos *tpos;
//     struct dev_context *devc;
//     
//     struct sr_datafeed_packet packet;
//     uint64_t remain_cnt;
// 
//     packet.status = SR_PKT_OK;
//     devc = transfer->user_data;
//     sdi = devc->cb_data;
//     trigger_pos = (struct ds_trigger_pos *)transfer->buffer;
// 
//     if (devc->status != DSL_ABORT)
//         devc->status = DSL_ERROR;
//     if (!devc->abort && transfer->status == LIBUSB_TRANSFER_COMPLETED &&
//         trigger_pos->check_id == TRIG_CHECKID) {
//         sr_info("%" PRIu64 ": receive_trigger_pos(): status %d; timeout %d; received %d bytes.",
//             g_get_monotonic_time(), transfer->status, transfer->timeout, transfer->actual_length);
//         remain_cnt = trigger_pos->remain_cnt_h;
//         remain_cnt = (remain_cnt << 32) + trigger_pos->remain_cnt_l;
//         if (transfer->actual_length == dsl_header_size(devc)) {
//             if (sdi->mode != LOGIC ||
//                 devc->stream ||
//                 remain_cnt < devc->limit_samples) {
//                 if (sdi->mode == LOGIC && (!devc->stream || (devc->status == DSL_ABORT))) {
//                     devc->actual_samples = (devc->limit_samples - remain_cnt) & ~SAMPLES_ALIGN;
//                     devc->actual_bytes = devc->actual_samples / DSLOGIC_ATOMIC_SAMPLES * dsl_en_ch_num(sdi) * DSLOGIC_ATOMIC_SIZE;
//                     devc->actual_samples = devc->actual_bytes / dsl_en_ch_num(sdi) * 8;
//                 }
// 
//                 packet.type = SR_DF_TRIGGER;
//                 packet.payload = trigger_pos;
//                 sr_session_send(sdi, &packet);
// 
//                 devc->status = DSL_DATA;
//             }
//         }
//     } else if (!devc->abort) {
//         sr_err("%s: trigger packet data error.", __func__);
//         packet.type = SR_DF_TRIGGER;
//         packet.payload = trigger_pos;
//         packet.status = SR_PKT_DATA_ERROR;
//         sr_session_send(sdi, &packet);
//     }
// 
//     free_transfer(transfer);
}

SR_PRIV int dslogic_acquisition_start(const struct sr_dev_inst *sdi)
{
	const unsigned int timeout = get_timeout(sdi);

	struct sr_dev_driver *di;
	struct drv_context *drvc;
	struct dev_context *devc;
	struct sr_usb_dev_inst *usb;
	struct dslogic_trigger_pos *tpos;
	struct libusb_transfer *transfer;
    struct ctl_wr_cmd wr_cmd;
    struct libusb_pollfd **lupfd;
	int ret, i;

	di = sdi->driver;
	drvc = di->context;
	devc = sdi->priv;
	usb = sdi->conn;

	devc->ctx = drvc->sr_ctx;
	devc->sent_samples = 0;
	devc->empty_transfer_count = 0;
	devc->acq_aborted = FALSE;

// 	usb_source_add(sdi->session, devc->ctx, timeout, receive_data, drvc);

	if ((ret = command_stop_acquisition(sdi)) != SR_OK)
		return ret;

	if ((ret = fpga_configure(sdi)) != SR_OK)
		return ret;

	if ((ret = command_start_acquisition(sdi)) != SR_OK)
		return ret;

// 	sr_dbg("Getting trigger.");
// 	tpos = g_malloc(sizeof(struct dslogic_trigger_pos));
// 	transfer = libusb_alloc_transfer(0);
// 	libusb_fill_bulk_transfer(transfer, usb->devhdl, 6 | LIBUSB_ENDPOINT_IN,
// 			(unsigned char *)tpos, sizeof(struct dslogic_trigger_pos),
// 			trigger_receive, (void *)sdi, 0);
// 	if ((ret = libusb_submit_transfer(transfer)) < 0) {
// 		sr_err("Failed to request trigger: %s.", libusb_error_name(ret));
// 		libusb_free_transfer(transfer);
// 		g_free(tpos);
// 		return SR_ERR;
// 	}
// 
// 	devc->transfers = g_try_malloc0(sizeof(*devc->transfers));
// 	if (!devc->transfers) {
// 		sr_err("USB trigger_pos transfer malloc failed.");
// 		return SR_ERR_MALLOC;
// 	}
// 	devc->num_transfers = 1;
// 	devc->submitted_transfers++;
// 	devc->transfers[0] = transfer;
    
    /* setup callback function for data transfer */
//     lupfd = libusb_get_pollfds(drvc->sr_ctx->libusb_ctx);
//     for (i = 0; lupfd[i]; i++);
//     if (!(devc->usbfd = g_try_malloc(sizeof(struct libusb_pollfd) * (i + 1))))
//     	return SR_ERR;
//     for (i = 0; lupfd[i]; i++) {
//         sr_source_add(lupfd[i]->fd, lupfd[i]->events, dsl_get_timeout(sdi), receive_data, sdi);
//         devc->usbfd[i] = lupfd[i]->fd;
//     }
//     devc->usbfd[i] = -1;
//     free(lupfd);

    wr_cmd.header.dest = DSL_CTL_START;
    wr_cmd.header.size = 0;
    if ((ret = command_ctl_wr(usb->devhdl, wr_cmd)) != SR_OK) {
        devc->status = DSL_ERROR;
        devc->abort = TRUE;
        return ret;
    }
    devc->status = DSL_START;
    
    /* Send header packet to the session bus. */
    //std_session_send_df_header(cb_data, LOG_PREFIX);
//     std_session_send_df_header(sdi, LOG_PREFIX);

	return ret;
}

SR_PRIV int dslogic_acquisition_stop(struct sr_dev_inst *sdi)
{
	command_stop_acquisition(sdi);
// 	abort_acquisition(sdi->priv);
	return SR_OK;
}
