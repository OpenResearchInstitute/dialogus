// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Opulent Voice frame-level transmission and reception code
 * 
 * Copyright (C) 2025 Open Research Institute
 * Skunkwrx and Abraxas3d
 * 
 **/


#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <iio.h>
#include <math.h>
#include <netinet/in.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>


/* helper macros */
#define MHZ(x) ((long long)(x*1000000.0 + .5))
#define GHZ(x) ((long long)(x*1000000000.0 + .5))

/* Operating frequencies */
#define CHANNEL_BITRATE (54200)		// OPV bit rate
#define CHANNEL_CENTER MHZ(905.05)	// a legit channel in USA band plan
#define CHANNEL_IF_SPACING (32)		// somewhat arbitrary choice
#define IF_FREQUENCY (CHANNEL_BITRATE * CHANNEL_IF_SPACING / 4)		// equals 433_600
#define LO_FREQ (CHANNEL_CENTER + IF_FREQUENCY)		// Channel is lower sideband from the LO
#define RF_BANDWIDTH MHZ(1)	// Must be at least the signal bandwidth + twice the IF_FREQUENCY

#define IIO_ENSURE(expr) { \
	if (!(expr)) { \
		(void) fprintf(stderr, "assertion failed (%s:%d)\n", __FILE__, __LINE__); \
		(void) abort(); \
	} \
}

// Sizes for encapsulated Opulent Voice frames
#define OVP_SINGLE_FRAME_SIZE 134	// Opulent Voice Protocol Packet Size

// Opulent Voice Protocol constants
#define OVP_MAGIC_BYTES 0xBBAADD
#define OVP_HEADER_SIZE 12
#define OVP_PAYLOAD_SIZE 122
#define OVP_UDP_PORT 57372
#define OVP_FRAME_PERIOD_MS 40	// Fixed 40ms timing

// Sizes for final over-the-air frames sent to MSK modulator
// Assuming that the modulator only adds the frame sync word and does no other processing
#define OVP_ENCODED_HEADER_SIZE (OVP_HEADER_SIZE * 2)	// Header expands by 2x due to FEC coding
#define OVP_ENCODED_PAYLOAD_SIZE (OVP_PAYLOAD_SIZE * 2)	// Payload expands by 2x due to FEC coding

#define OVP_MODULATOR_FRAME_SIZE (OVP_ENCODED_HEADER_SIZE + OVP_ENCODED_PAYLOAD_SIZE)
#define OVP_MODULATOR_PAYLOAD_OFFSET (OVP_ENCODED_HEADER_SIZE)

// Sizes for frames returned by MSK demodulator
// Assuming that the demod only strips off the frame sync word and does no other processing
#define OVP_DEMOD_FRAME_SIZE (OVP_ENCODED_HEADER_SIZE + OVP_ENCODED_PAYLOAD_SIZE)
#define OVP_DEMOD_PAYLOAD_OFFSET (OVP_ENCODED_HEADER_SIZE)

#define TX_SYNC_CTRL_WORD 0x00000000
#define TX_SYNC_COUNT (54200 * 20)	// long preamble for test detection

// Offsets into either DMAC (TX or RX) for registers in that block.
// All the registers are defined in /hdl/library/axi_dmac/index.html
#define DMAC_PERIPHERAL_ID 0x0004
#define DMAC_SCRATCH 0x0008
#define DMAC_INTERFACE_DESCRIPTION_1 0x0010
#define DMAC_IRQ_MASK 0x0080
#define DMAC_IRQ_PENDING 0x0084
#define DMAC_IRQ_SOURCE 0x0088
#define DMAC_FLAGS 0x040c

// For the MSK registers, we use RDL headers
#include "msk_top_regs.h"


//-=-=-=-=-=-=-= GLOBAL VARIABLES -=-=-=-=-=-=-=-=
// Don't @ me bro! At least it's not GOTOs

// one bit time is 19 microseconds
float num_microseconds = 5*20;
float one_bit_time = 19;
float percent_error = 55.0;
pthread_t ovp_debug_thread;

// -=-=-=-=-=-=- Opulent Voice Global Variables =-=-=-=-=-=-=-
// OVP UDP Interface
static int ovp_udp_socket = -1;
static volatile int ovp_transmission_active = 0;
static struct sockaddr_in ovp_listen_addr;
static pthread_t ovp_udp_thread;
static volatile int ovp_running = 0;
static pthread_mutex_t timeline_lock = PTHREAD_MUTEX_INITIALIZER;	// shared by all threads
static bool encapsulated_frame_host_known = false;

// UDP connection used for encapsulated frame packets in both direction
static struct sockaddr_in udp_client_addr;
static socklen_t udp_client_len;
static ssize_t udp_bytes_received;

// OVP periodic reporting
static pthread_t ovp_reporter_thread;

// OVP transmit timeline manager
int64_t decision_time = 0;		// us timestamp after which a new frame is late
int ovp_txbufs_this_frame = 0;	// number of UDP frames seen before decision time (should be 1)
static pthread_t ovp_timeline_thread;
static pthread_cond_t timeline_start = PTHREAD_COND_INITIALIZER;
static pthread_mutex_t tls_lock = PTHREAD_MUTEX_INITIALIZER;

// OVP transmit session management variables
static unsigned char active_station_id_binary[6] = {0,0,0,0,0,0};
static char active_station_id_ascii[11] = "";	// 10 chars + null terminator is max possible
static int hang_timer_active = 0;
static int dummy_frames_sent = 0;	// Count of dummy frames sent in this hang time
static int hang_timer_frames = 25;	// Number of dummy frames before ending session
static uint32_t session_ts_base = 0;	// timestamp at start of a session
static int64_t session_T0 = 0;	// Origin of time for this session (microseconds)

static uint8_t randomization_sequence[OVP_SINGLE_FRAME_SIZE] = {
	163, 129,  92, 196, 201,   8,  14,  83, 204, 161,
	251,  41, 158,  79,  22, 224, 151,  78,  43,  87,
	 18, 167,  63, 194,  77, 107,  15,   8,  48,  70,
	 17,  86,  13,  26,  19, 231,  80, 151,  97, 243,
	190, 227, 153, 176, 100,  57,  34,  44, 240,   9,

	225, 134, 207, 115,  89, 194,  92, 142, 227, 215,
	 63, 112, 212,  39, 194, 224, 129, 146, 218, 252,
	202,  90, 128,  66, 131,  21,  15, 162, 158,  21,
	156, 139, 219, 164,  70,  28,  16, 159, 179,  71,
	108,  94,  21,  18,  31, 173,  56,  61,   3, 186,

	144, 141, 190, 211, 101,  35,  50, 184, 171,  16,
	 98, 126, 198,  38, 124,  19, 201, 101,  61,  21,
	 21, 237,  53, 244,  87, 245,  88,  17, 157, 142,
	232,  52, 201,  89
};

// OVP Frame Buffer for transmit (sized for actual frames)
static uint8_t ovp_frame_buffer[OVP_SINGLE_FRAME_SIZE];

// OVP modulator frame buffer
// This buffer is used whenever we build a frame to send to the
// modulator. It's global to simplify the building of dummy frames
// and postamble frames, which reuse the sync word and frame header
// from the preceding Opulent Voice frames.
static uint8_t modulator_frame_buffer[OVP_MODULATOR_FRAME_SIZE];

// OVP receiver thread
static pthread_t ovp_rx_thread;

// OVP Statistics
static uint64_t ovp_frames_received = 0;
static uint64_t ovp_frames_processed = 0;
static uint64_t ovp_frame_errors = 0;
static uint64_t ovp_sessions_started = 0;
static uint64_t ovp_sessions_ended = 0;
static uint64_t ovp_dummy_frames_sent = 0;
static uint64_t ovp_untimely_frames = 0;

// Opulent Voice Protocol functions
void start_transmission_session(void);
void end_transmission_session_normally(void);
void abort_transmission_session(void);
void stop_ovp_listener(void);
void stop_timeline_manager(void);
void stop_periodic_statistics_reporter(void);
void stop_ovp_receiver(void);
void print_ovp_statistics(void);
int enable_msk_transmission(void);
int disable_msk_transmission(void);
int process_and_send_ovp_frame_to_txbuf(uint8_t *frame_data, size_t frame_size);
void encode_ovp_header(uint8_t *input, uint8_t *output);
void encode_ovp_payload(uint8_t *input, uint8_t *output);
void decode_ovp_header(uint8_t *input, uint8_t *output);
void decode_ovp_payload(uint8_t *input, uint8_t *output);
void create_dummy_frame(void);
void create_postamble_frame(void);
void stop_debug_thread(void);
void dump_bytes(char *name, uint8_t *buf, size_t length);
void dump_buffer(char *name, struct iio_buffer *buf);


/* Register addresses for using the hardware timer */
#define PERIPH_BASE 0xf8f00000
#define GLOBAL_TMR_UPPER_OFFSET 0x0204
#define GLOBAL_TMR_LOWER_OFFSET 0x0200
/* Global Timer runs on the CPU clock, divided by 2 */
#define COUNTS_PER_SECOND (666666687 / 2 / 2)
static uint32_t *timer_register_map;
/* Collect telementry and make decisions after this duration */
#define REPORTING_INTERVAL (COUNTS_PER_SECOND / 1000)


//read from a memory mapped register
unsigned int read_mapped_reg(unsigned int *virtual_addr, int offset)
{
		return virtual_addr[offset>>2];
}

// Addresses of the DMACs via their AXI lite control interface register blocks
unsigned int *tx_dmac_register_map;
unsigned int *rx_dmac_register_map;

// Address of the MSK block via its AXI lite control interface register block
static msk_top_regs_t *msk_register_map = NULL;

//read a value from the MSK register
//value = READ_MSK(MSK_Init);
#define READ_MSK(offset) (msk_register_map->offset)

//write to a memory mapped register
unsigned int write_mapped_reg(unsigned int *virtual_addr, int offset, unsigned int value)
{
	virtual_addr[offset>>2] = value;
	return 0;
}

//from devmem in sbin, we know: read_result = *(volatile uint32_t*)virt_addr;
//write a value to an MSK register
//WRITE_MSK(MSK_Init, 0x00000001);
#define WRITE_MSK(offset, value) *(volatile uint32_t*)&(msk_register_map->offset) = value

// Some registers require synchronization across clock domains.
// This is implemented by first writing (any value) and then reading the register.
// The current value is captured, triggered by the write, and can then be read safely.
// power = capture_and_read_msk(OFFSET_MSK(rx_power));
uint32_t capture_and_read_msk(size_t offset) {
	volatile uint32_t *reg_ptr;
	volatile uint32_t dummy_read __attribute__((unused));
	
	reg_ptr = (volatile uint32_t *)((char *)msk_register_map + offset);

	// write to capture
	*reg_ptr = 0xDEADBEEF;	// write a distinctive value in case it shows up in a read

	// Trusting that the CPU is slow enough to make this safe across clock domains
	// without any artificial delay here.
	// or maybe not!!!
	dummy_read = *reg_ptr;

	// read actual value
	return *reg_ptr;
}

//get the address offset of an MSK register
//value = OFFSET_MSK(MSK_Init);
#define OFFSET_MSK(offset) (offsetof(msk_top_regs_t, offset))

// Timestamp facility, hardware locked to the sample clock
uint64_t get_timestamp(void) {
	uint32_t high, low;

	// Reading global timer counter register
	/* This is the method used in the library code for XTime_GetTime().
	   It handles the case where the first read of the two timer regs
	   spans a carry between the two timer words. */
	do {
		high = read_mapped_reg(timer_register_map, GLOBAL_TMR_UPPER_OFFSET);
		low = read_mapped_reg(timer_register_map, GLOBAL_TMR_LOWER_OFFSET);
		// printf("%08x %08x\n", high, low);
	} while (read_mapped_reg(timer_register_map, GLOBAL_TMR_UPPER_OFFSET) != high);
	return((((uint64_t) high) << 32U) | (uint64_t) low);
}

uint32_t get_timestamp_ms(void) {
	uint64_t ts = get_timestamp();
	double ts_seconds = ts / (double)COUNTS_PER_SECOND;
	double ts_ms = ts_seconds * 1000.0;
	return (uint32_t)ts_ms;
}

uint64_t get_timestamp_us(void) {
	uint64_t ts = get_timestamp();
	double ts_seconds = ts / (double)COUNTS_PER_SECOND;
	double ts_us = ts_seconds * 1000000.0;
	return (uint64_t)ts_us;
}

void print_timestamp(void) {
	printf("timestamp: %f\n", get_timestamp() / (double)COUNTS_PER_SECOND);
}

/* RX is input from the antenna, TX is output to the antenna */
enum iodev { RX, TX };

/* common RX and TX streaming params */
struct stream_cfg {
	long long bw_hz;	// Analog bandwidth in Hz
	long long fs_hz;	// Baseband sample rate in Hz
	long long lo_hz;	// Local oscillator frequency in Hz
	const char* rf_port;	// Port name
};

/* static scratch mem for strings */
static char tmpstr[64];

/* IIO structs required for streaming */
static struct iio_context *ctx   = NULL;
static struct iio_channel *rx0_i = NULL;
static struct iio_channel *rx0_q = NULL;
static struct iio_channel *tx0_i = NULL;
static struct iio_channel *tx0_q = NULL;
static struct iio_buffer  *rxbuf = NULL;
static struct iio_buffer  *txbuf = NULL;

static bool stop;

/* cleanup and exit */
static void cleanup_and_exit(void)
{
	// End any active transmission session
	if (ovp_transmission_active) {
		abort_transmission_session();
	}

	stop_ovp_listener();
	stop_timeline_manager();
	stop_periodic_statistics_reporter();
	stop_ovp_receiver();

	printf("\nFinal statistics report:\n");
	print_ovp_statistics();	// make sure one last report goes out, even if it's a duplicate

	printf("* Destroying buffers\n");
	if (rxbuf) {
		iio_buffer_cancel(rxbuf);
		iio_buffer_destroy(rxbuf);
	}
	if (txbuf) {
		iio_buffer_cancel(txbuf);
		iio_buffer_destroy(txbuf);
	}

	printf("* Disabling streaming channels\n");
	if (rx0_i) { iio_channel_disable(rx0_i); }
	if (rx0_q) { iio_channel_disable(rx0_q); }
	if (tx0_i) { iio_channel_disable(tx0_i); }
	if (tx0_q) { iio_channel_disable(tx0_q); }

	printf("* Destroying context\n");
	if (ctx) { iio_context_destroy(ctx); }
	exit(0);
}

// We want the console user to be able to interrupt processing by pressing Ctrl-C,
// or by sending the equivalent signal, but we don't want that to crash this
// program abruptly. So we catch the signal and give the threads a chance
// to shut down cleanly. This is the signal handler to do that.
static void handle_sig(int sig)
{
	printf("Got signal %d, shutting down ...\n", sig);
	stop = true;
}

/* check return value of attr_write function */
static void errchk(int v, const char* what) {
	 if (v < 0) {
		fprintf(stderr, "Error %d writing to channel \"%s\"\nvalue may not be supported.\n", v, what);
		cleanup_and_exit();
	}
}

/* write attribute: long long int */
static void wr_ch_lli(struct iio_channel *chn, const char* what, long long val)
{
	errchk(iio_channel_attr_write_longlong(chn, what, val), what);
}

/* write attribute: string */
static void wr_ch_str(struct iio_channel *chn, const char* what, const char* str)
{
	errchk(iio_channel_attr_write(chn, what, str), what);
}

/* helper function generating channel names */
static char* get_ch_name(const char* type, int id)
{
	snprintf(tmpstr, sizeof(tmpstr), "%s%d", type, id);
	return tmpstr;
}

/* returns ad9361 phy device */
static struct iio_device* get_ad9361_phy(void)
{
	struct iio_device *dev = iio_context_find_device(ctx, "ad9361-phy");
	IIO_ENSURE(dev && "No ad9361-phy found");
	return dev;
}

/* finds AD9361 streaming IIO devices */
static bool get_ad9361_stream_dev(enum iodev d, struct iio_device **dev)
{
	switch (d) {
	case TX: *dev = iio_context_find_device(ctx, "cf-ad9361-dds-core-lpc"); return *dev != NULL;
	case RX: *dev = iio_context_find_device(ctx, "cf-ad9361-lpc"); return *dev != NULL;
	default: IIO_ENSURE(0); return false;
	}
}

/* finds AD9361 streaming IIO channels */
static bool get_ad9361_stream_ch(enum iodev d, struct iio_device *dev, int chid, struct iio_channel **chn)
{
	*chn = iio_device_find_channel(dev, get_ch_name("voltage", chid), d == TX);
	if (!*chn)
		*chn = iio_device_find_channel(dev, get_ch_name("altvoltage", chid), d == TX);
	return *chn != NULL;
}

/* finds AD9361 phy IIO configuration channel with id chid */
static bool get_phy_chan(enum iodev d, int chid, struct iio_channel **chn)
{
	switch (d) {
	case RX: *chn = iio_device_find_channel(get_ad9361_phy(), get_ch_name("voltage", chid), false); return *chn != NULL;
	case TX: *chn = iio_device_find_channel(get_ad9361_phy(), get_ch_name("voltage", chid), true);  return *chn != NULL;
	default: IIO_ENSURE(0); return false;
	}
}

/* finds AD9361 local oscillator IIO configuration channels */
static bool get_lo_chan(enum iodev d, struct iio_channel **chn)
{
	switch (d) {
	 // LO chan is always output, i.e. true
	case RX: *chn = iio_device_find_channel(get_ad9361_phy(), get_ch_name("altvoltage", 0), true); return *chn != NULL;
	case TX: *chn = iio_device_find_channel(get_ad9361_phy(), get_ch_name("altvoltage", 1), true); return *chn != NULL;
	default: IIO_ENSURE(0); return false;
	}
}

/* applies streaming configuration through IIO */
bool cfg_ad9361_streaming_ch(struct stream_cfg *cfg, enum iodev type, int chid)
{
	struct iio_channel *chn = NULL;

	// Configure phy and lo channels
	printf("* Acquiring AD9361 phy channel %d\n", chid);
	if (!get_phy_chan(type, chid, &chn)) {	return false; }
	wr_ch_str(chn, "rf_port_select",     cfg->rf_port);
	wr_ch_lli(chn, "rf_bandwidth",       cfg->bw_hz);
	wr_ch_lli(chn, "sampling_frequency", cfg->fs_hz);

	// Configure LO channel
	printf("* Acquiring AD9361 %s lo channel\n", type == TX ? "TX" : "RX");
	if (!get_lo_chan(type, &chn)) { return false; }
	wr_ch_lli(chn, "frequency", cfg->lo_hz);
	return true;
}

// print an indication of received signal strength (RSSI)
void print_rssi(void) {
	char rssi_buffer[256];

	static struct iio_channel *my_dev_ch = NULL;
	if (my_dev_ch == NULL) {
		my_dev_ch = iio_device_find_channel(get_ad9361_phy(),"voltage0", false);
	}
	
	int ret = iio_channel_attr_read(my_dev_ch, "rssi", rssi_buffer, sizeof(rssi_buffer));
	if (ret < 0) {
		char rssi_error[256];
		iio_strerror(-(int)ret, rssi_error, sizeof(rssi_error));
		printf("iio_channel_attr_read(channel, rssi, rssi_buffer, size of rssi_buffer) failed : %s\n", rssi_error);
		printf("rssi: 9999.\n");	// for the output parser
	} else {
		printf("rssi: %s.\n", rssi_buffer);
	}
}

// -=-=-=-= Opulent Voice Functions =-=-=-=-=-=-

/* Decode a base-40 encoded callsign to its text representation
 *
 * encoded -- array of 6 bytes encoded as specified for Opulent Voice
 * buffer  -- array of 11 chars to receive the decoded station ID
 */
void decode_station_id(unsigned char *encoded, char *buffer) /* buffer[11] */
{
	static const char callsign_map[] = "xABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-/.";
	
	char result[11];
	
	uint64_t numeric_id = ((uint64_t)encoded[0] << 40) \
						+ ((uint64_t)encoded[1] << 32) \
						+ ((uint64_t)encoded[2] << 24) \
						+ ((uint64_t)encoded[3] << 16) \
						+ ((uint64_t)encoded[4] << 8 )  \
						+  (uint64_t)encoded[5];

	// decode each base-40 digit and map them to the appropriate character.
	size_t index = 0;
	while (numeric_id > 0)
	{
		result[index++] = callsign_map[numeric_id % 40];
		numeric_id /= 40;
	}
	result[index] = 0x00;

	strncpy(buffer, result, 11);

}

// Send OVP frame data to MSK modulator via IIO buffer in two steps.
// Frame data should already be formatted with scrambling + FEC coding
// or filled out as a special (preamble/postamble/dummy) frame.
// Step 1: move the data into txbuf using iio calls
// (Step 2: iio_push txbuf to the kernel)
// This is step 1.
int load_ovp_frame_into_txbuf(uint8_t *frame_data, size_t frame_size) {
	if (!txbuf) {
		printf("OVP: Error - TX buffer not initialized\n");
		return -1;
	}
	
	if (!frame_data || frame_size != OVP_MODULATOR_FRAME_SIZE) {
		printf("OVP: Error - invalid frame data\n");
		return -1;
	}
	
	printf("OVP: Sending %zu bytes to MSK modulator\n", frame_size);
	
	// Get buffer pointers and step size
	char *p_dat, *p_end;
	ptrdiff_t p_inc;
	
	p_inc = iio_buffer_step(txbuf);
	p_end = iio_buffer_end(txbuf);
	p_dat = (char *)iio_buffer_first(txbuf, tx0_i);
	
	// Track how much frame data we've sent
	size_t frame_offset = 0;
	
	// Fill TX buffer with frame data bytes
	// The MSK modulator expects raw data bytes, not I/Q samples
	// It will internally convert to MSK I/Q modulation
	for (/* p_dat above */; p_dat < p_end; p_dat += p_inc) {
		if (frame_offset < frame_size) {
			// Send frame byte as 16-bit data to MSK modulator
			// MSK modulator expects data width configured in Tx_Data_Width register (32 bits)
			// Use both I and Q channels to send 32 bits total per sample
			uint8_t data_byte = frame_data[frame_offset];

			// Pack byte into 16-bit I/Q format for MSK modulator input.
			// MSK will process this as bit data, not as I/Q samples.
			// If TX_DATA_WIDTH is 8, use the low byte of the I channel.
			// If TX_DATA_WIDTH is 16, use the full I channel.
			// If TX_DATA_WIDTH is 24, use the full I channel and the low byte of the Q channel.
			// If TX_DATA_WIDTH is 32, use the full I and Q channels.
			// Since we may be sending an odd number of bytes (271 with the sync word) we must use TX_DATA_WIDTH of 8.
			((int16_t*)p_dat)[0] = (int16_t)(data_byte);	// Real (I) - lower byte
			((int16_t*)p_dat)[1] = (int16_t)(0);			// Imag (Q)

			frame_offset++;
		} else {
			// Pad with zeros if frame is shorter than buffer
			((int16_t*)p_dat)[0] = 0;
			((int16_t*)p_dat)[1] = 0;
		}
	}

	return 0;
}

// Send OVP frame data to MSK modulator via IIO buffer in two steps.
// Step 1: move the data into txbuf using iio calls
// Step 2: iio_push txbuf to the kernel
// This is step 2.
int push_txbuf_to_msk(void) {

	static uint32_t old_xfer_count = 0;
	uint32_t local_ts_base;

	local_ts_base = get_timestamp_ms();
	// printf("OVP: time between iio_buffer_push starts was %dms\n", local_ts_base - push_ts_base);
	ssize_t result = iio_buffer_push(txbuf);
	printf("OVP: iio_buffer_push took %dms.\n", get_timestamp_ms()-local_ts_base);

	if (result < 0) {
		printf("OVP: Error pushing buffer to MSK: %zd\n", result);
		return -1;
	}
	
	// Check that data is flowing to MSK block
	uint32_t new_xfer_count = capture_and_read_msk(OFFSET_MSK(axis_xfer_count));
	uint32_t delta = new_xfer_count - old_xfer_count;
	
	printf("OVP: Buffer pushed, axis_xfer_count delta: %u\n", delta);
	printf("OVP: Current axis_xfer_count: %u\n", new_xfer_count);
	old_xfer_count = new_xfer_count;
	return 0;
}


// Enable PTT and start MSK transmission
int enable_msk_transmission(void) {
	WRITE_MSK(MSK_Control, 0x00000001);	// PTT on, loopback off
	
	// Small delay to let hardware settle
	usleep(1000);
	
	uint32_t status = READ_MSK(MSK_Status);
	printf("OVP: MSK_Status after PTT enable: 0x%08x\n", status);
	
	return 0;
}


// Disable PTT and stop MSK transmission
int disable_msk_transmission(void) {
	printf("timeline @ %lld: Disabling MSK transmission (PTT OFF)\n", get_timestamp_us() - session_T0);
	WRITE_MSK(MSK_Control, 0x00000000);	// PTT off
	
	uint32_t status = READ_MSK(MSK_Status);
	printf("OVP: MSK_Status after PTT disable: 0x%08x\n", status);
	
	return 0;
}


// A transmission session is a sequence of consecutive frames to transmit
// with no (or minimal) gaps. This program has to infer the start and end
// of a transmission session from the arrival of encapsulated frames over
// the network. A first arriving frame implies turning on the transmitter
// and sending a preamble. A last arriving frame (including a tail of
// dummy frames called a "hang time") implies sending a postamble, waiting
// for it to be transmitted, and then turning off the transmitter.

// Processing triggered by the arrival of a first frame after an idle period.
void start_transmission_session(void) {
	if (ovp_transmission_active) {	// should never happen
		return;	// Session already active
	}
	
	printf("OVP: Starting transmission session for station %s\n", active_station_id_ascii);
	session_ts_base = get_timestamp_ms();	// for debug prints

	session_T0 = get_timestamp_us();	// used for timeline management
	decision_time = session_T0 + 60e3;	// 60ms to the middle of the next frame, T0-referenced

	// Send preamble: pure 1100 bit pattern for 40ms (no OVP header)
	uint8_t preamble_frame[OVP_MODULATOR_FRAME_SIZE];	// Full 40ms of data
	
	// Fill with 1100 repeating pattern (0xCC = 11001100 binary)
	for (unsigned int i = 0; i < sizeof(preamble_frame); i++) {
		preamble_frame[i] = 0xCC;	// 1100 1100 repeating
	}
	
	// Send preamble to MSK modulator (pure bit pattern, no framing)
	printf("OVP: Sending 40ms preamble (1100 pattern, %zu bytes)\n", sizeof(preamble_frame));
	load_ovp_frame_into_txbuf(preamble_frame, sizeof(preamble_frame));
	printf("preamble loaded!\n");
	enable_msk_transmission();
	printf("transmission enabled!\n");
	push_txbuf_to_msk();
	printf("preamble pushed!\n");

	ovp_transmission_active = 1;
	ovp_sessions_started++;

	// Inform the timeline manager to start managing the session
	pthread_cond_signal(&timeline_start);

	printf("T0 = %lld\n", session_T0);
	printf("Td = %lld\n", decision_time);
	return;
}


// Processing triggered by the completion of a transmission session
// due to no new encapsulated frames arriving for a full hang time.
// That is, the normal way a transmission ends.
void end_transmission_session_normally(void) {

	printf("OVP: Ending transmission session for station %s\n", active_station_id_ascii);

	// There's no definite way to synchronize with the end of the postamble
	// going out the antenna. We'll have to settle for a fixed wait time,
	// which should work provided the timeline is working as designed.
	usleep(65000);
	disable_msk_transmission();
	
	ovp_transmission_active = 0;
	ovp_sessions_ended++;
	hang_timer_active = 0;
	dummy_frames_sent = 0;

	printf("OVP: Session ended after %dms, ready for next transmission\n", get_timestamp_ms()-session_ts_base);
}


// Processing triggered by an abrupt end to a transmission session,
// such as termination of the whole program.
void abort_transmission_session(void) {

	if (ovp_transmission_active) {
		printf("OVP: Aborting transmission session\n");
	}

	disable_msk_transmission();
	
	if (ovp_transmission_active) {
		ovp_sessions_ended++;
	}

	hang_timer_active = 0;
	dummy_frames_sent = 0;

	if (ovp_transmission_active) {
		printf("OVP: Session aborted after %dms\n", get_timestamp_ms()-session_ts_base);
		ovp_transmission_active = 0;
	}
}

//!!! DOES THIS WORK with randomizing and interleaving? Not sure.
// Modify the frame leftover in modulator_frame_buffer to transform it
// into a dummy frame, by replacing its payload with the encoded dummy payload.
void create_dummy_frame(void) {
	// The logical payload of a dummy frame is all zeroes.
	// The COBS decoder will take this as termination of any previously open
	// packet, followed by a bunch of zero-length packets (which are dropped).
	static uint8_t ovp_dummy_payload[OVP_PAYLOAD_SIZE] = {  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
															0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
															0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
															0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
															0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
															0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
															0,0 };

	// To save the trouble of encoding the dummy payload over and over,
	// we will compute it just the first time and then store it here.
	static uint8_t modulator_dummy_payload[OVP_ENCODED_PAYLOAD_SIZE];
	static bool payload_computed = false;

	// We are taking advantage of the fact that the payload of a dummy
	// frame is always the same. Payload is all zeroes, and the convolutional
	// encoding always gets the same encoded payload (it's used as a block code).
	if (! payload_computed) {
		encode_ovp_payload(ovp_dummy_payload, modulator_dummy_payload);
		payload_computed = true;
	}

	// We are taking advantage of the modulator_frame_buffer being global,
	// and dummy frames always coming after normal frames that were built
	// in the modulator_frame_buffer. We only need to change the payload.
	//
	// Note that this doesn't update the authentication tag in the frame.
	// We are not able to do that in any case, since authentication is
	// handled by Interlocutor, not this program. The air interface spec
	// permits this behavior.
	uint8_t *p = modulator_frame_buffer + OVP_MODULATOR_PAYLOAD_OFFSET;
	memcpy(p, modulator_dummy_payload, OVP_ENCODED_PAYLOAD_SIZE);
}

void create_postamble_frame(void) {

	static uint8_t modulator_postamble_payload[OVP_ENCODED_PAYLOAD_SIZE];
	static bool payload_computed = false;

	// We are taking advantage of the fact that the payload of a postamble
	// frame is always the same. Payload is fixed (copies of the barker-11
	// code), and the convolutional encoding always gets the same encoded
	// payload (it's used as a block code).
	if (! payload_computed) {
		// Fill payload with 11-bit Barker sequence: 11100010010
		uint16_t barker_11 = 0x0712;	// 11100010010 in binary (right-aligned)

		// Repeat Barker sequence throughout payload
		int bit_pos = 0;
		for (int i=0; i < OVP_ENCODED_PAYLOAD_SIZE; i++) {	// Fill payload section
			uint8_t byte_val = 0;

			for (int bit = 7; bit >= 0; bit--) {	// MSB first
				// Get current bit from Barker sequence
				int barker_bit = (barker_11 >> (bit_pos % 11)) & 1;
				if (barker_bit) {
					byte_val |= (1 << bit);
				}
				bit_pos++;
			}
			modulator_postamble_payload[i] = byte_val;
		}
		payload_computed = true;
	}

	// We are taking advantage of the modulator_frame_buffer being global,
	// and the postamble frame always coming after other frames that were built
	// in the modulator_frame_buffer. We only need to change the payload.
	//
	// Note that this doesn't update the authentication tag in the frame.
	// We are not able to do that in any case, since authentication is
	// handled by Interlocutor, not this program.
	uint8_t *p = modulator_frame_buffer + OVP_MODULATOR_PAYLOAD_OFFSET;
	memcpy(p, modulator_postamble_payload, OVP_ENCODED_PAYLOAD_SIZE);
}


// Initialize network port for UDP encapsulated frames (both directions)
int init_udp_socket(void) {
	static bool initialized = false;

	if (initialized) {
		return 0;
	}

	// create the socket
	ovp_udp_socket = socket(AF_INET, SOCK_DGRAM, 0);
	if (ovp_udp_socket < 0) {
		perror("OVP: Failed to create UDP socket");
		return -1;
	}
	
	// Set socket options
	int opt = 1;
	if (setsockopt(ovp_udp_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
		perror("OVP: Failed to set socket options");
		close(ovp_udp_socket);
		return -1;
	}

	// Configure listen address
	memset(&ovp_listen_addr, 0, sizeof(ovp_listen_addr));
	ovp_listen_addr.sin_family = AF_INET;
	ovp_listen_addr.sin_addr.s_addr = INADDR_ANY;
	ovp_listen_addr.sin_port = htons(OVP_UDP_PORT);
	
	// Bind socket
	if (bind(ovp_udp_socket, (struct sockaddr*)&ovp_listen_addr, sizeof(ovp_listen_addr)) < 0) {
		perror("OVP: Failed to bind UDP socket");
		close(ovp_udp_socket);
		ovp_udp_socket = -1;
		return -1;
	}

	initialized = true;
	return 0;
}


// Initialize OVP UDP listener
int init_ovp_udp_listener(void) {

	if (init_udp_socket()) {
		printf("Error initializing UDP socket\n");
		return -1;
	} else {
		printf("OVP: UDP listener initialized on port %d\n", OVP_UDP_PORT);
	}

	return 0;
}

// Validate OVP frame format
int validate_ovp_frame(uint8_t *frame_data, size_t frame_size) {
	if (frame_size < OVP_HEADER_SIZE) {
		printf("OVP: Frame too short (%zu bytes)\n", frame_size);
		return -1;
	}
	
	// Check magic bytes (0xBBAADD)
	uint32_t magic = (frame_data[6] << 16) | (frame_data[7] << 8) | frame_data[8];
	if (magic != OVP_MAGIC_BYTES) {
		printf("OVP: Invalid magic bytes 0x%06x (expected 0x%06x)\n", magic, OVP_MAGIC_BYTES);
		return -1;
	}
	
	// Additional validation could go here:
	// - Station ID validation (base-40 encoding)
	// - Authentication token check
	
	return 0;
}

// apply or remove randomization, in place, to reduce spectral features of the data
void randomize_ovp_frame(uint8_t *buf) {
	for (int i=0; i < OVP_SINGLE_FRAME_SIZE; i++) {
		buf[i] ^= randomization_sequence[i];
	}
}

// FEC-encode OVP header
void encode_ovp_header(uint8_t *input, uint8_t *output) {
	//!!! placeholder FEC
	memcpy(output, input, OVP_HEADER_SIZE);
	memcpy(output + OVP_HEADER_SIZE, input, OVP_HEADER_SIZE);	// Simple repetition placeholder
}

// FEC-decode OVP header
void decode_ovp_header(uint8_t *input, uint8_t *output) {
	//!!! placeholder FEC
	memcpy(output, input, OVP_HEADER_SIZE);	// matches placeholder encoder
}

// FEC-encode OVP payload
void encode_ovp_payload(uint8_t *input, uint8_t *output) {
	//!!! placeholder FEC
	memcpy(output, input, OVP_PAYLOAD_SIZE);
	memcpy(output + OVP_PAYLOAD_SIZE, input, OVP_PAYLOAD_SIZE);	// Simple repetition placeholder
}

// FEC-decode OVP payload
void decode_ovp_payload(uint8_t *input, uint8_t *output) {
	//!!! placeholder FEC
	memcpy(output, input, OVP_PAYLOAD_SIZE);	// matches placeholder encoder
}

// apply interleaving to break up block errors for the decoder
void interleave_ovp_frame(uint8_t *buf) {
	//!!! dummy implementation, write this.
	buf[0] = buf[0];
}

// remove interleaving
void deinterleave_ovp_frame(uint8_t *buf) {
	//!!! dummy implementation, write this.
	buf[0] = buf[0];
}


// Process OVP frame payload through software pipeline
int process_ovp_payload_software(uint8_t *ovp_frame, size_t frame_size) {
	int count = 0;
	uint8_t randomized_frame[OVP_SINGLE_FRAME_SIZE];

	if (frame_size != OVP_SINGLE_FRAME_SIZE) {
		printf("OVP: Wrong frame size %d\n", frame_size);
		return -1;
	}

	// apply randomization to reduce spectral features of the data
	memcpy(randomized_frame, ovp_frame, OVP_SINGLE_FRAME_SIZE);
	randomize_ovp_frame(randomized_frame);

	uint8_t encoded_header[OVP_ENCODED_HEADER_SIZE];
	encode_ovp_header(randomized_frame, encoded_header);

	uint8_t encoded_payload[OVP_ENCODED_PAYLOAD_SIZE];
	encode_ovp_payload(randomized_frame + OVP_HEADER_SIZE, encoded_payload);

	uint8_t *p = modulator_frame_buffer;

	// Components of a transmitted OVP frame:
	memcpy(p, encoded_header, OVP_ENCODED_HEADER_SIZE);
	p += OVP_ENCODED_HEADER_SIZE;
	count += OVP_ENCODED_HEADER_SIZE;

	memcpy(p, encoded_payload, OVP_ENCODED_PAYLOAD_SIZE);
	count += OVP_ENCODED_PAYLOAD_SIZE;

	if (count != OVP_MODULATOR_FRAME_SIZE) {
		printf("OVP: Modulator frame size %d does not match expected %d\n", count, OVP_MODULATOR_FRAME_SIZE);
		return -1;
	}

	// apply interleaving to break up block errors at the decoder(s)
	interleave_ovp_frame(modulator_frame_buffer);

	// The modulator_frame_buffer now contains the full modulator frame
	// ready for transmission via MSK modulator
	return 0;
}


// Complete processing and send an OVP frame to MSK modulator
int process_and_send_ovp_frame_to_txbuf(uint8_t *frame_data, size_t frame_size) {
	
	// Process the frame (no preamble/postamble per frame)
	int result;
	result = process_ovp_payload_software(frame_data, frame_size);

	if (result == 0) {
		// Preload txbuf with the OVP frame. We'll push it at the decision time.
		// If we've already preloaded txbuf for this frame slot, this overwrites.
		load_ovp_frame_into_txbuf(modulator_frame_buffer, sizeof(modulator_frame_buffer));
		ovp_txbufs_this_frame++;	// for detection of overwrites.
		// Do not push_txbuf_to_msk() at this time. In case of overwrites,
		// we want to transmit the last one only.
	}
	
	if (result == 0) {
		ovp_frames_processed++;
	} else {
		ovp_frame_errors++;
	}
	
	return result;
}


// Process complete OVP frame (keeping a copy of the station ID)
int process_ovp_frame(uint8_t *frame_data, size_t frame_size) {
	int result;

	ovp_frames_received++;
	
	// Validate frame
	if (validate_ovp_frame(frame_data, frame_size) < 0) {
		ovp_frame_errors++;
		return -1;
	}
	
	// Extract and store station ID for regulatory compliance
	memcpy(active_station_id_binary, frame_data, 6);	// Station ID length of 6 bytes
	decode_station_id(active_station_id_binary, active_station_id_ascii);

	// Real frame received - cancel hang timer
	if (hang_timer_active) {
		printf("OVP: Real frame received from %s, canceling hang timer\n", active_station_id_ascii);
		hang_timer_active = 0;
		dummy_frames_sent = 0;
	}

	// Start transmission session if not active
	if (!ovp_transmission_active) {
		start_transmission_session();
	}

	printf("OVP: Processing real frame %zu bytes from %s\n", frame_size, active_station_id_ascii);

	result = process_and_send_ovp_frame_to_txbuf(frame_data, frame_size);
	return result;
}


// UDP listener thread
void* ovp_udp_listener_thread(__attribute__((unused)) void *arg) {
	uint32_t recv_ts;
	uint32_t last_recv_ts = 0;


	while (ovp_running) {

		udp_bytes_received = recvfrom(
			ovp_udp_socket,
			ovp_frame_buffer,
			sizeof(ovp_frame_buffer),
			0, // blocking receive - will wait for frames
			(struct sockaddr*)&udp_client_addr,
			&udp_client_len
		);

		// notify the receive process that we know the client address
		encapsulated_frame_host_known = true;

		// might be shutting down now, don't grab the mutex
		if (!ovp_running) {
			break;
		}
		pthread_mutex_lock(&timeline_lock);
		
		if (udp_bytes_received == OVP_SINGLE_FRAME_SIZE) {
			//printf("OVP: Received %zd bytes from %s:%d\n", 
			//			bytes_received,
			//			inet_ntoa(client_addr.sin_addr),
			//			ntohs(client_addr.sin_port));
			
			recv_ts = get_timestamp_ms();
			printf("OVP: Received %zd bytes from %s:%d after %dms ending with ", udp_bytes_received,
					inet_ntoa(udp_client_addr.sin_addr),
					ntohs(udp_client_addr.sin_port),
					recv_ts - last_recv_ts);
			last_recv_ts = recv_ts;
			for (int i=OVP_SINGLE_FRAME_SIZE-9; i<OVP_SINGLE_FRAME_SIZE; i++) {
				printf("%02x ", ovp_frame_buffer[i]);
			}
			printf("\n");

			// Process the frame
			process_ovp_frame(ovp_frame_buffer, udp_bytes_received);
		} else if (udp_bytes_received >= 0) {
			printf("OVP: Warning - received unexpected frame size %zd bytes (expected %d)\n", 
					udp_bytes_received, OVP_SINGLE_FRAME_SIZE);
			ovp_frame_errors++;
		} else if (udp_bytes_received < 0) {
			if (errno != EAGAIN && errno != EWOULDBLOCK && ovp_running) {
				perror("OVP: UDP receive error");	// don't exit on receive errors
			}
		}
		pthread_mutex_unlock(&timeline_lock);
	}
	
	printf("OVP: UDP listener thread exiting\n");
	return NULL;
}

// Start OVP UDP listener
int start_ovp_listener(void) {
	if (init_ovp_udp_listener() < 0) {
		return -1;
	}
	
	ovp_running = 1;
	
	if (pthread_create(&ovp_udp_thread, NULL, ovp_udp_listener_thread, NULL) != 0) {
		perror("OVP: Failed to create UDP thread");
		close(ovp_udp_socket);
		ovp_udp_socket = -1;
		ovp_running = 0;
		return -1;
	}
	
	printf("OVP: Listener started successfully\n");
	return 0;
}

// Stop OVP UDP listener
void stop_ovp_listener(void) {
	if (ovp_running) {
		ovp_running = 0;
		
		// Close socket to unblock recvfrom in thread
		if (ovp_udp_socket >= 0) {
			close(ovp_udp_socket);
			ovp_udp_socket = -1;
		}
		
		if (ovp_udp_thread) {
			pthread_cancel(ovp_udp_thread);
		}
		
		printf("OVP: UDP listener stopped\n");
	}
}

void dump_bytes(char *name, uint8_t *buf, size_t length) {
	printf("%s: ", name);

	for (size_t i=0; i < length; i++) {
		printf("%02x ", buf[i]);
	}

	printf("\n");
}

void dump_buffer(char *name, struct iio_buffer *buf) {
	printf("%s: ", name);

	printf("@%p-> ", buf);
	ptrdiff_t p_inc = iio_buffer_step(buf);
	char *p_end = iio_buffer_end(buf);
	char *first = (char *)iio_buffer_first(buf, rx0_i);
	for (char *p_dat = first; p_dat < p_end; p_dat += p_inc) {
		printf("%02x ", ((int16_t*)p_dat)[0] & 0x00ff);
	}

	printf("\n");
}
	

// OVP Timeline Manager thread
// Wakes up at each frame decision time (Td) during an active session
// and sends a frame. In priority order:
//		* A real frame if one is available, or
//		* A postamble frame if the final hang time is completed, or
//		* A dummy frame.
void* ovp_timeline_manager_thread(__attribute__((unused)) void *arg) {
	int64_t now;
	static int hang_time_dummy_count = 0;
	uint32_t local_ts_base;

	while (!stop) {
		while (ovp_transmission_active) {
			// printf("timeline grabbing mutex\n");
			pthread_mutex_lock(&timeline_lock);
			if (decision_time != 0) {
				now = get_timestamp_us();
				// printf("now %lld Td %lld -> decision in %lld us\n", now, decision_time, decision_time - now);
				if (now >= decision_time) {
					if (ovp_txbufs_this_frame > 0) {
						// We've already put a real frame into txbuf at this point
						if (ovp_txbufs_this_frame > 1) {
							ovp_untimely_frames += ovp_txbufs_this_frame - 1;
							printf("timeline @ %lld: frame + %d untimely frames ", get_timestamp_us() - session_T0, ovp_txbufs_this_frame - 1);
						} else {
							printf("timeline @ %lld: frame ", get_timestamp_us() - session_T0);
						}
						ovp_frames_processed++;
						hang_time_dummy_count = 0;
						hang_timer_active = 0;
					} else {
						if (hang_time_dummy_count >= hang_timer_frames) {
							create_postamble_frame();
							printf("timeline @ %lld: postamble ", get_timestamp_us() - session_T0);
							hang_time_dummy_count = 0;
							hang_timer_active = 0;
							ovp_transmission_active = 0;
						} else {
							create_dummy_frame();
							printf("timeline @ %lld: dummy frame ", get_timestamp_us() - session_T0);
							ovp_dummy_frames_sent++;
							hang_time_dummy_count++;
							hang_timer_active = 1;
						}
						process_ovp_payload_software(modulator_frame_buffer, sizeof(modulator_frame_buffer));
						load_ovp_frame_into_txbuf(modulator_frame_buffer, sizeof(modulator_frame_buffer));
					}
					local_ts_base = get_timestamp_ms();
					iio_buffer_push(txbuf);
					printf("iio_buffer_push took %dms.\n", get_timestamp_ms()-local_ts_base);
					decision_time += 40e3;
					ovp_txbufs_this_frame = 0;
				}

				// handle normal end of a transmission session (postamble was just pushed)
				if (! ovp_transmission_active) {
					end_transmission_session_normally();
				}
			}
			pthread_mutex_unlock(&timeline_lock);
			// printf("timeline released mutex\n");

			now = get_timestamp_us();
			if (decision_time - now > 0) {
				usleep(decision_time - now);	// wait until next decision time
			}

		}
		
		printf("OVP: Timeline paused until next transmission\n");
		pthread_cond_wait(&timeline_start, &tls_lock);
	}

	printf("OVP: Timeline manager thread exiting\n");
	return NULL;
}

int start_timeline_manager(void) {
	if (pthread_create(&ovp_timeline_thread, NULL, ovp_timeline_manager_thread, NULL) != 0) {
		perror("OVP: Failed to create timeline manager thread");
		return -1;
	}
	
	printf("OVP: Timeline manager started successfully\n");
	return 0;
}

void stop_timeline_manager(void) {
	if (ovp_timeline_thread) {
		pthread_cancel(ovp_timeline_thread);
	}

	printf("OVP: Timeline manager stopped\n");
}


void print_ovp_statistics(void) {
	printf("\n=== OVP Statistics ===\n");
	printf("Frames received:     %llu\n", (unsigned long long)ovp_frames_received);
	printf("Frames processed:    %llu\n", (unsigned long long)ovp_frames_processed);
	printf("Frame errors:        %llu\n", (unsigned long long)ovp_frame_errors);
	printf("Untimely frames:     %llu\n", (unsigned long long)ovp_untimely_frames);
	printf("Dummy frames sent:   %llu\n", (unsigned long long)ovp_dummy_frames_sent);
	printf("Sessions started:    %llu\n", (unsigned long long)ovp_sessions_started);
	printf("Sessions ended:      %llu\n", (unsigned long long)ovp_sessions_ended);
	printf("Active session:      %s\n", ovp_transmission_active ? "YES" : "NO");
	if (ovp_transmission_active) {
		printf("Active station ID:   %s\n", active_station_id_ascii);
	}
	printf("Hang timer active:   %s\n", hang_timer_active ? "YES" : "NO");
	if (hang_timer_active) {
		printf("Dummy frames sent:   %d/%d (for %s)\n", 
				dummy_frames_sent, hang_timer_frames, active_station_id_ascii);
	}
	if (ovp_frames_received > 0) {
		printf("Success rate:        %.1f%%\n", 
			(double)ovp_frames_processed / ovp_frames_received * 100.0);
	}
	printf("======================\n");
}

// OVP Periodic Statistics Reporter thread
// Wakes up at intervals of about 10 seconds and print a report.
void* ovp_periodic_statistics_reporter_thread(__attribute__((unused)) void *arg) {
	while (!stop) {
		pthread_mutex_lock(&timeline_lock);
		print_ovp_statistics();
		pthread_mutex_unlock(&timeline_lock);
		// don't sleep all at once; makes exiting the program too slow.
		for (int i=0; i < 100; i++) {
			usleep(100e3);	// 100 iterations of 100ms = 10 seconds
			if (stop) {
				break;
			}
		}
	}
	printf("OVP: Periodic reporter thread exiting\n");
	return NULL;
}

int start_periodic_statistics_reporter(void) {
	if (pthread_create(&ovp_reporter_thread, NULL, ovp_periodic_statistics_reporter_thread, NULL) != 0) {
		perror("OVP: Failed to create periodic statistics reporter thread");
		return -1;
	}
	
	printf("OVP: Reporter started successfully\n");
	return 0;
}

void stop_periodic_statistics_reporter(void) {
	if (ovp_reporter_thread) {
		pthread_cancel(ovp_reporter_thread);
	}
		
	printf("OVP: Reporter stopped\n");
}


// Receiver thread
// The receiver thread pulls frames of data from IIO and eventually
// encapsulates them for transmission over the network.
//
// We are assuming that the modem (FPGA) detects the frame sync word
// at the beginning of each frame and uses that to establish frame
// boundaries and byte alignment. The modem consumes the frame sync
// word and does not send it to us.
//
// Having received the frame over the air, the FPGA might or might not
// do more of the processing to extract the decoded data from the frame.
// For now, we are assuming that it does no further processing, and
// simply ships the unprocessed frame to us via IIO.
//
// The receiver thread blocks in a iio_buffer_refill() call with an
// infinite timeout. Initially, before any frame sync word has been
// detected, nothing happens. After a first frame sync word is detected,
// the FPGA gathers up the whole frame and then burst-transfers it to
// IIO. At that point, the iio_buffer_refill() call will return with
// one frame of data (this depends on the size of rxbuf matching the
// size of an unprocessed frame, not including the sync word). While
// further frames are being received, along with their properly-aligned
// frame sync words, each call to iio_buffer_refill() should return
// promptly (at 40ms boundaries) with a new frame full of data.
//
// Eventually the modem will fail to receive frame sync words,
// either because the transmission has ended or because it has faded
// out. The modem may deliver some number of frames full of garbage,
// and is expected to eventually decide that no signal is present and
// stop sending frames to us. When a frame sync is again detected by
// the modem, it may or may not be time-aligned with the previous
// series of frames. Dealing with this is entirely up to the modem
// and to downstream application processors. We just accept frames
// from the modem, encapsulate them for network transmission, and
// forward them along without delay.
//
// libiio is not thread-safe. Recommended solution is to clone the context
// and re-create everything in the cloned context. They have already been
// appropriately configured in main.
//

struct iio_buffer *rx_buf;

void* ovp_receiver_thread(__attribute__((unused)) void *arg) {
	uint32_t refill_ts_base;
	ssize_t nbytes_rx;
	uint8_t received_frame[OVP_DEMOD_FRAME_SIZE];
	uint8_t decoded_frame[OVP_SINGLE_FRAME_SIZE];
	
	struct iio_context *rx_ctx;
	struct iio_device *rx_dev;
	struct iio_channel *rx_ch_i, *rx_ch_q;

	rx_ctx = iio_context_clone(ctx);
	if (!rx_ctx) {
		printf("Failed to create receive context\n");
		cleanup_and_exit();
	}

	// set the timeout to infinity; we may need to wait any length of time
	// before the first frame of a transmission is received, so we need the
	// receive buffer_refill to never time out.
	int ret = iio_context_set_timeout(rx_ctx, 0);
	if (ret < 0) {
			char timeout_test[256];
			iio_strerror(-(int)ret, timeout_test, sizeof(timeout_test));
			printf("* rx_ctx set_timeout failed : %s\n", timeout_test);
	} else {
			printf("* rx_ctx set_timeout returned %d, which is a success.\n", ret);
	}


	// Find the SAME devices/channels (already configured by main)
	rx_dev = iio_context_find_device(rx_ctx, "cf-ad9361-lpc");
	rx_ch_i = iio_device_find_channel(rx_dev, "voltage0", RX);
	rx_ch_q = iio_device_find_channel(rx_dev, "voltage1", RX);

	// Enable channels in THIS context
	iio_channel_enable(rx_ch_i);
	iio_channel_enable(rx_ch_q);

	rx_buf = iio_device_create_buffer(rx_dev, OVP_DEMOD_FRAME_SIZE, false);
	if (!rx_buf) {
		perror("Could not create RX buffer");
		cleanup_and_exit();
	}


	while(!stop) {
		// Refill RX buffer
		refill_ts_base = get_timestamp_ms();
		nbytes_rx = iio_buffer_refill(rx_buf);
		if (nbytes_rx < 0) {
				if (nbytes_rx == -ETIMEDOUT) {
					printf("Refill timeout (no sync word yet?)\n");
					nbytes_rx = 0;
				} else {
					printf("Error refilling buf %d\n",(int) nbytes_rx); cleanup_and_exit();
				}
		} else {
			printf("OVP: streaming buffer_refill of %d bytes took %dms\n", nbytes_rx, get_timestamp_ms() - refill_ts_base);

			// nbytes_rx includes the three wasted bytes for each byte transferred via AXI-S
			if (nbytes_rx != OVP_DEMOD_FRAME_SIZE * 4) {
				printf("Warning: unexpected rx frame size %d; discarded!\n", nbytes_rx);
				continue;
			}

			// make a copy of the received data, removing IIO padding
			ptrdiff_t p_inc = iio_buffer_step(rx_buf);
			char *p_end = iio_buffer_end(rx_buf);
			uint8_t *p_out = received_frame;
			char *first = (char *)iio_buffer_first(rx_buf, rx0_i);
			for (char *p_dat = first; p_dat < p_end; p_dat += p_inc) {
				*p_out++ = ((int16_t*)p_dat)[0] & 0x00ff;
			}

			// dump received buffer for visual check
			printf("Received raw data @ %p: ", first);
			for (int i=0; i < OVP_DEMOD_FRAME_SIZE; i++) {
				printf("%02x ", received_frame[i]);
			}
			printf("\n");

			// remove the interleaving so the decoder can do its work
			deinterleave_ovp_frame(received_frame);

			// decode both parts of the frame
			decode_ovp_header(received_frame, decoded_frame);
			decode_ovp_payload(received_frame + OVP_ENCODED_HEADER_SIZE, decoded_frame + OVP_HEADER_SIZE);

			// remove the randomization
			randomize_ovp_frame(decoded_frame);

			//!!! dump received data for visual check
			printf("Received processed data: ");
			for (int i=0; i < OVP_SINGLE_FRAME_SIZE; i++) {
				printf("%02x ", decoded_frame[i]);
			}
			printf("\n");

			if (!encapsulated_frame_host_known) {
				printf("OTA frame received before first UDP datagram; discarded.");
				continue;
			}

			// !!! add encapsulation and network transmission here




		}
	}

	iio_buffer_destroy(rx_buf);
	iio_context_destroy(rx_ctx);

	printf("OVP: receiver thread exiting\n");
	return NULL;
}

// Initialize OVP receiver thread
int init_ovp_receiver(void) {

	if (init_udp_socket()) {
		return -1;
	}

	encapsulated_frame_host_known = false;
	return 0;
}


// Start OVP receiver thread
int start_ovp_receiver(void) {
	if (init_ovp_receiver() < 0) {
		return -1;
	}
		
	if (pthread_create(&ovp_rx_thread, NULL, ovp_receiver_thread, NULL) != 0) {
		perror("OVP: Failed to create receiver thread");
		return -1;
	}
	
	printf("OVP: Receiver started successfully\n");
	return 0;
}

// Stop OVP receiver thread
void stop_ovp_receiver(void) {
	if (ovp_rx_thread) {
		if (rx_buf) {
			iio_buffer_cancel(rx_buf);
		}
		pthread_cancel(ovp_rx_thread);
	}
		
	printf("OVP: receiver stopped\n");
}


// ---------- Debug Thread -----------------------------

void* ovp_debug_thread_func(__attribute__((unused)) void *arg) {
	uint32_t sync_status;
	bool frame_sync_locked;
	bool frame_buffer_overflow;
	uint32_t frames_received = 0;
	uint32_t frame_sync_errors = 0;

	while (!stop) {
		uint32_t now;

		pthread_mutex_lock(&timeline_lock);
			now = get_timestamp_ms();
			printf("debugthread fifo at %d ms: tx fifo: %08x rx fifo: %08x\n", now,
							capture_and_read_msk(OFFSET_MSK(tx_async_fifo_rd_wr_ptr)),
							capture_and_read_msk(OFFSET_MSK(rx_async_fifo_rd_wr_ptr)));
			printf("debugthread power at %d %d ", now,
							capture_and_read_msk(OFFSET_MSK(rx_power)));
			print_rssi();
			sync_status = capture_and_read_msk(OFFSET_MSK(rx_frame_sync_status));
			frame_sync_locked = sync_status & 0x00000001;
			frame_buffer_overflow = sync_status & 0x00000002;
			frames_received = (sync_status & 0x03fffffc) >> 2;
			frame_sync_errors = ((sync_status & 0xfc000000) >> 26) & 0x3f;
			printf("debugthread frame at %d ms: raw 0x%08x rcvd %d, errs %d %s %s\n", now,
				sync_status,
				frames_received,
				frame_sync_errors,
				frame_sync_locked ? "LOCKED" : "unlocked",
				frame_buffer_overflow ? "OVERFLOW" : "");
		pthread_mutex_unlock(&timeline_lock);

		usleep(10000);
	}
	printf("debug thread exiting\n");
	return NULL;
}

int start_debug_thread(void) {

	if (pthread_create(&ovp_debug_thread, NULL, ovp_debug_thread_func, NULL) != 0) {
		perror("OVP: Failed to create debug thread");
		return -1;
	}
	
	printf("OVP: debug thread started successfully\n");
	return 0;
}

void stop_debug_thread(void) {
	if (ovp_debug_thread) {
		pthread_cancel(ovp_debug_thread);
	}
		
	printf("OVP: debug thread stopped\n");
}


// -=-=-=-=-=-=-=-=-=-= MAIN FUNCTION =-=-=-=-=-=-=-=-=-=-=-
/* Configuration based on ADI's example for simple configuration and streaming */

/* usage:
 * Default context, assuming local IIO devices, i.e., this script is run on ADALM-Pluto for example
 $./a.out
 * URI context, find out the uri by typing `iio_info -s` at the command line of the host PC
 $./a.out usb:x.x.x
 */
int main (int argc, char **argv)
{
	// Streaming devices
	struct iio_device *tx;
	struct iio_device *rx;

	// Stream configurations
	struct stream_cfg rxcfg;
	struct stream_cfg txcfg;

	// Listen to ctrl+c and IIO_ENSURE
	signal(SIGINT, handle_sig);

	// OPV hardware RX stream config
	rxcfg.bw_hz = RF_BANDWIDTH;
	rxcfg.fs_hz = MHZ(61.44);	// 2.5 MS/s rx sample rate
	rxcfg.lo_hz = LO_FREQ;
	rxcfg.rf_port = "A_BALANCED";	// port A (select for rf freq.)

	// OPV hardware TX stream config
	txcfg.bw_hz = RF_BANDWIDTH;
	txcfg.fs_hz = MHZ(61.44);	// 2.5 MS/s tx sample rate
	txcfg.lo_hz = LO_FREQ;
	txcfg.rf_port = "A";	// port A (select for rf freq.)

	printf("* Acquiring IIO context\n");
	if (argc == 1) {
		IIO_ENSURE((ctx = iio_create_default_context()) && "No context");
	}
	else if (argc == 2) {
		IIO_ENSURE((ctx = iio_create_context_from_uri(argv[1])) && "No context");
	}
	IIO_ENSURE(iio_context_get_devices_count(ctx) > 0 && "No devices");

	printf("* Acquiring AD9361 streaming devices\n");
	IIO_ENSURE(get_ad9361_stream_dev(TX, &tx) && "No tx dev found");
	IIO_ENSURE(get_ad9361_stream_dev(RX, &rx) && "No rx dev found");

	printf("* Configuring AD9361 for streaming\n");
	IIO_ENSURE(cfg_ad9361_streaming_ch(&rxcfg, RX, 0) && "RX port 0 not found");
	IIO_ENSURE(cfg_ad9361_streaming_ch(&txcfg, TX, 0) && "TX port 0 not found");

	printf("* Initializing AD9361 IIO streaming channels\n");
	IIO_ENSURE(get_ad9361_stream_ch(RX, rx, 0, &rx0_i) && "RX chan i not found");
	IIO_ENSURE(get_ad9361_stream_ch(RX, rx, 1, &rx0_q) && "RX chan q not found");
	IIO_ENSURE(get_ad9361_stream_ch(TX, tx, 0, &tx0_i) && "TX chan i not found");
	IIO_ENSURE(get_ad9361_stream_ch(TX, tx, 1, &tx0_q) && "TX chan q not found");

	printf("* Enabling IIO streaming channels\n");
	iio_channel_enable(rx0_i);
	iio_channel_enable(rx0_q);
	iio_channel_enable(tx0_i);
	iio_channel_enable(tx0_q);

	// number of kernel buffers can be increased from the default of 4 below
	// this has to be done before iio_device_create_buffer()
	int ret = iio_device_set_kernel_buffers_count(tx, 4);
	if (ret < 0) {
		char buf_test[256];
		iio_strerror(-(int)ret, buf_test, sizeof(buf_test));
		printf("* set_kernel_buffers (tx) failed : %s\n", buf_test);
	} else {
		printf("* set_kernel_buffers (tx) returned %d, which is a success.\n", ret);
	}

	ret = iio_device_set_kernel_buffers_count(rx, 4);
	if (ret < 0) {
		char buf_test[256];
		iio_strerror(-(int)ret, buf_test, sizeof(buf_test));
		printf("* set_kernel_buffers (rx) failed : %s\n", buf_test);
	} else {
		printf("* set_kernel_buffers (rx) returned %d, which is a success.\n", ret);
	}

	// set the timeout to infinity; we may need to wait any length of time
	// before the first frame of a transmission is received, so we need the
	// receive buffer_refill to never time out.
	ret = iio_context_set_timeout(ctx, 0);
		if (ret < 0) {
				char timeout_test[256];
				iio_strerror(-(int)ret, timeout_test, sizeof(timeout_test));
				printf("* set_timeout failed : %s\n", timeout_test);
		}
		else {
				printf("* set_timeout returned %d, which is a success.\n", ret);
		}

	printf("* Creating TX IIO buffer, size %d\n", OVP_MODULATOR_FRAME_SIZE);
	txbuf = iio_device_create_buffer(tx, OVP_MODULATOR_FRAME_SIZE, false);
	if (!txbuf) {
		perror("Could not create TX buffer");
		cleanup_and_exit();
	}

	printf("Setting for memory-mapped registers in the PL.\n");
	int ddr_memory = open("/dev/mem", O_RDWR | O_SYNC);
	// Memory map the address of the TX-DMAC via its AXI lite control interface register block
	tx_dmac_register_map = mmap(NULL, 65535, PROT_READ | PROT_WRITE, MAP_SHARED, ddr_memory, 0x7c420000);
	// Memory map the address of the RX-DMAC via its AXI lite control interface register block
	rx_dmac_register_map = mmap(NULL, 65535, PROT_READ | PROT_WRITE, MAP_SHARED, ddr_memory, 0x7c400000);
	// Memory map the address of the MSK block via its AXI lite control interface
	msk_register_map = mmap(NULL, 65535, PROT_READ | PROT_WRITE, MAP_SHARED, ddr_memory, 0x43c00000);
	// Memory map the address of the peripherals block so we can use the hardware timer
	timer_register_map = mmap(NULL, 65535, PROT_READ | PROT_WRITE, MAP_SHARED, ddr_memory, PERIPH_BASE);
	if (timer_register_map == MAP_FAILED) {
		printf("Failed top map timer registers\n");
		close(ddr_memory);
		exit(1);
	}

	unsigned int interface = read_mapped_reg(tx_dmac_register_map, DMAC_INTERFACE_DESCRIPTION_1);
	printf("TX DMAC Interface Description (0x%08x@0x%04x)\n", interface, DMAC_INTERFACE_DESCRIPTION_1);
	interface = read_mapped_reg(rx_dmac_register_map, DMAC_INTERFACE_DESCRIPTION_1);
	printf("RX DMAC Interface Description (0x%08x@0x%04x)\n", interface, DMAC_INTERFACE_DESCRIPTION_1);

	printf("Writing to scratch register in TX-DMAC.\n");
	write_mapped_reg(tx_dmac_register_map, DMAC_SCRATCH, 0x5555AAAA);
	printf("Reading from scratch register in TX-DMAC. We see: (0x%08x@%04x)\n", read_mapped_reg(tx_dmac_register_map, DMAC_SCRATCH), DMAC_SCRATCH);
	printf("Reading the TX-DMAC peripheral ID: (0x%08x@%04x)\n", read_mapped_reg(tx_dmac_register_map, DMAC_PERIPHERAL_ID), DMAC_PERIPHERAL_ID);
	printf("Writing to scratch register in RX-DMAC.\n");
	write_mapped_reg(rx_dmac_register_map, DMAC_SCRATCH, 0x5555AAAA);
	printf("Reading from scratch register in RX-DMAC. We see: (0x%08x@%04x)\n", read_mapped_reg(rx_dmac_register_map, DMAC_SCRATCH), DMAC_SCRATCH);
	printf("Reading the RX-DMAC peripheral ID: (0x%08x@%04x)\n", read_mapped_reg(tx_dmac_register_map, DMAC_PERIPHERAL_ID), DMAC_PERIPHERAL_ID);


	printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
	printf("Configure MSK for minimum viable product test.\n");
	printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
	printf("Reading from MSK block HASH ID LOW: (0x%08x@%04x)\n", READ_MSK(Hash_ID_Low), OFFSET_MSK(Hash_ID_Low));
	printf("Reading from MSK block HASH ID HIGH: (0x%08x@%04x)\n", READ_MSK(Hash_ID_High), OFFSET_MSK(Hash_ID_High));
	printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");


	printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
	printf("Configure the TX_SYNC_CTRL register to 0x%08x.\n", TX_SYNC_CTRL_WORD);
	WRITE_MSK(Tx_Sync_Ctrl, TX_SYNC_CTRL_WORD);
	printf("Reading TX_SYNC_CTRL. We see: (0x%08x@%04x)\n", READ_MSK(Tx_Sync_Ctrl), OFFSET_MSK(Tx_Sync_Ctrl));
	printf("Configure the TX_SYNC_CNT register to TX_SYNC_COUNT bit times.\n");
	WRITE_MSK(Tx_Sync_Cnt, TX_SYNC_COUNT);
	printf("Reading TX_SYNC_CNT. We see: (0x%08x@%04x)\n", READ_MSK(Tx_Sync_Cnt), OFFSET_MSK(Tx_Sync_Cnt));
	printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");

	printf("Initial FIFOs before INIT: tx fifo: %08x rx fifo: %08x\n",
							capture_and_read_msk(OFFSET_MSK(tx_async_fifo_rd_wr_ptr)),
							capture_and_read_msk(OFFSET_MSK(rx_async_fifo_rd_wr_ptr)));

	printf("Initialize MSK block.\n");
	printf("Read MSK_INIT: (0x%08x@%04x)\n", READ_MSK(MSK_Init), OFFSET_MSK(MSK_Init));
	printf("bit 0: 0 is normal operation and 1 is initialize modem (reset condition).\n");
	printf("Assert INIT: Write 1 to MSK_INIT\n");
	WRITE_MSK(MSK_Init, 0x00000001);
	printf("Reading MSK_INIT. We see: (0x%08x@%04x)\n", READ_MSK(MSK_Init), OFFSET_MSK(MSK_Init));

	printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
	printf("OVP_FRAME_MODE: Configuring for frame-driven transmission\n");
	printf("PTT off, loopback off, waiting for OVP frames\n");
	WRITE_MSK(MSK_Control, 0x00000000);	// All control bits off
	printf("Reading back MSK_CONTROL status register. We see: (0x%08x@%04x)\n", READ_MSK(MSK_Control), OFFSET_MSK(MSK_Control));
	printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");

	printf("Reading the MSK_STATUS register, we see: (0x%08x@%04x)\n", READ_MSK(MSK_Status), OFFSET_MSK(MSK_Status));
	printf("Bit 0 is demod_sync(not implemented), bit 1 is tx_enable, bit 2 is rx_enable\n");
	printf("tx_enable is data to DAC enabled. rx_enable is data from ADC enable.\n");
	printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
	printf("TX_BIT_COUNT register is read, we see: (0x%08x@%04x)\n", capture_and_read_msk(OFFSET_MSK(Tx_Bit_Count)), OFFSET_MSK(Tx_Bit_Count));
	printf("This register reads out the count of data requests made by the modem.\n");
	printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
	printf("TX_ENABLE_COUNT register is read and write. It holds the number of clocks on which Tx Enable is active.\n");
	printf("First we read it, we see: (0x%08x@%04x)\n", capture_and_read_msk(OFFSET_MSK(Tx_Enable_Count)), OFFSET_MSK(Tx_Enable_Count));
	printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
	printf("Writing fb, f1, f2 (values are calculated for MSK TX).\n");

	double bitrate, freq_if, delta_f, f1, f2, br_fcw, f1_fcw_tx, f2_fcw_tx, f1_fcw_rx, f2_fcw_rx, tx_sample_rate, tx_rx_sample_ratio;
	double rx_sample_rate __attribute__((unused));

	bitrate = CHANNEL_BITRATE;
	freq_if = IF_FREQUENCY;
	tx_sample_rate = 61440000;
	tx_rx_sample_ratio = 25;								// Rx downsampling implemented in FPGA
	rx_sample_rate = tx_sample_rate / tx_rx_sample_ratio;	// Rx effective sample rate

	delta_f = bitrate/4;
	f1 = freq_if - delta_f;
	f2 = freq_if + delta_f;
	br_fcw = (bitrate/tx_sample_rate) * pow(2.0, 32.0);
	f1_fcw_tx = (f1/tx_sample_rate) * pow(2.0, 32.0);
	f2_fcw_tx = (f2/tx_sample_rate) * pow(2.0, 32.0);
	f1_fcw_rx = (f1/tx_sample_rate) * pow(2.0, 32.0);	// use non-downsampled rate; downsampling happens after the NCO
	f2_fcw_rx = (f2/tx_sample_rate) * pow(2.0, 32.0);

	WRITE_MSK(Fb_FreqWord, (uint32_t) br_fcw);
	WRITE_MSK(TX_F1_FreqWord, (uint32_t) f1_fcw_tx);
	WRITE_MSK(TX_F2_FreqWord, (uint32_t) f2_fcw_tx);

	printf("FB_FREQWORD: (0x%08x@%04x)\n", READ_MSK(Fb_FreqWord), OFFSET_MSK(Fb_FreqWord));
	printf("expecting to see: %f float cast as uint32_t: 0x%08x \n", br_fcw, (uint32_t) br_fcw);
	printf("TX_F1_FREQWORD: (0x%08x@%04x)\n", READ_MSK(TX_F1_FreqWord), OFFSET_MSK(TX_F1_FreqWord));
	printf("expecting to see: %f float cast as uint32_t: 0x%08x \n", f1_fcw_tx, (uint32_t) f1_fcw_tx);
	printf("TX_F2_FREQWORD: (0x%08x@%04x)\n", READ_MSK(TX_F2_FreqWord), OFFSET_MSK(TX_F2_FreqWord));
	printf("expecting to see: %f float cast as uint32_t: 0x%08x \n", f2_fcw_tx, (uint32_t) f2_fcw_tx);
	printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
	printf("Writing f1, f2 (values are calculated for MSK RX).\n");

	WRITE_MSK(RX_F1_FreqWord, (uint32_t) f1_fcw_rx);
	WRITE_MSK(RX_F2_FreqWord, (uint32_t) f2_fcw_rx);

	printf("RX_F1_FREQWORD: (0x%08x@%04x)\n", READ_MSK(RX_F1_FreqWord), OFFSET_MSK(RX_F1_FreqWord));
	printf("expecting to see: %f float cast as uint32_t: 0x%08x \n", f1_fcw_rx, (uint32_t) f1_fcw_rx);
	printf("RX_F2_FREQWORD: (0x%08x@%04x)\n", READ_MSK(RX_F2_FreqWord), OFFSET_MSK(RX_F2_FreqWord));
	printf("expecting to see: %f float cast as uint32_t: 0x%08x \n", f2_fcw_rx, (uint32_t) f2_fcw_rx);

	printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
	printf("Reading the LPF_CONFIG_0 register.\n");
	printf("First we read it, we see: (0x%08x@%04x)\n", READ_MSK(LPF_Config_0), OFFSET_MSK(LPF_Config_0));
	printf("bit 0 is whether or not we freeze the accumulator's current value.\n");
	printf("bit 1 holds the PI accumulator at zero.\n");
	printf("bits 31:16 are the LPF IIR alpha value.\n");
	printf("Reading the LPF_CONFIG_1 register.\n");
	printf("First we read it, we see: (0x%08x@%04x)\n", READ_MSK(LPF_Config_1), OFFSET_MSK(LPF_Config_1));
	printf("bit 23:0 sets the integral gain of the PI controller integrator.\n");
	printf("bit 31:24 sets the integral gain bit shift.\n");
	printf("Reading the LPF_CONFIG_2 register.\n");
	printf("First we read it, we see: (0x%08x@%04x)\n", READ_MSK(LPF_Config_2), OFFSET_MSK(LPF_Config_2));
	printf("bit 23:0 sets the proportional gain of the PI controller integrator.\n");
	printf("bit 31:24 sets the proportional gain bit shift.\n");

	WRITE_MSK(LPF_Config_0, 0x00000002);	//zero and hold accumulators
	printf("wrote 0x00000002 to LPF_Config_0: (0x%08x@%04x)\n", READ_MSK(LPF_Config_0), OFFSET_MSK(LPF_Config_0));
	usleep(num_microseconds);
	WRITE_MSK(LPF_Config_0, 0x00000000);	//accumulators in normal operation
	printf("Wrote all 0 LPF_Config_0: (0x%08x@%04x)\n", READ_MSK(LPF_Config_0), OFFSET_MSK(LPF_Config_0));

	printf("Write some default values for PI gain and bit shift.\n");
	WRITE_MSK(LPF_Config_1, 0x005a5a5a);
	WRITE_MSK(LPF_Config_2, 0x00a5a5a5);
	usleep(num_microseconds);
	printf("LPF_Config_0: (0x%08x@%04x)\n", READ_MSK(LPF_Config_0), OFFSET_MSK(LPF_Config_0));
	printf("LPF_Config_1: (0x%08x@%04x)\n", READ_MSK(LPF_Config_1), OFFSET_MSK(LPF_Config_1));
	printf("LPF_Config_2: (0x%08x@%04x)\n", READ_MSK(LPF_Config_2), OFFSET_MSK(LPF_Config_2));
	usleep(num_microseconds);

	printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
	printf("Set TX_DATA_WIDTH to 8.\n");
	WRITE_MSK(Tx_Data_Width, 0x00000008);

	printf("Read TX_DATA_WIDTH.\n");
	printf("We see: (0x%08x@%04x)\n", READ_MSK(Tx_Data_Width), OFFSET_MSK(Tx_Data_Width));

	printf("Set RX_DATA_WIDTH to 8.\n");
	WRITE_MSK(Rx_Data_Width, 0x00000008);
	printf("Read RX_DATA_WIDTH.\n");
	printf("We see: (0x%08x@%04x)\n", READ_MSK(Rx_Data_Width), OFFSET_MSK(Rx_Data_Width));

	printf("Initialize PRBS_CONTROL to zero. PRBS inactive (bit 0)\n");
	WRITE_MSK(PRBS_Control, 0x00000000);
	printf("We read PRBS_CONTROL: (0x%08x@%04x)\n", READ_MSK(PRBS_Control), OFFSET_MSK(PRBS_Control));

	//initial values of parameterized LPF_CONFIG are set up here
/*
	int32_t proportional_gain =           0x00904073;	// from matlab model 
	int32_t integral_gain =               0x0002D3AD;	//
	int32_t proportional_gain_bit_shift = 0x0000001C;	//
	int32_t integral_gain_bit_shift =     0x00000020;	//
*/

	int32_t proportional_gain =           0x007FFFFF;	// 0x00000243; //0x0012984F for 32 bits 0x00001298 for 24 bits 243 for OE 
	int32_t integral_gain =          	  0x007FFFFF;	// 0x000005A7; //0x0000C067 for 32 bits and 80 for 0E
	int32_t proportional_gain_bit_shift = 18;	//0x0000000E; //0x18 is 24 and 0x20 is 32 and 0E is 14
	int32_t integral_gain_bit_shift =     27;	//0x00000019; //0x18 is 24 and 0x20 is 32 and 0E is 14

	int32_t proportional_config = (proportional_gain_bit_shift << 24) | (proportional_gain & 0x00FFFFFF);
	int32_t integral_config = (integral_gain_bit_shift << 24) | (integral_gain & 0x00FFFFFF);
	printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
	printf("Write proportional and integral gains to LPF_CONFIG_2 and LPF_CONFIG_1.\n");
	printf("Proportional config: (0x%08x) integral config: (0x%08x)\n", proportional_config, integral_config);
	WRITE_MSK(LPF_Config_1, integral_config);
	WRITE_MSK(LPF_Config_2, proportional_config);

	//test xfer register reads
	printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
	printf("The value of axis_xfer_count is: (0x%08x@%04x)\n", capture_and_read_msk(OFFSET_MSK(axis_xfer_count)), OFFSET_MSK(axis_xfer_count));

	//discard 24 receiver samples and 24 NCO Samples
	WRITE_MSK(Rx_Sample_Discard, 0x00001818);
		printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
	printf("Read RX_SAMPLE_DISCARD: (0x%08x@%04x)\n", READ_MSK(Rx_Sample_Discard), OFFSET_MSK(Rx_Sample_Discard));
	printf("bits 0:7 are receiver sample discard and bits 15:8 are NCO sample discard.\n");

	printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
	printf("Read NCO Telemetry:\n");
	printf("f1_nco_adjust: (0x%08x) f2_nco_adjust: (0x%08x)\n", capture_and_read_msk(OFFSET_MSK(f1_nco_adjust)), capture_and_read_msk(OFFSET_MSK(f2_nco_adjust)));
	printf("f1_error:      (0x%08x) f2_error:      (0x%08x)\n", capture_and_read_msk(OFFSET_MSK(f1_error)), capture_and_read_msk(OFFSET_MSK(f2_error)));

	printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
	printf("Deassert INIT: Write 0 to MSK_INIT\n");
	usleep(num_microseconds);
	WRITE_MSK(MSK_Init, 0x00000000);
	printf("Read MSK_INIT: (0x%08x@%04x)\n", READ_MSK(MSK_Init), OFFSET_MSK(MSK_Init));

	printf("Initial FIFOs at INIT: tx fifo: %08x rx fifo: %08x\n",
							capture_and_read_msk(OFFSET_MSK(tx_async_fifo_rd_wr_ptr)),
							capture_and_read_msk(OFFSET_MSK(rx_async_fifo_rd_wr_ptr)));
	
	printf("Rx DMAC FLAGS after init: 0x%08x\n", read_mapped_reg(rx_dmac_register_map, DMAC_FLAGS));
	printf("Rx DMAC Irq_Mask after init: 0x%08x\n", read_mapped_reg(rx_dmac_register_map, DMAC_IRQ_MASK));

	if (start_debug_thread() < 0) {
		printf("Failed to start debug thread\n");
		return -1;
	}

	// Start OVP Timeline Manager
	if (start_timeline_manager() < 0) {
		printf("Failed to start OVP timeline manager\n");
		return -1;
	}

	// Start periodic statistics reporter
	if (start_periodic_statistics_reporter() < 0) {
		printf("Failed to start periodic statistics reporter\n");
		return -1;
	}

	// Start OVP UDP listener
	if (start_ovp_listener() < 0) {
		printf("Failed to start OVP listener\n");
		return -1;	// exit if UDP setup fails
	} else {
		printf("OVP listener started on port %d\n", OVP_UDP_PORT);
		printf("Ready to receive frames from Interlocutor\n");
		printf("Press Ctrl+C to stop\n");
	}

	// Start OVP Receiver
	if (start_ovp_receiver() < 0) {
		printf("Failed to start OVP receiver\n");
		return -1;	// exit if OVP receiver can't be started
	}

	// Main loop for OVP mode - keep program running
	while (!stop) {
		usleep(1e6);	// one second sleep
	}

	cleanup_and_exit();

	return 0;
}
