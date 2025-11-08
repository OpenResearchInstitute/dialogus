// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Opulent Voice transmission code
 *
 * Copyright (C) 2025 IABG mbH and ORI
 * Author: Michael Feilen <feilen_at_iabg.de>,
 * Skunkwrx,
 * and Abraxas3d
 **/


#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <signal.h>
#include <stdio.h>
#include <iio.h>
#include <errno.h>
#include <pthread.h>

#ifdef OVP_FRAME_MODE
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#endif // OVP_FRAME_MODE

/* for pow() */
#include <math.h>

/* for usleep function */
#include <unistd.h>

/* for memory management */
#include <fcntl.h>
#include <termios.h>
#include <sys/mman.h>


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
#define OVP_SINGLE_FRAME_SIZE   134     // Opulent Voice Protocol Packet Size

// Opulent Voice Protocol constants
#define OVP_MAGIC_BYTES 0xBBAADD
#define OVP_HEADER_SIZE 12
#define OVP_PAYLOAD_SIZE 122
#define OVP_UDP_PORT            57372
#define OVP_FRAME_PERIOD_MS     40      // Fixed 40ms timing

// Sizes for final over-the-air frames sent to MSK modulator
#define OVP_SYNC_WORD_SIZE 3       // Size of sync word prepended to each frame
#define OVP_ENCODED_HEADER_SIZE (OVP_HEADER_SIZE * 2)	// Header expands by 2x due to FEC coding
#define OVP_ENCODED_PAYLOAD_SIZE (OVP_PAYLOAD_SIZE * 2)	// Payload expands by 2x due to FEC coding
// Data expands by 2x due to FEC coding, and a sync word may be prepended
#ifdef PREPEND_SYNC_WORD
#define OVP_MODULATOR_FRAME_SIZE (OVP_SYNC_WORD_SIZE + OVP_ENCODED_HEADER_SIZE + OVP_ENCODED_PAYLOAD_SIZE)
#define OVP_MODULATOR_PAYLOAD_OFFSET (OVP_SYNC_WORD_SIZE + OVP_ENCODED_HEADER_SIZE)
#else
#define OVP_MODULATOR_FRAME_SIZE (OVP_ENCODED_HEADER_SIZE + OVP_ENCODED_PAYLOAD_SIZE)
#define OVP_MODULATOR_PAYLOAD_OFFSET (OVP_ENCODED_HEADER_SIZE)
#endif // PREPEND_SYNC_WORD

#ifdef OVP_FRAME_MODE

#ifdef PREPEND_SYNC_WORD
// Our 24-bit sync word is a concatenation of the 11-bit Barker sequence and the 13-bit Barker sequence
// 11-bit Barker: 11100010010
// 13-bit Barker: 1111100110101
// Combined:     1110 0010 0101 1111 0011 0101
// in hex: 0xE25F35
const uint8_t ovp_sync_word[OVP_SYNC_WORD_SIZE] = {0xE2, 0x5F, 0x35};
#endif // PREPEND_SYNC_WORD

// OVP Pipeline Configuration (if using FPGA pipeline)
#define OVP_CFG_PREAMBLE_EN     (1 << 3)
#define OVP_CFG_FEC_EN          (1 << 0)
#define OVP_CFG_WHITENING_EN    (1 << 1)
#define OVP_CFG_SCRAMBLING_EN   (1 << 2)
#define OVP_CFG_POSTAMBLE_EN    (1 << 4)
#endif // OVP_FRAME_MODE

//ENDLESS_PRBS will block STREAMING as is

// we can have RX_ACTIVE with STREAMING
// we can have RX_ACTIVE with ENDLESS_PRBS
// since they are different while loops, it's one or the other
// RX_ACTIVE off chooses the internal digital PRBS loopback.
// RX_ACTIVE on has PTT off and loopback off.
// RF_LOOPBACK has PTT on and loopback off.
// RF_LOOPBACK is used for physically looping back TX to RX
// with a cable and an attenuator.
// OVP_FRAME_MODE sets up a listener for Interlocutor-produced frames.
// these frames come in from a socket and are then delivered to the
// modem through memory mapping. 

// Better to leave these definitions to the compiler command line.
//#define STREAMING
//#define RX_ACTIVE
//#define RF_LOOPBACK
//#define ENDLESS_PRBS
//#define NO_INIT_ON_SUCCESS
//#define OVP_FRAME_MODE

#if defined (RX_ACTIVE) && defined (RF_LOOPBACK)
#error "RX_ACTIVE and RF_LOOPBACK both defined is not valid."
#endif

#if ! defined (RX_ACTIVE) && ! defined (RF_LOOPBACK) && ! defined (OVP_FRAME_MODE)
#error "RX_ACTIVE and RF_LOOPBACK and OVP_FRAME_MODE all undefined, not sure what to do."
#endif

#if defined (STREAMING) && defined (ENDLESS_PRBS)
#error "STREAMING and ENDLESS_PRBS both defined is not valid."
#endif

#if defined (RX_ACTIVE) && ! defined (STREAMING) && ! defined (ENDLESS_PRBS)
#error "STREAMING and ENDLESS_PRBS both undefined in RX_ACTIVE mode, not sure what to do."
#endif

#define TX_SYNC_CTRL_WORD 0x00000000
#define TX_SYNC_COUNT (54200 * 20)	// long preamble for test detection

// Offsets into either DMAC (TX or RX) for registers in that block.
// Only registers we actually use are defined here.
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
int i; //index variable for loops

pthread_t ovp_debug_thread;

// -=-=-=-=-=-=- Opulent Voice Global Variables =-=-=-=-=-=-=-
static pthread_mutex_t timeline_lock = PTHREAD_MUTEX_INITIALIZER;	// shared by all threads

#ifdef OVP_FRAME_MODE
// OVP UDP Interface
static int ovp_udp_socket = -1;
static volatile int ovp_transmission_active = 0;
static struct sockaddr_in ovp_listen_addr;
static pthread_t ovp_udp_thread;
static volatile int ovp_running = 0;

// OVP periodic reporting
static pthread_t ovp_reporter_thread;

// OVP timeline manager
int64_t decision_time = 0;		// us timestamp after which a new frame is late
int ovp_txbufs_this_frame = 0;	// number of UDP frames seen before decision time (should be 1)
static pthread_t ovp_timeline_thread;
static pthread_cond_t timeline_start = PTHREAD_COND_INITIALIZER;
static pthread_mutex_t tls_lock = PTHREAD_MUTEX_INITIALIZER;

// OVP session management variables
static unsigned char active_station_id_binary[6] = {0,0,0,0,0,0};
static char active_station_id_ascii[11] = "";	// 10 chars + null terminator is max possible
static int hang_timer_active = 0;
static int dummy_frames_sent = 0;	// Count of dummy frames sent in this hang time
static int hang_timer_frames = 25;	// Number of dummy frames before ending session
static uint32_t session_ts_base = 0;	// timestamp at start of a session
static int64_t session_T0 = 0;	// Origin of time for this session (microseconds)

// OVP Frame Buffer (sized for actual frames)
static uint8_t ovp_frame_buffer[OVP_SINGLE_FRAME_SIZE];

// OVP modulator frame buffer
// This buffer is used whenever we build a frame to send to the
// modulator. It's global to simplify the building of dummy frames
// and postamble frames, which reuse the sync word and frame header
// from the preceding Opulent Voice frames.
static uint8_t modulator_frame_buffer[OVP_MODULATOR_FRAME_SIZE];

// OVP Statistics
static uint64_t ovp_frames_received = 0;
static uint64_t ovp_frames_processed = 0;
static uint64_t ovp_frame_errors = 0;
static uint64_t ovp_sessions_started = 0;
static uint64_t ovp_sessions_ended = 0;
static uint64_t ovp_dummy_frames_sent = 0;
static uint64_t ovp_untimely_frames = 0;

// Opulent Voice Protocol functions
int start_transmission_session(void);
void end_transmission_session_normally(void);
void abort_transmission_session(void);
void stop_ovp_listener(void);
void stop_timeline_manager(void);
void stop_periodic_statistics_reporter(void);
void print_ovp_statistics(void);
int enable_msk_transmission(void);
int disable_msk_transmission(void);
int process_and_send_ovp_frame_to_txbuf(uint8_t *frame_data, size_t frame_size);
void encode_ovp_header(uint8_t *input, uint8_t *output);
void encode_ovp_payload(uint8_t *input, uint8_t *output);
void create_dummy_frame(void);
void create_postamble_frame(void);
#endif // OVP_FRAME_MODE

void stop_debug_thread(void);

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
unsigned int read_dma(unsigned int *virtual_addr, int offset)
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
unsigned int write_dma(unsigned int *virtual_addr, int offset, unsigned int value)
{       virtual_addr[offset>>2] = value;
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
	uint32_t dummy_read __attribute__((unused));
	
	reg_ptr = (volatile uint32_t *)((char *)msk_register_map + offset);

	// write to capture
	*reg_ptr = 0x0ABD0DEF;	// distinctive 12-bit values in case it shows up in a read

	// Trusting that the CPU is slow enough to make this safe across clock domains
	// without any artificial delay here. Or maybe we need a little delay:
	//usleep(1);
	// or try reading twice in case that helps:
	dummy_read = *reg_ptr;

	// read actual value
	return *reg_ptr;
}

//get the address offset of an MSK register
//value = OFFSET_MSK(MSK_Init);
#define OFFSET_MSK(offset) (offsetof(msk_top_regs_t, offset))


uint64_t get_timestamp(void) {
    uint32_t high, low;

    // Reading global timer counter register
    /* This is the method used in the library code for XTime_GetTime().
       It handles the case where the first read of the two timer regs
       spans a carry between the two timer words. */
    do {
        high = read_dma(timer_register_map, GLOBAL_TMR_UPPER_OFFSET);
        low = read_dma(timer_register_map, GLOBAL_TMR_LOWER_OFFSET);
        // printf("%08x %08x\n", high, low);
    } while (read_dma(timer_register_map, GLOBAL_TMR_UPPER_OFFSET) != high);
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

/* RX is input, TX is output */
enum iodev { RX, TX };

/* common RX and TX streaming params */
struct stream_cfg {
	long long bw_hz; // Analog bandwidth in Hz
	long long fs_hz; // Baseband sample rate in Hz
	long long lo_hz; // Local oscillator frequency in Hz
	const char* rf_port; // Port name
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
	#ifdef OVP_FRAME_MODE
	    // End any active transmission session
	    if (ovp_transmission_active) {
	        abort_transmission_session();
	    }

	    stop_ovp_listener();
		stop_timeline_manager();
		stop_periodic_statistics_reporter();

		printf("\nFinal statistics report:\n");
		print_ovp_statistics();	// make sure one last report goes out, even if it's a duplicate

	#endif // OVP_FRAME_MODE

	stop_debug_thread();

	printf("* Destroying buffers\n");
	if (rxbuf) { iio_buffer_destroy(rxbuf); }
	if (txbuf) { iio_buffer_destroy(txbuf); }

	printf("* Disabling streaming channels\n");
	if (rx0_i) { iio_channel_disable(rx0_i); }
	if (rx0_q) { iio_channel_disable(rx0_q); }
	if (tx0_i) { iio_channel_disable(tx0_i); }
	if (tx0_q) { iio_channel_disable(tx0_q); }

	printf("* Destroying context\n");
	if (ctx) { iio_context_destroy(ctx); }
	exit(0);
}

static void handle_sig(int sig)
{
	printf("Got signal %d, shutting down ...\n", sig);
	stop = true;
}

/* check return value of attr_write function */
static void errchk(int v, const char* what) {
	 if (v < 0) { fprintf(stderr, "Error %d writing to channel \"%s\"\nvalue may not be supported.\n", v, what); cleanup_and_exit(); }
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
	struct iio_device *dev =  iio_context_find_device(ctx, "ad9361-phy");
	IIO_ENSURE(dev && "No ad9361-phy found");
	return dev;
}

/* finds AD9361 streaming IIO devices */
static bool get_ad9361_stream_dev(enum iodev d, struct iio_device **dev)
{
	switch (d) {
	case TX: *dev = iio_context_find_device(ctx, "cf-ad9361-dds-core-lpc"); return *dev != NULL;
	case RX: *dev = iio_context_find_device(ctx, "cf-ad9361-lpc");  return *dev != NULL;
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

#ifdef OVP_FRAME_MODE

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
// Frame data should already be formatted with sync word + scrambling + FEC coding
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
    ptrdiff_t p_inc = iio_buffer_step(txbuf);
    p_end = iio_buffer_end(txbuf);
    
    // Track how much frame data we've sent
    size_t frame_offset = 0;
    
    // Fill TX buffer with frame data bytes
    // The MSK modulator expects raw data bytes, not I/Q samples
    // It will internally convert to MSK I/Q modulation
    for (p_dat = (char *)iio_buffer_first(txbuf, tx0_i); p_dat < p_end; p_dat += p_inc) {
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
// Step 2: iio_push txbuf to the kernel
int push_txbuf_to_msk(void) {

	static uint32_t old_xfer_count = 0;
	static uint32_t push_ts_base;
	uint32_t local_ts_base;

	local_ts_base = get_timestamp_ms();
	printf("OVP: time between iio_buffer_push starts was %dms\n", local_ts_base - push_ts_base);
	push_ts_base = local_ts_base;
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
    WRITE_MSK(MSK_Control, 0x00000001);  // PTT on, loopback off
    
    // Small delay to let hardware settle
    usleep(1000);
    
    uint32_t status = READ_MSK(MSK_Status);
    printf("OVP: MSK_Status after PTT enable: 0x%08x\n", status);
    
    return 0;
}


// Disable PTT and stop MSK transmission  
int disable_msk_transmission(void) {
    printf("OVP: Disabling MSK transmission (PTT OFF)\n");
    WRITE_MSK(MSK_Control, 0x00000000);  // PTT off
    
    uint32_t status = READ_MSK(MSK_Status);
    printf("OVP: MSK_Status after PTT disable: 0x%08x\n", status);
    
    return 0;
}


int start_transmission_session(void) {
    if (ovp_transmission_active) {
        return 0;  // Session already active
    }
    
    printf("OVP: Starting transmission session for station %s\n", active_station_id_ascii);
    session_ts_base = get_timestamp_ms();	// for debug prints

	session_T0 = get_timestamp_us();	// used for timeline management
	decision_time = session_T0 + 60e3;	// 60ms to the middle of the next frame, T0-referenced

    // Send preamble: pure 1100 bit pattern for 40ms (no OVP header)
    uint8_t preamble_frame[OVP_MODULATOR_FRAME_SIZE];  // Full 40ms of data
    
    // Fill with 1100 repeating pattern (0xCC = 11001100 binary)
    for (unsigned int i = 0; i < sizeof(preamble_frame); i++) {
        preamble_frame[i] = 0xCC;  // 1100 1100 repeating
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
    return 0;
}


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

void create_dummy_frame(void) {
	static uint8_t ovp_dummy_payload[] = {  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
											0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
											0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
											0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
											0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
											0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
											0,0 };
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
	// handled by Interlocutor, not this program.
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
		uint16_t barker_11 = 0x0712;  // 11100010010 in binary (right-aligned)

		// Repeat Barker sequence throughout payload
		int bit_pos = 0;
		for (int i=0; i < OVP_ENCODED_PAYLOAD_SIZE; i++) {  // Fill payload section
			uint8_t byte_val = 0;

			for (int bit = 7; bit >= 0; bit--) {  // MSB first
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


/* future FPGA pipeline code
static uint64_t ovp_pipeline_errors = 0;

// OVP Processing Configuration
static uint32_t ovp_pipeline_config = 
    OVP_CFG_PREAMBLE_EN | 
    OVP_CFG_FEC_EN | 
    OVP_CFG_WHITENING_EN |
    OVP_CFG_SCRAMBLING_EN | 
    OVP_CFG_POSTAMBLE_EN;
*/




// Initialize OVP UDP listener
int init_ovp_udp_listener(void) {
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


    printf("OVP: UDP listener initialized on port %d\n", OVP_UDP_PORT);
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

// FEC-encode OVP header
void encode_ovp_header(uint8_t *input, uint8_t *output) {
	memcpy(output, input, OVP_HEADER_SIZE);
	memcpy(output + OVP_HEADER_SIZE, input, OVP_HEADER_SIZE);  // Simple repetition placeholder
}

// FEC-encode OVP payload
void encode_ovp_payload(uint8_t *input, uint8_t *output) {
	memcpy(output, input, OVP_PAYLOAD_SIZE);
	memcpy(output + OVP_PAYLOAD_SIZE, input, OVP_PAYLOAD_SIZE);  // Simple repetition placeholder
}


// Process OVP frame payload through software pipeline
int process_ovp_payload_software(uint8_t *ovp_frame, size_t frame_size) {
	int count = 0;

    if (frame_size != OVP_SINGLE_FRAME_SIZE) {
        printf("OVP: Wrong frame size %d\n", frame_size);
        return -1;
    }

	uint8_t encoded_header[OVP_ENCODED_HEADER_SIZE];
	encode_ovp_header(ovp_frame, encoded_header);

	uint8_t encoded_payload[OVP_ENCODED_PAYLOAD_SIZE];
	encode_ovp_payload(ovp_frame + OVP_HEADER_SIZE, encoded_payload);

	uint8_t *p = modulator_frame_buffer;

	// Components of a transmitted OVP frame:
	memcpy(p, ovp_sync_word, OVP_SYNC_WORD_SIZE);
	p += OVP_SYNC_WORD_SIZE;
	count += OVP_SYNC_WORD_SIZE;

	memcpy(p, encoded_header, OVP_ENCODED_HEADER_SIZE);
	p += OVP_ENCODED_HEADER_SIZE;
	count += OVP_ENCODED_HEADER_SIZE;

	memcpy(p, encoded_payload, OVP_ENCODED_PAYLOAD_SIZE);
	count += OVP_ENCODED_PAYLOAD_SIZE;

	if (count != OVP_MODULATOR_FRAME_SIZE) {
		printf("OVP: Modulator frame size %d does not match expected %d\n", count, OVP_MODULATOR_FRAME_SIZE);
		return -1;
	}

	// The modulator_frame_buffer now contains the full modulator frame
	// ready for transmission via MSK modulator
    return 0;
}

// Process OVP frame payload through FPGA pipeline (when available)
int process_ovp_payload_fpga(uint8_t *payload, size_t payload_size) {
    
    //printf("OVP: Processing %zu payload bytes (FPGA pipeline)\n", payload_size);
    
    // Check if FPGA pipeline is available and configured
    // This would check pipeline registers from msk_top_regs.h
    
    // Wait for pipeline ready
    // (Implementation depends on specific register interface from pluto_msk)
    
    // Write payload to FPGA buffer
    // Configure pipeline settings
    // Trigger processing
    // Wait for completion
    
    // For now, fall back to software processing    
    if (process_ovp_payload_software(payload, payload_size) < 0) {
        return -1;
    }
    
    // Send processed data to existing MSK modulator
    // This integrates with existing MSK transmission functions from msk_rx_init.c
    
    printf("OVP: FPGA processing complete\n");
    return 0;
}

// Complete processing and send an OVP frame to MSK modulator
int process_and_send_ovp_frame_to_txbuf(uint8_t *frame_data, size_t frame_size) {
    
    // Process the frame (no preamble/postamble per frame)
    int result;
    #ifdef USE_FPGA_PIPELINE
    result = process_ovp_payload_fpga(frame_data, frame_size);
    #else // not USE_FPGA_PIPELINE
    result = process_ovp_payload_software(frame_data, frame_size);

    if (result == 0) {
		// Preload txbuf with the OVP frame. We'll push it at the decision time.
		// If we've already preloaded txbuf for this frame slot, this overwrites.
		load_ovp_frame_into_txbuf(modulator_frame_buffer, sizeof(modulator_frame_buffer));
		ovp_txbufs_this_frame++;	// for detection of overwrites.
		// Do not push_txbuf_to_msk() at this time. In case of overwrites,
		// we want to transmit the last one only.
    }
    #endif // not USE_FPGA_PIPELINE
    
    if (result == 0) {
        ovp_frames_processed++;
    } else {
        ovp_frame_errors++;
    }
    
    return result;
}

// Process complete OVP frame (updated with station ID tracking)
int process_ovp_frame(uint8_t *frame_data, size_t frame_size) {
	int result;

    ovp_frames_received++;
    
    // Validate frame
    if (validate_ovp_frame(frame_data, frame_size) < 0) {
        ovp_frame_errors++;
        return -1;
    }
    
    // Extract and store station ID for regulatory compliance
    memcpy(active_station_id_binary, frame_data, 6);  // Station ID length of 6 bytes
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
    struct sockaddr_in client_addr;
    socklen_t client_len;
    ssize_t bytes_received;
	uint32_t recv_ts;
	uint32_t last_recv_ts = 0;

	client_len = sizeof(client_addr);

    while (ovp_running) {

        bytes_received = recvfrom(
            ovp_udp_socket,
            ovp_frame_buffer,
            sizeof(ovp_frame_buffer),
            0, // blocking receive - will wait for frames
            (struct sockaddr*)&client_addr,
            &client_len
        );

		// might be shutting down now, don't grab the mutex
		if (!ovp_running) {
			break;
		}
		pthread_mutex_lock(&timeline_lock);
        
        if (bytes_received == OVP_SINGLE_FRAME_SIZE) {
            //printf("OVP: Received %zd bytes from %s:%d\n", 
            //        bytes_received,
            //        inet_ntoa(client_addr.sin_addr),
            //            ntohs(client_addr.sin_port));
            
			recv_ts = get_timestamp_ms();
			printf("OVP: Received %zd bytes from %s:%d after %dms ending with ", bytes_received,
				   inet_ntoa(client_addr.sin_addr),
				   ntohs(client_addr.sin_port),
				   recv_ts - last_recv_ts);
			last_recv_ts = recv_ts;
			for (int i=OVP_SINGLE_FRAME_SIZE-9; i<OVP_SINGLE_FRAME_SIZE; i++) {
				printf("%02x ", ovp_frame_buffer[i]);
			}
			printf("\n");

            // Process the frame
			process_ovp_frame(ovp_frame_buffer, bytes_received);
		} else if (bytes_received >= 0) {
			printf("OVP: Warning - received unexpected frame size %zd bytes (expected %d)\n", 
					bytes_received, OVP_SINGLE_FRAME_SIZE);
			ovp_frame_errors++;
        } else if (bytes_received < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK && ovp_running) {
                perror("OVP: UDP receive error"); // don't exit on receive errors
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
			printf("timeline grabbing mutex\n");
			pthread_mutex_lock(&timeline_lock);
			if (decision_time != 0) {
				now = get_timestamp_us();
				printf("now %lld Td %lld -> decision in %lld us\n", now, decision_time, decision_time - now);
				if (now >= decision_time) {
					if (ovp_txbufs_this_frame > 0) {
						if (ovp_txbufs_this_frame > 1) {
							ovp_untimely_frames += ovp_txbufs_this_frame - 1;
						}
						ovp_frames_processed++;
						hang_time_dummy_count = 0;
						hang_timer_active = 0;
					} else {
						if (hang_time_dummy_count >= hang_timer_frames) {
							create_postamble_frame();
							hang_time_dummy_count = 0;
							hang_timer_active = 0;
							ovp_transmission_active = 0;
						} else {
							create_dummy_frame();
							ovp_dummy_frames_sent++;
							hang_time_dummy_count++;
							hang_timer_active = 1;
						}
					}
					local_ts_base = get_timestamp_ms();
					iio_buffer_push(txbuf);
					printf("OVP: iio_buffer_push took %dms.\n", get_timestamp_ms()-local_ts_base);
					decision_time += 40e3;
					ovp_txbufs_this_frame = 0;
				}

				// handle normal end of a transmission session (postamble was just pushed)
				if (! ovp_transmission_active) {
					end_transmission_session_normally();
				}
			}
			pthread_mutex_unlock(&timeline_lock);
			printf("timeline released mutex\n");

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

#endif // OVP_FRAME_MODE

#ifdef STREAM_RX_IN_THREAD
// streaming rx
static pthread_t streaming_rx_thread;

// OVP Streaming Receive thread
// Repeatedly try a buffer_refill on the receiver
void* ovp_streaming_rx_thread(__attribute__((unused)) void *arg) {
	int nbytes_rx = 0;
	int32_t refill_ts_base;

	while (!stop) {
		refill_ts_base = get_timestamp_ms();
		nbytes_rx = iio_buffer_refill(rxbuf);
		if (nbytes_rx < 0) {
				if (nbytes_rx == -ETIMEDOUT) {
					printf("Refill timeout in thread (no sync word yet?) took %dms\n", get_timestamp_ms() - refill_ts_base);
					nbytes_rx = 0;
				} else {
					printf("Error refilling buf (in thread) %d\n",(int) nbytes_rx); cleanup_and_exit();
				}
		} else {
			printf("OVP: streaming buffer_refill (threaded) of %d bytes took %dms\n", nbytes_rx, get_timestamp_ms() - refill_ts_base);
		}

		usleep(50000);	// 50ms

	}

	printf("OVP: streaming rx thread exiting\n");
	return NULL;
}

int start_streaming_rx(void) {
    if (pthread_create(&streaming_rx_thread, NULL, ovp_streaming_rx_thread, NULL) != 0) {
        perror("OVP: Failed to create streaming rx thread");
        return -1;
    }
    
    printf("OVP: streaming rx started successfully\n");
    return 0;
}

void stop_streaming_rx(void) {
    if (streaming_rx_thread) {
        pthread_cancel(streaming_rx_thread);
    }

    printf("OVP: streaming rx stopped\n");
}
#endif // STREAM_RX_IN_THREAD

// ---------- Debug Thread -----------------------------

void* ovp_debug_threadf(__attribute__((unused)) void *arg) {
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
			printf("debugthread dmac at %d %08x %08x\n", now,
							read_dma(tx_dmac_register_map, DMAC_FLAGS),
							read_dma(rx_dmac_register_map, DMAC_FLAGS));
// polling the DMA controller interrupts doesn't work, it's too fast and probably
// also happening all at once during an interrupt handler.
//			printf("debugthread dmac at %d %08x %08x %08x\n", now,
//							read_dma(rx_dmac_register_map, DMAC_IRQ_MASK),
//							read_dma(rx_dmac_register_map, DMAC_IRQ_PENDING),
//							read_dma(rx_dmac_register_map, DMAC_IRQ_SOURCE));
		pthread_mutex_unlock(&timeline_lock);

		usleep(10000);
	}
	printf("debug thread exiting\n");
	return NULL;
}

int start_debug_thread(void) {

    if (pthread_create(&ovp_debug_thread, NULL, ovp_debug_threadf, NULL) != 0) {
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


// ------------ Fake Stream Transmission ----------------

#define TXSTREAM_DATAFRAMES	7	// Number of distinct frames in the rotation
#define TXSTREAM_SIZE	(OVP_SINGLE_FRAME_SIZE * 2 * TXSTREAM_DATAFRAMES)
#define TXSTREAM_HEADERSIZE	32	// start with 32 bytes of a simple pattern

// these patterns are designed to be unambiguous even when bit-shifted,
// Just count the number of 1's in eight bits.
uint8_t txstream_header_pattern[TXSTREAM_DATAFRAMES] = { 0x01, 0x03, 0x07, 0x0f, 0x1f, 0x3f, 0x7f };

uint8_t txstream[TXSTREAM_SIZE];
size_t txstream_index;

void txstream_init(void)
{
	size_t index = 0;
	int i;

	// fill each data frame
	for (int frame = 0; frame < TXSTREAM_DATAFRAMES; frame++) {
		// repeat the identifier pattern to make a visually distinct header
		for (i = 0; i < TXSTREAM_HEADERSIZE; i++) {
			txstream[index++] = txstream_header_pattern[frame];
		}

		// fill in a systematic pattern for the remaining bytes
		for ( ; i < OVP_SINGLE_FRAME_SIZE*2; i++) {
			txstream[index++] = (frame & 0x01) ? 0x55 : 0xaa;
		}

	}

	/*
	// stick in some things like sync words, just to see
	txstream[16] = 0xe2;
	txstream[17] = 0x5f;
	txstream[18] = 0x35;

	txstream[16+268] = 0x47;
	txstream[17+268] = 0xfa;
	txstream[18+268] = 0xac;

	txstream[16+268*2] = 0xac;
	txstream[17+268*2] = 0xfa;
	txstream[18+268*2] = 0x47;

	txstream[16+268*3] = 0x35;
	txstream[17+268*3] = 0x5f;
	txstream[18+268*3] = 0xe2;
*/

/* old pattern


	// Create the data we want to transmit repeatedly
	for (size_t i = 0; i < TXSTREAM_SIZE; i++) {
		//txstream[i] = i % 256;
		txstream[i] = i % 16;
		//txstream[i] = rand() % 256;
	}

	for (size_t i=0; i < TXSTREAM_SIZE/2; i++) {
		txstream[i] = 0xaa;
	}
*/



	txstream_index = 0;
}

uint8_t txstream_next_byte(void)
{		
	uint8_t next_byte = txstream[txstream_index++];

	if (txstream_index >= TXSTREAM_SIZE) {
		txstream_index = 0;
	}

	return next_byte;
}

// ------------ Debug: dump IIO buffer ----------------

void dump_buffer(char *buffer_name, struct iio_buffer *buf)
{
	uint8_t *p_dat;
	uint8_t *p_end;

	printf("Dump of %s:\n", buffer_name);

	p_end = iio_buffer_end(rxbuf);
	for (p_dat = (uint8_t *)iio_buffer_first(buf, tx0_i); p_dat < p_end; p_dat++) {
		printf("%02x ", *p_dat);
	}
	printf("\n");

}


// -=-=-=-=-=-=-=-=-=-= MAIN FUNCTION =-=-=-=-=-=-=-=-=-=-=-

/* simple configuration and streaming */
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
	rxcfg.fs_hz = MHZ(61.44);   // 2.5 MS/s rx sample rate
	rxcfg.lo_hz = LO_FREQ;
	rxcfg.rf_port = "A_BALANCED"; // port A (select for rf freq.)

	// OPV hardware TX stream config
	txcfg.bw_hz = RF_BANDWIDTH;
	txcfg.fs_hz = MHZ(61.44);   // 2.5 MS/s tx sample rate
	txcfg.lo_hz = LO_FREQ;
	txcfg.rf_port = "A"; // port A (select for rf freq.)

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


	// set the timeout for refill and push etc.
	// default value is 1000ms
	// argument is the iio context and the number of milliseconds
	ret = iio_context_set_timeout(ctx, 1000);
        if (ret < 0) {
                char timeout_test[256];
                iio_strerror(-(int)ret, timeout_test, sizeof(timeout_test));
                printf("* set_timeout failed : %s\n", timeout_test);
        }
        else {
                printf("* set_timeout returned %d, which is a success.\n", ret);
        }


	printf("* Creating non-cyclic IIO buffers, size %d\n", OVP_MODULATOR_FRAME_SIZE);
    rxbuf = iio_device_create_buffer(rx, OVP_MODULATOR_FRAME_SIZE, false);
	if (!rxbuf) {
		perror("Could not create RX buffer");
		cleanup_and_exit();
	}

	#if !defined(STREAM_RX_IN_THREAD)
	if (iio_buffer_set_blocking_mode(rxbuf, false) < 0) {	//!!!set non-blocking
		printf("Failed to make rxbuf non-blocking\n");
	}
	#endif // STREAM_RX_IN_THREAD

	txbuf = iio_device_create_buffer(tx, OVP_MODULATOR_FRAME_SIZE, false);
	if (!txbuf) {
		perror("Could not create TX buffer");
		cleanup_and_exit();
	}

	printf("Hello World!\n");
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

	unsigned int interface = read_dma(tx_dmac_register_map, DMAC_INTERFACE_DESCRIPTION_1);
	printf("TX DMAC Interface Description (0x%08x@0x%04x)\n", interface, DMAC_INTERFACE_DESCRIPTION_1);
	interface = read_dma(rx_dmac_register_map, DMAC_INTERFACE_DESCRIPTION_1);
	printf("RX DMAC Interface Description (0x%08x@0x%04x)\n", interface, DMAC_INTERFACE_DESCRIPTION_1);

	printf("Writing to scratch register in TX-DMAC.\n");
    write_dma(tx_dmac_register_map, DMAC_SCRATCH, 0x5555AAAA);
    printf("Reading from scratch register in TX-DMAC. We see: (0x%08x@%04x)\n", read_dma(tx_dmac_register_map, DMAC_SCRATCH), DMAC_SCRATCH);
	printf("Reading the TX-DMAC peripheral ID: (0x%08x@%04x)\n", read_dma(tx_dmac_register_map, DMAC_PERIPHERAL_ID), DMAC_PERIPHERAL_ID);
	printf("Writing to scratch register in RX-DMAC.\n");
    write_dma(rx_dmac_register_map, DMAC_SCRATCH, 0x5555AAAA);
    printf("Reading from scratch register in RX-DMAC. We see: (0x%08x@%04x)\n", read_dma(rx_dmac_register_map, DMAC_SCRATCH), DMAC_SCRATCH);
	printf("Reading the RX-DMAC peripheral ID: (0x%08x@%04x)\n", read_dma(tx_dmac_register_map, DMAC_PERIPHERAL_ID), DMAC_PERIPHERAL_ID);

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


	printf("Initialize MSK block.\n");
	printf("Read MSK_INIT: (0x%08x@%04x)\n", READ_MSK(MSK_Init), OFFSET_MSK(MSK_Init));
	printf("bit 0: 0 is normal operation and 1 is initialize modem (reset condition).\n");
	printf("Assert INIT: Write 1 to MSK_INIT\n");
	WRITE_MSK(MSK_Init, 0x00000001);
	printf("Reading MSK_INIT. We see: (0x%08x@%04x)\n", READ_MSK(MSK_Init), OFFSET_MSK(MSK_Init));

	// OVP_FRAME_MODE 
	#ifdef OVP_FRAME_MODE
	printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
	printf("OVP_FRAME_MODE: Configuring for frame-driven transmission\n");
	printf("PTT off, loopback off, waiting for OVP frames\n");
	WRITE_MSK(MSK_Control, 0x00000000);  // All control bits off
	printf("Reading back MSK_CONTROL status register. We see: (0x%08x@%04x)\n", READ_MSK(MSK_Control), OFFSET_MSK(MSK_Control));
	printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
	#endif //OVP_FRAME_MODE

	//Receiver Active
	#ifdef RX_ACTIVE
	printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
    printf("RX_ACTIVE on: Writing MSK_CONTROL register for receiver active.\n");
	printf("PTT and loopback disabled.\n");
	WRITE_MSK(MSK_Control, 0x00000000);
    printf("Reading back MSK_CONTROL status register. We see: (0x%08x@%04x)\n", READ_MSK(MSK_Control), OFFSET_MSK(MSK_Control));
    printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");

	#elif !defined(OVP_FRAME_MODE) && !defined(RF_LOOPBACK)
	//digital loopback
	printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
	printf("RX_ACTIVE off: Writing MSK_CONTROL register for digital loopback.\n");
	printf("PTT and loopback enabled.\n");
	WRITE_MSK(MSK_Control, 0x00000003);
	printf("Reading back MSK_CONTROL status register. We see: (0x%08x@%04x)\n", READ_MSK(MSK_Control), OFFSET_MSK(MSK_Control));
    printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");

	#elif !defined(OVP_FRAME_MODE) && defined(RF_LOOPBACK)
	printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
	printf("RF_LOOPBACK on: Writing MSK_CONTROL register for RF loopback.\n");
	printf("PTT enabled and loopback disabled.\n");
	WRITE_MSK(MSK_Control, 0x00000001);
	printf("Reading back MSK_CONTROL status register. We see: (0x%08x@%04x)\n", READ_MSK(MSK_Control), OFFSET_MSK(MSK_Control));
	printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
	#endif // not OVP_FRAME_MODE and not RF_LOOPBACK

	printf("Reading the MSK_STATUS register, we see: (0x%08x@%04x)\n", READ_MSK(MSK_Status), OFFSET_MSK(MSK_Status));
	printf("Bit 0 is demod_sync(not implemented), bit 1 is tx_enable, bit 2 is rx_enable\n");
	printf("tx_enable is data to DAC enabled. rx_enable is data from ADC enable.\n");
    printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
	printf("TX_BIT_COUNT register is read, we see: (0x%08x@%04x)\n",  capture_and_read_msk(OFFSET_MSK(Tx_Bit_Count)), OFFSET_MSK(Tx_Bit_Count));
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

	WRITE_MSK(LPF_Config_0, 0x00000002); //zero and hold accumulators
	printf("wrote 0x00000002 to LPF_Config_0: (0x%08x@%04x)\n", READ_MSK(LPF_Config_0), OFFSET_MSK(LPF_Config_0));
	usleep(num_microseconds);
	WRITE_MSK(LPF_Config_0, 0x00000000); //accumulators in normal operation
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
	printf("Read TX_DATA_WIDTH, which is the modem transmit input/output data width.\n");
	printf("We see: (0x%08x@%04x)\n", READ_MSK(Tx_Data_Width), OFFSET_MSK(Tx_Data_Width));

	printf("Set TX_DATA_WIDTH to 8.\n");
	WRITE_MSK(Tx_Data_Width, 0x00000008);

	printf("Read TX_DATA_WIDTH.\n");
	printf("We see: (0x%08x@%04x)\n", READ_MSK(Tx_Data_Width), OFFSET_MSK(Tx_Data_Width));

	printf("Read RX_DATA_WIDTH, which is the modem transmit input/output data width.\n");
	printf("We see: (0x%08x@%04x)\n", READ_MSK(Rx_Data_Width), OFFSET_MSK(Rx_Data_Width));
	printf("Set RX_DATA_WIDTH to 32.\n");
	WRITE_MSK(Rx_Data_Width, 0x00000008);
	printf("Read RX_DATA_WIDTH.\n");
	printf("We see: (0x%08x@%04x)\n", READ_MSK(Rx_Data_Width), OFFSET_MSK(Rx_Data_Width));

	printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
	printf("Pseudo Random Binary Sequence control registers are read.\n");
        printf("We read PRBS_CONTROL: (0x%08x@%04x)\n", READ_MSK(PRBS_Control), OFFSET_MSK(PRBS_Control)); 
	printf("bit 0 is PRBS data select. 0 is normal data transmit and 1 is PRBS transmit.\n");
	printf("bit 1 is PRBS error insert. 0 is no error insertion and 1 inserts a bit error in transmit.\n");
	printf("NOTE: error is inserted in both normal and PRBS data selection modes.\n");
	printf("bit 2 clears PRBS counters.\n");
	printf("bit 3 transition is a manual PRBS sync.\n");
	printf("bit 31:16 prbs_sync_threshold.\n");
	printf("We choose a prbs_sync_threshold of 25 percent of bitrate to start out.\n"); 
        printf("We read PRBS_INITIAL_STATE: (0x%08x@%04x)\n", READ_MSK(PRBS_Initial_State), OFFSET_MSK(PRBS_Initial_State));
	printf("This is the PRBS seed value. It sets the starting value of the PRBS generator.\n");
        printf("We read PRBS_POLYNOMIAL: (0x%08x@%04x)\n", READ_MSK(PRBS_Polynomial), OFFSET_MSK(PRBS_Polynomial));
	printf("Bit positions set to 1 indicate polynomial feedback positions.\n");
	printf("We read PRBS_ERROR_MASK: (0x%08x@%04x)\n", READ_MSK(PRBS_Error_Mask), OFFSET_MSK(PRBS_Error_Mask));
	printf("Bit positions set to 1 indicate bits that are inverted when a bit error is inserted.\n");
        printf("We read PRBS_BIT_COUNT: (0x%08x@%04x)\n", capture_and_read_msk(OFFSET_MSK(PRBS_Bit_Count)), OFFSET_MSK(PRBS_Bit_Count));
	printf("Number of bits received by the PRBS monitor since last BER\n");
	printf("can be calculated as the ratio of received bits to errored-bits.\n");
	printf("We read PRBS_ERROR_COUNT: (0x%08x@%04x)\n", capture_and_read_msk(OFFSET_MSK(PRBS_Error_Count)), OFFSET_MSK(PRBS_Error_Count));
	printf("Number of errored-bits received by the PRBS monitor since last BER\n");
	printf("can be calculated as the ratio of received bits to errored-bits.\n");
	printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
	printf("We read LPF_ACCUM_F1: (0x%08x@%04x)\n", capture_and_read_msk(OFFSET_MSK(LPF_Accum_F1)), OFFSET_MSK(LPF_Accum_F1));
	printf("PI controller accumulator value.\n");
	printf("We read LPF_ACCUM_F2: (0x%08x@%04x)\n", capture_and_read_msk(OFFSET_MSK(LPF_Accum_F2)), OFFSET_MSK(LPF_Accum_F2));
	printf("PI controller accumulator value.\n");


    printf("Initialize PRBS_CONTROL to zero. PRBS inactive (bit 0)\n");
    WRITE_MSK(PRBS_Control, 0x00000000);
    printf("We read PRBS_CONTROL: (0x%08x@%04x)\n", READ_MSK(PRBS_Control), OFFSET_MSK(PRBS_Control)); 
#if !defined(OVP_FRAME_MODE) && !defined(STREAMING)
	printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
	printf("Attempt to set up PRBS for modes that use it.\n");

	//printf("Write 0x34EE0001 to PRBS_CONTROL. PRBS active (bit 0)\n");
	//printf("auto sync threshold of 25 percent of bit rate, which is 0x34EE.\n");
	//WRITE_MSK(PRBS_Control, 0x34EE0001);
	printf("Write 0xff000001 to PRBS_CONTROL. PRBS active (bit 0)\n");
	printf("auto sync threshold of 0xff00 (nearly max).\n");
	WRITE_MSK(PRBS_Control, 0xFF000001);

	printf("We read PRBS_CONTROL: (0x%08x@%04x)\n", READ_MSK(PRBS_Control), OFFSET_MSK(PRBS_Control));
	printf("Write a value to PRBS_INITIAL_STATE, as the seed.\n");
        WRITE_MSK(PRBS_Initial_State, 0x8E7589FD);
        printf("We read PRBS_INITIAL_STATE: (0x%08x@%04x)\n", READ_MSK(PRBS_Initial_State), OFFSET_MSK(PRBS_Initial_State));
//	printf("Write 0xA3000000 to PRBS_POLYNOMIAL (32,30,26,25), a max length Fibonacci sequence generators.\n");
//	write_dma(msk_virtual_addr, PRBS_POLYNOMIAL, 0xA3000000);
	printf("Write 0x48000000 to PRBS_POLYNOMIAL (31,28), a max length Fibonacci sequence generators.\n");
//	write_dma(msk_virtual_addr, 0x48, 0x48000000);
        WRITE_MSK(PRBS_Polynomial, 0x48000000);
        printf("We read PRBS_POLYNOMIAL: (0x%08x@%04x)\n", READ_MSK(PRBS_Polynomial), OFFSET_MSK(PRBS_Polynomial));
	printf("Write 0x00000001 to PRBS_ERROR_MASK.\n");
//	write_dma(msk_virtual_addr, 0x4c, 0x00000001);
	WRITE_MSK(PRBS_Error_Mask, 0x00000001);
        printf("We read PRBS_ERROR_MASK: (0x%08x@%04x)\n", READ_MSK(PRBS_Error_Mask), OFFSET_MSK(PRBS_Error_Mask));
#endif // not OVP_FRAME_MODE and not STREAMING_MODE


	//initial values of parameterized LPF_CONFIG are set up here
/*
	int32_t proportional_gain =           0x00904073; // from matlab model 
	int32_t integral_gain =               0x0002D3AD; //
	int32_t proportional_gain_bit_shift = 0x0000001C; //
	int32_t integral_gain_bit_shift =     0x00000020; //
*/

	int32_t proportional_gain =           0x007FFFFF; //0x00000243; //0x0012984F for 32 bits 0x00001298 for 24 bits 243 for OE 
	int32_t integral_gain =          	  0x007FFFFF; //     0x000005A7; //0x0000C067 for 32 bits and 80 for 0E
	int32_t proportional_gain_bit_shift = 18; //0x0000000E; //0x18 is 24 and 0x20 is 32 and 0E is 14
	int32_t integral_gain_bit_shift =     27; //0x00000019; //0x18 is 24 and 0x20 is 32 and 0E is 14

#ifndef OVP_FRAME_MODE
	// If we are searching for good gains, use these increments. Negative for decrement. Zero for constant gain.
	int32_t proportional_gain_increment __attribute__((unused)) = 0; //- 0x00001000;
	int32_t integral_gain_increment __attribute__((unused)) = 0;
	int32_t proportional_shift_increment __attribute__((unused)) = 0;
	int32_t integral_shift_increment __attribute__((unused))= 0;
#endif // not OVP_FRAME_MODE

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

#ifdef STREAM_RX_IN_THREAD
	// Start streaming receive manager
	if (start_streaming_rx() < 0) {
		printf("Failed to start streaming receiver\n");
		return -1;
	}
#endif // STREAM_RX_IN_THREAD

#if !defined(OVP_FRAME_MODE) && !defined(STREAMING)
	//loop variables
	int max_without_zeros = 0;
	int spectacular_success = 0;

	// ENDLESS_PRBS runs PRBS based transmit indefinitely
	#ifdef ENDLESS_PRBS
	while(!stop) {
	#endif

	uint64_t reporting_interval = REPORTING_INTERVAL;
	uint64_t next_reporting_timestamp = get_timestamp() + reporting_interval;	// report status periodically starting now

	while(percent_error > 0.1){

		if(stop){
			break;
		}

	    printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
		printf("100 (or more) buckets of bits on the bus.\n");

		while (get_timestamp() < next_reporting_timestamp) {
			usleep(one_bit_time);
		}

		printf("(1) Total PRBS_BIT_COUNT:   (0x%08x@%04x)\n", capture_and_read_msk(OFFSET_MSK(PRBS_Bit_Count)), OFFSET_MSK(PRBS_Bit_Count));
		printf("(1) Total PRBS_ERROR_COUNT: (0x%08x@%04x)\n", capture_and_read_msk(OFFSET_MSK(PRBS_Error_Count)), OFFSET_MSK(PRBS_Error_Count));
		percent_error = (   (double)(capture_and_read_msk(OFFSET_MSK(PRBS_Error_Count))  /  (double)(capture_and_read_msk(OFFSET_MSK(PRBS_Bit_Count)))    )*100;
		printf("(1) %2.1f %d %d 0x%08x 0x%08x\n", percent_error, capture_and_read_msk(OFFSET_MSK(LPF_Accum_F1)), capture_and_read_msk(OFFSET_MSK(LPF_Accum_F2)),
								READ_MSK(LPF_Config_2), READ_MSK(LPF_Config_1));

		printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
		printf("Read NCO Telemetry:\n");
		printf("f1_nco_adjust: (0x%08x) f2_nco_adjust: (0x%08x)\n", capture_and_read_msk(OFFSET_MSK(f1_nco_adjust)), capture_and_read_msk(OFFSET_MSK(f2_nco_adjust)));
		printf("f1_error:      (0x%08x) f2_error:      (0x%08x)\n", capture_and_read_msk(OFFSET_MSK(f1_error)), capture_and_read_msk(OFFSET_MSK(f2_error)));

		print_rssi();
		print_timestamp();

		if (isnan(percent_error)) {
			printf("BOOM!\n");
			stop = true;
		}

		next_reporting_timestamp += reporting_interval;
		printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");

		if (percent_error > 49.0){

/* 			//Differential encoding put in as of 15 Dec 2024
			printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
			printf("Toggle RX_INVERT to test 180 degree phase shift.\n");
			WRITE_MSK(MSK_Control,(READ_MSK(MSK_Control)^0x00000004));
			usleep(num_microseconds);

			printf("We read MSK_CONTROL: (0x%08x@%04x)\n", READ_MSK(MSK_Control), OFFSET_MSK(MSK_Control));
                        // usleep(num_microseconds*1000); //test increasing this delay?
*/

			printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
			printf("percent_error was greater than 49.0 percent so we resync.\n");
			printf("resync PRBS by toggling bit 3 of PRBS_CONTROL.\n");
			WRITE_MSK(PRBS_Control, (READ_MSK(PRBS_Control)^0x00000008));
			usleep(num_microseconds);
			printf("We read PRBS_CONTROL: (0x%08x@%04x)\n", READ_MSK(PRBS_Control), OFFSET_MSK(PRBS_Control));

			printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
			printf("reset the PRBS_BIT_COUNT and PRBS_ERROR_COUNT counters.\n");
			printf("Toggle bit 2 of PRBS_CONTROL.\n");
			WRITE_MSK(PRBS_Control, (READ_MSK(PRBS_Control)^0x00000004));
			usleep(num_microseconds);

			printf("We read PRBS_CONTROL: (0x%08x@%04x)\n", READ_MSK(PRBS_Control), OFFSET_MSK(PRBS_Control));
			printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
			printf("We read MSK_CONTROL: (0x%08x@%04x)\n", READ_MSK(MSK_Control), OFFSET_MSK(MSK_Control));

			max_without_zeros++;
			//printf("Increment max_without_zeros here.\nmax_without_zeros now equals: %d\n", max_without_zeros);

			if (max_without_zeros > 20) {
				//increment proportional and/or integral gains here
				proportional_gain = proportional_gain + proportional_gain_increment;
				integral_gain = integral_gain + integral_gain_increment;

				proportional_gain_bit_shift += proportional_shift_increment;
				if (proportional_gain_bit_shift < 0) {
					proportional_gain_bit_shift = 32;
				} else if (proportional_gain_bit_shift > 32) {
					proportional_gain_bit_shift = 0;
				}
		
				integral_gain_bit_shift += integral_shift_increment;
				if (integral_gain_bit_shift < 0) {
					integral_gain_bit_shift = 32;
				} else if (integral_gain_bit_shift > 32) {
					integral_gain_bit_shift = 0;
				}
		

				int32_t proportional_config = (proportional_gain_bit_shift << 24) | (proportional_gain & 0x00FFFFFF);
				int32_t integral_config = (integral_gain_bit_shift << 24) | (integral_gain & 0x00FFFFFF);
				printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
				printf("Write proportional and integral gains to LPF_CONFIG_2 and LPF_CONFIG_1.\n");
				printf("Proportional config: (0x%08x) integral config: (0x%08x)\n", proportional_config, integral_config);
				WRITE_MSK(LPF_Config_1, integral_config);
				WRITE_MSK(LPF_Config_2, proportional_config);


				max_without_zeros = 0;

				printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
				printf("max_without_zeros exceeded. Giving up on this gain pair.\n");

				printf("Assert RX INIT: Write 1 to bit 2 of MSK_INIT\n");
				WRITE_MSK(MSK_Init, 0x00000004);
				printf("Reading MSK_INIT. We see: (0x%08x@%04x)\n", READ_MSK(MSK_Init), OFFSET_MSK(MSK_Init));
				usleep(num_microseconds);
				printf("De-Assert INIT: Write 0 to MSK_INIT\n");
				WRITE_MSK(MSK_Init, 0x00000000);
				printf("Reading MSK_INIT. We see: (0x%08x@%04x)\n", READ_MSK(MSK_Init), OFFSET_MSK(MSK_Init));
				usleep(num_microseconds);



				printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
				printf("Zero the accumulators.\n");
				printf("Bit 1 of LPF_CONFIG_0 is set and then cleared.\n");
				printf("31:16 is the filter alpha.\n");
				printf("Write 0xXXXX0002 to LPF_CONFIG_0.\n");
				WRITE_MSK(LPF_Config_0, 0x00000002);
				usleep(num_microseconds);
				printf("We read LPF_CONFIG_0: (0x%08x@%04x)\n", READ_MSK(LPF_Config_0), OFFSET_MSK(LPF_Config_0));
				WRITE_MSK(LPF_Config_0, 0x00000000);
				usleep(num_microseconds);
				printf("We read LPF_CONFIG_0: (0x%08x@%04x)\n", READ_MSK(LPF_Config_0), OFFSET_MSK(LPF_Config_0));
				printf("We read LPF_ACCUM_F1: (0x%08x@%04x)\n", capture_and_read_msk(OFFSET_MSK(LPF_Accum_F1)), OFFSET_MSK(LPF_Accum_F1));
				printf("F1 PI controller accumulator value.\n");
				printf("We read LPF_ACCUM_F2: (0x%08x@%04x)\n", capture_and_read_msk(OFFSET_MSK(LPF_Accum_F2)), OFFSET_MSK(LPF_Accum_F2));
				printf("F2 PI controller accumulator value.\n");

				printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
				printf("percent_error was greater than 49.0 percent so we resync.\n");
				printf("resync PRBS by toggling bit 3 of PRBS_CONTROL.\n");
				WRITE_MSK(PRBS_Control, (READ_MSK(PRBS_Control)^0x00000008));
				usleep(num_microseconds);
				printf("We read PRBS_CONTROL: (0x%08x@%04x)\n", READ_MSK(PRBS_Control), OFFSET_MSK(PRBS_Control));

				printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
				printf("reset the PRBS_BIT_COUNT and PRBS_ERROR_COUNT counters.\n");
				printf("Toggle bit 2 of PRBS_CONTROL.\n");
				WRITE_MSK(PRBS_Control, (READ_MSK(PRBS_Control)^0x00000004));
				usleep(num_microseconds);
				printf("We read PRBS_CONTROL: (0x%08x@%04x)\n", READ_MSK(PRBS_Control), OFFSET_MSK(PRBS_Control));
				printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
				printf("We read MSK_CONTROL: (0x%08x@%04x)\n", READ_MSK(MSK_Control), OFFSET_MSK(MSK_Control));

				next_reporting_timestamp = get_timestamp() + reporting_interval;	// we've spent some time re-initing, don't count that.

			} //end of if MAX_WITHOUT_ZEROS > 20
		}// end of if percent_error > 49.0
	}// end of while percent_error > 0.1
	printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
	printf("fell out of while percent_error > 0.1 (hey, low error!) \n");

	spectacular_success = 0; //start with a fresh slate for success
	max_without_zeros = 0; //clear out any partial counts from above


	while(percent_error < 49.0){

		if(stop){
			break;
		}
		printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
		printf("less than 49.0 percent error here.\n");
		printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");


		printf("reset the PRBS_BIT_COUNT and PRBS_ERROR_COUNT counters.\n");
		printf("Toggle bit 2 of PRBS_CONTROL.\n");
		WRITE_MSK(PRBS_Control, (READ_MSK(PRBS_Control)^0x00000004));
		usleep(num_microseconds);
		printf("We read PRBS_CONTROL: (0x%08x@%04x)\n", READ_MSK(PRBS_Control), OFFSET_MSK(PRBS_Control));
		printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
		printf("We read MSK_CONTROL: (0x%08x@%04x)\n", READ_MSK(MSK_Control), OFFSET_MSK(MSK_Control));
		printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");

		next_reporting_timestamp = get_timestamp() + reporting_interval;	// we've spent some time re-initing, don't count that.

		printf("100 buckets of bits on the bus, 100 buckets of bits.\n");

		while (get_timestamp() < next_reporting_timestamp) {
			usleep(one_bit_time);
		}

		printf("(2) Total PRBS_BIT_COUNT:   (0x%08x@%04x)\n", capture_and_read_msk(OFFSET_MSK(PRBS_Bit_Count)), OFFSET_MSK(PRBS_Bit_Count));
		printf("(2) Total PRBS_ERROR_COUNT: (0x%08x@%04x)\n", capture_and_read_msk(OFFSET_MSK(PRBS_Error_Count)), OFFSET_MSK(PRBS_Error_Count));
		percent_error = ((double)(capture_and_read_msk(OFFSET_MSK(PRBS_Error_Count)))/(double)(capture_and_read_msk(OFFSET_MSK(PRBS_Bit_Count))))*100;
		printf("(2) %2.1f %d %d 0x%08x 0x%08x\n", percent_error, capture_and_read_msk(OFFSET_MSK(LPF_Accum_F1)), capture_and_read_msk(OFFSET_MSK(LPF_Accum_F2)),
				READ_MSK(LPF_Config_2), READ_MSK(LPF_Config_1));


		printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
		printf("Read NCO Telemetry:\n");
		printf("f1_nco_adjust: (0x%08x) f2_nco_adjust: (0x%08x)\n", capture_and_read_msk(OFFSET_MSK(f1_nco_adjust)), capture_and_read_msk(OFFSET_MSK(f2_nco_adjust)));
		printf("f1_error:      (0x%08x) f2_error:      (0x%08x)\n", capture_and_read_msk(OFFSET_MSK(f1_error)), capture_and_read_msk(OFFSET_MSK(f2_error)));

		print_rssi();
		print_timestamp();

		if (isnan(percent_error)) {
			printf("BOOM!\n");
			stop = true;
		}
		printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");

		spectacular_success++;

#ifndef NO_INIT_ON_SUCCESS
		if(spectacular_success > 20) {
			spectacular_success = 0;
			printf("Paul, we had a good run.\n");
			printf("(3) %2.1f %d %d 0x%08x 0x%08x\n", percent_error, capture_and_read_msk(OFFSET_MSK()LPF_Accum_F1), capture_and_read_msk(OFFSET_MSK(LPF_Accum_F2)), 
					READ_MSK(LPF_Config_2), READ_MSK(LPF_Config_1));

			printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
			printf("Read NCO Telemetry:\n");
			printf("f1_nco_adjust: (0x%08x) f2_nco_adjust: (0x%08x)\n", capture_and_read_msk(OFFSET_MSK(f1_nco_adjust)), capture_and_read_msk(OFFSET_MSK(f2_nco_adjust)));
			printf("f1_error:      (0x%08x) f2_error:      (0x%08x)\n", capture_and_read_msk(OFFSET_MSK(f1_error)), capture_and_read_msk(OFFSET_MSK(f2_error)));

			print_rssi();
			print_timestamp();
			printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");

			break;
		}
#endif // NO_INIT_ON_SUCCESS
	} //end of while percent_error < 49.0


	//time to test one or both of new PI gains
	printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
	printf("After spectacular success, we may increment gain pair here.\n");
	proportional_gain = proportional_gain + proportional_gain_increment;
	integral_gain = integral_gain + integral_gain_increment;

	proportional_gain_bit_shift += proportional_shift_increment;
	if (proportional_gain_bit_shift < 0) {
		proportional_gain_bit_shift = 32;
	} else if (proportional_gain_bit_shift > 32) {
		proportional_gain_bit_shift = 0;
	}

	integral_gain_bit_shift += integral_shift_increment;
	if (integral_gain_bit_shift < 0) {
		integral_gain_bit_shift = 32;
	} else if (integral_gain_bit_shift > 32) {
		integral_gain_bit_shift = 0;
	}

	proportional_config = (proportional_gain_bit_shift << 24) | (proportional_gain & 0x00FFFFFF);
	integral_config = (integral_gain_bit_shift << 24) | (integral_gain & 0x00FFFFFF);
	printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
	printf("Write proportional and integral gains to LPF_CONFIG_2 and LPF_CONFIG_1.\n");
	printf("Proportional config: (0x%08x) integral config: (0x%08x)\n", proportional_config, integral_config);
	WRITE_MSK(LPF_Config_1, integral_config);
	WRITE_MSK(LPF_Config_2, proportional_config);

	printf("Assert RX INIT: Write 1 to bit 2 of MSK_INIT\n");
	WRITE_MSK(MSK_Init, 0x00000004);
	printf("Reading MSK_INIT. We see: (0x%08x@%04x)\n", READ_MSK(MSK_Init), OFFSET_MSK(MSK_Init));
	usleep(num_microseconds);
	printf("De-Assert INIT: Write 0 to MSK_INIT\n");
	WRITE_MSK(MSK_Init, 0x00000000);
	printf("Reading MSK_INIT. We see: (0x%08x@%04x)\n", READ_MSK(MSK_Init), OFFSET_MSK(MSK_Init));
	usleep(num_microseconds);

	printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
	printf("Zero the accumulators.\n");
	printf("Bit 1 of LPF_CONFIG_0 is set and then cleared.\n");
	printf("31:16 is the filter alpha.\n");
	printf("Write 0xXXXX0002 to LPF_CONFIG_0.\n");
	WRITE_MSK(LPF_Config_0, 0x00000002);
	usleep(num_microseconds);
	printf("We read LPF_CONFIG_0: (0x%08x@%04x)\n", READ_MSK(LPF_Config_0), OFFSET_MSK(LPF_Config_0));
	usleep(num_microseconds);
	WRITE_MSK(LPF_Config_0, 0x00000000);
	usleep(num_microseconds);
	printf("We read LPF_CONFIG_0: (0x%08x@%04x)\n", READ_MSK(LPF_Config_0), OFFSET_MSK(LPF_Config_0));

	printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
	printf("resync PRBS by toggling bit 3 of PRBS_CONTROL.\n");
	WRITE_MSK(PRBS_Control, (READ_MSK(PRBS_Control)^0x00000008));
	usleep(num_microseconds);
	printf("We read PRBS_CONTROL: (0x%08x@%04x)\n", READ_MSK(PRBS_Control), OFFSET_MSK(PRBS_Control));

	printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
	printf("reset the PRBS_BIT_COUNT and PRBS_ERROR_COUNT counters.\n");
	printf("Toggle bit 2 of PRBS_CONTROL.\n");
	WRITE_MSK(PRBS_Control, (READ_MSK(PRBS_Control)^0x00000004));
	usleep(num_microseconds);
	printf("We read PRBS_CONTROL: (0x%08x@%04x)\n", READ_MSK(PRBS_Control), OFFSET_MSK(PRBS_Control));
	printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
	printf("We read MSK_CONTROL: (0x%08x@%04x)\n", READ_MSK(MSK_Control), OFFSET_MSK(MSK_Control));

	//in the case that RX_INIT causes errors to begin to accumulate at this point then
	//percent_error needs to not skip loop 1. try setting it to 55.0 (default) here.
	percent_error = 55.0;

	next_reporting_timestamp = get_timestamp() + reporting_interval;	// we've spent some time re-initing, don't count that.

	#ifdef ENDLESS_PRBS
	}
	#endif //ENDLESS_PRBS
#endif // not OVP_FRAME_MODE and not STREAMING_MODE

if (start_debug_thread() < 0) {
	printf("Failed to start debug thread\n");
	return -1;
}

#ifdef OVP_FRAME_MODE
    printf("OVP Frame Mode Enabled\n");

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
        return -1; // exit if UDP setup fails
    } else {
        printf("OVP listener started on port %d\n", OVP_UDP_PORT);
        printf("Ready to receive frames from Interlocutor\n");
        printf("Press Ctrl+C to stop\n");
    }


    // Main loop for OVP mode - keep program running
    while (!stop) {
        usleep(1e6);  // one second sleep
    }
#endif // OVP_FRAME_MODE



#ifdef STREAMING

	// RX and TX sample counters
	size_t nrx = 0;
	size_t ntx = 0;
    uint32_t old_data __attribute__((unused)) = 0, new_data __attribute__((unused)) = 0;
	static uint32_t push_ts_base;
	uint32_t local_ts_base;
	uint32_t refill_ts_base __attribute__((unused));

	printf("* Initializing fake transmit stream\n");
	txstream_init();

	printf("* Starting IO streaming (press CTRL+C to cancel)\n");
	while (!stop)
	{
		ssize_t nbytes_rx, nbytes_tx;
		char *p_dat, *p_end;
		ptrdiff_t p_inc;

		//dump_buffer("txbuf", txbuf);

		local_ts_base = get_timestamp_ms();
		printf("OVP: time between stream IIO starts was %dms\n", local_ts_base - push_ts_base);
		push_ts_base = local_ts_base;

#if !defined(RX_ACTIVE)
		// Schedule TX buffer
		// nbytes_tx = iio_buffer_push(txbuf);

#if !defined(STREAM_RX_IN_THREAD)
		//!!! try to keep the FIFOs happy by checking RX every time. rxbuf must be non-blocking.
		nbytes_rx = iio_buffer_refill(rxbuf);
		if (nbytes_rx == -EAGAIN) {
			printf("buffer_refill says EAGAIN\n");
		} else if (nbytes_rx > 0) {
			printf("streaming rx fetched %d bytes\n", nbytes_rx);
		} else {
			printf("streaming rx reports error %d\n", -nbytes_rx);
		}
#endif // STREAM_RX_IN_THREAD

    	nbytes_tx = iio_buffer_push(txbuf);
		printf("OVP: streaming iio_buffer_push of %d bytes took %dms.\n", nbytes_tx, get_timestamp_ms()-local_ts_base);


		// Use AXI stream transfer count register to see if data is getting to our logic
//!!!
		old_data = new_data;
		new_data = capture_and_read_msk(OFFSET_MSK(axis_xfer_count));
		printf("AXIS_XFER_COUNT delta after iio_buffer_push is (0x%08x)\n", new_data - old_data);
		printf("AXIS_XFER_COUNT after iio_buffer_push is (0x%08x)\n", new_data);
		printf("TX_BIT_COUNT register is read, we see: (0x%08x@%04x)\n",  capture_and_read_msk(OFFSET_MSK(Tx_Bit_Count)), OFFSET_MSK(Tx_Bit_Count));
		printf("TX_BIT_COUNT decimal %d %lld\n", READ_MSK(Tx_Bit_Count), get_timestamp_us());


		if (nbytes_tx < 0) {
			printf("Error pushing buf %d\n", (int) nbytes_tx); cleanup_and_exit();
		}
#else
		nbytes_tx = 0;
#endif //not RX_ACTIVE

// disable rx: #if 0
#if defined(RX_ACTIVE) && !defined(STREAM_RX_IN_THREAD)
		// Refill RX buffer
		refill_ts_base = get_timestamp_ms();
		nbytes_rx = iio_buffer_refill(rxbuf);
		if (nbytes_rx < 0) {
				if (nbytes_rx == -ETIMEDOUT) {
					printf("Refill timeout with STREAMING (no sync word yet?)\n");
					nbytes_rx = 0;
				} else {
					printf("Error refilling buf %d\n",(int) nbytes_rx); cleanup_and_exit();
				}
		} else {
			printf("OVP: streaming buffer_refill of %d bytes took %dms\n", nbytes_rx, get_timestamp_ms() - refill_ts_base);
		}

		// READ: Get pointers to RX buf and read IQ from RX buf port 0
		p_inc = iio_buffer_step(rxbuf);
		p_end = iio_buffer_end(rxbuf);
		for (p_dat = (char *)iio_buffer_first(rxbuf, rx0_i); p_dat < p_end; p_dat += p_inc) {
			// Example: swap I and Q
			//const int16_t i = ((int16_t*)p_dat)[0]; // Real (I)
			//const int16_t q = ((int16_t*)p_dat)[1]; // Imag (Q)
			// ((int16_t*)p_dat)[0] = q;	// dumb example transformation
			// ((int16_t*)p_dat)[1] = i;
		}
// disable rx: #endif // 0
#else
		nbytes_rx = 0;
#endif // RX_ACTIVE & !STREAM_RX_IN_THREAD

		// WRITE: Get pointers to TX buf and write IQ to TX buf port 0
		uint8_t value;
		printf("Filling txbuf: ");
		p_inc = iio_buffer_step(txbuf);
		p_end = iio_buffer_end(txbuf);
		for (p_dat = (char *)iio_buffer_first(txbuf, tx0_i); p_dat < p_end; p_dat += p_inc) {
			// Example: fill with zeros
			// 12-bit sample needs to be MSB aligned so shift by 4
			// https://wiki.analog.com/resources/eval/user-guides/ad-fmcomms2-ebz/software/basic_iq_datafiles#binary_format
			//((int16_t*)p_dat)[0] = 0x0800 << 4; // Real (I)
			//((int16_t*)p_dat)[1] = 0x0800 << 4; // Imag (Q)

			//((int16_t*)p_dat)[0] = 0x00a5; // Real (I)
			//((int16_t*)p_dat)[1] = 0x0000; // Imag (Q)
			value = txstream_next_byte();
			printf("%02x ", value);
			((int16_t*)p_dat)[0] = value;
			((int16_t*)p_dat)[1] = 0;
		}
		printf("\n");

		// Sample counter increment and status output
		nrx += nbytes_rx / iio_device_get_sample_size(rx);
		ntx += nbytes_tx / iio_device_get_sample_size(tx);
		printf("\tRX %8.2f MSmp, TX %8.2f MSmp\n", nrx/1e6, ntx/1e6);

		//dump_buffer("rxbuf", rxbuf);
		
// disable rx: #if 0
#if defined(RX_ACTIVE) && !defined(STREAM_RX_IN_THREAD) 
		if (nbytes_rx > 0) {
			// dump received buffer for visual check
			printf("Received: ");
			p_inc = iio_buffer_step(rxbuf);
			p_end = iio_buffer_end(rxbuf);
			for (p_dat = (char *)iio_buffer_first(rxbuf, rx0_i); p_dat < p_end; p_dat += p_inc) {
				printf("%02x ", ((int16_t*)p_dat)[0] & 0x00ff);
			}
			printf("\n");
		}
// disable rx: #endif // 0
#endif // RX_ACTIVE & !STREAM_RX_IN_THREAD
	}
#endif // STREAMING

	cleanup_and_exit();

	return 0;
}
