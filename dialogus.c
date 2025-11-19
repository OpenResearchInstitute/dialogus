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
#include <iio.h>
#include <math.h>
#include <netinet/in.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

#include "debugthread.h"
#include "fec.h"
#include "frame_header.h"
#include "interleaver.h"
#include "numerology.h"
#include "radio.h"
#include "randomizer.h"
#include "receiver.h"
#include "registers.h"
#include "statistics.h"
#include "timestamp.h"
#include "udp_listener.h"
#include "udp_socket.h"

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


#define TX_SYNC_CTRL_WORD 0x00000000
#define TX_SYNC_COUNT 0	// not currently used


//-=-=-=-=-=-=-= GLOBAL VARIABLES -=-=-=-=-=-=-=-=
// Don't @ me bro! At least it's not GOTOs

// one bit time is 19 microseconds
float num_microseconds = 5*20;

// -=-=-=-=-=-=- Opulent Voice Global Variables =-=-=-=-=-=-=-
// OVP UDP Interface
volatile int ovp_transmission_active = 0;
volatile int ovp_running = 0;
pthread_mutex_t timeline_lock = PTHREAD_MUTEX_INITIALIZER;	// shared by all threads

// UDP connection used for encapsulated frame packets in both direction
struct sockaddr_in udp_client_addr;
socklen_t udp_client_len;
ssize_t udp_bytes_received;

// OVP transmit timeline manager
int64_t decision_time = 0;		// us timestamp after which a new frame is late
int ovp_txbufs_this_frame = 0;	// number of UDP frames seen before decision time (should be 1)
static pthread_t ovp_timeline_thread;
static pthread_cond_t timeline_start = PTHREAD_COND_INITIALIZER;
static pthread_mutex_t tls_lock = PTHREAD_MUTEX_INITIALIZER;

// OVP transmit session management variables
int hang_timer_active = 0;
int dummy_frames_sent = 0;	// Count of dummy frames sent in this hang time
int hang_timer_frames = 25;	// Number of dummy frames before ending session
static uint32_t session_ts_base = 0;	// timestamp at start of a session
static int64_t session_T0 = 0;	// Origin of time for this session (microseconds)


// OVP modulator frame buffer
// This buffer is used whenever we build a frame to send to the
// modulator. It's global to simplify the building of dummy frames
// and postamble frames, which reuse the sync word and frame header
// from the preceding Opulent Voice frames.
static uint8_t modulator_frame_buffer[OVP_MODULATOR_FRAME_SIZE];


// Opulent Voice Protocol functions
void start_transmission_session(void);
void end_transmission_session_normally(void);
void abort_transmission_session(void);
void stop_timeline_manager(void);
int enable_msk_transmission(void);
int disable_msk_transmission(void);
int process_and_send_ovp_frame_to_txbuf(uint8_t *frame_data, size_t frame_size);
void create_dummy_frame(void);
void create_postamble_frame(void);


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
	   struct iio_context *ctx   = NULL;
	   struct iio_channel *rx0_i = NULL;
static struct iio_channel *rx0_q = NULL;
static struct iio_channel *tx0_i = NULL;
static struct iio_channel *tx0_q = NULL;
static struct iio_buffer  *rxbuf = NULL;
static struct iio_buffer  *txbuf = NULL;

bool stop = false;

/* cleanup and exit */
void cleanup_and_exit(void)
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

	printf("Goodbye from Dialogus version %s\n", DIALOGUS_VERSION);

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
	enable_msk_transmission();
	push_txbuf_to_msk();

	ovp_transmission_active = 1;
	ovp_sessions_started++;

	// Inform the timeline manager to start managing the session
	pthread_cond_signal(&timeline_start);

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

	//!!! DOES THIS WORK with scrambling, etc? 
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

	// Check magic bytes (0xBBAADD) (dummy for authentication token check)
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
	save_header_station_id(frame_data);

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



// -=-=-=-=-=-=-=-=-=-= MAIN FUNCTION =-=-=-=-=-=-=-=-=-=-=-
/* Configuration based on ADI's example for simple configuration and streaming */

int main (void)
{

	printf("Hello from Dialogus version %s\n", DIALOGUS_VERSION);

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
	IIO_ENSURE((ctx = iio_create_default_context()) && "No context");
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

	if (init_register_access() != 0) {
		perror("Register access setup failed");
		cleanup_and_exit();
	}

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
	rx_sample_rate = tx_sample_rate / tx_rx_sample_ratio;	// Rx effective sample rate (not used)

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
