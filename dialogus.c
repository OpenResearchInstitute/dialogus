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
#include "iio_ops.h"
#include "interleaver.h"
#include "msk_setup.h"
#include "numerology.h"
#include "radio.h"
#include "randomizer.h"
#include "receiver.h"
#include "registers.h"
#include "statistics.h"
#include "timestamp.h"
#include "tx_timeline.h"
#include "udp_listener.h"
#include "udp_socket.h"


//-=-=-=-=-=-=-= GLOBAL VARIABLES -=-=-=-=-=-=-=-=
// Don't @ me bro! At least it's not GOTOs

// -=-=-=-=-=-=- Opulent Voice Global Variables =-=-=-=-=-=-=-
// OVP UDP Interface
volatile int ovp_transmission_active = 0;
pthread_mutex_t timeline_lock = PTHREAD_MUTEX_INITIALIZER;	// shared by all threads

// UDP connection used for encapsulated frame packets in both direction
struct sockaddr_in udp_client_addr;
socklen_t udp_client_len;

// OVP transmit timeline manager
pthread_cond_t timeline_start = PTHREAD_COND_INITIALIZER;
pthread_mutex_t tls_lock = PTHREAD_MUTEX_INITIALIZER;

// OVP transmit session management variables
bool hang_timer_active = false;
int dummy_frames_sent = 0;	// Count of dummy frames sent in this hang time
int hang_timer_frames = 25;	// Number of dummy frames before ending session
static uint32_t session_ts_base = 0;	// timestamp at start of a session
int64_t session_T0 = 0;	// Origin of time for this session (microseconds)


// OVP modulator frame buffer
// This buffer is used whenever we build a frame to send to the
// modulator. It's global to simplify the building of dummy frames
// and postamble frames, which reuse the sync word and frame header
// from the preceding Opulent Voice frames.
uint8_t modulator_frame_buffer[OVP_MODULATOR_FRAME_SIZE];


// Opulent Voice Protocol functions (forward declarations)
void start_transmission_session(void);
void end_transmission_session_normally(void);
void abort_transmission_session(void);

bool stop = false;

/* cleanup and exit */
void cleanup_and_exit(int retval)
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

	iio_teardown();

	printf("Goodbye from Dialogus version %s\n", DIALOGUS_VERSION);

	exit(retval);
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


// -=-=-=-= Opulent Voice Functions =-=-=-=-=-=-


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
	tx_timeline_set_decision_time(session_T0 + 60e3);	// 60ms to the middle of the next frame, T0-referenced

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
	hang_timer_active = false;
	dummy_frames_sent = 0;
	tx_timeline_set_decision_time(0);	// decision time no longer valid until next transmission starts

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

	hang_timer_active = false;
	dummy_frames_sent = 0;
	tx_timeline_set_decision_time(0);	// decision time no longer valid until next transmission starts

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
		tx_timeline_txbuf_filled();	// notify the timeline to detect underruns/overwrites.
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
		hang_timer_active = false;
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


// -=-=-=-=-=-=-=-=-=-= MAIN FUNCTION =-=-=-=-=-=-=-=-=-=-=-
/* Configuration based on ADI's example for simple configuration and streaming */

int main (void)
{

	printf("Hello from Dialogus version %s\n", DIALOGUS_VERSION);

	// Listen to ctrl+c and IIO_ENSURE
	signal(SIGINT, handle_sig);

	// Setup everything having to do with IIO for Opulent Voice operations
	iio_setup();

	// Memory-map several ranges of registers for direct access
	if (init_register_access() != 0) {
		perror("Register access setup failed");
		cleanup_and_exit(1);
	}

	// Setup the MSK modem in the PL for Opulent Voice operations
	msk_setup();

	if (start_debug_thread() < 0) {
		perror("Failed to start debug thread\n");
		cleanup_and_exit(1);
	}

	// Start OVP Timeline Manager
	if (start_timeline_manager() < 0) {
		perror("Failed to start OVP timeline manager\n");
		cleanup_and_exit(1);
	}

	// Start periodic statistics reporter
	if (start_periodic_statistics_reporter() < 0) {
		perror("Failed to start periodic statistics reporter\n");
		cleanup_and_exit(1);
	}

	// Start OVP UDP listener
	if (start_ovp_listener() < 0) {
		perror("Failed to start OVP listener\n");
		cleanup_and_exit(1);
	}

	// Start OVP Receiver
	if (start_ovp_receiver() < 0) {
		perror("Failed to start OVP receiver\n");
		cleanup_and_exit(1);
	}

	printf("OVP listener started on port %d\n", OVP_UDP_PORT);
	printf("Ready to receive frames from Interlocutor\n");
	printf("Press Ctrl+C to stop\n");

	// Main loop for OVP mode - keep program running
	while (!stop) {
		usleep(1e5);	// tenth-second sleep
	}

	cleanup_and_exit(0);

	return 0;
}
