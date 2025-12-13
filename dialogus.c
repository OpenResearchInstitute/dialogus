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
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

#include "config.h"
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

// We support two different functional partitions for Opulent Voice radios.
// The difference is in how much of the actual signal processing is done
// in this software (running on the ARM core in the Zynq PS) vs how much
// is done in the modem implemented in the FPGA hardware (the Zynq PL).
//
// If software_processing is true, this program converts between 134-byte
// logical frames and 268-byte physical frames, exchanging the physical
// frames with the modem.
//
// If software_processing is false, this program deals exclusively in
// 134-byte logical frames, and any conversion between frame types
// (such as randomization, forward error correction, and interleaving)
// is handled by the modem hardware.
//
// We choose to implement this as a runtime switch instead of a #define
// compile-time option, so that it can readily be controlled by a
// user-supplied configuration file. However, we do not support changing
// the setting on the fly. It must be set before the IIO buffers are
// created, and never changed without restarting the program.
//
// For flexibility, we will make this switchable separately for the
// transmitter and the receiver.
//
bool software_tx_processing = false;
bool software_rx_processing = false;

// OVP logical transmit frame buffer
// This buffer is used whenever we build a frame for transmission.
// It's a single global buffer to simplify the building of dummy
// frames and postamble frames, which reuse the frame header from
// immediately preceding frames.
uint8_t logical_frame_buffer[OVP_SINGLE_FRAME_SIZE];

// OVP modulator frame buffer
// This buffer is used whenever we convert a logical frame into the
// physical format before sending it to the modulator. This is only
// used when software_tx_processing is enabled.
//
// It's global for symmetry with logical_frame_buffer.
//
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

	printf("OVP: Enabling transmit chain\n");
	WRITE_MSK(MSK_Init, 0x00000000);  // Deassert txinit    
	usleep(100);  // might not be needed?

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

	printf("OVP: Disabling transmit chain\n");
	WRITE_MSK(MSK_Init, 0x00000002);  // Assert txinit (bit 1)    

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
		perror("Transmission session started while already active");
		exit(1);
	}

	printf("OVP: Starting transmission session for station %s\n", active_station_id_ascii);
	session_ts_base = get_timestamp_ms();	// for debug prints

	session_T0 = get_timestamp_us();	// used for timeline management
	if (software_tx_processing) {
		tx_timeline_set_decision_time(session_T0 + 40e3 + 20e3);	// preamble plus 20ms to the middle of the next frame, T0-referenced
	} else {
		tx_timeline_set_decision_time(session_T0 + 20e3);	// 20ms to the middle of the next frame, T0-referenced
	}
	
	// Time to send a preamble, here before sending the first frame of
	// this new transmission session.
	//
	// If we are not doing software_tx_processing, we have no way to send
	// a preamble to the transmitter, only logical frames. In that case,
	// We have to assume that the modulator is set up to automatically send
	// a preamble before the first frame of a transmission session.
	if (software_tx_processing) {
		// Send preamble: pure 1100 bit pattern for 40ms (no OVP header)
		uint8_t preamble_frame[OVP_MODULATOR_FRAME_SIZE];	// Full 40ms of data

		// Fill with 1100 repeating pattern (0xCC = 11001100 binary)
		for (unsigned int i = 0; i < sizeof(preamble_frame); i++) {
			preamble_frame[i] = 0xCC;	// 1100 1100 repeating
		}

		// Send preamble to MSK modulator (pure bit pattern, no header, no processing)
		printf("OVP: Sending 40ms preamble (1100 pattern, %zu bytes)\n", sizeof(preamble_frame));
		load_ovp_frame_into_txbuf(preamble_frame, sizeof(preamble_frame));
		enable_msk_transmission();
		push_txbuf_to_msk();	// special handling for preamble "frame" - bypasses processing
	} else {
		enable_msk_transmission();	// this should trigger automatic preamble generation
	}

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

// Modify the frame leftover in logical_frame_buffer to transform it
// into a dummy frame, by replacing its payload with the encoded dummy payload.
void create_dummy_logical_frame(void) {
	// The logical payload of a dummy frame is all zeroes.
	//
	// The COBS decoder on the receiving end will take this as termination
	// of any packet previously in progress (complete or not), followed by
	// a bunch of zero-length packets (which are dropped).
	//
	// A series of dummy frames always follows at least one frame containing
	// a real payload, which was handled in logical_frame_buffer. We can
	// create an appropriate logical dummy frame by just overwriting the
	// payload portion of logical_frame_buffer with zeroes.
	//
	// Note that this doesn't update the authentication tag in the frame.
	// We are not able to do that in any case, since authentication is
	// handled by Interlocutor, not this program. The air interface spec
	// permits this behavior.
	memset(logical_frame_buffer + OVP_HEADER_SIZE, 0x00, OVP_PAYLOAD_SIZE);
}

// The payload of a postamble frame is filled with copies of the
// 11-bit Barker code. Precompute this using this little Python snippet:
/*
barker11 = "11100010010"
barkers = barker11 * 89
postamble = [int(barkers[i:i+8],2) for i in range(0,122*8,8)]
*/
// We replace the last byte with zero because that acts as a COBS
// frame delimiter, preventing the preambles from stacking up in the
// COBS decoder and interfering with the first frame of the next
// transmission.
static uint8_t postamble_payload[OVP_PAYLOAD_SIZE] = {
	226, 92, 75, 137, 113, 46, 37, 196, 184, 151, 18,	// e2 5c 4b 89 71 2e 25 c4 b8 97 12
	226, 92, 75, 137, 113, 46, 37, 196, 184, 151, 18,
	226, 92, 75, 137, 113, 46, 37, 196, 184, 151, 18,
	226, 92, 75, 137, 113, 46, 37, 196, 184, 151, 18,
	226, 92, 75, 137, 113, 46, 37, 196, 184, 151, 18,
	226, 92, 75, 137, 113, 46, 37, 196, 184, 151, 18,
	226, 92, 75, 137, 113, 46, 37, 196, 184, 151, 18,
	226, 92, 75, 137, 113, 46, 37, 196, 184, 151, 18,
	226, 92, 75, 137, 113, 46, 37, 196, 184, 151, 18,
	226, 92, 75, 137, 113, 46, 37, 196, 184, 151, 18,
	226, 92, 75, 137, 113, 46, 37, 196, 184, 151, 18,
	0
};


// Modify the frame leftover in logical_frame_buffer to transform it
// into a postamble frame, by replacing its payload with the distinctive
// postamble payload.
void create_postamble_logical_frame(void) {
	// A postamble frame always follows at least one frame containing
	// a real payload, and possibly a sequence of dummy frames, all of
	// which were handled in logical_frame_buffer. We can create an
	// appropriate logical postamble frame by just overwriting the
	// payload portion of logical_frame_buffer with the distinctive
	// postamble sequence.
	//
	// Note that this doesn't update the authentication tag in the frame.
	// We are not able to do that in any case, since authentication is
	// handled by Interlocutor, not this program. The air interface spec
	// permits this behavior.
	memcpy(logical_frame_buffer + OVP_HEADER_SIZE, postamble_payload, OVP_PAYLOAD_SIZE);
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
int validate_ovp_frame(uint8_t *frame_data) {


	// Check magic bytes (0xBBAADD) (!!! dummy for authentication token check)
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
void process_ovp_frame_in_software(uint8_t *ovp_frame) {
	uint8_t randomized_frame[OVP_SINGLE_FRAME_SIZE];

	// apply randomization to reduce spectral features of the data
	memcpy(randomized_frame, ovp_frame, OVP_SINGLE_FRAME_SIZE);
	randomize_ovp_frame(randomized_frame);

	// One FEC over the whole frame
	encode_ovp_frame(randomized_frame, modulator_frame_buffer);

	// apply interleaving to break up block errors at the decoder(s)
	interleave_ovp_frame(modulator_frame_buffer);

	// The modulator_frame_buffer now contains the full modulator frame
	// ready for transmission via MSK modulator
	ovp_frames_processed++;
}


// Accept a complete OVP frame from the encapsulated stream
// (keeping a copy of the station ID)
void accept_decapsulated_frame(uint8_t *frame_data) {
	ovp_frames_received++;

	// Validate frame
	if (validate_ovp_frame(frame_data) < 0) {
		ovp_frame_errors++;
		return;
	}

	// Extract and store station ID for regulatory compliance, etc.
	save_header_station_id(frame_data);

	// Real frame received - cancel hang timer if running
	if (hang_timer_active) {
		printf("OVP: canceling hang timer\n");
		hang_timer_active = false;
		dummy_frames_sent = 0;
	}

	// Copy the decapsulated frame into processing buffer
	memcpy(logical_frame_buffer, frame_data, OVP_SINGLE_FRAME_SIZE);

	// Start transmission session if not active
	if (!ovp_transmission_active) {
		start_transmission_session();
	}

	printf("OVP: Accepted real frame from %s\n", active_station_id_ascii);

	tx_timeline_frame_ready();	// notify the timeline to detect underruns/overwrites.

	// additional processing will take place at decision time
}


// -=-=-=-=-=-=-=-=-=-= MAIN FUNCTION =-=-=-=-=-=-=-=-=-=-=-
/* Configuration based on ADI's example for simple configuration and streaming */

int main (void)
{

	printf("Hello from Dialogus version %s\n", DIALOGUS_VERSION);

	configure_dialogus();

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
