
#include <errno.h>
#include <iio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include "fec.h"
#include "interleaver.h"
#include "numerology.h"
#include "radio.h"
#include "randomizer.h"
#include "receiver.h"
#include "registers.h"
#include "timestamp.h"

extern void cleanup_and_exit(void);
extern int init_udp_socket(void);
extern struct iio_channel *rx0_i;
extern struct iio_context *ctx;
extern bool stop;

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

pthread_t ovp_rx_thread;
struct iio_buffer *rx_buf;
static bool encapsulated_frame_host_known = false;

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

void receiver_ok_to_forward_frames(bool ok) {
    encapsulated_frame_host_known = ok;
}
