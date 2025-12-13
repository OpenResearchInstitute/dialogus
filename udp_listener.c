
#include <arpa/inet.h>
#include <errno.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/socket.h>
#include <unistd.h>

#include "numerology.h"
#include "receiver.h"
#include "statistics.h"
#include "timestamp.h"
#include "udp_socket.h"

extern struct sockaddr_in udp_client_addr;
extern socklen_t udp_client_len;
extern pthread_mutex_t timeline_lock;	// shared by all threads
extern void accept_decapsulated_frame(uint8_t *frame_data);
extern int init_ovp_udp_listener(void);

static pthread_t ovp_udp_thread;

// True if the thread has started successfully and not stopped
static bool udp_listener_running;

// OVP Frame Buffer for transmit (sized for logical frames)
static uint8_t ovp_frame_buffer[OVP_SINGLE_FRAME_SIZE];

// UDP listener thread
void* ovp_udp_listener_thread(__attribute__((unused)) void *arg) {
	uint32_t recv_ts;
	uint32_t last_recv_ts = 0;
	ssize_t udp_bytes_received;	// not cumulative, updated on each recvfrom call


	while (udp_listener_running) {

		udp_client_len = sizeof(udp_client_addr);	// length passed in to recvfrom

		udp_bytes_received = recvfrom(
			ovp_udp_socket,
			ovp_frame_buffer,
			sizeof(ovp_frame_buffer),
			0, // blocking receive - will wait for frames
			(struct sockaddr*)&udp_client_addr,
			&udp_client_len
		);

		// notify the receive process that we know the client address
		receiver_ok_to_forward_frames(true);

		// might be shutting down now, don't grab the mutex
		if (!udp_listener_running) {
			break;
		}
		printf("MUTEX timeline_lock: locking in listener at %d\n", get_timestamp_ms());
		pthread_mutex_lock(&timeline_lock);
		printf("MUTEX timeline_lock: acquired in listener at %d\n", get_timestamp_ms());

		if (udp_bytes_received == OVP_SINGLE_FRAME_SIZE) {
			recv_ts = get_timestamp_ms();
			printf("OVP: Received %zd encapsulated bytes from %s:%d after %dms ending with ", udp_bytes_received,
					inet_ntoa(udp_client_addr.sin_addr),
					ntohs(udp_client_addr.sin_port),
					recv_ts - last_recv_ts);
			last_recv_ts = recv_ts;
			for (int i=0 /*!!! OVP_SINGLE_FRAME_SIZE - 9*/; i<OVP_SINGLE_FRAME_SIZE; i++) {
				printf("%02x ", ovp_frame_buffer[i]);
			}
			printf("\n");

			// Process the frame
			accept_decapsulated_frame(ovp_frame_buffer);
		} else if (udp_bytes_received >= 0) {
			printf("OVP: Warning - received unexpected frame size %zd bytes (expected %d)\n", 
					udp_bytes_received, OVP_SINGLE_FRAME_SIZE);
			ovp_frame_errors++;
		} else if (udp_bytes_received < 0) {
			if (errno != EAGAIN && errno != EWOULDBLOCK && udp_listener_running) {
				perror("OVP: UDP receive error");	// don't exit on receive errors
			}
		}

		printf("MUTEX RELEASE in listener at %d\n", get_timestamp_ms());
		pthread_mutex_unlock(&timeline_lock);
		printf("MUTEX RELEASED in listener at %d\n", get_timestamp_ms());
	}

	printf("OVP: UDP listener thread exiting\n");
	return NULL;
}

// Start OVP UDP listener
int start_ovp_listener(void) {
	if (init_ovp_udp_listener() < 0) {
		return -1;
	}

	udp_listener_running = true;

	if (pthread_create(&ovp_udp_thread, NULL, ovp_udp_listener_thread, NULL) != 0) {
		perror("OVP: Failed to create UDP thread");
		close(ovp_udp_socket);
		ovp_udp_socket = -1;
		udp_listener_running = false;
		return -1;
	}

	printf("OVP: Listener started successfully\n");
	return 0;
}

// Stop OVP UDP listener
void stop_ovp_listener(void) {
	if (udp_listener_running) {
		udp_listener_running = false;

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

