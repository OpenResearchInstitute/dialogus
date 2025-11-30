
#include <arpa/inet.h>
#include <errno.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include "numerology.h"
#include "receiver.h"
#include "statistics.h"
#include "timestamp.h"
#include "tcp_socket.h"

extern pthread_mutex_t timeline_lock;	// shared by all threads
extern void accept_decapsulated_frame(uint8_t *frame_data);

static pthread_t ovp_tcp_thread;

// True if the thread has started successfully and not stopped
static bool tcp_listener_running;

// frame Buffer for transmit (sized for one logical frame plus COBS overhead)
static uint8_t frame_buffer[OVP_SINGLE_FRAME_SIZE + 1];

// TCP listener thread
void* ovp_tcp_listener_thread(__attribute__((unused)) void *arg) {
	uint32_t recv_ts;
	uint32_t last_recv_ts = 0;
    int encap_socket;   // file descriptor for a connected TCP socket
    struct sockaddr_in encap_addr;
    socklen_t encap_addr_len = sizeof(encap_addr);

	while (tcp_listener_running) {

        // Wait for a client to connect and create a socket for it
        encap_socket = accept(ovp_tcp_socket, (struct sockaddr *)&encap_addr, &encap_addr_len);
        if (encap_socket <  0) {
            printf("Failed in accept call for TCP encapsulation (%d)\n", errno);
            continue;
        }

        printf("TCP encapsulated frame client connected %s:%d\n", inet_ntoa(encap_addr.sin_addr), ntohs(encap_addr.sin_port));

   		// notify the receive process that we are connected to a client
		receiver_ok_to_forward_frames(true);

        while (encap_socket >= 0) {
            ssize_t bytes_received = recv(encap_socket, frame_buffer, sizeof(frame_buffer), MSG_WAITALL);
            if (bytes_received < 0) {
                printf("Error receiving TCP-encapsulated frame (%d)\n", bytes_received);
            } else if (bytes_received == 0) {
                printf("TCP encapsulated frame client disconnected\n");
                close(encap_socket);
                encap_socket = -1;
            } else if (bytes_received != sizeof(frame_buffer)) {
                printf("TCP encapsulated frame too short\n");
            } else {
                // might be shutting down now, don't grab the mutex
                if (!tcp_listener_running) {
                    break;
                }

                if (frame_buffer[OVP_SINGLE_FRAME_SIZE] != 0) {
                    printf("TCP-encapsulated frame not synchronized\n");
                    //!!! try to re-sync with COBS here
                    continue;
                }

		        pthread_mutex_lock(&timeline_lock);
                    recv_ts = get_timestamp_ms();
			        printf("OVP: Received %zd encapsulated bytes from %s:%d after %dms ending with ", bytes_received,
					inet_ntoa(encap_addr.sin_addr),
					ntohs(encap_addr.sin_port),
					recv_ts - last_recv_ts);
			        last_recv_ts = recv_ts;
                    for (int i=0 /*!!! OVP_SINGLE_FRAME_SIZE - 9*/; i<OVP_SINGLE_FRAME_SIZE; i++) {
                        printf("%02x ", frame_buffer[i]);
                    }
                    printf("\n");

                    // Process the frame
                    accept_decapsulated_frame(frame_buffer);

		        pthread_mutex_unlock(&timeline_lock);
            }
        }
    }
	printf("OVP: TCP listener thread exiting\n");
	return NULL;
}


// Initialize OVP TCP listener
int init_ovp_tcp_listener(void) {

	if (init_tcp_socket()) {
		printf("Error initializing TCP socket\n");
		return -1;
	} else {
		printf("OVP: UDP listener initialized on port %d\n", OVP_UDP_PORT);
	}

	return 0;
}


// Start OVP TCP listener
int start_ovp_tcp_listener(void) {
	if (init_ovp_tcp_listener() < 0) {
		return -1;
	}

	tcp_listener_running = true;

	if (pthread_create(&ovp_tcp_thread, NULL, ovp_tcp_listener_thread, NULL) != 0) {
		perror("OVP: Failed to create TCP listener thread");
		close(ovp_tcp_socket);
		ovp_tcp_socket = -1;
		tcp_listener_running = false;
		return -1;
	}

	printf("OVP: TCP Listener started successfully\n");
	return 0;
}

// Stop OVP TCP listener
void stop_ovp_tcp_listener(void) {
	if (tcp_listener_running) {
		tcp_listener_running = false;

		// Close socket to unblock recv in thread
		if (ovp_tcp_socket >= 0) {
			close(ovp_tcp_socket);
			ovp_tcp_socket = -1;
		}

		if (ovp_tcp_thread) {
			pthread_cancel(ovp_tcp_thread);
		}

		printf("OVP: TCP listener stopped\n");
	}
}

