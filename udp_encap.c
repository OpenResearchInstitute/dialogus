
#include <arpa/inet.h>
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include "debug_printf.h"
#include "numerology.h"
#include "receiver.h"
#include "timestamp.h"
#include "udp_socket.h"

extern socklen_t udp_client_len;
extern pthread_mutex_t timeline_lock;	// shared by all threads
extern void cleanup_and_exit(int retval);

// Address for sending encap frames back over the network
struct sockaddr_in udp_encap_destination;

// UDP listener calls this when it knows the address that's sending us encapsulated frames
void register_encap_address(struct sockaddr_in *address) {
    udp_encap_destination = *address;
    udp_encap_destination.sin_port = htons(OVP_UDP_PORT);  // don't reply to originating port
    printf("Registered encap destination %s:%d\n",
                inet_ntoa(udp_encap_destination.sin_addr),
				ntohs(udp_encap_destination.sin_port));
}

// Receiver cal
void forward_encap_frame(uint8_t *frame_buffer, ssize_t length) {
    ssize_t bytes_sent;
    
    debug_printf(LEVEL_INFO, DEBUG_MUTEX, "MUTEX timeline_lock: locking in udp_encap\n");
    pthread_mutex_lock(&timeline_lock);
    debug_printf(LEVEL_INFO, DEBUG_MUTEX, "MUTEX timeline_lock: acquired in udp_encap\n");

    bytes_sent = sendto(
            ovp_udp_socket,
            frame_buffer,
            length,
            0,  // no flags
            (const struct sockaddr *)&udp_encap_destination,
            udp_client_len
        );
    if (bytes_sent < 0) {
        debug_printf(LEVEL_MEDIUM, DEBUG_ENCAP, "Error sending encapsulated frame = %d\n", errno);        
    } else if (bytes_sent != length) {
        debug_printf(LEVEL_URGENT, DEBUG_ENCAP, "Sent only part of an encapsulated frame");
        cleanup_and_exit(1);
    }

    debug_printf(LEVEL_INFO, DEBUG_MUTEX, "MUTEX RELEASE in udp_encap\n");
    pthread_mutex_unlock(&timeline_lock);
    debug_printf(LEVEL_INFO, DEBUG_MUTEX, "MUTEX RELEASED in udp_encap\n");
}
