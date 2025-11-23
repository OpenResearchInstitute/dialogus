
#include <arpa/inet.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

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
    udp_encap_destination.sin_port = OVP_UDP_PORT;  // don't reply to originating port
}

// Receiver cal
void forward_encap_frame(uint8_t *frame_buffer, ssize_t length) {
    ssize_t bytes_sent;
    
    pthread_mutex_lock(&timeline_lock);

    bytes_sent = sendto(
            ovp_udp_socket,
            frame_buffer,
            length,
            0,  // no flags
            (const struct sockaddr *)&udp_encap_destination,
            udp_client_len
        );
    if (bytes_sent < 0) {
        printf("Error sending encapsulated frame\n");        
    } else if (bytes_sent != length) {
        perror("Sent only part of an encapsulated frame");
        cleanup_and_exit(1);
    }

    pthread_mutex_unlock(&timeline_lock);
}
