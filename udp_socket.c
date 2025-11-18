
#include <arpa/inet.h>
#include <sys/socket.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "numerology.h"
#include "udp_socket.h"


int ovp_udp_socket = -1;
static struct sockaddr_in ovp_listen_addr;

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
	int opt = 1;	// allow reuse of local addresses
	if (setsockopt(ovp_udp_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
		perror("OVP: Failed to set socket option");
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
