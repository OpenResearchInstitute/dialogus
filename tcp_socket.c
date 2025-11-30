
#include <arpa/inet.h>
#include <errno.h>
#include <sys/socket.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "numerology.h"
#include "udp_socket.h"


int ovp_tcp_socket = -1;
static struct sockaddr_in listen_addr;

// Initialize network port for TCP encapsulated frames (both directions)
// We only support a single TCP connection at a time, but if the connection
// is closed, it may be reopened by the same host or a different host.
int init_tcp_socket(void) {
	// create the socket
	ovp_tcp_socket = socket(AF_INET, SOCK_STREAM, 0);
	if (ovp_tcp_socket < 0) {
		printf("OVP: Failed to create TCP socket (%d)\n", errno);
		return -1;
	}

	// Set socket options
	int opt = 1;	// allow reuse of local addresses
	if (setsockopt(ovp_tcp_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
		printf("OVP: Failed to set socket option SO_REUSEADDR (%d)\n", errno);
		close(ovp_tcp_socket);
        ovp_tcp_socket = -1;
		return -1;
	}
	if (setsockopt(ovp_tcp_socket, SOL_SOCKET, SO_KEEPALIVE, &opt, sizeof(opt)) < 0) {
		printf("OVP: Failed to set socket option SO_KEEPALIVE (%d)\n", errno);
		close(ovp_tcp_socket);
        ovp_tcp_socket = -1;
		return -1;
	}

    // Configure listen address
	memset(&listen_addr, 0, sizeof(listen_addr));
	listen_addr.sin_family = AF_INET;
	listen_addr.sin_addr.s_addr = INADDR_ANY;
	listen_addr.sin_port = htons(OVP_TCP_PORT);

	// Bind socket
	if (bind(ovp_tcp_socket, (struct sockaddr*)&listen_addr, sizeof(listen_addr)) < 0) {
		printf("OVP: Failed to bind TCP socket (%d)\n", errno);
		close(ovp_tcp_socket);
		ovp_tcp_socket = -1;
		return -1;
	}

	// Set the socket to listen for incoming connections
 	if (listen(ovp_tcp_socket, 1)) {
		printf("OVP: Failed to listen to TCP socket (%d)\n", errno);
		close(ovp_tcp_socket);
		ovp_tcp_socket = -1;
		return -1;
	}

	return 0;
}
