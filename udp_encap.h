#ifndef UDP_ENCAP_H
#define UDP_ENCAP_H

#include <arpa/inet.h>
#include <stdint.h>
#include <sys/socket.h>
#include <sys/types.h>

void register_encap_address(struct sockaddr_in *address);
void forward_encap_frame(uint8_t *frame_buffer, ssize_t length);

#endif // UDP_ENCAP_H