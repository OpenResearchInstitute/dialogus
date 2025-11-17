
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "numerology.h"



// FEC-encode OVP header
void encode_ovp_header(uint8_t *input, uint8_t *output) {
	//!!! placeholder FEC
	memcpy(output, input, OVP_HEADER_SIZE);
	memcpy(output + OVP_HEADER_SIZE, input, OVP_HEADER_SIZE);	// Simple repetition placeholder
}

// FEC-decode OVP header
void decode_ovp_header(uint8_t *input, uint8_t *output) {
	//!!! placeholder FEC
	memcpy(output, input, OVP_HEADER_SIZE);	// matches placeholder encoder
}

// FEC-encode OVP payload
void encode_ovp_payload(uint8_t *input, uint8_t *output) {
	//!!! placeholder FEC
	memcpy(output, input, OVP_PAYLOAD_SIZE);
	memcpy(output + OVP_PAYLOAD_SIZE, input, OVP_PAYLOAD_SIZE);	// Simple repetition placeholder
}

// FEC-decode OVP payload
void decode_ovp_payload(uint8_t *input, uint8_t *output) {
	//!!! placeholder FEC
	memcpy(output, input, OVP_PAYLOAD_SIZE);	// matches placeholder encoder
}
