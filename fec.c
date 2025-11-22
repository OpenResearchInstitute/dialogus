
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "fec.h"
#include "numerology.h"

// FEC-encode entire OVP frame (one FEC to rule them all)
void encode_ovp_frame(uint8_t *input, uint8_t *output) {
	//!!! placeholder FEC. Does not match placeholder FEC in the HDL.
	memcpy(output, input, OVP_SINGLE_FRAME_SIZE);
	memcpy(output + OVP_SINGLE_FRAME_SIZE, input, OVP_SINGLE_FRAME_SIZE);	// simple repetition placeholder
}

void decode_ovp_frame(uint8_t *input, uint8_t *output) {
	//!!! placeholder FEC, matches the above encoder
	memcpy(output, input, OVP_SINGLE_FRAME_SIZE);
}

// The rest of these are unused, since we switched from separate FEC
// on the header and the payload to one overall FEC.

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
