
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include "interleaver.h"


// apply interleaving to break up block errors for the decoder
void interleave_ovp_frame(uint8_t *buf) {
	//!!! dummy implementation, write this.
	buf[0] = buf[0];
}

// remove interleaving
void deinterleave_ovp_frame(uint8_t *buf) {
	//!!! dummy implementation, write this.
	buf[0] = buf[0];
}

