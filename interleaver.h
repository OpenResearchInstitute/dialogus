#ifndef INTERLEAVER_H
#define INTERLEAVER_H


#include <stdint.h>

void interleave_ovp_frame(uint8_t *buf);
void deinterleave_ovp_frame(uint8_t *buf);

#endif // INTERLEAVER_H