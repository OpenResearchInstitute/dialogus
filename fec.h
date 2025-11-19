#ifndef FEC_H
#define FEC_H


#include <stdint.h>

extern void encode_ovp_header(uint8_t *input, uint8_t *output);
extern void decode_ovp_header(uint8_t *input, uint8_t *output);
extern void encode_ovp_payload(uint8_t *input, uint8_t *output);
extern void decode_ovp_payload(uint8_t *input, uint8_t *output);

#endif // FEC_H