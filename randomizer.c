


#include "numerology.h"
#include "randomizer.h"


static uint8_t randomization_sequence[OVP_SINGLE_FRAME_SIZE] = {
	163, 129,  92, 196, 201,   8,  14,  83, 204, 161,
	251,  41, 158,  79,  22, 224, 151,  78,  43,  87,
	 18, 167,  63, 194,  77, 107,  15,   8,  48,  70,
	 17,  86,  13,  26,  19, 231,  80, 151,  97, 243,
	190, 227, 153, 176, 100,  57,  34,  44, 240,   9,

	225, 134, 207, 115,  89, 194,  92, 142, 227, 215,
	 63, 112, 212,  39, 194, 224, 129, 146, 218, 252,
	202,  90, 128,  66, 131,  21,  15, 162, 158,  21,
	156, 139, 219, 164,  70,  28,  16, 159, 179,  71,
	108,  94,  21,  18,  31, 173,  56,  61,   3, 186,

	144, 141, 190, 211, 101,  35,  50, 184, 171,  16,
	 98, 126, 198,  38, 124,  19, 201, 101,  61,  21,
	 21, 237,  53, 244,  87, 245,  88,  17, 157, 142,
	232,  52, 201,  89
};


// apply or remove randomization, in place, to reduce spectral features of the data
void randomize_ovp_frame(uint8_t *buf) {
	for (int i=0; i < OVP_SINGLE_FRAME_SIZE; i++) {
		buf[i] ^= randomization_sequence[i];
	}
}
