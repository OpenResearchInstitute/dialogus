

#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include "debug_utils.h"

extern struct iio_channel *rx0_i;

void dump_bytes(char *name, uint8_t *buf, size_t length) {
	printf("%s: ", name);

	for (size_t i=0; i < length; i++) {
		printf("%02x ", buf[i]);
	}

	printf("\n");
}

void dump_buffer(char *name, struct iio_buffer *buf) {
	printf("%s: ", name);

	printf("@%p-> ", buf);
	ptrdiff_t p_inc = iio_buffer_step(buf);
	char *p_end = iio_buffer_end(buf);
	char *first = (char *)iio_buffer_first(buf, rx0_i);
	for (char *p_dat = first; p_dat < p_end; p_dat += p_inc) {
		printf("%02x ", ((int16_t*)p_dat)[0] & 0x00ff);
	}

	printf("\n");
}