#ifndef DEBUG_UTILS_H
#define DEBUG_UTILS_H

#include <iio.h>

void dump_bytes(char *name, uint8_t *buf, size_t length);
void dump_buffer(char *name, struct iio_buffer *buf);

#endif // DEBUG_UTILS_H