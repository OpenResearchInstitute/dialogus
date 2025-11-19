#ifndef TIMESTAMP_H
#define TIMESTAMP_H


#include <stdint.h>

extern uint64_t get_timestamp(void);
extern uint32_t get_timestamp_ms(void);
extern uint64_t get_timestamp_us(void);
extern void print_timestamp(void);

#endif // TIMESTAMP_H