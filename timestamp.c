#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include "registers.h"
#include "timestamp.h"


// Timestamp facility, hardware locked to the sample clock
uint64_t get_timestamp(void) {
	uint32_t high, low;

	// Reading global timer counter register
	/* This is the method used in the library code for XTime_GetTime().
	   It handles the case where the first read of the two timer regs
	   spans a carry between the two timer words. */
	do {
		high = read_mapped_reg(timer_register_map, GLOBAL_TMR_UPPER_OFFSET);
		low = read_mapped_reg(timer_register_map, GLOBAL_TMR_LOWER_OFFSET);
		// printf("%08x %08x\n", high, low);
	} while (read_mapped_reg(timer_register_map, GLOBAL_TMR_UPPER_OFFSET) != high);
	return((((uint64_t) high) << 32U) | (uint64_t) low);
}

uint32_t get_timestamp_ms(void) {
	uint64_t ts = get_timestamp();
	double ts_seconds = ts / (double)COUNTS_PER_SECOND;
	double ts_ms = ts_seconds * 1000.0;
	return (uint32_t)ts_ms;
}

uint64_t get_timestamp_us(void) {
	uint64_t ts = get_timestamp();
	double ts_seconds = ts / (double)COUNTS_PER_SECOND;
	double ts_us = ts_seconds * 1000000.0;
	return (uint64_t)ts_us;
}

void print_timestamp(void) {
	printf("timestamp: %f\n", get_timestamp() / (double)COUNTS_PER_SECOND);
}

