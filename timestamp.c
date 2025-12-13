#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "registers.h"
#include "timestamp.h"


/* Global Timer runs on the CPU clock, divided by 2 */
// #define COUNTS_PER_SECOND (666666687 / 2 / 2)   // ADALM Pluto
// #define COUNTS_PER_SECOND (750000000 / 2 / 2)   // LibreSDR baseclock

static double counts_per_second = 0.0;	// will crash if you don't set it!

// Timestamp facility, hardware locked to the sample clock
uint64_t get_timestamp(void) {
	uint32_t high, low;
	FILE *f;
	char text_value[20];

	// Determine the conversion factor between global timer counts and seconds,
	// once the first time this mechanism is used.
	// On these systems, the BogoMIPS rating is half the CPU clock,
	// and the conversion factor is half of that.
	if (counts_per_second == 0.0) {
		f = popen("grep -i BogoMIPS /proc/cpuinfo | grep -o [0-9.]*", "r");
		if (f == NULL) {
			printf("Could not get BogoMIPS from /proc/cpuinfo\n");
			counts_per_second = 187500000.0;	// default value, matches LibreSDR
		} else if (fgets(text_value, sizeof(text_value), f) == NULL) {	// first line only
			printf("BogoMIPS value didn't contain a number\n");
			counts_per_second = 187500000.0;
		} else {
			if (strcmp(text_value, "333.33\n") == 0) {	// ADALM PLUTO
				counts_per_second = 1.0e9 * 2.0 / 3.0;	// substitute exact value
			} else {	// LibreSDR clock rates are already exact values (750, 850, 900, 1000, 1100 MHz)
				counts_per_second = atof(text_value) * 1.0e6 / 2.0;
			}
			printf("Global timer counts_per_second appears to be %.2f\n", counts_per_second);
		}
		pclose(f);
		printf("Closed\n");
	}

	// Reading global timer counter register
	/* This is the method used in the library code for XTime_GetTime().
	   It handles the case where the first read of the two timer regs
	   spans a carry between the two timer words. */
	// But if it's early in initialization and we don't have the timer registers
	// mapped yet, we just return 0.
	if (timer_register_map == NULL) {
		return(0);
	} else {
		do {
			high = read_mapped_reg(timer_register_map, GLOBAL_TMR_UPPER_OFFSET);
			low = read_mapped_reg(timer_register_map, GLOBAL_TMR_LOWER_OFFSET);
		} while (read_mapped_reg(timer_register_map, GLOBAL_TMR_UPPER_OFFSET) != high);
		return((((uint64_t) high) << 32U) | (uint64_t) low);
	}
}

uint32_t get_timestamp_ms(void) {
	uint64_t ts = get_timestamp();
	double ts_seconds = ts / counts_per_second;
	double ts_ms = ts_seconds * 1000.0;
	return (uint32_t)ts_ms;
}

uint64_t get_timestamp_us(void) {
	uint64_t ts = get_timestamp();
	double ts_seconds = ts / counts_per_second;
	double ts_us = ts_seconds * 1000000.0;
	return (uint64_t)ts_us;
}

void print_timestamp(void) {
	printf("timestamp: %f\n", get_timestamp() / counts_per_second);
}

