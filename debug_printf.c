

#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>

#include "debug_printf.h"
#include "timestamp.h"

static int filtered_out_count = 0;

static bool should_print(debug_level level, debug_topics topic) {

    switch (topic) {
        case DEBUG_ALL:
                return true;

        case DEBUG_NONE:
                return false;

        case DEBUG_THREADS:
                return level <= LEVEL_LOW;
        
        case DEBUG_TIMELINE:
                return level <= LEVEL_LOW;

        case DEBUG_FIFO:
                return level <= LEVEL_LOW;

        case DEBUG_MUTEX:
                return level <= LEVEL_MEDIUM;

        case DEBUG_MSK:
                return level <= LEVEL_INFO;

        case DEBUG_IIO:
                return level <= LEVEL_INFO;

        case DEBUG_SESSION:
                return level <= LEVEL_INFO;

        case DEBUG_FRAMES:
                return level <= LEVEL_INFO;
        
        default:
                printf("Unhandled topic %d in debug_printf.\n", topic);
                return true;
    }
}

void debug_printf(debug_level level, debug_topics topic, const char *format, ...) {
    char extended_format[1000];

    if (should_print(level, topic)) {
        snprintf(extended_format, sizeof(extended_format), "DBG @ %lld: %s", get_timestamp_us(), format);
        va_list args;
        va_start(args, format);
        vprintf(extended_format, args);
        va_end(args);
    } else {
        filtered_out_count++;
    }
}

void debug_printf_summary(void) {
    if (filtered_out_count > 0) {
        printf("%d messages were filtered out.", filtered_out_count);
    } else {
        printf("No messages were filtered out.\n");
    }
}