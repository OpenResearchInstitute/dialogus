


typedef enum {
    LEVEL_MUTE,
    LEVEL_QUIET,
    LEVEL_URGENT,
    LEVEL_MEDIUM,
    LEVEL_LOW,
    LEVEL_INFO,
    LEVEL_BORING,
} debug_level;

typedef enum {
    DEBUG_ALL,
    DEBUG_NONE,
    DEBUG_THREADS,
    DEBUG_TIMELINE,
    DEBUG_FIFO,
    DEBUG_MUTEX,
    DEBUG_MSK,
    DEBUG_IIO,
    DEBUG_SESSION,
    DEBUG_FRAMES,
} debug_topics;


extern void debug_printf(debug_level level, debug_topics topic, const char *format, ...);
extern void debug_printf_summary(void);