


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
    DEBUG_ALL = 0,
    DEBUG_NONE = 255,
    DEBUG_THREADS = 1,
    DEBUG_TIMELINE,
    DEBUG_FIFO,
    DEBUG_MUTEX,
    DEBUG_MSK,
    DEBUG_IIO,
    DEBUG_SESSION,
    DEBUG_FRAMES,
    DEBUG_ENCAP,
    DEBUG_RX,
    DEBUG_REGS,
} debug_topics;


extern void debug_printf(debug_level level, debug_topics topic, const char *format, ...);
extern void debug_printf_summary(void);