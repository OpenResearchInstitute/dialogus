
// OVP Statistics
extern uint64_t ovp_frames_received;
extern uint64_t ovp_frames_processed;
extern uint64_t ovp_frame_errors;
extern uint64_t ovp_sessions_started;
extern uint64_t ovp_sessions_ended;
extern uint64_t ovp_dummy_frames_sent;
extern uint64_t ovp_untimely_frames;


extern void print_ovp_statistics(void);
extern int start_periodic_statistics_reporter(void);
extern void stop_periodic_statistics_reporter(void);