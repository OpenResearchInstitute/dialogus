#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include "statistics.h"

// OVP Statistics
uint64_t ovp_frames_received = 0;
uint64_t ovp_frames_processed = 0;
uint64_t ovp_frame_errors = 0;
uint64_t ovp_sessions_started = 0;
uint64_t ovp_sessions_ended = 0;
uint64_t ovp_dummy_frames_sent = 0;
uint64_t ovp_untimely_frames = 0;
extern volatile int ovp_transmission_active;
extern char *active_station_id_ascii;
extern int hang_timer_active;
extern int dummy_frames_sent;
extern int hang_timer_frames;


// OVP periodic reporting thread mechanisms
static pthread_t ovp_reporter_thread;
extern bool stop;
extern pthread_mutex_t timeline_lock;

void print_ovp_statistics(void) {
	printf("\n=== OVP Statistics ===\n");
	printf("Frames received:     %llu\n", (unsigned long long)ovp_frames_received);
	printf("Frames processed:    %llu\n", (unsigned long long)ovp_frames_processed);
	printf("Frame errors:        %llu\n", (unsigned long long)ovp_frame_errors);
	printf("Untimely frames:     %llu\n", (unsigned long long)ovp_untimely_frames);
	printf("Dummy frames sent:   %llu\n", (unsigned long long)ovp_dummy_frames_sent);
	printf("Sessions started:    %llu\n", (unsigned long long)ovp_sessions_started);
	printf("Sessions ended:      %llu\n", (unsigned long long)ovp_sessions_ended);
	printf("Active session:      %s\n", ovp_transmission_active ? "YES" : "NO");
	if (ovp_transmission_active) {
		printf("Active station ID:   %s\n", active_station_id_ascii);
	}
	printf("Hang timer active:   %s\n", hang_timer_active ? "YES" : "NO");
	if (hang_timer_active) {
		printf("Dummy frames sent:   %d/%d (for %s)\n", 
				dummy_frames_sent, hang_timer_frames, active_station_id_ascii);
	}
	if (ovp_frames_received > 0) {
		printf("Success rate:        %.1f%%\n", 
			(double)ovp_frames_processed / ovp_frames_received * 100.0);
	}
	printf("======================\n");
}

// OVP Periodic Statistics Reporter thread
// Wakes up at intervals of about 10 seconds and print a report.
void* ovp_periodic_statistics_reporter_thread(__attribute__((unused)) void *arg) {
	while (!stop) {
		pthread_mutex_lock(&timeline_lock);
		print_ovp_statistics();
		pthread_mutex_unlock(&timeline_lock);
		// don't sleep all at once; makes exiting the program too slow.
		for (int i=0; i < 100; i++) {
			usleep(100e3);	// 100 iterations of 100ms = 10 seconds
			if (stop) {
				break;
			}
		}
	}
	printf("OVP: Periodic reporter thread exiting\n");
	return NULL;
}

int start_periodic_statistics_reporter(void) {
	if (pthread_create(&ovp_reporter_thread, NULL, ovp_periodic_statistics_reporter_thread, NULL) != 0) {
		perror("OVP: Failed to create periodic statistics reporter thread");
		return -1;
	}
	
	printf("OVP: Reporter started successfully\n");
	return 0;
}

void stop_periodic_statistics_reporter(void) {
	if (ovp_reporter_thread) {
		pthread_cancel(ovp_reporter_thread);
	}
		
	printf("OVP: Reporter stopped\n");
}

