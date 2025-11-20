#ifndef TX_TIMELINE_H
#define TX_TIMELINE_H


void tx_timeline_set_decision_time(int64_t new_decision_time);
void tx_timeline_txbuf_filled(void);
int start_timeline_manager(void);
void stop_timeline_manager(void);

#endif // TX_TIMELINE_H