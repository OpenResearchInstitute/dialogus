#ifndef FRAME_HEADER_H
#define FRAME_HEADER_H

#include <stdint.h>

extern unsigned char active_station_id_binary[];
extern char active_station_id_ascii[];

extern void save_header_station_id(uint8_t *frame_data);

#endif // FRAME_HEADER_H