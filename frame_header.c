

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "frame_header.h"

static void decode_station_id(unsigned char *encoded, char *buffer);

// Store station ID (in two formats) from each incoming encapsulated frame
unsigned char active_station_id_binary[6] = {0,0,0,0,0,0};
char active_station_id_ascii[11] = "";	// 10 chars + null terminator is max possible

// Extract and store Station ID from frame header
void save_header_station_id(uint8_t *frame_data) {

	// Extract and store station ID for regulatory compliance
	memcpy(active_station_id_binary, frame_data, 6);	// Station ID length of 6 bytes
	decode_station_id(active_station_id_binary, active_station_id_ascii);
}

/* Decode a base-40 encoded callsign to its text representation
 *
 * encoded -- array of 6 bytes encoded as specified for Opulent Voice
 * buffer  -- array of 11 chars to receive the decoded station ID
 */
static void decode_station_id(unsigned char *encoded, char *buffer) { /* buffer[11] */

	static const char callsign_map[] = "xABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-/.";

	char result[11];

	uint64_t numeric_id = ((uint64_t)encoded[0] << 40) \
						+ ((uint64_t)encoded[1] << 32) \
						+ ((uint64_t)encoded[2] << 24) \
						+ ((uint64_t)encoded[3] << 16) \
						+ ((uint64_t)encoded[4] << 8 )  \
						+  (uint64_t)encoded[5];

	// decode each base-40 digit and map them to the appropriate character.
	size_t index = 0;
	while (numeric_id > 0)
	{
		result[index++] = callsign_map[numeric_id % 40];
		numeric_id /= 40;
	}
	result[index] = 0x00;

	strncpy(buffer, result, 11);

}
