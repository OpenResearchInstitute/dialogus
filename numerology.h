#ifndef NUMEROLOGY_H
#define NUMEROLOGY_H


// Sizes for encapsulated Opulent Voice frames
#define OVP_SINGLE_FRAME_SIZE 134	// Opulent Voice Protocol Packet Size

// Opulent Voice Protocol constants
#define OVP_MAGIC_BYTES 0xBBAADD
#define OVP_HEADER_SIZE 12
#define OVP_PAYLOAD_SIZE 122
#define OVP_UDP_PORT 57372
#define OVP_FRAME_PERIOD_MS 40	// Fixed 40ms timing

// Sizes for final over-the-air frames sent to MSK modulator
// Assuming that the modulator only adds the frame sync word and does no other processing
#define OVP_ENCODED_HEADER_SIZE (OVP_HEADER_SIZE * 2)	// Header expands by 2x due to FEC coding
#define OVP_ENCODED_PAYLOAD_SIZE (OVP_PAYLOAD_SIZE * 2)	// Payload expands by 2x due to FEC coding

#define OVP_MODULATOR_FRAME_SIZE (OVP_ENCODED_HEADER_SIZE + OVP_ENCODED_PAYLOAD_SIZE)
#define OVP_MODULATOR_PAYLOAD_OFFSET (OVP_ENCODED_HEADER_SIZE)

// Sizes for frames returned by MSK demodulator
// Assuming that the demod only strips off the frame sync word and does no other processing
#define OVP_DEMOD_FRAME_SIZE (OVP_ENCODED_HEADER_SIZE + OVP_ENCODED_PAYLOAD_SIZE)
#define OVP_DEMOD_PAYLOAD_OFFSET (OVP_ENCODED_HEADER_SIZE)

#endif // NUMEROLOGY_H