#ifndef IIO_OPS_H
#define IIO_OPS_H

/* common RX and TX streaming params */
struct stream_cfg {
	long long bw_hz;	// Analog bandwidth in Hz
	long long fs_hz;	// Baseband sample rate in Hz
	long long lo_hz;	// Local oscillator frequency in Hz
	const char* rf_port;	// Port name
};


extern struct iio_buffer *txbuf;

extern void iio_teardown(void);
extern void iio_setup(struct stream_cfg rxcfg, struct stream_cfg txcfg);
extern int load_ovp_frame_into_txbuf(uint8_t *frame_data, size_t frame_size);
extern int push_txbuf_to_msk(void);

#endif // IIO_OPS_H