#ifndef IIO_OPS_H
#define IIO_OPS_H

extern struct iio_buffer *txbuf;

extern void iio_teardown(void);
extern void iio_setup(void);
extern int load_ovp_frame_into_txbuf(uint8_t *frame_data, size_t frame_size);
extern int push_txbuf_to_msk(void);

#endif // IIO_OPS_H