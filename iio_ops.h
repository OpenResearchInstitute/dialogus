#ifndef IIO_OPS_H
#define IIO_OPS_H



void iio_teardown(void);
void iio_setup(void);
int load_ovp_frame_into_txbuf(uint8_t *frame_data, size_t frame_size);
int push_txbuf_to_msk(void);

#endif // IIO_OPS_H