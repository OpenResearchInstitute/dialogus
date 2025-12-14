

#include <errno.h>
#include <iio.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "debug_printf.h"
#include "iio_ops.h"
#include "numerology.h"
#include "radio.h"
#include "registers.h"
#include "timestamp.h"

extern bool software_tx_processing;
extern void cleanup_and_exit(int retval);


#define IIO_ENSURE(expr) { \
	if (!(expr)) { \
		(void) fprintf(stderr, "assertion failed (%s:%d)\n", __FILE__, __LINE__); \
		(void) abort(); \
	} \
}

/* common RX and TX streaming params */
struct stream_cfg {
	long long bw_hz;	// Analog bandwidth in Hz
	long long fs_hz;	// Baseband sample rate in Hz
	long long lo_hz;	// Local oscillator frequency in Hz
	const char* rf_port;	// Port name
};

/* scratch mem for strings */
// Strings in this buffer are transient. Caller must not expect
// the string to persist after it is used.
static char tmpstr[64];

/* IIO structs required for streaming */
	   struct iio_context *ctx   = NULL;
	   struct iio_channel *rx0_i = NULL;
static struct iio_channel *rx0_q = NULL;
static struct iio_channel *tx0_i = NULL;
static struct iio_channel *tx0_q = NULL;
static struct iio_buffer  *rxbuf = NULL;
	   struct iio_buffer  *txbuf = NULL;


/* check return value of attr_write function */
static void errchk(int v, const char* what) {
	 if (v < 0) {
		fprintf(stderr, "Error %d writing to channel \"%s\"\nvalue may not be supported.\n", v, what);
		cleanup_and_exit(1);
	}
}

/* write attribute: long long int */
static void wr_ch_lli(struct iio_channel *chn, const char* what, long long val)
{
	errchk(iio_channel_attr_write_longlong(chn, what, val), what);
}

/* write attribute: string */
static void wr_ch_str(struct iio_channel *chn, const char* what, const char* str)
{
	errchk(iio_channel_attr_write(chn, what, str), what);
}

/* helper function generating channel names */
// The channel name must not be retained by the caller.
static char* get_ch_name(const char* type, int id)
{
	snprintf(tmpstr, sizeof(tmpstr), "%s%d", type, id);
	return tmpstr;
}

/* returns ad9361 phy device */
static struct iio_device* get_ad9361_phy(void)
{
	struct iio_device *dev = iio_context_find_device(ctx, "ad9361-phy");
	IIO_ENSURE(dev && "No ad9361-phy found");
	return dev;
}

/* finds AD9361 streaming IIO devices */
static bool get_ad9361_stream_dev(enum iodev d, struct iio_device **dev)
{
	switch (d) {
	case TX: *dev = iio_context_find_device(ctx, "cf-ad9361-dds-core-lpc"); return *dev != NULL;
	case RX: *dev = iio_context_find_device(ctx, "cf-ad9361-lpc"); return *dev != NULL;
	default: IIO_ENSURE(0); return false;
	}
}

/* finds AD9361 streaming IIO channels */
static bool get_ad9361_stream_ch(enum iodev d, struct iio_device *dev, int chid, struct iio_channel **chn)
{
	*chn = iio_device_find_channel(dev, get_ch_name("voltage", chid), d == TX);
	if (!*chn)
		*chn = iio_device_find_channel(dev, get_ch_name("altvoltage", chid), d == TX);
	return *chn != NULL;
}

/* finds AD9361 phy IIO configuration channel with id chid */
static bool get_phy_chan(enum iodev d, int chid, struct iio_channel **chn)
{
	switch (d) {
	case RX: *chn = iio_device_find_channel(get_ad9361_phy(), get_ch_name("voltage", chid), false); return *chn != NULL;
	case TX: *chn = iio_device_find_channel(get_ad9361_phy(), get_ch_name("voltage", chid), true);  return *chn != NULL;
	default: IIO_ENSURE(0); return false;
	}
}

/* finds AD9361 local oscillator IIO configuration channels */
static bool get_lo_chan(enum iodev d, struct iio_channel **chn)
{
	switch (d) {
	 // LO chan is always output, i.e. true
	case RX: *chn = iio_device_find_channel(get_ad9361_phy(), get_ch_name("altvoltage", 0), true); return *chn != NULL;
	case TX: *chn = iio_device_find_channel(get_ad9361_phy(), get_ch_name("altvoltage", 1), true); return *chn != NULL;
	default: IIO_ENSURE(0); return false;
	}
}

/* applies streaming configuration through IIO */
bool cfg_ad9361_streaming_ch(struct stream_cfg *cfg, enum iodev type, int chid)
{
	struct iio_channel *chn = NULL;

	// Configure phy and lo channels
	debug_printf(LEVEL_INFO, DEBUG_IIO, "* Acquiring AD9361 phy channel %d\n", chid);
	if (!get_phy_chan(type, chid, &chn)) {	return false; }
	wr_ch_str(chn, "rf_port_select",     cfg->rf_port);
	wr_ch_lli(chn, "rf_bandwidth",       cfg->bw_hz);
	wr_ch_lli(chn, "sampling_frequency", cfg->fs_hz);

	// Configure LO channel
	debug_printf(LEVEL_INFO, DEBUG_IIO, "* Acquiring AD9361 %s lo channel\n", type == TX ? "TX" : "RX");
	if (!get_lo_chan(type, &chn)) { return false; }
	wr_ch_lli(chn, "frequency", cfg->lo_hz);
	return true;
}


void iio_setup(void)
{
	// Streaming devices
	struct iio_device *tx;
	struct iio_device *rx;

	// OPV hardware RX stream config
	struct stream_cfg rxcfg;
	rxcfg.bw_hz = RF_BANDWIDTH;
	rxcfg.fs_hz = MHZ(61.44);	// 2.5 MS/s rx sample rate
	rxcfg.lo_hz = LO_FREQ;
	rxcfg.rf_port = "A_BALANCED";	// port A (select for rf freq.)

	// OPV hardware TX stream config
	struct stream_cfg txcfg;
	txcfg.bw_hz = RF_BANDWIDTH;
	txcfg.fs_hz = MHZ(61.44);	// 2.5 MS/s tx sample rate
	txcfg.lo_hz = LO_FREQ;
	txcfg.rf_port = "A";	// port A (select for rf freq.)

	debug_printf(LEVEL_INFO, DEBUG_IIO, "* Acquiring IIO context\n");
	IIO_ENSURE((ctx = iio_create_default_context()) && "No context");
	IIO_ENSURE(iio_context_get_devices_count(ctx) > 0 && "No devices");

	debug_printf(LEVEL_INFO, DEBUG_IIO, "* Acquiring AD9361 streaming devices\n");
	IIO_ENSURE(get_ad9361_stream_dev(TX, &tx) && "No tx dev found");
	IIO_ENSURE(get_ad9361_stream_dev(RX, &rx) && "No rx dev found");

	debug_printf(LEVEL_INFO, DEBUG_IIO, "* Configuring AD9361 for streaming\n");
	IIO_ENSURE(cfg_ad9361_streaming_ch(&rxcfg, RX, 0) && "RX port 0 not found");
	IIO_ENSURE(cfg_ad9361_streaming_ch(&txcfg, TX, 0) && "TX port 0 not found");

	debug_printf(LEVEL_INFO, DEBUG_IIO, "* Initializing AD9361 IIO streaming channels\n");
	IIO_ENSURE(get_ad9361_stream_ch(RX, rx, 0, &rx0_i) && "RX chan i not found");
	IIO_ENSURE(get_ad9361_stream_ch(RX, rx, 1, &rx0_q) && "RX chan q not found");
	IIO_ENSURE(get_ad9361_stream_ch(TX, tx, 0, &tx0_i) && "TX chan i not found");
	IIO_ENSURE(get_ad9361_stream_ch(TX, tx, 1, &tx0_q) && "TX chan q not found");

	debug_printf(LEVEL_INFO, DEBUG_IIO, "* Enabling IIO streaming channels\n");
	// We are enabling all four of these channels, even though we are actually
	// only using the lower half of the i channel in each case. This may not be
	// strictly necessary, but it seems safer to stick with the full I/Q format
	// for each of transmit and receive, since that is the normal use case for
	// IIO in an SDR. We choose not to diverge from the Analog Devices example
	// code unnecessarily.
	// A call to iio_channel_enable() apparently can't fail. It returns void.
	iio_channel_enable(rx0_i);
	iio_channel_enable(rx0_q);
	iio_channel_enable(tx0_i);
	iio_channel_enable(tx0_q);

	// The number of kernel buffers can be increased from the default of 4 below.
	// This has to be done before iio_device_create_buffer().
	// We go ahead and set the default value explicitly, just to be sure.
	int ret = iio_device_set_kernel_buffers_count(tx, 4);
	if (ret < 0) {
		char buf_test[256];
		iio_strerror(-(int)ret, buf_test, sizeof(buf_test));
		debug_printf(LEVEL_URGENT, DEBUG_IIO, "* set_kernel_buffers (tx) failed : %s\n", buf_test);
		exit(1);
	}

	ret = iio_device_set_kernel_buffers_count(rx, 4);
	if (ret < 0) {
		char buf_test[256];
		iio_strerror(-(int)ret, buf_test, sizeof(buf_test));
		debug_printf(LEVEL_URGENT, DEBUG_IIO, "* set_kernel_buffers (rx) failed : %s\n", buf_test);
		exit(1);
	}


	// Set the timeout to 1 second. This will apply for transmit buffer
	// pushes. For receive, we will have cloned the context and changed
	// its timeout to infinity.
	ret = iio_context_set_timeout(ctx, 1000);
	if (ret < 0) {
		char timeout_test[256];
		iio_strerror(-(int)ret, timeout_test, sizeof(timeout_test));
		debug_printf(LEVEL_URGENT, DEBUG_IIO, "* set_timeout failed : %s\n", timeout_test);
		exit(1);
	}

	debug_printf(LEVEL_INFO, DEBUG_IIO, "* Creating TX IIO buffer, size %d\n", software_tx_processing ? OVP_MODULATOR_FRAME_SIZE : OVP_SINGLE_FRAME_SIZE);
	txbuf = iio_device_create_buffer(tx, software_tx_processing ? OVP_MODULATOR_FRAME_SIZE : OVP_SINGLE_FRAME_SIZE, false);
	if (!txbuf) {
		debug_printf(LEVEL_URGENT, DEBUG_IIO, "Could not create TX buffer");
		cleanup_and_exit(1);
	}

}


void iio_teardown(void)
{
	debug_printf(LEVEL_INFO, DEBUG_IIO, "* Destroying buffers\n");
	if (rxbuf) {
		iio_buffer_cancel(rxbuf);
		iio_buffer_destroy(rxbuf);
	}
	if (txbuf) {
		iio_buffer_cancel(txbuf);
		iio_buffer_destroy(txbuf);
	}

	debug_printf(LEVEL_INFO, DEBUG_IIO, "* Disabling streaming channels\n");
	if (rx0_i) { iio_channel_disable(rx0_i); }
	if (rx0_q) { iio_channel_disable(rx0_q); }
	if (tx0_i) { iio_channel_disable(tx0_i); }
	if (tx0_q) { iio_channel_disable(tx0_q); }

	debug_printf(LEVEL_INFO, DEBUG_IIO, "* Destroying context\n");
	if (ctx) { iio_context_destroy(ctx); }
}


// print an indication of received signal strength (RSSI)
void print_rssi(void) {
	char rssi_buffer[256];

	static struct iio_channel *my_dev_ch = NULL;
	if (my_dev_ch == NULL) {
		my_dev_ch = iio_device_find_channel(get_ad9361_phy(),"voltage0", false);
	}

	int ret = iio_channel_attr_read(my_dev_ch, "rssi", rssi_buffer, sizeof(rssi_buffer));
	if (ret < 0) {
		char rssi_error[256];
		iio_strerror(-(int)ret, rssi_error, sizeof(rssi_error));
		debug_printf(LEVEL_URGENT, DEBUG_IIO, "iio_channel_attr_read(channel, rssi, rssi_buffer, size of rssi_buffer) failed : %s\n", rssi_error);
		debug_printf(LEVEL_INFO, DEBUG_IIO, "rssi: 9999.\n");	// for the output parser
	} else {
		debug_printf(LEVEL_INFO, DEBUG_IIO, "rssi: %s.\n", rssi_buffer);
	}
}


// Send OVP frame data to MSK modulator via IIO buffer in two steps.
// Frame data should already be formatted with scrambling + FEC coding
// or filled out as a special (preamble/postamble/dummy) frame.
// Step 1: move the data into txbuf using iio calls
// (Step 2: iio_push txbuf to the kernel)
// This is step 1.
int load_ovp_frame_into_txbuf(uint8_t *frame_data, size_t frame_size) {
	if (!txbuf) {
		debug_printf(LEVEL_URGENT, DEBUG_IIO, "Error - TX buffer not initialized\n");
		exit(1);
	}

	if (!frame_data || frame_size != (software_tx_processing ? OVP_MODULATOR_FRAME_SIZE : OVP_SINGLE_FRAME_SIZE)) {
		debug_printf(LEVEL_MEDIUM, DEBUG_FRAMES, "Error - invalid frame data\n");
		exit(1);
	}

	debug_printf(LEVEL_INFO, DEBUG_IIO, "Sending %zu bytes to MSK modulator\n", frame_size);

	// Get buffer pointers and step size
	char *p_dat, *p_end;
	ptrdiff_t p_inc;

	p_inc = iio_buffer_step(txbuf);
	p_end = iio_buffer_end(txbuf);
	p_dat = (char *)iio_buffer_first(txbuf, tx0_i);

	// Track how much frame data we've sent
	size_t frame_offset = 0;

	printf("Tracing load_ovp_frame_into_txbuf: ");
	// Fill TX buffer with frame data bytes
	// The MSK modulator expects raw data bytes, not I/Q samples
	// It will internally convert to MSK I/Q modulation
	for (/* p_dat above */; p_dat < p_end; p_dat += p_inc) {
		if (frame_offset < frame_size) {
			// Send frame byte as 16-bit data to MSK modulator
			// MSK modulator expects data width configured in Tx_Data_Width register (32 bits)
			// Use both I and Q channels to send 32 bits total per sample
			uint8_t data_byte = frame_data[frame_offset];

			printf("%02x ", data_byte);
			// Pack byte into 16-bit I/Q format for MSK modulator input.
			// MSK will process this as bit data, not as I/Q samples.
			// If TX_DATA_WIDTH is 8, use the low byte of the I channel.
			// If TX_DATA_WIDTH is 16, use the full I channel.
			// If TX_DATA_WIDTH is 24, use the full I channel and the low byte of the Q channel.
			// If TX_DATA_WIDTH is 32, use the full I and Q channels.
			// Since we may be sending an odd number of bytes (271 with the sync word) we must use TX_DATA_WIDTH of 8.
			((int16_t*)p_dat)[0] = (int16_t)(data_byte);	// Real (I) - lower byte
			((int16_t*)p_dat)[1] = (int16_t)(0);			// Imag (Q)

			frame_offset++;
		} else {
			// Pad with zeros if frame is shorter than buffer
			((int16_t*)p_dat)[0] = 0;
			((int16_t*)p_dat)[1] = 0;
		}
	}
	printf("\n");

	return 0;
}

// Send OVP frame data to MSK modulator via IIO buffer in two steps.
// Step 1: move the data into txbuf using iio calls
// Step 2: iio_push txbuf to the kernel
// This is step 2.
int push_txbuf_to_msk(void) {

	static uint32_t old_xfer_count = 0;
	uint32_t local_ts_base;

	local_ts_base = get_timestamp_ms();
	// debug_printf(LEVEL_INFO, DEBUG_IIO, "time between iio_buffer_push starts was %dms\n", local_ts_base - push_ts_base);
	ssize_t result = iio_buffer_push(txbuf);
	debug_printf(LEVEL_INFO, DEBUG_IIO, "iio_buffer_push took %dms.\n", get_timestamp_ms()-local_ts_base);

	if (result < 0) {
		debug_printf(LEVEL_MEDIUM, DEBUG_IIO, "Error pushing buffer to MSK: %zd\n", result);
		cleanup_and_exit(1);
	}

	// Check that data is flowing to MSK block
	uint32_t new_xfer_count = capture_and_read_msk(OFFSET_MSK(axis_xfer_count));
	uint32_t delta = new_xfer_count - old_xfer_count;

	debug_printf(LEVEL_INFO, DEBUG_MSK, "Buffer pushed, axis_xfer_count delta: %u\n", delta);
	debug_printf(LEVEL_INFO, DEBUG_MSK, "Current axis_xfer_count: %u\n", new_xfer_count);
	old_xfer_count = new_xfer_count;
	return 0;
}
