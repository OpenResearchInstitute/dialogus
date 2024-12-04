// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * msk_rx receiver test code
 *
 * Copyright (C) 2024 IABG mbH and ORI
 * Author: Michael Feilen <feilen_at_iabg.de> and Abraxas3d
 **/

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <signal.h>
#include <stdio.h>
#include <iio.h>

/* for pow() */
#include <math.h>

/* for usleep function */
#include <unistd.h>

/* for memory management */
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/mman.h>


/* helper macros */
#define MHZ(x) ((long long)(x*1000000.0 + .5))
#define GHZ(x) ((long long)(x*1000000000.0 + .5))

#define IIO_ENSURE(expr) { \
	if (!(expr)) { \
		(void) fprintf(stderr, "assertion failed (%s:%d)\n", __FILE__, __LINE__); \
		(void) abort(); \
	} \
}


//ENDLESS_PRBS will block STREAMING as is

// we can have RX_ACTIVE with STREAMING
// we can have RX_ACTIVE with ENDLESS_PRBS
// since they are different while loops, it's one or the other
// RX_ACTIVE off chooses the internal digital PRBS loopback.
// RX_ACTIVE on has PTT off and loopback off.
// RF_LOOPBACK has PTT on and loopback off.
// RF_LOOPBACK is used for physically looping back TX to RX
// with a cable and an attentuator.

//#define STREAMING
//#define RX_ACTIVE
#define RF_LOOPBACK
#define ENDLESS_PRBS







#define TX_DMAC_CONTROL_REGISTER 0x00
#define TX_DMAC_STATUS_REGISTER 0x04
#define TX_DMAC_IDENTIFICATION 0x000c
#define TX_DMAC_SCRATCH 0x0008
#define TX_DMAC_INTERFACE 0x0010
#define TX_DMAC_DEST_ADDRESS 0x0410
#define TX_DMAC_SRC_ADDRESS 0x0414
#define TX_DMAC_PERIPHERAL_ID 0x004
#define ENCODER_CONTROL_REGISTER 0x00

// For the MSK registers, we use RDL headers
#include "msk_top_regs.h"

// one bit time is 19 microseconds
float num_microseconds = 5*20;
float one_bit_time = 19;
float percent_error = 55.0;
int i; //index variable for loops


//read from a memory mapped register
unsigned int read_dma(unsigned int *virtual_addr, int offset)
{
        return virtual_addr[offset>>2];
}

//read a value from the MSK register
//value = READ_MSK(MSK_Init);
#define READ_MSK(offset) (msk_register_map->offset)

//write to a memory mapped register
unsigned int write_dma(unsigned int *virtual_addr, int offset, unsigned int value)
{       virtual_addr[offset>>2] = value;
        return 0;
}

//from devmem in sbin, we know: read_result = *(volatile uint32_t*)virt_addr;
//write a value to an MSK register
//WRITE_MSK(MSK_Init, 0x00000001);
#define WRITE_MSK(offset, value) *(volatile uint32_t*)&(msk_register_map->offset) = value


//get the address offset of an MSK register
//value = OFFSET_MSK(MSK_Init);
#define OFFSET_MSK(offset) (offsetof(msk_top_regs_t, offset))



//delete
void  dma_interface(unsigned int *virtual_addr)
{
        unsigned int interface = read_dma(virtual_addr, TX_DMAC_INTERFACE);
        printf("TX DMAC Interface Description (0x%08x@0x%04x):\n", interface, TX_DMAC_INTERFACE);
        //break out and parse the fields in human readable format
}









/* RX is input, TX is output */
enum iodev { RX, TX };

/* common RX and TX streaming params */
struct stream_cfg {
	long long bw_hz; // Analog banwidth in Hz
	long long fs_hz; // Baseband sample rate in Hz
	long long lo_hz; // Local oscillator frequency in Hz
	const char* rfport; // Port name
};

/* static scratch mem for strings */
static char tmpstr[64];

/* IIO structs required for streaming */
static struct iio_context *ctx   = NULL;
static struct iio_channel *rx0_i = NULL;
static struct iio_channel *rx0_q = NULL;
static struct iio_channel *tx0_i = NULL;
static struct iio_channel *tx0_q = NULL;
static struct iio_buffer  *rxbuf = NULL;
static struct iio_buffer  *txbuf = NULL;

static bool stop;

/* cleanup and exit */
static void shutdown(void)
{

	printf("* Destroying buffers\n");
	if (rxbuf) { iio_buffer_destroy(rxbuf); }
	if (txbuf) { iio_buffer_destroy(txbuf); }

	printf("* Disabling streaming channels\n");
	if (rx0_i) { iio_channel_disable(rx0_i); }
	if (rx0_q) { iio_channel_disable(rx0_q); }
	if (tx0_i) { iio_channel_disable(tx0_i); }
	if (tx0_q) { iio_channel_disable(tx0_q); }

	printf("* Destroying context\n");
	if (ctx) { iio_context_destroy(ctx); }
	exit(0);
}

static void handle_sig(int sig)
{
	printf("Waiting for process to finish... Got signal %d\n", sig);
	stop = true;
}

/* check return value of attr_write function */
static void errchk(int v, const char* what) {
	 if (v < 0) { fprintf(stderr, "Error %d writing to channel \"%s\"\nvalue may not be supported.\n", v, what); shutdown(); }
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
static char* get_ch_name(const char* type, int id)
{
	snprintf(tmpstr, sizeof(tmpstr), "%s%d", type, id);
	return tmpstr;
}

/* returns ad9361 phy device */
static struct iio_device* get_ad9361_phy(void)
{
	struct iio_device *dev =  iio_context_find_device(ctx, "ad9361-phy");
	IIO_ENSURE(dev && "No ad9361-phy found");
	return dev;
}

/* finds AD9361 streaming IIO devices */
static bool get_ad9361_stream_dev(enum iodev d, struct iio_device **dev)
{
	switch (d) {
	case TX: *dev = iio_context_find_device(ctx, "cf-ad9361-dds-core-lpc"); return *dev != NULL;
	case RX: *dev = iio_context_find_device(ctx, "cf-ad9361-lpc");  return *dev != NULL;
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
	printf("* Acquiring AD9361 phy channel %d\n", chid);
	if (!get_phy_chan(type, chid, &chn)) {	return false; }
	wr_ch_str(chn, "rf_port_select",     cfg->rfport);
	wr_ch_lli(chn, "rf_bandwidth",       cfg->bw_hz);
	wr_ch_lli(chn, "sampling_frequency", cfg->fs_hz);

	// Configure LO channel
	printf("* Acquiring AD9361 %s lo channel\n", type == TX ? "TX" : "RX");
	if (!get_lo_chan(type, &chn)) { return false; }
	wr_ch_lli(chn, "frequency", cfg->lo_hz);
	return true;
}

/* simple configuration and streaming */
/* usage:
 * Default context, assuming local IIO devices, i.e., this script is run on ADALM-Pluto for example
 $./a.out
 * URI context, find out the uri by typing `iio_info -s` at the command line of the host PC
 $./a.out usb:x.x.x
 */
int main (int argc, char **argv)
{
	// Streaming devices
	struct iio_device *tx;
	struct iio_device *rx;

	// RX and TX sample counters
	size_t nrx = 0;
	size_t ntx = 0;

	// Stream configurations
	struct stream_cfg rxcfg;
	struct stream_cfg txcfg;

	// Listen to ctrl+c and IIO_ENSURE
	signal(SIGINT, handle_sig);


/*
	// original RX stream config
	rxcfg.bw_hz = MHZ(2);   // 2 MHz rf bandwidth
	rxcfg.fs_hz = MHZ(2.5);   // 2.5 MS/s rx sample rate
	rxcfg.lo_hz = GHZ(2.5); // 2.5 GHz rf frequency
	rxcfg.rfport = "A_BALANCED"; // port A (select for rf freq.)
*/


        // OPV hardware RX stream config
        rxcfg.bw_hz = MHZ(3);   // 2 MHz rf bandwidth
        rxcfg.fs_hz = MHZ(61.44);   // 2.5 MS/s rx sample rate
        rxcfg.lo_hz = MHZ(905.05); // 905.05 MHz rf frequency
        rxcfg.rfport = "A_BALANCED"; // port A (select for rf freq.)

/*
	// original TX stream config
	txcfg.bw_hz = MHZ(1.5); // 1.5 MHz rf bandwidth
	txcfg.fs_hz = MHZ(2.5);   // 2.5 MS/s tx sample rate
	txcfg.lo_hz = GHZ(2.5); // 2.5 GHz rf frequency
	txcfg.rfport = "A"; // port A (select for rf freq.)
*/

        // OPV hardware TX stream config
        txcfg.bw_hz = MHZ(3); // 1.5 MHz rf bandwidth
        txcfg.fs_hz = MHZ(61.44);   // 2.5 MS/s tx sample rate
        txcfg.lo_hz = MHZ(905.05); // 905.05 MHz rf frequency
        txcfg.rfport = "A"; // port A (select for rf freq.)


	printf("* Acquiring IIO context\n");
	if (argc == 1) {
		IIO_ENSURE((ctx = iio_create_default_context()) && "No context");
	}
	else if (argc == 2) {
		IIO_ENSURE((ctx = iio_create_context_from_uri(argv[1])) && "No context");
	}
	IIO_ENSURE(iio_context_get_devices_count(ctx) > 0 && "No devices");

	printf("* Acquiring AD9361 streaming devices\n");
	IIO_ENSURE(get_ad9361_stream_dev(TX, &tx) && "No tx dev found");
	IIO_ENSURE(get_ad9361_stream_dev(RX, &rx) && "No rx dev found");

	printf("* Configuring AD9361 for streaming\n");
	IIO_ENSURE(cfg_ad9361_streaming_ch(&rxcfg, RX, 0) && "RX port 0 not found");
	IIO_ENSURE(cfg_ad9361_streaming_ch(&txcfg, TX, 0) && "TX port 0 not found");

	printf("* Initializing AD9361 IIO streaming channels\n");
	IIO_ENSURE(get_ad9361_stream_ch(RX, rx, 0, &rx0_i) && "RX chan i not found");
	IIO_ENSURE(get_ad9361_stream_ch(RX, rx, 1, &rx0_q) && "RX chan q not found");
	IIO_ENSURE(get_ad9361_stream_ch(TX, tx, 0, &tx0_i) && "TX chan i not found");
	IIO_ENSURE(get_ad9361_stream_ch(TX, tx, 1, &tx0_q) && "TX chan q not found");

	printf("* Enabling IIO streaming channels\n");
	iio_channel_enable(rx0_i);
	iio_channel_enable(rx0_q);
	iio_channel_enable(tx0_i);
	iio_channel_enable(tx0_q);



        // number of kernel buffers can be increased from the default of 4 below
	// this has to be done before iio_device_create_buffer()
        int ret = iio_device_set_kernel_buffers_count(tx, 4);
        if (ret < 0) {
                char buf_test[256];
                iio_strerror(-(int)ret, buf_test, sizeof(buf_test));
                printf("* set_kernel_buffers failed : %s\n", buf_test);
        }
	else {
		printf("* set_kernel_buffers returned %d, which is a success.\n", ret);
	}



/*
	// set the timeout higher to see if we can get a RX buffer refill without errors
	// argument is the iio context and the number of milliseconds
	ret = iio_context_set_timeout(ctx, 990);
        if (ret < 0) {
                char timeout_test[256];
                iio_strerror(-(int)ret, timeout_test, sizeof(timeout_test));
                printf("* set_timout failed : %s\n", timeout_test);
        }
        else {
                printf("* set_timout returned %d, which is a success.\n", ret);
        }

*/




	printf("* Creating non-cyclic IIO buffers with 1 MiS\n");
// original size of the rxbuf
//	rxbuf = iio_device_create_buffer(rx, 1024*1024, false);
// size of our rxbuf
        rxbuf = iio_device_create_buffer(rx, 1024*1, false);
	if (!rxbuf) {
		perror("Could not create RX buffer");
		shutdown();
	}
// original size of the txbuf
//	txbuf = iio_device_create_buffer(tx, 1024*1024, false);
// size of our txbuf
        txbuf = iio_device_create_buffer(tx, 1024*1, false);
	if (!txbuf) {
		perror("Could not create TX buffer");
		shutdown();
	}




        printf("Hello World!\n");
        printf("Opening character device files in DDR memory.\n");
        int ddr_memory = open("/dev/mem", O_RDWR | O_SYNC);
        printf("Memory map the address of the TX-DMAC via its AXI lite control interface register block.\n");
        unsigned int *dma_virtual_addr = mmap(NULL, 65535, PROT_READ | PROT_WRITE, MAP_SHARED, ddr_memory, 0x7c420000);

	printf("RDL Memory map the address of the MSK block via its AXI lite control interface.\n");
	msk_top_regs_t *msk_register_map = mmap(NULL, 65535, PROT_READ | PROT_WRITE, MAP_SHARED, ddr_memory, 0x43c00000);

        printf("Memory map the address of the MSK block via its AXI lite control interface.\n");
        unsigned int *msk_virtual_addr = mmap(NULL, 65535, PROT_READ | PROT_WRITE, MAP_SHARED, ddr_memory, 0x43c00000);

        printf("Create a buffer for some transmitted data.\n");
	unsigned int transmit_data[4*100];
        dma_interface(dma_virtual_addr);






	printf("Writing to scratch register in TX-DMAC.\n");
        write_dma(dma_virtual_addr, TX_DMAC_SCRATCH, 0x5555AAAA);
        printf("Reading from scratch register in TX-DMAC. We see: (0x%08x@%04x)\n", read_dma(dma_virtual_addr, TX_DMAC_SCRATCH), TX_DMAC_SCRATCH);
	printf("Reading the TX-DMAC peripheral ID: (0x%08x@%04x)\n", read_dma(dma_virtual_addr, TX_DMAC_PERIPHERAL_ID), TX_DMAC_PERIPHERAL_ID);





        printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
        printf("Configure MSK for minimum viable product test.\n");
        printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
        printf("Reading from MSK block HASH ID LOW: (0x%08x@%04x)\n", READ_MSK(Hash_ID_Low), OFFSET_MSK(Hash_ID_Low));
        printf("Reading from MSK block HASH ID HIGH: (0x%08x@%04x)\n", READ_MSK(Hash_ID_High), OFFSET_MSK(Hash_ID_High));
        printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
	printf("Initialize MSK block.\n");
	printf("Read MSK_INIT: (0x%08x@%04x)\n", READ_MSK(MSK_Init), OFFSET_MSK(MSK_Init));
	printf("bit 0: 0 is normal operation and 1 is initialize modem (reset condition).\n");
	printf("Assert INIT: Write 1 to MSK_INIT\n");
	WRITE_MSK(MSK_Init, 0x00000001);
	printf("Reading MSK_INIT. We see: (0x%08x@%04x)\n", READ_MSK(MSK_Init), OFFSET_MSK(MSK_Init));


	//Receiver Active
	#ifdef RX_ACTIVE
	printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
        printf("RX_ACTIVE on: Writing MSK_CONTROL register for receiver only.\n");
	printf("PTT and loopback disabled.\n");
	WRITE_MSK(MSK_Control, 0x00000000);
        printf("Reading back MSK_CONTROL status register. We see: (0x%08x@%04x)\n", READ_MSK(MSK_Control), OFFSET_MSK(MSK_Control));
        printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");

	#else
	//digital loopback
	printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
	printf("RX_ACTIVE off: Writing MSK_CONTROL register for digital loopback.\n");
        printf("PTT,loopback, rx_invert enabled.\n");
	WRITE_MSK(MSK_Control, 0x00000007);
	printf("Reading back MSK_CONTROL status register. We see: (0x%08x@%04x)\n", READ_MSK(MSK_Control), OFFSET_MSK(MSK_Control));
        printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
	#endif

	#ifdef RF_LOOPBACK
        printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
        printf("RF_LOOPACK on: Writing MSK_CONTROL register for RF loopback.\n");
        printf("PTT enabled and loopback disabled.\n");
        WRITE_MSK(MSK_Control, 0x00000001);
        printf("Reading back MSK_CONTROL status register. We see: (0x%08x@%04x)\n", READ_MSK(MSK_Control), OFFSET_MSK(MSK_Control));
        printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");

	#else
	#endif 


	printf("Reading the MSK_STATUS register, we see: (0x%08x@%04x)\n", READ_MSK(MSK_Status), OFFSET_MSK(MSK_Status));
	printf("Bit 0 is demod_sync(not implemented), bit 1 is tx_enable, bit 2 is rx_enable\n");
	printf("tx_enable is data to DAC enabled. rx_enable is data from ADC enable.\n");
        printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
	printf("TX_BIT_COUNT register is read, we see: (0x%08x@%04x)\n",  READ_MSK(Tx_Bit_Count), OFFSET_MSK(Tx_Bit_Count));
	printf("This register reads out the count of data requests made by the modem.\n");
        printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
	printf("TX_ENABLE_COUNT register is read and write. It holds the number of clocks on which Tx Enable is active.\n");
	printf("First we read it, we see: (0x%08x@%04x)\n", READ_MSK(Tx_Enable_Count), OFFSET_MSK(Tx_Enable_Count));





        printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
	printf("Writing fb, f1, f2 (values are calculated for MSK TX).\n");

	double bitrate, freq_if, delta_f, f1, f2, br_fcw, f1_fcw_tx, f2_fcw_tx, f1_fcw_rx, f2_fcw_rx, tx_sample_rate, rx_sample_rate, tx_rx_sample_ratio;


	bitrate = 54200;
	freq_if = (bitrate/4)*32;
	tx_sample_rate = 61440000;
	tx_rx_sample_ratio = 25;
	rx_sample_rate = tx_sample_rate / tx_rx_sample_ratio;

	delta_f = bitrate/4;
	f1 = freq_if - delta_f;
	f2 = freq_if + delta_f;
	br_fcw = (bitrate/tx_sample_rate) * pow(2.0, 32.0);
	f1_fcw_tx = (f1/tx_sample_rate) * pow(2.0, 32.0);
	f2_fcw_tx = (f2/tx_sample_rate) * pow(2.0, 32.0);
        f1_fcw_rx = (f1/tx_sample_rate) * pow(2.0, 32.0);
        f2_fcw_rx = (f2/tx_sample_rate) * pow(2.0, 32.0);


        WRITE_MSK(Fb_FreqWord, (uint32_t) br_fcw);
        WRITE_MSK(TX_F1_FreqWord, (uint32_t) f1_fcw_tx);
        WRITE_MSK(TX_F2_FreqWord, (uint32_t) f2_fcw_tx);



        printf("FB_FREQWORD: (0x%08x@%04x)\n", READ_MSK(Fb_FreqWord), OFFSET_MSK(Fb_FreqWord));
        printf("expecting to see: %f float cast as uint32_t: 0x%08x \n", br_fcw, (uint32_t) br_fcw);
        printf("TX_F1_FREQWORD: (0x%08x@%04x)\n", READ_MSK(TX_F1_FreqWord), OFFSET_MSK(TX_F1_FreqWord));
        printf("expecting to see: %f float cast as uint32_t: 0x%08x \n", f1_fcw_tx, (uint32_t) f1_fcw_tx);
        printf("TX_F2_FREQWORD: (0x%08x@%04x)\n", READ_MSK(TX_F2_FreqWord), OFFSET_MSK(TX_F2_FreqWord));
        printf("expecting to see: %f float cast as uint32_t: 0x%08x \n", f2_fcw_tx, (uint32_t) f2_fcw_tx);


        printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
        printf("Writing f1, f2 (values are calculated for MSK RX).\n");

        WRITE_MSK(RX_F1_FreqWord, (uint32_t) f1_fcw_rx);
        WRITE_MSK(RX_F2_FreqWord, (uint32_t) f2_fcw_rx);

        printf("RX_F1_FREQWORD: (0x%08x@%04x)\n", READ_MSK(RX_F1_FreqWord), OFFSET_MSK(RX_F1_FreqWord));
        printf("expecting to see: %f float cast as uint32_t: 0x%08x \n", f1_fcw_rx, (uint32_t) f1_fcw_rx);
        printf("RX_F2_FREQWORD: (0x%08x@%04x)\n", READ_MSK(RX_F2_FreqWord), OFFSET_MSK(RX_F2_FreqWord));
        printf("expecting to see: %f float cast as uint32_t: 0x%08x \n", f2_fcw_rx, (uint32_t) f2_fcw_rx);

        printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
        printf("Reading the LPF_CONFIG_0 register.\n");
        printf("First we read it, we see: (0x%08x@%04x)\n", READ_MSK(LPF_Config_0), OFFSET_MSK(LPF_Config_0));
	printf("bit 0 is whether or not we freeze the accumulator's current value.\n");
	printf("bit 1 holds the PI accumulator at zero.\n");
	printf("bits 31:16 are the LPF IIR alpha value.\n");
        printf("Reading the LPF_CONFIG_1 register.\n");
        printf("First we read it, we see: (0x%08x@%04x)\n", READ_MSK(LPF_Config_1), OFFSET_MSK(LPF_Config_1));
        printf("bit 23:0 sets the integral gain of the PI controller integrator.\n");
	printf("bit 31:24 sets the integral gain bit shift.\n");
        printf("Reading the LPF_CONFIG_2 register.\n");
        printf("First we read it, we see: (0x%08x@%04x)\n", READ_MSK(LPF_Config_2), OFFSET_MSK(LPF_Config_2));
        printf("bit 23:0 sets the proportional gain of the PI controller integrator.\n");
        printf("bit 31:24 sets the proportional gain bit shift.\n");

	WRITE_MSK(LPF_Config_0, 0x00000002); //zero and hold accumulators
        printf("wrote 0x00000002 to LPF_Config_0: (0x%08x@%04x)\n", READ_MSK(LPF_Config_0), OFFSET_MSK(LPF_Config_0));
        usleep(num_microseconds);
        WRITE_MSK(LPF_Config_0, 0x00000000); //accumulators in normal operation
        printf("Wrote all 0 LPF_Config_0: (0x%08x@%04x)\n", READ_MSK(LPF_Config_0), OFFSET_MSK(LPF_Config_0));

        printf("Write some default values for PI gain and bit shift.\n");
        WRITE_MSK(LPF_Config_1, 0x005a5a5a);
	WRITE_MSK(LPF_Config_2, 0x00a5a5a5);
	usleep(num_microseconds);
        printf("LPF_Config_0: (0x%08x@%04x)\n", READ_MSK(LPF_Config_0), OFFSET_MSK(LPF_Config_0));
        printf("LPF_Config_1: (0x%08x@%04x)\n", READ_MSK(LPF_Config_1), OFFSET_MSK(LPF_Config_1));
        printf("LPF_Config_2: (0x%08x@%04x)\n", READ_MSK(LPF_Config_2), OFFSET_MSK(LPF_Config_2));
	usleep(num_microseconds);

        printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
        printf("Read TX_DATA_WIDTH, which is the modem transmit input/output data width.\n");
        printf("We see: (0x%08x@%04x)\n", READ_MSK(Tx_Data_Width), OFFSET_MSK(Tx_Data_Width));
	printf("Set TX_DATA_WIDTH to 32.\n");
	WRITE_MSK(Tx_Data_Width, 0x00000020);
        printf("Read TX_DATA_WIDTH.\n");
        printf("We see: (0x%08x@%04x)\n", READ_MSK(Tx_Data_Width), OFFSET_MSK(Tx_Data_Width));

        printf("Read RX_DATA_WIDTH, which is the modem transmit input/output data width.\n");
        printf("We see: (0x%08x@%04x)\n", READ_MSK(Rx_Data_Width), OFFSET_MSK(Rx_Data_Width));
        printf("Set RX_DATA_WIDTH to 32.\n");
	WRITE_MSK(Rx_Data_Width, 0x00000020);
        printf("Read RX_DATA_WIDTH.\n");
        printf("We see: (0x%08x@%04x)\n", READ_MSK(Rx_Data_Width), OFFSET_MSK(Rx_Data_Width));



        printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
	printf("Pseudo Random Binary Sequence control registers are read.\n");
        printf("We read PRBS_CONTROL: (0x%08x@%04x)\n", READ_MSK(PRBS_Control), OFFSET_MSK(PRBS_Control)); 
	printf("bit 0 is PRBS data select. 0 is normal data transmit and 1 is PRBS transmit.\n");
	printf("bit 1 is PRBS error insert. 0 is no error insertion and 1 inserts a bit error in transmit.\n");
	printf("NOTE: error is inserted in both normal and PRBS data selection modes.\n");
	printf("bit 2 clears PRBS counters.\n");
	printf("bit 3 transition is a manual PRBS sync.\n");
	printf("bit 31:16 prbs_sync_threshold.\n"); 
        printf("We read PRBS_INITIAL_STATE: (0x%08x@%04x)\n", READ_MSK(PRBS_Initial_State), OFFSET_MSK(PRBS_Initial_State));
	printf("This is the PRBS seed value. It sets the starting value of the PRBS generator.\n");
        printf("We read PRBS_POLYNOMIAL: (0x%08x@%04x)\n", READ_MSK(PRBS_Polynomial), OFFSET_MSK(PRBS_Polynomial));
	printf("Bit positions set to 1 indicate polynomial feedback positions.\n");
        printf("We read PRBS_ERROR_MASK: (0x%08x@%04x)\n", READ_MSK(PRBS_Error_Mask), OFFSET_MSK(PRBS_Error_Mask));
	printf("Bit positions set to 1 indicate bits that are inverted when a bit error is inserted.\n");
        printf("We read PRBS_BIT_COUNT: (0x%08x@%04x)\n", READ_MSK(PRBS_Bit_Count), OFFSET_MSK(PRBS_Bit_Count));
	printf("Number of bits received by the PRBS monitor since last BER\n");
	printf("can be calculated as the ratio of received bits to errored-bits.\n");
        printf("We read PRBS_ERROR_COUNT: (0x%08x@%04x)\n", READ_MSK(PRBS_Error_Count), OFFSET_MSK(PRBS_Error_Count));
        printf("Number of errored-bits received by the PRBS monitor since last BER\n");
        printf("can be calculated as the ratio of received bits to errored-bits.\n");
        printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
        printf("We read LPF_ACCUM_F1: (0x%08x@%04x)\n", READ_MSK(LPF_Accum_F1), OFFSET_MSK(LPF_Accum_F1));
        printf("PI conotroller accumulator value.\n");
        printf("We read LPF_ACCUM_F2: (0x%08x@%04x)\n", READ_MSK(LPF_Accum_F2), OFFSET_MSK(LPF_Accum_F2));
        printf("PI conotroller accumulator value.\n");


        printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
	printf("Attempt to set up PRBS.\n");
	printf("Write 0x00000001 to PRBS_CONTROL. PRBS active (bit 0), no errors inserted (bit 1).\n");
//	write_dma(msk_virtual_addr, 0x40, 0x87654321);
	WRITE_MSK(PRBS_Control, 0x00000001);
        printf("We read PRBS_CONTROL: (0x%08x@%04x)\n", READ_MSK(PRBS_Control), OFFSET_MSK(PRBS_Control));
	printf("Write a value to PRBS_INITIAL_STATE, as the seed.\n");
//	write_dma(msk_virtual_addr, 0x44, 0x8E7589FD);
        WRITE_MSK(PRBS_Initial_State, 0x8E7589FD);
        printf("We read PRBS_INITIAL_STATE: (0x%08x@%04x)\n", READ_MSK(PRBS_Initial_State), OFFSET_MSK(PRBS_Initial_State));
//        printf("Write 0xA3000000 to PRBS_POLYNOMIAL (32,30,26,25), a max length Fibonacci sequence generators.\n");
//        write_dma(msk_virtual_addr, PRBS_POLYNOMIAL, 0xA3000000);
	printf("Write 0x48000000 to PRBS_POLYNOMIAL (31,28), a max length Fibonacci sequence generators.\n");
//        write_dma(msk_virtual_addr, 0x48, 0x48000000);
        WRITE_MSK(PRBS_Polynomial, 0x48000000);
        printf("We read PRBS_POLYNOMIAL: (0x%08x@%04x)\n", READ_MSK(PRBS_Polynomial), OFFSET_MSK(PRBS_Polynomial));
	printf("Write 0x00000001 to PRBS_ERROR_MASK.\n");
//        write_dma(msk_virtual_addr, 0x4c, 0x00000001);
        WRITE_MSK(PRBS_Error_Mask, 0x00000001);
        printf("We read PRBS_ERROR_MASK: (0x%08x@%04x)\n", READ_MSK(PRBS_Error_Mask), OFFSET_MSK(PRBS_Error_Mask));


	//initial values of parameterized LPF_CONFIG are set up here
	int32_t proportional_gain = 0x00000243; //0x0012984F for 32 bits 0x00001298 for 24 bits 243 for OE 
	int32_t integral_gain = 0x00000080; //0x0000C067 for 32 bits
	int32_t gain_bit_shift = 0x0000000E; //0x18 is 24 and 0x20 is 32
	int32_t proportional_config = (gain_bit_shift << 24) | (proportional_gain & 0x00FFFFFF);
	int32_t integral_config = (gain_bit_shift << 24) | (integral_gain & 0x00FFFFFF);
	printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
	printf("Write proportional and integral gains to LPF_CONFIG_2 and LPF_CONFIG_1.\n");
	printf("Proportional config: (0x%08x) integral config: (0x%08x)\n", proportional_config, integral_config);
        WRITE_MSK(LPF_Config_1, integral_config);
	WRITE_MSK(LPF_Config_2, proportional_config);

	//test xfer register reads
	printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
	printf("The value of axis_xfer_count is: (0x%08x@%04x)\n", READ_MSK(axis_xfer_count), OFFSET_MSK(axis_xfer_count));

	//discard 24 receiver samples and 24 NCO Samples
	WRITE_MSK(Rx_Sample_Discard, 0x00001818);
        printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
	printf("Read RX_SAMPLE_DISCARD: (0x%08x@%04x)\n", READ_MSK(Rx_Sample_Discard), OFFSET_MSK(Rx_Sample_Discard));
	printf("bits 0:7 are receiver sample discard and bits 15:8 are NCO sample discard.\n");




        printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
        printf("Deassert INIT: Write 0 to MSK_INIT\n");
        usleep(num_microseconds);
        WRITE_MSK(MSK_Init, 0x00000000);
        printf("Read MSK_INIT: (0x%08x@%04x)\n", READ_MSK(MSK_Init), OFFSET_MSK(MSK_Init));




	//loop variables
	int buckets = 0;
	int max_without_zeros = 0;
	int zero_segments = 0;
	int spectacular_success = 0;

	// ENDLESS_PRBS runs PRBS based transmit indefinitely
	#ifdef ENDLESS_PRBS
	while(!stop)
	{
	#endif


	while(percent_error > 0.1){

		if(stop){
			break;
		}


	        printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
		printf("100 (or more) buckets of bits on the bus.\n");

		for (buckets = 0; buckets < 10000; buckets++) {
	                usleep(one_bit_time);
		}

	        printf("(1) Total PRBS_BIT_COUNT:   (0x%08x@%04x)\n", READ_MSK(PRBS_Bit_Count), OFFSET_MSK(PRBS_Bit_Count));
	        printf("(1) Total PRBS_ERROR_COUNT: (0x%08x@%04x)\n", READ_MSK(PRBS_Error_Count), OFFSET_MSK(PRBS_Error_Count));
		percent_error = (   (double)(READ_MSK(PRBS_Error_Count))  /  (double)(READ_MSK(PRBS_Bit_Count))    )*100;
		printf("(1) %2.1f %d %d 0x%08x 0x%08x\n", percent_error, READ_MSK(LPF_Accum_F1), READ_MSK(LPF_Accum_F2), READ_MSK(LPF_Config_2),
								READ_MSK(LPF_Config_1));

                if (isnan(percent_error)) {
                   printf("BOOM!\n");
                   stop = true;
                   }

		if (percent_error > 49.0){

                        printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
                        printf("Toggle RX_INVERT to test 180 degree phase shift.\n");
                        WRITE_MSK(MSK_Control,(READ_MSK(MSK_Control)^0x00000004));
                        usleep(num_microseconds);

			printf("We read MSK_CONTROL: (0x%08x@%04x)\n", READ_MSK(MSK_Control), OFFSET_MSK(MSK_Control));
                        //usleep(num_microseconds*1000); //test increasing this delay?


		        printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
			printf("percent_error was greater than 49.0 percent so we resync.\n");
		        printf("resync PRBS by toggling bit 3 of PRBS_CONTROL.\n");
		        WRITE_MSK(PRBS_Control, (READ_MSK(PRBS_Control)^0x00000008));
		        usleep(num_microseconds);
		        printf("We read PRBS_CONTROL: (0x%08x@%04x)\n", READ_MSK(PRBS_Control), OFFSET_MSK(PRBS_Control));

                        printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
                        printf("reset the PRBS_BIT_COUNT and PRBS_ERROR_COUNT counters.\n");
			printf("Toggle bit 2 of PRBS_CONTROL.\n");
                        WRITE_MSK(PRBS_Control, (READ_MSK(PRBS_Control)^0x00000004));
                        usleep(num_microseconds);
                        printf("We read PRBS_CONTROL: (0x%08x@%04x)\n", READ_MSK(PRBS_Control), OFFSET_MSK(PRBS_Control));
                        printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
                        printf("We read MSK_CONTROL: (0x%08x@%04x)\n", READ_MSK(MSK_Control), OFFSET_MSK(MSK_Control));


                        max_without_zeros++;
			//printf("Increment max_without_zeros here.\nmax_without_zeros now equals: %d\n", max_without_zeros);

			if(max_without_zeros > 20){
				//increment proportional and/or integral gains here
				proportional_gain = proportional_gain + 1;
				//integral_gain = integral_gain + 1;

			        int32_t proportional_config = (gain_bit_shift << 24) | (proportional_gain & 0x00FFFFFF);
			        int32_t integral_config = (gain_bit_shift << 24) | (integral_gain & 0x00FFFFFF);
			        printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
			        printf("Write proportional and integral gains to LPF_CONFIG_2 and LPF_CONFIG_1.\n");
			        printf("Proportional config: (0x%08x) integral config: (0x%08x)\n", proportional_config, integral_config);
			        WRITE_MSK(LPF_Config_1, integral_config);
			        WRITE_MSK(LPF_Config_2, proportional_config);

				max_without_zeros = 0;

                                printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
                                printf("max_without_zeros exceeded. Giving up on this gain pair.\n");

        			printf("Assert RX INIT: Write 1 to bit 2 of MSK_INIT\n");
        			WRITE_MSK(MSK_Init, 0x00000004);
        			printf("Reading MSK_INIT. We see: (0x%08x@%04x)\n", READ_MSK(MSK_Init), OFFSET_MSK(MSK_Init));
				usleep(num_microseconds);
        			printf("De-Assert INIT: Write 0 to MSK_INIT\n");
        			WRITE_MSK(MSK_Init, 0x00000000);
        			printf("Reading MSK_INIT. We see: (0x%08x@%04x)\n", READ_MSK(MSK_Init), OFFSET_MSK(MSK_Init));
				usleep(num_microseconds);



	                        printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
                                printf("Zero the accumulators.\n");
                                printf("Bit 1 of LPF_CONFIG_0 is set and then cleared.\n");
				printf("31:16 is the filter alpha.\n");
                                printf("Write 0xXXXX0002 to LPF_CONFIG_0.\n");
                                WRITE_MSK(LPF_Config_0, 0x00000002);
                                usleep(num_microseconds);
                                printf("We read LPF_CONFIG_0: (0x%08x@%04x)\n", READ_MSK(LPF_Config_0), OFFSET_MSK(LPF_Config_0));
                                WRITE_MSK(LPF_Config_0, 0x00000000);
                                usleep(num_microseconds);
                                printf("We read LPF_CONFIG_0: (0x%08x@%04x)\n", READ_MSK(LPF_Config_0), OFFSET_MSK(LPF_Config_0));
			        printf("We read LPF_ACCUM_F1: (0x%08x@%04x)\n", READ_MSK(LPF_Accum_F1), OFFSET_MSK(LPF_Accum_F1));
			        printf("F1 PI controller accumulator value.\n");
			        printf("We read LPF_ACCUM_F2: (0x%08x@%04x)\n", READ_MSK(LPF_Accum_F2), OFFSET_MSK(LPF_Accum_F2));
			        printf("F2 PI controller accumulator value.\n");



	                        printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
	                        printf("percent_error was greater than 49.0 percent so we resync.\n");
	                        printf("resync PRBS by toggling bit 3 of PRBS_CONTROL.\n");
	                        WRITE_MSK(PRBS_Control, (READ_MSK(PRBS_Control)^0x00000008));
	                        usleep(num_microseconds);
	                        printf("We read PRBS_CONTROL: (0x%08x@%04x)\n", READ_MSK(PRBS_Control), OFFSET_MSK(PRBS_Control));

	                        printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
	                        printf("reset the PRBS_BIT_COUNT and PRBS_ERROR_COUNT counters.\n");
	                        printf("Toggle bit 2 of PRBS_CONTROL.\n");
	                        WRITE_MSK(PRBS_Control, (READ_MSK(PRBS_Control)^0x00000004));
	                        usleep(num_microseconds);
	                        printf("We read PRBS_CONTROL: (0x%08x@%04x)\n", READ_MSK(PRBS_Control), OFFSET_MSK(PRBS_Control));
	                        printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
	                        printf("We read MSK_CONTROL: (0x%08x@%04x)\n", READ_MSK(MSK_Control), OFFSET_MSK(MSK_Control));





				} //end of if MAX_WITHOUT_ZEROS > 20
			}// end of if percent_error > 49.0
	}// end of while percent_error > 0.1
	printf("fell out of while percent_error > 0.1 (hey, low error!) \n");



	spectacular_success = 0; //start with a fresh slate for success
	max_without_zeros = 0; //clear out any partial counts from above


		while(percent_error < 49.0){

			if(stop){
				break;
			}
                        printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
                        printf("less than 49.0 percent error here.\n");
			printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");


                        printf("reset the PRBS_BIT_COUNT and PRBS_ERROR_COUNT counters.\n");
                        printf("Toggle bit 2 of PRBS_CONTROL.\n");
                        WRITE_MSK(PRBS_Control, (READ_MSK(PRBS_Control)^0x00000004));
                        usleep(num_microseconds);
                        printf("We read PRBS_CONTROL: (0x%08x@%04x)\n", READ_MSK(PRBS_Control), OFFSET_MSK(PRBS_Control));
                        printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
                        printf("We read MSK_CONTROL: (0x%08x@%04x)\n", READ_MSK(MSK_Control), OFFSET_MSK(MSK_Control));
                        printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");


	                printf("100 buckets of bits on the bus, 100 buckets of bits.\n");

			for(zero_segments = 0; zero_segments < 10000; zero_segments++){
                        	usleep(one_bit_time);
				}

	                printf("(2) Total PRBS_BIT_COUNT:   (0x%08x@%04x)\n", READ_MSK(PRBS_Bit_Count), OFFSET_MSK(PRBS_Bit_Count));
	                printf("(2) Total PRBS_ERROR_COUNT: (0x%08x@%04x)\n", READ_MSK(PRBS_Error_Count), OFFSET_MSK(PRBS_Error_Count));
	                percent_error = ((double)(READ_MSK(PRBS_Error_Count))/(double)(READ_MSK(PRBS_Bit_Count)))*100;
                        printf("(2) %2.1f %d %d 0x%08x 0x%08x\n", percent_error, READ_MSK(LPF_Accum_F1), READ_MSK(LPF_Accum_F2),
								READ_MSK(LPF_Config_2), READ_MSK(LPF_Config_1));

			if (isnan(percent_error)) {
			printf("BOOM!\n");
			stop = true;
			}

			spectacular_success++;

			if(spectacular_success > 20) {
				spectacular_success = 0;
				printf("Paul, we had a good run.\n");
	                        printf("(3) %2.1f %d %d 0x%08x 0x%08x\n", percent_error, READ_MSK(LPF_Accum_F1), READ_MSK(LPF_Accum_F2), 
										READ_MSK(LPF_Config_2), READ_MSK(LPF_Config_1));
				break;
				}



			} //end of while percent_error < 49.0


		//time to test one or both of new PI gains
                printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
		printf("After spectacular success, we may increment gain pair here.\n");
		proportional_gain = proportional_gain + 1;
		//integral_gain = integral_gain + 1;

                int32_t proportional_config = (gain_bit_shift << 24) | (proportional_gain & 0x00FFFFFF);
                int32_t integral_config = (gain_bit_shift << 24) | (integral_gain & 0x00FFFFFF);
                printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
                printf("Write proportional and integral gains to LPF_CONFIG_2 and LPF_CONFIG_1.\n");
                printf("Proportional config: (0x%08x) integral config: (0x%08x)\n", proportional_config, integral_config);
                WRITE_MSK(LPF_Config_1, integral_config);
                WRITE_MSK(LPF_Config_2, proportional_config);

                printf("Assert RX INIT: Write 1 to bit 2 of MSK_INIT\n");
                WRITE_MSK(MSK_Init, 0x00000004);
                printf("Reading MSK_INIT. We see: (0x%08x@%04x)\n", READ_MSK(MSK_Init), OFFSET_MSK(MSK_Init));
                usleep(num_microseconds);
                printf("De-Assert INIT: Write 0 to MSK_INIT\n");
                WRITE_MSK(MSK_Init, 0x00000000);
                printf("Reading MSK_INIT. We see: (0x%08x@%04x)\n", READ_MSK(MSK_Init), OFFSET_MSK(MSK_Init));
                usleep(num_microseconds);



                printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
                printf("Zero the accumulators.\n");
                printf("Bit 1 of LPF_CONFIG_0 is set and then cleared.\n");
                printf("31:16 is the filter alpha.\n");
                printf("Write 0xXXXX0002 to LPF_CONFIG_0.\n");
                WRITE_MSK(LPF_Config_0, 0x00000002);
                usleep(num_microseconds);
                printf("We read LPF_CONFIG_0: (0x%08x@%04x)\n", READ_MSK(LPF_Config_0), OFFSET_MSK(LPF_Config_0));
                usleep(num_microseconds);
                WRITE_MSK(LPF_Config_0, 0x00000000);
                usleep(num_microseconds);
                printf("We read LPF_CONFIG_0: (0x%08x@%04x)\n", READ_MSK(LPF_Config_0), OFFSET_MSK(LPF_Config_0));




                printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
                printf("resync PRBS by toggling bit 3 of PRBS_CONTROL.\n");
                WRITE_MSK(PRBS_Control, (READ_MSK(PRBS_Control)^0x00000008));
                usleep(num_microseconds);
                printf("We read PRBS_CONTROL: (0x%08x@%04x)\n", READ_MSK(PRBS_Control), OFFSET_MSK(PRBS_Control));

                printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
                printf("reset the PRBS_BIT_COUNT and PRBS_ERROR_COUNT counters.\n");
                printf("Toggle bit 2 of PRBS_CONTROL.\n");
                WRITE_MSK(PRBS_Control, (READ_MSK(PRBS_Control)^0x00000004));
                usleep(num_microseconds);
                printf("We read PRBS_CONTROL: (0x%08x@%04x)\n", READ_MSK(PRBS_Control), OFFSET_MSK(PRBS_Control));
                printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
                printf("We read MSK_CONTROL: (0x%08x@%04x)\n", READ_MSK(MSK_Control), OFFSET_MSK(MSK_Control));

		//in the case that RX_INIT causes errors to begin to accumulate at this point then
		//percent_error needs to not skip loop 1. try setting it to 55.0 (default) here.
		percent_error = 55.0;


	#ifdef ENDLESS_PRBS
	}
	#endif









	int old_data = 0;
	int new_data = 0;



#ifdef STREAMING

	printf("* Starting IO streaming (press CTRL+C to cancel)\n");
	while (!stop)
	{
		ssize_t nbytes_rx, nbytes_tx;
		char *p_dat, *p_end;
		ptrdiff_t p_inc;

		// Schedule TX buffer
		nbytes_tx = iio_buffer_push(txbuf);

		// Use AXI stream transfer count register to see if data is getting to our logic
                old_data = new_data;
                new_data = msk_register_map->axis_xfer_count;
//		printf("AXIS_XFER_COUNT delta after iio_buffer_push is (0x%08x)\n", new_data - old_data);
//              printf("AXIS_XFER_COUNT after iio_buffer_push is (0x%08x)\n", new_data);


		if (nbytes_tx < 0) { printf("Error pushing buf %d\n", (int) nbytes_tx); shutdown(); }







		// Refill RX buffer
		nbytes_rx = iio_buffer_refill(rxbuf);
		if (nbytes_rx < 0) { printf("Error refilling buf %d\n",(int) nbytes_rx); shutdown(); }

		// READ: Get pointers to RX buf and read IQ from RX buf port 0
		p_inc = iio_buffer_step(rxbuf);
		p_end = iio_buffer_end(rxbuf);
		for (p_dat = (char *)iio_buffer_first(rxbuf, rx0_i); p_dat < p_end; p_dat += p_inc) {
			// Example: swap I and Q
			const int16_t i = ((int16_t*)p_dat)[0]; // Real (I)
			const int16_t q = ((int16_t*)p_dat)[1]; // Imag (Q)
			((int16_t*)p_dat)[0] = q;
			((int16_t*)p_dat)[1] = i;
		}








		// WRITE: Get pointers to TX buf and write IQ to TX buf port 0
		p_inc = iio_buffer_step(txbuf);
		p_end = iio_buffer_end(txbuf);
		for (p_dat = (char *)iio_buffer_first(txbuf, tx0_i); p_dat < p_end; p_dat += p_inc) {
			// Example: fill with zeros
			// 12-bit sample needs to be MSB aligned so shift by 4
			// https://wiki.analog.com/resources/eval/user-guides/ad-fmcomms2-ebz/software/basic_iq_datafiles#binary_format
			//((int16_t*)p_dat)[0] = 0x0800 << 4; // Real (I)
			//((int16_t*)p_dat)[1] = 0x0800 << 4; // Imag (Q)
                        ((int16_t*)p_dat)[0] = 0xAAAA; // Real (I)
                        ((int16_t*)p_dat)[1] = 0xAAAA; // Imag (Q)
		}

		// Sample counter increment and status output
		nrx += nbytes_rx / iio_device_get_sample_size(rx);
		ntx += nbytes_tx / iio_device_get_sample_size(tx);
		printf("\tRX %8.2f MSmp, TX %8.2f MSmp\n", nrx/1e6, ntx/1e6);
	}


#endif








	shutdown();

	return 0;
}
