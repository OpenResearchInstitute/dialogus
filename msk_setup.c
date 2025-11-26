

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>

#include "numerology.h"
#include "registers.h"

extern bool software_tx_processing;

// one bit time is 19 microseconds
float num_microseconds_between_writes_to_same_register = 5*20;


void msk_setup(void)
{
    printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
	printf("Configure MSK for minimum viable product test.\n");
	printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
	printf("Reading from MSK block HASH ID LOW: (0x%08x@%04x)\n", READ_MSK(Hash_ID_Low), OFFSET_MSK(Hash_ID_Low));
	printf("Reading from MSK block HASH ID HIGH: (0x%08x@%04x)\n", READ_MSK(Hash_ID_High), OFFSET_MSK(Hash_ID_High));
	printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");

	printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
	if (software_tx_processing) {
		printf("Configure the TX_SYNC_CTRL register to 0x%08x.\n", TX_SYNC_CTRL_DISABLE);
		WRITE_MSK(Tx_Sync_Ctrl, TX_SYNC_CTRL_DISABLE);
		printf("Reading TX_SYNC_CTRL. We see: (0x%08x@%04x)\n", READ_MSK(Tx_Sync_Ctrl), OFFSET_MSK(Tx_Sync_Ctrl));
	} else {
		printf("Configure the TX_SYNC_CTRL register to 0x%08x.\n", TX_SYNC_CTRL_AUTO);
		WRITE_MSK(Tx_Sync_Ctrl, TX_SYNC_CTRL_AUTO);
		printf("Reading TX_SYNC_CTRL. We see: (0x%08x@%04x)\n", READ_MSK(Tx_Sync_Ctrl), OFFSET_MSK(Tx_Sync_Ctrl));

		printf("Configure the TX_SYNC_CNT register to TX_SYNC_COUNT bit times.\n");
		WRITE_MSK(Tx_Sync_Cnt, TX_SYNC_COUNT);
		printf("Reading TX_SYNC_CNT. We see: (0x%08x@%04x)\n", READ_MSK(Tx_Sync_Cnt), OFFSET_MSK(Tx_Sync_Cnt));
	}
	printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");

	printf("Initial FIFOs before INIT: tx fifo: %08x rx fifo: %08x\n",
							capture_and_read_msk(OFFSET_MSK(tx_async_fifo_rd_wr_ptr)),
							capture_and_read_msk(OFFSET_MSK(rx_async_fifo_rd_wr_ptr)));

	printf("Initialize MSK block.\n");
	printf("Read MSK_INIT: (0x%08x@%04x)\n", READ_MSK(MSK_Init), OFFSET_MSK(MSK_Init));
	printf("bit 0: 0 is normal operation and 1 is initialize modem (reset condition).\n");
	printf("Assert INIT: Write 1 to MSK_INIT\n");
	WRITE_MSK(MSK_Init, 0x00000001);
	printf("Reading MSK_INIT. We see: (0x%08x@%04x)\n", READ_MSK(MSK_Init), OFFSET_MSK(MSK_Init));

	printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
	printf("OVP_FRAME_MODE: Configuring for frame-driven transmission\n");
	printf("PTT off, loopback off, waiting for OVP frames\n");
	WRITE_MSK(MSK_Control, 0x00000000);	// All control bits off
	printf("Reading back MSK_CONTROL status register. We see: (0x%08x@%04x)\n", READ_MSK(MSK_Control), OFFSET_MSK(MSK_Control));
	printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");

	printf("Reading the MSK_STATUS register, we see: (0x%08x@%04x)\n", READ_MSK(MSK_Status), OFFSET_MSK(MSK_Status));
	printf("Bit 0 is demod_sync(not implemented), bit 1 is tx_enable, bit 2 is rx_enable\n");
	printf("tx_enable is data to DAC enabled. rx_enable is data from ADC enable.\n");
	printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
	printf("TX_BIT_COUNT register is read, we see: (0x%08x@%04x)\n", capture_and_read_msk(OFFSET_MSK(Tx_Bit_Count)), OFFSET_MSK(Tx_Bit_Count));
	printf("This register reads out the count of data requests made by the modem.\n");
	printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
	printf("TX_ENABLE_COUNT register is read and write. It holds the number of clocks on which Tx Enable is active.\n");
	printf("First we read it, we see: (0x%08x@%04x)\n", capture_and_read_msk(OFFSET_MSK(Tx_Enable_Count)), OFFSET_MSK(Tx_Enable_Count));
	printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
	printf("Writing fb, f1, f2 (values are calculated for MSK TX).\n");

	double bitrate, freq_if, delta_f, f1, f2, br_fcw, f1_fcw_tx, f2_fcw_tx, f1_fcw_rx, f2_fcw_rx, tx_sample_rate, tx_rx_sample_ratio;
	double rx_sample_rate __attribute__((unused));

	bitrate = CHANNEL_BITRATE;
	freq_if = IF_FREQUENCY;
	tx_sample_rate = 61440000;
	tx_rx_sample_ratio = 25;								// Rx downsampling implemented in FPGA
	rx_sample_rate = tx_sample_rate / tx_rx_sample_ratio;	// Rx effective sample rate (not used)

	delta_f = bitrate/4;
	f1 = freq_if - delta_f;
	f2 = freq_if + delta_f;
	br_fcw = (bitrate/tx_sample_rate) * pow(2.0, 32.0);
	f1_fcw_tx = (f1/tx_sample_rate) * pow(2.0, 32.0);
	f2_fcw_tx = (f2/tx_sample_rate) * pow(2.0, 32.0);
	f1_fcw_rx = (f1/tx_sample_rate) * pow(2.0, 32.0);	// use non-downsampled rate; downsampling happens after the NCO
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

	WRITE_MSK(LPF_Config_0, 0x00000002);	//zero and hold accumulators
	printf("wrote 0x00000002 to LPF_Config_0: (0x%08x@%04x)\n", READ_MSK(LPF_Config_0), OFFSET_MSK(LPF_Config_0));
	usleep(num_microseconds_between_writes_to_same_register);
	WRITE_MSK(LPF_Config_0, 0x00000000);	//accumulators in normal operation
	printf("Wrote all 0 LPF_Config_0: (0x%08x@%04x)\n", READ_MSK(LPF_Config_0), OFFSET_MSK(LPF_Config_0));

	printf("Write some default values for PI gain and bit shift.\n");
	WRITE_MSK(LPF_Config_1, 0x005a5a5a);
	WRITE_MSK(LPF_Config_2, 0x00a5a5a5);
	printf("LPF_Config_0: (0x%08x@%04x)\n", READ_MSK(LPF_Config_0), OFFSET_MSK(LPF_Config_0));
	printf("LPF_Config_1: (0x%08x@%04x)\n", READ_MSK(LPF_Config_1), OFFSET_MSK(LPF_Config_1));
	printf("LPF_Config_2: (0x%08x@%04x)\n", READ_MSK(LPF_Config_2), OFFSET_MSK(LPF_Config_2));

	printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
	printf("Set TX_DATA_WIDTH to 8.\n");
	WRITE_MSK(Tx_Data_Width, 0x00000008);

	printf("Read TX_DATA_WIDTH.\n");
	printf("We see: (0x%08x@%04x)\n", READ_MSK(Tx_Data_Width), OFFSET_MSK(Tx_Data_Width));

	printf("Set RX_DATA_WIDTH to 8.\n");
	WRITE_MSK(Rx_Data_Width, 0x00000008);
	printf("Read RX_DATA_WIDTH.\n");
	printf("We see: (0x%08x@%04x)\n", READ_MSK(Rx_Data_Width), OFFSET_MSK(Rx_Data_Width));

	printf("Initialize PRBS_CONTROL to zero. PRBS inactive (bit 0)\n");
	WRITE_MSK(PRBS_Control, 0x00000000);
	printf("We read PRBS_CONTROL: (0x%08x@%04x)\n", READ_MSK(PRBS_Control), OFFSET_MSK(PRBS_Control));

	//initial values of parameterized LPF_CONFIG are set up here
/*
	int32_t proportional_gain =           0x00904073;	// from matlab model 
	int32_t integral_gain =               0x0002D3AD;	//
	int32_t proportional_gain_bit_shift = 0x0000001C;	//
	int32_t integral_gain_bit_shift =     0x00000020;	//
*/

	int32_t proportional_gain =           0x007FFFFF;	// 0x00000243; //0x0012984F for 32 bits 0x00001298 for 24 bits 243 for OE 
	int32_t integral_gain =          	  0x007FFFFF;	// 0x000005A7; //0x0000C067 for 32 bits and 80 for 0E
	int32_t proportional_gain_bit_shift = 18;	//0x0000000E; //0x18 is 24 and 0x20 is 32 and 0E is 14
	int32_t integral_gain_bit_shift =     27;	//0x00000019; //0x18 is 24 and 0x20 is 32 and 0E is 14

	int32_t proportional_config = (proportional_gain_bit_shift << 24) | (proportional_gain & 0x00FFFFFF);
	int32_t integral_config = (integral_gain_bit_shift << 24) | (integral_gain & 0x00FFFFFF);
	printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
	printf("Write proportional and integral gains to LPF_CONFIG_2 and LPF_CONFIG_1.\n");
	printf("Proportional config: (0x%08x) integral config: (0x%08x)\n", proportional_config, integral_config);
	WRITE_MSK(LPF_Config_1, integral_config);
	WRITE_MSK(LPF_Config_2, proportional_config);

	//test xfer register reads
	printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
	printf("The value of axis_xfer_count is: (0x%08x@%04x)\n", capture_and_read_msk(OFFSET_MSK(axis_xfer_count)), OFFSET_MSK(axis_xfer_count));

	//discard 24 receiver samples and 24 NCO Samples
	WRITE_MSK(Rx_Sample_Discard, 0x00001818);
	printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
	printf("Read RX_SAMPLE_DISCARD: (0x%08x@%04x)\n", READ_MSK(Rx_Sample_Discard), OFFSET_MSK(Rx_Sample_Discard));
	printf("bits 0:7 are receiver sample discard and bits 15:8 are NCO sample discard.\n");

	printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
	printf("Read NCO Telemetry:\n");
	printf("f1_nco_adjust: (0x%08x) f2_nco_adjust: (0x%08x)\n", capture_and_read_msk(OFFSET_MSK(f1_nco_adjust)), capture_and_read_msk(OFFSET_MSK(f2_nco_adjust)));
	printf("f1_error:      (0x%08x) f2_error:      (0x%08x)\n", capture_and_read_msk(OFFSET_MSK(f1_error)), capture_and_read_msk(OFFSET_MSK(f2_error)));

	printf("FIFOs during INIT: tx fifo: %08x rx fifo: %08x\n",
						capture_and_read_msk(OFFSET_MSK(tx_async_fifo_rd_wr_ptr)),
						capture_and_read_msk(OFFSET_MSK(rx_async_fifo_rd_wr_ptr)));


	printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
	printf("Deassert overall INIT, assert tx INIT: Write 2 to MSK_INIT\n");
	WRITE_MSK(MSK_Init, 0x00000002);
	printf("Read MSK_INIT: (0x%08x@%04x)\n", READ_MSK(MSK_Init), OFFSET_MSK(MSK_Init));

	printf("Initial FIFOs at INIT: tx fifo: %08x rx fifo: %08x\n",
							capture_and_read_msk(OFFSET_MSK(tx_async_fifo_rd_wr_ptr)),
							capture_and_read_msk(OFFSET_MSK(rx_async_fifo_rd_wr_ptr)));

}