#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "registers.h"

// Addresses of the DMACs via their AXI lite control interface register blocks
unsigned int *tx_dmac_register_map;
unsigned int *rx_dmac_register_map;

// Address of the MSK block via its AXI lite control interface register block
msk_top_regs_t *msk_register_map = NULL;

// Address of the peripherals block that contains the hardware timer
uint32_t *timer_register_map = NULL;


//read from a memory mapped register
unsigned int read_mapped_reg(unsigned int *virtual_addr, int offset)
{
		return virtual_addr[offset>>2];
}


// Set up memory-mapped register access
int init_register_access(void) {
    printf("Setting up for memory-mapped registers in the PL.\n");

	int ddr_memory = open("/dev/mem", O_RDWR | O_SYNC);
	// Memory map the address of the TX-DMAC via its AXI lite control interface register block
	tx_dmac_register_map = mmap(NULL, 65535, PROT_READ | PROT_WRITE, MAP_SHARED, ddr_memory, 0x7c420000);
	if (tx_dmac_register_map == MAP_FAILED) {
		printf("Failed to map TX-DMAC registers\n");
		close(ddr_memory);
		exit(1);
	}

    // Memory map the address of the RX-DMAC via its AXI lite control interface register block
	rx_dmac_register_map = mmap(NULL, 65535, PROT_READ | PROT_WRITE, MAP_SHARED, ddr_memory, 0x7c400000);
	if (rx_dmac_register_map == MAP_FAILED) {
		printf("Failed to map RX-DMAC registers\n");
		close(ddr_memory);
		exit(1);
	}

    // Memory map the address of the MSK block via its AXI lite control interface
	msk_register_map = mmap(NULL, 65535, PROT_READ | PROT_WRITE, MAP_SHARED, ddr_memory, 0x43c00000);
	if (msk_register_map == MAP_FAILED) {
		printf("Failed to map MSK registers\n");
		close(ddr_memory);
		exit(1);
	}

    // Memory map the address of the peripherals block so we can use the hardware timer
	timer_register_map = mmap(NULL, 65535, PROT_READ | PROT_WRITE, MAP_SHARED, ddr_memory, PERIPH_BASE);
	if (timer_register_map == MAP_FAILED) {
		printf("Failed to map timer registers\n");
		close(ddr_memory);
		exit(1);
	}

	unsigned int tx_description = read_mapped_reg(tx_dmac_register_map, DMAC_INTERFACE_DESCRIPTION_1);
	printf("TX DMAC Interface Description (0x%08x@0x%04x)\n", tx_description, DMAC_INTERFACE_DESCRIPTION_1);
	unsigned int rx_description = read_mapped_reg(rx_dmac_register_map, DMAC_INTERFACE_DESCRIPTION_1);
	printf("RX DMAC Interface Description (0x%08x@0x%04x)\n", rx_description, DMAC_INTERFACE_DESCRIPTION_1);

	printf("Writing to scratch register in TX-DMAC.\n");
	write_mapped_reg(tx_dmac_register_map, DMAC_SCRATCH, 0x5555AAAA);
	printf("Reading from scratch register in TX-DMAC. We see: (0x%08x@%04x)\n", read_mapped_reg(tx_dmac_register_map, DMAC_SCRATCH), DMAC_SCRATCH);
	printf("Reading the TX-DMAC peripheral ID: (0x%08x@%04x)\n", read_mapped_reg(tx_dmac_register_map, DMAC_PERIPHERAL_ID), DMAC_PERIPHERAL_ID);
	printf("Writing to scratch register in RX-DMAC.\n");
	write_mapped_reg(rx_dmac_register_map, DMAC_SCRATCH, 0x5555AAAA);
	printf("Reading from scratch register in RX-DMAC. We see: (0x%08x@%04x)\n", read_mapped_reg(rx_dmac_register_map, DMAC_SCRATCH), DMAC_SCRATCH);
	printf("Reading the RX-DMAC peripheral ID: (0x%08x@%04x)\n", read_mapped_reg(tx_dmac_register_map, DMAC_PERIPHERAL_ID), DMAC_PERIPHERAL_ID);

    return 0;
}

//write to a memory mapped register
unsigned int write_mapped_reg(unsigned int *virtual_addr, int offset, unsigned int value)
{
	virtual_addr[offset>>2] = value;
	return 0;
}


// Some registers require synchronization across clock domains.
// This is implemented by first writing (any value) and then reading the register.
// The current value is captured, triggered by the write, and can then be read safely.
// power = capture_and_read_msk(OFFSET_MSK(rx_power));
uint32_t capture_and_read_msk(size_t offset) {
	volatile uint32_t *reg_ptr;
	volatile uint32_t dummy_read __attribute__((unused));
	
	reg_ptr = (volatile uint32_t *)((char *)msk_register_map + offset);

	// write to capture
	*reg_ptr = 0xDEADBEEF;	// write a distinctive value in case it shows up in a read

	// Trusting that the CPU is slow enough to make this safe across clock domains
	// without any artificial delay here.
	// or maybe not!!!
	dummy_read = *reg_ptr;

	// read actual value
	return *reg_ptr;
}
