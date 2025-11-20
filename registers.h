#ifndef REGISTERS_H
#define REGISTERS_H


#include <stddef.h>
#include <sys/mman.h>

// For the MSK registers, we use RDL headers
#include "msk_top_regs.h"

// Offsets into either DMAC (TX or RX) for registers in that block.
// All the registers are defined in /hdl/library/axi_dmac/index.html
#define DMAC_PERIPHERAL_ID 0x0004
#define DMAC_SCRATCH 0x0008
#define DMAC_INTERFACE_DESCRIPTION_1 0x0010
#define DMAC_IRQ_MASK 0x0080
#define DMAC_IRQ_PENDING 0x0084
#define DMAC_IRQ_SOURCE 0x0088
#define DMAC_FLAGS 0x040c


/* Register addresses for using the hardware timer */
#define PERIPH_BASE 0xf8f00000
#define GLOBAL_TMR_UPPER_OFFSET 0x0204
#define GLOBAL_TMR_LOWER_OFFSET 0x0200
/* Global Timer runs on the CPU clock, divided by 2 */
#define COUNTS_PER_SECOND (666666687 / 2 / 2)
/* Collect telementry and make decisions after this duration */
#define REPORTING_INTERVAL (COUNTS_PER_SECOND / 1000)


// Addresses of the DMACs via their AXI lite control interface register blocks
extern unsigned int *tx_dmac_register_map;
extern unsigned int *rx_dmac_register_map;

// Address of the MSK block via its AXI lite control interface register block
extern msk_top_regs_t *msk_register_map;

// Address of the peripherals block that contains the hardware timer
extern uint32_t *timer_register_map;

int init_register_access(void);
unsigned int read_mapped_reg(unsigned int *virtual_addr, int offset);
unsigned int write_mapped_reg(unsigned int *virtual_addr, int offset, unsigned int value);
uint32_t capture_and_read_msk(size_t offset);

//read a value from the MSK register
//value = READ_MSK(MSK_Init);
#define READ_MSK(offset) (msk_register_map->offset)

//from devmem in sbin, we know: read_result = *(volatile uint32_t*)virt_addr;
//write a value to an MSK register
//WRITE_MSK(MSK_Init, 0x00000001);
#define WRITE_MSK(offset, value) *(volatile uint32_t*)&(msk_register_map->offset) = value

//get the address offset of an MSK register
//value = OFFSET_MSK(MSK_Init);
#define OFFSET_MSK(offset) (offsetof(msk_top_regs_t, offset))


// Values for Tx_Sync_Ctrl register
#define TX_SYNC_CTRL_DISABLE 0
#define TX_SYNC_CTRL_AUTO   0x0000000d  // enabled when transmitter turns on, both tone frequencies
#define TX_SYNC_CTRL_FORCE  0x0000000e  // enabled now, both tone frequencies


#endif // REGISTERS_H