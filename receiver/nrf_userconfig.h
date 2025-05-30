#ifndef _NRF_USERCONFIG_H
#define _NRF_USERCONFIG_H

/* CPU clock cycles for the specified amounts of time--accurate minimum delays
 * required for reliable operation of the nRF24L01+'s state machine.
 */
/* Settings for 1MHz MCLK.*/
#define DELAY_CYCLES_5MS       5000
#define DELAY_CYCLES_130US     130
#define DELAY_CYCLES_15US      15


/* SPI port--Select which USCI port we're using.
 * Applies only to USCI devices.  USI users can keep these
 * commented out.
 */
#define SPI_DRIVER_USCI_A 1


/* Operational pins -- IRQ, CE, CSN (SPI chip-select)
 */

/* IRQ */
#define nrfIRQport 2
#define nrfIRQpin BIT2

/* CSN SPI chip-select */
#define nrfCSNport 2
#define nrfCSNportout P2OUT
#define nrfCSNpin BIT4

/* CE Chip-Enable (used to put RF transceiver on-air for RX or TX) */
#define nrfCEport 2
#define nrfCEportout P2OUT
#define nrfCEpin BIT0

#endif
