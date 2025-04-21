#include <msp430.h>

#include "msprf24.h"
#include "nrf_userconfig.h"
#include "stdint.h"

#define attempt_value 10
volatile unsigned int user;
unsigned int TXByte;
char buf[32];  // Use max buffer size
void setup_USCI_A1(void);
void main() {
    char addr[5] = "00005";

    WDTCTL = WDTHOLD | WDTPW;

    // Red, Green LED used for status
    P1DIR |= BIT0;
    P1OUT &= ~BIT0;
    P4DIR |= BIT7;
    P4OUT &= ~BIT7;

    /* Initial values for nRF24L01+ library config variables */

    rf_crc = RF24_EN_CRC | RF24_CRCO;  // CRC enabled, 16-bit
    rf_addr_width = 5;
    rf_speed_power = RF24_SPEED_2MBPS | RF24_POWER_0DBM;
    rf_channel = 120;

    msprf24_init();  // All RX pipes closed by default
    msprf24_set_pipe_packetsize(0, 32);
    msprf24_open_pipe(0, 1);  // Open pipe#0 with Enhanced ShockBurst enabled for receiving Auto-ACKs

    msprf24_standby();
    user = msprf24_current_state();
    w_tx_addr(addr);
    w_rx_addr(0, addr);  // Pipe 0 receives auto-ack's, autoacks are sent back to the TX addr so the PTX node
    setup_USCI_A1();
    while (1) {
        while (!(UCA1IFG & UCRXIFG));  // Check if data is available
        char received = UCA1RXBUF;     // Read received byte
        while (!(UCA1IFG & UCTXIFG));  // Wait for TX buffer ready
        UCA1TXBUF = received;          // Echo back received byte
        int attempts = attempt_value;
        buf[0] = received;

        w_tx_payload(32, buf);
        msprf24_activate_tx();
        LPM4;
        if (rf_irq & RF24_IRQ_FLAGGED) {
            msprf24_get_irq_reason();
            if (rf_irq & RF24_IRQ_TX) {
                P1OUT &= ~BIT0;  // Red LED off
                P4OUT |= BIT7;   // Green LED on, transmission successful.
            }
            if (rf_irq & RF24_IRQ_TXFAILED) {
                P4OUT &= ~BIT7;  // Green LED off
                P1OUT |= BIT0;   // Red LED on, transmission failed
            }

            msprf24_irq_clear(rf_irq);
            user = msprf24_get_last_retransmits();
        }
    }
}

/*
    Arguments: Nothing
    Returns: Nothing

    Sets up USCI_A1 for UART communication with PC
*/
void setup_USCI_A1(void) {
    P1OUT |= BIT0;          // Red LED on
    _delay_cycles(800000);  // LED used to confirm USCI_A1 is being setup
    P1OUT &= ~BIT0;
    UCA1CTL1 = UCSWRST;  // Recommended to place USCI in reset first
    P4SEL |= BIT4 + BIT5;
    UCA1CTL1 |= UCSSEL_2;  // Use SMCLK
    UCA1BR0 = 109;         // Set baud rate to 9600 with 1.048MHz clock (Data Sheet 36.3.13)
    UCA1BR1 = 0;           // Set baud rate to 9600 with 1.048MHz clock
    UCA1MCTL = UCBRS_2;    // Modulation UCBRSx = 2
    UCA1CTL1 &= ~UCSWRST;  // Initialize USCI state machine
}
