#include <msp430.h>

#include "intrinsics.h"
#include "msp430f5529.h"
#include "msprf24.h"
#include "nrf_userconfig.h"
#include "stdint.h"

#define TXD BIT4                        // P4.4 - UART Transmitter
#define RXD BIT5                        // P4.5 - UART Receiver
#define right_power 72                  // Chosen by experimentation
#define left_power 35                   // Chosen by experimentation
#define center_power 53                 // Chosen by experimentation
#define PWM_clock 655                   // PWM period 20ms
#define thrust_reduction 50             // Percentage thrust should decrease for differential thrust
#define slow_power (PWM_clock / 2)      // 75% RPM on DG01D
#define fast_power (2 * PWM_clock) / 3  // 100% RPM on DG01D

volatile unsigned int user;
void init_PWM(void);
void stop(void);
void rudder_left(void);
void rudder_centre(void);
void rudder_right(void);
void increase_speed(void);
void decrease_speed(void);
void reverse(void);
void differential_thrust(void);
int power_counter = 0;  // Which power stage are we at
int reverse_flag = 0;
void main() {
    char addr[5] = "00005";
    char buf[32];

    WDTCTL = WDTHOLD | WDTPW;

    // Red LED will be our output
    P1DIR |= BIT0;
    P4DIR |= BIT7;
    P1OUT &= ~BIT0;
    P1OUT &= ~BIT7;
    P2DIR |= BIT3;
    // P2OUT &= ~BIT3;
    P2OUT |= BIT3;  //
    P6DIR |= BIT2 + BIT3;

    user = 0xFE;

    /* Initial values for nRF24L01+ library config variables */
    rf_crc = RF24_EN_CRC | RF24_CRCO;  // CRC enabled, 16-bit
    rf_addr_width = 5;
    rf_speed_power = RF24_SPEED_2MBPS | RF24_POWER_0DBM;
    rf_channel = 120;

    msprf24_init();
    msprf24_set_pipe_packetsize(0, 32);
    msprf24_open_pipe(0, 1);  // Open pipe#0 with Enhanced ShockBurst

    // Set our RX address
    // addr[0] = 0xDE; addr[1] = 0xAD; addr[2] = 0xBE; addr[3] = 0xEF; addr[4] = 0x00;
    w_rx_addr(0, addr);

    /*****************
     * CONFIGURE UART *
     *****************/
    /*
    P4DIR |=  TXD;
    P4DIR &= ~RXD;

    UCA1CTL1 |= UCSWRST;    // Put USCI_A1 in reset to configure

    P4SEL |=  TXD;
    P4SEL |=  RXD;

    UCA1CTL1 |= UCSSEL_2;   // Use SMCLK for UART baud generator

    UCA1BR0 = 109;          // Set baud rate
    UCA1BR1 = 0;
    UCA1MCTL |= UCBRS_2;    // Set modulation for UART

    UCA1CTL1 &= ~UCSWRST;   // Release USCI_A1 from rest
    */
    /*****************
     * CONFIGURE UART *
     *****************/
    init_PWM();
    // Receive mode
    if (!(RF24_QUEUE_RXEMPTY & msprf24_queue_state())) {
        flush_rx();
    }
    msprf24_activate_rx();
    LPM0;
    /*
        Main loop. The reciever will wait until the NRF24L01+ ISR is flagged, and will take the
        first byte from the buffer as command to initiate desired command
    */
    while (1) {
        if (rf_irq & RF24_IRQ_FLAGGED) {
            msprf24_get_irq_reason();
        }
        if (rf_irq & RF24_IRQ_RX) {
            r_rx_payload(32, buf);  // Receive payload
            msprf24_irq_clear(RF24_IRQ_RX);
            user = buf[0];
            /*
                Each of these correspond with input from transmitter and therefore PC
            */
            if (buf[0] == 'w') {
                increase_speed();
            } else if (buf[0] == 'a') {
                rudder_left();
            } else if (buf[0] == 's') {
                decrease_speed();
            } else if (buf[0] == 'd') {
                rudder_right();
            } else if (buf[0] == 'c') {
                rudder_centre();
            } else if (buf[0] == 'r') {
                reverse();
            } else if (buf[0] == 'z') {
                stop();
            }
            differential_thrust();  // For simplicity, this is triggered after all other commands
        } else {
            user = 0xFF;
        }
        LPM0;
    }
}
/*
 Arguments: Nothing
 Returns: Nothing

 Initiates the PWMs for the motors on PIN1, ie TA0CCR 0,1,2,3.
*/
void init_PWM(void) {
    TA0CCR0 = 655;        // PWM period 20ms
    TA0CCTL1 = OUTMOD_7;  // TA0CCR1 reset/set-high voltage
    TA0CCR1 = center_power;
    P1DIR |= BIT2;         // Output on Pin 1.2
    P1SEL |= BIT2;         // Pin 1.2 selected as PWM
    P1DIR |= BIT3 + BIT4;  // Output on Pin 1.3
    P1SEL |= BIT3 + BIT4;  // Pin 1.3 selected as PWM

    P6OUT |= BIT2;  // Engines increase_speed
    TA0CCR2 = 0;    // Left Engine PWM
    TA0CCR3 = 0;    // Right Engine PWM
    TA0CCTL2 = OUTMOD_7;
    TA0CCTL3 = OUTMOD_7;
    TA0CTL = TASSEL_1 | MC_1 | ID_0;  // TASSEL_2 + MC_1 + TAIE +ID_0;
}

/*
    Arguments: Nothing
    Returns: Nothing

    Decreases TA0CCR1. If TA0CCR1 is right\_power, go to centre power. Else left\_power.
*/
void rudder_left(void) {
    if (TA0CCR1 == right_power) {
        TA0CTL &= ~MC_1;  // Stop the timer
        TA0CCR1 = center_power;
        TA0CTL |= MC_1;  // Restart timer in Up mode
    } else {
        TA0CTL &= ~MC_1;  // Stop the timer
        TA0CCR1 = left_power;
        TA0CTL |= MC_1;  // Restart timer in Up mode
    }
}

/*
    Arguments: Nothing
    Returns: Nothing

    Force TA0CCR1 to center\_power. More of a debug command.
*/
void rudder_centre(void) {
    TA0CTL &= ~MC_1;  // Stop the timer
    TA0CCR1 = center_power;
    TA0CTL |= MC_1;  // Restart timer in Up mode
}

/*
    Arguments: Nothing
    Returns: Nothing

    Increases TA0CCR1. If TA0CCR1 is left\_power, go to centre power. Else right\_power.
*/
void rudder_right(void) {
    if (TA0CCR1 == left_power) {
        TA0CTL &= ~MC_1;  // Stop the timer
        TA0CCR1 = center_power;
        TA0CTL |= MC_1;  // Restart timer in Up mode
    } else {
        TA0CTL &= ~MC_1;  // Stop the timer
        TA0CCR1 = right_power;
        TA0CTL |= MC_1;  // Restart timer in Up mode
    }
}
/*
    Arguments: Nothing
    Returns: Nothing

    Increases the duty cycle for both TA0CCR2 and TA0CCR3. This causes the DG10D motors to run faster increasing speed.
    There are two speed modes available: slow_power and fast_power.
*/
void increase_speed(void) {
    if ((TA0CCR2 == 0) | (TA0CCR3 == 0)) {
        TA0CTL &= ~MC_1;
        TA0CCR2 = slow_power;
        TA0CCR3 = slow_power;
        TA0CTL |= MC_1;
        power_counter = 1;
    } else {
        TA0CTL &= ~MC_1;
        TA0CCR2 = fast_power;
        TA0CCR3 = fast_power;
        TA0CTL |= MC_1;
        power_counter = 2;
    }
}
/*
    Arguments: Nothing
    Returns: Nothing

    Decreases the duty cycle for both TA0CCR2 and TA0CCR3. If at fast\_power, go to slow\_power. If at slow\_power, go to 0 duty cycle.
*/
void decrease_speed(void) {
    if ((TA0CCR2 == fast_power) | (TA0CCR3 == fast_power)) {
        TA0CTL &= ~MC_1;
        TA0CCR2 = slow_power;
        TA0CCR3 = slow_power;
        TA0CTL |= MC_1;
        power_counter = 1;
    } else {
        stop();
    }
}
/*
    Arguments: Nothing
    Returns: Nothing

    Reverses DG10D polarity. Refer to Pin connections with L298N.
*/
void reverse(void) {
    if (TA0CCR2 == 0) {
        P6OUT ^= BIT2;
        P6OUT ^= BIT3;
        reverse_flag ^= 1;
    }
}

/*
    Arguments: Nothing
    Returns: Nothing

    Forces TA0CCR2 and TA0CCR3 to 0.
*/
void stop(void) {
    TA0CTL &= ~MC_1;
    TA0CCR2 = 0;
    TA0CCR3 = 0;
    TA0CTL |= MC_1;
    power_counter = 0;
}

/*
    Arguments: Nothing
    Returns: Nothing

    A function to adjust  TA0CCR2 and TA0CCR3 to aid with turning.

    If the boat is going straight, do nothing.

    If the boat is going forward and turning left, reduce TA0CCR2 by thrust_reduction
    If the boat is going forward and turning right, reduce TA0CCR1 by thrust_reduction

    If the boat is going backward and turning left, reduce TA0CCR1 by thrust_reduction
    If the boat is going forward and turning right, reduce TA0CCR2 by thrust_reduction
*/
void differential_thrust(void) {
    int engine_power;
    switch (power_counter) {
        case 0:
            engine_power = 0;
            break;
        case 1:
            engine_power = slow_power;
            break;
        case 2:
            engine_power = fast_power;
            break;
        default:
            engine_power = 0;  // should never happen
            break;
    }
    if (TA0CCR1 == left_power) {
        if (reverse_flag == 0) {
            TA0CTL &= ~MC_1;
            TA0CCR2 = engine_power - ((thrust_reduction * engine_power) / 100);
            TA0CCR3 = engine_power;
            TA0CTL |= MC_1;
        } else {
            TA0CTL &= ~MC_1;
            TA0CCR3 = engine_power - ((thrust_reduction * engine_power) / 100);
            TA0CCR2 = engine_power;
            TA0CTL |= MC_1;
        }
    } else if (TA0CCR1 == right_power) {
        if (reverse_flag == 0) {
            TA0CTL &= ~MC_1;
            TA0CCR3 = engine_power - ((thrust_reduction * engine_power) / 100);
            TA0CCR2 = engine_power;
            TA0CTL |= MC_1;
        } else {
            TA0CTL &= ~MC_1;
            TA0CCR2 = engine_power - ((thrust_reduction * engine_power) / 100);
            TA0CCR3 = engine_power;
            TA0CTL |= MC_1;
        }
    } else {
        TA0CTL &= ~MC_1;
        TA0CCR2 = engine_power;
        TA0CCR3 = engine_power;
        TA0CTL |= MC_1;
    }
}
