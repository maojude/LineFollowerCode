/**
 * @file SensorTest
 * @brief Standalone sensor diagnostic for threshold calibration
 * @note Motors stay OFF. LED + PORTB show live sensor states.
 */

#include <xc.h>
#include <stdint.h>

#pragma config FOSC = XT
#pragma config WDTE = OFF
#pragma config PWRTE = ON
#pragma config BOREN = ON
#pragma config LVP = OFF
#pragma config CPD = OFF
#pragma config WRT = OFF
#pragma config CP = OFF

#define _XTAL_FREQ 4000000

#define LED PORTCbits.RC6

// ==========================================
// ADJUST THIS VALUE DURING TESTING
// ==========================================
unsigned int BLACK_LINE_NOISE = 600;

unsigned int ADC_Read(unsigned char channel)
{
    ADCON0 &= 0xC5;
    ADCON0 |= (channel << 3);
    __delay_us(20);
    GO_nDONE = 1;
    while (GO_nDONE)
        ;
    return ((ADRESH << 8) | ADRESL);
}

void main(void)
{
    unsigned int sensorValues[8];
    unsigned char i;
    unsigned char active_map;

    // Port directions
    TRISA = 0b00101111; // RA0-3, RA5 as analog inputs
    TRISB = 0b00000000; // PORTB all outputs (sensor display)
    TRISC = 0b00000000;
    TRISD = 0b00000000;
    TRISE = 0b00000111; // RE0-2 as analog inputs

    // Clear all ports
    PORTA = 0x00;
    PORTB = 0x00;
    PORTC = 0x00;
    PORTD = 0x00;
    PORTE = 0x00;

    // ADC setup
    ADCON0 = 0x41; // Fosc/8, channel 0, ADC ON
    ADCON1 = 0x80; // Right justified, all analog

    // No PWM, no motor setup needed

    for (;;)
    {
        active_map = 0;

        for (i = 0; i < 8; i++)
        {
            sensorValues[i] = ADC_Read(i);
            if (sensorValues[i] > BLACK_LINE_NOISE)
            {
                active_map |= (1 << i);
            }
        }

        // PORTB: each bit = one sensor (RB0=IR0 ... RB7=IR7)
        PORTB = active_map;

        // LED: ON if any sensor sees black
        LED = (active_map > 0) ? 1 : 0;

        __delay_ms(20);
    }
}