/**
 * @file LineFollower_Hardcoded_
 * @brief 8-Sensor IR Array, TB6612FNG, No Calibration
 */

#include <xc.h>
#include <stdint.h>

// --- Configuration Bits ---
#pragma config FOSC = XT
#pragma config WDTE = OFF
#pragma config PWRTE = ON
#pragma config BOREN = ON
#pragma config LVP = OFF
#pragma config CPD = OFF
#pragma config WRT = OFF
#pragma config CP = OFF

#define _XTAL_FREQ 4000000

// --- Hardware Pin Mapping ---
#define AIN1 PORTCbits.RC0 // Left Motor Direction 1
#define AIN2 PORTCbits.RC3 // Left Motor Direction 2
#define BIN1 PORTDbits.RD0 // Right Motor Direction 1
#define BIN2 PORTDbits.RD1 // Right Motor Direction 2
#define LED PORTCbits.RC6  // Status LED (for testing)

// ==========================================
// --- GLOBAL TUNING PARAMETERS ---
// Adjust these values to tune your robot!
// ==========================================

// Motor Speeds (0 to 255)
unsigned char BASE_SPEED = 50;       // Speed when going straight
unsigned char TURN_SPEED_OUTER = 35; // Speed of the outside wheel when turning
unsigned char TURN_SPEED_INNER = 20; // Speed of the inside wheel when turning

// Position Thresholds (0 to 7000)
unsigned int THRESHOLD_LEFT = 3000;  // If position is below this, turn left
unsigned int THRESHOLD_RIGHT = 4000; // If position is above this, turn right
unsigned int BLACK_LINE_NOISE = 600; // ADC values below this are ignored

// ==========================================

// --- Function Prototypes ---
void System_Init(void);
unsigned int ADC_Read(unsigned char channel);
void setMotors(signed char leftDir, unsigned char leftSpeed, signed char rightDir, unsigned char rightSpeed);

void System_Init(void)
{
    // ==========================================
    // 1. I/O Port Direction Configuration (TRIS)
    // 1 = Input (Sensors), 0 = Output (Motors, LEDs, Unused pins)
    // ==========================================

    TRISA = 0b00101111; // RA0-RA3, RA5 as Inputs (IR0-IR4). Rest as Outputs.
    TRISB = 0b00000000; // All PORTB pins as Outputs (Unused, safely secured).
    TRISC = 0b00000000; // All PORTC pins as Outputs (AIN1, AIN2, PWMA, PWMB).
    TRISD = 0b00000000; // All PORTD pins as Outputs (BIN1, BIN2, LED).
    TRISE = 0b00000111; // RE0-RE2 as Inputs (IR5-IR7). Rest as Outputs.

    // ==========================================
    // 2. Initial Pin States (PORT)
    // 1 = HIGH (5V), 0 = LOW (0V)
    // ==========================================

    PORTA = 0x00; // Ensure any unused output pins start LOW
    PORTB = 0x00; // Ensure any unused output pins start LOW
    PORTC = 0x00; // Motors stopped immediately on startup
    PORTD = 0x00; // Motors stopped, LED off
    PORTE = 0x00; // Ensure any unused output pins start LOW

    // ==========================================
    // 3. ADC Channels (Analog to Digital Converter)
    // ==========================================

    // ADCON0 controls the ADC operation.
    // 0x41 (01000001): ADC Clock = Fosc/8, ADON = 1
    ADCON0 = 0x41;

    // ADCON1 controls pin configuration and result formatting.
    // 0x80 (10000000): ADFM = 1 (Right-justified), Pins to Analog mode.
    ADCON1 = 0x80;

    // ==========================================
    // 4. PWM Timer2 (Pulse Width Modulation for Motors)
    // ==========================================

    PR2 = 249;          // Set PWM frequency to ~1 kHz
    T2CON = 0b00000101; // Timer2 ON, Prescaler 1:4

    CCP1CON = 0x0C; // Configure CCP1 (RC2 / PWMA) for PWM mode
    CCP2CON = 0x0C; // Configure CCP2 (RC1 / PWMB) for PWM mode

    CCPR1L = 0; // Start PWMA at 0% speed
    CCPR2L = 0; // Start PWMB at 0% speed
}

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

void setMotors(signed char leftDir, unsigned char leftSpeed, signed char rightDir, unsigned char rightSpeed)
{
    if (leftDir == 1)
    {
        AIN1 = 1;
        AIN2 = 0;
    }
    else if (leftDir == -1)
    {
        AIN1 = 0;
        AIN2 = 1;
    }
    else
    {
        AIN1 = 0;
        AIN2 = 0;
    }

    if (rightDir == 1)
    {
        BIN1 = 1;
        BIN2 = 0;
    }
    else if (rightDir == -1)
    {
        BIN1 = 0;
        BIN2 = 1;
    }
    else
    {
        BIN1 = 0;
        BIN2 = 0;
    }

    CCPR1L = leftSpeed;
    CCPR2L = rightSpeed;
}

void main(void)
{
    unsigned int sensorValues[8];
    unsigned char i;
    unsigned long weighted_sum;
    unsigned long sum;
    unsigned int position;
    unsigned int active_sensors;

    System_Init();

    __delay_ms(1000);

    for (;;)
    {
        weighted_sum = 0;
        sum = 0;
        active_sensors = 0;

        // 1. Read sensors
        for (i = 0; i < 8; i++)
        {
            sensorValues[i] = ADC_Read(i);

            // Using the global noise threshold
            if (sensorValues[i] > BLACK_LINE_NOISE)
            {
                weighted_sum += (unsigned long)sensorValues[i] * i * 1000;
                sum += sensorValues[i];
                active_sensors++;
            }
        }

        // 2. Calculate Position (0 to 7000)
        if (active_sensors > 0)
        {
            position = (unsigned int)(weighted_sum / sum);
        }
        else
        {
            position = 3500;
        }

        // 3. Proportional Motor Logic using Globals
        if (position < THRESHOLD_LEFT)
        {
            // Turn Left
            setMotors(-1, TURN_SPEED_INNER, 1, TURN_SPEED_OUTER);
        }
        else if (position > THRESHOLD_RIGHT)
        {
            // Turn Right
            setMotors(1, TURN_SPEED_OUTER, -1, TURN_SPEED_INNER);
        }
        else
        {
            // Go Straight
            setMotors(1, BASE_SPEED, 1, BASE_SPEED);
        }
    }
}