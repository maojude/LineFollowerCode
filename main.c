#include <xc.h>

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
#define LED PORTDbits.RC6  // Status LED (for testing)

#define THRESHOLD 500       // as if nga threshold separating black and white
#define BLACK_THRESHOLD 850 // pure black gyud o tumong kaayo sa black

void system_init()
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

    // Timer2 for PWM
    // For n20 motors, frequency is ~1kHz
    // PWM Freq = Fosc / (4 * Prescaler * (PR2 + 1)) Prescaler is 4
    // 1000 = 4000000 (4 * 4 (PR2 + 1))
    PR2 = 249;
    T2CON = 0x05;

    // Configuration CCP Modules for PWM Mode
    CCP1CON = 0x0C; // Left Motor PWM Mode
    CCP2CON = 0x0C; // Right Motor PWM Mode

    // Initialize duty cycle speeds (zero sa para stop ra)
    CCPR1L = 0;
    CCPR2L = 0;
}

unsigned int readADC(unsigned char channel)
{

    // Select channel in ADCON0 register
    if (channel > 7)
        return 0;
    // Clear old channel bits (5, 4, 3) while keeping clock bits (7, 6) untouched
    ADCON0 &= 0xC7;
    // Shift new channel into bits 5-3 and OR it into the register
    ADCON0 |= (unsigned char)(channel << 3);
    // Turn ADC module ON
    ADCON0 |= 0x01;

    __delay_us(20);
    GO = 1;

    while (GO)
        ;

    return (unsigned int)((ADRESH << 8) + ADRESL);
}

void setMotors(signed char leftDir, unsigned char leftSpeed, signed char rightDir, unsigned char rightSpeed)
{
    // direction 1 for forward, -1 for reverse, and 0 for stop

    // for left motor
    if (leftDir == 1)
    {
        AIN1 = 1; // clockwise (forward)
        AIN2 = 0;
    }
    else if (leftDir == -1)
    {
        AIN1 = 0; // counterclockwise (reverse)
        AIN2 = 1;
    }
    else
    {
        AIN1 = 0;
        AIN2 = 0;
    }

    // for right motor
    if (rightDir == 1)
    {
        BIN1 = 1; // clockwise (forward)
        BIN2 = 0;
    }
    else if (rightDir == -1)
    {
        BIN1 = 0; // counterclockwise (reverse)
        BIN2 = 1;
    }
    else
    {
        BIN1 = 0;
        BIN2 = 0;
    }

    // Apply speed to motors
    CCPR1L = (leftSpeed > 249) ? 249 : leftSpeed;   // kung lapas PR2
    CCPR2L = (rightSpeed > 249) ? 249 : rightSpeed; // kung lapas PR2
}

void main()
{
    system_init();
    __delay_ms(1000);

    unsigned int s0, s1, s2, s3, s4, s5, s6, s7; // for each sensor values

    while (1)
    {
        s0 = readADC(0); // Far Left  (RA0)
        s1 = readADC(1); // Mid-Left 1 (RA1)
        s2 = readADC(2); // Mid-Left 2 (RA2)
        s3 = readADC(3); // Center Left(RA3)
        s4 = readADC(4); // Center Right(RA5) Note: RA4 is usually open-drain/not AN4
        s5 = readADC(5); // Mid-Right 2 (RE0)
        s6 = readADC(6); // Mid-Right 1 (RE1)
        s7 = readADC(7); // Far Right   (RE2)

        // logic: kung mas dako sa threshold, black og mas gamay white
        if (s0 < THRESHOLD && s1 < THRESHOLD && s2 < THRESHOLD && s3 < THRESHOLD &&
            s4 < THRESHOLD && s5 < THRESHOLD && s6 < THRESHOLD && s7 < THRESHOLD)
        {
            setMotors(0, 0, 0, 0); // brake
            continue;
        }

        // Extreme Hard Drift Right (Line escaping left edge, only s0, s1, s2 hit)
        else if (s0 > THRESHOLD && s1 > THRESHOLD && s2 > THRESHOLD)
        {
            setMotors(1, 15, 1, 195); // Pivot aggressively on the left wheel
        }

        // Extreme Hard Drift Left (Line escaping right edge, only s5, s6, s7 hit)
        else if (s5 > THRESHOLD && s6 > THRESHOLD && s7 > THRESHOLD)
        {
            setMotors(1, 195, 1, 15); // Pivot aggressively on the right wheel
        }

        // Medium Drift Right (Line covering s0 to s3)
        else if (s0 > THRESHOLD && s1 > THRESHOLD && s2 > THRESHOLD && s3 > THRESHOLD)
        {
            setMotors(1, 40, 1, 160); // Sharp turn left
        }

        // Medium Drift Left (Line covering s4 to s7)
        else if (s4 > THRESHOLD && s5 > THRESHOLD && s6 > THRESHOLD && s7 > THRESHOLD)
        {
            setMotors(1, 160, 1, 40); // Sharp turn right
        }

        // Slight drift right (line ni liko pa wala)
        else if (s1 > THRESHOLD && s2 > THRESHOLD && s3 > THRESHOLD && s4 > THRESHOLD)
        {
            setMotors(1, 90, 1, 140); // pa liko wala
        }

        // Slight drift left (line ni liko pa tuo)
        else if (s3 > THRESHOLD && s4 > THRESHOLD && s5 > THRESHOLD && s6 > THRESHOLD)
        {
            setMotors(1, 140, 1, 90); // pa liko tuo
        }

        // Kung naa sa center ang black (assuming perfect center)
        else if (s3 > THRESHOLD && s4 > THRESHOLD && s5 > THRESHOLD)
        {
            setMotors(1, 130, 1, 130); // 130 hinay lng sa
        }

        // Fallback Catch (Handles partial or noisy overlapping states)
        else
        {
            // If center sensors still see something, creep forward slowly to find the way
            if (s3 > THRESHOLD || s4 > THRESHOLD)
            {
                setMotors(1, 100, 1, 100);
            }
            else
            {
                setMotors(0, 0, 0, 0); // Otherwise halt
            }
        }

        __delay_ms(4);
    }
}