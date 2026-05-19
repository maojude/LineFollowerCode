/**
 * @file LineFollower_PID_V2
 * @brief 8-Sensor IR Array, TB6612FNG, PID Control (-70 to +70 Scale)
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
#define LED PORTCbits.RC6  // Status LED

// ==========================================
// --- PID TUNING PARAMETERS ---
// ==========================================
// Max error is now +/- 70. 
// A Kp of 1.0 means at max error, adjustment is 70.
float Kp = 1.0;  
float Ki = 0.0;  // Keep at 0 for line followers
float Kd = 10.0; // Start with 10.0 for dampening

// Motor Constraints
int BASE_SPEED = 50; // Cruising speed (0 to 255)
int MAX_PWM = 255;   // Hardware limit

unsigned int BLACK_LINE_NOISE = 600; // ADC values below this are ignored

// PID State Variables
int last_error = 0;
long integral = 0;

// The new mathematical weights for the sensors
const int weights[8] = {-70, -50, -30, -10, 10, 30, 50, 70};

// --- Function Prototypes ---
void System_Init(void);
unsigned int ADC_Read(unsigned char channel);
void setMotors(signed char leftDir, unsigned char leftSpeed, signed char rightDir, unsigned char rightSpeed);

void System_Init(void)
{
    TRISA = 0b00101111;
    TRISB = 0b00000000;
    TRISC = 0b00000000;
    TRISD = 0b00000000;
    TRISE = 0b00000111;

    PORTA = 0x00;
    PORTB = 0x00;
    PORTC = 0x00;
    PORTD = 0x00;
    PORTE = 0x00;

    ADCON0 = 0x41;
    ADCON1 = 0x80;

    PR2 = 249;
    T2CON = 0b00000101;

    CCP1CON = 0x0C;
    CCP2CON = 0x0C;

    CCPR1L = 0;
    CCPR2L = 0;
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
    // Left Motor (AIN1/AIN2)
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

    // Right Motor (BIN1/BIN2)
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
    long sum_weights;
    long sum_vals;
    int error;
    unsigned int active_sensors;

    System_Init();
    __delay_ms(1000); // 1-second delay before motors start

    for (;;)
    {
        sum_weights = 0;
        sum_vals = 0;
        active_sensors = 0;
        error = 0;

        // 1. Read sensors using the new scale
        for (i = 0; i < 8; i++)
        {
            sensorValues[i] = ADC_Read(i);

            if (sensorValues[i] > BLACK_LINE_NOISE)
            {
                sum_weights += (long)sensorValues[i] * weights[i];
                sum_vals += sensorValues[i];
                active_sensors++;
            }
        }

        // 2. Calculate Error Directly
        if (active_sensors > 0)
        {
            error = (int)(sum_weights / sum_vals);
        }
        else
        {
            // Line Lost: Use the last known error direction to keep turning
            error = (last_error < 0) ? -70 : 70;
        }

        // 3. PID Calculations
        int derivative = error - last_error;
        integral += error;

        // Anti-Windup for Integral (Optional but good practice)
        if (integral > 10000)
            integral = 10000;
        if (integral < -10000)
            integral = -10000;

        // Calculate adjustment
        int adjustment = (int)((Kp * error) + (Ki * integral) + (Kd * derivative));
        
        // Update last_error for the NEXT loop's derivative calculation
        last_error = error; 

        // 4. Calculate Motor Speeds
        int leftSpeed = BASE_SPEED + adjustment;
        int rightSpeed = BASE_SPEED - adjustment;

        // 5. Motor Direction Logic & Clamping
        signed char leftDir = 1;
        signed char rightDir = 1;

        // Handle Left Motor Reversing
        if (leftSpeed < 0)
        {
            leftSpeed = -leftSpeed; // Make speed positive for PWM
            leftDir = -1;           // Set direction to reverse
        }
        if (leftSpeed > MAX_PWM)
            leftSpeed = MAX_PWM; // Clamp to 255

        // Handle Right Motor Reversing
        if (rightSpeed < 0)
        {
            rightSpeed = -rightSpeed;
            rightDir = -1;
        }
        if (rightSpeed > MAX_PWM)
            rightSpeed = MAX_PWM;

        // 6. Actuate Motors
        setMotors(leftDir, (unsigned char)leftSpeed, rightDir, (unsigned char)rightSpeed);
    }
}