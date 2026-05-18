#include <xc.h>
#include <stdint.h>
#include <stdlib.h>

// PIC16F877A Configuration Bit Settings
#pragma config FOSC = HS, WDTE = OFF, PWRTE = ON, BOREN = ON, LVP = OFF, CPD = OFF, WRT = OFF, CP = OFF

#define _XTAL_FREQ 20000000

// Pin Definitions
#define L_MOTOR_DIR RC0
#define R_MOTOR_DIR RC4
#define LED RD2
#define BUTTON RB0

// --- PID Constants ---
float Kp = 1.8;
float Ki = 0.0;
float Kd = 30.0; // Increased to 30.0 to help stop post-curve oscillations

// Motor Constants
#define MAX_SPEED 145 // Slightly increased headroom
#define BASE_SPEED 85
#define CURVE_SPEED_MIN 30 // Increased to maintain torque during sharp pivot
#define RECOVERY_SPEED 80  // Faster search as requested
#define GAP_SPEED 45
#define GAP_TIMEOUT 150

// Calibration Data
uint16_t sensor_min[8], sensor_max[8], sensor_mid[8];

// PID & Safety Variables
float error = 0, last_error = 0, derivative = 0, filtered_derivative = 0;
uint16_t line_lost_timer = 0;

// Function Prototypes
void init_hardware(void);
uint16_t read_adc(uint8_t channel);
void set_motors(int16_t left_pwm, int16_t right_pwm);
void calibrate_sensors(void);

void main(void)
{
    for (uint8_t i = 0; i < 10; i++)
    {
        __delay_ms(100);
    }
    init_hardware();
    __delay_ms(1000);

    while (1)
    {
        set_motors(0, 0);
        line_lost_timer = 0;
        last_error = 0;
        filtered_derivative = 0;

        LED = 1;
        while (BUTTON == 0)
        {
        }
        __delay_ms(100);
        while (BUTTON == 1)
        {
        }
        __delay_ms(100);
        calibrate_sensors();

        LED = 1;
        while (BUTTON == 0)
        {
        }
        __delay_ms(100);
        while (BUTTON == 1)
        {
        }
        __delay_ms(100);
        LED = 0;

        while (1)
        {
            float sum_weights = 0;
            float sum_vals = 0;
            const int16_t weights[] = {-70, -50, -30, -10, 10, 30, 50, 70};

            for (uint8_t i = 0; i < 8; i++)
            {
                uint16_t raw_val = read_adc(i);
                int32_t range = (int32_t)sensor_max[i] - sensor_min[i];
                if (range < 100)
                    continue;

                int32_t normalized = ((int32_t)raw_val - sensor_min[i]) * 1000 / range;
                if (normalized < 0)
                    normalized = 0;
                if (normalized > 1000)
                    normalized = 1000;

                if (normalized > 200)
                {
                    sum_weights += (float)normalized * weights[i];
                    sum_vals += (float)normalized;
                }
            }

            if (sum_vals > 800)
            {
                LED = 1;
                line_lost_timer = 0;
                error = sum_weights / sum_vals;

                float abs_error = (error < 0) ? -error : error;

                // Deadband
                if (abs_error < 3.0f)
                    error = 0;

                // Responsive Filter
                derivative = error - last_error;
                filtered_derivative = (filtered_derivative * 0.3f) + (derivative * 0.7f);

                // --- Adaptive Strategy for Curves & Corners ---
                float kp_scale = 1.0f;
                float speed_drop_factor = 1.1f;

                if (abs_error < 15.0f)
                {
                    kp_scale = 0.5f; // Soft-center for straights
                }
                else if (abs_error > 35.0f)
                {
                    kp_scale = 1.7f;          // "Snap" gain for steep curves/90-degree corners
                    speed_drop_factor = 1.6f; // Aggressive braking to prevent slipping
                }

                float adjustment = (error * Kp * kp_scale) + (filtered_derivative * Kd);

                if (adjustment > 145.0f)
                    adjustment = 145.0f;
                if (adjustment < -145.0f)
                    adjustment = -145.0f;

                // Apply adaptive speed reduction
                int16_t dynamic_speed = BASE_SPEED - (int16_t)(abs_error * speed_drop_factor);
                if (dynamic_speed < CURVE_SPEED_MIN)
                    dynamic_speed = CURVE_SPEED_MIN;

                set_motors(dynamic_speed - (int16_t)adjustment, dynamic_speed + (int16_t)adjustment);
                last_error = error;
            }
            else
            {
                LED = 0;
                if (line_lost_timer == 0)
                {
                    set_motors(-30, -30);
                    __delay_ms(8);
                }

                line_lost_timer++;
                if (line_lost_timer > 600)
                {
                    set_motors(0, 0);
                    break;
                }

                if (line_lost_timer < GAP_TIMEOUT && ((last_error < 15.0f) && (last_error > -15.0f)))
                {
                    set_motors(GAP_SPEED, GAP_SPEED);
                }
                else
                {
                    if (last_error < -5.0f)
                        set_motors(RECOVERY_SPEED, -RECOVERY_SPEED);
                    else if (last_error > 5.0f)
                        set_motors(-RECOVERY_SPEED, RECOVERY_SPEED);
                    else
                        set_motors(GAP_SPEED, GAP_SPEED);
                }
            }
            __delay_us(200);
        }
    }
}

void init_hardware(void)
{
    PORTC = 0xFF;
    TRISA = 0xFF;
    TRISE = 0x07;
    TRISB = 0xC1;
    TRISC = 0x00;
    TRISD = 0x00;
    ADCON1 = 0x80;
    ADCON0 = 0x81;
    PR2 = 255;
    CCP1CON = 0x0C;
    CCP2CON = 0x0C;
    T2CON = 0x04;
    CCPR1L = 255;
    CCPR2L = 255;
}

void calibrate_sensors(void)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        sensor_min[i] = 1023;
        sensor_max[i] = 0;
    }
    for (uint16_t t = 0; t < 400; t++)
    {
        for (uint8_t i = 0; i < 8; i++)
        {
            uint16_t val = read_adc(i);
            if (val < sensor_min[i])
                sensor_min[i] = val;
            if (val > sensor_max[i])
                sensor_max[i] = val;
        }
        __delay_ms(10);
        if (t % 25 == 0)
            LED = !LED;
    }
    for (uint8_t i = 0; i < 8; i++)
    {
        sensor_mid[i] = (sensor_min[i] + sensor_max[i]) / 2;
    }
}

uint16_t read_adc(uint8_t channel)
{
    ADCON0 &= 0xC7;
    ADCON0 |= (channel << 3);
    __delay_us(20);
    GO_nDONE = 1;
    while (GO_nDONE)
        ;
    return (uint16_t)((ADRESH << 8) | ADRESL);
}

void set_motors(int16_t left_pwm, int16_t right_pwm)
{
    if (left_pwm >= 0)
    {
        L_MOTOR_DIR = 1;
        if (left_pwm > MAX_SPEED)
            left_pwm = MAX_SPEED;
        CCPR1L = (uint8_t)(255 - left_pwm);
    }
    else
    {
        L_MOTOR_DIR = 0;
        left_pwm = -left_pwm;
        if (left_pwm > MAX_SPEED)
            left_pwm = MAX_SPEED;
        CCPR1L = (uint8_t)left_pwm;
    }
    if (right_pwm >= 0)
    {
        R_MOTOR_DIR = 1;
        if (right_pwm > MAX_SPEED)
            right_pwm = MAX_SPEED;
        CCPR2L = (uint8_t)(255 - right_pwm);
    }
    else
    {
        R_MOTOR_DIR = 0;
        right_pwm = -right_pwm;
        if (right_pwm > MAX_SPEED)
            right_pwm = MAX_SPEED;
        CCPR2L = (uint8_t)right_pwm;
    }
}