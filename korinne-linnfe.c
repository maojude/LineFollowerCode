/**
 * @file LineFollower_Hardcoded_UART.c
 * @brief 8-Sensor IR Array, TB6612FNG, STBY Pin, No Calibration, Throttled UART
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
#define AIN2 PORTDbits.RD0      
#define AIN1 PORTDbits.RD1      
#define BIN2 PORTDbits.RD2      
#define BIN1 PORTDbits.RD3      
#define STBY PORTDbits.RD4      
#define LED  PORTBbits.RB0      

// ==========================================
// --- GLOBAL TUNING PARAMETERS ---
// Adjust these values to tune your robot!
// ==========================================

// Motor Speeds (0 to 255)
unsigned char BASE_SPEED = 50;       // Speed when going straight
unsigned char TURN_SPEED_OUTER = 35; // Speed of the outside wheel when turning
unsigned char TURN_SPEED_INNER = 20; // Speed of the inside wheel when turning

// Position Thresholds (0 to 7000)
unsigned int THRESHOLD_LEFT = 3000;   // If position is below this, turn left
unsigned int THRESHOLD_RIGHT = 4000;  // If position is above this, turn right
unsigned int BLACK_LINE_NOISE = 600;  // ADC values below this are ignored

// ==========================================

// --- Function Prototypes ---
void System_Init(void);
unsigned int ADC_Read(unsigned char channel);
void UART_Tx_Char(char data);
void UART_Tx_String(const char *str);
void UART_Tx_Number(unsigned int num);
void setMotors(signed char leftDir, unsigned char leftSpeed, signed char rightDir, unsigned char rightSpeed);

void System_Init(void) {
    // 1. Motor & LED I/O
    TRISCbits.TRISC1 = 0;       // RC1 (CCP2) Output for PWMB
    TRISCbits.TRISC2 = 0;       // RC2 (CCP1) Output for PWMA
    TRISD &= 0xE0;              // RD0-RD4 as Outputs
    TRISBbits.TRISB0 = 0;       // RB0 as Output for LED
    
    PORTD &= 0xE0;              // Motors LOW
    LED = 0;                    
    
    STBY = 1;                   // Set STBY to 1 to enable motors

    // 2. ADC Channels
    TRISA |= 0x2F;              
    TRISE |= 0x07;              
    
    ADCON0 = 0x41;              
    ADCON1 = 0x80;              

    // 3. PWM Timer2
    PR2 = 249;                  
    T2CON = 0b00000101;         
    CCP1CON = 0x0C;             
    CCP2CON = 0x0C;             
    CCPR1L = 0;                 
    CCPR2L = 0;                 

    // 4. UART Configuration
    TRISCbits.TRISC6 = 0;       
    TRISCbits.TRISC7 = 1;       
    SPBRG = 25;                 
    TXSTA = 0x24;               
    RCSTA = 0x90;               
}

unsigned int ADC_Read(unsigned char channel) {
    ADCON0 &= 0xC5; 
    ADCON0 |= (channel << 3); 
    __delay_us(20);             
    GO_nDONE = 1;               
    while(GO_nDONE);            
    return ((ADRESH << 8) | ADRESL); 
}

void UART_Tx_Char(char data) {
    while(!TRMT);
    TXREG = data;
}

void UART_Tx_String(const char *str) {
    while(*str) {
        UART_Tx_Char(*str++);
    }
}

void UART_Tx_Number(unsigned int num) {
    char buffer[6]; 
    int i = 5;
    buffer[--i] = '\0'; 

    if (num == 0) {
        buffer[--i] = '0';
    } else {
        while (num > 0 && i > 0) {
            buffer[--i] = (num % 10) + '0';
            num /= 10;
        }
    }
    UART_Tx_String(&buffer[i]); 
}

void setMotors(signed char leftDir, unsigned char leftSpeed, signed char rightDir, unsigned char rightSpeed) {
    if (leftDir == 1)      { AIN1 = 1; AIN2 = 0; }
    else if (leftDir == -1){ AIN1 = 0; AIN2 = 1; }
    else                   { AIN1 = 0; AIN2 = 0; }

    if (rightDir == 1)      { BIN1 = 1; BIN2 = 0; }
    else if (rightDir == -1){ BIN1 = 0; BIN2 = 1; }
    else                   { BIN1 = 0; BIN2 = 0; }

    CCPR1L = leftSpeed;         
    CCPR2L = rightSpeed;        
}

void main(void) {
    unsigned int sensorValues[8];
    unsigned char i;
    unsigned long weighted_sum;
    unsigned long sum;
    unsigned int position;
    unsigned int active_sensors;
    unsigned int telemetry_timer = 0;

    System_Init();
    
    UART_Tx_String("System Ready. No Calibration.\r\n");
    __delay_ms(1000); 

    for(;;) {
        weighted_sum = 0;
        sum = 0;
        active_sensors = 0;

        if (telemetry_timer % 20 == 0) LED = !LED;

        // 1. Read sensors
        for (i = 0; i < 8; i++) {
            sensorValues[i] = ADC_Read(i);
            
            // Using the global noise threshold
            if (sensorValues[i] > BLACK_LINE_NOISE) { 
                weighted_sum += (unsigned long)sensorValues[i] * i * 1000;
                sum += sensorValues[i];
                active_sensors++;
            }
        }
        
        // 2. Calculate Position (0 to 7000)
        if (active_sensors > 0) {
            position = (unsigned int)(weighted_sum / sum);
        } else {
            position = 3500; 
        }

        // 3. Proportional Motor Logic using Globals
        if (position < THRESHOLD_LEFT) { 
            // Turn Left
            setMotors(-1, TURN_SPEED_INNER, 1, TURN_SPEED_OUTER); 
        }
        else if (position > THRESHOLD_RIGHT) { 
            // Turn Right
            setMotors(1, TURN_SPEED_OUTER, -1, TURN_SPEED_INNER); 
        }
        else { 
            // Go Straight
            setMotors(1, BASE_SPEED, 1, BASE_SPEED);  
        }

        // 4. Throttled UART Telemetry
        telemetry_timer++;
        if (telemetry_timer >= 50) {
            UART_Tx_String("P:");
            UART_Tx_Number(position);
            UART_Tx_String("\r\n");
            telemetry_timer = 0; 
        }
    }
}