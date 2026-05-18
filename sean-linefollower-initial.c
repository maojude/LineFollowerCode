#include <xc.h>

// Configuration Bits (Updated for 4MHz XT)
#pragma config FOSC = XT   
#pragma config WDTE = OFF  
#pragma config PWRTE = ON  
#pragma config BOREN = ON  
#pragma config LVP = OFF   
#pragma config CPD = OFF   
#pragma config WRT = OFF   
#pragma config CP = OFF    

#define _XTAL_FREQ 4000000     // 4MHz for __delay_ms()

// Motor Direction Pin Mapping (TB6612FNG to PORTD)
#define AIN1 PORTDbits.RD0      // Left Motor Dir 1
#define AIN2 PORTDbits.RD1      // Left Motor Dir 2
#define BIN1 PORTDbits.RD2      // Right Motor Dir 1
#define BIN2 PORTDbits.RD3      // Right Motor Dir 2

void System_Init(void) {
    // 1. Configure I/O Pins
    TRISCbits.TRISC1 = 0;       // RC1 (CCP2) Output for PWMB (Right Speed)
    TRISCbits.TRISC2 = 0;       // RC2 (CCP1) Output for PWMA (Left Speed)
    TRISD = 0x00;              // RD0, RD1, RD2, RD3 as Outputs for Direction Logic, RD4, RD5 as LED Output
    PORTD = 0xF0;              // Initialize direction pins LOW, Initialize LED HIGH
    ADCON1 = 0x06;	       // Initialize PORTA pins as digital
    TRISA = 0x1F;	       // Initialize RA0 - RA4 as sensor input
   
    // 2. Configure Timer2 for PWM (Recalculated for 4MHz)
    // Target PWM Frequency: ~1kHz (Optimal for N20 motors)
    // Formula: PWM Freq = Fosc / (4 * Prescaler * (PR2 + 1))
    // 4000000 / (4 * 4 * 250) = 1000 Hz
    PR2 = 249;                  // Set period for 1kHz
    T2CON = 0b00000101;         // Timer2 ON, Prescaler 1:4

    // 3. Configure CCP Modules for PWM Mode
    CCP1CON = 0x0C;             // Left Motor PWM Mode
    CCP2CON = 0x0C;             // Right Motor PWM Mode

    // 4. Initialize Duty Cycle at 0 (Stopped)
    CCPR1L = 0;                 
    CCPR2L = 0;    
}

/*
 * Motor Control Function
 * dir: 1 (Forward), -1 (Reverse), 0 (Stop/Brake)
 * speed: 0 to 249 (Max PWM Duty Cycle based on PR2)
 */
void setMotors(signed char leftDir, unsigned char leftSpeed, signed char rightDir, unsigned char rightSpeed) {
    
    // Set Left Motor (A) Direction
    if (leftDir == 1) {
        AIN1 = 1; AIN2 = 0;     //CW, FWD 
    } else if (leftDir == -1) {
        AIN1 = 0; AIN2 = 1;     //CCW, BWD
    } else {
        AIN1 = 0; AIN2 = 0;     
    }

    // Set Right Motor (B) Direction
    if (rightDir == 1) {
        BIN1 = 1; BIN2 = 0;    //CW, FWD   
    } else if (rightDir == -1) {
        BIN1 = 0; BIN2 = 1;     //CCW, BWD
    } else {
        BIN1 = 0; BIN2 = 0;     
    }

    // Apply Speed (0 to 249)
    CCPR1L = leftSpeed;         
    CCPR2L = rightSpeed;        
}

void main(void) {
    System_Init();
    
    __delay_ms(1000);           // Power-on stabilization delay

    while(1) {
      if((PORTA & 0x1F) == 0x11 || (PORTA & 0x1F) == 0b00011011){ //move forward
	    // 1. Move Forward at ~50% Speed (125/249)
	    setMotors(1, 125, 1, 125);
	 }
      else if((PORTA & 0x1F) == 0b00010011){ 
	    // turn slightly left, slight forward;
	    setMotors(1, 62, 1, 125);
	 }
      else if((PORTA & 0x1F) == 0b00011001){ 
	    // turn slightly right, slight forward
	    setMotors(1, 125, 1, 62);
	 }
      else if((PORTA & 0x1F) == 0b00010111 || (PORTA & 0x1F) == 0b00000111 || (PORTA & 0x1F) == 0b00000011){ 
	    // turn more left
	    setMotors(1, 31, 1, 200);
	 }
      else if((PORTA & 0x1F) == 0b00011101 || (PORTA & 0x1F) == 0b00011100 || (PORTA & 0x1F) == 0b00011000){ 
	    //turn more right
	    setMotors(1, 200, 1, 31);
	 }
      else if((PORTA & 0x1F) == 0b00011110){ 
	   //turn hard right
	    setMotors(1, 200, 1, 24);
	 }
      else if((PORTA & 0x1F) == 0b00001111){ 
	    //turn hard left
	    setMotors(1, 24, 1, 200);
	 }
      else if((PORTA & 0x1F) == 0b00010000){ //near veer off to right 
	   //turn hard left, left wheel stop
	    setMotors(0, 0, 1, 125);
	 }
      else if((PORTA & 0x1F) == 0b00000001){ //near veer off to left 
	    //turn hard right, right wheel stop
	    setMotors(1, 125, 0, 0);
	 }
      else if((PORTA & 0x1F) == 0x00 || (PORTA & 0x1F) == 0x00){ 
	    //stop
	    setMotors(0, 0, 0, 0);
	 }
    }
}