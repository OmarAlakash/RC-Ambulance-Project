#include <stdio.h>
#include <xc.h>
#include "config.h"
#define _XTAL_FREQ     4000000
#define Baud_rate 9600
#define LDR_THRESHOLD 25 // value of adc to turn on and off front lights 
#define TRIG1 RB4 // Define pins for ultrasonic sensors
#define ECHO1 RB5 // Define pins for ultrasonic sensors
#define TRIG2 RB6 // Define pins for ultrasonic sensors
#define ECHO2 RB7 // Define pins for ultrasonic sensors
#define DHT11_PIN  RC0   // Define the DHT11 data pin
#define DHT11_TRIS TRISC0 // TRIS register for DHT11 pin
#define TEMP_THRESHOLD 25

void delay_ms(unsigned int);
void delay_us(unsigned int);
void __interrupt() ISR();
void init_timer();
void Initialize_UART(void); // initalize UART for bluetooth module
void Initialize_Hbridge(); // Set H-Bridge pins as output
void Forward(); // moving function
void Backward(); // moving function
void Right(); // moving function
void Left(); // moving function
void Stop(); // moving function
void MOVE(); // respond to the char recived and move 
char UART_get_char(); // read char from bluetooth module 
unsigned int adc();
void Ultrasonic_Init(void); //setting output and input pins for ultrasonic
unsigned int Read_Ultrasonic1(void); //read distance from ultrasonic 1
unsigned int Read_Ultrasonic2(void); //read distance from ultrasonic 2
void PWM_Init();
void Set_PWM_Duty(unsigned int duty_us);
void openFridge();
void closeFridge();
void DHT11_Start();
unsigned char DHT11_Check_Response();
void DHT11_Read();
unsigned char DHT11_Read_Byte();
void Set_LED_DutyCycle(int percentage);
void LED_Init();
//*********************************************************************
void UART_send_string(char* st_pt);
void UART_send_char(char bt); // send char through Bluetooth

char get_value; // store value recived from bluetooth
char b[50];
unsigned int adc_val;
int led_val;
unsigned int uv1;
unsigned int uv2;
char no_backward;
char no_forward;
char status = 's';
unsigned char temperature = 0;
unsigned char humidity = 0;

unsigned int timer = 0;

void main(void) {
    Initialize_UART();
    Initialize_Hbridge();
    Ultrasonic_Init();
    init_timer();
    PWM_Init();
    Stop();
    TRISA0=1; //RA0 is input (ADC) LDR
    LED_Init(); // output (Front Lights)
    TRISC5=0; //RC5 is output (fridge fan (relay))
    TRISD6 = 0;
    TRISD5 = 0;
    RD6=0; // Warning Lights
    RD5=1; //Warning Lights
    RC5=1; // RELAY OFF BY DEFULT
    closeFridge();

    while (1) {
         timer++;
        if(timer == 3){
            timer = 0;
            RD6 = !RD6;
            RD5 = !RD5;
            
        adc_val = adc();
        if(adc_val > LDR_THRESHOLD){
           Set_LED_DutyCycle(0);
        }
        else{
            led_val = 50-(adc_val*2);
            Set_LED_DutyCycle(led_val);
        }
            
        }
        uv1 = Read_Ultrasonic1();
        uv2 = Read_Ultrasonic2();
        if(uv1<7 && uv1>0){
            if(status == 'f'){
                Stop();                
            }
            no_forward = 1;
        }
        else{
            no_forward = 0;
        }
        if(uv2<7 && uv2>0){
            if(status == 'b'){
                Stop();
            }
            no_backward = 1;
        }
        else{
            no_backward = 0;
        }
        MOVE();
        DHT11_Read();
        if(temperature > TEMP_THRESHOLD){
            RC5 = 0;
        }
        else  if(temperature <= TEMP_THRESHOLD){
            RC5 = 1;
        }
    }
}

void delay_ms(unsigned int x) {
    unsigned int i, j;
    for(i = 0; i < x; i++) {
        for(j = 0; j < 1000; j++) {
            asm("NOP");
            asm("NOP");
            asm("NOP");
            asm("NOP");
        }
    }
}

void delay_us(unsigned int x) {
    unsigned int i;
    for(i = 0; i < x; i++) {
        asm("NOP");
        asm("NOP");
        asm("NOP");
        asm("NOP");
    }
}


void PWM_Init() {
    TRISC2 = 0;            // Set RC2 as output
    PR2 = 249;             // Period register for 50 Hz (20 ms)
    T2CON = 0b00000111;    // Timer2 ON, Prescaler 1:16
    CCP1CON = 0b00001100;  // CCP1 in PWM mode
    CCPR1L = 0;            // Initial duty cycle (0%)
}
void LED_Init() {
    // Set the CCP1 pin (RC1) as output (CCP1 is now on pin 16, which is RC1)
    TRISC1 = 0;  // Set RC1 as output
    T1CON = 0x04;     // Set Timer2 prescaler to 4
//    TMR2 = 0;         // Clear Timer2 register
    PR2 = 249;        // Set the period (for 20kHz PWM, depends on the clock frequency)
    T1CONbits.TMR1ON = 1; // Start Timer2
    // Set the CCP1 module for PWM mode
    CCP2CON = 0x0C;  // PWM mode
    CCPR1L = 00;    // Initialize the duty cycle to 0 (off)
}

// Set PWM Duty Cycle for Servo (Pulse Width)
void Set_PWM_Duty(unsigned int duty_us) {
    unsigned int duty = (duty_us * (_XTAL_FREQ / 4)) / (16 * (PR2 + 1) * 1000); 
    CCPR1L = duty >> 2;              // Upper 8 bits
    CCP1CONbits.CCP1X = (duty & 0x02) >> 1; // Lower bit 1
    CCP1CONbits.CCP1Y = duty & 0x01;        // Lower bit 0
}

void init_timer(){
    INTCONbits.GIE = 1;   // Enable global interrupts
    INTCONbits.PEIE = 1;  // Enable peripheral interrupts
}
void __interrupt() ISR() {
    PORTBbits.RB0 = !PORTBbits.RB0;  // Toggle LED state
}

void Set_LED_DutyCycle(int percentage) {
    if (percentage > 100) {
        percentage = 100;  // Ensure the percentage is not greater than 100
    }
    if (percentage < 0) {
        percentage = 0;  // Ensure the percentage is not greater than 100
        RC1=0;
        return;
    }
    

    // The duty cycle is the percentage of the PR2 value.
    // PR2 is 255 for a 20kHz PWM frequency. So, duty cycle is (percentage * 255) / 100
    unsigned int duty = (percentage * 255) / 100;
    
    CCPR2L = duty;    // Set the duty cycle (8 most significant bits)
    CCP2CONbits.CCP2M1 = duty & 0x03;  // Set the 2 least significant bits (if any)
}

void Ultrasonic_Init(void) {
    ADCON1 = 0x06; // Configure all PORTB pins as digital
    TRISB4 = 0;
    TRISB5 = 1;
    TRISB6 = 0;
    TRISB7 = 1; 

    TRIG1 = 0;
    TRIG2 = 0;
}
unsigned int Read_Ultrasonic1(void) {
    unsigned int timeout = 300; // Timeout for waiting
    unsigned int time = 0;
    TRIG1 = 1;
   delay_us(10);
    TRIG1 = 0;
    while (!ECHO1 && timeout--) delay_us(1);
    if (timeout == 0) return -1; // No echo detected
    // Measure the high time of ECHO1
    timeout = 30000;
    while (ECHO1 && timeout--) {
        delay_us(1);
        time++;
    }
    if (timeout == 0) return -1; // Echo lasted too long
    return time / 5.8;
}
unsigned int Read_Ultrasonic2(void) {
    unsigned int timeout = 300; // Timeout for waiting
    unsigned int time = 0;
    TRIG2 = 1;
    delay_us(10);
    TRIG2 = 0;
    while (!ECHO2&& timeout--) delay_us(1);
    timeout = 30000;

    while (ECHO2 && timeout--) {
        delay_us(1);
        time++;
    }
    if (timeout == 0) return -1; // Echo lasted too long
    return time / 5.8;
}


void Initialize_UART(void) {
    TRISC6 = 0; // pic TX
    TRISC7 = 1;  // pic RX
    SPBRG = ((_XTAL_FREQ / 16) / Baud_rate) - 1;
    BRGH = 1;
    SYNC = 0;
    SPEN = 1;
    TXEN = 1; 
    CREN = 1;
    TX9 = 0;
    RX9 = 0;
}

void Initialize_Hbridge() {
    TRISB0 = 0;
    TRISB1 = 0;
    TRISB2 = 0;
    TRISB3 = 0;
}

void Forward() {
    RB0 = 1;
    RB1 = 0;
    RB2 = 1;
    RB3 = 0;
}

void Backward() {
    RB0 = 0;
    RB1 = 1;
    RB2 = 0;
    RB3 = 1;
}

void Right() {
    RB0 = 1;
    RB1 = 0;
    RB2 = 0;
    RB3 = 1;
}

void Left() {
    RB0 = 0;
    RB1 = 1;
    RB2 = 1;
    RB3 = 0;
}

void Stop() {
    RB0 = 0;
    RB1 = 0;
    RB2 = 0;
    RB3 = 0;
}
void MOVE() {
    get_value = UART_get_char();
    if (get_value == 'F' && no_forward == 0) 
    {
        status = 'f';
        Forward();
    }
    if (get_value == 'B' && no_backward == 0)
    {
        status = 'b';
        Backward();
    }
    if (get_value == 'R') {
        status = 'r';
        Right();
    }
    if (get_value == 'L') {
        status = 'l';
        Left();
    }
    if (get_value == 'S') {
        status = 's';
        Stop();
    }
    if (get_value == 'M') {
        openFridge();
    }
    if (get_value == 'm') {
        closeFridge();
    }
}

char UART_get_char() {
    if (OERR) { // Check for error and reset if there is error 
        CREN = 0;
        CREN = 1;
    }

    if (!RCIF) { // No data available
        return 0xFF; // Return a special value 0xFF if no data is available
    }

    return RCREG;
}

unsigned int adc()
{
    unsigned int adcval;
    ADCON1=0xc0;                    //right justified
    ADCON0=0x85;                    //adc on, fosc/64
    while(GO_nDONE);                //wait until conversion is finished
    adcval=((ADRESH<<8)|(ADRESL));    //store the result
    adcval=(adcval/3)-1;
    return adcval;  
}
void closeFridge(){
    Set_PWM_Duty(6);
}
void openFridge(){
    Set_PWM_Duty(3);
}
void DHT11_Start() {
    DHT11_TRIS = 0; // Set as output
    DHT11_PIN = 0;  // Pull DATA line LOW
    delay_ms(18); // Hold for at least 18 ms
    DHT11_PIN = 1;  // Pull DATA line HIGH
    delay_us(20); // Wait for 20?40 탎
    DHT11_TRIS = 1; // Set as input (release the line)
}

unsigned char DHT11_Check_Response() {
    unsigned char response = 0;
    delay_us(40);
    if (DHT11_PIN == 0) { // Wait for LOW signal (80 탎)
        delay_us(80);
        if (DHT11_PIN == 1) { // Wait for HIGH signal (80 탎)
            response = 1;
        }
    }
    while (DHT11_PIN); // Wait for the signal to go LOW
    return response;
}

unsigned char DHT11_Read_Byte() {
    unsigned char i, data = 0;
    for (i = 0; i < 8; i++) {
        while (!DHT11_PIN); // Wait for the pin to go HIGH
        delay_us(30);     // Wait for 30 탎 to read the bit
        if (DHT11_PIN == 1) {
            data |= (1 << (7 - i)); // Set the bit if the pin is HIGH
        }
        while (DHT11_PIN); // Wait for the pin to go LOW
    }
    return data;
}

void DHT11_Read(){
    unsigned char checksum;
    
    DHT11_Start(); // Send start signal to DHT11
    if (DHT11_Check_Response()) { // Check if DHT11 responded
        humidity = DHT11_Read_Byte();         // Read integral part of humidity
        DHT11_Read_Byte();                   // Skip decimal part (DHT11 does not provide it)
        temperature = DHT11_Read_Byte();     // Read integral part of temperature
        DHT11_Read_Byte();                   // Skip decimal part (DHT11 does not provide it)
        checksum = DHT11_Read_Byte();        // Read checksum
    }
}

// ***********************************************************************************************************\

void UART_send_string(char* st_pt) {
    while (*st_pt) //if there is a char

        UART_send_char(*st_pt++); //process it as a byte data

}
void UART_send_char(char bt) {

    while (!TXIF); // hold the program till TX buffer is free

    TXREG = bt; //Load the transmitter buffer with the received value

}
