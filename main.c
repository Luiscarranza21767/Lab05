/* Universidad del Valle de Guatemala
 IE2023 Programación de Microcontroladores
 Autor: Luis Pablo Carranza
 Compilador: XC8, MPLAB X IDE (v6.00)
 Proyecto: Laboratorio 4
 Hardware PIC16F887
 Creado: 10/10/22
 Última Modificación: 10/10/22*/

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT // Oscillator Selection bits (INTOSC 
//oscillator without clock out)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and 
//can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR 
//pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code 
//protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code 
//protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/
//External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-
//Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin 
//has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit 
//(Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits 
//(Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#define _XTAL_FREQ 500000

void setup(void);
void setupINTOSC();
void setup_ADC(void);
void setup_PWM(void);

unsigned int valADC;
unsigned int vPWM;
unsigned int vPWMl;
unsigned int vPWMh;

int main() {
    setup();
    setupINTOSC();
    setup_ADC();
    setup_PWM();
    
    while(1){
        ADCON0bits.GO = 1;
        while (ADCON0bits.GO == 1); // Revisa si ya terminó la conversión ADC
        ADIF = 0;               // Apaga la bandera del ADC
        
        valADC = ((ADRESH << 2) + (ADRESL >> 6));
        
        vPWM = (0.03128*valADC + 31);
        vPWMl = vPWM & 0x003;
 
        vPWMh = (vPWM & 0x3FC) >> 2;
        
        CCP1CONbits.DC1B = vPWMl;
        CCPR1L = vPWMh;

        __delay_ms(1);
    }
}

//******************************************************************************
// Configuración de puertos
//******************************************************************************
void setup(void){
    ANSELH = 0;
    TRISB = 0;
    TRISC = 0; 
    TRISD = 0;
    PORTC = 0;
    PORTB = 1;
    PORTD = 0;
}

void setupINTOSC(void){
    OSCCONbits.IRCF = 0b011;
    OSCCONbits.SCS = 1;
}
//******************************************************************************
// Configuración del ADC
//******************************************************************************
void setup_ADC(void){
    PORTAbits.RA0 = 0;      // Inicia el bit 0 de PORTA en 0
    TRISAbits.TRISA0 = 1;   // RA0 es entrada
    ANSELbits.ANS0 = 1;     // RA0 es analógico
    
    ADCON0bits.ADCS1 = 0;
    ADCON0bits.ADCS0 = 1;   // Fosc/8
    
    ADCON1bits.VCFG1 = 0;   // Ref VSS
    ADCON1bits.VCFG0 = 0;   // Ref VDD
    
    ADCON1bits.ADFM = 0;    // Justificado a la izquierda
    
    ADCON0bits.CHS3 = 0;
    ADCON0bits.CHS2 = 0;
    ADCON0bits.CHS1 = 0;
    ADCON0bits.CHS0 = 0;    // Selección del canal AN0
    
    ADCON0bits.ADON = 1;    // Habilitar el convertidor ADC
    __delay_us(100);
}
void setup_PWM(void){
    TRISCbits.TRISC2 = 1;
    
    PR2 = 155;
    
    CCP1CON = 0b00001100;
    TMR2IF = 0;
    T2CONbits.T2CKPS = 0b11;
    TMR2ON = 1;
    
    while(!TMR2IF);
    TRISCbits.TRISC2 = 0;
    
}