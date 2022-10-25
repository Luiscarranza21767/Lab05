/* Universidad del Valle de Guatemala
 IE2023 Programación de Microcontroladores
 Autor: Luis Pablo Carranza
 Compilador: XC8, MPLAB X IDE (v6.00)
 Proyecto: Laboratorio 5
 Hardware PIC16F887
 Creado: 17/10/22
 Última Modificación: 17/10/22*/

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
void setup_PWM1(void);
void setup_PWM2(void);
void setupTMR0(void);
void mapeo(void);

// Variables para la configuración del PWM
unsigned int valADC;
unsigned int vPWM;
unsigned int vPWMl;
unsigned int vPWMh;
unsigned int cont;
unsigned int PWM3;

void __interrupt() isr (void){
    if (INTCONbits.T0IF){
        cont++;
        // Revisa si el contador es menor que el ADC
        if (cont <= PWM3){
            PORTCbits.RC3 = 1;  // Si sí, enciende el led
        }
        else {
            PORTCbits.RC3 = 0;  // Si no, lo apaga
        }
        TMR0 = 254;
        INTCONbits.T0IF = 0;
    }
}

int main() {
    setup();
    setupINTOSC();
    setup_ADC();
    setup_PWM1();
    setup_PWM2();
    setupTMR0();
    
    while(1){
        cont = 0;
        // Iniciar la conversión ADC
        ADCON0bits.CHS = 0b0000;
        ADCON0bits.GO = 1;
        while (ADCON0bits.GO == 1); // Revisa si ya terminó la conversión ADC
        ADIF = 0;               // Apaga la bandera del ADC
        // Hará el mapeo del ADC a valores para el servo
        mapeo();
        
        // Carga los bits bajos a CCP1CON <5:4>
        CCP1CONbits.DC1B = vPWMl;
        // Carga los bits altos a CCPR1L
        CCPR1L = vPWMh;
        
        ADCON0bits.CHS = 0b0001;
        ADCON0bits.GO = 1;
        while (ADCON0bits.GO == 1); // Revisa si ya terminó la conversión ADC
        ADIF = 0;               // Apaga la bandera del ADC
        // Hará el mapeo del ADC a valores para el servo
        mapeo();
           
        CCP2CONbits.DC2B0 = vPWMl & 0x01;
        CCP2CONbits.DC2B1 = ((vPWMl & 0x02) >> 1);
        // Carga los bits altos a CCPR1L
        CCPR2L = vPWMh;
        
        ADCON0bits.CHS = 0b0010;
        ADCON0bits.GO = 1;
        while (ADCON0bits.GO == 1); // Revisa si ya terminó la conversión ADC
        ADIF = 0;               // Apaga la bandera del ADC
        
        PWM3 = ((ADRESH << 2) + (ADRESL >> 6));
        
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
//******************************************************************************
// Configuración de oscilador interno
//******************************************************************************
void setupINTOSC(void){
    OSCCONbits.IRCF = 0b011;    // Oscilador 500KHz
    OSCCONbits.SCS = 1;         // Oscilador interno
}
//******************************************************************************
// Configuración del ADC
//******************************************************************************
void setup_ADC(void){
    PORTAbits.RA0 = 0;      // Inicia el bit 0 de PORTA en 0
    TRISAbits.TRISA0 = 1;   // RA0 es entrada
    ANSELbits.ANS0 = 1;     // RA0 es analógico
    
    PORTAbits.RA1 = 0;      // Configuración del canal analógico RA1
    TRISAbits.TRISA1 = 1;
    ANSELbits.ANS1 = 1;
    
    PORTAbits.RA2 = 0;      // Configuración del canal analógico RA2
    TRISAbits.TRISA2 = 1;
    ANSELbits.ANS2 = 1;
    
    
    ADCON0bits.ADCS1 = 0;
    ADCON0bits.ADCS0 = 1;   // Fosc/8
    
    ADCON1bits.VCFG1 = 0;   // Ref VSS
    ADCON1bits.VCFG0 = 0;   // Ref VDD
    
    ADCON1bits.ADFM = 0;    // Justificado a la izquierda
    
    ADCON0bits.ADON = 1;    // Habilitar el convertidor ADC
    __delay_us(100);
}
//******************************************************************************
// Configuración del PWM
//******************************************************************************
void setup_PWM1(void){
    TRISCbits.TRISC2 = 1;       // CCP1
    PR2 = 155;                  // Periodo de 2 ms
    CCP1CON = 0b00001100;       // P1A como PWM
    TMR2IF = 0;                 // bandera de TMR2 apagada
    T2CONbits.T2CKPS = 0b11;    // Prescaler 1:16
    TMR2ON = 1;                 // Habilitar TMR2
    while(!TMR2IF);             //
    TRISCbits.TRISC2 = 0;       // Habilitar la salida del PWM
}

void setup_PWM2(void){
    TRISCbits.TRISC1 = 1;       // CCP0
    PR2 = 155;                  // Periodo de 2 ms
    CCP2CON = 0b00001100;       // P2A como PWM
    TMR2IF = 0;                 // bandera de TMR2 apagada
    T2CONbits.T2CKPS = 0b11;    // Prescaler 1:16
    TMR2ON = 1;                 // Habilitar TMR2
    while(!TMR2IF);
    TRISCbits.TRISC1 = 0;       // Habilitar la salida del PWM
    
}

void mapeo(void){
    // Carga el resultado a valADC en una variable de 1024 bits
    valADC = ((ADRESH << 2) + (ADRESL >> 6));
        
    // Mapea el resultado a los valores calculados para 1 y 2ms
    vPWM = (0.0635*valADC + 31);
        
    // Obtiene los 2 bits bajos de la variable vPWM
    vPWMl = vPWM & 0x003;
        
    // Obtiene los 8 bits más altos de vPWM 
    vPWMh = (vPWM & 0x3FC) >> 2;
}

void setupTMR0(void){
    INTCONbits.GIE = 1;         //Habilitar interrupciones globales
    INTCONbits.T0IE = 1;        // Habilitar interrupción de TMR0
    INTCONbits.T0IF = 0;        // Desactivar la bandera de TMR0
    
    OPTION_REGbits.T0CS = 0;    // Fosc/4
    OPTION_REGbits.PSA = 0;     // Prescaler para TMR0
    OPTION_REGbits.PS = 0b011;  // Prescaler 1:16
    TMR0 = 250;                 // Valor inicial del TMR0
}
