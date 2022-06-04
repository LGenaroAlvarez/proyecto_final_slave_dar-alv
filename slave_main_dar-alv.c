/*
 * File:   slave_main_dar-alv.c
 * Author: luisg
 *
 * Created on May 24, 2022, 4:46 PM
 */
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)
// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)
// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
#include <xc.h>
#include <stdint.h>
/*------------------------------------------------------------------------------
 * CONSTANTES 
 ------------------------------------------------------------------------------*/
#define _XTAL_FREQ 4000000
#define IN_MIN 0                // VALOR MINIMO DE ENTRADA DEL POTENCIOMETRO
#define IN_MAX 255              // VALOR MAXIMO  DE ENTRADA DEL POTENCIOMETRO
#define OUT_MIN 13              // Valor minimo de ancho de pulso de señal PWM
#define OUT_MAX 80              // Valor maximo de ancho de pulso de señal PWM
/*------------------------------------------------------------------------------
 * VARIABLES 
 ------------------------------------------------------------------------------*/

unsigned short CCPR = 0;// Variable para almacenar ancho de pulso al hacer la interpolación lineal
unsigned short CCPRx = 0;

/*------------------------------------------------------------------------------
 * PROTOTIPO DE FUNCIONES 
 ------------------------------------------------------------------------------*/
void setup(void);

// FUNCION DE MAPEO
unsigned short map(uint8_t val, uint8_t in_min, uint8_t in_max, 
            unsigned short out_min, unsigned short out_max);

/*------------------------------------------------------------------------------
 * INTERRUPCIONES 
 ------------------------------------------------------------------------------*/
void __interrupt() isr (void){
    if (PIR1bits.SSPIF){
        if (PORTAbits.RA0 == 1){
            PORTB = SSPBUF;
            CCPR = map(SSPBUF, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX); // Valor de ancho de pulso
            CCPR1L = (uint8_t)(CCPR>>2);    // Guardamos los 8 bits mas significativos en CPR1L
            CCP1CONbits.DC1B = CCPR & 0b11; // Guardamos los 2 bits menos significativos en DC1B
        }

        else if (PORTAbits.RA1 == 1){
            PORTD = SSPBUF;
            CCPRx = map(SSPBUF, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX); // Valor de ancho de pulso
            CCPR2L = (uint8_t)(CCPRx>>2);    // Guardamos los 8 bits mas significativos en CPR1L
            CCP2CONbits.DC2B0 = CCPRx & 0b01;
            CCP2CONbits.DC2B1 = CCPRx & 0b10;
        }
        PIR1bits.SSPIF = 0;             // Limpiamos bandera de interrupcion
    }
    //return;
}
/*------------------------------------------------------------------------------
 * CICLO PRINCIPAL
 ------------------------------------------------------------------------------*/
void main(void) {
    setup();
    while(1){        
        
    }
    return;
}
/*------------------------------------------------------------------------------
 * CONFIGURACION 
 ------------------------------------------------------------------------------*/
void setup(void){
    ANSEL = 0;
    ANSELH = 0;
 
    TRISA = 0b00100011;
    TRISB = 0;
    TRISD = 0;
    PORTA = 0;
    PORTB = 0;
    PORTD = 0;
    
    OSCCONbits.IRCF = 0b0011;    // 500kHz
    OSCCONbits.SCS = 1;         // Reloj interno
    
    // Configuracion de SPI
    // Configs del esclavo
    TRISC = 0b00011000; // -> SDI y SCK entradas, SD0 como salida
    PORTC = 0;

    // SSPCON <5:0>
    SSPCONbits.SSPM = 0b0100;   // -> SPI Esclavo, SS hablitado
    SSPCONbits.CKP = 0;         // -> Reloj inactivo en 0
    SSPCONbits.SSPEN = 1;       // -> Habilitamos pines de SPI
    // SSPSTAT<7:6>
    SSPSTATbits.CKE = 1;        // -> Dato enviado cada flanco de subida
    SSPSTATbits.SMP = 0;        // -> Dato al final del pulso de reloj

   
    //CONFIG DE INTERRUPCIONES
    PIR1bits.SSPIF = 0;         // LIMPIAR BANDERA DE INTERRUPCIONES DE SP1
    PIE1bits.SSPIE = 1;         // ACTIVAR INTERRUPCIONES DE SPI
    INTCONbits.GIE = 1;         // ACTIVAR INTERRUPCIONES GLOBALES
    INTCONbits.PEIE = 1;        // ACTIVAR INTERRUPCIONES DE PERIFERICOS

   
    //PWM CONFIG
    TRISCbits.TRISC2 = 1;           // CCP1 COMO ENTRADA (SALIDA DESABILITADA)
    TRISCbits.TRISC1 = 1;           // CCP2 COMO ENTRADA (SALIDA DESABILITADA)
    PR2 = 156;                      // PERIODO DE TMR2 EN 20mS

    //CCP CONFIG
    CCP1CON = 0;                    // CCP1 APAGADO
    CCP2CON = 0;                    // CCP2 APAGADO
    CCP1CONbits.P1M = 0;            // CAMBIO DE MODO A "SINGLE OUTPUT"
    CCP1CONbits.CCP1M = 0b1100;     // PWM PARA CCP1
    CCP2CONbits.CCP2M = 0b1100;     // PWM PARA CCP2

    CCPR1L = 155>>2;                // CONFIGURACION DE ANCHO DE PULSO PARA CCP1 Y CCP2
    CCP1CONbits.DC1B = 155 & 0b11;  //
    CCPR2L = 155>>2;                //
    CCP2CONbits.DC2B0 = 155 & 0b01; //
    CCP2CONbits.DC2B1 = 155 & 0b10; //


    T2CONbits.T2CKPS = 0b11;        // RPESCALER DEL TMR2 EN 1:16
    PIR1bits.TMR2IF = 0;            // LIMPIEZA DE BANDERA DE INTERRUPCION DE TMR2
    T2CONbits.TMR2ON = 1;           // TMR2 ENCENDIDO
    while(!PIR1bits.TMR2IF);        // CICLO INDIVIDUAL DE TMR2 EN ESPERA
    PIR1bits.TMR2IF = 0;            // LIMPIEZA DE BANDERA DE INTERRUPCION DE TMR2

    TRISCbits.TRISC2 = 0;
    TRISCbits.TRISC1 = 0;
    return;
}

// FUNCION DE MAPEO DE DATOS
unsigned short map(uint8_t x, uint8_t x0, uint8_t x1, 
            unsigned short y0, unsigned short y1){
    return (unsigned short)(y0+((float)(y1-y0)/(x1-x0))*(x-x0));
}