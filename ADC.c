/* 
 * File:   ADC.c
 * Author: joseg
 *
 * Created on July 22, 2022, 11:24 AM
 */

#include <stdio.h>
#include <stdlib.h>
#include "ADC.h"
#include <xc.h>
#define _XTAL_FREQ 4000000

uint8_t valor_adc = 0;


void adc_init (uint8_t adc_cs, uint8_t vref_plus, uint8_t vref_minus){

    
    ADCON0bits.ADCS = adc_cs & 0b11;  // Seleccionamos el canal del ADC
    ADCON0bits.ADON = 1;                // ADC enable bit
    ADCON1bits.VCFG1 = vref_plus;       // Voltage de referencia positivo
    ADCON1bits.VCFG0 = vref_minus;      // Voltage de referencia negativo
    PIE1bits.ADIE = 1;
    PIR1bits.ADIF = 0;
    
    __delay_ms(1);
    
    
}

void adc_start(uint8_t channel){
    ADCON0bits.CHS = channel & 0b1111;  // canal a trabajar
    ADCON0bits.GO= 1;            // Status de conversion de datos
}

uint8_t adc_read (void){
    PIR1bits.ADIF = 0;                  // Bandera de interrupcion
    valor_adc = ADRESH; // COrrimiento de los bits
    return valor_adc;
}