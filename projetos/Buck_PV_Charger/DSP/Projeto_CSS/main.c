/*
 * main.c
 * Autor: Matheus Tauffer de Paula
*/

#include "Peripheral_Setup.h"

#define OPENLOOP                                  // OPENLOOP / CLOSEDLOOP. OBS: comentar para D fixo
#define FILTER_FREQ_1                             // FILTER_FREQ_1 = 1kHz
#define STEP_D

// Constantes do filtro digital para leitura do potenci�metro
#ifdef FILTER_FREQ_1
#define b0 0.030459028
#define b1 0.030459028
#define a1 -0.93908194
#endif

/**
 * main.c
*/

// Declara��o de fun��es
void updatePWM(float d);

// Declara��o das vari�veis globais
uint16_t adc, adc1, adcf, adcf1, cont, test;
float duty, dutymax, dutymin;

#ifdef OPENLOOP
// Declara��o de fun��es de interrup��o
__interrupt void isr_adc(void);
#endif

int main(void){

    InitSysCtrl();                                  // Inicializa��o do sistema
    DINT;                                           // Desabilita as Interrup��es globais da CPU
    InitPieCtrl();                                  // Inicializa o registrador de controle PIE em seu estado padr�o
    IER = 0x0000;                                   // Desabilita as interrup��es da CPU
    IFR = 0x0000;                                   // Limpa todas as flags de interrup��o da CPU
    InitPieVectTable();                             // Inicializa��o da tabela de vetores do PIE

    EINT;                                           // Habilita��o das Interrup��es Globais INTM
    ERTM;                                           // Habilita��o da Interrup��o Global em tempo real DBGM

#ifdef OPENLOOP
    EALLOW;
    PieVectTable.ADCA1_INT = &isr_adc;              // Passa o endere�o da fun��o de interrup��o para a tabela de interrup��es
    EDIS;
#endif

    // Inicializa��o das vari�veis:
    duty = 0.6;                                   //
    dutymax = 0.95;
    dutymin = 0.05;
    //adc = 0;
    adc1 = 0;
    adcf1 = 0;

    // Configura��o de perif�ricos:
    Setup_GPIO();                                   // Chamada da fun��o que configura as GPIOs
    Setup_ePWM();                                   // Chamada da fun��o que configura os PWMs
    Setup_ADC();                                    // Chamada da fun��o que configura o ADC
    Setup_DAC();                                    // Chamada da fun��o que configura o DAC

    // Chamada a fun��o que atualiza a raz�o c�clica e o tempo morto do PWM
    updatePWM(duty);

    while(1){
        //updatePWM(duty);
    }

    return 0;
}

void updatePWM(float d){

    // Saturador de raz�o c�clica
    if(d > dutymax)
        d = dutymax;
    else if(d < dutymin)
        d = dutymin;

    // Configura��o da raz�o c�clica
    EPwm1Regs.CMPA.bit.CMPA = (d)*(EPwm1Regs.TBPRD);         // Configura a raz�o c�clica do m�dulo PWM1
}

#ifdef OPENLOOP
__interrupt void isr_adc(void){
    GpioDataRegs.GPADAT.bit.GPIO14 = 1;                      // Sinaliza o in�cio da convers�o A/D

    // Resultado da convers�o A/D (4 aquisi��es sucessivas)
    adc = (AdcaResultRegs.ADCRESULT0 + AdcaResultRegs.ADCRESULT1 + AdcaResultRegs.ADCRESULT2 + AdcaResultRegs.ADCRESULT3)/4;
    //test = (AdcaResultRegs.ADCRESULT4 + AdcaResultRegs.ADCRESULT5 + AdcaResultRegs.ADCRESULT6 + AdcaResultRegs.ADCRESULT7)/4;
    //test = AdcaResultRegs.ADCRESULT4;

    // Implementa��o de filtro digital para leitura do pot�nciometro
    adcf = (b0*adc)+(b1*adc1)-(a1*adcf1);                    // Eq. a diferen�as para o filtro do pot�nciometro
    adc1 = adc;
    adcf1 = adcf;

    DacaRegs.DACVALS.bit.DACVALS = adcf/3;                   // Convers�o p/ que 1V no DAC corresponda a D = 1

    duty = ((float)adcf)/4095;                               // Convers�o da leitura do A/D para raz�o c�clica

    #ifdef STEP_D                                            // Realiza��o de degraus de raz�o c�clica
        cont = cont + 1;
        if(cont <= 600){
            duty = 0.24;
        }else{
            duty = 0.4;
        }
        if(cont == 1200){
            cont = 0;
        }
        DacaRegs.DACVALS.bit.DACVALS = (uint16_t)(4095*duty);
    #endif

    updatePWM(duty);                                         // Chamada a fun��o que atualiza a raz�o c�clica

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;                   // Limpa a flag da interrup��o INT1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

    GpioDataRegs.GPADAT.bit.GPIO14 = 0;                      // Sinaliza o t�rmino da convers�o A/D
}
#endif

#ifdef CLOSEDLOOP
__interrupt void isr_adc(void){

}
#endif
