/*
Autor: Matheus Tauffer de Paula
*/

#include "Peripheral_Setup.h"

#define CLOSEDLOOP                                  // OPENLOOP / CLOSEDLOOP. OBS: comentar para D fixo
#define FILTER_FREQ_1                             // FILTER_FREQ_1 = 1kHz
//#define STEP_D

#define kcont1 0.003937
#define kcont2 0.003875

// Constantes do filtro digital para leitura do potenciômetro
#ifdef FILTER_FREQ_1
#define b0 0.030459028
#define b1 0.030459028
#define a1 -0.93908194
#endif

/**
 * main.c
*/

// Declaração de funções
void updatePWM(float d);

// Declaração das variáveis globais
uint16_t adc, adc1, adcf, adcf1, cont, ad4, ad1, ad2;
float duty, dutymax, dutymin, vo_0, vo_ref, ek, ek_1, uk, uk_1;

#ifdef OPENLOOP
// Declaração de funções de interrupção
__interrupt void isr_adc(void);
#endif

int main(void){

    InitSysCtrl();                                  // Inicialização do sistema
    DINT;                                           // Desabilita as Interrupções globais da CPU
    InitPieCtrl();                                  // Inicializa o registrador de controle PIE em seu estado padrão
    IER = 0x0000;                                   // Desabilita as interrupções da CPU
    IFR = 0x0000;                                   // Limpa todas as flags de interrupção da CPU
    InitPieVectTable();                             // Inicialização da tabela de vetores do PIE

    EINT;                                           // Habilitação das Interrupções Globais INTM
    ERTM;                                           // Habilitação da Interrupção Global em tempo real DBGM

#ifdef OPENLOOP
    EALLOW;
    PieVectTable.ADCA1_INT = &isr_adc;              // Passa o endereço da função de interrupção para a tabela de interrupções
    EDIS;
#endif

    // Inicialização das variáveis:
    duty = 0.6;                                   //
    dutymax = 0.9;
    dutymin = 0.1;
    //adc = 0;
    adc1 = 0;
    adcf1 = 0;

    // Configuração de periféricos:
    Setup_GPIO();                                   // Chamada da função que configura as GPIOs
    Setup_ePWM();                                   // Chamada da função que configura os PWMs
    Setup_ADC();                                    // Chamada da função que configura o ADC
    Setup_DAC();                                    // Chamada da função que configura o DAC

    // Chamada a função que atualiza a razão cíclica e o tempo morto do PWM
    updatePWM(duty);

    while(1){
        //updatePWM(duty);
    }

    return 0;
}

void updatePWM(float d){

    // Saturador de razão cíclica
    if(d > dutymax)
        d = dutymax;
    else if(d < dutymin)
        d = dutymin;

    // Configuração da razão cíclica
    EPwm1Regs.CMPA.bit.CMPA = (d)*(EPwm1Regs.TBPRD);         // Configura a razão cíclica do módulo PWM1
}

#ifdef OPENLOOP
__interrupt void isr_adc(void){
    GpioDataRegs.GPADAT.bit.GPIO14 = 1;                      // Sinaliza o início da conversão A/D

    // Resultado da conversão A/D (4 aquisições sucessivas)
    adc = (AdcaResultRegs.ADCRESULT0 + AdcaResultRegs.ADCRESULT1 + AdcaResultRegs.ADCRESULT2 + AdcaResultRegs.ADCRESULT3)/4;    // Potenciômetro
    ad2 = (AdcaResultRegs.ADCRESULT4 + AdcaResultRegs.ADCRESULT5 + AdcaResultRegs.ADCRESULT6 + AdcaResultRegs.ADCRESULT7)/4;    // Iin
    ad1 = (AdcaResultRegs.ADCRESULT8 + AdcaResultRegs.ADCRESULT9 + AdcaResultRegs.ADCRESULT10 + AdcaResultRegs.ADCRESULT11)/4;  // Vin
    ad4 = (AdcaResultRegs.ADCRESULT12 + AdcaResultRegs.ADCRESULT13 + AdcaResultRegs.ADCRESULT14 + AdcaResultRegs.ADCRESULT15)/4;// Vout

    // Implementação de filtro digital para leitura do potênciometro
    adcf = (b0*adc)+(b1*adc1)-(a1*adcf1);                    // Eq. a diferenças para o filtro do potênciometro
    adc1 = adc;
    adcf1 = adcf;

    DacaRegs.DACVALS.bit.DACVALS = adcf/3;                   // Conversão p/ que 1V no DAC corresponda a D = 1

    duty = ((float)adcf)/4095;                               // Conversão da leitura do A/D para razão cíclica

    #ifdef STEP_D                                            // Realização de degraus de razão cíclica
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

    updatePWM(duty);                                         // Chamada a função que atualiza a razão cíclica

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;                   // Limpa a flag da interrupção INT1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

    GpioDataRegs.GPADAT.bit.GPIO14 = 0;                      // Sinaliza o término da conversão A/D
}
#endif

#ifdef CLOSEDLOOP
__interrupt void isr_adc(void){
    GpioDataRegs.GPADAT.bit.GPIO14 = 1;                      // Sinaliza o início da conversão A/D

    // Resultado da conversão A/D (4 aquisições sucessivas)
    ad4 = (AdcaResultRegs.ADCRESULT12 + AdcaResultRegs.ADCRESULT13 + AdcaResultRegs.ADCRESULT14 + AdcaResultRegs.ADCRESULT15)/4;

    // Implementação de filtro digital para leitura do potênciometro
    //adcf = (b0*adc)+(b1*adc1)-(a1*adcf1);                    // Eq. a diferenças para o filtro do potênciometro
    //adc1 = adc;
    //adcf1 = adcf;

    vo_0 = (1*ad4)/4095;
    vo_ref = 12;

    // Cálculo do erro
    ek = vo_ref - vo_0;

    // Eq. a diferenças
    uk = uk_1 + (kcont1*ek) - (kcont2*ek_1);

    // Atualização das variáveis
    ek_1 = ek;
    uk_1 = uk;
    duty = uk;

    updatePWM(duty);                                         // Chamada a função que atualiza a razão cíclica

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;                   // Limpa a flag da interrupção INT1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

    GpioDataRegs.GPADAT.bit.GPIO14 = 0;                      // Sinaliza o término da conversão A/D
}
#endif
