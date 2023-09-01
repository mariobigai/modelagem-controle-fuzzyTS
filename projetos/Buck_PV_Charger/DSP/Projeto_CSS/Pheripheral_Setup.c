/*
 * Peripheral_Setup.c
 *
 *  Created on: 11 de mai de 2023
 *      Author: Matheus
 */

#include "Peripheral_Setup.h"

void Setup_GPIO(void){

    EALLOW;                                             // Habilita a edição dos registradores protegidos

    GpioCtrlRegs.GPAGMUX1.bit.GPIO0 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;                 // Pino 0 configurado como EPWM1A

    GpioCtrlRegs.GPAGMUX1.bit.GPIO14 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 0;                // Pino 14 configurado como GPIO
    GpioCtrlRegs.GPADIR.bit.GPIO14 = 1;                 // Pino 14 configurado como saída

    EDIS;                                               // Desabilita a edição dos registradores
}

void Setup_ePWM(void){

    EALLOW;                                             // Acesso a registradores restritos

    CpuSysRegs.PCLKCR2.bit.EPWM1 = 1;                   // Habilita o clock do Módulo de PWM n° 1
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;               // Desabilita o clock do contador para realizar a configuração a seguir

    EDIS;                                               // Desabilita o acesso a registradores restritos

    // PWM1A e PWM1B (Pinos P0 e P1)
    EPwm1Regs.TBPRD = 666;                              // Configura o período do PWM 1

    EPwm1Regs.TBPHS.bit.TBPHS = 0;                      // Configura o deslocamento de phase (fração do valor colocado em TBPRD)
    EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;         //
    EPwm1Regs.TBCTR = 0x0000;                           // Reset do contador
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;          // Configura o modo de contagem como crescente
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;             // Desabilita o deslocamento de fase
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;            // Clock (prescaler) alta velocidade
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;               // Clock (prescaler)

    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;         // Habilita o SHADOW
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD;   // Carrega quando atingir zero ou o PRD
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;         // Habilita o SHADOW
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO_PRD;   // Carrega quando atingir zero ou o PRD

    EPwm1Regs.AQCTLA.bit.PRD = AQ_NO_ACTION;            // Nenhuma ação ao atingir PRD
    EPwm1Regs.AQCTLA.bit.ZRO = AQ_SET;                  // Set ao atingir zero
    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;                // Reset ao atingir CMPA na subida
    EPwm1Regs.AQCTLA.bit.CAD = AQ_NO_ACTION;            // Nenhuma ação ao atingir CMPA na descida

    //EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;           // Habilita o pulso complementar
    //EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;      // Habilita o Módulo de Tempo Morto

    // Geração do trigger para o ADC
    EPwm1Regs.ETSEL.bit.SOCAEN = 1;                     // Habilita o SOC (start of convertion) no grupo A
    EPwm1Regs.ETSEL.bit.SOCASEL = ET_CTR_PRDZERO;       // Dispara o ADC no PRD e no ZERO
    EPwm1Regs.ETPS.bit.SOCAPRD = ET_1ST;                // Trigga em todos os eventos

    EALLOW;

    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;               // Habilita o clock do contador

    EDIS;
}

void Setup_ADC(void){

    Uint16 acqps;

    // Determinar a janela de aquisição mínima de acordo com a resolução utilizada
    if(ADC_RESOLUTION_12BIT == AdcaRegs.ADCCTL2.bit.RESOLUTION){
        acqps = 14;                                     // 75 ns
        //acqps = 56;
    }else{                                              // 16 bits
        acqps = 63;                                     // 320 ns
    }

    EALLOW;

    CpuSysRegs.PCLKCR13.bit.ADC_A = 1;                  // Habilita o clock do ADC A
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6;                  // Configura o divisor de clock para 4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);  // Função de configuração do ADC
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;               // Configuração de onde é disparado o pulso da interrupção (antes do resultado nesse caso)
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;                  // Alimenta o ADC
    DELAY_US(1000);

    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 3;                  // Selação do ADC a ser utilizado
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = acqps;              // Tamanho da janela de amostragem
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5;                // Confifuração do evento que dispara o ADC (PWM1 SOCA/C)
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 3;                  // Selação do ADC a ser utilizado
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = acqps;              // Tamanho da janela de amostragem
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 5;                // Confifuração do evento que dispara o ADC (PWM1 SOCA/C)
    AdcaRegs.ADCSOC2CTL.bit.CHSEL = 3;                  // Selação do ADC a ser utilizado
    AdcaRegs.ADCSOC2CTL.bit.ACQPS = acqps;              // Tamanho da janela de amostragem
    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = 5;                // Confifuração do evento que dispara o ADC (PWM1 SOCA/C)
    AdcaRegs.ADCSOC3CTL.bit.CHSEL = 3;                  // Selação do ADC a ser utilizado
    AdcaRegs.ADCSOC3CTL.bit.ACQPS = acqps;              // Tamanho da janela de amostragem
    AdcaRegs.ADCSOC3CTL.bit.TRIGSEL = 5;                // Confifuração do evento que dispara o ADC (PWM1 SOCA/C)

    //AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 3;

    // TESTE
    AdcaRegs.ADCSOC4CTL.bit.CHSEL = 2;                  // Selação do ADC a ser utilizado
    AdcaRegs.ADCSOC4CTL.bit.ACQPS = acqps;              // Tamanho da janela de amostragem
    AdcaRegs.ADCSOC4CTL.bit.TRIGSEL = 5;                // Confifuração do evento que dispara o ADC (PWM1 SOCA/C)
    AdcaRegs.ADCSOC5CTL.bit.CHSEL = 2;                  // Selação do ADC a ser utilizado
    AdcaRegs.ADCSOC5CTL.bit.ACQPS = acqps;              // Tamanho da janela de amostragem
    AdcaRegs.ADCSOC5CTL.bit.TRIGSEL = 5;                // Confifuração do evento que dispara o ADC (PWM1 SOCA/C)
    AdcaRegs.ADCSOC6CTL.bit.CHSEL = 2;                  // Selação do ADC a ser utilizado
    AdcaRegs.ADCSOC6CTL.bit.ACQPS = acqps;              // Tamanho da janela de amostragem
    AdcaRegs.ADCSOC6CTL.bit.TRIGSEL = 5;                // Confifuração do evento que dispara o ADC (PWM1 SOCA/C)
    AdcaRegs.ADCSOC7CTL.bit.CHSEL = 2;                  // Selação do ADC a ser utilizado
    AdcaRegs.ADCSOC7CTL.bit.ACQPS = acqps;              // Tamanho da janela de amostragem
    AdcaRegs.ADCSOC7CTL.bit.TRIGSEL = 5;                // Confifuração do evento que dispara o ADC (PWM1 SOCA/C)

    AdcaRegs.ADCSOC8CTL.bit.CHSEL = 1;                  // Selação do ADC a ser utilizado
    AdcaRegs.ADCSOC8CTL.bit.ACQPS = acqps;              // Tamanho da janela de amostragem
    AdcaRegs.ADCSOC8CTL.bit.TRIGSEL = 5;                // Confifuração do evento que dispara o ADC (PWM1 SOCA/C)
    AdcaRegs.ADCSOC9CTL.bit.CHSEL = 1;                  // Selação do ADC a ser utilizado
    AdcaRegs.ADCSOC9CTL.bit.ACQPS = acqps;              // Tamanho da janela de amostragem
    AdcaRegs.ADCSOC9CTL.bit.TRIGSEL = 5;                // Confifuração do evento que dispara o ADC (PWM1 SOCA/C)
    AdcaRegs.ADCSOC10CTL.bit.CHSEL = 1;                  // Selação do ADC a ser utilizado
    AdcaRegs.ADCSOC10CTL.bit.ACQPS = acqps;              // Tamanho da janela de amostragem
    AdcaRegs.ADCSOC10CTL.bit.TRIGSEL = 5;                // Confifuração do evento que dispara o ADC (PWM1 SOCA/C)
    AdcaRegs.ADCSOC11CTL.bit.CHSEL = 1;                  // Selação do ADC a ser utilizado
    AdcaRegs.ADCSOC11CTL.bit.ACQPS = acqps;              // Tamanho da janela de amostragem
    AdcaRegs.ADCSOC11CTL.bit.TRIGSEL = 5;                // Confifuração do evento que dispara o ADC (PWM1 SOCA/C)

    AdcaRegs.ADCSOC12CTL.bit.CHSEL = 4;                  // Selação do ADC a ser utilizado
    AdcaRegs.ADCSOC12CTL.bit.ACQPS = acqps;              // Tamanho da janela de amostragem
    AdcaRegs.ADCSOC12CTL.bit.TRIGSEL = 5;                // Confifuração do evento que dispara o ADC (PWM1 SOCA/C)
    AdcaRegs.ADCSOC13CTL.bit.CHSEL = 4;                  // Selação do ADC a ser utilizado
    AdcaRegs.ADCSOC13CTL.bit.ACQPS = acqps;              // Tamanho da janela de amostragem
    AdcaRegs.ADCSOC13CTL.bit.TRIGSEL = 5;                // Confifuração do evento que dispara o ADC (PWM1 SOCA/C)
    AdcaRegs.ADCSOC14CTL.bit.CHSEL = 4;                  // Selação do ADC a ser utilizado
    AdcaRegs.ADCSOC14CTL.bit.ACQPS = acqps;              // Tamanho da janela de amostragem
    AdcaRegs.ADCSOC14CTL.bit.TRIGSEL = 5;                // Confifuração do evento que dispara o ADC (PWM1 SOCA/C)
    AdcaRegs.ADCSOC15CTL.bit.CHSEL = 4;                  // Selação do ADC a ser utilizado
    AdcaRegs.ADCSOC15CTL.bit.ACQPS = acqps;              // Tamanho da janela de amostragem
    AdcaRegs.ADCSOC15CTL.bit.TRIGSEL = 5;                // Confifuração do evento que dispara o ADC (PWM1 SOCA/C)

    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 15;
    // FIM TESTE

    //AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 3;              // Ao final de SOC3 a flag de interrupção (INT1) é setada
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;                // Habilita a interrupção
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;              // Limpa a flag de interrupção INT1

    EDIS;

    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;                  // ADC (linha 1 e coluna 1)
    IER |= M_INT1;                                      // Habilita a interrupção da linha
}

void Setup_DAC(void){
    EALLOW;
    CpuSysRegs.PCLKCR16.bit.DAC_A = 1;                  // Habilita o clock do DAC A
    DacaRegs.DACCTL.bit.SYNCSEL = 0x00;                 // O sinal EPWM1SYNCPER atualiza o registrador DACVALA
    DacaRegs.DACCTL.bit.LOADMODE = 0x01;                // Carrega no próximo EPWMSYNCPER especificado pelo SYNCSEL
    DacaRegs.DACCTL.bit.DACREFSEL = 0x01;               // Seleção da referência do DAC (igual ao ADC)
    DacaRegs.DACVALS.bit.DACVALS = 0;
    DacaRegs.DACOUTEN.bit.DACOUTEN = 1;                 // Habilita a saída do DAC
    DacaRegs.DACLOCK.all = 0x00;
    EDIS;
}
