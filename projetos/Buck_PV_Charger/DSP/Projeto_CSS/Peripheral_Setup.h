/*
 * Peripheral_Setup.h
 *
 *  Created on: 11 de mai de 2023
 *      Author: Matheus
 */

#ifndef PERIPHERAL_SETUP_H_
#define PERIPHERAL_SETUP_H_
#include "F28x_Project.h"               // Importação das bibliotecas do Control Suite

void Setup_GPIO(void);
void Setup_ePWM(void);
void Setup_ADC(void);
void Setup_DAC(void);

#endif /* PERIPHERAL_SETUP_H_ */
