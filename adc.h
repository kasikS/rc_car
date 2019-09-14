/*
 * adc.h
 *
 *  Created on: Sep 14, 2019
 *      Author: kasik
 */

#ifndef ADC_H_
#define ADC_H_

extern const int ADC_NUMBER;

void adc_init(void);
void adc_read(uint32_t * adcReadout);


#endif /* ADC_H_ */
