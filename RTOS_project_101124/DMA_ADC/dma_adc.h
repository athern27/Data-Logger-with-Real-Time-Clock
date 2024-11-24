/*
 * dma_adc.h
 *
 *  Created on: Nov 24, 2024
 *      Author: karti
 */

#ifndef DMA_ADC_H_
#define DMA_ADC_H_

uint32_t tim_cnt = 0;

uint16_t vref_avg = 0;
uint16_t temp_avg = 0;
float vdda = 0; // Result of VDDA calculation
float vref = 0; // Result of vref calculation
float temp = 0; // Result of temp calculation
#define ADC_RESOLUTION 4095
#define ADC_SAMPLES 100

extern UART_HandleTypeDef huart2;

int _write(int fd, char *ptr, int len){
    HAL_StatusTypeDef hstatus;

    if (fd == 1 || fd == 2) {
        hstatus = HAL_UART_Transmit(&huart2, (uint8_t*) ptr, len, HAL_MAX_DELAY);
        if (hstatus == HAL_OK)
            return len;
        else
            return -1;
    }
    return -1;
}

// Process half a buffer full of data
static inline void process_adc_buffer(uint16_t *buffer){
    uint32_t sum1 = 0, sum2 = 0;
    for (int i = 0; i < ADC_SAMPLES; ++i) {
        sum1 += buffer[i * 2];
        sum2 += buffer[1 + i * 2];
    }

    vref_avg = sum2 / ADC_SAMPLES;
    temp_avg = sum1 / ADC_SAMPLES;

    // VDDA can be calculated based on the measured vref and the calibration data
    vdda = (float) VREFINT_CAL_VREF * (float) *VREFINT_CAL_ADDR / vref_avg / 1000;

    // Knowing vdda and the resolution of adc - the actual voltage can be calculated
    vref = (float) vdda / ADC_RESOLUTION * vref_avg;
    //vref = __LL_ADC_CALC_VREFANALOG_VOLTAGE(vref_avg, ADC_RESOLUTION_12B);

    temp = (float) ( (float)( (float)(TEMPSENSOR_CAL2_TEMP - TEMPSENSOR_CAL1_TEMP) / (float)(*TEMPSENSOR_CAL2_ADDR - *TEMPSENSOR_CAL1_ADDR)) * (temp_avg - *TEMPSENSOR_CAL1_ADDR) + TEMPSENSOR_CAL1_TEMP);
    //temp = __LL_ADC_CALC_TEMPERATURE(vref, temp_avg, ADC_RESOLUTION_12B);
}


#endif /* DMA_ADC_H_ */
