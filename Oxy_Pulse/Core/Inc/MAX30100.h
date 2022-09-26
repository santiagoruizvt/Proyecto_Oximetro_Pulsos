/*
 * MAX30100.h
 *
 *  Created on: Aug 26, 2022
 *      Author: ssant
 */

#ifndef INC_MAX30100_H_
#define INC_MAX30100_H_


#define FIFO_WR_PTR_ADDRESS 			  	0x02
#define DEFAULT_OPERATING_MODE           	MAX30100_MODE_SPO2_HR
#define DEFAULT_SAMPLING_RATE				MAX30100_SAMPLING_RATE_100HZ
#define DEFAULT_LED_PULSE_WIDTH          	MAX30100_PULSE_WIDTH_1600US_ADC_16
#define DEFAULT_IR_LED_CURRENT           	MAX30100_LED_CURRENT_50MA
#define STARTING_RED_LED_CURRENT         	MAX30100_LED_CURRENT_27_1MA
#define MEAN_FILTER_SIZE        		  	15

#define PULSE_MIN_THRESHOLD         		100 //probar con 300 si no funciona
#define PULSE_MAX_THRESHOLD         		2000
#define PULSE_GO_DOWN_THRESHOLD     		1
#define PULSE_GO_DOWN_THRESHOLD     		1
#define PULSE_BPM_SAMPLE_SIZE       		10 //tamaño del promedio

#define MAGIC_ACCEPTABLE_INTENSITY_DIFF    	65000
#define RED_LED_CURRENT_ADJUSTMENT_MS      	500

#define RESET_SPO2_EVERY_N_PULSES     		4

typedef struct FIFO_t
{
	uint16_t rawIR;
	uint16_t rawRED;
} FIFO_t;

typedef struct pulseoxymeter_t
{
	bool pulseDetected;
	float heartBPM;
	float irCardiogram;
	float irDcValue;
	float redDcValue;
	float SaO2;
	uint32_t lastBeatThreshold;
	float dcFilteredIR;
	float dcFilteredRed;
} pulseoxymeter_t;

typedef struct meanDiffFilter_t
{
	float values[MEAN_FILTER_SIZE];
	uint8_t index;
	float sum;
	uint8_t count;
} meanDiffFilter_t;

typedef struct butterworthFilter_t
{
	float v[2];
	float result;
}butterworthFilter_t;

typedef enum MODE {
    MAX30100_MODE_HR_ONLY                 = 0x02,
    MAX30100_MODE_SPO2_HR                 = 0x03
} MODE;

#define MAX30100_SPO2_HI_RES_EN           (1 << 6)

typedef enum SAMPLING_RATE {
    MAX30100_SAMPLING_RATE_50HZ           = 0x00,
    MAX30100_SAMPLING_RATE_100HZ          = 0x01,
    MAX30100_SAMPLING_RATE_167HZ          = 0x02,
    MAX30100_SAMPLING_RATE_200HZ          = 0x03,
    MAX30100_SAMPLING_RATE_400HZ          = 0x04,
    MAX30100_SAMPLING_RATE_600HZ          = 0x05,
    MAX30100_SAMPLING_RATE_800HZ          = 0x06,
    MAX30100_SAMPLING_RATE_1000HZ         = 0x07
} SAMPLING_RATE;

typedef enum LED_PULSE_WIDTH {
    MAX30100_PULSE_WIDTH_200US_ADC_13     = 0x00,
    MAX30100_PULSE_WIDTH_400US_ADC_14     = 0x01,
    MAX30100_PULSE_WIDTH_800US_ADC_15     = 0x02,
    MAX30100_PULSE_WIDTH_1600US_ADC_16    = 0x03,
} LED_PULSE_WIDTH;

typedef enum LED_CURRENT {
    MAX30100_LED_CURRENT_0MA              = 0x00,
    MAX30100_LED_CURRENT_4_4MA            = 0x01,
    MAX30100_LED_CURRENT_7_6MA            = 0x02,
    MAX30100_LED_CURRENT_11MA             = 0x03,
    MAX30100_LED_CURRENT_14_2MA           = 0x04,
    MAX30100_LED_CURRENT_17_4MA           = 0x05,
    MAX30100_LED_CURRENT_20_8MA           = 0x06,
    MAX30100_LED_CURRENT_24MA             = 0x07,
    MAX30100_LED_CURRENT_27_1MA           = 0x08,
    MAX30100_LED_CURRENT_30_6MA           = 0x09,
    MAX30100_LED_CURRENT_33_8MA           = 0x0A,
    MAX30100_LED_CURRENT_37MA             = 0x0B,
    MAX30100_LED_CURRENT_40_2MA           = 0x0C,
    MAX30100_LED_CURRENT_43_6MA           = 0x0D,
    MAX30100_LED_CURRENT_46_8MA           = 0x0E,
    MAX30100_LED_CURRENT_50MA             = 0x0F
} LED_CURRENT;

typedef enum Pulse_MEF{
	PULSE_IDLE							  =0X00,
	PULSE_TRACE_UP						  =0x01,
	PULSE_TRACE_DOWN					  =0x02
} Pulse_MEF;

extern FIFO_t *mainFIFO;

void Inicio_Heart_Rate(void);
void Inicio_SPO2_HR(void);
void MAX30100_I2C_Write(uint8_t, uint8_t, uint8_t);
void Lectura_FIFO(FIFO_t *);
void Filtrado_DC(uint16_t*,float*,float*);
void Mean_Median_Filter(float,meanDiffFilter_t*,float*);
void Filtro_PasabajosButterworth(float*,butterworthFilter_t*,float*);
bool detectPulse(float,pulseoxymeter_t *);
void Balance_Intensidades(float, float);
void Resetea_Resultados(pulseoxymeter_t *,meanDiffFilter_t *,float);
#endif /* INC_MAX30100_H_ */
