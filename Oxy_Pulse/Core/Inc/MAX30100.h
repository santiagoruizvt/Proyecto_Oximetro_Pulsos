/*
 * MAX30100.h
 *
 *  Created on: Aug 26, 2022
 *      Author: ssant
 */

#ifndef INC_MAX30100_H_
#define INC_MAX30100_H_

#define DEFAULT_OPERATING_MODE           	MAX30100_MODE_SPO2_HR

#define DEFAULT_SAMPLING_RATE				MAX30100_SAMPLING_RATE_100HZ
#define DEFAULT_LED_PULSE_WIDTH          	MAX30100_PULSE_WIDTH_1600US_ADC_16

#define DEFAULT_IR_LED_CURRENT           	MAX30100_LED_CURRENT_50MA
#define STARTING_RED_LED_CURRENT         	MAX30100_LED_CURRENT_27_1MA
//Ajustes para el balance de intensidades de RED LED
#define MAGIC_ACCEPTABLE_INTENSITY_DIFF    	65000
#define RED_LED_CURRENT_ADJUSTMENT_MS      	500

#define FIFO_WR_PTR_ADDRESS 			  	0x02


//SaO2
#define RESET_SPO2_EVERY_N_PULSES     		4

//Parámetros para el filtro DC
#define ALPHA 								0.95
#define MEAN_FILTER_SIZE        		  	15

//Parámetros para la detección del pulso
#define PULSE_MIN_THRESHOLD         		100 //probar con 300 si no funciona
#define PULSE_MAX_THRESHOLD         		2000
#define PULSE_GO_DOWN_THRESHOLD     		1

#define PULSE_BPM_SAMPLE_SIZE       		10 //tamaño del promedio móvil

#define MAX30100_DEVICE                   0x57

//Part ID Registers
#define MAX30100_REV_ID                   0xFE
#define MAX30100_PART_ID                  0xFF

//status registers
#define MAX30100_INT_STATUS               0x00
#define MAX30100_INT_ENABLE               0x01

//Fifo registers
#define MAX30100_FIFO_WRITE               0x02
#define MAX30100_FIFO_OVERFLOW_COUNTER    0x03
#define MAX30100_FIFO_READ                0x04
#define MAX30100_FIFO_DATA                0x05

//Config registers
#define MAX30100_MODE_CONF                0x06
#define MAX30100_SPO2_CONF                0x07
#define MAX30100_LED_CONF                 0x09

//Temperature registers
#define MAX30100_TEMP_INT                 0x16
#define MAX30100_TEMP_FRACTION            0x17

//Bit defines MODE Register
#define MAX30100_MODE_SHDN                (1<<7)
#define MAX30100_MODE_RESET               (1<<6)
#define MAX30100_MODE_TEMP_EN             (1<<3)
// Tipos de variables, structs y enums

typedef struct FIFO_t
{
	uint16_t rawIR;
	uint16_t rawRED;
} FIFO_t;

typedef struct pulseoxymeter_t
{
	bool 		pulseDetected;
	float 		heartBPM;
	float 		irCardiogram;
	float 		irDcValue;
	float 		redDcValue;
	float 		SaO2;
	uint32_t 	lastBeatThreshold;
	float 		dcFilteredIR;
	float 		dcFilteredRed;
} pulseoxymeter_t;

typedef struct meanDiffFilter_t
{
	float 		values[MEAN_FILTER_SIZE];
	uint8_t 	index;
	float 		sum;
	uint8_t 	count;
} meanDiffFilter_t;

typedef struct butterworthFilter_t
{
	float 		v[2];
	float 		result;
}butterworthFilter_t;

typedef struct dcFilter_t
{
	float 		w;
	float 		result;
}dcFilter_t;

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

void MAX30100_Init(void);
void Inicio_Heart_Rate(void);
void Inicio_SPO2_HR(void);
void MAX30100_I2C_Reset_FIFO(void);
void MAX30100_I2C_Write(uint8_t, uint8_t, uint8_t);
void MAX30100_I2C_Read(uint8_t, uint8_t, uint8_t);
void MAX30100_I2C_FIFO_Read(uint8_t , uint8_t[]);
void Lectura_FIFO(FIFO_t *);
void Filtrado_DC(float,float*,float*);
void Mean_Median_Filter(float,meanDiffFilter_t*,float*);
void Filtro_PasabajosButterworth(float*,butterworthFilter_t*,float*);
bool detectPulse(float);
void Balance_Intensidades(float, float);
void Resetea_Resultados(pulseoxymeter_t *);
void SetHighresModeEnabled(void);
pulseoxymeter_t Actualizar_Resultados(void);
#endif /* INC_MAX30100_H_ */
