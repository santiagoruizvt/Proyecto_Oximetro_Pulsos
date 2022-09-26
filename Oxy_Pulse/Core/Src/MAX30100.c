/*
 * MAX30100.c
 *
 *  Created on: Aug 26, 2022
 *      Author: ssant
 */
#include "main.h"
#include "stdbool.h"
#include "MAX30100.h"


extern bool flag_HR_ON;
extern I2C_HandleTypeDef hi2c1;

//ADDRESS LISTOS PARA USAR
uint8_t MAX_ADDRESS_WR =0xAE;
uint8_t MAX_ADDRESS_RD =0xAF;
//REGISTROS
uint8_t INTERRUPT_ENABLE=0X01;
uint8_t FIFO_WR_PTR=0x02;
uint8_t OVF_COUNTER=0x03;
uint8_t FIFO_RD_PTR=0x04;
uint8_t FIFO_DATA=0x05;
uint8_t MODE_RG=0x06;
uint8_t SPO2_CONFIGURATION=0x07;
uint8_t LED_CONFIGURATION=0x09;

//VALORES PARA LOS REGISTROS
uint8_t CLEAR=0x00;
/*
uint8_t ILEDS=0xFF;
uint8_t SRATEPULSEW=0x03|0x04;
uint8_t HR_ONLY=0x02;
uint8_t SPO2=0x03;
uint8_t SPO2_SAMPLING_RATE=0x01;
uint8_t PULSE_WIDTH_1600US_ADC_16=0x03
*/

//VARIABLE PARA LECTURA DEL FIFO
uint8_t BYTE_RD=0;
uint8_t NUM_AVAILABLE_SAMPLES=0;
uint8_t BYTE_LSB=0;
uint8_t BYTE_MSB=0;

uint8_t redLEDCurrent=(uint8_t) STARTING_RED_LED_CURRENT;
float lastREDLedCurrentCheck=0;
uint8_t currentPulseDetectorState;
float currentBPM;
float valuesBPM[PULSE_BPM_SAMPLE_SIZE];
float valuesBPMSum;
uint8_t valuesBPMCount=0;
uint8_t bpmIndex=0;
uint32_t lastBeatThreshold;

uint8_t currentPulseDetectorState = PULSE_IDLE;


/**
  * @brief  Transmits in master mode an amount of data in blocking mode.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval HAL status
  */



void Inicio_Heart_Rate(void)
{
	uint8_t buffer=0;
	//HAL_I2C_Master_Transmit(hi2c, DevAddress, pData, Size, Timeout)
	//HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)MAX_ADDRESS_WR, &ILEDS, sizeof(ILEDS), 100); //0x09
	//HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)MAX_ADDRESS_WR, &SRATEPULSEW, sizeof(SRATEPULSEW), 100);  //0x07
	//HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)MAX_ADDRESS_WR, &HR_ONLY, sizeof(HR_ONLY), 100); //0x06
	//HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)MAX_ADDRESS_WR, &LED_CONFIGURATION, 2, 10);
	//HAL_I2C_Master_Transmit(&hi2c1, ILEDS, pData, Size, Timeout)
	//MAX30100_I2C_Write(MAX_ADDRESS_WR,LED_CONFIGURATION,ILEDS);
	//MAX30100_I2C_Write(MAX_ADDRESS_WR,LED_CONFIGURATION,ILEDS);
	//MAX30100_I2C_Write(MAX_ADDRESS_WR,SPO2_SR_LED_PW,SRATEPULSEW);
	MAX30100_I2C_Write(MAX_ADDRESS_WR,MODE_RG,MAX30100_MODE_HR_ONLY);
	buffer=((DEFAULT_SAMPLING_RATE<<2)|DEFAULT_LED_PULSE_WIDTH |0b01000000);
	MAX30100_I2C_Write(MAX_ADDRESS_WR,SPO2_CONFIGURATION,buffer);
	buffer=((DEFAULT_SAMPLING_RATE<<2)|DEFAULT_LED_PULSE_WIDTH |0b01000000);
	MAX30100_I2C_Write(MAX_ADDRESS_WR,SPO2_CONFIGURATION,buffer);
	flag_HR_ON=1;
}

// INICIALIZACION EN MODO HR y SPO2
void Inicio_SPO2_HR(void)
{
	uint8_t buffer=0;
	/*
	MAX30100_I2C_Write(MAX_ADDRESS_WR,MODE,SPO2);
	MAX30100_I2C_Write(MAX_ADDRESS_WR,LED_CONFIGURATION,ILEDS);
	MAX30100_I2C_Write(MAX_ADDRESS_WR,SPO2_SR_LED_PW,SRATEPULSEW);
	*/
	MAX30100_I2C_Write(MAX_ADDRESS_WR,MODE_RG,DEFAULT_OPERATING_MODE);
	buffer=((DEFAULT_SAMPLING_RATE<<2)|DEFAULT_LED_PULSE_WIDTH |0b01000000);
	MAX30100_I2C_Write(MAX_ADDRESS_WR,SPO2_CONFIGURATION,buffer);
	buffer=((STARTING_RED_LED_CURRENT<<4)|DEFAULT_IR_LED_CURRENT);
	MAX30100_I2C_Write(MAX_ADDRESS_WR,LED_CONFIGURATION,buffer);
	flag_HR_ON=0;
}
/*FUNCION PARA LA LECTURA DEL FIFO
 *SE RECIBE UN PUNTERO A UNA VARIABLE TIPO FIFO COMPUESTA POR DOS CAMPOS rawIR y rawRED donde se guardará cada muestra leída. VER HOJA DE DATOS.
*/

void Lectura_FIFO(FIFO_t *FIFO)
{
	//uint8_t NUM_AVAILABLE_SAMPLES=0;
	//uint8_t NUM_SAMPLES_TO_READ=0;
	//uint8_t *p1=(uint8_t*)&FIFO->rawIR;
	//uint8_t *p2=(uint8_t*)&FIFO->rawRED;
	uint16_t aux=0;
	//MAX30100_I2C_Write(MAX_ADDRESS_WR,FIFO_WR_PTR,CLEAR);
	//MAX30100_I2C_Read(MAX_ADDRESS_RD,FIFO_WR_PTR,BYTE_RD);
	HAL_I2C_Master_Transmit(&hi2c1, MAX_ADDRESS_WR, &FIFO_WR_PTR, sizeof(FIFO_WR_PTR), 10);
	HAL_I2C_Master_Receive(&hi2c1, MAX_ADDRESS_RD, &BYTE_RD, sizeof(BYTE_RD), 10);
	FIFO_RD_PTR=BYTE_RD;
	//NUM_AVAILABLE_SAMPLES=FIFO_WR_PTR–FIFO_RD_PTR;
//	NUM_AVAILABLE_SAMPLES=0x10;  //SETEO EL NUMERO DE MUESTRAS EN 16, PROBLEMAS AL COMPILAR CON LA LINEA DE ARRIBA
	//NUM_SAMPLES_TO_READ=NUM_AVAILABLE_SAMPLES;
	HAL_I2C_Master_Transmit(&hi2c1, MAX_ADDRESS_WR, &FIFO_DATA, sizeof(FIFO_DATA), 10);
//for(int i=0;i<NUM_SAMPLES_TO_READ;i++)
//	{
		HAL_I2C_Master_Receive(&hi2c1, MAX_ADDRESS_RD, &BYTE_RD, sizeof(BYTE_RD), 10);
		BYTE_MSB=BYTE_RD;
		HAL_I2C_Master_Receive(&hi2c1, MAX_ADDRESS_RD, &BYTE_RD, sizeof(BYTE_RD), 10);
		BYTE_LSB=BYTE_RD;

		// BUSCO ACOMODAR LOS DATOS QUE LEO DE A BYTES EN UNA VARIABLE FIFO.IRraw de 16bits
		aux=BYTE_MSB;
		FIFO->rawIR=((aux<<8)|BYTE_LSB);

		HAL_I2C_Master_Receive(&hi2c1, MAX_ADDRESS_RD, &BYTE_RD, sizeof(BYTE_RD), 10);
		BYTE_MSB=BYTE_RD;
		HAL_I2C_Master_Receive(&hi2c1, MAX_ADDRESS_RD, &BYTE_RD, sizeof(BYTE_RD), 10);
		BYTE_LSB=BYTE_RD;

		aux=BYTE_MSB;
		FIFO->rawRED=((aux<<8)|BYTE_LSB);
	}



void Filtrado_DC(uint16_t *raw_values,float *ptrprev_w, float *output_signal)
{
	float x=0;
	float w=0;
	float alpha=0.95;
	float prev_w=*ptrprev_w;
	float result=0;
	x=*raw_values;

	//for(int i=0;i<=15;i++){

		w = x + alpha * prev_w;
		//(float) output_signal = w - prev_w;
		result = w - prev_w;

		*ptrprev_w=w;
		*output_signal=result;
	//}
}

void Mean_Median_Filter(float M,meanDiffFilter_t *filterValues,float *ir_meanfiltrado)
{
	float avg = 0;
	float aux = 0;
	//filterValues->index = 0;
	//filterValues->sum = 0;
	//filterValues->count = 0;
	//for(int i=0;i<=15;i++)
	//{
		filterValues->sum -= filterValues->values[filterValues->index];
		filterValues->values[filterValues->index] = M;
		filterValues->sum += filterValues->values[filterValues->index];
		filterValues->index++;
		filterValues->index = filterValues->index % MEAN_FILTER_SIZE;
		if(filterValues->count < MEAN_FILTER_SIZE)
		filterValues->count++;

		avg = filterValues->sum / filterValues->count;
		aux = avg-M;
		*ir_meanfiltrado=aux;
	//}
}

void Filtro_PasabajosButterworth(float* x,butterworthFilter_t* filterResult ,float*ir_lpbfiltrado)
{
	float aux=*x;
	filterResult->v[0] = filterResult->v[1];

	  //Fs = 100Hz and Fc = 10Hz
	filterResult->v[1] = (2.452372752527856026e-1 * aux) + (0.50952544949442879485 * filterResult->v[0]);

	filterResult->result = filterResult->v[0] + filterResult->v[1];

	*ir_lpbfiltrado=filterResult->result;
}

bool detectPulse(float sensor_value,pulseoxymeter_t *result)
{
	//SOLO LA PRIMERA VEZ QUE SE USA LA FUNCIÓN LOS STATICS VALEN 0
	static float prev_sensor_value = 0;
	static uint8_t values_went_down = 0;
	static uint32_t currentBeat = 0;
	static uint32_t lastBeat = 0;

	  if(sensor_value > PULSE_MAX_THRESHOLD)
	  {
	    currentPulseDetectorState = PULSE_IDLE;
	    prev_sensor_value = 0;
	    lastBeat = 0;
	    currentBeat = 0;
	    values_went_down = 0;
	    lastBeatThreshold = 0;
	    return false;
	  }
	    switch(currentPulseDetectorState)
	     {
	       case PULSE_IDLE:
	         if(sensor_value >= PULSE_MIN_THRESHOLD) {
	           currentPulseDetectorState = PULSE_TRACE_UP;
	           values_went_down = 0;
	         }
	         break;

	       case PULSE_TRACE_UP:
	         if(sensor_value > prev_sensor_value)
	         {
	           currentBeat = HAL_GetTick();
	           lastBeatThreshold = sensor_value;
	           result->lastBeatThreshold = lastBeatThreshold;
	         }
	         else
	         {
/*
	           if(debug == true)
	           {
	             Serial.print("Peak reached: ");
	             Serial.print(sensor_value);
	             Serial.print(" ");
	             Serial.println(prev_sensor_value);
	           }*/

	           uint32_t beatDuration = currentBeat - lastBeat;
	           lastBeat = currentBeat;

	           float rawBPM = 0;
	           if(beatDuration > 0)
	             rawBPM = 60000.0 / (float)beatDuration;
	           //if(debug == true)
	             //Serial.println(rawBPM);

	           //This method sometimes glitches, it's better to go through whole moving average everytime
	           //IT's a neat idea to optimize the amount of work for moving avg. but while placing, removing finger it can screw up
	           //valuesBPMSum -= valuesBPM[bpmIndex];
	           //valuesBPM[bpmIndex] = rawBPM;
	           //valuesBPMSum += valuesBPM[bpmIndex];

	           valuesBPM[bpmIndex] = rawBPM;
	           valuesBPMSum = 0;
	           for(int i=0; i<PULSE_BPM_SAMPLE_SIZE; i++)
	           {
	             valuesBPMSum += valuesBPM[i];
	           }

	           /*if(debug == true)
	           {
	             Serial.print("CurrentMoving Avg: ");
	             for(int i=0; i<PULSE_BPM_SAMPLE_SIZE; i++)
	             {
	               Serial.print(valuesBPM[i]);
	               Serial.print(" ");
	             }

	             Serial.println(" ");
	           }*/

	           bpmIndex++;
	           bpmIndex = bpmIndex % PULSE_BPM_SAMPLE_SIZE;

	           if(valuesBPMCount < PULSE_BPM_SAMPLE_SIZE)
	             valuesBPMCount++;

	           currentBPM = valuesBPMSum / valuesBPMCount;
	           result->heartBPM = currentBPM;
	           /*if(debug == true)
	           {
	             Serial.print("AVg. BPM: ");
	             Serial.println(currentBPM);
	           }*/


	           currentPulseDetectorState = PULSE_TRACE_DOWN;

	           return true;
	         }
	         break;

	       case PULSE_TRACE_DOWN:
	         if(sensor_value < prev_sensor_value)
	         {
	           values_went_down++;
	         }


	         if(sensor_value < PULSE_MIN_THRESHOLD)
	         {
	           currentPulseDetectorState = PULSE_IDLE;
	         }
	         break;
	     }

	     prev_sensor_value = sensor_value;
	     return false;

}

void Balance_Intensidades(float redLedDC, float IRLedDC)
{
	uint8_t buffer=0;
	uint8_t aux1=0;
	uint8_t aux2=0;
    aux1=(uint8_t)redLedDC;
    aux2=(uint8_t)IRLedDC;

	if( HAL_GetTick() - lastREDLedCurrentCheck >= RED_LED_CURRENT_ADJUSTMENT_MS)
	  {
	    //Serial.println( redLedDC - IRLedDC );
	    if( IRLedDC - redLedDC > MAGIC_ACCEPTABLE_INTENSITY_DIFF && redLEDCurrent < MAX30100_LED_CURRENT_50MA)
	    {
	      redLEDCurrent++;
	      buffer=((aux1<<4)|aux2);
	      MAX30100_I2C_Write(MAX_ADDRESS_WR,LED_CONFIGURATION,(uint8_t)buffer);
	      //if(debug == true)
	        //Serial.println("RED LED Current +");
	    }
	    else if(redLedDC - IRLedDC > MAGIC_ACCEPTABLE_INTENSITY_DIFF && redLEDCurrent > 0)
	    {
	      redLEDCurrent--;
	      buffer=((aux1<<4)|aux2);
	      MAX30100_I2C_Write(MAX_ADDRESS_WR,LED_CONFIGURATION,(uint8_t)buffer);
	      //if(debug == true)
	        //Serial.println("RED LED Current -");
	    }

	    lastREDLedCurrentCheck = HAL_GetTick();
	  }
}

void Resetea_Resultados(pulseoxymeter_t *result,meanDiffFilter_t *meanfilter,float currentSaO2Value)
{
	result->pulseDetected=false;
	result->heartBPM=0.0;
	result->irCardiogram=0.0;
	result->irDcValue=0.0;
	result->redDcValue=0.0;
	result->SaO2=currentSaO2Value;
	result->lastBeatThreshold=0;
	result->dcFilteredIR=0.0;
	result->dcFilteredRed=0.0;
	for(int i=0;i<MEAN_FILTER_SIZE;i++)
	{
		meanfilter->values[i]=0;
	}
	meanfilter->index=0;
	meanfilter->sum=0;
	meanfilter->count=0;
}

void MAX30100_I2C_Write(uint8_t address, uint8_t reg, uint8_t data)
{
	uint8_t dt[2];
	dt[0] = reg;
	dt[1] = data;
	HAL_I2C_Master_Transmit(&hi2c1, address, dt, 2, 10);
}

void MAX30100_I2C_Read(uint8_t address, uint8_t reg, uint8_t data)
{
	uint8_t dt[2];
	dt[0] = reg;
	dt[1] = data;
	HAL_I2C_Master_Receive(&hi2c1, address, dt,2, 10);
}
