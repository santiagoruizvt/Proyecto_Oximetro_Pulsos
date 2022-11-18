/*
 * MAX30100.c
 *
 *  Created on: Aug 26, 2022
 *      Author: ssant
 */
#include "main.h"
#include "math.h"
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

uint8_t IRLedCurrent=0;
uint8_t redLEDCurrent=0;
float lastREDLedCurrentCheck=0;
uint8_t currentPulseDetectorState;
float currentBPM;
float valuesBPM[PULSE_BPM_SAMPLE_SIZE];
float valuesBPMSum;
uint8_t valuesBPMCount=0;
uint8_t bpmIndex=0;
uint32_t lastBeatThreshold;
uint16_t samplesRecorded=0;
uint16_t pulsesDetected=0;

float IRprev_w=0;
float REDprev_w=0;
float ir_dcfiltrado=0;
float red_dcfiltrado=0;
float ir_meanfiltrado=0;
float ir_lpbfiltrado=0;
float irACValueSqSum=0;
float redACValueSqSum=0;
float currentSaO2Value=0;
float ratioRMS=0;

uint8_t 			currentPulseDetectorState = PULSE_IDLE;
butterworthFilter_t lpbFilterIR;
meanDiffFilter_t 	meanDiffIR;
dcFilter_t 			dcFilterIR;
dcFilter_t 			dcFilterRed;

//*************************************************************
//Función: 			MAX30100_Init
//Descripción: 		Inicialización del módulo MAX30100
//Parámetros: 		void
//Valor devuelto: 	void
//*************************************************************

void MAX30100_Init(void)
{
	uint8_t buffer=0;
	uint8_t aux=0;

	currentPulseDetectorState = PULSE_IDLE;

	//				SETEO DEL MODO
	//////////////////////////////////////////////////////////////
	MAX30100_I2C_Read(MAX_ADDRESS_RD,MAX30100_MODE_CONF,buffer);
	aux=(buffer&0xF8)|DEFAULT_OPERATING_MODE;
	MAX30100_I2C_Write(MAX_ADDRESS_WR,MODE_RG,aux);
	//////////////////////////////////////////////////////////////

	//				SETEO SAMPLING_RATE
	//////////////////////////////////////////////////////////////
	MAX30100_I2C_Read(MAX_ADDRESS_RD, MAX30100_SPO2_CONF, buffer);
	aux=(buffer&0xE3)|(DEFAULT_SAMPLING_RATE<<2);
	MAX30100_I2C_Write(MAX_ADDRESS_WR, MAX30100_SPO2_CONF, aux);
	//////////////////////////////////////////////////////////////

	//				SETEO LED_Pulse_Width
	/////////////////////////////////////////////////////////////
	MAX30100_I2C_Read(MAX_ADDRESS_RD,MAX30100_SPO2_CONF, buffer);
	//aux=(buffer&0xFC)|(DEFAULT_LED_PULSE_WIDTH);
	aux=0x47;
	MAX30100_I2C_Write(MAX_ADDRESS_WR, MAX30100_SPO2_CONF, aux);
	////////////////////////////////////////////////////////////7

	redLEDCurrent = (uint8_t) STARTING_RED_LED_CURRENT;
	lastREDLedCurrentCheck=0;
	IRLedCurrent=DEFAULT_IR_LED_CURRENT;

	//				SETEO_CORRIENTES_LEDs
	//////////////////////////////////////////////////////////////
	buffer=((redLEDCurrent << 4) | IRLedCurrent );
	MAX30100_I2C_Write(MAX_ADDRESS_WR, LED_CONFIGURATION, buffer);
	//////////////////////////////////////////////////////////////

	//				SETEO_HighresModeEnabled
	//////////////////////////////////////////////////////////////
	//SetHighresModeEnabled();

	dcFilterIR.w = 0;
	dcFilterIR.result = 0;

	dcFilterRed.w = 0;
	dcFilterRed.result = 0;


	lpbFilterIR.v[0] = 0;
	lpbFilterIR.v[1] = 0;
	lpbFilterIR.result = 0;

	meanDiffIR.index = 0;
	meanDiffIR.sum = 0;
	meanDiffIR.count = 0;


	valuesBPM[0] = 0;
	valuesBPMSum = 0;
	valuesBPMCount = 0;
	bpmIndex = 0;


	irACValueSqSum = 0;
	redACValueSqSum = 0;
	samplesRecorded = 0;
	pulsesDetected = 0;
	currentSaO2Value = 0;

	lastBeatThreshold = 0;

}

pulseoxymeter_t Actualizar_Resultados(void)
{
	pulseoxymeter_t result;
	FIFO_t fifo;

	Resetea_Resultados(&result);

	Lectura_FIFO(&fifo);

	Filtrado_DC((float)fifo.rawIR,&IRprev_w,&ir_dcfiltrado);
	Filtrado_DC((float)fifo.rawRED,&REDprev_w,&red_dcfiltrado);

	Mean_Median_Filter(ir_dcfiltrado,&meanDiffIR,&ir_meanfiltrado);
	Filtro_PasabajosButterworth(&ir_meanfiltrado,&lpbFilterIR,&ir_lpbfiltrado);

	irACValueSqSum  +=ir_dcfiltrado * ir_dcfiltrado;
	redACValueSqSum +=red_dcfiltrado * red_dcfiltrado;
	samplesRecorded++;

	if( detectPulse( ir_lpbfiltrado, &result ) && samplesRecorded > 0 )
	{
		result.pulseDetected=true;
		pulsesDetected++;

	    ratioRMS = log( sqrt(redACValueSqSum/samplesRecorded) ) / log( sqrt(irACValueSqSum/samplesRecorded) );

	    //if( debug == true )
	      //{
	        //Serial.print("RMS Ratio: ");
	        //Serial.println(ratioRMS);
	      //}
	    currentSaO2Value = 115.0 - 18.0 * ratioRMS;
	    result.SaO2 = currentSaO2Value;

	    if( pulsesDetected % RESET_SPO2_EVERY_N_PULSES == 0)
	    {
	    	irACValueSqSum = 0;
	        redACValueSqSum = 0;
	        samplesRecorded = 0;
	     }
	}

	Balance_Intensidades( REDprev_w, IRprev_w);

	result.heartBPM = currentBPM;
	result.irCardiogram = lpbFilterIR.result;
	result.irDcValue = IRprev_w;
	result.redDcValue = REDprev_w;
	result.lastBeatThreshold = lastBeatThreshold;
	result.dcFilteredIR = ir_dcfiltrado;
	result.dcFilteredRed = red_dcfiltrado;

	return result;
}

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

//*************************************************************
//Función: 			Lectura_FIFO
//Descripción: 		Lectura de los valores de IR y REDLED desde
//					el registro FIFO del MAX30100
//Parámetros: 		FIFO_t* puntero a estructura FIFO
//Valor devuelto: 	void
//*************************************************************

void Lectura_FIFO(FIFO_t *FIFO)
{
	uint8_t data[4]={0};

	HAL_I2C_Master_Transmit(&hi2c1, MAX_ADDRESS_WR, &FIFO_DATA, sizeof(FIFO_DATA), 10);

	MAX30100_I2C_FIFO_Read(MAX_ADDRESS_RD,data);

	FIFO->rawIR=(data[0]|data[1]);
	FIFO->rawRED=(data[2]|data[3]);
}

//*************************************************************
//Función: 			Filtrado_DC
//Descripción: 		Filtrado para quitar la señal continua
//					presente en los valores leídos
//Parámetros: 		float datos del FIFO
//					float* valor previo w
//					float* dato filtrado
//Valor devuelto: 	void
//*************************************************************

void Filtrado_DC(float raw_values,float *ptrprev_w, float *output_signal)
{
	float x=0;
	float w=0;
	float alpha=0.95;
	float prev_w=*ptrprev_w;
	float result=0;

		x=raw_values;

		w = x + alpha * prev_w;
		result = w - prev_w;

		*ptrprev_w=w;
		*output_signal=result;
}

//*************************************************************
//Función: 			Mean_Median_Filter
//Descripción: 		Filtro de mediana para reducir el ruido y
//					limpiar la señal
//Parámetros: 		float  datos de la salida del Filtrado_DC
//					meanDiffFilter_t* puntero a estructura
//					float* puntero a dato filtrado resultante
//Valor devuelto: 	void
//*************************************************************

void Mean_Median_Filter(float M,meanDiffFilter_t *filterValues,float *ir_meanfiltrado)
{
		float avg = 0;
		float aux = 0;
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
}

//*************************************************************
//Función: 			Filtro_PasabajosButterworth
//Descripción: 		Filtro pasabajos butterworth implementado
//					con una ecuación en diferencia
//Parámetros: 		float  datos de la salida del Median_Filter
//					butterworthFilter_t* puntero a estructura
//					float* puntero a dato filtrado resultante
//Valor devuelto: 	void
//*************************************************************

void Filtro_PasabajosButterworth(float* x,butterworthFilter_t* filterResult ,float*ir_lpbfiltrado)
{
	float aux=*x;
	filterResult->v[0] = filterResult->v[1];

	  //Fs = 100Hz and Fc = 10Hz
	filterResult->v[1] = (2.452372752527856026e-1 * aux) + (0.50952544949442879485 * filterResult->v[0]);
	  //Fs = 100Hz and Fc = 4Hz

	//filterResult->v[1] = (1.367287359973195227e-1 * aux) + (0.72654252800536101020 * filterResult->v[0]);

	filterResult->result = filterResult->v[0] + filterResult->v[1];

	*ir_lpbfiltrado=filterResult->result;
}

//*************************************************************
//Función: 			detectPulse
//Descripción: 		Máquina de estados para la detección del pulso
//Parámetros: 		float  datos de la salida del LPF
//					pulseoxymeter_t* puntero a estructura result
//Valor devuelto: 	void
//*************************************************************
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
	         }
	         else
	         {

	           uint32_t beatDuration = currentBeat - lastBeat;
	           lastBeat = currentBeat;

	           float rawBPM = 0;
	           if(beatDuration > 0)
	             rawBPM = 60000.0 / (float)beatDuration;

	           valuesBPM[bpmIndex] = rawBPM;
	           valuesBPMSum = 0;
	           for(int i=0; i<PULSE_BPM_SAMPLE_SIZE; i++)
	           {
	             valuesBPMSum += valuesBPM[i];
	           }

	           bpmIndex++;
	           bpmIndex = bpmIndex % PULSE_BPM_SAMPLE_SIZE;

	           if(valuesBPMCount < PULSE_BPM_SAMPLE_SIZE)
	             valuesBPMCount++;

	           currentBPM = valuesBPMSum / valuesBPMCount;

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

//*************************************************************
//Función: 			Balance_Intensidades
//Descripción: 		Corrección de las intensidades de los LEDS
//					para evitar la saturación de la lectura
//Parámetros: 		float datos del led rojo
//					float datos del led IR
//Valor devuelto: 	void
//*************************************************************
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

//FUNCIÓN NO UTILIZADA
//*************************************************************
//Función: 			SetHighresModeEnabled
//Descripción: 		Modificacion del registro HI_RES_EN
//Parámetros: 		void
//Valor devuelto: 	void
//*************************************************************
void SetHighresModeEnabled(void)
{
	uint8_t buffer=0;
	uint8_t aux=00000001;

	MAX30100_I2C_Read(MAX_ADDRESS_RD, SPO2_CONFIGURATION, buffer);
	if(buffer>aux)
	{
		buffer=(buffer|MAX30100_SPO2_HI_RES_EN);
		MAX30100_I2C_Write(MAX_ADDRESS_WR,SPO2_CONFIGURATION,buffer);
	}
	else
	{
		buffer=(buffer & ~MAX30100_SPO2_HI_RES_EN);
		MAX30100_I2C_Write(MAX_ADDRESS_WR,SPO2_CONFIGURATION,buffer);
	}
}

//*************************************************************
//Función: 			Resetea_Resultados
//Descripción: 		Resetea la estructura result para comenzar
//					o realizar una nueva lectura
//Parámetros: 		pulseoxymeter_t* puntero a estructura result
//Valor devuelto: 	void
//*************************************************************

void Resetea_Resultados(pulseoxymeter_t *result)
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
}

//*************************************************************
//Función: 			MAX30100_I2C_Write
//Descripción: 		Escritura de un buffer a traves del bus I2C
//Parámetros: 		uint8_t dirección del dispositivo
//					uint8_t registro a escribir
//					uint8_t data a escribir
//Valor devuelto: 	void
//*************************************************************

void MAX30100_I2C_Write(uint8_t address, uint8_t reg, uint8_t data)
{
	uint8_t dt[2];
	dt[0] = reg;
	dt[1] = data;
	HAL_I2C_Master_Transmit(&hi2c1, address, dt, 2, 10);
}

//*************************************************************
//Función: 			MAX30100_I2C_Read
//Descripción: 		Lectura de un buffer a traves del bus I2C
//Parámetros: 		uint8_t dirección del dispositivo
//					uint8_t registro a escribir
//					uint8_t data a escribir
//Valor devuelto: 	void
//*************************************************************

void MAX30100_I2C_Read(uint8_t address, uint8_t reg, uint8_t data)
{
	uint8_t dt[2];
	dt[0] = reg;
	dt[1] = data;
	HAL_I2C_Master_Receive(&hi2c1, address, dt,2, 10);
}

//*************************************************************
//Función: 			MAX30100_I2C_FIFO_Read
//Descripción: 		Lectura del FIFO del MAX30100, se leen 2 bytes
//					por cada LED
//Parámetros: 		void
//Valor devuelto: 	void
//*************************************************************
void MAX30100_I2C_FIFO_Read(uint8_t address, uint8_t dt[4])
{
	HAL_I2C_Master_Receive(&hi2c1, address, dt,4, 10);
}

//*************************************************************
//Función: 			MAX30100_I2C_Reset_FIFO
//Descripción: 		Reseteo el registro FIFO
//Parámetros: 		void
//Valor devuelto: 	void
//*************************************************************
void MAX30100_I2C_Reset_FIFO(void)
{
	MAX30100_I2C_Write(MAX_ADDRESS_WR, (uint8_t) MAX30100_FIFO_WRITE, (uint8_t)0);
	MAX30100_I2C_Write(MAX_ADDRESS_WR, (uint8_t) MAX30100_FIFO_OVERFLOW_COUNTER, (uint8_t)0);
	MAX30100_I2C_Write(MAX_ADDRESS_WR, (uint8_t) MAX30100_FIFO_READ, (uint8_t)0);
}
