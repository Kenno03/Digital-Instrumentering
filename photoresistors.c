#include "stm32f30x_conf.h" // STM32 config
#include "30010_io.h" // Input/output library for this course
#include "photoresistors.h"


void ADC_setup_5_photo(){
	// This function configures the ADC clock, the GPIO pins connected to the mbed potentiometers, and sets the desired ADC settings

	// Enable clocks
	RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div8);

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC12,ENABLE);

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

	// Initialize GPIO pins as analog inputs
	GPIO_InitTypeDef GPIO_InitStructAll; // Define typedef struct for setting pin
	// Sets PA6(ch10) to input
	GPIO_StructInit(&GPIO_InitStructAll); // Initialize GPIO struct
	GPIO_InitStructAll.GPIO_Mode = GPIO_Mode_AN; // Set as input
	GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_6; // Set so the configuration is on pin 6
	GPIO_Init(GPIOA, &GPIO_InitStructAll); // Setup of GPIO with the settings chosen

	// Sets PA7(ch15) to input
	GPIO_StructInit(&GPIO_InitStructAll); // Initialize GPIO struct
	GPIO_InitStructAll.GPIO_Mode = GPIO_Mode_AN; // Set as input
	GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_7; // Set so the configuration is on pin 7
	GPIO_Init(GPIOA, &GPIO_InitStructAll); // Setup of GPIO with the settings chosen

	// Sets PB11(ch14) as input
	GPIO_StructInit(&GPIO_InitStructAll); // Initialize GPIO struct
	GPIO_InitStructAll.GPIO_Mode = GPIO_Mode_AN; // Set as input
	GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_11; // Set so the configuration is on pin 11
	GPIO_Init(GPIOB, &GPIO_InitStructAll); // Setup of GPIO with the settings chosen

	// Sets PC2(ch8) as input
	GPIO_StructInit(&GPIO_InitStructAll); // Initialize GPIO struct
	GPIO_InitStructAll.GPIO_Mode = GPIO_Mode_AN; // Set as input
	GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_2; // Set so the configuration is on pin 2
	GPIO_Init(GPIOC, &GPIO_InitStructAll); // Setup of GPIO with the settings chosen

	// Sets PC3(ch9) as input
	GPIO_StructInit(&GPIO_InitStructAll); // Initialize GPIO struct
	GPIO_InitStructAll.GPIO_Mode = GPIO_Mode_AN; // Set as input
	GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_3; // Set so the configuration is on pin 3
	GPIO_Init(GPIOC, &GPIO_InitStructAll); // Setup of GPIO with the settings chosen


	// Configure the ADC with the desired settings
	ADC_InitTypeDef ADC_InitStructAll;

	ADC_StructInit(&ADC_InitStructAll);
	ADC_InitStructAll.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructAll.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructAll.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructAll.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0;
	ADC_InitStructAll.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;
	ADC_InitStructAll.ADC_NbrOfRegChannel = 1;
	ADC_Init(ADC1, &ADC_InitStructAll);

	// Enable ADC 1
	ADC_Cmd(ADC1, ENABLE);

	while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_RDY)){}

	// set internal reference voltage source
	ADC_VoltageRegulatorCmd(ADC1,ENABLE);
	//Wait for at least 10uS before continuing...
	for(uint32_t i = 0; i<10000;i++);

	// Run the internal calibration
	ADC_Cmd(ADC1,DISABLE);
	while(ADC_GetDisableCmdStatus(ADC1)){} // wait for disable of ADC
	ADC_SelectCalibrationMode(ADC1,ADC_CalibrationMode_Single);
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1)){}
	for(uint32_t i = 0; i<100;i++);

	// Enable the ADC and setup reference voltage
	ADC_Cmd(ADC1,ENABLE);
	while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_RDY)){}

	ADC_VrefintCmd(ADC1,ENABLE); // setup reference voltage to channel 18
	for(uint32_t i = 0; i<10000;i++); // Wait
}

uint16_t ADC1_measure_channel(uint8_t ch){
	// This function measures the voltage of a given channel and outputs a 16-bit digital value of the ADC reading
	// The function is used for the 5 photoresistors

	uint16_t x = 0;

	// Choose channel to read from
    uint32_t adc_channel;

    // case 1 - 5 corresponds to the following pins:
	// PC2(ch8), PC3(ch9), PA6(ch10), PB11(ch14), PA7(ch15)

    switch(ch)
    {
        case 1:  adc_channel = ADC_Channel_8;  break;
        case 2:  adc_channel = ADC_Channel_9;  break;
        case 3: adc_channel = ADC_Channel_10; break;
        case 4: adc_channel = ADC_Channel_14; break;
        case 5: adc_channel = ADC_Channel_15; break;
        default: return 0;    // If invalid channel is given, return 0
    }

    ADC_RegularChannelConfig(ADC1, adc_channel, 1, ADC_SampleTime_1Cycles5);

	// Start the measurement and wait
	ADC_StartConversion(ADC1); // Start ADC read
	while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == 0); // Wait for ADC read

	// Save the value
	x = ADC_GetConversionValue(ADC1); // Read the ADC value

	// return the measured digital value
	return x;
}
