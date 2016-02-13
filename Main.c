#include "stm32f30x.h"																										// Device header
//#include "PinAssignments.h"
//int GetADCData(uint8_t);

//static __IO uint32_t TimeDelay;																									// Global variable for SysTick delay function
//__IO uint16_t  ADC1ConvertedValue = 0, ADC1ConvertedVoltage = 0, calibration_value = 0;
//__IO uint32_t TimingDelay = 0;
///*
//Description:	Creates a configurable delay using the SysTick timer interrupt
//Parameters:		Time - Length of delay (ms)
//Returns:			-
//*/
//void Delay(__IO uint32_t Time)
//{
//	TimeDelay = Time;
//	while(TimeDelay != 0);
//}

///*
//Description:	Interrupt handler for the SysTick timer
//							1. Decrements the TimeDelay variable for the Delay() function
//Parameters:		-
//Returns:			-
//*/
//void SysTick_Handler(void)
//{
//	if(TimeDelay != 0)
//	{
//		TimeDelay--;
//	}
//}

//void ConfigureTimers(void)
//{
//	// Configure SysTick timer to generate interrupts every 1ms
//	if(SysTick_Config(SystemCoreClock / 1000))
//	{
//		// Capture SysTick initialisation error
//		while(1);
//	}
//}
//		
//void ConfigureGPIO(void)
//{
//	GPIO_InitTypeDef GPIO_InitStruct;															//GPIO init structure
//	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE); 					//Enable clock to GPIOA pins
//	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE); 					//Enable clock to GPIOB pins

//	//Configure Status LEDs  
//	GPIO_InitStruct.GPIO_Pin = STATUS_R_PIN | STATUS_G_PIN | STATUS_B_PIN;
//	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;										
//	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;							
//	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
//	GPIO_Init(GPIOB, &GPIO_InitStruct);
//	
//	//Initialize all status LED's to be OFF
//	GPIO_WriteBit(GPIOB, STATUS_B_PIN, 1);
//	GPIO_WriteBit(GPIOB, STATUS_R_PIN, 1);
//	GPIO_WriteBit(GPIOB, STATUS_G_PIN, 1);
//	
//	//Configure GPIO pins for I2C2 (Alternate Function mode, AF)
//	//Pins are on GPIOA
//	GPIO_InitStruct.GPIO_Pin = I2C2_SCL | I2C2_SDA;
//	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;										
//	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;							
//	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; //external pull-up resistors used
//	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
//	GPIO_Init(GPIOA, &GPIO_InitStruct);
//	
//	//Configure GYRO Interrupt Pin (INPUT)
//	//GPIOB 
//	GPIO_InitStruct.GPIO_Pin = GYRO_DRDY;
//	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;										
//	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
//	GPIO_Init(GPIOB, &GPIO_InitStruct);
//}

//void ConfigureClock(void)
//{
//}

//void ConfigureADC(void)
//{
//	ADC_InitTypeDef       ADC_InitStruct;
//	ADC_CommonInitTypeDef ADC_CommonInitStruct;

//  /*!< At this stage the microcontroller clock setting is already configured, 
//       this is done through SystemInit() function which is called from startup
//       file (startup_stm32f30x.s) before to branch to application main.
//       To reconfigure the default setting of SystemInit() function, refer to
//       system_stm32f30x.c file
//     */ 

//  /* Configure the ADC clock */
//  RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div2);
//  
//  /* Enable ADC1 clock */
//  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC12, ENABLE);
// 
//  ADC_StructInit(&ADC_InitStruct);

//  /* Calibration procedure */  
//	ADC_VoltageRegulatorCmd(ADC1, ENABLE);
//	
//	//Need to wait a bit after turning on the power supply. 1ms is plenty 
//	Delay(1);
//  
//  ADC_SelectCalibrationMode(ADC1, ADC_CalibrationMode_Single);
//  ADC_StartCalibration(ADC1);
//  
//  while(ADC_GetCalibrationStatus(ADC1) != RESET );
//  calibration_value = ADC_GetCalibrationValue(ADC1);
//     
//  ADC_CommonInitStruct.ADC_Mode = ADC_Mode_Independent;                                                                    
//  ADC_CommonInitStruct.ADC_Clock = ADC_Clock_AsynClkMode;                    
//  ADC_CommonInitStruct.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;             
//  ADC_CommonInitStruct.ADC_DMAMode = ADC_DMAMode_OneShot;                  
//  ADC_CommonInitStruct.ADC_TwoSamplingDelay = 0;          
//  ADC_CommonInit(ADC1, &ADC_CommonInitStruct);
//  
//  ADC_InitStruct.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Enable;
//  ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b; 
//  ADC_InitStruct.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0;         
//  ADC_InitStruct.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;
//  ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
//  ADC_InitStruct.ADC_OverrunMode = ADC_OverrunMode_Disable;   
//  ADC_InitStruct.ADC_AutoInjMode = ADC_AutoInjec_Disable;  
//  ADC_InitStruct.ADC_NbrOfRegChannel = 1;
//  ADC_Init(ADC1, &ADC_InitStruct);
//  
//  /* ADC1 regular channel2 configuration */ 
//  ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_7Cycles5);
//}




void Init_TxMes(CanTxMsg *TxMessage){
          /* Transmit INITIALIZATION*/
     TxMessage->IDE = CAN_ID_STD;
     TxMessage->DLC = 2; 
     TxMessage->StdId = 0x321;
     TxMessage->RTR = CAN_RTR_DATA;
}

void Init_RxMes(CanRxMsg *RxMessage)
{
  uint8_t i = 0;

  RxMessage->StdId = 0;
  RxMessage->IDE = CAN_ID_STD;
  RxMessage->DLC = 0;
  RxMessage->FMI = 0;
  for (i = 0; i < 8; i++)
  {
    RxMessage->Data[i] = 0;
  }
}

/* 
 * things to test: whether code works on other GPIO pins.
 */

void ConfigureCAN(void)
{
	
	
	GPIO_InitTypeDef GPIO_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	CAN_InitTypeDef CAN_InitStruct;
	CAN_FilterInitTypeDef CAN_FilterInitStruct;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE); 
   
	/* Configure CAN1 RX pin */
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStruct);
//	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_9);
    
  /* Configure CAN1 TX pin */
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStruct); 
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_9);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_9);
		
  /* Enable the CAN global Interrupt */
  NVIC_InitStruct.NVIC_IRQChannel = CAN1_SCE_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);
    
  NVIC_InitStruct.NVIC_IRQChannel = CAN1_RX1_IRQn;
  NVIC_Init(&NVIC_InitStruct);
     
  /* Configure CAN1 **************************************************/  
  
	CAN_DeInit(CAN1);
	
	/* Struct init*/
  CAN_StructInit(&CAN_InitStruct);
  CAN_InitStruct.CAN_TTCM = DISABLE;
  CAN_InitStruct.CAN_ABOM = DISABLE;
  CAN_InitStruct.CAN_AWUM = DISABLE;
  CAN_InitStruct.CAN_NART = DISABLE;
  CAN_InitStruct.CAN_RFLM = DISABLE;
  CAN_InitStruct.CAN_TXFP = ENABLE;
  CAN_InitStruct.CAN_Mode = CAN_Mode_Normal;   
  CAN_InitStruct.CAN_SJW = CAN_SJW_1tq;  
  CAN_InitStruct.CAN_BS1 = CAN_BS1_3tq;
  CAN_InitStruct.CAN_BS2 = CAN_BS2_2tq;
  CAN_InitStruct.CAN_Prescaler =6;
  /*Initializes the CAN1 */
  CAN_Init(CAN1,&CAN_InitStruct);
     
     
  /*CAN1 filter init */
  CAN_FilterInitStruct.CAN_FilterNumber = 1;
  CAN_FilterInitStruct.CAN_FilterMode = CAN_FilterMode_IdMask;
  CAN_FilterInitStruct.CAN_FilterScale = CAN_FilterScale_32bit;
  CAN_FilterInitStruct.CAN_FilterIdHigh = 0x6420;
  CAN_FilterInitStruct.CAN_FilterIdLow = 0x0000;
  CAN_FilterInitStruct.CAN_FilterMaskIdHigh = 0x0000;
  CAN_FilterInitStruct.CAN_FilterMaskIdLow = 0x0000;
  CAN_FilterInitStruct.CAN_FilterFIFOAssignment = 0;
  CAN_FilterInitStruct.CAN_FilterActivation = ENABLE;
  CAN_FilterInit(&CAN_FilterInitStruct);
 
}



//void ConfigureSPI(void)
//{
//}

//void ConfigureI2C(void)
//{
//	I2C_InitTypeDef I2C_InitStruct;
//	NVIC_InitTypeDef NVIC_InitStruct;

//	//Enable the clock to I2C2
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
//	
//	//Reset I2C2
//	RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C2, ENABLE);
//	RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C2, DISABLE);
//	
//	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
//	I2C_InitStruct.I2C_OwnAddress1 = OWN_I2C_ADDRESS;
//	I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
//	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
//	I2C_InitStruct.I2C_Timing = 0xC062121F;                                   //this is the wrong value

//	I2C_StructInit(&I2C_InitStruct);
//	
//	//Transmit direction 
//	I2C_MasterRequestConfig(I2C2,I2C_Direction_Transmitter);

//	//Configure slave address - Gyroscope
//	I2C_SlaveAddressConfig(I2C2, GYRO_I2C_ADDRESS);

//	//Set number of bytes per message
//	I2C_NumberOfBytesConfig(I2C2,2);
//	
//	I2C_AutoEndCmd(I2C2,ENABLE);
//}

//void SendI2CData(void)
//{
//	I2C_SendData(I2C2,0x80);
//	I2C_Cmd(I2C2,ENABLE);
//	I2C_GenerateSTART(I2C2, ENABLE);
//}

int main(void)
{
	int i;
	uint8_t TransmitMailbox = 0;
	CanTxMsg TxMessage;
	ConfigureCAN();

	Init_TxMes(&TxMessage);

//	int temp = 0;
//	ConfigureTimers();
//	ConfigureGPIO();
//	ConfigureClock();
//	ConfigureADC();
//	ConfigureSPI();
//	ConfigureI2C();
//	SendI2CData();

	while(1)
		
	{
		TxMessage.Data[0] = 0xAB;
    TxMessage.Data[1] = 0xCD;

    TransmitMailbox = CAN_Transmit(CAN1,&TxMessage);

    i = 0;
    while((CAN_TransmitStatus(CAN1, TransmitMailbox) != CANTXOK) && (i != 0xFF)) // I put a breakpoint here to check and found the message is pending
			i++;
//		//ADC testing code
//		temp = GetADCData(ADC_Channel_5);
//		if (temp != 0)
//		{
//				GPIO_WriteBit(GPIOB, STATUS_G_PIN, 0);
//		}
	}
}

/*
GetADCData 
Inputs 
-ADC Channel to use
Output
- Converted data from ADC, in hex
*/
//int GetADCData(uint8_t ADC_Channel)
//{
//	//Always using ADC1, as there is only one ADC for this MCU
//	ADC_RegularChannelConfig(ADC1, ADC_Channel, 1, ADC_SampleTime_7Cycles5);

//	  /* Enable ADCx */
//  ADC_Cmd(ADC1, ENABLE);
//  
//  /* wait for ADRDY */
//  while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_RDY));
//  
//  /* Start ADCx Software Conversion */ 
//  ADC_StartConversion(ADC1);   
//  
//	/* Test EOC flag */
//	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
//	
//	/* Get ADCx converted data */
//	ADC1ConvertedValue =ADC_GetConversionValue(ADC1);
//	
//	/* Compute the voltage */
//	ADC1ConvertedVoltage = (ADC1ConvertedValue *3300)/0xFFF;
//	
//	return ADC1ConvertedVoltage;
//}