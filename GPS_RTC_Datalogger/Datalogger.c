#include "stm32f30x_conf.h" // STM32 config
#include "30010_io.h" 	    // Input/output library for this course
#include "lcd.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define RECV_BUF_LENGTH 600
#define IDEAL_VDD_A 3.3
#define VREFINT_CAL *((uint16_t*) ((uint32_t) 0x1FFFF7BA)) //calibrated at 3.3V@ 30C
volatile uint8_t RECV_BUF_USART3[RECV_BUF_LENGTH];
volatile uint8_t RECV_BUF_USART1[RECV_BUF_LENGTH];
uint8_t TO_CSV[RECV_BUF_LENGTH];
volatile int recv_index_usart3 = 0;
volatile int recv_index_usart1 = 0;
volatile uint8_t usart3_idle = 0;
volatile uint8_t usart1_idle = 0;
uint8_t RTC_SET = 0;

// some open logger commands
uint8_t list_directory_command[] = {'l', 's', '\r'};
uint8_t append_log_file[] = {'a', 'p', 'p', 'e', 'n', 'd', ' ', 'l', 'o', 'g', 'F', 'i', 'l', 'e', '.', 'c', 's', 'v', '\r'};
uint8_t log_filename[] = {'L', 'O', 'G', 'F', 'I', 'L', 'E', '.', 'C', 'S', 'V'};
uint8_t csv_header[] = {'G', 'P', 'S', '_', 'T', 'i', 'm', 'e', ',', 'R', 'T', 'C', '_', 'T', 'I', 'M', 'E', ',', 'L', 'a', 't', 'i', 't', 'u', 'd', 'e', ',', 'L', 'o', 'n', 'g', 'i', 't', 'u', 'd', 'e', ',', 'T', 'e', 'm', 'p', 'e', 'r', 'a', 't', 'u', 'r', 'e', '(', 'C', ')', 'D', 'a', 't', 'e', '\r'};
uint8_t NMEA_GPRMC[] = {'$','G','P','R','M','C'};
uint8_t NMEA_GPGGA[] = {'$','G','P','G','G','A'};
const uint8_t comma = ',';
const uint8_t newline = '\n';

// structure for log data
typedef struct {
  uint8_t GPS_TIME[10];
  uint8_t GPS_DATE[10];
  uint8_t RTC_TIME[10];
  uint8_t Latitude[10];
  uint8_t Longitude[12];
  int Temperature;
  uint8_t status;
} Data_Log;

Data_Log Log;


void enable_clocks(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE); // Enable USART3 peripheral clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE); // Enable USART1 peripheral clock
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC,ENABLE); // Enable Clock for GPIOC: PC_11 = RX_USART3; PC_10 = TX_USART3; PC_4 = TX_USART1;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,ENABLE); // Enable Clock for GPIOB: PB_14 = RTS_USART3;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE); // Enable Clock for GPIOA: PA_10 = RX; PA_12 = RTS_USART1;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE); // Enable RTC Power Controller interface clock
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC12,ENABLE); // Enable Clock for ADC1 Peripheral
}


void rtc_backup_domain_access(void)
{
	// rtc power controller interface clock has been enabled at enable_clocks()
	PWR_BackupAccessCmd(ENABLE);
	RCC_LSEConfig(RCC_LSE_ON);
	while(!RCC_GetFlagStatus(RCC_FLAG_LSERDY));
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
	RCC_RTCCLKCmd(ENABLE);
}

ErrorStatus init_rtc(void)
{
	RTC_InitTypeDef RTC_InitStruct;
	RTC_StructInit(&RTC_InitStruct);
	return RTC_Init(&RTC_InitStruct);
}

void init_usart_pins(void) //Function prototype for Joystick
{
	GPIO_InitTypeDef GPIO_InitStructAll; // Define typedef struct for setting pins

	/* USART3 CONFIG PINS BEGIN */
	GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_14; // RTS
	GPIO_InitStructAll.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructAll.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructAll.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructAll.GPIO_OType = GPIO_OType_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructAll);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14,GPIO_AF_7); //Sets pin y at port x to alternative function

	GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_11; // RX
	GPIO_InitStructAll.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructAll.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructAll.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructAll.GPIO_OType = GPIO_OType_OD;
	GPIO_Init(GPIOC, &GPIO_InitStructAll);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11,GPIO_AF_7); //Sets pin y at port x to alternative function

	GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_10; // TX
	GPIO_InitStructAll.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructAll.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructAll.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructAll.GPIO_OType = GPIO_OType_OD;
	GPIO_Init(GPIOC, &GPIO_InitStructAll);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10,GPIO_AF_7); //Sets pin y at port x to alternative function
	/* USART3 CONFIG PINS END */


	/* USART1 CONFIG PINS BEGIN */
	GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_12; // RTS
	GPIO_InitStructAll.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructAll.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructAll.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructAll.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructAll);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource12,GPIO_AF_7); //Sets pin y at port x to alternative function

	GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_10; // RX
	GPIO_InitStructAll.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructAll.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructAll.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructAll.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructAll);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10,GPIO_AF_7); //Sets pin y at port x to alternative function

	GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_4; // TX
	GPIO_InitStructAll.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructAll.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructAll.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructAll.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructAll);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource4,GPIO_AF_7); //Sets pin y at port x to alternative function
	/* USART1 CONFIG PINS END */

}

void init_usart3(void)
{
	USART_InitTypeDef USART_InitStructAll; // Define typedef struct for SPI
	USART_StructInit(&USART_InitStructAll);
	// play with these variables until it suites the slave
	USART_InitStructAll.USART_BaudRate = 9600;
	USART_InitStructAll.USART_WordLength = USART_WordLength_8b;
	USART_InitStructAll.USART_StopBits = USART_StopBits_1;
	USART_InitStructAll.USART_Parity = USART_Parity_No;
	USART_InitStructAll.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructAll.USART_HardwareFlowControl = USART_CR3_RTSE;
	USART_Init(USART3,&USART_InitStructAll);
}


void init_usart1(void)
{
	USART_InitTypeDef USART_InitStructAll; // Define typedef struct for SPI
	USART_StructInit(&USART_InitStructAll);
	// play with these variables until it suites the slave
	USART_InitStructAll.USART_BaudRate = 19200;
	USART_InitStructAll.USART_WordLength = USART_WordLength_8b;
	USART_InitStructAll.USART_StopBits = USART_StopBits_1;
	USART_InitStructAll.USART_Parity = USART_Parity_No;
	USART_InitStructAll.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructAll.USART_HardwareFlowControl = USART_CR3_RTSE;
	USART_Init(USART1,&USART_InitStructAll);
}

void ADC_PIN_CONF(void) //Function to configure ADC Pins
{
	GPIO_InitTypeDef GPIO_InitStructAll; // Define typedef struct for setting pins

	GPIO_StructInit(&GPIO_InitStructAll); // Initialize GPIO struct
	GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_0; // Set so the configuration is on pin 0
	GPIO_InitStructAll.GPIO_Mode = GPIO_Mode_AN; // Set as analogue input
	GPIO_InitStructAll.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructAll);
}


void ADC_settings(void) //Function to setup ADC
{
	ADC_InitTypeDef ADC_InitStructAll; // Define typedef struct for setting pins

	ADC_StructInit(&ADC_InitStructAll); // Initialize GPIO struct
	ADC_InitStructAll.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructAll.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructAll.ADC_NbrOfRegChannel = 1;
	ADC_InitStructAll.ADC_DataAlign = ADC_DataAlign_Right;
}


uint16_t set_VREF(void)
{
	ADC_VrefintCmd(ADC1,ENABLE);
	//Wait for at least 10uS before continuing...
	for(uint32_t i = 0; i<10000;i++);

	ADC_SelectCalibrationMode(ADC1,ADC_CalibrationMode_Single);
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1)){}
	for(uint32_t i = 0; i<100;i++);

	ADC_Cmd(ADC1,ENABLE);
	while((!ADC_GetFlagStatus(ADC1,ADC_FLAG_RDY))){}

	ADC_RegularChannelConfig(ADC1, ADC_Channel_18, 1, ADC_SampleTime_2Cycles5);
	ADC_StartConversion(ADC1); // Start ADC read
	while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == 0); // Wait for ADC read
	return ADC_GetConversionValue(ADC1);
}


float ADC_measure_PA(float ssize)
{
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_1Cycles5);
	ADC_StartConversion(ADC1); // Start ADC read
	while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == 0); // Wait for ADC read
	return ADC_GetConversionValue(ADC1) * ssize;
}



/* USER CODE BEGIN 4 */
int custom_write( USART_TypeDef * USARTx, uint8_t *buf, int nbytes)
{
	uint8_t num_of_byte = 0;
    while(num_of_byte <= nbytes - 1)
    {
        //waiting until the Transmit Empty flag is set
    	while(!USART_GetFlagStatus(USARTx,USART_FLAG_TXE));
        //send data byte by byte
    	USART_SendData(USARTx, buf[num_of_byte++]);
    }
    //Wait until the transmit complete Flag to be raised
    while(!USART_GetFlagStatus(USARTx, USART_FLAG_TC));
    return nbytes;
}


void configure_interrupts(void)
{
	 // NVIC for timer

	 NVIC_InitTypeDef NVIC_InitStructure;

	 //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	 NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	 //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	 //NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	 NVIC_Init(&NVIC_InitStructure);

	 USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	 USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
	 USART_ITConfig(USART3, USART_IT_ERR, ENABLE);

	 //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	 NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	 //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	 //NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	 NVIC_Init(&NVIC_InitStructure);

	 USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	 USART_ITConfig(USART1, USART_IT_ERR, ENABLE);
	 USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);


}


void USART3_EXTI28_IRQHandler(void)
{
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
		RECV_BUF_USART3[recv_index_usart3] = USART_ReceiveData(USART3);
		//if(RECV_BUF[recv_index] == '\n')
		//{
			//printFlag = 1;
			//recv_index = -1;
		//}
		recv_index_usart3 += 1;
		usart3_idle = 0;
    }

	if (USART_GetITStatus(USART3,USART_IT_IDLE))
	{
		usart3_idle = 1;
		recv_index_usart3 = 0;
		USART_ClearITPendingBit(USART3,USART_IT_IDLE);
	}

	if (USART_GetITStatus(USART3,USART_IT_ERR))
	{
		USART_ClearITPendingBit(USART3,USART_IT_ORE);
	}

}


void USART1_EXTI25_IRQHandler(void)
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		RECV_BUF_USART1[recv_index_usart1] = USART_ReceiveData(USART1);
		//printf("%c",RECV_BUF_USART1[recv_index_usart1]);
		/*if(RECV_BUF_USART1[recv_index_usart1] == '\n')
		{
			printFlag = 1;
			recv_index_usart1 = -1;
		}*/
		recv_index_usart1 += 1;
		usart1_idle = 0;
	}

	if (USART_GetITStatus(USART1,USART_IT_IDLE))
	{
		usart1_idle = 1;
		recv_index_usart1 = 0;
		USART_ClearITPendingBit(USART1,USART_IT_IDLE);
	}

	if (USART_GetITStatus(USART1,USART_IT_ERR))
	{
		USART_ClearITPendingBit(USART1,USART_IT_ORE);
	}
    USART_ClearITPendingBit(USART1,USART_IT_NE);
	USART_ClearITPendingBit(USART1,USART_IT_FE);

}


void init_openLogger(void)
{
	/*Open logger has been manually configured to be in command mode by default using the config.txt file*/
	for(uint64_t i = 0; i < 10000000; i++){}; // wait a little bit for open logger to start

	while(!usart3_idle); // make sure that usart3 is not busy. you've got 1 second

	// run list command to check if log_file exists
	custom_write(USART1,list_directory_command,strlen(list_directory_command));
	for(uint64_t i = 0; i < 10000000; i++){}; //wait for response

	// check if log_file exists
	if(strstr(RECV_BUF_USART1, log_filename) != 0)
	{
	    // log_file exists. Send append command to start writing data
		while(!usart3_idle); // make sure that usart3 is not busy. you've got 1 second
		memset(RECV_BUF_USART1,0,RECV_BUF_LENGTH);
	    custom_write(USART1,append_log_file,sizeof(append_log_file)/sizeof(append_log_file[0]));
	} else {
        // log_file doesn't exists. Send append command and write csv header
		while(!usart3_idle); // make sure that usart3 is not busy. you've got 1 second
		memset(RECV_BUF_USART1,0,RECV_BUF_LENGTH);
	    custom_write(USART1,append_log_file,sizeof(append_log_file)/sizeof(append_log_file[0]));
	    for(uint64_t i = 0; i < 10000000; i++){};
        while(!usart3_idle); // make sure that usart3 is not busy. you've got 1 second
	    custom_write(USART1,csv_header,sizeof(csv_header)/sizeof(csv_header[0]));
    }

        // reset array elements
	    memset(RECV_BUF_USART1,0,RECV_BUF_LENGTH);

}


Data_Log parseNMEA(void)
{
	uint8_t *nmea_gprmc = strstr(RECV_BUF_USART3, NMEA_GPRMC); // get pointer to gprmc
	uint8_t nmea_sentence[80] = {0}; // save gprmc
	uint8_t nmea_count = 0;
	Data_Log temp_log;
	temp_log.status = 0;

	if(nmea_gprmc != NULL)
	{
		//increment pointer until newline character is found_use while loop
		while(*nmea_gprmc != newline)
		{
			nmea_sentence[nmea_count] = *nmea_gprmc;
			// increment pointer
			nmea_gprmc++;
			nmea_count++;
		}
		if(strlen(nmea_sentence) < 60)
		{
			memset(RECV_BUF_USART3,0,RECV_BUF_LENGTH);
			return temp_log;
		}

		// get the commas
		uint8_t *nmea_gprmc_comma = strchr(nmea_sentence,comma);
		nmea_gprmc_comma++; // move to the next character after comma

		// get UTC time
		temp_log.GPS_TIME[0] = * nmea_gprmc_comma;
		temp_log.GPS_TIME[1] = *(nmea_gprmc_comma + 1);
		temp_log.GPS_TIME[2] = ':';
		temp_log.GPS_TIME[3] = *(nmea_gprmc_comma + 2);
		temp_log.GPS_TIME[4] = *(nmea_gprmc_comma + 3);
		temp_log.GPS_TIME[5] =':';
		temp_log.GPS_TIME[6] = *(nmea_gprmc_comma + 4);
		temp_log.GPS_TIME[7] = *(nmea_gprmc_comma + 5);
		temp_log.GPS_TIME[8] = '\0';

		nmea_gprmc_comma = strchr(nmea_gprmc_comma + 5, comma); // get next comma after the date

		nmea_gprmc_comma = nmea_gprmc_comma + 3; // jump to the latitude
		temp_log.Latitude[0] = *nmea_gprmc_comma;
		temp_log.Latitude[1] = *(nmea_gprmc_comma + 1);
		temp_log.Latitude[2] = '.';
		temp_log.Latitude[3] = *(nmea_gprmc_comma + 2);
		temp_log.Latitude[4] = *(nmea_gprmc_comma + 3);
		temp_log.Latitude[5] = *(nmea_gprmc_comma + 5);
		temp_log.Latitude[6] = *(nmea_gprmc_comma + 6);
		//temp_log.Latitude[7] = *(nmea_gprmc_comma + 7);
		//temp_log.Latitude[8] = *(nmea_gprmc_comma + 8);
		temp_log.Latitude[7] = '\0';

		nmea_gprmc_comma = strchr(nmea_gprmc_comma + 7, comma); // get next comma after Latitude

		nmea_gprmc_comma = nmea_gprmc_comma + 3; // jump to the longitude
		temp_log.Longitude[0] = *nmea_gprmc_comma;
		temp_log.Longitude[1] = *(nmea_gprmc_comma + 1);
		temp_log.Longitude[2] = *(nmea_gprmc_comma + 2);
		temp_log.Longitude[3] = '.';
		temp_log.Longitude[4] = *(nmea_gprmc_comma + 3);
		temp_log.Longitude[5] = *(nmea_gprmc_comma + 4);
		temp_log.Longitude[6] = *(nmea_gprmc_comma + 6);
		temp_log.Longitude[7] = *(nmea_gprmc_comma + 7);
		//temp_log.Longitude[8] = *(nmea_gprmc_comma + 8);
		//temp_log.Longitude[9] = *(nmea_gprmc_comma + 9);
		temp_log.Longitude[8] = '\0';

		nmea_gprmc_comma = strchr(nmea_gprmc_comma + 8, comma) + 1; // get next comma after Longitude
		nmea_gprmc_comma = strchr(nmea_gprmc_comma, comma) + 1;
		nmea_gprmc_comma = strchr(nmea_gprmc_comma, comma) + 1;
		//nmea_gprmc_comma = strchr(nmea_gprmc_comma, comma) + 1;

		nmea_gprmc_comma = nmea_gprmc_comma + 1; // jump to date
		temp_log.GPS_DATE[0] = *nmea_gprmc_comma;
		temp_log.GPS_DATE[1] = *(nmea_gprmc_comma + 1);
		temp_log.GPS_DATE[2] = '-';
		temp_log.GPS_DATE[3] = *(nmea_gprmc_comma + 2);
		temp_log.GPS_DATE[4] = *(nmea_gprmc_comma + 3);
		temp_log.GPS_DATE[5] = '-';
		temp_log.GPS_DATE[6] = *(nmea_gprmc_comma + 4);
		temp_log.GPS_DATE[7] = *(nmea_gprmc_comma + 5);
		temp_log.GPS_DATE[8] = '\0';

		temp_log.status = 1;
		//printf("%s\n",temp_log.GPS_DATE);
		memset(RECV_BUF_USART3,0,RECV_BUF_LENGTH);
		return temp_log;

	} else {
		memset(RECV_BUF_USART3,0,RECV_BUF_LENGTH);
		return temp_log;
	}
}


int main(void)
{
	RTC_TimeTypeDef RTC_TimeStruct;
	RTC_DateTypeDef RTC_DateStruct;

	uint8_t NMEA_YEAR[3];
	uint8_t NMEA_MONTH[3];
	uint8_t NMEA_DAY[3];
	uint8_t NMEA_HR[3];
	uint8_t NMEA_MIN[3];
	uint8_t NMEA_SEC[3];
	uint8_t RTC_TIME[9] = {0};
	uint8_t ADC_READ[10] = {0};
	uint8_t nmea_year_to_int = 0, nmea_month_to_int = 0, nmea_day_to_int = 0;
	uint8_t nmea_hr_to_int = 0, nmea_min_to_int = 0, nmea_sec_to_int = 0;

	enable_clocks();
	RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div8); // Select ADC Clock
	init_usart_pins();
	uart_init(9600);
	init_usart1();
	init_usart3();
	ADC_PIN_CONF();
	ADC_settings();
	USART_Cmd(USART3,ENABLE); //Enable usart3
	USART_Cmd(USART1,ENABLE); //Enable usart1
	configure_interrupts();
	rtc_backup_domain_access();
	init_rtc();
	init_openLogger();
	float step_size = ((3.3*VREFINT_CAL)/set_VREF())/4095;

	while(1)
	{
		if(usart3_idle)
		{
			Log = parseNMEA();
			if(Log.status)
			{
				if(!RTC_SET)
				{
					RTC_SET = 1;
					//Get date and compare it
					NMEA_DAY[0] = Log.GPS_DATE[0];
					NMEA_DAY[1] = Log.GPS_DATE[1];
					NMEA_DAY[2] = '\0';
					nmea_day_to_int = atoi(NMEA_DAY);

					NMEA_MONTH[0] = Log.GPS_DATE[3];
					NMEA_MONTH[1] = Log.GPS_DATE[4];
					NMEA_MONTH[2] = '\0';
					nmea_month_to_int = atoi(NMEA_MONTH);

					NMEA_YEAR[0] = Log.GPS_DATE[6];
					NMEA_YEAR[1] = Log.GPS_DATE[7];
					NMEA_YEAR[2] = '\0';
					nmea_year_to_int = atoi(NMEA_YEAR);

					NMEA_HR[0] = Log.GPS_TIME[0];
					NMEA_HR[1] = Log.GPS_TIME[1];
					NMEA_HR[2] = '\0';
					nmea_hr_to_int = atoi(NMEA_HR);

					NMEA_MIN[0] = Log.GPS_TIME[3];
					NMEA_MIN[1] = Log.GPS_TIME[4];
					NMEA_MIN[2] = '\0';
					nmea_min_to_int = atoi(NMEA_MIN);

					NMEA_SEC[0] = Log.GPS_TIME[6];
					NMEA_SEC[1] = Log.GPS_TIME[7];
					NMEA_SEC[2] = '\0';
					nmea_sec_to_int = atoi(NMEA_MIN);

					RTC_TimeStruct.RTC_Hours = nmea_hr_to_int;
					RTC_TimeStruct.RTC_Minutes = nmea_min_to_int;
					RTC_TimeStruct.RTC_Seconds = nmea_sec_to_int;

					RTC_DateStruct.RTC_Month = nmea_month_to_int;
					RTC_DateStruct.RTC_Date = nmea_day_to_int;
					RTC_DateStruct.RTC_Year = nmea_year_to_int;
					//RTC_DateStruct.RTC_WeekDay = RTC_Weekday_Thursday;

					RTC_SetTime(RTC_Format_BIN, &RTC_TimeStruct);
					RTC_SetDate(RTC_Format_BIN, &RTC_DateStruct);
					RTC_DayLightSavingConfig(RTC_DayLightSaving_ADD1H,RTC_StoreOperation_Set);
				} else {
					RTC_GetTime(RTC_Format_BIN, &RTC_TimeStruct);
					//RTC_GetDate(RTC_Format_BIN, &RTC_DateStruct);

					RTC_TIME[0] = ((uint8_t) RTC_TimeStruct.RTC_Hours / 10) + '0';
					RTC_TIME[1] = (RTC_TimeStruct.RTC_Hours % 10) + '0';
					RTC_TIME[2] = ':';
					RTC_TIME[3] = ((uint8_t) RTC_TimeStruct.RTC_Minutes / 10) + '0';
					RTC_TIME[4] = (RTC_TimeStruct.RTC_Minutes % 10) + '0';
					RTC_TIME[5] = ':';
					RTC_TIME[6] = ((uint8_t) RTC_TimeStruct.RTC_Seconds / 10) + '0';
					RTC_TIME[7] = (RTC_TimeStruct.RTC_Seconds % 10) + '0';
					RTC_TIME[8] = '\0';

					custom_write(USART1, Log.GPS_TIME, strlen(Log.GPS_TIME));
					custom_write(USART1, ",", 1);
					custom_write(USART1, RTC_TIME, strlen(RTC_TIME));
					custom_write(USART1, ",", 1);
					custom_write(USART1, Log.Latitude, strlen(Log.Latitude));
					custom_write(USART1, ",", 1);
					custom_write(USART1, Log.Longitude, strlen(Log.Longitude));
					custom_write(USART1, ",", 1);
					custom_write(USART1, Log.GPS_DATE, strlen(Log.GPS_DATE));
					custom_write(USART1, ",", 1);
					sprintf(ADC_READ,"%.03f\r\n", ADC_measure_PA(step_size));
					custom_write(USART1, ADC_READ, strlen(ADC_READ));
					memset(ADC_READ,0,sizeof(ADC_READ)/sizeof(ADC_READ[0]));
				}
			}
			//for(uint64_t i = 0; i < 10000000; i++);
		}
	}
}
