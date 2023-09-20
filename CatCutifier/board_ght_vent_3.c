#include "header.h"

ebm_Papst_registers_group_1 *ebm_Papst_registers_group_1_PTR;
ebm_Papst_registers_group_2 *ebm_Papst_registers_group_2_PTR;
ebm_Papst_registers_group_3 *ebm_Papst_registers_group_3_PTR;
ebm_Papst_registers_group_4 *ebm_Papst_registers_group_4_PTR;
M95X Eeprom;
#ifdef MB1
	TMbRTUPort Mb1;
#endif
#ifdef MB2
	TMbRTUPort Mb2;
#endif
#ifdef MB3
	TMbRTUPort Mb3;
#endif
TBrdData BrdData;
#ifdef UIP
UIP_PORT uip;
char UipSendFunc(char);
void UipCsFunc(char);
UIP_UDP_SERVER uip_udp_server;
UIP_UDP_CLIENT uip_udp_client;
#endif

const TMbSlaveInfo SlaveInfo = {MB_ADR, DEVICE_NUMBER, VERSION, VERSION_DATE, "GHT_VENT", "Sensorless PMSM controller for ventilator application" };

static void EepromSetCs(uint8_t);
#ifdef MB1
	static void ModbusTxControl1(char);
#endif
#ifdef MB2
	static void ModbusTxControl2(char);
#endif
#ifdef MB3
	static void ModbusTxControl3(char);
#endif
#if FILE_OP_ENABLE
static short *GetFileRecord(short, short, short, char *, char *);
#endif

unsigned short* MBS_sGetData(unsigned short,unsigned short,unsigned short *);
void ModBusRTUReinit(TMbRTUPort *hPort);


void InitBoardPeripherial(void)
{

MBS_Board_GPIO_Init(); // LED,OUT,en_PWM

MBS_Board_SPI2_Init(); // EEPROM

MBS_Board_SPI3_Init();

MBS_Board_USART3_Init(); // ModbusRTU  (MB1)
MBS_Board_USART6_Init(); // ModbusRTU  (MB2)

MBS_Board_TIM1_Init(); // PWM

MBS_Board_ADC_Init(); // ADC


Eeprom.SpiId    = 1;
Eeprom.SpiBaud  = 0x4;
Eeprom.Type     = M95256_TYPE;
Eeprom.BusyTime = (uint16_t)(0.02*MAIN_ISR_FREQ);
Eeprom.CsFunc   = EepromSetCs;
M95X_Init(&Eeprom);

ReadBoardData();

// Modbus Communication driver initialization
#ifdef MB1
Mb1.Params.UartID      = 2; //USART3
Mb1.Params.Mode        = MB1_MODE;
Mb1.Params.Slave       = BrdData.Mb1.Slave;
Mb1.Params.BaudRate    = BrdData.Mb1.Speed;
Mb1.Params.BrrValue    = BrdData.Mb1.Speed;
Mb1.Params.Parity      = BrdData.Mb1.Parity;
Mb1.Params.StopBits    = BrdData.Mb1.StopBits;
Mb1.Params.UserMode    = 0;
Mb1.Params.RetryCount  = 0;
Mb1.Params.Scale       = (MAIN_ISR_FREQ)/1000;
Mb1.Params.RxDelay     = 5;
Mb1.Params.TxDelay     = 2;
#if MB1_MODE == MB_RTU_SLAVE
Mb1.Params.ConnTimeout = (uint16_t)(3.0*MAIN_ISR_FREQ_1kHz);
#elif MB1_MODE == MB_RTU_MASTER
Mb1.Params.ConnTimeout = (uint16_t)(0.2*MAIN_ISR_FREQ_1kHz);
#endif
Mb1.Params.AckTimeout  = 1*MAIN_ISR_FREQ_1kHz;
Mb1.Params.SlaveInfo   = (TMbSlaveInfo *)&SlaveInfo;
Mb1.Params.TrEnable    = ModbusTxControl1;
Mb1.Params.GetData     = MBS_sGetData;
#if FILE_OP_ENABLE
Mb1.Params.GetRecord   = (TMbGetRecord)GetFileRecord;;
#else
Mb1.Params.GetRecord   = 0;
#endif
ModBusRTUInit(&Mb1);
#endif

#ifdef MB2
// Modbus Communication driver initialization
Mb2.Params.UartID      = 5; //USART6
Mb2.Params.Mode        = MB2_MODE;
Mb2.Params.Slave       = BrdData.Mb2.Slave;
Mb2.Params.BaudRate    = BrdData.Mb2.Speed;
Mb2.Params.BrrValue    = BrdData.Mb2.Speed;
Mb2.Params.Parity      = BrdData.Mb2.Parity;
Mb2.Params.StopBits    = BrdData.Mb2.StopBits;
Mb2.Params.UserMode    = 0;
Mb2.Params.RetryCount  = 0;
Mb2.Params.Scale       = (MAIN_ISR_FREQ)/1000;
Mb2.Params.RxDelay     = 0;
Mb2.Params.TxDelay     = 1;
#if MB2_MODE == MB_RTU_SLAVE
Mb2.Params.ConnTimeout = (uint16_t)(3.0*MAIN_ISR_FREQ);
#elif MB2_MODE == MB_RTU_MASTER
Mb2.Params.ConnTimeout = (uint16_t)(0.2*MAIN_ISR_FREQ);
#endif
Mb2.Params.AckTimeout  = 1*MAIN_ISR_FREQ;
Mb2.Params.SlaveInfo   = (TMbSlaveInfo *)&SlaveInfo;
Mb2.Params.TrEnable    = ModbusTxControl2;
Mb2.Params.GetData     = MBS_sGetData;
Mb2.Params.GetRecord   = 0;
ModBusRTUInit(&Mb2);
#endif

#ifdef MB3
// Modbus Communication driver initialization
#ifdef ST_LINK
	Mb3.Params.UartID      = 5;
#else
	Mb3.Params.UartID      = 2;
#endif
Mb3.Params.Mode        = MB3_MODE;
Mb3.Params.Slave       = BrdData.Mb3.Slave;
Mb3.Params.BaudRate    = BrdData.Mb3.Speed;
Mb3.Params.BrrValue    = BrdData.Mb3.Speed;
Mb3.Params.Parity      = BrdData.Mb3.Parity;
Mb3.Params.StopBits    = BrdData.Mb3.StopBits;
Mb3.Params.UserMode    = 0;
Mb3.Params.RetryCount  = 0;
Mb3.Params.Scale       = (MAIN_ISR_FREQ_1kHz)/1000;
Mb3.Params.RxDelay     = 5;
Mb3.Params.TxDelay     = 2;
#if MB3_MODE == MB_RTU_SLAVE
Mb3.Params.ConnTimeout = (uint16_t)(3.0*MAIN_ISR_FREQ_1kHz);
#elif MB3_MODE == MB_RTU_MASTER
Mb3.Params.ConnTimeout = (uint16_t)(0.2*MAIN_ISR_FREQ_1kHz);
#endif
Mb3.Params.AckTimeout  = 1*MAIN_ISR_FREQ_1kHz;
Mb3.Params.SlaveInfo   = (TMbSlaveInfo *)&SlaveInfo;
Mb3.Params.TrEnable    = ModbusTxControl3;
Mb3.Params.GetData     = MBS_sGetData;
Mb3.Params.GetRecord   = 0;
ModBusRTUInit(&Mb3);
#endif

#ifdef UIP
	memset(&uip, 0, sizeof(UIP_PORT));
	memset(&uip_udp_server, 0, sizeof(UIP_UDP_SERVER));
	memset(&uip_udp_client, 0, sizeof(UIP_UDP_CLIENT));
	uip.Params       = &BrdData.Uip;
	uip.periodMs     = MAIN_ISR_FREQ/1000;
	uip.connTimeout  = 3000;
	uip.SlaveInfo    = (UIP_SLAVE_INFO *)&SlaveInfo;
	uip.SendFunc     = UipSendFunc;
	uip.CsFunc       = UipCsFunc;
	uip.GetData      = (TUipGetData)((long)&MBS_sGetData);
	uip.GetRecord    = 0;
	uip.UdpClient    = (UIP_UDP_CLIENT *)&uip_udp_server;
	uip.UdpServer    = (UIP_UDP_SERVER *)&uip_udp_client;
	UipCsFunc(0);
	DelayUs(2000);
	UipCsFunc(1);
	DelayUs(100);
 #ifdef UIP_SPI1
   SPI_init(0, 1, 0, 2, 8);
 #endif
   uip_init(&uip);
#endif


}

void MBS_Board_GPIO_Init(void)
{
	  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	  /* GPIO Ports Clock Enable */

	  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOE);
	  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
	  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
	  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOH);
	  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOG);
	  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);


	  //Led state//
	  LL_GPIO_SetOutputPin(LED_RED_GPIO_Port, LED_RED_Pin);
	  LL_GPIO_SetOutputPin(LED_GRE_GPIO_Port, LED_GRE_Pin);
	  LL_GPIO_SetOutputPin(LED_ORG_GPIO_Port, LED_ORG_Pin);

	  //OUT//
	  LL_GPIO_ResetOutputPin(DO_RELEY1_GPIO_Port, DO_RELEY1_Pin);

	  //An In Control//
	  LL_GPIO_ResetOutputPin(DO_SW_IN1_GPIO_Port, DO_SW_IN1_Pin);

	  //EN_PWM//
	  LL_GPIO_ResetOutputPin(EN_PWM_PCH_GPIO_Port, EN_PWM_PCH_Pin);

	  //reset_fault//
	  LL_GPIO_ResetOutputPin(nRES_FAULT_GPIO_Port, nRES_FAULT_Pin);

	  //heating control//
	  LL_GPIO_ResetOutputPin(PWM_H_LO_I_GPIO_Port, PWM_H_LO_I_Pin);

	  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;

	  //Led Init//
	  GPIO_InitStruct.Pin = LED_RED_Pin;
	  	  	   LL_GPIO_Init(LED_RED_GPIO_Port, &GPIO_InitStruct);
	  GPIO_InitStruct.Pin = LED_GRE_Pin;
	  	  	   LL_GPIO_Init(LED_GRE_GPIO_Port, &GPIO_InitStruct);
	  GPIO_InitStruct.Pin = LED_ORG_Pin;
	   	  	   LL_GPIO_Init(LED_ORG_GPIO_Port, &GPIO_InitStruct);

	  //OUT_Init//
	  GPIO_InitStruct.Pin = DO_RELEY1_Pin;
	  	  	   LL_GPIO_Init(DO_RELEY1_GPIO_Port, &GPIO_InitStruct);

	  //EN_PWMs_Init//
	  GPIO_InitStruct.Pin = EN_PWM_PCH_Pin;
	           LL_GPIO_Init(EN_PWM_PCH_GPIO_Port, &GPIO_InitStruct);

	  //An In Control//
	  GPIO_InitStruct.Pin = DO_SW_IN1_Pin;
	  	  	   LL_GPIO_Init(DO_SW_IN1_GPIO_Port, &GPIO_InitStruct);

	  //reset_fault_Init//
	  GPIO_InitStruct.Pin = nRES_FAULT_Pin;
	   	  	   LL_GPIO_Init(nRES_FAULT_GPIO_Port, &GPIO_InitStruct);

	  //heating control//
	  GPIO_InitStruct.Pin = PWM_H_LO_I_Pin;
	  	  	  LL_GPIO_Init(PWM_H_LO_I_GPIO_Port, &GPIO_InitStruct);

	  // Digit_Inputs
	  /**/
	  GPIO_InitStruct.Pin = DI_I1_Pin;
	  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	  LL_GPIO_Init(DI_I1_GPIO_Port, &GPIO_InitStruct);

	  /**/
	  GPIO_InitStruct.Pin = DI_I2_Pin;
	  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	  LL_GPIO_Init(DI_I2_GPIO_Port, &GPIO_InitStruct);
}

void MBS_Board_SPI2_Init(void)
{
	  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	  /* Peripheral clock enable */
	  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);
	  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

	  /**SPI2 GPIO Configuration
	  PB10   ------> SPI2_SCK
	  PB14   ------> SPI2_MISO
	  PB15   ------> SPI2_MOSI
	  */

	  LL_GPIO_SetOutputPin(SPI2_FRAM_CS_GPIO_Port, SPI2_FRAM_CS_Pin);


	  GPIO_InitStruct.Pin = SPI2_FRAM_SCK_Pin|SPI2_FRAM_MISO_Pin|SPI2_FRAM_MOSI_Pin;
	  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
	  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);


	  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
	  GPIO_InitStruct.Pin = SPI2_FRAM_CS_Pin;
	  	  	   LL_GPIO_Init(SPI2_FRAM_CS_GPIO_Port, &GPIO_InitStruct);
}

void MBS_Board_SPI3_Init(void)
{
	  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	  /* Peripheral clock enable */
	  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI3);
	  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);

	  /**SPI2 GPIO Configuration
	  PB10   ------> SPI2_SCK
	  PB14   ------> SPI2_MISO
	  PB15   ------> SPI2_MOSI
	  */

	  LL_GPIO_SetOutputPin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin);


	  GPIO_InitStruct.Pin = SPI3_SCK_Pin|SPI3_MISO_Pin|SPI3_MOSI_Pin;
	  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
	  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);


	  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
	  GPIO_InitStruct.Pin = SPI3_NSS_Pin;
	  	  	   LL_GPIO_Init(SPI3_NSS_GPIO_Port, &GPIO_InitStruct);


}


void MBS_Board_USART3_Init(void)
{
	 LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	 LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	 LL_APB2_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);

	  /**USART1 GPIO Configuration
	  PB10   ------> USART3_TX
	  PB11   ------> USART3_RX
	  */

	 LL_GPIO_ResetOutputPin(USART3_TR_MB1_GPIO_Port, USART3_TR_MB1_Pin);

	 GPIO_InitStruct.Pin = USART3_TX_MB1_Pin | USART3_RX_MB1_Pin;
	 GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	 GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	 GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	 GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	 GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
	 LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	 GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	 GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	 GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	 GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
	 GPIO_InitStruct.Pin = USART3_TR_MB1_Pin;
	 LL_GPIO_Init(USART3_TR_MB1_GPIO_Port, &GPIO_InitStruct);
}


void MBS_Board_USART6_Init(void)
{
	 LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	 LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
	 LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART6);

	  /**USART2 GPIO Configuration
	  PC8   ------> USART6_TX
	  PC7   ------> USART6_RX
	  */

	  LL_GPIO_ResetOutputPin(USART6_TR_MB2_GPIO_Port, USART6_TR_MB2_Pin);

	  GPIO_InitStruct.Pin = USART6_TX_MB2_Pin| USART6_RX_MB2_Pin;
	  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	  GPIO_InitStruct.Alternate = LL_GPIO_AF_8;
	  LL_GPIO_Init(USART6_TX_MB2_GPIO_Port, &GPIO_InitStruct);

	  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
	  GPIO_InitStruct.Pin = USART6_TR_MB2_Pin;
	  LL_GPIO_Init(USART6_TR_MB2_GPIO_Port, &GPIO_InitStruct);
}

void MBS_Board_TIM1_Init (void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* USER CODE END TIM1_Init 2 */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
      /**TIM1 GPIO Configuration
      PA7     ------> TIM1_CH1N
      PA8     ------> TIM1_CH1
      PB0     ------> TIM1_CH2N
      PA9     ------> TIM1_CH2
      PB1     ------> TIM1_CH3N
      PA10     ------> TIM1_CH3
      */
    GPIO_InitStruct.Pin = PWM_U_LO_I_Pin|PWM_U_HO_I_Pin|PWM_V_HO_I_Pin|PWM_W_HO_I_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
    LL_GPIO_Init(PWM_U_LO_I_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = PWM_V_LO_I_Pin|PWM_W_LO_I_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
    LL_GPIO_Init(PWM_V_LO_I_GPIO_Port, &GPIO_InitStruct);


	GPIO_InitStruct.Pin = EN_PWM_TIM1_BKIN_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
	LL_GPIO_Init(EN_PWM_TIM1_BKIN_GPIO_Port, &GPIO_InitStruct);
}

void MBS_Board_ADC_Init(void)
{
	 LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	 LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
	 LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

	  GPIO_InitStruct.Pin = ADC1_U_fU_CH10_Pin |ADC2_U_fV_CH11_Pin|ADC3_U_fW_CH12_Pin|ADC3_MTEMP_CH13_Pin|ADC2_TEMP_cool_CH14_Pin|ADC1_TEMP_PCB_CH15_Pin;
	  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
	  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	  GPIO_InitStruct.Pin = ADC1_I_FU_CH0_Pin|ADC2_I_FV_CH1_Pin|ADC3_I_FW_CH2_Pin|ADC3_CUR_IN_ANI_CH3_Pin|ADC2_U10V_IN_ANI_CH4_Pin|ADC1_DC_VOLT_CH5_Pin;
	  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
	  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void ExecuteBoardBackgroundTask(void)
{
#ifdef UIP
	if (LL_GPIO_IsOutputPinSet(SPI2_FRAM_CS_GPIO_Port, SPI2_FRAM_CS_Pin))
	{
		uip_process(&uip);
	}
#endif

}

void Slow_ExecuteBoardInterruptTask (void)
{

}

void Fast_ExecuteBoardInterruptTask(void)
{
	if (IsBoardMemoryReady())
	{
	 #ifdef MB1
		if (Mb1.Params.Mode!=MB_RTU_MASTER)
		{
		 ModbusCheckParams(&Mb1, &BrdData ,&BrdData.Mb1);
		}
	 #endif
	 #ifdef MB2
		if (Mb2.Params.Mode!=MB_RTU_MASTER)
		{
		ModbusCheckParams(&Mb2, &BrdData ,&BrdData.Mb2);
		}
	 #endif
	#ifdef MB3
		if (Mb3.Params.Mode!=MB_RTU_MASTER)
		{
		ModbusCheckParams(&Mb3, &BrdData ,&BrdData.Mb3);
		}
	#endif
	 }
	#ifdef MB1
	 ModBusRTUTimings(&Mb1);
	 ModBusRTUInvoke(&Mb1);
	 if (Mb1.Params.Mode!=MB_RTU_MASTER)
	 {
		 if (!Mb1.Packet.Connected) { ModbusTxControl1(0); BrdData.Status.bit.Mb1 = 0;}
		 else { BrdData.Status.bit.Mb1 = 1; }
	 }
	 else
	{
		 if (Mb1.Packet.Connected) { BrdData.Status.bit.Mb1 = 1; }
		 else					   { BrdData.Status.bit.Mb1 = 0; }
	}
	#endif
	#ifdef MB2
	 ModBusRTUTimings(&Mb2);
	 ModBusRTUInvoke(&Mb2);
	 if (Mb2.Params.Mode!=MB_RTU_MASTER)
	 {
		if (!Mb2.Packet.Connected) { ModbusTxControl2(0); BrdData.Status.bit.Mb2 = 0;}
		else { BrdData.Status.bit.Mb2 = 1; }
	 }
	 else
	 {
		 if (Mb2.Packet.Connected) { BrdData.Status.bit.Mb2 = 1; }
		 else					   { BrdData.Status.bit.Mb2 = 0; }
	 }
	 #endif
	#ifdef MB3
	 ModBusRTUTimings(&Mb3);
	 ModBusRTUInvoke(&Mb3);
	 if (Mb3.Params.Mode!=MB_RTU_MASTER)
	 {
		 if (!Mb3.Packet.Connected)  { ModbusTxControl3(0); BrdData.Status.bit.Mb3 = 0;  }
		 else { BrdData.Status.bit.Mb3 = 1; }
	 }
	 else
	 {
		 if (Mb3.Packet.Connected) { BrdData.Status.bit.Mb3 = 1; }
		 else					   { BrdData.Status.bit.Mb3 = 0; }
	 }
	#endif

	if (LL_GPIO_IsOutputPinSet(SPI2_FRAM_CS_GPIO_Port, SPI2_FRAM_CS_Pin))
	{
		M95X_Update(&Eeprom);
	}
	DataProject_Update(&BrdData);
	#ifdef UIP
	uip_periodic(&uip);
	if (uip.connected) {BrdData.Status.bit.Uip = 1;}
	else {BrdData.Status.bit.Uip = 0;}
	if (IsBoardMemoryReady())
	{
		if (uip.wrFlag) { WriteToBoardMemory(0, ((int32_t)&BrdData.Uip - (int32_t)&BrdData)>>1, (short *)&BrdData.Uip, sizeof(UIP_PARAMS)>>1); uip.wrFlag=0; }
	}
 	 #endif
}

///////////////////////MODBUS///////////////////////////////////////////////////
#ifdef MB1
void USART3_IRQHandler(void)
{
  if (USART3->SR & (USART_SR_NE|USART_SR_FE|USART_SR_PE|USART_SR_ORE))
 	   LL_USART_ClearFlag_FE(USART3);

  if (LL_USART_IsActiveFlag_RXNE(USART3) != RESET)
  {
	 LL_USART_ClearFlag_RXNE(USART3);
     ModBusRTURxIsr(&Mb1);
  }

  if (LL_USART_IsActiveFlag_TC(USART3)  != RESET)
  {
	 LL_USART_ClearFlag_TC(USART3);
     ModBusRTUTxIsr(&Mb1);
  }
}
 static void ModbusTxControl1(char State)
 {
//  if (State) { LL_GPIO_SetOutputPin(USART3_TR_MB1_GPIO_Port, USART3_TR_MB1_Pin); }
//  else 	   { LL_GPIO_ResetOutputPin(USART3_TR_MB1_GPIO_Port, USART3_TR_MB1_Pin); }
 }
#endif

#ifdef MB2
 void USART6_IRQHandler(void)
 {
   if (USART6->SR & (USART_SR_NE|USART_SR_FE|USART_SR_PE|USART_SR_ORE))
   {
	   ModBusRTUReinit(&Mb2);
   }

   if (LL_USART_IsActiveFlag_RXNE(USART6) != RESET)
   {
	 LL_USART_ClearFlag_RXNE(USART6);
     ModBusRTURxIsr(&Mb2);
   }

   if (LL_USART_IsActiveFlag_TC(USART6)  != RESET)
   {
	 LL_USART_ClearFlag_TC(USART6);
     ModBusRTUTxIsr(&Mb2);
   }
 }
 static void ModbusTxControl2(char State)
 {
	 if (State) { LL_GPIO_SetOutputPin(USART6_TR_MB2_GPIO_Port, USART6_TR_MB2_Pin); }
	  else 	   { LL_GPIO_ResetOutputPin(USART6_TR_MB2_GPIO_Port, USART6_TR_MB2_Pin); }

 }
#endif


#ifdef MB3
  void USART3_IRQHandler(void)
  {
    if (USART3->SR & (USART_SR_NE|USART_SR_FE|USART_SR_PE|USART_SR_ORE))
   	   LL_USART_ClearFlag_FE(USART3);
    if (LL_USART_IsActiveFlag_RXNE(USART3) != RESET)
    {
	  LL_USART_ClearFlag_RXNE(USART3);
      ModBusRTURxIsr(&Mb3);
    }
    if (LL_USART_IsActiveFlag_TC(USART3)  != RESET)
    {
	  LL_USART_ClearFlag_TC(USART3);
      ModBusRTUTxIsr(&Mb3);
    }
  }

  static void ModbusTxControl3(char State)
 {
   if (State) { LL_GPIO_SetOutputPin(USART3_PULT_TR_MB2_GPIO_Port, USART3_PULT_TR_MB2_Pin); }
   else     { LL_GPIO_ResetOutputPin(USART3_PULT_TR_MB2_GPIO_Port, USART3_PULT_TR_MB2_Pin); }
 }
#endif

void ModbusCheckParams(TMbRTUPort *hPort,TBrdData *structure, TMbParStr *params)
{
  uint16_t flag = 0;
  if (hPort->Params.Slave != params->Slave)
  {
    hPort->Params.Slave = params->Slave;
    flag = 1;
  }
  if (hPort->Params.BaudRate != params->Speed)
  {
    hPort->Params.BaudRate = params->Speed;
    hPort->Params.BrrValue = params->Speed;
    flag = 1;
  }
  if (hPort->Params.Parity != params->Parity)
  {
    hPort->Params.Parity = params->Parity;
    flag = 1;
  }
  if (hPort->Params.StopBits != params->StopBits)
  {
    hPort->Params.StopBits = params->StopBits;
    flag = 1;
  }
  if (flag)
  {
 	ModBusRTUInit(hPort);
    WriteToBoardMemory(0, ((int32_t)params - (int32_t)structure)>>1, (short *)params, sizeof(TMbParStr)>>1);
  }
}

#define CheckRange(sAddr, Cnt, Val, sMod) \
  if((Addr >= sAddr) && (lAddr < (sAddr + Cnt))) { \
    Data = (unsigned short *)(Val) + (Addr - sAddr); *Mode = sMod; }

unsigned short* MBS_sGetData(unsigned short Addr,unsigned short Count,unsigned short *Mode)
{
  unsigned short *Data = 0;
  unsigned short  lAddr = Addr + Count - 1;
  CheckRange(0x600, sizeof(struct MBS_CONTROL) / sizeof(short), &Kernel.control, 0) else
  CheckRange(0x0, Kernel.glCount * sizeof(long) / sizeof(short), Kernel.glData, 1) else
  CheckRange(0x8620, 0x800 / sizeof(short), Kernel.monAddr, 0) else
  CheckRange(0x620, 0x10000 / sizeof(short), Kernel.cfg, 0) else
  CheckRange(0xD000, sizeof(ebm_Papst_registers_group_1) / sizeof(short), ebm_Papst_registers_group_1_PTR, 0) else
  CheckRange(0xD100, sizeof(ebm_Papst_registers_group_2) / sizeof(short), ebm_Papst_registers_group_2_PTR, 0) else
  CheckRange(0xD1A0, sizeof(ebm_Papst_registers_group_3) / sizeof(short), ebm_Papst_registers_group_3_PTR, 0) else
  CheckRange(0xD15A, sizeof(ebm_Papst_registers_group_4) / sizeof(short), ebm_Papst_registers_group_4_PTR, 0)
  return Data;
}

#if FILE_OP_ENABLE
static short *GetFileRecord(short FileNum, short RecNum, short RecSize, char *AckFlag, char *Exception)
{
  #define MB_FILE_REC_SIZE  10000
  uint32_t FileAddr;

  if (RecSize > MB_RTU_WORD_MAX) { *Exception = EX_ILLEGAL_DATA_VALUE; return 0; }
  if (RecNum >= MB_FILE_REC_SIZE) { *Exception = EX_ILLEGAL_DATA_ADDRESS; return 0; }
  if ((RecNum + RecSize) > MB_FILE_REC_SIZE) { *Exception = EX_ILLEGAL_DATA_ADDRESS; return 0; }

  FileAddr = (uint32_t)FileNum * MB_FILE_REC_SIZE + RecNum;
  if ((FileAddr + RecSize) > FILE_SIZE) { *Exception = EX_ILLEGAL_DATA_ADDRESS; return 0; }

  return ((short *)FILE_ADDR + FileAddr);
}
#endif


///////////////////////////////////

///////////EEPROM/////////////////////////////////////////////////////////////
int16_t GetBoardMemoryStatus(void)
{
 return Eeprom.Status;
}

int16_t IsBoardMemoryReady(void)
{
 return M95X_IsReady(&Eeprom);
}

int16_t ReadFromBoardMemory(int16_t Mode, int32_t Addr, int16_t *Buf, int32_t Size)
{

 if (M95X_IsError(&Eeprom)) return 0;
 if (((Addr + Size)<<1) > (int32_t)(Eeprom.Size)) {Eeprom.Status = -3; return 0;}
 M95X_Func(&Eeprom, M95X_READ, Mode, Addr << 1, (uint8_t *)Buf, Size << 1);
 do {M95X_Update(&Eeprom);} while (!M95X_IsReady(&Eeprom));

 return 0;
}

int16_t WriteToBoardMemory(int16_t Mode, int32_t Addr, int16_t *Buf, int32_t Size)
{
 if (Eeprom.Status == -1) return 0;
 if (((Addr + Size)<<1) > (int32_t)Eeprom.Size ) {Eeprom.Status = -3; return 0;}
 M95X_Func(&Eeprom, M95X_WRITE, Mode, Addr << 1, (uint8_t *)Buf, Size << 1);
 return 0;
}

static void EepromSetCs(uint8_t State)
{
if (State) { LL_GPIO_SetOutputPin(SPI2_FRAM_CS_GPIO_Port, SPI2_FRAM_CS_Pin); }
else 	   { LL_GPIO_ResetOutputPin(SPI2_FRAM_CS_GPIO_Port, SPI2_FRAM_CS_Pin); }
}

void ReadBoardData(void)
{
	  MbsProjData = &BrdData;
	  MbsProjDataSize = sizeof(BrdData)>>1;;
	  MBS_ReadProjectData();
	#if (PROJECT_MEMORY_ERASE == 1)

	  MBS_FLASH_EraseEEPROM();
	  BrdData.Version       = VERSION;
	  BrdData.DevNumber      = SlaveInfo.DeviceNumber;

	 #ifdef MB1
	  BrdData.Mb1.Slave      = MB1_ADR;
	  BrdData.Mb1.Speed      = MB1_SPEED;
	  BrdData.Mb1.Parity     = MB1_PARITY;
	  BrdData.Mb1.StopBits   = MB1_STOPBIT;
	 #endif

	 #ifdef MB2
	  BrdData.Mb2.Slave      = MB2_ADR;
	  BrdData.Mb2.Speed      = MB2_SPEED;
	  BrdData.Mb2.Parity     = MB2_PARITY;
	  BrdData.Mb2.StopBits   = MB2_STOPBIT;
	 #endif

	 #ifdef MB3
	  BrdData.Mb3.Slave      = MB3_ADR;
	  BrdData.Mb3.Speed      = MB3_SPEED;
	  BrdData.Mb3.Parity     = MB3_PARITY;
	  BrdData.Mb3.StopBits   = MB3_STOPBIT;
	 #endif

	  BrdData.Uip.macaddr[0] = 0x00;
	  BrdData.Uip.macaddr[1] = 0x20;
	  BrdData.Uip.macaddr[2] = 0x4A;
	  BrdData.Uip.macaddr[3] = 0x86;
	  BrdData.Uip.macaddr[4] = 0x27;
	  BrdData.Uip.macaddr[5] = 0x31;
	  BrdData.Uip.ipaddr[0]  = 192;
	  BrdData.Uip.ipaddr[1]  = 168;
	  BrdData.Uip.ipaddr[2]  = 127;
	  BrdData.Uip.ipaddr[3]  = 10;
	  BrdData.Uip.tcpSlave   = 1;
	  BrdData.Uip.tcpPort    = 502;
	  BrdData.Uip.dhcpEnable = 0;

	  BrdData.Status.all = 0;
	  BrdData.Status.bit.NewVersion = 1;
	  BrdData.Status.bit.WriteProjectData=0;
	  BrdData.Status.bit.WriteGlobalData=0;

	  DelayUsecs(2000);
	#else
	 if (BrdData.Version != VERSION)
	  {
	    BrdData.Version    = VERSION;
	   #ifdef MB1
	    BrdData.Mb1.Slave      = MB1_ADR;
	    BrdData.Mb1.Speed      = MB1_SPEED;
	    BrdData.Mb1.Parity     = MB1_PARITY;
	    BrdData.Mb1.StopBits   = MB1_STOPBIT;
	   #endif
	   #ifdef MB2

	    BrdData.Mb2.Slave      = MB2_ADR;
	    BrdData.Mb2.Speed      = MB2_SPEED;
	    BrdData.Mb2.Parity     = MB2_PARITY;
	    BrdData.Mb2.StopBits   = MB2_STOPBIT;
	   #endif
	   #ifdef MB3
	    BrdData.Mb3.Slave      = MB3_ADR;
	    BrdData.Mb3.Speed      = MB3_SPEED;
	    BrdData.Mb3.Parity     = MB3_PARITY;
	    BrdData.Mb3.StopBits   = MB3_STOPBIT;
	   #endif
	      BrdData.Uip.macaddr[0] = 0x00;
	      BrdData.Uip.macaddr[1] = 0x20;
	      BrdData.Uip.macaddr[2] = 0x4A;
	      BrdData.Uip.macaddr[3] = 0x86;
	      BrdData.Uip.macaddr[4] = 0x27;
	      BrdData.Uip.macaddr[5] = 0x31;
	      BrdData.Uip.ipaddr[0]  = 192;
	      BrdData.Uip.ipaddr[1]  = 168;
	      BrdData.Uip.ipaddr[2]  = 127;
	      BrdData.Uip.ipaddr[3]  = 10;
	      BrdData.Uip.tcpSlave   = 1;
	      BrdData.Uip.tcpPort    = 502;
	  #ifdef DHCP_EN
	      BrdData.Uip.dhcpEnable = 1;
	  #else
	      BrdData.Uip.dhcpEnable = 0;
	  #endif

	      BrdData.Status.all = 0;
	      BrdData.Status.bit.NewVersion = 1;
	      BrdData.Status.bit.WriteProjectData=1;
	      BrdData.Status.bit.WriteGlobalData=1;
	    #ifdef FLASH_MBS_CFG
	      BrdData.Status.bit.WriteConfiguration=1;
	    #endif
	  }
	#endif
}

void DataProject_Update (TBrdData *p)
{
#ifdef FLASH_MBS_CFG
 int16_t status_flash;
 if (!IsBoardMemoryReady()) {  return;  }

 if (p->Status.bit.NewVersion)        { p->Status.bit.NewVersion=0; p->Status.bit.WriteProjectData=1;       }
 else if (p->Status.bit.WriteConfiguration)
 {
  __disable_irq();
  p->Status.bit.WriteConfiguration=0;
  status_flash = EraseBoardMemory();
  if (!status_flash)  status_flash = MBS_FLASH_WriteConfiguration();
  MbsMemStatus = status_flash;
  __enable_irq();
 }
 else if (p->Status.bit.WriteGlobalData)    { p->Status.bit.WriteGlobalData=0;    MBS_FLASH_WriteGlobalData();	}
 else if (p->Status.bit.WriteProjectData)
 {
	 p->Status.bit.WriteProjectData=0;   MBS_WriteProjectData();
 }
 else if (p->Status.bit.ResetEEPROM)     	{ p->Status.bit.ResetEEPROM=0;    	  MBS_EraseEEPROM();			}
 else if (p->Status.bit.ResetProject)
    	{
	   	p->Status.bit.NewVersion = 1; //��������� ������ ��
	    p->Status.bit.WriteConfiguration=1; //������ ������� � ���������� ������
	    p->Status.bit.WriteProjectData=1; //������ ���������� �������
	    p->Status.bit.WriteGlobalData=1; //������ ���������� ����������
	    p->Status.bit.ResetProject=0; // ���������� ���� "����� ������������"
	    MBS_ReadConfiguration() ; // ��������� �� FLASH ������������
	    MbsStatus     = 0; // ���������� ���� ������� KERNEL
	    MbsEnable    = 1;  // �������� ������ MexBIOS
	    MBS_Init(); // �������� ������������� ���������� MEXBIOS
	    }
 #else
 if (!IsBoardMemoryReady()) {  return;  }
 if (p->Status.bit.NewVersion)        		{ p->Status.bit.NewVersion=0;    p->Status.bit.WriteProjectData=1;  }
 else if (p->Status.bit.WriteGlobalData)   { p->Status.bit.WriteGlobalData=0;  MBS_WriteGlobalData();      	}
 else if (p->Status.bit.WriteProjectData)  { p->Status.bit.WriteProjectData=0; MBS_WriteProjectData();         }
 else if (p->Status.bit.ResetEEPROM)     	{ p->Status.bit.ResetEEPROM=0;    MBS_EraseEEPROM();        		}
 else if (p->Status.bit.ResetProject)
 	 	 {
		 p->Status.bit.NewVersion = 1; //��������� ������ ��
		 p->Status.bit.WriteConfiguration=1; //������ ������� � ���������� ������
		 p->Status.bit.WriteProjectData=1; //������ ���������� �������
		 p->Status.bit.WriteGlobalData=1; ///������ ���������� ����������
		 p->Status.bit.ResetProject=0; // ���������� ���� "����� ������������"
		 MBS_ReadConfiguration() ;  // ��������� �� FLASH ������������
		 MbsStatus     = 0;  // ���������� ���� ������� KERNEL
		 MbsEnable    = 1;  // �������� ������ MexBIOS
		 MBS_Init(); // �������� ������������� ���������� MEXBIOS
	     }
#endif
}


/////UIP////////////////////////////////////////////////////////////////////////////////
#ifdef UIP
char UipSendFunc(char Data)
{
 #ifdef UIP_SPI1
	return SPI_send(0,(uint16_t)Data);
 #endif
}
void UipCsFunc(char Level)
 {
 #ifdef UIP_SPI1
	  if (Level) {  LL_GPIO_SetOutputPin(SPI1_ETH_CS_GPIO_Port, SPI1_ETH_CS_Pin); }
	  else 	      { LL_GPIO_ResetOutputPin(SPI1_ETH_CS_GPIO_Port, SPI1_ETH_CS_Pin); }
 #endif
 }
#endif

void ModBusRTUReinit(TMbRTUPort *hPort)
{
	SCI_deinit((hPort->Params).UartID);
    (hPort->Frame).Timer1_5 = 0;
    (hPort->Frame).Timer3_5 = 0;
    (hPort->Frame).TimerPre = 0;
    (hPort->Frame).TimerPost = 0;
    (hPort->Frame).TimerAck = 0;
    (hPort->Frame).TimerConn = 0;
    if ((hPort->Params).BaudRate < 0xc1)
    {
        (hPort->Frame).Timeout1_5 = (unsigned short)(((unsigned int)(uint8_t)(hPort->Params).Scale * 0xb4) / (unsigned int)(int)(hPort->Params).BaudRate);
        (hPort->Frame).Timeout3_5 = (unsigned short)(((unsigned int)(uint8_t)(hPort->Params).Scale * 0x181) / (unsigned int)(int)(hPort->Params).BaudRate);
    }
    else
    {
        (hPort->Frame).Timeout1_5 = (unsigned short)((int)((unsigned int)(uint8_t)(hPort->Params).Scale * 3) >> 2);
        (hPort->Frame).Timeout3_5 = (unsigned short)((int)((unsigned int)(uint8_t)(hPort->Params).Scale * 7) >> 2);
    }
    if ((hPort->Params).TrEnable != (TMbRTUTrEnable)0x0)
    {
        (*(hPort->Params).TrEnable)('\0');
    }
    SCI_init((hPort->Params).UartID, (int)(hPort->Params).BrrValue, (hPort->Params).Parity, 8, (hPort->Params).StopBits);
    (hPort->Packet).Request = '\0';
    (hPort->Packet).Response = '\0';
    (hPort->Packet).Exception = '\t';
    (hPort->Packet).Acknoledge = '\0';
    (hPort->Packet).Connected = '\0';
    (hPort->Packet).RetryCounter = '\0';
    (hPort->Frame).NewMessage = '\0';
    (hPort->Frame).WaitResponse = '\0';
    (hPort->Frame).RxCounter = 0;
    (hPort->Frame).RxLength = 0;
    (hPort->Frame).TxCounter = 0;
    (hPort->Frame).TxLength = 0;
    memset(&hPort->Stat, 0, 4);
    if ((hPort->Params).Mode == '\0')
    {
        (hPort->Frame).TimerPost = 1;
    }
    return;
}
