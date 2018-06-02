/*********************************************************************************************************
*
* File                : main.c
* Hardware Environment: 
* Build Environment   : RealView MDK-ARM  Version: 4.20
* Version             : V1.0
* By                  : 
*
*                                  (c) Copyright 2005-2011, WaveShare
*                                       http://www.waveshare.net
*                                          All Rights Reserved
*
*********************************************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "SysTick/systick.h" 
#include <stdlib.h>
#include <stdio.h>

#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/* Private variables ---------------------------------------------------------*/
uint8_t CAN1_RxRdy,CAN2_RxRdy;
uint16_t CAN1_Val_Tx,CAN1_Val_Rx,CAN2_Val_Tx,CAN2_Val_Rx;
CanTxMsg TxMessage;

/* Private function prototypes -----------------------------------------------*/
void GPIO_Configuration(void);
void USART_Configuration(void);
void NVIC_Configuration(void);
void CAN_Configuration(void);
void CanWriteData( CAN_TypeDef* CANx ,CanTxMsg *TxMessage );

/*******************************************************************************
* Function Name  : Delay
* Description    : Delay Time
* Input          : - nCount: Delay Time
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void  Delay (uint32_t nCount)
{
  for(; nCount != 0; nCount--);
}


/*******************************************************************************
* Function Name  : main
* Description    : Main program
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
int main(void)
{
  uint16_t n;
  delay_init();
  CAN_Configuration();
  USART_Configuration();

  printf("\r\n****************************************************************\r\n");
  printf("CAN-Bus Test \r\n");
  n=0;
  /* Infinite loop */
  while (1)
  {
    CAN1_Val_Tx = n;
    TxMessage.Data[0] = CAN1_Val_Tx>>8;
    TxMessage.Data[1] = CAN1_Val_Tx; 	
    CanWriteData(CAN1,&TxMessage);
    delay_ms (500);          /* delay 500 ms */

    if( CAN1_RxRdy == ENABLE )
    {
      CAN1_RxRdy = DISABLE;
      printf("CAN1 Receive Data: %d \r\n\r\n",CAN1_Val_Rx);
    }

    if( CAN2_RxRdy == ENABLE )
    {
      CAN2_RxRdy = DISABLE;
      TxMessage.Data[0] = CAN2_Val_Tx>>8;
      TxMessage.Data[1] = CAN2_Val_Tx; 	
      CanWriteData(CAN2,&TxMessage);
      printf("CAN2 Receive Data: %d \r\n",CAN2_Val_Rx);
    }
    n++;
    if(n==10000)
    {
      n=0;
    }
  }
}

/*******************************************************************************
* Function Name  : GPIO_Configuration
* Description    : Configures the different GPIO ports.
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO ,ENABLE);
  /* CAN1 and CAN2 periph clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1 | RCC_APB1Periph_CAN2, ENABLE);						 
  /* Remap CAN1 and CAN2 GPIOs */
  GPIO_PinRemapConfig(GPIO_Remap2_CAN1 , ENABLE);
  GPIO_PinRemapConfig(GPIO_Remap_CAN2, ENABLE);	 
  /* Configure CAN1 RX pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  /* Configure CAN1 pin: TX */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  /* Configure CAN2 RX pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  /* Configure CAN2 pin: TX */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);  

  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE); 						 
/**
 *  LED1 -> PB0 , LED2 -> PB1 , LED3 -> PB14 , LED4 -> PB15
 */					 
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Key */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    : Configures the nested vectored interrupt controller.
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

  /* Enable CAN1 RX0 interrupt IRQ channel */
  NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Enable CAN2 RX0 interrupt IRQ channel */
  NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
* Function Name  : CAN_Configuration
* Description    : Configures the CAN
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void CAN_Configuration(void)
{
  CAN_InitTypeDef        CAN_InitStructure;
  CAN_FilterInitTypeDef  CAN_FilterInitStructure;

  NVIC_Configuration();
  GPIO_Configuration();
  /* CAN register init */
  CAN_DeInit(CAN1);
  CAN_DeInit(CAN2);
  CAN_StructInit(&CAN_InitStructure);

  CAN1_RxRdy = CAN2_RxRdy = DISABLE;
  CAN1_Val_Tx = CAN1_Val_Rx = CAN2_Val_Tx = CAN2_Val_Rx = 0;

  /* CAN cell init */
  CAN_InitStructure.CAN_TTCM = DISABLE; /* 时间触发禁止, 时间触发：CAN硬件的内部定时器被激活，并且被用于产生时间戳 */
  CAN_InitStructure.CAN_ABOM = DISABLE; /* 自动离线禁止，自动离线：一旦硬件监控到128次11个隐性位，就自动退出离线状态。在这里要软件设定后才能退出 */
  CAN_InitStructure.CAN_AWUM = DISABLE; /* 自动唤醒禁止，有报文来的时候自动退出休眠	*/
  CAN_InitStructure.CAN_NART = DISABLE; /* 报文重传, 如果错误一直传到成功止，否则只传一次 */
  CAN_InitStructure.CAN_RFLM = DISABLE; /* 接收FIFO锁定, 1--锁定后接收到新的报文摘不要，0--接收到新的报文则覆盖前一报文	*/
  CAN_InitStructure.CAN_TXFP = ENABLE;  /* 发送优先级  0---由标识符决定  1---由发送请求顺序决定	*/
  CAN_InitStructure.CAN_Mode = CAN_Mode_Normal; /* 模式	*/
  CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;      /* 重新同步跳宽，只有can硬件处于初始化模式时才能访问这个寄存器 */
  CAN_InitStructure.CAN_BS1 = CAN_BS1_3tq;      /* 时间段1 */
  CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;      /* 时间段2 */
  CAN_InitStructure.CAN_Prescaler = 5;          /* 波特率预分频数 */  
 
  /* 波特率计算方法 */
  /* CANbps= Fpclk/((BRP+1)*((Tseg1+1)+(Tseg2+1)+1)  此处计算为  CANbps=36000000/(45*(4+3+1))=1200kHz */   														  //此处Tseg1+1 = CAN_BS1_8tp
  /* 配置大方向: Tseg1>=Tseg2  Tseg2>=tq; Tseg2>=2TSJW */						

  /*Initializes the CAN1  and CAN2 */
  CAN_Init(CAN1, &CAN_InitStructure);
  CAN_Init(CAN2, &CAN_InitStructure);

  /* CAN1 filter init */
  /* 配置CAN过滤器 */
  /* 32位对应的id */
  /* stdid[10:0]，extid[17:0],ide,rtr	*/
  /* 16位对应的id */
  /* stdid[10:0],ide,rtr,extid[17:15] */
  /* 一般使用屏蔽模式	*/
  /* 要注意的是fifo接收存满了中断，还有就是fifo的概念，即取的一直是最早那一个数据， 要释放才能取下一个数据 */
  /* 常使用的中断有 */
  /* 1,有信息中断，即fifo挂号中断 */
  /* 2,fifo满中断	*/
  /* 3,fifo满之后又有信息来则中断，即fifo溢出中断	*/
  CAN_FilterInitStructure.CAN_FilterNumber = 0;     /* 过滤器1 */
  CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;  /* 屏敝模式 */
  CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; /* 32位 */
  CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;  /* 以下四个都为0, 表明不过滤任何id */
  CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;  
  CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
  CAN_FilterInit(&CAN_FilterInitStructure);

  /* CAN2 filter init */
  CAN_FilterInitStructure.CAN_FilterNumber = 14;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;
  CAN_FilterInit(&CAN_FilterInitStructure);

  /* IT Configuration for CAN1 */  
  CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);

  /* IT Configuration for CAN2 */  
  CAN_ITConfig(CAN2, CAN_IT_FMP0, ENABLE);
}

/*******************************************************************************
* Function Name  : CanWriteData
* Description    : Can Write Date to CAN-BUS
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void CanWriteData( CAN_TypeDef* CANx ,CanTxMsg *TxMessage )
{
  /* transmit */
  TxMessage->StdId = 0xA5A5;     /* 设置标准id  注意标准id的最高7位不能全是隐性(1)。共11位 */
//TxMessage->ExtId = 0x00;       //设置扩展id  扩展id共18位
  TxMessage->RTR = CAN_RTR_DATA; /* 设置为数据帧 */
  TxMessage->IDE = CAN_ID_STD;   /* 使用标准id	*/
  TxMessage->DLC = 8;            /* 数据长度, can报文规定最大的数据长度为8字节 */
 
  CAN_Transmit(CANx,TxMessage);  /* 返回这个信息请求发送的邮箱号0,1,2或没有邮箱申请发送no_box */	
}

/*******************************************************************************
* Function Name  : CAN1_RX0_IRQHandler
* Description    : This function handles CAN1 RX0 interrupts 
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void CAN1_RX0_IRQHandler(void)
{
  CanRxMsg RxMessage;
  CAN_Receive(CAN1,CAN_FIFO0, &RxMessage);  /* 此函数包含释放提出报文了的,在非必要时,不需要自己释放 */

  CAN1_Val_Rx = RxMessage.Data[0] <<8 ;
  CAN1_Val_Rx += RxMessage.Data[1];

  CAN1_RxRdy = ENABLE;
}

/*******************************************************************************
* Function Name  : CAN2_RX0_IRQHandler
* Description    : This function handles CAN2 RX0 interrupts
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void CAN2_RX0_IRQHandler(void)
{
  CanRxMsg RxMessage;
  CAN_Receive(CAN2,CAN_FIFO0, &RxMessage);  /* 此函数包含释放提出报文了的,在非必要时,不需要自己释放 */

  CAN2_Val_Rx = RxMessage.Data[0] <<8 ;
  CAN2_Val_Rx += RxMessage.Data[1];

  CAN2_Val_Tx = 10000-CAN2_Val_Rx;

  CAN2_RxRdy = ENABLE;
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/

