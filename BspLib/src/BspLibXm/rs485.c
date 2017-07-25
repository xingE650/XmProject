/***********************************************************************************************************************
* Copyright (c) Hands Free Team. All rights reserved.
* FileName: usart.c
* Contact:  QQ Exchange Group -- 521037187
* Version:  V2.0
*
* LICENSING TERMS:  
* The Hands Free is licensed generally under a permissive 3-clause BSD license. 
* Contributions are requiredto be made under the same license.
*
* History: 
* <author>      <time>      <version>      <desc>
* mawenke       2015.10.1   V1.0           creat this file
*
* Description:    STM32F4 Initialization of USART
***********************************************************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif 

#include "rs485.h"
#include "nvic.h"

/***********************************************************************************************************************
* Function:     rs485Init(USART_TypeDef* USARTx , unsigned int BaudRate , uint8_t GPIO_AF)
*
* Scope:        public
*
* Description:  Initialize USART 
*
* Arguments:
*
* Return:
*
* Cpu_Time:  
*
* History:
***********************************************************************************************************************/
void rs485Init(USART_TypeDef* USARTx , uint32_t BaudRate , uint8_t GPIO_AF)
{

    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//Multiplexing
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//frequency 50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //Multiplexing push-pull output
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;   //pull up
    
    if(USARTx==USART1){
        
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
        
        if(GPIO_AF == 0){
            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
            GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);
            GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
            GPIO_Init(GPIOA,&GPIO_InitStructure);
        }
        else if(GPIO_AF == 1){
            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
            GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_USART1);
            GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_USART1);
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
            GPIO_Init(GPIOB,&GPIO_InitStructure);
        }
        USART1_NVIC_Configuration();
    }
		//we use the usart2
    else if(USARTx==USART2){
        
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
        
        if(GPIO_AF == 0) {
            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
            GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);
            GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;
            GPIO_Init(GPIOA, &GPIO_InitStructure);
        }
        else if(GPIO_AF == 1){
            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
            GPIO_PinAFConfig(GPIOD,GPIO_PinSource5,GPIO_AF_USART2);
            GPIO_PinAFConfig(GPIOD,GPIO_PinSource6,GPIO_AF_USART2);
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6;
            GPIO_Init(GPIOD, &GPIO_InitStructure);
        }
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; //GPIOA1
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//输出
				GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
				GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽输出
				GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
				GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA1
        USART2_NVIC_Configuration();
        
    }
    
    else if(USARTx==USART3){
        
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
        
        if(GPIO_AF == 0){
            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
            GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3);
            GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3);
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
            GPIO_Init(GPIOB, &GPIO_InitStructure);
        }
        else if(GPIO_AF == 1){
            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
            GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_USART3);
            GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_USART3);
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
            GPIO_Init(GPIOC, &GPIO_InitStructure);
        }
        else if(GPIO_AF == 2){
            
            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
            GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,GPIO_AF_USART3);
            GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_USART3);
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;
            GPIO_Init(GPIOD, &GPIO_InitStructure);
        }
        
        USART3_NVIC_Configuration();
    }
    else if(USARTx==UART4){
        
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
        
        if(GPIO_AF == 0)
        {
            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
            GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_UART4);
            GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_UART4);
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
            GPIO_Init(GPIOA, &GPIO_InitStructure);
        }
        else if(GPIO_AF == 1){
            
        }
        UART4_NVIC_Configuration();
    }
    else if(USARTx==UART5){
        
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
        
        if(GPIO_AF == 0)
        {
            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD, ENABLE);
            GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_UART5);
            GPIO_PinAFConfig(GPIOD,GPIO_PinSource2,GPIO_AF_UART5);
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
            GPIO_Init(GPIOC, &GPIO_InitStructure);
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
            GPIO_Init(GPIOD, &GPIO_InitStructure);
        }
        else if(GPIO_AF == 1){
            
        }
        UART5_NVIC_Configuration();
        
    }
    else if(USARTx==USART6){
        
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);
        
        if(GPIO_AF == 0)
        {
            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
            GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_USART6);
            GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_USART6);
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
            GPIO_Init(GPIOC,&GPIO_InitStructure);
        }
        else if(GPIO_AF == 1)
        {
            
        }
        USART6_NVIC_Configuration();
    }
    
    USART_InitStructure.USART_BaudRate = BaudRate ;  //set naud rate 9600 115200 256000 921600
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USARTx, &USART_InitStructure);
    
    USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE); //enable interrupt
    USART_ClearITPendingBit(USARTx, USART_IT_RXNE);
    USART_Cmd(USARTx, ENABLE);                     //enable usart
    
}
