/***********************************************************************************************************************
* Copyright (c) Hands Free Team. All rights reserved.
* FileName: 
* Contact:  QQ Exchange Group -- 521037187
* Version:  V2.0
*
* LICENSING TERMS:
* The Hands Free is licensed generally under a permissive 3-clause BSD license. 
* Contributions are requiredto be made under the same license.
*
* History:
* <author>      <time>      <version>      <desc>
*
* Description:   
***********************************************************************************************************************/

#include "main_includes.h"
#include <string>
#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_it.h"

void USART1_IRQHandler(void)
{

    unsigned char data;
#if SYSTEM_SUPPORT_OS == 1
    OSIntEnter();
#endif
    
      data=USART1->DR;
		
				 
		mpu_6050.mpu_data[mpu_6050.data_len++]=data;         
		if (mpu_6050.mpu_data[0]!=0x55)               
		{
			mpu_6050.data_len=0;
			return;
		}
		if (mpu_6050.data_len<11) {return;}             
		else
		{
			switch(mpu_6050.mpu_data[1])                               
			{
				case 0x53:	
					memcpy(&mpu_6050.mpu_angle_temp,&mpu_6050.mpu_data[6],2);
					break;
				default:break;
			}
			mpu_6050.data_len=0;                                         
		}
	

        USART_ClearITPendingBit(USART1, USART_IT_RXNE);   // clear interrupt flag
  
#if SYSTEM_SUPPORT_OS == 1
    OSIntExit();
#endif
}

void USART2_IRQHandler(void)
{
    unsigned char data;
#if SYSTEM_SUPPORT_OS == 1
    OSIntEnter();
#endif
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        data = USART2->DR;
       if(usart2_queue.fullCheck()==0){
            usart2_queue.putData(data);
        }
       
				//board.setLedState(0,2);
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);   // clear interrupt flag
    }
#if SYSTEM_SUPPORT_OS == 1
    OSIntExit();
#endif
}

void USART3_IRQHandler(void)
{
    unsigned char data;
#if SYSTEM_SUPPORT_OS == 1
    OSIntEnter();
#endif
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        data=USART_ReceiveData(USART3);
        robot_head.gbpRxInterruptBuffer[(robot_head.gbRxBufferWritePointer++)] = data;
        USART_ClearITPendingBit(USART3,USART_IT_RXNE);  //clear interrupt flag
    }
#if SYSTEM_SUPPORT_OS == 1
    OSIntExit();
#endif
}

void UART4_IRQHandler(void)
{
    unsigned char data;
#if SYSTEM_SUPPORT_OS == 1
    OSIntEnter();
#endif
    if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
    {
        data = UART4->DR;
       if(usart2_queue.fullCheck()==0){
            usart2_queue.putData(data);
        }
      
				//board.setLedState(0,2);
        USART_ClearITPendingBit(UART4, USART_IT_RXNE);   // clear interrupt flag
    }
#if SYSTEM_SUPPORT_OS == 1
    OSIntExit();
#endif
}

void TIM6_DAC_IRQHandler(void)
{ 
#if SYSTEM_SUPPORT_OS == 1
    OSIntEnter();
#endif
    if(TIM_GetITStatus(TIM6 , TIM_IT_Update)== SET  )
    {
        board.cnt_1ms++;
        board.cnt_2ms++;
        board.cnt_5ms++;
        board.cnt_10ms++;
        board.cnt_20ms++;
        board.cnt_50ms++;
        TIM_ClearITPendingBit(TIM6 , TIM_FLAG_Update);     // clear interrupt flag
    }
#if SYSTEM_SUPPORT_OS == 1
    OSIntExit();
#endif
}

void CAN1_RX0_IRQHandler(void)
{
	CanRxMsg RxMessage;
	CAN_Receive(CAN1, 0 ,&RxMessage);

			if(RxMessage.StdId==0X11B)
        {
			// memcpy(motorcontrol.pre_byte,motorcontrol.byte,2*sizeof(RxMessage.Data[0]));
			memcpy(motorcontrol.byte, (RxMessage.Data+2),2*sizeof(RxMessage.Data[0]));
			// motorcontrol.Kalman_filtering(motorcontrol.pre_byte,motorcontrol.byte,0);
			// memcpy(motorcontrol.coord,(RxMessage.Data+4),4*sizeof(RxMessage.Data[0]));
			motorcontrol.Flag1_CAN_Receive=0XFF;
			
		}
			if(RxMessage.StdId==0X13B)
		{
			// memcpy(motorcontrol.pre_byte+2,motorcontrol.byte+2,2*sizeof(RxMessage.Data[0]));
			memcpy((motorcontrol.byte+2), (RxMessage.Data+2),2*sizeof(RxMessage.Data[0]));
			// motorcontrol.Kalman_filtering(motorcontrol.pre_byte+2,motorcontrol.byte+2,1);
			// memcpy((motorcontrol.coord+4),(RxMessage.Data+4),4*sizeof(RxMessage.Data[0]));
			motorcontrol.Flag1_CAN_Receive=0XFF;
			
		}
		
		
			if(RxMessage.Data[0]==0x08&&RxMessage.Data[2]==0x9B)
		{
			if(RxMessage.Data[1] == 0x2B)
		{
			motorcontrol.arm_cmd[0] = RxMessage.Data[7];  
			motorcontrol.arm_cmd[1] = RxMessage.Data[6];
			motorcontrol.arm_cmd[2] = RxMessage.Data[5];
			motorcontrol.arm_cmd[3] = RxMessage.Data[4];		
		}
		if(RxMessage.Data[1] == 0x2C)
		{
			motorcontrol.arm_cmd[4] = RxMessage.Data[7];  
			motorcontrol.arm_cmd[5] = RxMessage.Data[6];
			motorcontrol.arm_cmd[6] = RxMessage.Data[5];
			motorcontrol.arm_cmd[7] = RxMessage.Data[4];		
		}
		if(RxMessage.Data[1] == 0x2D)
		{
			motorcontrol.arm_cmd[8] = RxMessage.Data[7];  
			motorcontrol.arm_cmd[9] = RxMessage.Data[6];
			motorcontrol.arm_cmd[10] = RxMessage.Data[5];
			motorcontrol.arm_cmd[11] = RxMessage.Data[4];		
		}
		if(RxMessage.Data[1] == 0x2E)
		{
			motorcontrol.arm_cmd[12] = RxMessage.Data[7];  
			motorcontrol.arm_cmd[13] = RxMessage.Data[6];
			motorcontrol.arm_cmd[14] = RxMessage.Data[5];
			motorcontrol.arm_cmd[15] = RxMessage.Data[4];		
		}
		if(RxMessage.Data[1] == 0x2F)
		{
			motorcontrol.arm_cmd[16] = RxMessage.Data[7];  
			motorcontrol.arm_cmd[17] = RxMessage.Data[6];
			motorcontrol.arm_cmd[18] = RxMessage.Data[5];
			motorcontrol.arm_cmd[19] = RxMessage.Data[4];		
		}
		
		motorcontrol.Flag2_CAN_Receive=0XFF;
	}
	else
	{
		motorcontrol.Flag1_CAN_Receive=0;
		motorcontrol.Flag2_CAN_Receive=0;
	}
	
}

void CAN2_RX0_IRQHandler(void)
{
	CanRxMsg RxMessage;
	CAN_Receive(CAN2, 0 ,&RxMessage);

	if(RxMessage.StdId==0X0401)
	 {
		 
   memcpy(gypos.GYPOS_data,RxMessage.Data,4);
	 gypos.can2_flag=0xff; 
	 }
 }

void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) //�����ж�
	{
		platform.Pulse_Num++;
		if(platform.flag_dir==0xff)
			platform.Pulse_counter++;
		if(platform.flag_dir==0x01)
		  platform.Pulse_counter--;
		platform.current_h=((float)platform.Pulse_counter)/platform.per_pulse*platform.per_s;  //��������ǰ�߶ȣ������꣩
		if(platform.Pulse_Num>=platform.Desp_Num)    //������Ŀ�����������رն�ʱ��
		{
			platform.Pulse_Num=0;
			HF_PWM_Set_State(TIM3,1);//DISABLE
		
		}
	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //�����жϱ�־λ
}

void HardFault_Handler(void)
{
    unsigned char i;
#if SYSTEM_SUPPORT_OS == 1
    OSIntEnter();
#endif
    for(i=0;i<20;i++)
    {
        board.setBeepState(2);
        delay_ms(100);
    }
    //	__disable_fault_irq();  //reset
    //	NVIC_SystemReset();

#if SYSTEM_SUPPORT_OS == 1
    OSIntExit();
#endif
}

#ifdef __cplusplus
}
#endif 

