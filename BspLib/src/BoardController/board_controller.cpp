/***********************************************************************************************************************
* 晓萌 STM32F405主控板  
* 创建日期:2017/7/27
* 版本：V1.3
* 西北工业大学舞蹈机器人基地 家政2015电子组		
************************************************************************************************************************
* 1.文件名：board_controller.cpp
* 2.功能：板级抽象层，链接底层函数与package
* 3.已使用的IO口：
***********************************************************************************************************************/
#include "board_controller.h"

Board_controller  board_controller;

Board_controller::Board_controller(){
    cnt_1ms = 0;          //初始化各项参数
    cnt_2ms = 0;
    cnt_5ms = 0;
    cnt_10ms = 0;
    cnt_20ms = 0;
    cnt_50ms  = 0;
    cnt_500ms = 0 ;
	usart_value =0;
}
/***********************************************************************************************************************
* 1.函数名：boardBasicInit
* 2.功能：板级总初始化，包括各外设的初始化
***********************************************************************************************************************/
void Board_controller::boardBasicInit(void)    //各个端口初始化
{
    HF_System_Timer_Init();         //Initialize the measurement systemm
    ledInit();
	xm_communicate_Init();
    HF_Timer_Init(TIM6 , 1000);     //timer6 init , 1000us
	motor_commu_Init();  //gpio初始化
    GYPOS_commu_Init();
	xm_platform_init();
    //HF_ADC_Moder_Init(0X3E00 , 5 , 2.5f);  //ADC init
	mpu6050_Init();
}

/***********************************************************************************************************************
* Function:     void Board::boardBasicCall(void)
*
* Scope:        public
*
* Description:  update the underlying hardware information,Suggest calling every 10us
*
* Arguments:
*
* Return:
*
* Cpu_Time:  stm32f1(35 us)  stm32f4+nofpu(unknow us) stm32f4+fpu(unknow us)
*
* History:
***********************************************************************************************************************/
void Board_controller::boardBasicCall(void)   //100HZ
{
   
}
void Board_controller::motor_commu_Init(void)
{
	HF_CAN_Init(CAN1 , 0);
}

void Board_controller::motor_sendmsg(unsigned int ID, uint8_t *TxBuf)
{
	HF_CANTX_Message(CAN1 , ID , TxBuf);
}

void Board_controller::GYPOS_commu_Init(void)
{
	HF_CAN_Init(CAN2 , 0);
}



/***********************************************************************************************************************
* Function:
*
* Scope:
*
* Description:
*
* Arguments:
*
* Return:
*
* Cpu_Time:
*
* History:
***********************************************************************************************************************/
void Board_controller::ledInit(void)
{
    GPIO_InitTypeDef  GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_2 ;
    GPIO_Init(GPIOC , &GPIO_InitStruct);
	GPIO_ResetBits(GPIOC,GPIO_Pin_1 | GPIO_Pin_2);//GPIOA7,A8设置高，灯灭  GPIOA4控制485输入输出
}
void Board_controller::mpu6050_Init(void)
{
	  HF_USART_Init(USART1 , 921600 , 0);  //mpu6050 USART init
}

/***********************************************************************************************************************
* Function:
*
* Scope:
*
* Description:
*
* Arguments:
*
* Return:
*
* Cpu_Time:
*
* History:
***********************************************************************************************************************/
void Board_controller::setLedState(uint8_t led_id, uint8_t operation){
    if ( led_id == 0)
	{
        if(operation == 0){ GPIO_SetBits(GPIOC , GPIO_Pin_1);}
        else if(operation == 1) { GPIO_ResetBits(GPIOC , GPIO_Pin_1);}
				else if(operation == 2) { GPIO_ToggleBits(GPIOC , GPIO_Pin_1);}
    }
    else if(led_id == 1){
        if(operation == 0){ GPIO_SetBits(GPIOC , GPIO_Pin_2);}
        else if(operation == 1) { GPIO_ResetBits(GPIOC , GPIO_Pin_2);}
				else if(operation == 2) { GPIO_ToggleBits(GPIOC , GPIO_Pin_2);}
    }
}

/***********************************************************************************************************************
* Function:485init
*
* Scope:
*
* Description:
*
* Arguments:
*
* Return:
*
* Cpu_Time:
*
* History:
***********************************************************************************************************************/
void Board_controller::xm_communicate_Init(void)
{
	rs485Init(USART2,921600,0);
	rs485_cmd(0);
	//HC05_Init(UART4,9600,0);
}
void Board_controller::rs485_cmd(unsigned char state)//0 rx_enable 1 tx_enable
{
	if(state==0)GPIO_ResetBits(GPIOA,GPIO_Pin_1);
	if(state==1)GPIO_SetBits(GPIOA,GPIO_Pin_1);
}


void Board_controller::xm_platform_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	HF_PWMOut_Init(TIM3 , 84-1 , 200-1 , 0);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;     //用于控制DIR方向
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;        //普通输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;      //推挽输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure);              //初始化PA8
	
}

void Board_controller::Foreward_Move(void)
{
	GPIO_SetBits(GPIOA,GPIO_Pin_8);
}

void Board_controller::Backward_Move(void)
{
	GPIO_ResetBits(GPIOA,GPIO_Pin_8);
}



/***********************************************************************************************************************
* Function:
*
* Scope:
*
* Description:
*
* Arguments:
*
* Return:
*
* Cpu_Time:
*
* History:
***********************************************************************************************************************/
void Board_controller::motorInterfaceInit(uint8_t motor_id , float motor_pwm_T)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;

    if(motor_id == 1 ){
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE , ENABLE);
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
        GPIO_Init(GPIOE , &GPIO_InitStructure);
        GPIO_ResetBits(GPIOE , GPIO_Pin_8);
        HF_Encoder_Init(TIM2,1);   //encoder init for PID speed control
    }
    else if(motor_id == 2){
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE , ENABLE);
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 ;
        GPIO_Init(GPIOE , &GPIO_InitStructure);
        GPIO_ResetBits(GPIOE , GPIO_Pin_12);
        HF_Encoder_Init(TIM3,1);
    }
    else if(motor_id == 3){
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE , ENABLE);
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
        GPIO_Init(GPIOE , &GPIO_InitStructure);
        GPIO_ResetBits(GPIOE , GPIO_Pin_4);
        HF_Encoder_Init(TIM4,1);
    }
    else if(motor_id == 4){
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE , ENABLE);
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
        GPIO_Init(GPIOE , &GPIO_InitStructure);
        GPIO_ResetBits(GPIOE , GPIO_Pin_15);
        HF_Encoder_Init(TIM5,0);
    }

    //motor_pwm_T = 5000 , TIM1 motor pwm frequency  = (168M/4) / motor_pwm_T  = 16.8K
    HF_PWMOut_Init(TIM1 , 2-1 , motor_pwm_T , 1);

    //motor_pwm_T = 5000 , TIM9 motor pwm frequency  = (168M/4) / motor_pwm_T = 16.8K
    HF_PWMOut_Init(TIM9 , 2-1 , motor_pwm_T , 0);

    //motor_pwm_T = 5000 , TIM12 motor pwm frequency = (84M/2) / motor_pwm_T  = 16.8K
    HF_PWMOut_Init(TIM12 , 0 , motor_pwm_T , 0);
}

/***********************************************************************************************************************
* Function:
*
* Scope:
*
* Description:
*
* Arguments:
*
* Return:
*
* Cpu_Time:
*
* History:
***********************************************************************************************************************/
void Board_controller::motorEnable(uint8_t motor_id)
{
    if(motor_id == 1 ){
        GPIO_SetBits(GPIOE , GPIO_Pin_8);
    }
    else if(motor_id == 2){
        GPIO_SetBits(GPIOE , GPIO_Pin_12);
    }
    else if(motor_id == 3){
        GPIO_SetBits(GPIOE , GPIO_Pin_4);
    }
    else if(motor_id == 4){
        GPIO_SetBits(GPIOE , GPIO_Pin_15);
    }
}

/***********************************************************************************************************************
* Function:
*
* Scope:
*
* Description:
*
* Arguments:
*
* Return:
*
* Cpu_Time:
*
* History:
***********************************************************************************************************************/
/***********************************************************************************************************************
* Function:
*
* Scope:
*
* Description:
*
* Arguments:
*
* Return:
*
* Cpu_Time:
*
* History:
***********************************************************************************************************************/
void Board_controller::motorSetPWM(uint8_t motor_id , int pwm_value)
{
    if( motor_id ==1 ){
        if( pwm_value > 0) {
            HF_PWM_Set_TIM1_CCR1((unsigned short int)pwm_value);
            HF_PWM_Set_TIM1_CCR2(0);
            return;
        }
        else{
            HF_PWM_Set_TIM1_CCR1(0);
            HF_PWM_Set_TIM1_CCR2((unsigned short int)-pwm_value);
            return;
        }
    }
    else if(motor_id == 2 ){
        if( pwm_value > 0) {
            HF_PWM_Set_TIM1_CCR3((unsigned short int)pwm_value);
            HF_PWM_Set_TIM1_CCR4(0);
            return;
        }
        else{
            HF_PWM_Set_TIM1_CCR3(0);
            HF_PWM_Set_TIM1_CCR4((unsigned short int)-pwm_value);
            return;
        }
    }
    else if(motor_id ==3 ){
        if( pwm_value > 0) {
            HF_PWM_Set_TIM9_CCR1((unsigned short int)pwm_value);
            HF_PWM_Set_TIM9_CCR2(0);
            return;
        }
        else{
            HF_PWM_Set_TIM9_CCR1(0);
            HF_PWM_Set_TIM9_CCR2((unsigned short int)-pwm_value);
            return;
        }
    }
    else if(motor_id ==4 ){
        if( pwm_value > 0) {
            HF_PWM_Set_TIM12_CCR1((unsigned short int)pwm_value);
            HF_PWM_Set_TIM12_CCR2(0);
            return;
        }
        else{
            HF_PWM_Set_TIM12_CCR1(0);
            HF_PWM_Set_TIM12_CCR2((unsigned short int)-pwm_value);
            return;
        }
    }

}

/***********************************************************************************************************************
* Function:
*
* Scope:
*
* Description:
*
* Arguments:
*
* Return:
*
* Cpu_Time:
*
* History:
***********************************************************************************************************************/
void Board_controller::axServoInterfaceInit(void)
{
//    GPIO_InitTypeDef GPIO_InitStructure;

    HF_USART_Init(USART3 , 115200 , 0);

//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD , ENABLE);
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
//    GPIO_Init(GPIOD , &GPIO_InitStructure);
}
void Board_controller::axServoTxModel(void)
{
    USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);//开启相关中断
}
void Board_controller::axServoRxModel(void)
{
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启相关中断
}
void Board_controller::axServoSendTxByte(uint8_t tx_byte)
{
    USART_SendData(USART3 , tx_byte);
    while(USART_GetFlagStatus(USART3 , USART_FLAG_TC) == RESET);
}
uint8_t Board_controller::axServoGetRxByte(uint8_t *error)
{
    return 0;
}

/***********************************************************************************************************************
* Function:
*
* Scope:
*
* Description:
*
* Arguments:
*
* Return:
*
* Cpu_Time:
*
* History:
***********************************************************************************************************************/
void Board_controller::sbusInterfaceInit(void)
{
    HF_USART_Init(USART2,100000,1);
}

/***********************************************************************************************************************
* Function:
*
* Scope:
*
* Description:
*
* Arguments:
*
* Return:
*
* Cpu_Time:
*
* History:
***********************************************************************************************************************/
/***********************************************************************************************************************
* Function:
*
* Scope:
*
* Description:
*
* Arguments:
*
* Return:
*
* Cpu_Time:
*
* History:
***********************************************************************************************************************/
void Board_controller::pwmInterfaceInit(TIM_TypeDef* TIMx , uint8_t mode)
{
    if(mode == 0)     //motor mode
    {
        if(TIMx == TIM1 || TIMx == TIM8 || TIMx == TIM9)
        {
            HF_PWMOut_Init(TIMx , 2-1 , 5000 , 1);
        }
        else
        {
            HF_PWMOut_Init(TIMx , 0 , 5000 , 0);
        }
    }
    else if(mode == 1)  //analog servo mode
    {
        if(TIMx == TIM1 || TIMx == TIM8 || TIMx == TIM9)
        {
            HF_PWMOut_Init(TIMx , 168-1 , 20000 , 1);
        }
        else
        {
            HF_PWMOut_Init(TIMx  , 84-1 , 20000 , 0);
        }
    }
}

void Board_controller::setPWMValue(uint8_t channel_x , float pwm_value)
{
    if(pwm_value <0) return;
    if(channel_x == 1){
        HF_PWM_Set_TIM1_CCR1((unsigned short int)pwm_value);
    }
    if(channel_x == 2){
        HF_PWM_Set_TIM1_CCR2((unsigned short int)pwm_value);
    }
    if(channel_x == 3){
        HF_PWM_Set_TIM1_CCR3((unsigned short int)pwm_value);
    }
    if(channel_x == 4){
        HF_PWM_Set_TIM1_CCR4((unsigned short int)pwm_value);
    }
    if(channel_x == 5){
        HF_PWM_Set_TIM8_CCR1((unsigned short int)pwm_value);
    }
    if(channel_x == 6){
        HF_PWM_Set_TIM8_CCR2((unsigned short int)pwm_value);
    }
    if(channel_x == 7){
        HF_PWM_Set_TIM9_CCR1((unsigned short int)pwm_value);
    }
    if(channel_x == 8){
        HF_PWM_Set_TIM9_CCR2((unsigned short int)pwm_value);
    }
    if(channel_x == 9){
        HF_PWM_Set_TIM12_CCR1((unsigned short int)pwm_value);
    }
    if(channel_x == 10){
        HF_PWM_Set_TIM12_CCR2((unsigned short int)pwm_value);
    }
}

