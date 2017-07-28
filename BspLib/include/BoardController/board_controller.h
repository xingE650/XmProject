#ifndef BOARD_CONTROLLER_H
#define	BOARD_CONTROLLER_H
#ifdef __cplusplus
extern "C" {
#endif



#include "bsplib.h"


#ifdef __cplusplus
}
#endif 

class Board_controller
{
public:
    
    uint8_t system_init; //state of system: 0-->not initialize  1-->initialized
    uint16_t cnt_1ms;
    uint16_t cnt_2ms;
    uint16_t cnt_5ms;
    uint16_t cnt_10ms;
    uint16_t cnt_20ms;
    uint16_t cnt_50ms;
    uint16_t cnt_500ms;
    uint16_t flashSize;    //Unit: KB
	uint16_t usart_value;

public:
    Board_controller();
    void boardBasicInit(void);
    void boardBasicCall(void);
    /**********************************************************************************************************************/
    void motor_commu_Init(void);
	void motor_sendmsg(unsigned int ID, uint8_t *TxBuf);
    void GYPOS_commu_Init(void);
    void mpu6050_Init(void);
    void setLedState(uint8_t led_id, uint8_t operation);
	void xm_communicate_Init(void);
	void rs485_cmd(unsigned char state);
	void xm_platform_init(void);
	void Foreward_Move(void);
    void Backward_Move(void);
    /**********************************************************************************************************************/
    void motorInterfaceInit(uint8_t motor_id , float motor_pwm_T); //package "PAKG_MOTOR" support functions
    void motorEnable(uint8_t motor_id);
    void motorDisable(uint8_t motor_id);
    void motorSetPWM(uint8_t motor_id , int pwm_value);
    /**********************************************************************************************************************/
    void axServoInterfaceInit(void); //package " PAKG_SERVO" support  functions
    void axServoTxModel(void);
    void axServoRxModel(void);
    void axServoSendTxByte(uint8_t tx_byte);
    uint8_t axServoGetRxByte(uint8_t *error);
    /**********************************************************************************************************************/
    void sbusInterfaceInit(void);
    void pwmInterfaceInit(TIM_TypeDef* TIMx , uint8_t mode);
    void setPWMValue(uint8_t channel_x , float pwm_value);  //channel_x 1~6
    /**********************************************************************************************************************/\

private:
    void systemMonitoring(void);
    void ledInit(void);


};

extern Board_controller board_controller;

#endif 

