#include"motor_control_xm.h"
#include<math.h>
#include <string>
MOTORCONTROL motorcontrol;

void MOTORCONTROL::motor_reset(unsigned int CAN_ID)
{
		uint8_t can_cmd[8];
    
   
		can_cmd[0] = 0x55;
		can_cmd[1] = 0x55;
		can_cmd[2] = 0x55;
		can_cmd[3] = 0x55;
		can_cmd[4] = 0x55;
		can_cmd[5] = 0x55;
		can_cmd[6] = 0x55;
		can_cmd[7] = 0x55;
    
		board.motor_sendmsg(CAN_ID,can_cmd);//������Ӧ����������ָ��	
	

}
void MOTORCONTROL::arm_home(void)
{
	// float cmd1 = 0.0, cmd2 = 1.52, cmd3 = 0.17, cmd4 = -1.1, cmd5 = 0; 
	float cmd1 = 0.0, cmd2 = 1.57, cmd3 = 0, cmd4 = 0; 
	uint8_t num;
	float action[4];
	*(action) = cmd1;
	*(action+1) = -3*cmd2;
	*(action+2) = cmd3;
	*(action+3) =  cmd4;
	
	for(num=0;num<4;num++)
	{
		arm_angle(num+0X2C,*(action+num));
	}
	delay_ms(5000);
	arm_clear();
}
void MOTORCONTROL::arm_clear(void)
{
	unsigned int CAN_ID=0x2D;
	uint8_t can_cmd[8]={0x08,0x2D,0x98,0x00,0x00,0x00,0x00,0x00};
	for(int i=0;i<3;i++)
	{
		board.motor_sendmsg(CAN_ID+i,can_cmd);
		delay_us(1000);
	}
}	



void MOTORCONTROL::motor_config(unsigned int CAN_ID,unsigned char temp_time)//interval_time=(temp_time*20)ms
{
		
    uint8_t can_cmd[8];
     
    can_cmd[0] = temp_time;
    can_cmd[1] = 0x00;
    can_cmd[2] = 0x55;
    can_cmd[3] = 0x55;
    can_cmd[4] = 0x55;
    can_cmd[5] = 0x55;
    can_cmd[6] = 0x55;
    can_cmd[7] = 0x55;
    
   
    board.motor_sendmsg(CAN_ID,can_cmd);//������Ӧ����������ָ��	
}

/****************************************************************************************
                                     ģʽѡ��ָ��
CAN_ID��ȡֵ��Χ���£�
DRV_ALL_MODE_CHOICE_ID
DRV01_MODE_CHOICE_ID    DRV02_MODE_CHOICE_ID    DRV03_MODE_CHOICE_ID    DRV04_MODE_CHOICE_ID    DRV05_MODE_CHOICE_ID    
DRV06_MODE_CHOICE_ID    DRV07_MODE_CHOICE_ID    DRV08_MODE_CHOICE_ID    DRV09_MODE_CHOICE_ID    DRV10_MODE_CHOICE_ID    
DRV11_MODE_CHOICE_ID    DRV12_MODE_CHOICE_ID    DRV13_MODE_CHOICE_ID    DRV14_MODE_CHOICE_ID    DRV15_MODE_CHOICE_ID

mode��ȡֵ��Χ���£�
ENTER_PWM_MODE
ENTER_PWM_CURRENT_MODE
ENTER_PWM_VELOCITY_MODE
ENTER_PWM_POSITION_MODE
ENTER_PWM_VELOCITY_POSITION_MODE
ENTER_CURRENT_VELOCITY_MODE
ENTER_CURRENT_POSITION_MODE
ENTER_CURRENT_VELOCITY_POSITION_MODE
*****************************************************************************************/
void MOTORCONTROL::motor_mode(unsigned int CAN_ID,unsigned char mode)
{
	
    uint8_t can_cmd[8];
    
    
    can_cmd[0] = mode;
    can_cmd[1] = 0x55;
    can_cmd[2] = 0x55;
    can_cmd[3] = 0x55;
    can_cmd[4] = 0x55;
    can_cmd[5] = 0x55;
    can_cmd[6] = 0x55;
    can_cmd[7] = 0x55;
    

    board.motor_sendmsg(CAN_ID,can_cmd);//������Ӧ����������ָ��	
}
void MOTORCONTROL::motor_clear(unsigned int CAN_ID)
{
		uint8_t can_cmd[8];
    
    can_cmd[0] = (unsigned char)(5000>>8)&0xff;
    can_cmd[1] = (unsigned char)5000&0xff;
    can_cmd[2] = (unsigned char)(0);
    can_cmd[3] = (unsigned char)(0);
    can_cmd[4] = 0x55;
    can_cmd[5] = 0x55;
    can_cmd[6] = 0x55;
    can_cmd[7] = 0x55;
		board.motor_sendmsg(CAN_ID,can_cmd);//������Ӧ����������ָ��	
   
}

void MOTORCONTROL::motor_speed(unsigned int CAN_ID,short temp_velocity)
{
	 
		uint8_t can_cmd[8];
    if(temp_velocity > 2000)
    {
        temp_velocity = 2000;
    }
    else if(temp_velocity < -2000)
    {
        temp_velocity = -2000;
    }
    
    can_cmd[0] = (unsigned char)(5000>>8)&0xff;
    can_cmd[1] = (unsigned char)5000&0xff;
    can_cmd[2] = (unsigned char)((temp_velocity>>8)&0xff);
    can_cmd[3] = (unsigned char)(temp_velocity&0xff);
    can_cmd[4] = 0x55;
    can_cmd[5] = 0x55;
    can_cmd[6] = 0x55;
    can_cmd[7] = 0x55;
    
    board.motor_sendmsg(CAN_ID,can_cmd);//������Ӧ����������ָ��

}

void MOTORCONTROL::chassis_init(void)
{
	delay_ms(1000);
	motor_reset(0x100);
	delay_ms(1000);
	motor_mode(0x111,0x03);
	motor_mode(0x131,0x03);
	delay_ms(1000);
	motor_config(0x11a,0x02);
    motor_config(0x13a,0x02);
	delay_ms(1000);																								
}
void MOTORCONTROL::chassis_clear(void)
{
	motor_clear(0x104);
	motor_clear(0x114);
}
void MOTORCONTROL::wheel_control(void)
{
	short Speed_M1,Speed_M2;
		Speed_M1=(my_robot.expect_motor_speed.servo1)/(2*3.1416)*2940;		//2940= 60*49,degree = round/min
		motor_speed(0x114,(short)(-Speed_M1));
		Speed_M2=(my_robot.expect_motor_speed.servo2)/(2*3.1416)*2940;		
		motor_speed(0x134,(short)Speed_M2);
}
void MOTORCONTROL::arm_angle(unsigned char ARM_ID,float angle)
{
			uint8_t can_cmd[8];
			float pluse;
			uint8_t pdata[4];
		
			can_cmd[3] = 0x00;
			can_cmd[2] = 0x99;//���Ծ���λ��ת��ָ��
			can_cmd[1] = 0x2C;
			can_cmd[0] = 0x08;//canָ����λ����
			
			pluse = (angle/(2*3.1416))*PULSE[(ARM_ID-0X2C)];//PULSEΪһȦ������
			
			motor_math_itoh(pluse,pdata);  //��pulseת��Ϊ16������������int������pdata[4]��
						
			can_cmd[4]=*pdata;
			can_cmd[5]=*(pdata+1);
			can_cmd[6]=*(pdata+2);
			can_cmd[7]=*(pdata+3);
			board.motor_sendmsg(ARM_ID,can_cmd);//������Ӧ����������ָ��
  
			can_timeout();//if have problem, remove the ע��
}
void MOTORCONTROL::can_timeout(void)
{
	 uint32_t i = 0;
			while(Flag2_CAN_Receive==0)//��ʱ�ȴ�����CAN��������
			{
				i++;
				if(i>0x2fff)
				{
					i=0;
					break;
				}	
			}	
}
void MOTORCONTROL::motor_math_itoh(int N, uint8_t *h)
{
		uint8_t a[10]={0};
    int count=0;
    while(N != 0)
    {
       a[count++]=N&0xf;
       N>>=4;
			if(count==8) break;
    }
		
    *h=(a[1]<<4)|a[0];
    *(h+1)=(a[3]<<4)|a[2];
    *(h+2)=(a[5]<<4)|a[4];
    *(h+3)=(a[7]<<4)|a[6];
}
void MOTORCONTROL::motor_math_htoi(uint8_t *h, int *a)
{
    *a=((*h)<<24|(*(h+1)<<16)|(*(h+2)<<8)|*(h+3));
}
void MOTORCONTROL::arm_control(void)
{
	uint8_t num;
	float action[4];
	*(action) = my_robot.expect_arm_state.servo1;
	*(action+1) = -3*my_robot.expect_arm_state.servo2;
	*(action+2) = my_robot.expect_arm_state.servo3;
	*(action+3) = my_robot.expect_arm_state.servo4;
	for(num=0;num<4;num++)
	{
		arm_angle(num+0X2C,*(action+num));
	}
}
void MOTORCONTROL::Kalman_filtering(uint8_t pre_byte[2],uint8_t byte[2],char flag)
{
	short speed;
	if(flag==0)
	{
		speed=-((byte[0]<<8)|byte[1]);
		pre_speed= -((pre_byte[0]<<8)|pre_byte[1]);
	}
	else
	{
		speed=((byte[0]<<8)|byte[1]);
	    pre_speed= ((pre_byte[0]<<8)|pre_byte[1]);
	}
	Kalman_P=sqrt(Kalman_Q*Kalman_Q+Kalman_P+Kalman_P);
    Kalman_Kg=sqrt(Kalman_P*Kalman_P/(Kalman_P*Kalman_P+Kalman_R*Kalman_R));
	Kalman_P=sqrt((1-Kalman_Kg)*Kalman_P);
	speed=pre_speed+Kalman_Kg*(speed-pre_speed);
	if(flag==0)
	{
		speed=-speed;
	}
	byte[0]=(speed>>8)&0xff;
	byte[1]=speed&0xff;
}
void MOTORCONTROL::chassis_read_speed(void)//degree/s
{
	speed[0] = -((byte[0]<<8)|byte[1]);
	speed[1] = (byte[2]<<8)|byte[3];
	
	
}
void MOTORCONTROL::arm_motor_read(unsigned int CAN_ID)
{
			uint8_t can_cmd[8];
			can_cmd[3] = 0x00;
			can_cmd[2] = 0x9B;//��ȡ�Ƕ�ָ��
			can_cmd[1] = CAN_ID;
			can_cmd[0] = 0x08;
			can_cmd[4] = 0x00;
			can_cmd[5] = 0x00;
			can_cmd[6] = 0x00;
			can_cmd[7] = 0x00;
			board.motor_sendmsg(CAN_ID,can_cmd);//������Ӧ����������ָ��
			can_timeout();
	
}
void MOTORCONTROL::arm_read(void)
{
	uint8_t num;
	float pluse;
	int pluse_t;
	float angle[4];
	float cmd[4]={0};
		for(num=0;num<4;num++)
	{
		arm_motor_read(num+0X2C);
		motor_math_htoi((arm_cmd+num*4),&pluse_t);
		pluse=pluse_t;
		angle[num] = (pluse/PULSE[num])*(2*3.1416);
		
	}
	cmd[0] = angle[0];
	cmd[1] = -angle[1]/3;
	cmd[2] = angle[2];
	cmd[3] = angle[3] ;

	
	memcpy(&(my_robot.measure_arm_state),cmd,sizeof(my_robot.measure_arm_state));
}



void MOTORCONTROL::set_position(MSGCoord position_,MSGCoord cur_x){
		first_call_ =1;
		wheel_position=position_;
		des_.x=cur_x.x+position_.x;
		des_.y=cur_x.y+position_.y;
		des_.z=cur_x.z+position_.z;
		my_robot.expect_robot_speed.x=0.1;
		last_d_ = (position_.x)*(position_.x)+(position_.y)*(position_.y);
	
	 	my_robot.expect_robot_speed.z=0.1*(2*wheel_position.y)/(wheel_position.x*wheel_position.x+wheel_position.y*wheel_position.y);
						
	
		}
