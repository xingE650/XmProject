#ifndef PLATFORM_H
#define PLATFORM_H

#ifdef __cplusplus
extern "C" {
#endif
#include "robot_abstract.h"	
#include "board.h"	
#include "math.h"
	
class PLATFORM
{
	public:
		PLATFORM(){
			flag_dir 			=0xff;
			current_h			=0; 
			Pulse_Num 		=0 ;
			Desp_Num  		=0;
			Pulse_counter	=0;
			per_pulse			=1600;
			per_s					=0.505f;
			dest_height   =0;
			plat_state=0;
		}
		void platform_top_call(void);
		void set_per_pulse(uint16_t Pulse_counter_){Pulse_counter = Pulse_counter_;}
		void set_per_s(float per_s_){per_s = per_s_;}
		void set_height(float height_){plat_state=1;dest_height= height_*100;}
		uint8_t flag_dir ;///升降台升降方向标志位，便于记录总的脉冲数，0xff：升降台向上标志，0x01：升降台向下标志位
		float current_h; //当前高度
		uint16_t Pulse_Num;    	//Pulse_Num存储记录的脉冲个数
		uint16_t Desp_Num;     	//Desp_Num存储目标脉冲数
		int Pulse_counter;	//Pulse_counter存储总的脉冲数(用于计算高度）
		uint16_t per_pulse;    //步进电机转一圈的脉冲
		float per_s;   //转动一圈移动距离2.0cm，(导程2.00cm)
		float dest_height;
		
		
	private:
		uint8_t plat_state;
};

extern PLATFORM platform;
#ifdef __cplusplus
}
#endif

#endif
