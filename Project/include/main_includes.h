/***********************************************************************************************************************
* Copyright (c) Hands Free Team. All rights reserved.
* FileName: main_config.h 
* Contact:  QQ Exchange Group -- 521037187
* Version:  V2.0
*
* LICENSING TERMS:
* The Hands Free is licensed generally under a permissive 3-clause BSD license.
* Contributions are requiredto be made under the same license.
*
* History:
* <author>      <time>      <version>      <desc>
* mawenke      2015.10.1    V1.0          creat this file
*
* Description:   syetem include  file
***********************************************************************************************************************/
#ifndef MAIN_INCLUDES_H
#define MAIN_INCLUDES_H

/*******************************************************processor.h****************************************************/
#include "board.h"
#include "main_config.h"

/******************************************************package.h*******************************************************/


/******************************************************package.h*******************************************************/
/**************以下的定义来自main_config.h************************/
#ifdef PAKG_COMMON
#include "queue.h"
#endif

#ifdef PAKG_ROBOT_ABSTRACT
#include "robot_abstract.h"
#endif

#ifdef PAKG_MATH
#include "base_math_top.h"
#endif

#ifdef PAKG_MOTOR_CONTROL_XM
#include "motor_control_xm.h"
#endif

#ifdef PACK_GYPOS_XM
#include "gypos.h"
#endif

#ifdef PAKG_SERVO
#include "robot_head.h"
#endif

#ifdef PACK_ROBOT_WHEEL
#include "robot_wheel_top_xm.h"
#endif

#ifdef PACK_PLATFORM_XM
#include "platform.h"
#endif

#ifdef PACK_HF_LINK
#include "hf_link.h"
#endif

#ifdef PACK_MPU6050
#include "mpu6050.h"
#endif
#ifdef PACK_HC05
#include "HC_05.h"
#endif

/**********************************************************************************************************************/


/*****************************************************OS.h*************************************************************/

//GUI/EMWIN
#if SYSTEM_SUPPORT_GUI == 1
#include "GUI.h"	 
#include "GUIDEMO.h"
#endif

//RTOS
#if SYSTEM_SUPPORT_OS==1
#include "includes.h"				
#endif

/**********************************************************************************************************************/


/*****************************************************API.h************************************************************/


/**********************************************************************************************************************/
#include "hf_link_port.h"
#endif //#ifndef MAIN_INCLUDES_H

