/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : �ڴ˰���mcuоƬͷ�ļ�
  * @details        : �ڴ˰���mcuоƬͷ�ļ�
  * @author         : Charmander ��Ϊ���ӿƼ� QQ: 228303760
  * @date           : 2020/9/2
  * @version        : V0.1
  * @copyright      : Copyright (C)
  ******************************************************************************
  * @attention
  * NONE
  ******************************************************************************
  * @verbatim
  * �޸���ʷ:
  *      1. ���ڣ�2020/9/2
  *         ���ߣ�Charmander
  *         �޸ģ�
  * @endverbatim
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* �ڴ˰���mcuоƬͷ�ļ� */
#include "stm32f10x.h"

  extern float temperature;
	extern  int heart_rate;
	extern  u8 fall_flag;
	extern  float smoke;
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/


/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
