/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * Copyright (c) 2020 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define PC14_OSC32_IN_Pin GPIO_PIN_14
#define PC14_OSC32_IN_GPIO_Port GPIOC
#define PC15_OSC32_OUT_Pin GPIO_PIN_15
#define PC15_OSC32_OUT_GPIO_Port GPIOC
#define PH0_OSC_IN_Pin GPIO_PIN_0
#define PH0_OSC_IN_GPIO_Port GPIOH
#define PH1_OSC_OUT_Pin GPIO_PIN_1
#define PH1_OSC_OUT_GPIO_Port GPIOH
#define B1_Pin GPIO_PIN_0
#define B1_GPIO_Port GPIOA
#define LD4_Pin GPIO_PIN_12
#define LD4_GPIO_Port GPIOD
#define LD3_Pin GPIO_PIN_13
#define LD3_GPIO_Port GPIOD
#define LD5_Pin GPIO_PIN_14
#define LD5_GPIO_Port GPIOD
#define LD6_Pin GPIO_PIN_15
#define LD6_GPIO_Port GPIOD
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define ADR0_Port   GPIOC
#define ADR0_Pin    GPIO_PIN_1
#define ADR1_Port   GPIOC
#define ADR1_Pin    GPIO_PIN_2
#define ADR2_Port   GPIOC
#define ADR2_Pin    GPIO_PIN_3
#define ADR3_Port   GPIOA
#define ADR3_Pin    GPIO_PIN_4
#define ADR4_Port   GPIOA
#define ADR4_Pin    GPIO_PIN_5
#define ADR5_Port   GPIOD
#define ADR5_Pin    GPIO_PIN_15
#define ADR6_Port   GPIOA
#define ADR6_Pin    GPIO_PIN_7
#define ADR7_Port   GPIOC
#define ADR7_Pin    GPIO_PIN_4
#define ADR8_Port   GPIOC
#define ADR8_Pin    GPIO_PIN_5
#define ADR9_Port   GPIOB
#define ADR9_Pin    GPIO_PIN_0
#define ADR10_Port   GPIOB
#define ADR10_Pin    GPIO_PIN_1
#define ADR11_Port   GPIOB
#define ADR11_Pin    GPIO_PIN_13
#define ADR12_Port   GPIOB
#define ADR12_Pin    GPIO_PIN_14
#define ADR13_Port   GPIOB
#define ADR13_Pin    GPIO_PIN_15
#define ADR14_Port   GPIOD
#define ADR14_Pin    GPIO_PIN_8
#define ADR15_Port   GPIOD
#define ADR15_Pin    GPIO_PIN_9
#define ADR16_Port   GPIOD
#define ADR16_Pin    GPIO_PIN_10
#define ADR17_Port   GPIOD
#define ADR17_Pin    GPIO_PIN_11
#define ADR18_Port   GPIOD
#define ADR18_Pin    GPIO_PIN_12
#define ADR19_Port   GPIOD
#define ADR19_Pin    GPIO_PIN_13
#define ADR20_Port   GPIOD
#define ADR20_Pin    GPIO_PIN_14

#define DAT0_Port   GPIOB
#define DAT0_Pin    GPIO_PIN_2
#define DAT1_Port   GPIOE
#define DAT1_Pin    GPIO_PIN_7
#define DAT2_Port   GPIOE
#define DAT2_Pin    GPIO_PIN_8
#define DAT3_Port   GPIOE
#define DAT3_Pin    GPIO_PIN_9
#define DAT4_Port   GPIOE
#define DAT4_Pin    GPIO_PIN_10
#define DAT5_Port   GPIOE
#define DAT5_Pin    GPIO_PIN_11
#define DAT6_Port   GPIOE
#define DAT6_Pin    GPIO_PIN_12
#define DAT7_Port   GPIOE
#define DAT7_Pin    GPIO_PIN_13
#define DAT8_Port   GPIOE
#define DAT8_Pin    GPIO_PIN_14
#define DAT9_Port   GPIOE
#define DAT9_Pin    GPIO_PIN_15
#define DAT10_Port   GPIOB
#define DAT10_Pin    GPIO_PIN_10
#define DAT11_Port   GPIOC
#define DAT11_Pin    GPIO_PIN_6
#define DAT12_Port   GPIOC
#define DAT12_Pin    GPIO_PIN_7
#define DAT13_Port   GPIOC
#define DAT13_Pin    GPIO_PIN_8
#define DAT14_Port   GPIOC
#define DAT14_Pin    GPIO_PIN_9
#define DAT15_Port   GPIOA
#define DAT15_Pin    GPIO_PIN_8

#define CS1n_Port   GPIOB
#define CS1n_Pin    GPIO_PIN_13
#define CS2n_Port   GPIOB
#define CS2n_Pin    GPIO_PIN_15

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
