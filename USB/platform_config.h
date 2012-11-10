/**
  ******************************************************************************
  * @file    platform_config.h
  * @author  MCD Application Team
  * @version V3.4.0
  * @date    29-June-2012
  * @brief   Evaluation board specific configuration file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PLATFORM_CONFIG_H
#define __PLATFORM_CONFIG_H

/* Includes ------------------------------------------------------------------*/

#if defined(STM32L1XX_MD) || defined(STM32L1XX_HD)|| defined(STM32L1XX_MD_PLUS)
 #include "stm32l1xx.h"
 #if defined (USE_STM32L152_EVAL)
  #include "stm32l152_eval.h"
 #elif defined (USE_STM32L152D_EVAL)
  #include "stm32l152d_eval.h"
 #else
  #error "Missing define: USE_STM32L152_EVAL or USE_STM32L152D_EVAL"
 #endif /* USE_STM32L152_EVAL */
#elif defined (STM32F10X_MD) || defined (STM32F10X_HD) || defined (STM32F10X_XL) || defined (STM32F10X_CL)
 #include "stm32f10x.h"
 #if defined (USE_STM3210B_EVAL)
  #include "stm3210b_eval.h"
 #elif defined (USE_STM3210E_EVAL)
  #include "stm3210e_eval.h"
 #elif defined (USE_STM3210C_EVAL)
  #include "stm3210c_eval.h"
  #include "stm3210c_eval_ioe.h"
 #else
  //#error "Missing define: USE_STM3210B_EVAL, USE_STM3210E_EVAL or USE_STM3210C_EVAL"
 #endif /* USE_STM3210B_EVAL */
#endif

/* Define the STM32F10x hardware depending on the used evaluation board */
#ifdef USE_STM3210B_EVAL
  #define USB_DISCONNECT                      GPIOD  
  #define USB_DISCONNECT_PIN                  GPIO_Pin_9
  #define RCC_APB2Periph_GPIO_DISCONNECT      RCC_APB2Periph_GPIOD
  #define ADC1_DR_Address                     ((uint32_t)0x4001244C)

#elif defined (USE_STM3210E_EVAL)
  #define USB_DISCONNECT                      GPIOB  
  #define USB_DISCONNECT_PIN                  GPIO_Pin_14
  #define RCC_APB2Periph_GPIO_DISCONNECT      RCC_APB2Periph_GPIOB
  #define ADC1_DR_Address                     ((uint32_t)0x4001244C)

#elif defined (USE_STM3210C_EVAL)
  #define USB_DISCONNECT                      0  
  #define USB_DISCONNECT_PIN                  0
  #define RCC_APB2Periph_GPIO_DISCONNECT      0
  #define ADC1_DR_Address                     ((uint32_t)0x4001244C)

#elif defined (USE_STM32L152_EVAL) || defined (USE_STM32L152D_EVAL)
 /* 
   For STM32L15xx devices it is possible to use the internal USB pullup
   controlled by register SYSCFG_PMC (refer to RM0038 reference manual for
   more details).
   It is also possible to use external pullup (and disable the internal pullup)
   by setting the define USB_USE_EXTERNAL_PULLUP in file platform_config.h
   and configuring the right pin to be used for the external pull up configuration.
   To have more details on how to use an external pull up, please refer to 
   STM3210E-EVAL evaluation board manuals.
   */
 /* Uncomment the following define to use an external pull up instead of the 
    integrated STM32L15xx internal pull up. In this case make sure to set up
    correctly the external required hardware and the GPIO defines below.*/
/* #define USB_USE_EXTERNAL_PULLUP */

 #if !defined(USB_USE_EXTERNAL_PULLUP)
  #define STM32L15_USB_CONNECT                SYSCFG_USBPuCmd(ENABLE)
  #define STM32L15_USB_DISCONNECT             SYSCFG_USBPuCmd(DISABLE)
  #define RCC_AHBPeriph_GPIO_DISCONNECT       0

 #elif defined(USB_USE_EXTERNAL_PULLUP)
  /* PA0 is chosen just as illustrating example, you should modify the defines
    below according to your hardware configuration. */ 
  #define USB_DISCONNECT                      GPIOA
  #define USB_DISCONNECT_PIN                  GPIO_Pin_0
  #define RCC_AHBPeriph_GPIO_DISCONNECT       RCC_AHBPeriph_GPIOA
  #define STM32L15_USB_CONNECT                GPIO_ResetBits(USB_DISCONNECT, USB_DISCONNECT_PIN)
  #define STM32L15_USB_DISCONNECT             GPIO_SetBits(USB_DISCONNECT, USB_DISCONNECT_PIN)
 #endif /* USB_USE_EXTERNAL_PULLUP */

  #define ADC1_DR_Address                     ((uint32_t)0x40012458)

#else
  #define USB_DISCONNECT                      GPIOB
  #define USB_DISCONNECT_PIN                  GPIO_Pin_1
  #define RCC_APB2Periph_GPIO_DISCONNECT      RCC_APB2Periph_GPIOB
#endif /* USE_STM3210B_EVAL */

#if defined (USE_STM32L152_EVAL)

  #define RCC_AHBPeriph_GPIO_IOAIN            RCC_AHBPeriph_GPIOB
  #define GPIO_IOAIN                          GPIOB
  #define GPIO_IOAIN_PIN                      GPIO_Pin_12   /* PB.12 */
  #define ADC_AIN_CHANNEL                     ADC_Channel_18

#elif defined (USE_STM32L152D_EVAL)
  
  #define RCC_AHBPeriph_GPIO_IOAIN            RCC_AHBPeriph_GPIOF
  #define GPIO_IOAIN                          GPIOF
  #define GPIO_IOAIN_PIN                      GPIO_Pin_10   /* PF.10 */
  #define ADC_AIN_CHANNEL                     ADC_Channel_31


#else
  #define RCC_APB2Periph_GPIO_IOAIN           RCC_APB2Periph_GPIOC
  #define GPIO_IOAIN                          GPIOC
  #define GPIO_IOAIN_PIN                      GPIO_Pin_4   /* PC.04 */
  #define ADC_AIN_CHANNEL                     ADC_Channel_14

#endif /* USE_STM32L152_EVAL */

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#endif /* __PLATFORM_CONFIG_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

