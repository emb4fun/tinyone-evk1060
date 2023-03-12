/**************************************************************************
*  This file is part of the TAL project (Tiny Abstraction Layer)
*
*  Copyright (c) 2020-2022 by Michael Fischer (www.emb4fun.de).
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without 
*  modification, are permitted provided that the following conditions 
*  are met:
*  
*  1. Redistributions of source code must retain the above copyright 
*     notice, this list of conditions and the following disclaimer.
*
*  2. Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in the 
*     documentation and/or other materials provided with the distribution.
*
*  3. Neither the name of the author nor the names of its contributors may 
*     be used to endorse or promote products derived from this software 
*     without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL 
*  THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS 
*  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED 
*  AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF 
*  THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF 
*  SUCH DAMAGE.
**************************************************************************/
#if defined(USE_BOARD_RT1060EVK)
#define __TALLED_C__

/*=======================================================================*/
/*  Include                                                              */
/*=======================================================================*/
#include <stdint.h>
#include <string.h>
#include "tal.h"

#include "fsl_iomuxc.h"
#include "fsl_gpio.h"

/*=======================================================================*/
/*  All Structures and Common Constants                                  */
/*=======================================================================*/

/*
 * LED status
 */
#define LED_OFF   0
#define LED_ON    1

/*=======================================================================*/
/*  Definition of all local Data                                         */
/*=======================================================================*/

static uint8_t    LedStatus[TAL_LED_CHANNEL_MAX];

/*=======================================================================*/
/*  Definition of all local Procedures                                   */
/*=======================================================================*/

/*=======================================================================*/
/*  All code exported                                                    */
/*=======================================================================*/

/*************************************************************************/
/*  tal_LEDInit                                                          */
/*                                                                       */
/*  Initialize the LEDs of the STM3240G-EVAL board.                      */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void tal_LEDInit (void)
{
   /* Clear status information */
   memset(LedStatus, LED_OFF, sizeof(LedStatus));

#if !defined(TAL_ENABLE_ETH)
   /* Define the init structure for the output LED pin*/
   gpio_pin_config_t led_config = {kGPIO_DigitalOutput, 1, kGPIO_NoIntmode};

   /* iomuxc clock (iomuxc_clk_enable): 0x03u */
   CLOCK_EnableClock(kCLOCK_Iomuxc);

   /*
    * GPIO_AD_B0_08 is configured as GPIO1_IO09
    * Software Input On Field: Input Path is determined by functionality
    */
   IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_08_GPIO1_IO08, 0U);

   /*
    * GPIO_AD_B0_08 PAD functional properties :
    * Slew Rate Field: Slow Slew Rate
    * Drive Strength Field: R0/6
    * Speed Field: medium(100MHz)
    * Open Drain Enable Field: Open Drain Disabled
    * Pull / Keep Enable Field: Pull/Keeper Enabled
    * Pull / Keep Select Field: Keeper
    * Pull Up / Down Config. Field: 100K Ohm Pull Down
    * Hyst. Enable Field: Hysteresis Disabled
    */
   IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_08_GPIO1_IO08, 0x10B0u);

   /* Switch off all LEDs first to prevent glitches */   
   tal_LEDClear(TAL_LED_CHANNEL_1); 

   /* Init output LED GPIO. */
   GPIO_PinInit(GPIO1, 8, &led_config);
#endif
} /* tal_LEDInit */

/*************************************************************************/
/*  tal_LEDSet                                                           */
/*                                                                       */
/*  Set the given LED.                                                   */
/*                                                                       */
/*  In    : eChannel                                                     */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void tal_LEDSet (TAL_LED_CHANNEL eChannel)
{
#if !defined(TAL_ENABLE_ETH)
   /* Check for valid range */
   if ((uint8_t)eChannel < (uint8_t)TAL_LED_CHANNEL_MAX)
   {
      switch (eChannel)
      {
         case TAL_LED_CHANNEL_1: GPIO1->DR &= ~(1U << 8); break;
         
         default:
         {
            /* Do nothing */
            break;
         }
      }
      
      LedStatus[eChannel] = LED_ON;
   }
#else
   (void)eChannel;
#endif   
} /* hal_LEDSet */

/*************************************************************************/
/*  tal_LEDClear                                                         */
/*                                                                       */
/*  Clear the given LED.                                                 */
/*                                                                       */
/*  In    : eChannel                                                     */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void tal_LEDClear (TAL_LED_CHANNEL eChannel)
{
#if !defined(TAL_ENABLE_ETH)
   /* Check for valid range */
   if ((uint8_t)eChannel < (uint8_t)TAL_LED_CHANNEL_MAX)
   {
      switch (eChannel)
      {
         case TAL_LED_CHANNEL_1: GPIO1->DR |=  (1U << 8); break;

         default:
         {
            /* Do nothing */
            break;
         }
      }
      
      LedStatus[eChannel] = LED_OFF;
   }
#else
   (void)eChannel;
#endif   
} /* tal_LEDClear */

/*************************************************************************/
/*  tal_LEDToggle                                                        */
/*                                                                       */
/*  Toggle the given LED.                                                */
/*                                                                       */
/*  In    : eChannel                                                     */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void tal_LEDToggle (TAL_LED_CHANNEL eChannel)
{
#if !defined(TAL_ENABLE_ETH)
   /* Check for valid range */
   if ((uint8_t)eChannel < (uint8_t)TAL_LED_CHANNEL_MAX)
   {
      if (LED_ON == LedStatus[eChannel])
      { 
         tal_LEDClear(eChannel); 
      }
      else
      {
         tal_LEDSet(eChannel); 
      }         
   }
#else
   (void)eChannel;
#endif   
} /* tal_LEDClear */

#endif /* USE_BOARD_RT1060EVK */

/*** EOF ***/
