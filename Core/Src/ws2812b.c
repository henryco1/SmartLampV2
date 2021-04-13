/*
 * ws2812.c
 *
 *  Created on: Apr 13, 2021
 *      Author: henryco1
 */

#include "ws2812b.h"

void LED_Init(void)
{
	  GPIO_InitTypeDef GPIO_InitStruct = {0};

	  /*Configure GPIO pin : LD3_Pin */
	  GPIO_InitStruct.Pin = LD3_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_PULLUP;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
}

void TIM2_Init(void)
{

}

void DMA1_Init(void)
{

}
