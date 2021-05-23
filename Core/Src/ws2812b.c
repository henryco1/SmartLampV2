/*

  WS2812B CPU and memory efficient library

  Date: 28.9.2016

  Author: Martin Hubacek
  	  	  http://www.martinhubacek.cz
  	  	  @hubmartin

  Licence: MIT License

*/

#include <string.h>

#include "stm32L4xx_hal.h"
#include "ws2812b.h"

//extern ws2812_t ws2812b;

// WS2812 framebuffer - buffer for 2 LEDs - two times 24 bits
uint8_t dma_bit_buffer[24 * 2];
#define BUFFER_SIZE		(sizeof(dma_bit_buffer)/sizeof(uint8_t))

//TIM_HandleTypeDef  timer2_handle;
//TIM_OC_InitTypeDef timer2_oc1;
//TIM_OC_InitTypeDef timer2_oc2;

extern uint32_t timer_period;
extern uint8_t compare_pulse_logic_0;
extern uint8_t compare_pulse_logic_1;
extern DMA_HandleTypeDef hdma_tim2_ch3;
extern TIM_HandleTypeDef htim2;

void dma_transfer_complete_handler(DMA_HandleTypeDef *hdma_tim2_ch3);
void dma_transfer_half_handler(DMA_HandleTypeDef *hdma_tim2_ch3);
static void ws2812b_set_pixel(uint8_t row, uint16_t column, uint8_t red, uint8_t green, uint8_t blue);


// Gamma correction table
const uint8_t gammaTable[] = {
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,
    1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,
    2,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  4,  5,  5,  5,
    5,  6,  6,  6,  6,  7,  7,  7,  7,  8,  8,  8,  9,  9,  9, 10,
   10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,
   17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
   25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,
   37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,
   51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,
   69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
   90, 92, 93, 95, 96, 98, 99,101,102,104,105,107,109,110,112,114,
  115,117,119,120,122,124,126,127,129,131,133,135,137,138,140,142,
  144,146,148,150,152,154,156,158,160,162,164,167,169,171,173,175,
  177,180,182,184,186,189,191,193,196,198,200,203,205,208,210,213,
  215,218,220,223,225,228,231,233,236,239,241,244,247,249,252,255
};

static void load_next_framebuffer_data(ws2812b_buffer_item_t *buffer_item, uint32_t row)
{

	uint32_t r = buffer_item->frame_buffer_pointer[buffer_item->frame_buffer_counter++];
	uint32_t g = buffer_item->frame_buffer_pointer[buffer_item->frame_buffer_counter++];
	uint32_t b = buffer_item->frame_buffer_pointer[buffer_item->frame_buffer_counter++];

	if(buffer_item->frame_buffer_counter == buffer_item->frame_buffer_size)
		buffer_item->frame_buffer_counter = 0;

	ws2812b_set_pixel(buffer_item->channel, row, r, g, b);
}


// Transmit the framebuffer
static void ws2812b_send()
{
	// transmission complete flag
	ws2812b.transfer_complete = 0;

	uint32_t i;

	for( i = 0; i < WS2812_BUFFER_COUNT; i++ )
	{
		ws2812b.item[i].frame_buffer_counter = 0;

		load_next_framebuffer_data(&ws2812b.item[i], 0); // ROW 0
		load_next_framebuffer_data(&ws2812b.item[i], 1); // ROW 0
	}

	HAL_TIM_Base_Stop(&htim2);
	(&htim2)->Instance->CR1 &= ~((0x1U << (0U)));


	// clear all DMA flags
	// modify clear flag to clear channel 1?
	__HAL_DMA_CLEAR_FLAG(&hdma_tim2_ch3, DMA_FLAG_TC1 | DMA_FLAG_HT1 | DMA_FLAG_TE1);

	// configure the number of bytes to be transferred by the DMA controller
	hdma_tim2_ch3.Instance->CNDTR = BUFFER_SIZE;

	// clear all TIM2 flags
	__HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE | TIM_FLAG_CC1 | TIM_FLAG_CC2 | TIM_FLAG_CC3 | TIM_FLAG_CC4);

	// enable DMA channels
	__HAL_DMA_ENABLE(&hdma_tim2_ch3);

	// IMPORTANT: enable the TIM2 DMA requests AFTER enabling the DMA channels!
	__HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_UPDATE);

	// debugger says that this operation is 100-1 = 99
	TIM2->CNT = timer_period;

	// Set zero length for first pulse because the first bit loads after first TIM_UP
	TIM2->CCR3 = 0;

	// Enable PWM
	//(&timer2_handle)->Instance->CCMR1 &= ~((1 << 4) | (1 << 5));
	(&htim2)->Instance->CCMR2 |= (1 << 5);

	__HAL_DBGMCU_FREEZE_TIM2();

	// start TIM2
	__HAL_TIM_ENABLE(&htim2);

}





void dma_transfer_half_handler(DMA_HandleTypeDef *dma_handle)
{
	#if defined(LED4_PORT)
		LED4_PORT->BSRR = LED4_PIN;
	#endif

	// Is this the last LED?
	if(ws2812b.repeat_counter != (WS2812B_NUMBER_OF_LEDS / 2 - 1))
	{
		uint32_t i;

		for( i = 0; i < WS2812_BUFFER_COUNT; i++ )
		{
			load_next_framebuffer_data(&ws2812b.item[i], 0);
		}

	} else {

		//HAL_TIM_PWM_ConfigChannel()


		// If this is the last pixel, set the next pixel value to zeros, because
		// the DMA would not stop exactly at the last bit.
		ws2812b_set_pixel(0, 0, 0, 0, 0);
	}

	#if defined(LED4_PORT)
		LED4_PORT->BRR = LED4_PIN;
	#endif
}

void dma_transfer_complete_handler(DMA_HandleTypeDef *dma_handle)
{
	#if defined(LED5_PORT)
		LED5_PORT->BSRR = LED5_PIN;
	#endif

	ws2812b.repeat_counter++;

	if(ws2812b.repeat_counter == WS2812B_NUMBER_OF_LEDS / 2)
	{
		// Transfer of all LEDs is done, disable DMA but enable tiemr update IRQ to stop the 50us pulse
		ws2812b.repeat_counter = 0;

		// Disable PWM output
		// change CCMR1 to CCMR2
		(&htim2)->Instance->CCMR2 &= ~((1 << 4) | (1 << 5));
		(&htim2)->Instance->CCMR2 |= (1 << 6);



		// Enable TIM2 Update interrupt for 50us Treset signal
		__HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
		// Disable DMA
		__HAL_DMA_DISABLE(&hdma_tim2_ch3);
		// Disable the DMA requests
		__HAL_TIM_DISABLE_DMA(&htim2, TIM_DMA_UPDATE);

	} else {

		// Load bitbuffer with next RGB LED values
		uint32_t i;
		for( i = 0; i < WS2812_BUFFER_COUNT; i++ )
		{
			load_next_framebuffer_data(&ws2812b.item[i], 1);
		}

	}

	#if defined(LED5_PORT)
		LED5_PORT->BRR = LED5_PIN;
	#endif
}

// TIM2 Interrupt Handler gets executed on every TIM2 Update if enabled
/*
 * two options here: HAL_TIM_PeriodElapsedCallback and HAL_TIM_PWM_PulseFinishedCallback
 *
 * If I recall correctly: HAL_TIM_PeriodElapsedCallback is called when the timer reaches the value stored
 * in the 'reload register' (This is when the timer 'updates' and determines the frequency of your PWM
 * signal) As TDK has said above HAL_TIM_PWM_PulseFinishedCallback is called when the timer reaches the
 * pulse value stored in the C&C register. This is responsible for your duty cycle. Don't foget that if
 * you want HAL_TIM_PeriodElapsedCallback, you must call HAL_TIM_Base_Start_IT and if you want
 * HAL_TIM_PWM_PulseFinishedCallback, you must call HAL_TIM_PWM_Start_IT. To have both callbacks you must
 * call both 'start' functions.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

#if defined(LED5_PORT)
		LED5_PORT->BSRR = LED5_PIN;
	#endif

	// I have to wait 50us to generate Treset signal
	if (ws2812b.timer_period_counter < (uint8_t)WS2812_RESET_PERIOD)
	{
		// count the number of timer periods
		ws2812b.timer_period_counter++;
	}
	else
	{
		/*
		 * seems like here we want to handle pulse finished call back
		 * hubmartin's code uses timer oc, but here we want to use pwm
		 */
		ws2812b.timer_period_counter = 0;
		__HAL_TIM_DISABLE(&htim2);
		TIM2->CR1 = 0; // disable timer

		// disable the TIM2 Update
		__HAL_TIM_DISABLE_IT(&htim2, TIM_IT_UPDATE);
		// set transfer_complete flag
		ws2812b.transfer_complete = 1;
	}

#if defined(LED5_PORT)
		LED5_PORT->BRR = LED5_PIN;
	#endif

}



static void ws2812b_set_pixel(uint8_t row, uint16_t column, uint8_t red, uint8_t green, uint8_t blue)
{

	// Apply gamma
	red = gammaTable[red];
	green = gammaTable[green];
	blue = gammaTable[blue];

	uint32_t calculated_column = (column * 24);

#if defined(SETPIX_1)

	uint8_t i;
	for (i = 0; i < 8; i++)
	{
		// write new data for pixel
		dma_bit_buffer[(calculated_column+i)] = (((((green)<<i) & 0x80)>>7)<<row) ? compare_pulse_logic_1 : compare_pulse_logic_0;
		dma_bit_buffer[(calculated_column+8+i)] = (((((red)<<i) & 0x80)>>7)<<row) ? compare_pulse_logic_1 : compare_pulse_logic_0;
		dma_bit_buffer[(calculated_column+16+i)] = (((((blue)<<i) & 0x80)>>7)<<row) ? compare_pulse_logic_1 : compare_pulse_logic_0;
	}

#elif defined(SETPIX_2)



		// write new data for pixel
		dma_bit_buffer[(calculated_column+0)] = (((green)<<0) & 0x80) ? compare_pulse_logic_1 : compare_pulse_logic_0;
		dma_bit_buffer[(calculated_column+8+0)] = (((red)<<0) & 0x80) ? compare_pulse_logic_1 : compare_pulse_logic_0;
		dma_bit_buffer[(calculated_column+16+0)] = (((blue)<<0) & 0x80) ? compare_pulse_logic_1 : compare_pulse_logic_0;

		// write new data for pixel
		dma_bit_buffer[(calculated_column+1)] = (((green)<<1) & 0x80) ? compare_pulse_logic_1 : compare_pulse_logic_0;
		dma_bit_buffer[(calculated_column+8+1)] = (((red)<<1) & 0x80) ? compare_pulse_logic_1 : compare_pulse_logic_0;
		dma_bit_buffer[(calculated_column+16+1)] = (((blue)<<1) & 0x80) ? compare_pulse_logic_1 : compare_pulse_logic_0;

		// write new data for pixel
		dma_bit_buffer[(calculated_column+2)] = (((green)<<2) & 0x80) ? compare_pulse_logic_1 : compare_pulse_logic_0;
		dma_bit_buffer[(calculated_column+8+2)] = (((red)<<2) & 0x80) ? compare_pulse_logic_1 : compare_pulse_logic_0;
		dma_bit_buffer[(calculated_column+16+2)] = (((blue)<<2) & 0x80) ? compare_pulse_logic_1 : compare_pulse_logic_0;

		// write new data for pixel
		dma_bit_buffer[(calculated_column+3)] = (((green)<<3) & 0x80) ? compare_pulse_logic_1 : compare_pulse_logic_0;
		dma_bit_buffer[(calculated_column+8+3)] = (((red)<<3) & 0x80) ? compare_pulse_logic_1 : compare_pulse_logic_0;
		dma_bit_buffer[(calculated_column+16+3)] = (((blue)<<3) & 0x80) ? compare_pulse_logic_1 : compare_pulse_logic_0;

		// write new data for pixel
		dma_bit_buffer[(calculated_column+4)] = (((green)<<4) & 0x80) ? compare_pulse_logic_1 : compare_pulse_logic_0;
		dma_bit_buffer[(calculated_column+8+4)] = (((red)<<4) & 0x80) ? compare_pulse_logic_1 : compare_pulse_logic_0;
		dma_bit_buffer[(calculated_column+16+4)] = (((blue)<<4) & 0x80) ? compare_pulse_logic_1 : compare_pulse_logic_0;

		// write new data for pixel
		dma_bit_buffer[(calculated_column+5)] = (((green)<<5) & 0x80) ? compare_pulse_logic_1 : compare_pulse_logic_0;
		dma_bit_buffer[(calculated_column+8+5)] = (((red)<<5) & 0x80) ? compare_pulse_logic_1 : compare_pulse_logic_0;
		dma_bit_buffer[(calculated_column+16+5)] = (((blue)<<5) & 0x80) ? compare_pulse_logic_1 : compare_pulse_logic_0;

		// write new data for pixel
		dma_bit_buffer[(calculated_column+6)] = (((green)<<6) & 0x80) ? compare_pulse_logic_1 : compare_pulse_logic_0;
		dma_bit_buffer[(calculated_column+8+6)] = (((red)<<6) & 0x80) ? compare_pulse_logic_1 : compare_pulse_logic_0;
		dma_bit_buffer[(calculated_column+16+6)] = (((blue)<<6) & 0x80) ? compare_pulse_logic_1 : compare_pulse_logic_0;

		// write new data for pixel
		dma_bit_buffer[(calculated_column+7)] = (((green)<<7) & 0x80) ? compare_pulse_logic_1 : compare_pulse_logic_0;
		dma_bit_buffer[(calculated_column+8+7)] = (((red)<<7) & 0x80) ? compare_pulse_logic_1 : compare_pulse_logic_0;
		dma_bit_buffer[(calculated_column+16+7)] = (((blue)<<7) & 0x80) ? compare_pulse_logic_1 : compare_pulse_logic_0;

#elif defined(SETPIX_3)

		uint8_t *bit_buffer_offset = &dma_bit_buffer[calculated_column];

		// write new data for pixel
		*bit_buffer_offset++ = (green & 0x80) ? compare_pulse_logic_1 : compare_pulse_logic_0;
		*bit_buffer_offset++ = (green & 0x40) ? compare_pulse_logic_1 : compare_pulse_logic_0;
		*bit_buffer_offset++ = (green & 0x20) ? compare_pulse_logic_1 : compare_pulse_logic_0;
		*bit_buffer_offset++ = (green & 0x10) ? compare_pulse_logic_1 : compare_pulse_logic_0;
		*bit_buffer_offset++ = (green & 0x08) ? compare_pulse_logic_1 : compare_pulse_logic_0;
		*bit_buffer_offset++ = (green & 0x04) ? compare_pulse_logic_1 : compare_pulse_logic_0;
		*bit_buffer_offset++ = (green & 0x02) ? compare_pulse_logic_1 : compare_pulse_logic_0;
		*bit_buffer_offset++ = (green & 0x01) ? compare_pulse_logic_1 : compare_pulse_logic_0;

		*bit_buffer_offset++ = (red & 0x80) ? compare_pulse_logic_1 : compare_pulse_logic_0;
		*bit_buffer_offset++ = (red & 0x40) ? compare_pulse_logic_1 : compare_pulse_logic_0;
		*bit_buffer_offset++ = (red & 0x20) ? compare_pulse_logic_1 : compare_pulse_logic_0;
		*bit_buffer_offset++ = (red & 0x10) ? compare_pulse_logic_1 : compare_pulse_logic_0;
		*bit_buffer_offset++ = (red & 0x08) ? compare_pulse_logic_1 : compare_pulse_logic_0;
		*bit_buffer_offset++ = (red & 0x04) ? compare_pulse_logic_1 : compare_pulse_logic_0;
		*bit_buffer_offset++ = (red & 0x02) ? compare_pulse_logic_1 : compare_pulse_logic_0;
		*bit_buffer_offset++ = (red & 0x01) ? compare_pulse_logic_1 : compare_pulse_logic_0;

		*bit_buffer_offset++ = (blue & 0x80) ? compare_pulse_logic_1 : compare_pulse_logic_0;
		*bit_buffer_offset++ = (blue & 0x40) ? compare_pulse_logic_1 : compare_pulse_logic_0;
		*bit_buffer_offset++ = (blue & 0x20) ? compare_pulse_logic_1 : compare_pulse_logic_0;
		*bit_buffer_offset++ = (blue & 0x10) ? compare_pulse_logic_1 : compare_pulse_logic_0;
		*bit_buffer_offset++ = (blue & 0x08) ? compare_pulse_logic_1 : compare_pulse_logic_0;
		*bit_buffer_offset++ = (blue & 0x04) ? compare_pulse_logic_1 : compare_pulse_logic_0;
		*bit_buffer_offset++ = (blue & 0x02) ? compare_pulse_logic_1 : compare_pulse_logic_0;
		*bit_buffer_offset++ = (blue & 0x01) ? compare_pulse_logic_1 : compare_pulse_logic_0;

#endif
}


void ws2812b_init()
{
//	ws2812b_gpio_init();
//	dma_init();
//	tim2_init();
	  HAL_DMA_Start_IT(&hdma_tim2_ch3, (uint32_t)dma_bit_buffer, (uint32_t)&TIM2->CCR3, BUFFER_SIZE);
//	HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_3, (uint32_t *)dma_bit_buffer, BUFFER_SIZE);
	// Need to start the first transfer
	ws2812b.transfer_complete = 1;
}


void ws2812b_handle()
{
	if(ws2812b.start_transfer) {
		ws2812b.start_transfer = 0;
		ws2812b_send();
	}

}
