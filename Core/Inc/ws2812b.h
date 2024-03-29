/*

  WS2812B CPU and memory efficient library

  Date: 28.9.2016

  Author: Martin Hubacek
  	  	  http://www.martinhubacek.cz
  	  	  @hubmartin

  Licence: MIT License

*/

#ifndef INC_WS2812B_H_
#define INC_WS2812B_H_
#include "ws2812b.h"

// LED output port
#define WS2812B_PORT GPIOA
// LED output pins
#define WS2812B_PINS (GPIO_PIN_11)
// How many LEDs are in the series
#define WS2812B_NUMBER_OF_LEDS 144
// Number of paralel LED strips on the SAME gpio. Each has its own buffer.
#define WS2812_BUFFER_COUNT 1

// Choose one of the bit-juggling setpixel implementation
// *******************************************************
//#define SETPIX_1	// For loop, works everywhere, slow
//#define SETPIX_2	// Optimized unrolled loop
#define SETPIX_3	// Unrolled loop plus some pointer increment optimization
//#define SETPIX_4


// DEBUG OUTPUT
// ********************
#define LED4_PORT GPIOA
#define LED4_PIN GPIO_PIN_1

#define LED5_PORT GPIOA
#define LED5_PIN GPIO_PIN_1


// Public functions
// ****************
void ws2812b_init();
void ws2812b_handle();

void dma_transfer_half_handler(DMA_HandleTypeDef *dma_handle);
void dma_transfer_complete_handler(DMA_HandleTypeDef *dma_handle);
// Library structures
// ******************
// This value sets number of periods to generate 50uS Treset signal
#define WS2812_RESET_PERIOD 100

typedef struct ws2812b_buffer_item_t {
	uint8_t* frame_buffer_pointer;
	uint32_t frame_buffer_size;
	uint32_t frame_buffer_counter;
	uint8_t channel;	// digital output pin/channel
} ws2812b_buffer_item_t;



typedef struct ws2812b_t
{
	ws2812b_buffer_item_t item[WS2812_BUFFER_COUNT];
	uint8_t transfer_complete;
	uint8_t start_transfer;
	uint32_t timer_period_counter;
	uint32_t repeat_counter;
} ws2812b_t;

ws2812b_t ws2812b;


#endif /* INC_WS2812B_H_ */
