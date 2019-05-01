/*
 * wave_generation.c
 *
 *  Created on: 24 avr. 2019
 *      Author: zhonx
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "main.h"

#include "stdio.h"
#include "stdlib.h"
#include "math.h"

#include "cis.h"
#include "wave_generation.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define SAMPLING_FREQUENCY 	(128000)
#define WAVE_AMP_RESOLUTION (65535)
#define START_FREQUENCY     (43.654)  //FA 1
#define MAX_OCTAVE_NUMBER   (10)
#define SEMITONE_PER_OCTAVE (12)
#define COMMA_PER_SEMITONE  (4.5)
#define PIXEL_PER_COMMA     (6)

#define COMMA_PER_OCTAVE    ((SEMITONE_PER_OCTAVE) * (COMMA_PER_SEMITONE))
#define PIXEL_PER_OCTAVE    ((PIXEL_PER_COMMA) * (COMMA_PER_OCTAVE))

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static float calculate_frequency(uint32_t comma_cnt);

/* Private user code ---------------------------------------------------------*/

//	octave_number = ((12 / log(2)) * log(END_FREQUENCY / START_FREQUENCY) / 12);

/**
 * @brief  calculate frequency,
 * @param  comma cnt
 * @retval frequency
 */
static float calculate_frequency(uint32_t comma_cnt)
{
	float frequency = 0.0;
	frequency =  START_FREQUENCY * pow(2, (comma_cnt / (12.0 * (COMMA_PER_OCTAVE / (12.0 / (log(2)) * log((START_FREQUENCY * 2.0) / START_FREQUENCY))))));

	return frequency;
}

/**
 * @brief  build_waves,
 * @param  unitary_waveform pointer,
 * @param  waves structure pointer,
 * @retval buffer length on success, negative value otherwise
 */
uint32_t init_waves(__IO uint16_t **unitary_waveform, __IO struct wave *waves)
{
	uint32_t buffer_len = 0;
	uint32_t current_unitary_waveform_cell = 0;

	//compute cell number for storage all oscillators waveform
	for (uint32_t comma_cnt = 0; comma_cnt < COMMA_PER_OCTAVE; comma_cnt++)
	{
		//store only first octave frequencies ---- logarithmic distribution
		float frequency = calculate_frequency(comma_cnt);
		buffer_len += (uint32_t)(SAMPLING_FREQUENCY / frequency);
	}

	//allocate the contiguous memory area for storage all waveforms for the first octave
	*unitary_waveform = malloc(buffer_len * sizeof(uint16_t*));
	if (*unitary_waveform == NULL)
	{
		return -1;
	}

	//compute and store the waveform into unitary_waveform only for the first octave
	for (uint32_t current_comma_first_octave = 0; current_comma_first_octave < COMMA_PER_OCTAVE; current_comma_first_octave++)
	{
		//compute frequency for each comma into the first octave
		float frequency = calculate_frequency(current_comma_first_octave);

		//current aera size is the number of char cell for storage a waveform at the current frequency (one pixel per frequency oscillator)
		uint32_t current_aera_size = (uint32_t)(SAMPLING_FREQUENCY / frequency);

		//fill unitary_waveform buffer with sinusoidal waveform for each comma
		for (uint32_t x = 0; x < current_aera_size; x++)
		{
			//sanity check
			if (current_unitary_waveform_cell < buffer_len)
			{
				(*unitary_waveform)[current_unitary_waveform_cell] = (sin((x * 2.00 * M_PI) / current_aera_size) + 1.00) * WAVE_AMP_RESOLUTION / 2.00;
				current_unitary_waveform_cell++;
			}
		}

		//for each octave (only the first octave stay in RAM, for multiple octave start_ptr stay on first octave waveform but current_ptr jump cell according to multiple frequencies)
		for (uint32_t octave = 0; octave < MAX_OCTAVE_NUMBER; octave++)
		{
			//duplicate for each pixel with same frequency value, different waves[pix] for the same start_ptr
			for (uint32_t idx = 0; idx < PIXEL_PER_COMMA; idx++)
			{
				//compute the current pixel for associate an waveform pointer,
				// *** is current pix, --- octave separation
				// ***---------***---------***---------***---------***---------***---------***---------***--------- for current comma at each octave *** represent 3 duplication (according to PIXEL_PER_COMMA
				// ---***---------***---------***---------***---------***---------***---------***---------***------ for the second comma...
				// ------***---------***---------***---------***---------***---------***---------***---------***---
				// ---------***---------***---------***---------***---------***---------***---------***---------***
				uint32_t pix = current_comma_first_octave * PIXEL_PER_COMMA + PIXEL_PER_OCTAVE * octave + idx;
				//sanity check, if user demand is't possible
				if (pix < CIS_PIXELS_NB)
				{
					//store frequencies
					waves[pix].frequency = frequency * pow(2, octave);
					//store octave number
					waves[pix].octave = octave;
					//store aera size
					waves[pix].aera_size = current_aera_size / pow(2, octave);
					//store pointer address
					waves[pix].start_ptr = &(*unitary_waveform)[current_unitary_waveform_cell - current_aera_size];
					//set current pointer at the same address
					waves[pix].current_ptr = &(*unitary_waveform)[current_unitary_waveform_cell - current_aera_size];
				}
			}
		}
	}

	//print all buffer for debug (you can see the waveform with serial tracer on arduino ide)
//	for (uint32_t i = 0; i < buffer_len; i++)
//	{
//		printf("%d\n", (*unitary_waveform)[i]);
//		HAL_Delay(1);
//	}

	//print all structure table for debug

	for (uint32_t pix = 0; pix < CIS_PIXELS_NB; pix++)
	{
//		printf("FREQ = %0.2f, SIZE = %d, OCTAVE = %d\n", waves[pix].frequency, (int)waves[pix].aera_size, (int)waves[pix].octave);
//		HAL_Delay(20);
		uint16_t output = 0;
		for (uint32_t idx = 0; idx < waves[pix].aera_size; idx++)
		{
			output = *(waves[pix].start_ptr + (idx * (uint32_t)pow(2, waves[pix].octave)));
			printf("%d\n", output);
		}
	}

	return buffer_len;
}
