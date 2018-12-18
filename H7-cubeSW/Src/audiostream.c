/* Includes ------------------------------------------------------------------*/
#include "audiostream.h"
#include "main.h"
#include "leaf.h"
#include "codec.h"
#include "tim.h"

//the audio buffers are put in the D2 RAM area because that is a memory location that the DMA has access to.
int32_t audioOutBuffer[AUDIO_BUFFER_SIZE] __ATTR_RAM_D2;
int32_t audioInBuffer[AUDIO_BUFFER_SIZE] __ATTR_RAM_D2;

void audioFrame(uint16_t buffer_offset);
float audioTickL(float audioIn); 
float audioTickR(float audioIn);
void buttonCheck(void);

HAL_StatusTypeDef transmit_status;
HAL_StatusTypeDef receive_status;

uint16_t* adcVals;


int mode1 = 1;
int mode2 = 1;
int mode3 = 1;
int RGB_mode = 3;

#define NUM_BUTTONS 4
uint8_t buttonValues[NUM_BUTTONS];
uint8_t buttonValuesPrev[NUM_BUTTONS];
uint32_t buttonCounters[NUM_BUTTONS];
uint32_t buttonPressed[NUM_BUTTONS];


float sample = 0.0f;

uint16_t frameCounter = 0;

//audio objects
tRamp adc[8];
tCycle mySine[2];

/**********************************************/

typedef enum BOOL {
	FALSE = 0,
	TRUE
} BOOL;


void audioInit(I2C_HandleTypeDef* hi2c, SAI_HandleTypeDef* hsaiOut, SAI_HandleTypeDef* hsaiIn, RNG_HandleTypeDef* hrand, uint16_t* adc_array)
{
	// Initialize LEAF.

	LEAF_init(SAMPLE_RATE, AUDIO_FRAME_SIZE, &randomNumber);

	adcVals = adc_array; //in audiostream.h, the array of adc values is now called adcVals.

	for (int i = 0; i < 8; i++)
	{
		tRamp_init(&adc[i],7.0f, 1); //set all ramps for knobs to be 7ms ramp time and let the init function know they will be ticked every sample
	}

	tCycle_init(&mySine[0]);
	tCycle_init(&mySine[1]);
	//now to send all the necessary messages to the codec
	AudioCodec_init(hi2c);

	HAL_Delay(100);

	for (int i = 0; i < AUDIO_BUFFER_SIZE; i++)
	{
		audioOutBuffer[i] = 0;
	}

	// set up the I2S driver to send audio data to the codec (and retrieve input as well)
	transmit_status = HAL_SAI_Transmit_DMA(hsaiOut, (uint8_t *)&audioOutBuffer[0], AUDIO_BUFFER_SIZE);
	receive_status = HAL_SAI_Receive_DMA(hsaiIn, (uint8_t *)&audioInBuffer[0], AUDIO_BUFFER_SIZE);
}

void audioFrame(uint16_t buffer_offset)
{
	int i;
	int32_t current_sample = 0;

	frameCounter++;
	if (frameCounter >= 1)
	{
		frameCounter = 0;
		buttonCheck();
	}

	for (i = 0; i < (HALF_BUFFER_SIZE); i++)
	{
		if ((i & 1) == 0)
		{
			current_sample = (int32_t)(audioTickL((float) (audioInBuffer[buffer_offset + i] * INV_TWO_TO_31)) * TWO_TO_31);
		}
		else
		{
			current_sample = (int32_t)(audioTickR((float) (audioInBuffer[buffer_offset + i] * INV_TWO_TO_31)) * TWO_TO_31);
		}

		audioOutBuffer[buffer_offset + i] = current_sample;
	}
}

float audioTickL(float audioIn)
{
	tRamp_setDest(&adc[0], 1.0f - (adcVals[0] * INV_TWO_TO_16));
	tRamp_setDest(&adc[4], 1.0f - (adcVals[4] * INV_TWO_TO_16));
	float newFreq = LEAF_midiToFrequency(tRamp_tick(&adc[0]) * 127.0f) + (audioIn * tRamp_tick(&adc[4]) * 1000.0f);
	tCycle_setFreq(&mySine[0], newFreq);
	sample = tCycle_tick(&mySine[0]);
	return sample * .9f;
}

float audioTickR(float audioIn)
{
	tRamp_setDest(&adc[1], 1.0f - (adcVals[1] * INV_TWO_TO_16));
	tRamp_setDest(&adc[5], 1.0f - (adcVals[5] * INV_TWO_TO_16));
	float newFreq = LEAF_midiToFrequency(tRamp_tick(&adc[1]) * 127.0f) + (audioIn * tRamp_tick(&adc[5]) * 1000.0f);
	tCycle_setFreq(&mySine[1], newFreq);
	sample = tCycle_tick(&mySine[1]);
	return sample * .9f;
}


void buttonCheck(void)
{
	buttonValues[0] = !HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13);
	buttonValues[1] = !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6);
	buttonValues[2] = !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7);
	buttonValues[3] = !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8);

	for (int i = 0; i < 4; i++)
	{
	  if ((buttonValues[i] != buttonValuesPrev[i]) && (buttonCounters[i] < 40))
	  {
		  buttonCounters[i]++;
	  }
	  if ((buttonValues[i] != buttonValuesPrev[i]) && (buttonCounters[i] >= 40))
	  {
		  if (buttonValues[i] == 1)
		  {
			  buttonPressed[i] = 1;
		  }
		  buttonValuesPrev[i] = buttonValues[i];
		  buttonCounters[i] = 0;
	  }
	}

	if (buttonPressed[0] == 1)
	{
		RGB_mode++;
		if (RGB_mode > 3)
		{
			RGB_mode = 0;
		}
		if (RGB_mode == 0)
		{
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 9000); //R   ///the red channel values should be between 8000 and 16000
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0); //G
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0); //B
		}

		else if (RGB_mode == 1)
		{
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 00); //R
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 300); //G   //green channel values between 100 and 500
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0); //B
		}

		else if (RGB_mode == 2)
		{
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 00); //R
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 00); //G
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 200); //B   // blue channel between 0 and 400
		}

		else if (RGB_mode == 3)
		{
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0); //R
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0); //G
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0); //B
		}
		buttonPressed[0] = 0;
	}
	if (buttonPressed[1] == 1)
	{
		//currently the dimming function of the PWM timers overrides the use of the LED pins as digital outputs
		/*
		if (mode1 == 0)
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
			mode1 = 1;
		}
		else
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
			mode1 = 0;
		}
		*/
		buttonPressed[1] = 0;
	}
	if (buttonPressed[2] == 1)
	{
		//currently the dimming function of the PWM timers overrides the use of the LED pins as digital outputs
		/*
		if (mode2 == 0)
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
			mode2 = 1;
		}
		else
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
			mode2 = 0;
		}
		*/
		buttonPressed[2] = 0;
	}

	if (buttonPressed[3] == 1)
	{
		//Timer PWM isn't set up on this LED pin, so we can just set it to on/off.
		if (mode3 == 0)
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
			mode3 = 1;
		}
		else
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
			mode3 = 0;
		}
		buttonPressed[3] = 0;
	}
}

void HAL_SAI_ErrorCallback(SAI_HandleTypeDef *hsai)
{
	;
}

void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai)
{
	;
}

void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
  ;
}


void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai)
{
	audioFrame(HALF_BUFFER_SIZE);
}

void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
	audioFrame(0);
}
