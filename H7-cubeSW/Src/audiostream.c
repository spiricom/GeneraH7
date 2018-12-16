/* Includes ------------------------------------------------------------------*/
#include "audiostream.h"
#include "main.h"
#include "codec.h"

#define AUDIO_FRAME_SIZE     32
#define HALF_BUFFER_SIZE      AUDIO_FRAME_SIZE * 2 //number of samples per half of the "double-buffer" (twice the audio frame size because there are interleaved samples for both left and right channels)
#define AUDIO_BUFFER_SIZE     AUDIO_FRAME_SIZE * 4 //number of samples in the whole data structure (four times the audio frame size because of stereo and also double-buffering/ping-ponging)

// align is to make sure they are lined up with the data boundaries of the cache 
// at(0x3....) is to put them in the D2 domain of SRAM where the DMA can access them
// (otherwise the TX times out because the DMA can't see the data location) -JS


ALIGN_32BYTES (int16_t audioOutBuffer[AUDIO_BUFFER_SIZE] __ATTR_RAM_D2);
ALIGN_32BYTES (int16_t audioInBuffer[AUDIO_BUFFER_SIZE] __ATTR_RAM_D2);


int16_t inBuffer[HALF_BUFFER_SIZE];
int16_t outBuffer[HALF_BUFFER_SIZE];

uint16_t* adcVals;

uint8_t buttonAPressed = 0;

float sample = 0.0f;

void audioFrame(uint16_t buffer_offset);
float audioTickL(float audioIn); 
float audioTickR(float audioIn);
void buttonCheck(void);

int mode1 = 0;
int mode2 = 0;
int mode3 = 0;

tRamp* adc[8];
tCycle* mySine[2];

HAL_StatusTypeDef transmit_status;
HAL_StatusTypeDef receive_status;

typedef enum BOOL {
	FALSE = 0,
	TRUE
} BOOL;

void audioInit(I2C_HandleTypeDef* hi2c, SAI_HandleTypeDef* hsaiOut, SAI_HandleTypeDef* hsaiIn, RNG_HandleTypeDef* hrand, uint16_t* myADCArray)
{ 
	// Initialize the audio library. OOPS.
	OOPSInit(SAMPLE_RATE, &randomNumber);
	
	//now to send all the necessary messages to the codec
	AudioCodec_init(hi2c);

	HAL_Delay(100);

	adcVals = myADCArray;

	for (int i = 0; i < 8; i++)
	{
		adc[i] = tRampInit(12.0f, 1);
	}

	mySine[0] = tCycleInit();
	mySine[1] = tCycleInit();

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
	// set up the I2S driver to send audio data to the codec (and retrieve input as well)	
	transmit_status = HAL_SAI_Transmit_DMA(hsaiOut, (uint8_t *)&audioOutBuffer[0], AUDIO_BUFFER_SIZE);
	receive_status = HAL_SAI_Receive_DMA(hsaiIn, (uint8_t *)&audioInBuffer[0], AUDIO_BUFFER_SIZE);

}

uint16_t frameCounter = 0;

void audioFrame(uint16_t buffer_offset)
{
	uint16_t i = 0;
	int16_t current_sample = 0;


	frameCounter++;
	if (frameCounter >= 1)
	{
		frameCounter = 0;
		buttonCheck();
	}
	
	for (i = 0; i < (HALF_BUFFER_SIZE); i++)
	{
		if ((i & 1) == 0) {
			current_sample = (int16_t)(audioTickL((float) (audioInBuffer[buffer_offset + i] * INV_TWO_TO_15)) * TWO_TO_15);
		}
		else
		{
			current_sample = (int16_t)(audioTickR((float) (audioInBuffer[buffer_offset + i] * INV_TWO_TO_15)) * TWO_TO_15);
		}
		audioOutBuffer[buffer_offset + i] = current_sample;
	}
}


float audioTickL(float audioIn) 
{
	tRampSetDest(adc[0], (adcVals[0] * INV_TWO_TO_16));
	float newFreq = OOPS_midiToFrequency(tRampTick(adc[0]) * 127.0f);
	tCycleSetFreq(mySine[0], newFreq);
	sample = tCycleTick(mySine[0]);
	return sample;
}

float audioTickR(float audioIn) 
{
	tRampSetDest(adc[1], (adcVals[1] * INV_TWO_TO_16));
	float newFreq = OOPS_midiToFrequency(tRampTick(adc[1]) * 127.0f);
	tCycleSetFreq(mySine[1], newFreq);
	sample = tCycleTick(mySine[1]);
	return sample;
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


			/*

			//RGB LED controls
			//R
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);

			//G
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);

			//B
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);


			*/
			buttonPressed[0] = 0;
	}
	if (buttonPressed[1] == 1)
	{
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
		buttonPressed[1] = 0;
	}
	if (buttonPressed[2] == 1)
	{

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
		buttonPressed[2] = 0;
	}

	if (buttonPressed[3] == 1)
	{
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
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
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
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);;
}

void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{

	audioFrame(0);
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);;
}

