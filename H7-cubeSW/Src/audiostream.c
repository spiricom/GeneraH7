/* Includes ------------------------------------------------------------------*/
#include "audiostream.h"
#include "main.h"
#include "leaf.h"
#include "codec.h"
#include "tim.h"
#include "ui.h"

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



#define NUM_BUTTONS 4
uint8_t buttonValues[NUM_BUTTONS];
uint8_t buttonValuesPrev[NUM_BUTTONS];
uint32_t buttonCounters[NUM_BUTTONS];
uint32_t buttonPressed[NUM_BUTTONS];


float sample = 0.0f;

uint16_t frameCounter = 0;

//audio objects
tRamp adc[12];
t808Hihat myHihat;

tExpSmooth mySmooth;
tLivingString myLString;

tSawtooth saw;
tFIR fir;
tNoise noise;

float cs[32];

const float firCoeffs32[29] = {
  -0.0018225230f, -0.0015879294f, +0.0000000000f, +0.0036977508f, +0.0080754303f, +0.0085302217f, -0.0000000000f, -0.0173976984f,
  -0.0341458607f, -0.0333591565f, +0.0000000000f, +0.0676308395f, +0.1522061835f, +0.2229246956f, +0.2504960933f, +0.2229246956f,
  +0.1522061835f, +0.0676308395f, +0.0000000000f, -0.0333591565f, -0.0341458607f, -0.0173976984f, -0.0000000000f, +0.0085302217f,
  +0.0080754303f, +0.0036977508f, +0.0000000000f, -0.0015879294f, -0.0018225230f
};
/**********************************************/

typedef enum BOOL {
	FALSE = 0,
	TRUE
} BOOL;


void audioInit(I2C_HandleTypeDef* hi2c, SAI_HandleTypeDef* hsaiOut, SAI_HandleTypeDef* hsaiIn, uint16_t* adc_array)
{
	// Initialize LEAF.

	LEAF_init(SAMPLE_RATE, AUDIO_FRAME_SIZE, &randomNumber);

	adcVals = adc_array; //in audiostream.h, the array of adc values is now called adcVals.

	// adcVals array has the data organized as follows:
	// [0] = knob 1
	// [1] = knob 2
	// [2] = knob 3
	// [3] = knob 4
	// [4] = knob 5 (depending on jumper K)
	// [5] = knob 6 (depending on jumper L)
	// [6] = knob 7 or jack 11 (depending on jumper M)
	// [7] = knob 8 or jack 12 (depending on jumper N and jumper O)
	// [8] = jack 1
	// [9] = jack 2
	// [10] = jack 3 (depending on jumper A and jumper B)
	// [11] = jack 4 (depending on jumper C and jumper D)

	// note that the knobs come in with the data backwards (fully clockwise is near zero, counter-clockwise is near 65535)
	// the CVs come in as expected (0V = 0, 10V = 65535)


	for (int i = 0; i < 12; i++)
	{
		tRamp_init(&adc[i],7.0f, 1); //set all ramps for knobs/jacks to be 7ms ramp time and let the init function know they will be ticked every sample
		//if you want to read different knobs/jacks at different rates or with different smoothing times, you can reinit after this
	}

	t808Hihat_init (&myHihat);
	//now to send all the necessary messages to the codec
	AudioCodec_init(hi2c);
	tSawtooth_init(&saw);
	float cutoff = 10.0f;
	for (int i = 0; i < 32; ++i){
		cs[i] = sinf(cutoff * leaf.invSampleRate * i)/(cutoff * leaf.invSampleRate  * i);
	}
	cs[0] = 1.0f;
	tNoise_init(&noise, WhiteNoise);

	tFIR_init(&fir, firCoeffs32);

	HAL_Delay(100);

	for (int i = 0; i < AUDIO_BUFFER_SIZE; i++)
	{
		audioOutBuffer[i] = 0;
	}


	tExpSmooth_init(&mySmooth,0,0.0001);
	tLivingString_init(&myLString, 440.f, 0.2f, 0.f, 9000.f, 1.0f, 0.3f, 0.01f, 0.125f, 0);
	//tSimpleLivingString_setLevMode(&myLString, mode1);
	//float freq, float pickPos, float prepIndex, float dampFreq, float decay, float targetLev, float levSmoothFactor, float levStrength, int levMode)

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

uint8_t hatTriggered = 0;
float rightIn = 0.0f;

float tickedRamps[12];

float audioTickL(float audioIn)
{
	//read the analog inputs and smooth them with ramps
	tRamp_setDest(&adc[0], 1.0f - (adcVals[0] * INV_TWO_TO_16));
	tRamp_setDest(&adc[1], 1.0f - (adcVals[1] * INV_TWO_TO_16));
	tRamp_setDest(&adc[2], 1.0f - (adcVals[2] * INV_TWO_TO_16));
	tRamp_setDest(&adc[3], 1.0f - (adcVals[3] * INV_TWO_TO_16));
	tRamp_setDest(&adc[4], 1.0f - (adcVals[4] * INV_TWO_TO_16));
	tRamp_setDest(&adc[5], 1.0f - (adcVals[5] * INV_TWO_TO_16));
	tRamp_setDest(&adc[6], 1.0f - (adcVals[6] * INV_TWO_TO_16));
	tRamp_setDest(&adc[7], 1.0f - (adcVals[7] * INV_TWO_TO_16));
	tRamp_setDest(&adc[8], (adcVals[8] * INV_TWO_TO_16));
	tRamp_setDest(&adc[9], (adcVals[9] * INV_TWO_TO_16));
	tRamp_setDest(&adc[10], (adcVals[10] * INV_TWO_TO_16));
	tRamp_setDest(&adc[11], (adcVals[11] * INV_TWO_TO_16));


	for (int i = 0; i < 12; i++)
	{
		tickedRamps[i] = tRamp_tick(&adc[i]);
	}


	float drumGain = LEAF_clip(0.0f, tickedRamps[0] + tickedRamps[8], 2.0f);
	//if digital input on jack 5, then trigger drum/hihat




	/*
	if ((!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)) == 1)
	{
		if (hatTriggered == 0)
		{
			t808Hihat_on(&myHihat, drumGain);
			hatTriggered = 1;
		}
	}
	else
	{
		hatTriggered = 0;
	}
*/

	if (audioIn < -0.25f)
	{
		if (hatTriggered == 0)
		{
			t808Hihat_on(&myHihat, drumGain);
			hatTriggered = 1;
		}
	}
	else
	{
		hatTriggered = 0;
	}


	//OK, now some audio stuff


	float newFreq = LEAF_clip(0.0f, LEAF_midiToFrequency(tickedRamps[4] * 100.0f) + (tickedRamps[9] * 500.0f * tickedRamps[5]) + (rightIn * 1000.0f), 23000.0f);

	t808Hihat_setOscBandpassFreq(&myHihat, LEAF_clip (500.0f, ((tickedRamps[11] * 9000.0f) + 500.0f), 18000.0f));
	t808Hihat_setHighpassFreq(&myHihat, LEAF_midiToFrequency(tickedRamps[2] * 127.0f)); //knob 4 sets hipass freq
	t808Hihat_setOscNoiseMix(&myHihat, tickedRamps[1]);
	t808Hihat_setDecay(&myHihat, (tickedRamps[3] * 1000.0f) + (tickedRamps[10] * 1000.0f));
	t808Hihat_setOscFreq(&myHihat, newFreq);
	t808Hihat_setStickBandPassFreq(&myHihat, (tickedRamps[6] * 1000.0f) + 2500.0f);
	t808Hihat_setStickBandPassQ(&myHihat, (tickedRamps[7] * 2.0f) + 0.2f);

	sample = t808Hihat_tick(&myHihat) * 2.0f;


	LEAF_shaper(sample, 1.7f);
	return sample;
}

uint8_t inAttack = 0;
uint32_t sampleLength = 0;
float myRate = 1.0f;
float startTime = 0.0f;
float endTime = 23000.0f;
float smooth = 0.0f;

float audioTickR(float audioIn)
{
	rightIn = audioIn;

	sample = 0.0f;


/*
	smooth = 1.0f - LEAF_clip(0.1f, ((tickedRamps[1]) + (tickedRamps[9])), 1.0f);

    tExpSmooth_setFactor(&mySmooth,smooth *0.02);  // knob 2 controls smooth factor
    tExpSmooth_setDest(&mySmooth, LEAF_midiToFrequency((tickedRamps[0] * 72.0f) + (tickedRamps[8] * 120.0f))); // update frequency directly from knob 1


    float freq = tExpSmooth_tick(&mySmooth);
    tLivingString_setFreq(&myLString, freq);
    tLivingString_setPickPos(&myLString, LEAF_clip(0.05f, tickedRamps[2] + tickedRamps[10], .95f));
    tLivingString_setPrepIndex(&myLString, tickedRamps[3]);
    tLivingString_setDampFreq(&myLString, 20000.0f*(LEAF_clip(0.1f, (tickedRamps[4] + tickedRamps[11]), .99f)));
    tLivingString_setDecay(&myLString, 1.0f-(0.02f*tickedRamps[5]));
    tLivingString_setLevStrength(&myLString,tickedRamps[6]);


    sample=tLivingString_tick(&myLString, audioIn);


	/*
	float audioEnv = tEnvelopeFollower_tick(&env, audioIn);
	//if ((audioEnv > 0.3f) && (inAttack != 1))
	if ((mode1==1) && (inAttack != 1))
	{
		tBuffer_record(&buff);
		inAttack = 1;
	}
	else if (mode1==0)
	//else if ((audioEnv < 0.2f) && (inAttack == 1))
	{
		inAttack = 0;
	}


   myRate = LEAF_clip(0.25f, (((tickedRamps[2] * 2.0f)) + ((tickedRamps[10] * 2.0f))), 2.0f);
    //float startTime = LEAF_clip(0, ((tickedRamps[0] * 24000.0f) + (tickedRamps[8] * 24000.0f)), 24000);
    //float endTime = LEAF_clip(0, ((tickedRamps[1] * 24000.0f) + (tickedRamps[9] * 24000.0f)), 24000);

    //float myRate = 1.0f;
    startTime = 0.0f;
    endTime = 23000.0f;



    tSampler_setStart(&samp, startTime);
    tSampler_setEnd(&samp, endTime);
    tSampler_setRate(&samp, myRate);

    tBuffer_tick(&buff, audioIn); // ticking the buffer records in to buffer

    // dont tick sampler if buffer is active (not recording)
    if (buff.active == 0)
    {
        sample = tSampler_tick(&samp); // ticking sampler loops sample
    }

    //if (startTime > 23000)
    {
    	//startTime = 23000;
    }
    //if (endTime <= startTime + 4)
    {
    	//endTime = startTime + 4;
    }

*/
	/*
	sample *= 1.3f;
	LEAF_shaper(sample, 1.2f);
*/
	//tSawtooth_setFreq(&saw, 300.0f);
	//sample = tSawtooth_tick(&saw);

	sample = tNoise_tick(&noise);
	sample *= 0.8f;
	sample = tFIR_tick(&fir,sample);

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

		RGB_mode++;
		if (RGB_mode > 3)
		{
			RGB_mode = 0;
		}
		if (RGB_mode == 0)
		{
			RGB_LED_setColor(255, 0, 0);
		}

		else if (RGB_mode == 1)
		{
			RGB_LED_setColor(0, 255, 0);
		}

		else if (RGB_mode == 2)
		{
			RGB_LED_setColor(0, 0, 255);
		}

		else if (RGB_mode == 3)
		{

		}
		buttonPressed[0] = 0;
	}
	if (buttonPressed[1] == 1)
	{

		if (mode1 == 0)
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
			mode1 = 1;
		}
		else
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
			mode1 = 0;
		}

		buttonPressed[1] = 0;
	}
	if (buttonPressed[2] == 1)
	{
		if (mode2 == 0)
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
			mode2 = 1;
		}
		else
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
			mode2 = 0;
		}
		buttonPressed[2] = 0;
	}

	if (buttonPressed[3] == 1)
	{

		if (mode3 == 0)
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
			mode3 = 1;
		}
		else
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
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
