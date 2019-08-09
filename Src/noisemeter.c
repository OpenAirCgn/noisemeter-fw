#include "noisemeter.h"

#define ARM_MATH_CM4
#define __FPU_PRESENT 1
#include "arm_math.h"
#include "main.h"
#include "stm32l4xx_ll_opamp.h"

#ifdef USE_USB_SERIAL
#include "usbd_cdc_if.h"
#endif

// peripheral handles

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern DAC_HandleTypeDef hdac1;
extern I2C_HandleTypeDef hi2c1;
extern OPAMP_HandleTypeDef hopamp1;
extern SAI_HandleTypeDef hsai_BlockA1;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim15;

// memory buffer for mic read

uint16_t dmaBuf[2*BUFFER_SAMPLES];

// dBA weighting biquad filter

arm_biquad_casd_df1_inst_q15 dBA_filter;

q15_t dBA_coeffs[6*DBA_CASCADE_SIZE] = DBA_CASCADE_COEFFS;

q15_t dBA_state[4*DBA_CASCADE_SIZE];
uint32_t dbaSampleAvg = 0;
q15_t dBA_RMS;

// I2C output buffer

static uint8_t i2cData[4] = { 1,2,3,4 };

// forward function for USB audio (tweaked middleware)

#ifdef USE_USB_AUDIO
extern void Forward_Samples(uint16_t* samples, uint16_t numSamples);
#endif

// debug LED access

void LED_Set(uint8_t on) {
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, on);
}

void LED_Toggle(void) {
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, !HAL_GPIO_ReadPin(LED_GPIO_Port, LED_Pin));
}

void LED_Toggle_Modulo(void) {
  static uint32_t counter = 0;
  counter++;
  if (counter >= 500) {
    LED_Toggle();
    counter = 0;
  }
}

void Noisemeter_Init(void) {
  arm_biquad_cascade_df1_init_q15(&dBA_filter, DBA_CASCADE_SIZE, dBA_coeffs, dBA_state, DBA_FILTER_SHIFT);
  HAL_OPAMP_Start(&hopamp1);
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2048);
  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 2048);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)dmaBuf, 2*BUFFER_SAMPLES);  
  HAL_TIM_Base_Start_IT(&htim15);

  Noisemeter_SetAnalogGain(0);

#ifdef USE_I2C_SLAVE
  HAL_I2C_EnableListen_IT(&hi2c1);
#endif
}

#ifdef USE_I2C_SLAVE
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t transferDirection, uint16_t AddrMatchCode) {
  if (transferDirection == I2C_DIRECTION_RECEIVE) { //direction as seen from master
    //read data: pull to buffer
    ((q15_t*)i2cData)[0] = dBA_RMS;
    HAL_I2C_Slave_Seq_Transmit_IT(hi2c, i2cData, 4, 0);
  } else {
    __HAL_I2C_GENERATE_NACK(hi2c);  //TODO: read config
  }
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c) {
  HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c) {
  __HAL_I2C_GENERATE_NACK(hi2c);  //no further transmission
}

#endif


void Handle_Samples(uint16_t* base, uint16_t numSamples) {
  //show high level on LED to show we're alive
  uint16_t min = 0xffff;
  uint16_t max = 0x0000;
  for(int i=0;i<numSamples;i++) {
    uint16_t val = base[i];
    if (val < min) min = val;
    if (val > max) max = val;
  }
  LED_Set(max-min > 0x1000);

  //offset samples to signed numbers, idle near zero (IN-PLACE!)
  // uint16_t numWords = numSamples / 2;
  // uint32_t *wordBuf = (uint32_t*)base;
  // for (int i=0; i<numWords; i++) {
  //   wordBuf[i] ^=0x80008000;
  // }
  
  // //do dba weighting
  // q15_t *src = (q15_t*)base;
  // q15_t dbaSamples[numSamples];
  // arm_biquad_cascade_df1_q15(&dBA_filter, src, dbaSamples, numSamples);

  
  // //biquads may offset zero so compensate mean to zero. Important for RMS
  // q15_t mean;
  // arm_mean_q15 (dba, numSamples, &mean);  
  // dbaSampleAvg = ((dbaSampleAvg_SMOOTH-1)*dbaSampleAvg)/dbaSampleAvg_SMOOTH + mean;
  // arm_offset_q15 ( dba, -dbaSampleAvg/dbaSampleAvg_SMOOTH, dba, numSamples);
  // arm_rms_q15( dba, numSamples, &dBA_RMS);

#define SMOOTH_FACTOR 0x8000

  uint32_t sqErrSum = 0;
  for (int i=0; i<numSamples; i++) {
    uint16_t sample = base[i];
    uint16_t zero = 31718;//dbaSampleAvg / SMOOTH_FACTOR;
    int32_t err = (sample-zero);
    sqErrSum += err*err;
    dbaSampleAvg = dbaSampleAvg - (dbaSampleAvg/SMOOTH_FACTOR) + sample;
  }
  float sqErrAvg = (float)sqErrSum / (float)numSamples;
  float rms = sqrtf(sqErrAvg);
  float dba = 20 * log10(rms);


#ifdef USE_USB_SERIAL
  char strBuf[100];
//  sprintf(strBuf, "samples %i dba %7.2f avg %li min %i max %i\n",numSamples, dba, dbaSampleAvg/SMOOTH_FACTOR, min, max);
  sprintf(strBuf, "dba %7.2f min %i max %i\n", dba, min, max);
  CDC_Transmit_FS((uint8_t*)strBuf, strlen(strBuf));
#endif

#ifdef USE_USB_AUDIO
  //send to USB - needs appropriate numSamples (1ms)
  Forward_Samples((uint16_t*)dba, numSamples);
#endif
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  Handle_Samples(&(dmaBuf[BUFFER_SAMPLES]), BUFFER_SAMPLES);
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {
  Handle_Samples(dmaBuf, BUFFER_SAMPLES);
}


void Noisemeter_SetAnalogGain(uint8_t gain) {
  switch (gain) {
    case 0:
      LL_OPAMP_SetFunctionalMode(OPAMP1, LL_OPAMP_MODE_FOLLOWER);
      HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2048);
      break;
    case 1:
      LL_OPAMP_SetFunctionalMode(OPAMP1, LL_OPAMP_MODE_PGA);
      LL_OPAMP_SetPGAGain(OPAMP1, LL_OPAMP_PGA_GAIN_2);
      HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 1024);
      break;
    case 2:
      LL_OPAMP_SetFunctionalMode(OPAMP1, LL_OPAMP_MODE_PGA);
      LL_OPAMP_SetPGAGain(OPAMP1, LL_OPAMP_PGA_GAIN_4);
      HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 512);
      break;
    case 3:
      LL_OPAMP_SetFunctionalMode(OPAMP1, LL_OPAMP_MODE_PGA);
      LL_OPAMP_SetPGAGain(OPAMP1, LL_OPAMP_PGA_GAIN_8);
      HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 256);
      break;
    default:
      LL_OPAMP_SetFunctionalMode(OPAMP1, LL_OPAMP_MODE_PGA);
      LL_OPAMP_SetPGAGain(OPAMP1, LL_OPAMP_PGA_GAIN_16);
      HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 128);
      break;
  }
}
