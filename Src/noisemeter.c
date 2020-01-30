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
arm_biquad_casd_df1_inst_f32 dba_filter;
float dba_coeffs[5*DBA_CASCADE_SIZE] = DBA_CASCADE_COEFFS;
float dba_state[4*DBA_CASCADE_SIZE];
float dba_average_ringbuffer[DBA_AVERAGE_BUFSIZE];
float dba_val = 0.0f;

float dba_meansquare_1s = 0.0f;
float dba_meansquare_5s = 0.0f;
float dba_meansquare_10s = 0.0f;
float dba_meansquare_30s = 0.0f;
float dba_meansquare_1m = 0.0f;
float dba_meansquare_3m = 0.0f;
float dba_meansquare_5m = 0.0f;

float dba_val_1s = 0.0f;
float dba_val_5s = 0.0f;
float dba_val_10s = 0.0f;
float dba_val_30s = 0.0f;
float dba_val_1m = 0.0f;
float dba_val_3m = 0.0f;
float dba_val_5m = 0.0f;

// dBC weighting biquad filter
arm_biquad_casd_df1_inst_f32 dbc_filter;
float dbc_coeffs[5*DBC_CASCADE_SIZE] = DBC_CASCADE_COEFFS;
float dbc_state[4*DBC_CASCADE_SIZE];
float dbc_val = 0.0f;

float gain_db = 0.0f;

// I2C output buffer
static float i2cData[9] = { 0,0,0,0,0,0,0,0,0 };

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
  for (int i=0; i<DBA_AVERAGE_BUFSIZE; i++) {
    dba_average_ringbuffer[i] = 0.0;
  }
  bzero(dmaBuf, 2*BUFFER_SAMPLES*sizeof(uint16_t));

  arm_biquad_cascade_df1_init_f32(&dba_filter, DBA_CASCADE_SIZE, dba_coeffs, dba_state);
  arm_biquad_cascade_df1_init_f32(&dbc_filter, DBC_CASCADE_SIZE, dbc_coeffs, dbc_state);
  HAL_OPAMP_Start(&hopamp1);
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2048);
  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 2048);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)dmaBuf, 2*BUFFER_SAMPLES);  
  HAL_TIM_Base_Start_IT(&htim15);

  Noisemeter_SetAnalogGain(0);


#ifdef USE_I2C_SLAVE
  //allow I2C to preemt audio processing. I2C interrupts are short and we want to avoid I2C timeouts.
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  HAL_NVIC_SetPriority(I2C1_EV_IRQn, 0, 0);
  HAL_NVIC_SetPriority(I2C1_ER_IRQn, 0, 0);
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 15, 15);
  HAL_I2C_EnableListen_IT(&hi2c1);
#endif
}

#ifdef USE_I2C_SLAVE
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t transferDirection, uint16_t AddrMatchCode) {
  LED_Set(1);
  if (transferDirection == I2C_DIRECTION_RECEIVE) { //direction as seen from master
    //read data: pull to buffer
    i2cData[0] = dba_val;
    i2cData[1] = dbc_val;
    i2cData[2] = dba_val_1s;
    i2cData[3] = dba_val_5s;
    i2cData[4] = dba_val_10s;
    i2cData[5] = dba_val_30s;
    i2cData[6] = dba_val_1m;
    i2cData[7] = dba_val_3m;
    i2cData[8] = dba_val_5m;

    HAL_I2C_Slave_Seq_Transmit_IT(hi2c, (uint8_t*)i2cData, 9*sizeof(float), I2C_LAST_FRAME);
  } else {
    __HAL_I2C_GENERATE_NACK(hi2c);  //TODO: read config
  }
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c) {
  HAL_I2C_EnableListen_IT(hi2c);
  LED_Set(0);
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c) {
  __HAL_I2C_GENERATE_NACK(hi2c);  //no further transmission
  LED_Set(0);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
  __HAL_I2C_GENERATE_NACK(hi2c);  //no further transmission
  LED_Set(0);
}

#endif

float AverageFloats(float* base, uint16_t fromIdx, uint16_t toIdx, uint16_t size) {
  float sum = 0;
  if (toIdx > fromIdx) {
    for (int i=fromIdx+1; i<=toIdx; i++) {
      sum += base[i];
    }
    return sum / (toIdx-fromIdx);
  } else {
    for (int i=fromIdx+1; i<size; i++) {
      sum += base[i];
    }
    for (int i=0; i<=toIdx; i++) {
      sum += base[i];
    }
    return sum / (size+toIdx-fromIdx);
  }
}

void Handle_Samples(uint16_t* base, uint16_t numSamples) {
  // convert to float, offset to zero
  float src[numSamples];
  for (uint16_t i=0; i<numSamples; i++) {
    src[i] = (float)(base[i]) - 32768.0f;
  }
  //do dba weighting
  arm_biquad_cascade_df1_f32(&dba_filter, src, src, numSamples);
  float dba_rms;
  arm_rms_f32( src, numSamples, &dba_rms);
  dba_val = 20 * log10(dba_rms) + DBA_VAL_OFFSET - gain_db;

  //dba averaging
  static uint16_t dba_average_idx;
  dba_average_idx = (dba_average_idx + 1) % DBA_AVERAGE_BUFSIZE;
  dba_average_ringbuffer[dba_average_idx] = dba_rms * dba_rms;

  dba_val_1s = 20 * log10(sqrtf(AverageFloats(dba_average_ringbuffer, (dba_average_idx+DBA_AVERAGE_BUFSIZE-DBA_AVERAGE_1S_BUFFERS)%DBA_AVERAGE_BUFSIZE, dba_average_idx, DBA_AVERAGE_BUFSIZE))) + DBA_VAL_OFFSET - gain_db;
  dba_val_5s = 20 * log10(sqrtf(AverageFloats(dba_average_ringbuffer, (dba_average_idx+DBA_AVERAGE_BUFSIZE-DBA_AVERAGE_5S_BUFFERS)%DBA_AVERAGE_BUFSIZE, dba_average_idx, DBA_AVERAGE_BUFSIZE))) + DBA_VAL_OFFSET - gain_db;
  dba_val_10s = 20 * log10(sqrtf(AverageFloats(dba_average_ringbuffer, (dba_average_idx+DBA_AVERAGE_BUFSIZE-DBA_AVERAGE_10S_BUFFERS)%DBA_AVERAGE_BUFSIZE, dba_average_idx, DBA_AVERAGE_BUFSIZE))) + DBA_VAL_OFFSET - gain_db;
  dba_val_30s = 20 * log10(sqrtf(AverageFloats(dba_average_ringbuffer, (dba_average_idx+DBA_AVERAGE_BUFSIZE-DBA_AVERAGE_30S_BUFFERS)%DBA_AVERAGE_BUFSIZE, dba_average_idx, DBA_AVERAGE_BUFSIZE))) + DBA_VAL_OFFSET - gain_db;
  dba_val_1m = 20 * log10(sqrtf(AverageFloats(dba_average_ringbuffer, (dba_average_idx+DBA_AVERAGE_BUFSIZE-DBA_AVERAGE_1M_BUFFERS)%DBA_AVERAGE_BUFSIZE, dba_average_idx, DBA_AVERAGE_BUFSIZE))) + DBA_VAL_OFFSET - gain_db;
  dba_val_3m = 20 * log10(sqrtf(AverageFloats(dba_average_ringbuffer, (dba_average_idx+DBA_AVERAGE_BUFSIZE-DBA_AVERAGE_3M_BUFFERS)%DBA_AVERAGE_BUFSIZE, dba_average_idx, DBA_AVERAGE_BUFSIZE))) + DBA_VAL_OFFSET - gain_db;
  dba_val_5m = 20 * log10(sqrtf(AverageFloats(dba_average_ringbuffer, (dba_average_idx+DBA_AVERAGE_BUFSIZE-DBA_AVERAGE_5M_BUFFERS)%DBA_AVERAGE_BUFSIZE, dba_average_idx, DBA_AVERAGE_BUFSIZE))) + DBA_VAL_OFFSET - gain_db;

  // convert input data to float again, offset to zero
  // double work, save memory - we might reduce block size and avoid this if processing gets tight
  for (uint16_t i=0; i<numSamples; i++) {
    src[i] = (float)(base[i]) - 32768.0f;
  }
  //do dbc weighting
  arm_biquad_cascade_df1_f32(&dbc_filter, src, src, numSamples);
  float dbc_rms;
  arm_rms_f32( src, numSamples, &dbc_rms);
  dbc_val = 20 * log10(dbc_rms) + DBC_VAL_OFFSET - gain_db;

#ifdef USE_USB_SERIAL
  char strBuf[100];
//  sprintf(strBuf, "samples %i dba %7.2f avg %li min %i max %i\n",numSamples, dba, dbaSampleAvg/SMOOTH_FACTOR, min, max);
  sprintf(strBuf, "dba %7.2f dbc %7.2f I2C state %i mode %i error %li\n", dba_val, dbc_val,
    HAL_I2C_GetState(&hi2c1),HAL_I2C_GetMode(&hi2c1),HAL_I2C_GetError(&hi2c1));
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
      gain_db = 0.0f;
      break;
    case 1:
      LL_OPAMP_SetFunctionalMode(OPAMP1, LL_OPAMP_MODE_PGA);
      LL_OPAMP_SetPGAGain(OPAMP1, LL_OPAMP_PGA_GAIN_2);
      HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 1024);
      gain_db = 6.0f;
      break;
    case 2:
      LL_OPAMP_SetFunctionalMode(OPAMP1, LL_OPAMP_MODE_PGA);
      LL_OPAMP_SetPGAGain(OPAMP1, LL_OPAMP_PGA_GAIN_4);
      HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 512);
      gain_db = 12.0f;
      break;
    case 3:
      LL_OPAMP_SetFunctionalMode(OPAMP1, LL_OPAMP_MODE_PGA);
      LL_OPAMP_SetPGAGain(OPAMP1, LL_OPAMP_PGA_GAIN_8);
      HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 256);
      gain_db = 18.0f;
      break;
    default:
      LL_OPAMP_SetFunctionalMode(OPAMP1, LL_OPAMP_MODE_PGA);
      LL_OPAMP_SetPGAGain(OPAMP1, LL_OPAMP_PGA_GAIN_16);
      HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 128);
      gain_db = 24.0f;
      break;
  }
}
