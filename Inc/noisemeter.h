#ifndef _NOISEMETER_H_
#define _NOISEMETER_H_

#include <stdint.h>

#define SAMPLE_RATE 48000 /* Adopt IIR filters appropriately */
#define PROCESSOR_CLOCK 80000000
#define SAMPLE_RELOAD (PROCESSOR_CLOCK / SAMPLE_RATE)
#define BUFFER_MS 125
#define BUFFER_SAMPLES ((SAMPLE_RATE * BUFFER_MS) / 1000)
// #define USE_USB_AUDIO /* BUFFER_MS should be 1 for this */
#define USE_USB_SERIAL

#define DBA_CASCADE_SIZE 2
#define DBA_CASCADE_COEFFS {16384,0,0,0,0,0,16384,0,0,0,0,0}
//#define DBA_CASCADE_COEFFS {6114,0,0,-6114,19451,-4157,16238,0,-32475,16238,32475,-16092}
#define DBA_FILTER_SHIFT 1
#define DBA_MEAN_SMOOTH 256

#define USE_I2C_SLAVE 
#define I2C_SLAVE_ADDRESS 0x20 /* shifted left (alt: 0x10) */

void Noisemeter_Init(void);

void Noisemeter_SetAnalogGain(uint8_t gain); //gain 0=no gain, 1=x2, 2=x4, 3=x8, 4...=x16 

#endif

