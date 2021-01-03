#ifndef __BOARD__
#define __BOARD__

//
// Configuration
//
#define ALERT_LEVEL 50
#define ALERT_INTERVAL 60
#define ALERT_EVERY ALERT_INTERVAL * 5 * 1000
#define SENSOR_ID 123123
#define SENSOR_LOCATION "15 Isles of Venice, Apt 7"

#define LEQ_PERIOD 1          // second(s)
#define WEIGHTING C_weighting // Also avaliable: 'C_weighting' or 'None' (Z_weighting)
#define LEQ_UNITS "LAeq"      // customize based on above weighting used
#define DB_UNITS "dBA"        // customize based on above weighting used
#define USE_DISPLAY 0

// NOTE: Some microphones require at least DC-Blocker filter
#define MIC_EQUALIZER ICS43434 // See below for defined IIR filters or set to 'None' to disable
#define MIC_OFFSET_DB 3.0103   // Default offset (sine-wave RMS vs. dBFS). Modify this value for linear calibration

// Customize these values from microphone datasheet
#define MIC_SENSITIVITY -26   // dBFS value expected at MIC_REF_DB (Sensitivity value from datasheet)
#define MIC_REF_DB 94.0       // Value at which point sensitivity is specified in datasheet (dB)
#define MIC_OVERLOAD_DB 115.0 // dB - Acoustic overload point
#define MIC_NOISE_DB 29       // dB - Noise floor
#define MIC_BITS 24           // valid number of bits in I2S data
#define MIC_CONVERT(s) (s >> (SAMPLE_BITS - MIC_BITS))
#define MIC_TIMING_SHIFT 0 // Set to one to fix MSB timing for some microphones, i.e. SPH0645LM4H-x

#define LEQ_PERIOD 1          // second(s)
#define WEIGHTING C_weighting // Also avaliable: 'C_weighting' or 'None' (Z_weighting)
#define LEQ_UNITS "LAeq"      // customize based on above weighting used
#define DB_UNITS "dBA"        // customize based on above weighting used

//
// Sampling
//
#define SAMPLE_RATE 48000 // Hz, fixed to design of IIR filters
#define SAMPLE_BITS 32    // bits
#define SAMPLE_T int32_t
#define SAMPLES_SHORT (SAMPLE_RATE / 8) // ~125ms
#define SAMPLES_LEQ (SAMPLE_RATE * LEQ_PERIOD)
#define DMA_BANK_SIZE (SAMPLES_SHORT / 16)
#define DMA_BANKS 32

//
// I2S pins - Can be routed to almost any (unused) ESP32 pin.
//            SD can be any pin, including input only pins (36-39).
//            SCK (i.e. BCLK) and WS (i.e. L/R CLK) must be output capable pins
//
// Below ones are just example for my board layout, put here the pins you will use
//
#define I2S_WS 33       // LRCK Pin
#define I2S_BCK 19      // BCK Pin
#define I2S_DATA_OUT 22 // Data output Pin
#define I2S_DATA_IN 23  // Data input Pin

// I2S peripheral to use (0 or 1)
#define I2S_PORT I2S_NUM_0

//
// I2S Reader Task
//
// Rationale for separate task reading I2S is that IIR filter
// processing cam be scheduled to different core on the ESP32
// while main task can do something else, like update the
// display in the example
//
// As this is intended to run as separate high-priority task,
// we only do the minimum required work with the I2S data
// until it is 'compressed' into sum of squares
//
// FreeRTOS priority and stack size (in 32-bit words)
#define I2S_TASK_PRI 4
#define I2S_TASK_STACK 2048

#endif