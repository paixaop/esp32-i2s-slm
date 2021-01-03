#ifndef __MIC__
#define __MIC__

// NOTE: Some microphones require at least DC-Blocker filter
#define MIC_EQUALIZER ICS43434 // See below for defined IIR filters or set to 'None' to disable
#define MIC_OFFSET_DB 0.0      // Default offset (sine-wave RMS vs. dBFS). Modify this value for linear calibration

// SPM1423 of the M5 Atom Echo
#define MIC_SENSITIVITY -22   // dBFS value expected at MIC_REF_DB (Sensitivity value from datasheet)
#define MIC_REF_DB 94.0       // Value at which point sensitivity is specified in datasheet (dB)
#define MIC_OVERLOAD_DB 115.0 // dB - Acoustic overload point
#define MIC_NOISE_DB 29       // dB - Noise floor
#define MIC_BITS 24           // valid number of bits in I2S data
#define MIC_CONVERT(s) (s >> (SAMPLE_BITS - MIC_BITS))
#define MIC_TIMING_SHIFT 0 // Set to one to fix MSB timing for some microphones, i.e. SPH0645LM4H-x

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

#endif