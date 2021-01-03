#ifndef __BOARD__
#define __BOARD__

//
// Configuration
//

#define LEQ_PERIOD 1          // second(s)
#define WEIGHTING C_weighting // Also avaliable: 'C_weighting' or 'None' (Z_weighting)
#define LEQ_UNITS "LAeq"      // customize based on above weighting used
#define DB_UNITS "dBA"        // customize based on above weighting used

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