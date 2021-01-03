/*
 * Display A-weighted sound level measured by I2S Microphone
 *
 * (c)2019 Ivan Kostoski
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

/*
 * Sketch samples audio data from I2S microphone, processes the data
 * with digital IIR filters and calculates A or C weighted Equivalent
 * Continuous Sound Level (Leq)
 *
 * I2S is setup to sample data at Fs=48000KHz (fixed value due to
 * design of digital IIR filters). Data is read from I2S queue
 * in 'sample blocks' (default 125ms block, equal to 6000 samples)
 * by 'i2s_reader_task', filtered trough two IIR filters (equalizer
 * and weighting), summed up and pushed into 'samples_queue' as
 * sum of squares of filtered samples. The main task then pulls data
 * from the queue and calculates decibel value relative to microphone
 * reference amplitude, derived from datasheet sensitivity dBFS
 * value, number of bits in I2S data, and the reference value for
 * which the sensitivity is specified (typically 94dB, pure sine
 * wave at 1KHz).
 *
 * Displays line on the small OLED screen with 'short' LAeq(125ms)
 * response and numeric LAeq(1sec) dB value from the signal RMS.
 */

#include <driver/i2s.h>
#include "sos-iir-filter.h"

#include "utlgbotlib.h"
#include "telegram_config.h"
#include "wifi_config.h"
#include "moving_average.h"
#include "filters.h"
#include "board_config.h"

//
// Configuration
//
#define ALERT_LEVEL 50
#define ALERT_INTERVAL 60
#define ALERT_EVERY ALERT_INTERVAL * 5 * 1000
#define SENSOR_ID 123123
#define SENSOR_LOCATION "15 Isles of Venice, Apt 7"

#define NETWORKING 0

// Calculate reference amplitude value at compile time
constexpr double MIC_REF_AMPL = pow(10, double(MIC_SENSITIVITY) / 20) * ((1 << (MIC_BITS - 1)) - 1);

// Data we push to 'samples_queue'
struct sum_queue_t
{
    // Sum of squares of mic samples, after Equalizer filter
    float sum_sqr_SPL;
    // Sum of squares of weighted mic samples
    float sum_sqr_weighted;
    // Debug only, FreeRTOS ticks we spent processing the I2S data
    uint32_t proc_ticks;
};
QueueHandle_t samples_queue;

// Static buffer for block of samples
float samples[SAMPLES_SHORT] __attribute__((aligned(4)));

// Telegram Bot
uTLGBot Bot(TLG_TOKEN);

//
// I2S Microphone sampling setup
//
void mic_i2s_init()
{
    // Setup I2S to sample mono channel for SAMPLE_RATE * SAMPLE_BITS
    // NOTE: Recent update to Arduino_esp32 (1.0.2 -> 1.0.3)
    //       seems to have swapped ONLY_LEFT and ONLY_RIGHT channels
    const i2s_config_t i2s_config = {
        mode : i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM),
        sample_rate : SAMPLE_RATE,
        bits_per_sample : i2s_bits_per_sample_t(SAMPLE_BITS),
        channel_format : I2S_CHANNEL_FMT_ONLY_LEFT,
        communication_format : i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
        intr_alloc_flags : ESP_INTR_FLAG_LEVEL1,
        dma_buf_count : DMA_BANKS,
        dma_buf_len : DMA_BANK_SIZE,
        /*use_apll : true,
    tx_desc_auto_clear : false,
    fixed_mclk : 0*/
    };
    // I2S pin mapping
    const i2s_pin_config_t pin_config = {
        bck_io_num : I2S_BCK,
        ws_io_num : I2S_WS,
        data_out_num : -1, // not used
        data_in_num : I2S_DATA_IN
    };

    i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);

#if (MIC_TIMING_SHIFT > 0)
    // Undocumented (?!) manipulation of I2S peripheral registers
    // to fix MSB timing issues with some I2S microphones
    REG_SET_BIT(I2S_TIMING_REG(I2S_PORT), BIT(9));
    REG_SET_BIT(I2S_CONF_REG(I2S_PORT), I2S_RX_MSB_SHIFT);
#endif

    i2s_set_pin(I2S_PORT, &pin_config);
    i2s_set_clk(I2S_PORT, SAMPLE_RATE, i2s_bits_per_sample_t(SAMPLE_BITS), I2S_CHANNEL_MONO);

    //FIXME: There is a known issue with esp-idf and sampling rates, see:
    //       https://github.com/espressif/esp-idf/issues/2634
    //       In the meantime, the below line seems to set sampling rate at ~47999.992Hz
    //       fifs_req=24576000, sdm0=149, sdm1=212, sdm2=5, odir=2 -> fifs_reached=24575996
    //NOTE:  This seems to be fixed in ESP32 Arduino 1.0.4, esp-idf 3.2
    //       Should be safe to remove...
    //#include <soc/rtc.h>
    //rtc_clk_apll_enable(1, 149, 212, 5, 2);
}

//
// I2S Reader Task
//
// Rationale for separate task reading I2S is that IIR filter
// processing cam be scheduled to different core on the ESP32
// while main task can do something else, like update the
// display in the example
//
// As this is intended to run as separate hihg-priority task,
// we only do the minimum required work with the I2S data
// until it is 'compressed' into sum of squares
//
// FreeRTOS priority and stack size (in 32-bit words)
#define I2S_TASK_PRI 4
#define I2S_TASK_STACK 2048
//
void mic_i2s_reader_task(void *parameter)
{
    mic_i2s_init();

    // Discard first block, microphone may have startup time (i.e. INMP441 up to 83ms)
    size_t bytes_read = 0;
    i2s_read(I2S_PORT, &samples, SAMPLES_SHORT * sizeof(int32_t), &bytes_read, portMAX_DELAY);

    while (true)
    {
        // Block and wait for microphone values from I2S
        //
        // Data is moved from DMA buffers to our 'samples' buffer by the driver ISR
        // and when there is requested ammount of data, task is unblocked
        //
        // Note: i2s_read does not care it is writing in float[] buffer, it will write
        //       integer values to the given address, as received from the hardware peripheral.
        i2s_read(I2S_PORT, &samples, SAMPLES_SHORT * sizeof(SAMPLE_T), &bytes_read, portMAX_DELAY);

        TickType_t start_tick = xTaskGetTickCount();

        // Convert (including shifting) integer microphone values to floats,
        // using the same buffer (assumed sample size is same as size of float),
        // to save a bit of memory
        SAMPLE_T *int_samples = (SAMPLE_T *)&samples;
        for (int i = 0; i < SAMPLES_SHORT; i++)
            samples[i] = MIC_CONVERT(int_samples[i]);

        sum_queue_t q;
        // Apply equalization and calculate Z-weighted sum of squares,
        // writes filtered samples back to the same buffer.
        q.sum_sqr_SPL = MIC_EQUALIZER.filter(samples, samples, SAMPLES_SHORT);

        // Apply weighting and calucate weigthed sum of squares
        q.sum_sqr_weighted = WEIGHTING.filter(samples, samples, SAMPLES_SHORT);

        // Debug only. Ticks we spent filtering and summing block of I2S data
        q.proc_ticks = xTaskGetTickCount() - start_tick;

        // Send the sums to FreeRTOS queue where main task will pick them up
        // and further calcualte decibel values (division, logarithms, etc...)
        xQueueSend(samples_queue, &q, portMAX_DELAY);
    }
}

const char *noise_level(double level)
{
    if (level <= 30)
    {
        return "Faint";
    }

    if (level <= 40)
    {
        return "Whisper";
    }

    if (level <= 50)
    {
        return "Soft";
    }

    if (level <= 60)
    {
        return "Moderate";
    }

    if (level <= 70)
    {
        return "Normal Conversation";
    }

    if (level <= 80)
    {
        return "Loud";
    }

    if (level <= 90)
    {
        return "Loud. Avoid prolonged exposure";
    }

    if (level <= 110)
    {
        return "Very Loud. Dangerous over 30 minutes";
    }

    if (level <= 120)
    {
        return "Uncomfortable. Dangerous over 30 seconds";
    }

    return "Painful and Dangerous. Use hearing protection or avoid";
}

//
// Setup and main loop
//
// Note: Use doubles, not floats, here unless you want to pin
//       the task to whichever core it happens to run on at the moment
//
void setup()
{

    // If needed, now you can actually lower the CPU frquency,
    // i.e. if you want to (slightly) reduce ESP32 power consumption
    setCpuFrequencyMhz(80); // It should run as low as 80MHz

    Serial.begin(112500);
    delay(1000); // Safety

    // Connect to the WiFi network (see function below loop)
#if (NETWORKING > 0)
    connectToWiFi(WIFI_SSID, WIFI_PASS);
#endif

    // Create FreeRTOS queue
    samples_queue = xQueueCreate(8, sizeof(sum_queue_t));

    // Create the I2S reader FreeRTOS task
    // NOTE: Current version of ESP-IDF will pin the task
    //       automatically to the first core it happens to run on
    //       (due to using the hardware FPU instructions).
    //       For manual control see: xTaskCreatePinnedToCore
    xTaskCreate(mic_i2s_reader_task, "Mic I2S Reader", I2S_TASK_STACK, NULL, I2S_TASK_PRI, NULL);

    sum_queue_t q;
    uint32_t Leq_samples = 0;
    double Leq_sum_sqr = 0;
    double Leq_dB = 0;

    int alert_interval = 0;
    bool first_alert = true;
    Moving_Average<float, ALERT_INTERVAL> ma;
    char output[200];

    // Read sum of samaples, calculated by 'i2s_reader_task'
    while (xQueueReceive(samples_queue, &q, portMAX_DELAY))
    {

        // Calculate dB values relative to MIC_REF_AMPL and adjust for microphone reference
        double short_RMS = sqrt(double(q.sum_sqr_SPL) / SAMPLES_SHORT);
        double short_SPL_dB = MIC_OFFSET_DB + MIC_REF_DB + 20 * log10(short_RMS / MIC_REF_AMPL);

        // In case of acoustic overload or below noise floor measurement, report infinty Leq value
        if (short_SPL_dB > MIC_OVERLOAD_DB)
        {
            Leq_sum_sqr = INFINITY;
        }
        else if (isnan(short_SPL_dB) || (short_SPL_dB < MIC_NOISE_DB))
        {
            Leq_sum_sqr = -INFINITY;
        }

        // Accumulate Leq sum
        Leq_sum_sqr += q.sum_sqr_weighted;
        Leq_samples += SAMPLES_SHORT;

        // When we gather enough samples, calculate new Leq value
        if (Leq_samples >= SAMPLE_RATE * LEQ_PERIOD)
        {
            double Leq_RMS = sqrt(Leq_sum_sqr / Leq_samples);
            Leq_dB = MIC_OFFSET_DB + MIC_REF_DB + 20 * log10(Leq_RMS / MIC_REF_AMPL);
            Leq_sum_sqr = 0;
            Leq_samples = 0;

            if (Leq_dB > MIC_OVERLOAD_DB)
            {
                // Display 'Overload' if dB value is over the AOP
                Serial.println("Overload");
            }
            else if (isnan(Leq_dB) || (Leq_dB < MIC_NOISE_DB))
            {
                // Display 'Low' if dB value is below noise floor
                Serial.println("Low");
            }
            else
            {
                // Serial output, customize (or remove) as needed
                Serial.printf("Moving Average: %.1fdb Level: %.1fdB - %s\n", ma.get(), Leq_dB, noise_level(Leq_dB));
            }

            // Calculate the moving average of the level
            ma(Leq_dB);

            if (ma.get() >= ALERT_LEVEL)
            {
                Serial.printf("Moving Average above %d! - Time: %d\n", ALERT_LEVEL, millis());

                // If its the first time we're alerting don't care about time intervals
                if (first_alert || (millis() - alert_interval >= ALERT_EVERY))
                {
                    first_alert = false;
                    sprintf(output, "Noise Alert! The sensor at %s is reporting sustained noise above %0.1fdB - %s.", SENSOR_LOCATION, ma.get(), noise_level(ma));
                    Serial.println(output);

#if (NETWORKING > 0)
                    Bot.sendMessage(TLG_CHAT_ID, output);
#endif
                    alert_interval = millis();
                }
            }

            // Debug only
            //Serial.printf("%u processing ticks\n", q.proc_ticks);
        }
    }
}

void loop()
{
    // Nothing here..
}
