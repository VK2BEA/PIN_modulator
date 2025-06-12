/* 
 * Espressif ESP32 program to generate a 1kHz modulation waveform for the
 * HP8733B PIN modulator.
 * The hardware is an ESP32-WROVER connected to an I2S PCM5102A digital to analog converter.
 * ESP32 IO 22 - PCM5102A I2S BCK
 * ESP32 IO 5  - PCM5102A I2S LRCK
 * ESP32 IO 21 - PCM5102A I2S DIN
 * PCM5102A settings: FLT  - Normal Latency (gnd)
 *                    DEMP - Off (gnd)
 *                    XMST - Soft un-mute (high)
 *                    FMT  - I2S (gnd)
 * 	PCM5102A SCK (gnd) (not used)
 *				  
 * The attenuation of the HP8733B is logarithmic with diode current (approx 3dB per 0.1mA)
 * In order to amplitude modulate a signal, a corrective pre-distortion is applied to the modulating
 * sinewave.
 * 
 *
 * Copyright (c) 2025 Michael G. Katzmann.
 * 
 * This program is free software: you can redistribute it and/or modify  
 * it under the terms of the GNU General Public License as published by  
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 
 *
 * Output the log of a 1 KHz sinewave to a PCM5102A I2S audio board
 * for the HP8733B 3.7GHz to 8.3 GHz PIN modulator
 *
 * BCK       - IO 22
 * WR (LRCK) - IO  5  
 * DOUT      - IO 21
 */

#include <stdint.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "sdkconfig.h"
#include <math.h>
#include <string.h>
#include <endian.h>

#define PCM5102A_BCK    GPIO_NUM_22
#define PCM5102A_WR     GPIO_NUM_5
#define PCM5102A_DOUT   GPIO_NUM_21
#define PCM5102A_DIN    GPIO_NUM_16
#define PCM5102A_MCLK   GPIO_NUM_0

#define SAMPLES_PER_CYCLE 	32
#define CYCLES_PER_BUFFER	16	// 16
#define BUFF_SIZE			(sizeof( uint32_t) * 2 * SAMPLES_PER_CYCLE * CYCLES_PER_BUFFER)
uint32_t log_SineWave[ SAMPLES_PER_CYCLE ] = { 0 };
uint32_t sineWave    [ SAMPLES_PER_CYCLE ] = { 0 };

static i2s_chan_handle_t tx_chan;        // I2S tx channel handler

#define FIXED_POINT_SCALE ((1u<<31)-1) // 2^31 for a 32:32 fixed-point format


static void initialize_logSineTable( void ) {
	int32_t	littleEndian32;
	double	x;
	
	// Precalculat sinewave & log sinewave
	for( int i = 0; i < SAMPLES_PER_CYCLE; i++ ) {
        x = sin( i * 2*M_PI / SAMPLES_PER_CYCLE );
        littleEndian32 = (int32_t)( x * (double)FIXED_POINT_SCALE);
        sineWave[ i ] = littleEndian32;
		
        // apply a log function and scale -1 to +1
        // x = (pow( 10.0, (x+1.0)/2.0 ) - 5.5) / 4.5;

        // 95% AM modulation - from 1 (100% (0dB) to -1 (1% (-20dB))
        // 0.1mA (-4.1dB) to 0.9ma (-25dB) (-20.9dB)
        // Output voltage from DAC 3V is processed to 0.1 mA & -3V to 0.9ma (0.9mA)

#define MIN_SIGNAL_RATIO      0.01  // AM from 1.0 x max to 0.01 x max
#define MAX_ATTN_dB         -20.00  // 20dB i.e. a ratio of 1:100

        // Scale sample on sinewave and offset to 0.01 to 1.0 for log (-2 to 0)
        x = (x * (1.0 - MIN_SIGNAL_RATIO) + (1.0 + MIN_SIGNAL_RATIO) ) / 2.0;
        // take to log and scale ( log(0.01) == -2 and log10(1.0) == 0 )
        x = log10( x ) / (MAX_ATTN_dB/10.0);
        // Scale and offset to between -1.0 and 1.0
        x = -2.0 * (x - 0.5);
        if (x > -0.5) x=-0.5;
        littleEndian32 = (int32_t)( x * (double)FIXED_POINT_SCALE);
        log_SineWave[ i ] = littleEndian32;
    }
}


static void I2S_WriteTask(void *args)
{
    uint8_t *w_buf = (uint8_t *)calloc(1, BUFF_SIZE);
    assert(w_buf); // Check if w_buf allocation success
	
	size_t w_bytes = BUFF_SIZE;

    // Load the buffer with the 32 bit sine wave samples (bigendian)
	
	for (int i = 0, phase = 0; i < BUFF_SIZE; i += (2 * sizeof( uint32_t))) {
		// The samples (32 bit) are interleaved LRLRLR...etc
		// The buffer holds 16 cycles
		memcpy( &w_buf[i], &log_SineWave[ phase ], sizeof( uint32_t) );
		memcpy( &w_buf[i] + sizeof( uint32_t), &sineWave[ phase ], sizeof( uint32_t) );
		// after each cycle, reset the phase and repeat until the buffer is full
		if( (++phase) >= SAMPLES_PER_CYCLE)
		    phase = 0;
	}

	// Preload buffers
	while(w_bytes == BUFF_SIZE) {
        // Here we load the target buffer repeatedly, until all the DMA buffers are preloaded
        ESP_ERROR_CHECK(i2s_channel_preload_data(tx_chan, w_buf, BUFF_SIZE, &w_bytes));
    }
	
    // Enable the TX channel
    ESP_ERROR_CHECK(i2s_channel_enable(tx_chan));
    while (1) {
        // Write i2s data
        if (i2s_channel_write( tx_chan, w_buf, BUFF_SIZE, &w_bytes, 1000) == ESP_OK) {
			if( w_bytes != BUFF_SIZE )
				printf("Short write: i2s write %d bytes\n", w_bytes);
        } else {
            printf("Write Task: i2s write failed\n");
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    free(w_buf);
    vTaskDelete(NULL);
}



static void initialize_I2S(void)
{
    i2s_chan_config_t tx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
	// The dma_frame_number is set to 10 times the number of samples in a cycle
	// dma_buffer_size = dma_frame_num * slot_num * slot_bit_width / 8 (so dma buffer size is 240 x 2 x 32 / 8 = 1920)
	// The default dma_frame_number causes glitching with our signal
	tx_chan_cfg.dma_frame_num = SAMPLES_PER_CYCLE * 10;
/*	i2s_chan_config_t tx_chan_cfg =	{   
		.id                   = I2S_NUM_AUTO, 
		.role                 = I2S_ROLE_MASTER, 
		.dma_desc_num         = 6,
	 	.dma_frame_num        = 240, 
		.auto_clear_after_cb  = 0, 
		.auto_clear_before_cb = 0,
	 	.allow_pd             = 0, 
		.intr_priority        = 0,
	}; 	
*/
    ESP_ERROR_CHECK(i2s_new_channel(&tx_chan_cfg, &tx_chan, NULL));

    i2s_std_config_t tx_std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(32000),
/*		.clk_cfg  = { 	
			.sample_rate_hz = 32000, 
			.clk_src        = I2S_CLK_SRC_DEFAULT,
			.mclk_multiple  = I2S_MCLK_MULTIPLE_256,
		}, 
*/
#define PHILIPS
#ifdef  PHILIPS
		.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
#else
		.slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
/* 		.slot_cfg = {	
			.data_bit_width = I2S_DATA_BIT_WIDTH_32BIT,
			.slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
			.slot_mode      = I2S_SLOT_MODE_STEREO,
			.slot_mask      = I2S_STD_SLOT_BOTH,
			.ws_width       = I2S_DATA_BIT_WIDTH_32BIT, 
			.ws_pol 		= 0, 
			.bit_shift 		= 0,
			.msb_right      = 0,
		}, 
*/
#endif // PHILIPS
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,    // some codecs may require mclk signal, this example doesn't need it
			.bclk = PCM5102A_BCK,		// IO 22
			.ws   = PCM5102A_WR,		// IO 5
			.dout = PCM5102A_DOUT,		// IO 21
            .din  = PCM5102A_DIN,	    //
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_chan, &tx_std_cfg));
}


void app_main(void)
{
    initialize_I2S();

    initialize_logSineTable( );
    xTaskCreate(I2S_WriteTask, "I2S_WriteTask", 4096, NULL, 5, NULL);
}
