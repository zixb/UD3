/*
 * UD3
 *
 * Copyright (c) 2018 Jens Kerrinnes
 * Copyright (c) 2015 Steve Ward
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/*
The internal_interrupter enables the ZCDtoPWM hardware.  
Its a 16 bit PWM clocked at 1MHz, so thats 1uS per count.

*/
#ifndef INTERRUPTER_H
#define INTERRUPTER_H

#include <device.h>
#include "cli_basic.h"
#include "timers.h"

#define MAX_VOL (128<<16)
#define MIN_VOL 0
    
enum interrupter_DMA{
    INTR_DMA_TR,        // transient mode
    INTR_DMA_DDS        // synth mode
};

enum interrupter_mode{
    INTR_MODE_OFF=0,
    INTR_MODE_TR,       // transient mode
    INTR_MODE_BURST,    // burst mode
//    INTR_MODE_BLOCKED   // TODO: not used???
};

// The current state of burst mode
enum interrupter_burst{
    BURST_ON,
    BURST_OFF
};

enum interrupter_modulation{
    INTR_MOD_PW=0,                      // The pulse widths are scaled down (modulated) by volume
    INTR_MOD_CUR=1                      // The pulse width is left at full value and the bridge current is modulated by volume
};

typedef struct
{
	uint16_t pw;                            // pulse width in usec
	uint16_t prd;                           // period in usec
    enum interrupter_mode mode;
    enum interrupter_burst burst_state;     // Used by the burst timer callback to know if burst is on or off (toggles on every timer tick)
    enum interrupter_modulation mod;
    TimerHandle_t xBurst_Timer;             // The timer used for the bon and boff times in burst mode
} interrupter_params;
extern interrupter_params interrupter;

void interrupter_kill(void);
void interrupter_unkill(void);
void initialize_interrupter(void);
void configure_interrupter();
void update_interrupter();
void ramp_control(void);
void interrupter_oneshot(uint32_t pw, uint32_t vol);
void interrupter_update_ext();
void interrupter_set_pw_vol(uint8_t ch, uint16_t pw, uint32_t vol);
void interrupter_DMA_mode(uint8_t mode);

uint8_t callback_ext_interrupter(parameter_entry * params, uint8_t index, TERMINAL_HANDLE * handle);
uint8_t callback_BurstFunction(parameter_entry * params, uint8_t index, TERMINAL_HANDLE * handle);
uint8_t callback_TRFunction(parameter_entry * params, uint8_t index, TERMINAL_HANDLE * handle);
uint8_t callback_TRPFunction(parameter_entry * params, uint8_t index, TERMINAL_HANDLE * handle);
uint8_t callback_interrupter_mod(parameter_entry * params, uint8_t index, TERMINAL_HANDLE * handle);

#endif
