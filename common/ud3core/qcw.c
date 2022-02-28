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

//-----------------------------------------------------------------------------
// This file contains the code to configure the qcw ramp and to translate the
// ramp into a modulation of the phase shift of the pwmb component.  By shifting
// the phase, the duty cycle can be modulated by the ramp allowing the power
// to be adjusted during an interrupter burst.
//-----------------------------------------------------------------------------

#include <math.h>
#include <stdlib.h>
#include "qcw.h"

#include "ZCDtoPWM.h"
#include "tasks/tsk_overlay.h"
#include "tasks/tsk_midi.h"

typedef struct
{
    uint32_t time_start;
    uint32_t time_stop;
    uint32_t last_time;
} timer_params;
static timer_params timer;

// Each pixel of the ramp corresponds to 125 us horizontally
ramp_params volatile ramp;

static TimerHandle_t xQCW_Timer;       // Timer to repeat qcw pulses

// Called by the isr_synth interrupt at 8 kHz
void qcw_handle(){
    if(SG_Timer_ReadCounter() < timer.time_stop){
        // The pulse has run for the specified time.  shut it down
        qcw_modulate(0);
        qcw_reg = 0;
        QCW_enable_Control = 0;
        return;
    }
    
    // Update the modulation and continue
    if (ramp.index < sizeof(ramp.data)) {
        qcw_modulate(ramp.data[ramp.index]);
        ramp.index++;
	}
}

void qcw_handle_synth(){
    if(SG_Timer_ReadCounter() < timer.time_stop){
        QCW_enable_Control = 0;
        return;
    }
}

// Regenerates a linear map using the specified starting point, slope, and max value.
// Discards any changes made with the ramp command (using "ramp line" and "ramp point")
void qcw_regenerate_ramp(){
    if(ramp.changed){
        // Convert max_qcw_pw to ramp pixels (one X unit = 125 us)
        uint32_t max_index = (configuration.max_qcw_pw*10)/MIDI_ISR_US;   
        if (max_index > sizeof(ramp.data))
           max_index = sizeof(ramp.data);

        float ramp_val = param.qcw_offset;      // vertical offset at start
        float ramp_increment = param.qcw_ramp / 100.0;  // qcw_ramp is slope * 100, so ramp_increment is the slope of the line.
        for(uint16_t i=0; i<max_index; i++){
            if(ramp_val > param.qcw_max)
               ramp_val = param.qcw_max;
            ramp.data[i] = floor(ramp_val);
            if(i>param.qcw_holdoff) 
               ramp_val += ramp_increment;
        }
        ramp.changed = pdFALSE;
    }
}

// Sets a single pixel at x,y in ramp.data[].
void qcw_ramp_point(uint16_t x,uint8_t y){
    if(x<sizeof(ramp.data)){
        ramp.data[x] = y;
    }
}

// Uses a DDA to draw a line from x0, y0 to x1, y1 in ramp.data[].
void qcw_ramp_line(uint16_t x0,uint8_t y0,uint16_t x1, uint8_t y1){
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
	int dy = abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
	int err = (dx > dy ? dx : -dy) / 2, e2;
    
	for (;;) {
		qcw_ramp_point(x0, y0);
		if (x0 == x1 && y0 == y1)
			break;
		e2 = err;
		if (e2 > -dx) {
			err -= dy;
			x0 += sx;
		}
		if (e2 < dy) {
			err += dx;
			y0 += sy;
		}
	}
}

// Draws the ramp as an XY plot on the terminal
void qcw_ramp_visualize(CHART *chart, TERMINAL_HANDLE * handle){
    // Y increases down.  x0, y0 is the origin at the lower left corner
    uint16_t x0 = chart->offset_x;
    uint16_t y0 = chart->height + chart->offset_y;
    for(uint16_t i = 0; i<sizeof(ramp.data)-1; i++){
        send_chart_line(x0+i, y0-ramp.data[i], x0+i+1, y0-ramp.data[i+1], TT_COLOR_GREEN, handle);
    }
    
    // Draw a vertical red line at the max qcw pulse width.  max_qcw_pw is in ms*100.  
    // Multiplying by 10 yields us, and dividing by MIDI_ISR_US (125) gives the time 
    // in inrements of 125 us (pixels in the chart).
    uint16_t red_line = configuration.max_qcw_pw*10 / MIDI_ISR_US;
    send_chart_line(x0+red_line, y0-chart->height, x0+red_line, y0, TT_COLOR_RED, handle);
}

// Sets the timer start and stop values for the next pulse (in SG_Timer clocks)
void qcw_start(){
    timer.time_start = SG_Timer_ReadCounter();
    uint32_t sg_period = SG_PERIOD_NS;
    uint32_t cycles_to_stop = (configuration.max_qcw_pw*10000) / sg_period; // max_qcw_pw is in ms*100, so *10000 = ns
    uint32_t cycles_since_last_pulse = timer.time_stop - timer.time_start;
    
    // max_qcw_duty is in % * 10, so:
    // duty / 10 = onTime * 100 / (onTime + offTime)
    // rearranging gives:
    // onTime = offTime * duty / (1000 - duty)
    // TODO: Bug??? I don't understand where the next equation is coming from.  Seems like it should be:
    // uint32_t cycles_to_stop_limited = (cycles_since_last_pulse * configuration.max_qcw_duty) / (1000 - configuration.max_qcw_duty);
    // Assume duty is set to 35% and there have been 10 off cycles.  The original code will yield an on time of 10*350/500=7.  The
    // equation I propose will yield 10*350/(1000-350)=5.38.  But note that 7*100/(7+10)=41% while mine yields 5.38*100/(5.38+10)=35% (correct)
    // The original equation is correct only when the duty cycle equals 50% (so 1000-500 == 500).
    // Beware - my new equation will divide by 0 when duty=1000.  In that case the duty cycle is 100% and there is no off time.
    uint32_t cycles_to_stop_limited = (cycles_since_last_pulse * configuration.max_qcw_duty) / 500;
    
	if ((cycles_to_stop_limited > cycles_to_stop) || (cycles_to_stop_limited == 0)) {
		cycles_to_stop_limited = cycles_to_stop;
	}

    timer.time_stop = timer.time_start - cycles_to_stop_limited;
    ramp.index=0;
    
	//the next stuff is time sensitive, so disable interrupts to avoid glitches
	CyGlobalIntDisable;
	//now enable the QCW interrupter
	QCW_enable_Control = 1;
	params.pwmb_psb_val = params.pwm_top - params.pwmb_start_psb_val;   // Set phase shift to small value (full power) to get oscillation going.
	CyGlobalIntEnable;
}

// Modulate the output by shifting the phase of the HT pulses.  val can be 1 to 255 where
// 1 results in maximum shift (low power), and 256 results in minimum shift (full power).
void qcw_modulate(uint16_t val){
#if USE_DEBUG_DAC  
    if(QCW_enable_Control) DEBUG_DAC_SetValue(val); 
#endif
    //linearize modulation value based on fb_filter_out period
    // val=1->minimum power, val=255->maximum power
	uint16_t shift_period = (val * (params.pwm_top - fb_filter_out)) >> 8;
    shift_period += params.pwmb_start_psb_val;
    
	// assign new modulation value to the params.pwmb_psb_val ram (which is transferred to the pwmb cmp register by DMA).
	if (shift_period > (params.pwmb_start_prd - 4)) {
		params.pwmb_psb_val = 4;    // Maximum period, minimum shift, maximum power
	} else {
		params.pwmb_psb_val = params.pwm_top - shift_period;
	}  
}

void qcw_stop(){
    qcw_reg = 0;
#if USE_DEBUG_DAC 
    DEBUG_DAC_SetValue(0);
#endif
    QCW_enable_Control = 0;
}

// Called by the CLI when a ramp-related param is changed
uint8_t callback_rampFunction(parameter_entry * params, uint8_t index, TERMINAL_HANDLE * handle){
    ramp.changed = pdTRUE;
    if(!QCW_enable_Control){
        qcw_regenerate_ramp();
    }
    
    return pdPASS;
}

// Allows the user to edit the ramp by "drawing" lines and points
uint8_t CMD_ramp(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){
    if(argCount==0 || strcmp(args[0], "-?") == 0){
        ttprintf(   "Usage: ramp line x1 y1 x2 y2\r\n"
                    "       ramp point x y\r\n"
                    "       ramp clear\r\n"
                    "       ramp draw\r\n");
        return TERM_CMD_EXIT_SUCCESS;
    } 
  
    if(strcmp(args[0], "point") == 0 && argCount == 3){
        int x = atoi(args[1]);
        int y = atoi(args[2]);
        qcw_ramp_point(x,y);
        return TERM_CMD_EXIT_SUCCESS;
        
    } else if(strcmp(args[0], "line") == 0 && argCount == 5){
        int x0 = atoi(args[1]);
        int y0 = atoi(args[2]);
        int x1 = atoi(args[3]);
        int y1 = atoi(args[4]);
        qcw_ramp_line(x0,y0,x1,y1);
        return TERM_CMD_EXIT_SUCCESS;
        
    } else if(strcmp(args[0], "clear") == 0){
        for(uint16_t i = 0; i<sizeof(ramp.data);i++){
            ramp.data[i] = 0;
        }
        return TERM_CMD_EXIT_SUCCESS;
    } else if(strcmp(args[0], "draw") == 0){
        // TODO: What happens when this is done on a normal TTY (not teslaterm)?
        // Draw the current ramp on the terminal
        tsk_overlay_chart_stop();
        send_chart_clear(handle);
        CHART temp;
        temp.height = RAMP_CHART_HEIGHT;
        temp.width = RAMP_CHART_WIDTH;
        temp.offset_x = RAMP_CHART_OFFSET_X;
        temp.offset_y = RAMP_CHART_OFFSET_Y;
        temp.div_x = RAMP_CHART_DIV_X;
        temp.div_y = RAMP_CHART_DIV_Y;
        
        tt_chart_init(&temp, handle);
        qcw_ramp_visualize(&temp, handle);
        return TERM_CMD_EXIT_SUCCESS;
    }
     return TERM_CMD_EXIT_SUCCESS;
}

/*****************************************************************************
* Timer callback for the QCW autofire.  This is called back every param.qcw_repeat ms
******************************************************************************/
void vQCW_Timer_Callback(TimerHandle_t xTimer){
    qcw_regenerate_ramp();
    qcw_start();
    qcw_reg = 1;
    if(param.qcw_repeat<100)    // TODO: Not sure why this is here.  <100 is single shot mode and this timer is not started so how can this happen?
       param.qcw_repeat = 100;
    xTimerChangePeriod( xTimer, param.qcw_repeat / portTICK_PERIOD_MS, 0 );
}

BaseType_t QCW_delete_timer(void){
    if (xQCW_Timer != NULL) {
    	if(xTimerDelete(xQCW_Timer, 200 / portTICK_PERIOD_MS) != pdFALSE){
            xQCW_Timer = NULL;
            return pdPASS;
        }else{
            return pdFAIL;
        }
    }else{
        return pdFAIL;
    }
}

/*****************************************************************************
* starts QCW mode. Spawns a timer for the automatic QCW pulses.
******************************************************************************/
uint8_t CMD_qcw(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){
    if(argCount==0 || strcmp(args[0], "-?") == 0){
        ttprintf("Usage: qcw [start|stop]\r\n");
        return TERM_CMD_EXIT_SUCCESS;
    }
        
	if(strcmp(args[0], "start") == 0){
        SigGen_switch_synth(SYNTH_MIDI);
        
        // <100 is single shot mode.  If > 99 then set up a time to repeat the pulse at the specified interval.
        if(param.qcw_repeat > 99){
            if(xQCW_Timer==NULL){
                xQCW_Timer = xTimerCreate("QCW-Tmr", param.qcw_repeat / portTICK_PERIOD_MS, pdFALSE,(void * ) 0, vQCW_Timer_Callback);
                if(xQCW_Timer != NULL){
                    xTimerStart(xQCW_Timer, 0);
                    ttprintf("QCW Enabled\r\n");
                }else{
                    ttprintf("Cannot create QCW Timer\r\n");
                }
            }
        }else{
            // Here for single shot mode
            qcw_regenerate_ramp();
		    qcw_start();
            ttprintf("QCW single shot\r\n");
        }
		
		return TERM_CMD_EXIT_SUCCESS;
	}
    
	if(strcmp(args[0], "stop") == 0){
        if (xQCW_Timer != NULL) {
				if(!QCW_delete_timer()){
                    ttprintf("Cannot delete QCW Timer\r\n");
                }
		}
        qcw_stop();
		ttprintf("QCW Disabled\r\n");
        SigGen_switch_synth(param.synth);
		return TERM_CMD_EXIT_SUCCESS;
	}
    
	return TERM_CMD_EXIT_SUCCESS;
}