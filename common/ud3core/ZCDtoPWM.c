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

#include "ZCDtoPWM.h"
#include "cli_common.h"
#include "interrupter.h"
#include <device.h>
#include <math.h>

uint16_t fb_filter_in=1;
uint16_t fb_filter_out=1;

void initialize_ZCD_to_PWM(void) {
	//initialize all the timers/counters/PWMs
	PWMA_Start();
	PWMB_Start();
	FB_capture_Start();
	ZCD_counter_Start();
	FB_glitch_detect_Start();

	//initialize all comparators
	ZCD_compA_Start();
	ZCD_compB_Start();
	CT1_comp_Start();

	//initialize the DACs
	CT1_dac_Start();
	ZCDref_Start();

	//start the feedback filter block
	FB_Filter_Start();
	FB_Filter_SetCoherency(FB_Filter_CHANNEL_A, FB_Filter_KEY_MID);

	configure_ZCD_to_PWM();
}

void configure_CT1(void) {

	float max_tr_cl_dac_val_temp;
	float max_qcw_cl_dac_val_temp;
	float min_tr_cl_dac_val_temp;

	//figure out the CT setups
    // max_tr_current is the maximum transient mode current in amps (default=400)
    // ct1_ratio defaults to 600
    // burden defaults to 33 ohms*10.  This is the R41 3.3 ohm resistor.
    // DAC_VOLTS_PER_STEP = 0.016 = 4.08v / 255  (4.08v is the range of the 8 bit CT1_dac)
    // So the following line converts from amps on the primary to DAC counts
	max_tr_cl_dac_val_temp = (((float)configuration.max_tr_current / (float)configuration.ct1_ratio) * configuration.ct1_burden) / (DAC_VOLTS_PER_STEP * 10);
	if (max_tr_cl_dac_val_temp > 255) {
		max_tr_cl_dac_val_temp = 255;
	}
    params.max_tr_cl_dac_val = round(max_tr_cl_dac_val_temp);

    // Same thing for QCW
	max_qcw_cl_dac_val_temp = (((float)configuration.max_qcw_current / (float)configuration.ct1_ratio) * configuration.ct1_burden) / (DAC_VOLTS_PER_STEP * 10);
	if (max_qcw_cl_dac_val_temp > 255) {
		max_qcw_cl_dac_val_temp = 255;
	}

    // Same thing for minimum tr current (only used for MIDI modulation)
	min_tr_cl_dac_val_temp = (((float)configuration.min_tr_current / (float)configuration.ct1_ratio) * configuration.ct1_burden) / (DAC_VOLTS_PER_STEP * 10);
	if (min_tr_cl_dac_val_temp > 255) {
		min_tr_cl_dac_val_temp = 255;
	}
	params.min_tr_cl_dac_val = round(min_tr_cl_dac_val_temp);

    // The ctl_dac_val array is transferred via DMA from TR1_CL_DMA / QCW_CL_DMA to CT1_DAC
	ct1_dac_val[0] = params.max_tr_cl_dac_val;
	ct1_dac_val[1] = params.max_tr_cl_dac_val;
	ct1_dac_val[2] = round(max_qcw_cl_dac_val_temp);

    // The range of DAC values for the primary current
	params.diff_tr_cl_dac_val = params.max_tr_cl_dac_val - params.min_tr_cl_dac_val;
}

void configure_CT2(void) {
    if(configuration.ct2_type==CT2_TYPE_CURRENT){
        //DC CT mA_Count
        // magic numbers:
        // input current (in amps) = measured voltage (in volts) * ratio / burden (in ohms)
        // measured current (in ma) = DAC_counts * 5v/4096 dac counts * 1000
        // burden = configuration.ct2_burden / 10  (because the burden resistance is stored as ohms * 10)
        // which gives measured current (in ma) = DAC_counts * 5 * 10 * 1000 / configuration.ct2_burden / 4096
        params.idc_ma_count = (uint32_t)((configuration.ct2_ratio * 50 * 1000) / configuration.ct2_burden) / 4096;
    }else{
        // configuration.ct2_offset and configuration.ct2_voltage are in millivolts
        // configuration.ct2_current is in amps * 10
        params.ct2_offset_cnt = (uint32_t)(4096ul*(uint32_t)configuration.ct2_offset)/5000ul;           // DS: offset in dac counts
        uint32_t cnt_fs = ((4096ul*(uint32_t)configuration.ct2_voltage)/5000ul)-params.ct2_offset_cnt;  // voltage-offset in dac counts
        params.idc_ma_count = (((uint32_t)configuration.ct2_current*100ul) / cnt_fs);                   // current in ma / dac units
    }
}

void configure_ZCD_to_PWM(void) {
	//this function calculates the variables used at run-time for the ZCD to PWM hardware
	//it also initializes that hardware so its ready for interrupter control

	float pwm_start_prd_temp;
    uint16 fb_glitch_cmp;

	configure_CT1();
	configure_CT2();

	//initialize the ZCD counter
    // This counts the number of feedback pulses to know when to switch from the 
    // internal oscillator to the feedback CT.  ZCD_counter is configured for 
    // "less than or equal" so output is true when period counter is
    // <= compare value.  HOWEVER, nothing is connected to the comp output so this
    // doesn't matter.  The switch_to_fb signal is connected to the TC (terminal
    // count) output, so when the "period" counts have been counted it will set TC
    // high.
	if (configuration.start_cycles == 0) {
		ZCD_counter_WritePeriod(1);     // Always require at least 1 fb pulse
		ZCD_counter_WriteCompare(1);    // TODO:  Needs review.  Doesn't matter?
	} else {
		ZCD_counter_WritePeriod(configuration.start_cycles * 2);    // * 2 because there are 2 zero crossings per cycle
        // TODO:  Needs review.  Not sure where this 4 came from.  Actually I don't think this even matters 
        // because there is nothing connected to the comp output.
		ZCD_counter_WriteCompare(4);    
	}
    
    uint32_t lead_time_temp;
	//calculate lead time in cycles
    // configuration.lead_time is in nSec.  The next line is equivalent to:
    // configuration.lead_time * 1sec/1e9 nSec * 64000000 clocks / 1 sec
    // configuration.lead_time * 64 / 1000
    // which equals configuration.lead_time in bus clock cycles
    // Note that this will truncate the specified lead_time down to the nearest multiple of 15.26 ns (one bus cycle)
	lead_time_temp = (configuration.lead_time*1000) / (1000000/BCLK__BUS_CLK__MHZ);

    // Steve Ward's firmware calculates this more directly using floating point:
    // lead_time_temp = (float)config.lead_time/CPU_CLK_PERIOD;
    // params.lead_time = round(lead_time_temp);
    // where  CPU_CLK_PERIOD is equal to 15.6325 (the number of ns in a clock).  Of course it still has the same bus clock granularity of 15.6325 ns.
    
	//calculate starting period
    // configuration.start_freq is the starting frequency in khz*10 (or hz/100).  This is used 
    // when first starting up before any feedback is available from CT1.
    // This is the starting period in bus (64mhz) clock cycles
    // f = configuration.start_freq * 100hz
    // p = (1 sec / f) * (BCLK__BUS_CLK__HZ/1 sec)
    // half period = p / 2
    // so we have half period in bus cycles = BCLK__BUS_CLK__HZ / (configuration.start_freq * 100hz * 2)
    // I don't understand why pwm_start_prd_temp was declared float.  The calculations are all done in integer and they won't exceed the size of an int...
	pwm_start_prd_temp = BCLK__BUS_CLK__HZ / (configuration.start_freq * 200); //why 200? well 2 because its half-periods, and 100 because frequency is in hz*100
    
    // *2 because pwm_start_prd_temp is half periods
	params.pwm_top = round(pwm_start_prd_temp * 2);								  //top value for FB capture and pwm generators to avoid ULF crap
	params.pwma_start_prd = params.pwm_top - lead_time_temp;
	params.pwma_start_cmp = params.pwm_top - pwm_start_prd_temp + 4; //untested, was just 4;
    
    // They call pwmb the "oscillator" below.  pwmb is set to "less than or equal" 
    // so it will output high if the period counter is <= compare value.  So this is going 
    // to output low until the counter reaches 4.
	params.pwmb_start_prd = round(pwm_start_prd_temp); //experimental start up mode, 2 cycles of high frequency to start things up but stay ahead of the game.
    
    // The width of the generated pulse (4 bus clocks) = 62.5 ns.
	params.pwmb_start_cmp = 4;
	params.pwmb_start_psb_val = 15;				   //config.psb_start_val;// params.pwmb_start_prd>>3;
												   //Set up FB_GLITCH PWM
	fb_glitch_cmp = pwm_start_prd_temp / 4; //set the lock out period to be 1/4 of a cycle long
	if (fb_glitch_cmp > 255) {
		fb_glitch_cmp = 255;
	}
	FB_glitch_detect_WritePeriod(255);
	FB_glitch_detect_WriteCompare1(255 - fb_glitch_cmp);
	FB_glitch_detect_WriteCompare2(255);        // TODO: Review this.  Steve Ward initializes this to 255-fb_glitch_cmp + 2  ??

	//initialize PWM generator
	PWMA_WritePeriod(params.pwma_start_prd);
	PWMA_WriteCompare(params.pwma_start_cmp);

	//start up oscillator
	PWMB_WritePeriod(params.pwmb_start_prd);
	PWMB_WriteCompare(params.pwmb_start_cmp);
    
	//initialize FB Capture period to account for 10 count offset in timing due to reset delay.
    // TODO: Review this.  I believe the "reset delay" mentioned above is the time it takes the ZCD_Pulse 
    // signal to propagate through the DMA stuff and back into the Timer reset input.  I measured 
    // the lag from the rising edge of the ZCD_pulse to the rising edge of the FB_Timer reset at
    // 190 ns.  In addition, the docs say the FB_Timer count won't start until 2 clocks after 
    // reset goes low.  Since the reset pulse is 40 ns wide, that brings the total lag here to:
    // (190 ns + 40 ns) / 15.625 + 2 = 16.7 clocks (instead of the 10 programmed below).  I think 
    // this may be why the lead_time is off by 6 clocks ???
	FB_capture_WritePeriod(params.pwm_top - 10);

	//prime the feedback filter with fake values so that the first time we use it, its not absolutely garbage
	for (uint8_t z = 0; z < 5; z++) {
		FB_Filter_Write24(FB_Filter_CHANNEL_A, params.pwm_top - round(pwm_start_prd_temp));
        CyDelayUs(4);
	}

	//set reference voltage for zero current detector comparators
	ZCDref_Data = 25;
}

/* [] END OF FILE */
