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
// The interrupter controls how long resonant pulses are allowed to cycle within
// the bridge, and how long to wait before allowing the next set of pulses.
// The frequency that the interrupter turns on/off defines the audible pitch 
// of the sparks.  The resonant pulses themselves are much too fast to be heard.
//
// The width of the interrupter pulses can be scaled down (modulated) by a volume
// parameter (synth mode with pulse width modulation).  Alternatively, the pulse
// width can be left at full width and the volume parameter can be used to scale 
// the maximum current in the bridge (synth mode with current modulation).
//-----------------------------------------------------------------------------
#include "interrupter.h"
#include "ZCDtoPWM.h"
#include "SignalGenerator.h"
#include "qcw.h"
#include "tasks/tsk_fault.h"

#include <device.h>
#include <math.h>
#include <stdbool.h>

// TODO: Make this global...
//inline int min(int a, int b) { return a < b? a: b; }
#define MIN(a, b) ({ __typeof__ (a) _a = (a);  __typeof__ (b) _b = (b); _a < _b ? _a : _b; })
#define MAX(a, b) ({ __typeof__ (a) _a = (a);  __typeof__ (b) _b = (b); _a > _b ? _a : _b; })

#define MODULATION_CUR_BYTES 1
#define MODULATION_PW_BYTES  8
#define MODULATION_REQUEST_PER_BURST  1

/* DMA Configuration for int1_dma */
#define int1_dma_BYTES_PER_BURST 8
#define int1_dma_REQUEST_PER_BURST 1
#define int1_dma_SRC_BASE (CYDEV_SRAM_BASE)
#define int1_dma_DST_BASE (CYDEV_PERIPH_BASE)

#define INTERRUPTER_CLK_FREQ 1000000

interrupter_params interrupter;

uint16_t int1_prd, int1_cmp;                    // connected via DMA to interrupter1 component for transient mode
uint8 int1_dma_Chan;                            // DMA channel to transfer int1_prd and int1_cmp to the interrupter1 component.

uint16_t ch_prd[N_CHANNEL], ch_cmp[N_CHANNEL];  // Connected via DMA to interrupter1 component for synth mode with pulse width modulation
uint8_t ch_cur[N_CHANNEL];                      // Connected via DMA to ct1_dac component for synth mode with current modulation

#define N_TD 4                                  // Number of TD's per channel
uint8 ch_dma_Chan[N_CHANNEL];                   // DMA channels for synth mode
uint8_t ch_dma_TD[N_CHANNEL][N_TD];             // Contains the TD's for each synth mode channel

void interrupter_kill(void){
    sysfault.interlock = 1;
    interrupter.mode = INTR_MODE_OFF;
    interrupter.pw =0;
    param.pw=0;
    update_interrupter();
}

void interrupter_unkill(void){
    sysfault.interlock=0;
}

// Configures the DMA channels for the specified modulation method.  Modulation refers to scaling down
// the output according to a volume parameter.  The output can be scaled by decreasing the pulse width
// or by decreasing the maximum current in the bridge.
void interrupter_reconf_dma(enum interrupter_modulation mod){
    if(mod==INTR_MOD_PW){
        // Here for pulse width modulation
        ch_dma_Chan[0] = Ch1_DMA_DmaInitialize(MODULATION_PW_BYTES, MODULATION_REQUEST_PER_BURST, HI16(CYDEV_SRAM_BASE), HI16(CYDEV_PERIPH_BASE));
        ch_dma_Chan[1] = Ch2_DMA_DmaInitialize(MODULATION_PW_BYTES, MODULATION_REQUEST_PER_BURST, HI16(CYDEV_SRAM_BASE), HI16(CYDEV_PERIPH_BASE));
        ch_dma_Chan[2] = Ch3_DMA_DmaInitialize(MODULATION_PW_BYTES, MODULATION_REQUEST_PER_BURST, HI16(CYDEV_SRAM_BASE), HI16(CYDEV_PERIPH_BASE));
        ch_dma_Chan[3] = Ch4_DMA_DmaInitialize(MODULATION_PW_BYTES, MODULATION_REQUEST_PER_BURST, HI16(CYDEV_SRAM_BASE), HI16(CYDEV_PERIPH_BASE));
        
        for(uint8_t ch=0;ch<N_CHANNEL;ch++){
            CyDmaTdSetConfiguration(ch_dma_TD[ch][0], 2, ch_dma_TD[ch][1], TD_AUTO_EXEC_NEXT);
    	    CyDmaTdSetConfiguration(ch_dma_TD[ch][1], 2, ch_dma_TD[ch][2], TD_AUTO_EXEC_NEXT);
	        CyDmaTdSetConfiguration(ch_dma_TD[ch][2], 2, ch_dma_TD[ch][3], TD_AUTO_EXEC_NEXT);
	        CyDmaTdSetConfiguration(ch_dma_TD[ch][3], 2, ch_dma_TD[ch][0], Ch1_DMA__TD_TERMOUT_EN);  
        
            CyDmaTdSetAddress(ch_dma_TD[ch][0], LO16((uint32)&ch_prd[ch]), LO16((uint32)interrupter1_PERIOD_LSB_PTR));
        	CyDmaTdSetAddress(ch_dma_TD[ch][1], LO16((uint32)&ch_cmp[ch]), LO16((uint32)interrupter1_COMPARE1_LSB_PTR));
    	    CyDmaTdSetAddress(ch_dma_TD[ch][2], LO16((uint32)&ch_prd[ch]), LO16((uint32)interrupter1_COMPARE2_LSB_PTR));
        	CyDmaTdSetAddress(ch_dma_TD[ch][3], LO16((uint32)&ch_prd[ch]), LO16((uint32)interrupter1_COUNTER_LSB_PTR));
        
            CyDmaChSetInitialTd(ch_dma_Chan[ch], ch_dma_TD[ch][0]);
        }
    }else if(mod==INTR_MOD_CUR){
        // Here for current modulation.
        ch_dma_Chan[0] = Ch1_DMA_DmaInitialize(MODULATION_CUR_BYTES, MODULATION_REQUEST_PER_BURST, HI16(CYDEV_SRAM_BASE), HI16(CYDEV_SRAM_BASE));
        ch_dma_Chan[1] = Ch2_DMA_DmaInitialize(MODULATION_CUR_BYTES, MODULATION_REQUEST_PER_BURST, HI16(CYDEV_SRAM_BASE), HI16(CYDEV_SRAM_BASE));
        ch_dma_Chan[2] = Ch3_DMA_DmaInitialize(MODULATION_CUR_BYTES, MODULATION_REQUEST_PER_BURST, HI16(CYDEV_SRAM_BASE), HI16(CYDEV_SRAM_BASE));
        ch_dma_Chan[3] = Ch4_DMA_DmaInitialize(MODULATION_CUR_BYTES, MODULATION_REQUEST_PER_BURST, HI16(CYDEV_SRAM_BASE), HI16(CYDEV_SRAM_BASE));
    
        for(uint8_t ch=0;ch<N_CHANNEL;ch++){
            CyDmaTdSetConfiguration(ch_dma_TD[ch][0], 1, ch_dma_TD[ch][0], Ch1_DMA__TD_TERMOUT_EN);
            CyDmaTdSetAddress(ch_dma_TD[ch][0], LO16((uint32)&ch_cur[ch]), LO16((uint32)&ct1_dac_val[0]));
            CyDmaChSetInitialTd(ch_dma_Chan[ch], ch_dma_TD[ch][0]);
        }
    }
}

// Adds an alarm with the specified message to the alarm queue and stops the UD3
// TODO: Make this a global function so it can be used everywhere resources and memory are allocated.
void critical_error(const char *message)
{
    alarm_push(ALM_PRIO_CRITICAL, message, 0);
    interrupter_kill();
}

// One-time initialization of the interrupter components (called once at system startup)
void initialize_interrupter(void) {
	//over current detection
	//    OCD_StartEx(OCD_ISR);

	//initialize the PWM generators for safe PW and PRD
	int1_prd = 65000;   // An arbitrary large number to count down from
	int1_cmp = 64999;
    
	interrupter1_WritePeriod(int1_prd);
	interrupter1_WriteCompare1(int1_cmp);      // pwm1 will be true for one clock, then false.  int1_prd - int1_cmp is the on time for the pulse.
  	interrupter1_WriteCompare2(int1_prd);      // pwm2 will be false for one clock then true.  This creates the int1_fin signal.

	//Start up timers
	interrupter1_Start();
    
	// Variable declarations for int1_dma.  This transfers int1_prd and int1_cmp to the interrupter1 component.
	uint8_t int1_dma_TD[4];
	int1_dma_Chan = int1_dma_DmaInitialize(int1_dma_BYTES_PER_BURST, int1_dma_REQUEST_PER_BURST, HI16(int1_dma_SRC_BASE), HI16(int1_dma_DST_BASE));
    
    for(int i=0; i<4; ++i){
    	int1_dma_TD[i] = CyDmaTdAllocate();
        if(int1_dma_TD[i] == DMA_INVALID_TD)
            critical_error("CyDmaTdAllocate failure");
    }
    
	CyDmaTdSetConfiguration(int1_dma_TD[0], 2, int1_dma_TD[1], int1_dma__TD_TERMOUT_EN | TD_AUTO_EXEC_NEXT);
	CyDmaTdSetConfiguration(int1_dma_TD[1], 2, int1_dma_TD[2], int1_dma__TD_TERMOUT_EN | TD_AUTO_EXEC_NEXT);
	CyDmaTdSetConfiguration(int1_dma_TD[2], 2, int1_dma_TD[3], int1_dma__TD_TERMOUT_EN | TD_AUTO_EXEC_NEXT);
	CyDmaTdSetConfiguration(int1_dma_TD[3], 2, int1_dma_TD[0], int1_dma__TD_TERMOUT_EN);
    
	CyDmaTdSetAddress(int1_dma_TD[0], LO16((uint32)&int1_prd), LO16((uint32)interrupter1_PERIOD_LSB_PTR));
	CyDmaTdSetAddress(int1_dma_TD[1], LO16((uint32)&int1_cmp), LO16((uint32)interrupter1_COMPARE1_LSB_PTR));
	CyDmaTdSetAddress(int1_dma_TD[2], LO16((uint32)&int1_prd), LO16((uint32)interrupter1_COMPARE2_LSB_PTR));
	CyDmaTdSetAddress(int1_dma_TD[3], LO16((uint32)&int1_prd), LO16((uint32)interrupter1_COUNTER_LSB_PTR));
	CyDmaChSetInitialTd(int1_dma_Chan, int1_dma_TD[0]);
    
    //Allocate memory for DDS DMA TDs
    for(uint8_t ch=0;ch<N_CHANNEL;ch++){
        for(uint8_t i=0;i<N_TD;i++){
            ch_dma_TD[ch][i] = CyDmaTdAllocate();
            if(ch_dma_TD[ch][i] ==  DMA_INVALID_TD)
                critical_error("CyDmaTdAllocate failure");
        }
    }

    DDS32_1_Start();
    DDS32_2_Start();

    configure_interrupter();
}

// Called whenever an interrupter-related param is changed or the eeprom is loaded.
void configure_interrupter()
{
    // Safe defaults in case anything fails.
  	int1_prd = 65000;
	int1_cmp = 64999;

  	//initialize the PWM generators for safe PW and PRD
	interrupter1_WritePeriod(int1_prd);
	interrupter1_WriteCompare1(int1_cmp);      // pwm1 will be true for one clock, then false.  int1_prd - int1_cmp is the on time for the pulse.
  	interrupter1_WriteCompare2(int1_prd);      // pwm2 will be false for one clock then true

    // The minimum interrupter period for transient mode in clock ticks (which are equal to microseconds here).
	params.min_tr_prd = INTERRUPTER_CLK_FREQ / configuration.max_tr_prf;
    
    // Disable interrupter
    interrupter.mode = INTR_MODE_OFF;
    interrupter_reconf_dma(interrupter.mod);    // May be changed when eeprom is loaded or interrupter.mod is changed via cli
   
    DDS32_1_Disable_ch(0);
    DDS32_1_Disable_ch(1);
    DDS32_2_Disable_ch(0);
    DDS32_2_Disable_ch(1);
    
    interrupter_DMA_mode(INTR_DMA_DDS);         // set to synth mode by default
}

// Enables/disables the DMA channels for transient or synth mode
void interrupter_DMA_mode(uint8_t mode){
    switch(mode){
        case INTR_DMA_TR:       // transient mode: disable the 4 DDS channels, enable the int1_dma chan
            for(int i=0; i<4; ++i)
                CyDmaChDisable(ch_dma_Chan[i]);
            CyDmaChEnable(int1_dma_Chan, 1);
        break;
            
        case INTR_DMA_DDS:      // synth or sid mode
            if(interrupter.mod == INTR_MOD_PW){
                CyDmaChDisable(int1_dma_Chan);
            }else{
                CyDmaChEnable(int1_dma_Chan, 1);
            }
            for(int i=0; i<4; ++i)
                CyDmaChEnable(ch_dma_Chan[i], 1);
        break;
    }
}

// Not used
//void interrupter_set_pw(uint8_t ch, uint16_t pw){
//    if(ch<N_CHANNEL){
//        ct1_dac_val[0] = params.max_tr_cl_dac_val;
//        uint16_t prd = param.offtime + pw;
//        ch_prd[ch] = prd - 3;
//        ch_cmp[ch] = prd - pw - 3; 
//    }
//}

// This modulates (scales) the pulse width by the specified volume.  Only used with synth mode.
// o pw is the pulse width in us
// o vol is the volume in 7.16 fixed point (valid range is 0-128).  128 is full volume = full 
//   pulse width, so the final pulse width = pw * vol/128.
void interrupter_set_pw_vol(uint8_t ch, uint16_t pw, uint32_t vol){
    if(ch<N_CHANNEL){
        if(pw>configuration.max_tr_pw) 
            pw = configuration.max_tr_pw;
        
        if(interrupter.mod == INTR_MOD_PW){
            if(vol > MAX_VOL)
               vol = MAX_VOL;            
            // Scale pw by vol using fixed point.  vol is in 8.16 format, so shifting right by 6
            // (to avoid overflow in the multiply) results in 8.10 format.  Shifting right 17 
            // more results in 1.0 (no fixed point).  Another way to think of this is VOL_MAX
            // which equals (1 << 23) will become 1 after shifting right by 6 + 17.
            uint32_t scaled_pw = MIN(((uint32_t)pw*(vol>>6))>>17, configuration.max_tr_pw); 
            uint16_t prd = param.offtime + scaled_pw;
            ch_prd[ch] = prd - 3;
            ch_cmp[ch] = prd - scaled_pw - 3; 
        }else{
            // Here for INTR_MOD_CUR (current modulation).  The maximum volume uses the max dac value (params.max_tr_cl_dac_val)
            // Any other volume scales the max down to the min using linear interpolation.  Note that MAX_VOL = 1 << 23
            if (vol < MAX_VOL) {
        		ch_cur[ch] = params.min_tr_cl_dac_val + ((vol * params.diff_tr_cl_dac_val) >> 23);
                // Clamp to max just in case.
                if(ch_cur[ch] > params.max_tr_cl_dac_val) 
                   ch_cur[ch] = params.max_tr_cl_dac_val;
        	} else {
        		ch_cur[ch] = params.max_tr_cl_dac_val;  // Just use the max allowed
        	}
            uint16_t prd = param.offtime + pw;
            int1_prd = prd - 3;
	        int1_cmp = prd - pw - 3;
        }
    }
}

// Used by AutoTune
void interrupter_oneshot(uint32_t pw, uint32_t vol) {
    if(sysfault.interlock) return;
    
	if (vol < MAX_VOL) {
		ct1_dac_val[0] = params.min_tr_cl_dac_val + ((vol * params.diff_tr_cl_dac_val) >> 23);
        if(ct1_dac_val[0] > params.max_tr_cl_dac_val) ct1_dac_val[0] = params.max_tr_cl_dac_val;
	} else {
		ct1_dac_val[0] = params.max_tr_cl_dac_val;
	}

    if (pw > configuration.max_tr_pw) {
		pw = configuration.max_tr_pw;
	}
    uint16_t prd = param.offtime + pw;
    
	/* Update Interrupter PWMs with new period/pw */
	CyGlobalIntDisable;
	int1_prd = prd - 3;
	int1_cmp = prd - pw - 3;
	interrupter1_control_Control = 0b0001;      // Force an immendiate reset
	interrupter1_control_Control = 0b0000;      // One pulse only
    CyGlobalIntEnable;
}

// Update the interrupter when the external interrupter status changes.  This is only called 
// when configuration.ext_interrupter==1 or 2 (inverted) - never 0 (off).
// Should probably test for 0 and return if so just in case.
void interrupter_update_ext() {

	ct1_dac_val[0] = params.max_tr_cl_dac_val;
    uint16_t pw = MIN(param.pw, configuration.max_tr_pw);
    uint16_t prd = param.offtime + pw;
    
	/* Update Interrupter PWMs with new period/pw */
	CyGlobalIntDisable;
	int1_prd = prd - 3;         // DS: period is total length of the pulse in usec.  I don't know why they subtract 3 here.
	int1_cmp = prd - pw - 3;    // cmp is the "on" time of the pulse in usec.
    
    // TODO: Bug?  I don't think the following is correct.  According to the parameter help, configuration.ext_interrupter
    // can be 0 for none, 1 for normal external interrupter, and 2 for inverted external interrupter mode.  Control bit 4
    // (0b1000) appears to indicate inverted operation according to the hardware schematic (it is xor'ed with the external
    // interrupter signal.  So why is the code below setting it to 1 for the NORMAL case, and 0 for the INVERTED case?
    // Jens told Intra on the forum that the external interupter was a work in progress...
    if(configuration.ext_interrupter==1){
        // Here for normal external interrupter mode
        // 3rd bit is 1 to enable external trigger
        // 4th bit is 1 to invert the external trigger.
	    interrupter1_control_Control = 0b1100;
    }else{
        // Here for inverted external interrupter mode
        // 3rd bit is 1 to enable external trigger
        // 4th bit is 1 to invert the external trigger.
        interrupter1_control_Control = 0b0100;      
    }
    CyGlobalIntEnable;
}

// Called when the "ena_ext_int" parameter is changed by the user (which updates configuration.ext_interrupter)
uint8_t callback_ext_interrupter(parameter_entry * params, uint8_t index, TERMINAL_HANDLE * handle){
    if(configuration.ext_interrupter){
        // Here if external interrupter has been enabled
        alarm_push(ALM_PRIO_WARN,warn_interrupter_ext, configuration.ext_interrupter);
        interrupter_update_ext();
    }else{
        // Here if external interrupter has been disabled
        uint8 sfflag = system_fault_Read();
        system_fault_Control = 0; //halt tesla coil operation during updates!
        interrupter1_control_Control = 0b0000;
        system_fault_Control = sfflag;
    }
    
    return pdPASS;
}

// called when interrupter.mod is changed.  Description says "0=pw 1=current modulation"
uint8_t callback_interrupter_mod(parameter_entry * params, uint8_t index, TERMINAL_HANDLE * handle){
    uint8 sfflag = system_fault_Read();
    system_fault_Control = 0; //halt tesla coil operation during updates!
    
    interrupter_reconf_dma(interrupter.mod);
    interrupter_DMA_mode(interrupter.mode!=INTR_MODE_OFF? INTR_DMA_TR: INTR_DMA_DDS);
 
    system_fault_Control = sfflag;
    return pdPASS;
}

// Updates the global int1_prd and int1_cmp vars which are connected via DMA to the int1 component.
// Should be called if any of the following are changed:
//    interrupter.pw
//    interrupter.prd
//    configuration.max_tr_pw
//    params.min_tr_prd
void update_interrupter() {

    /* Check if PW = 0, this indicates that the interrupter should be shut off */
	if (interrupter.pw == 0 || sysfault.interlock) {
		interrupter1_control_Control = 0b0000;
        return;
	}
    
	/* Check the pulsewidth command */
	if (interrupter.pw > configuration.max_tr_pw) {
		interrupter.pw = configuration.max_tr_pw;
	}

	/* Check that the period is long enough to meet restrictions, if not, scale it */
	if (interrupter.prd < params.min_tr_prd) {
		interrupter.prd = params.min_tr_prd;
	}
	/* Compute the duty cycle and mod the PW if required */
    uint16_t limited_pw = ((uint32_t)interrupter.pw * 1000ul) / interrupter.prd; //gives duty cycle as 0.1% increment
	if (limited_pw > configuration.max_tr_duty - param.temp_duty) {
		limited_pw = (uint32_t)(configuration.max_tr_duty - param.temp_duty) * (uint32_t)interrupter.prd / 1000ul;
	} else {
		limited_pw = interrupter.pw;
	}
	ct1_dac_val[0] = params.max_tr_cl_dac_val;
    
	/* Update interrupter registers */
	CyGlobalIntDisable;
	if (interrupter.pw != 0) {
		int1_prd = interrupter.prd - 3;                 // The total period of the pulse in usec.  Not sure where the -3 comes from
		int1_cmp = interrupter.prd - limited_pw - 3;    // The "on" time of the pulse
        
        // Force the interrupter1 component to update
		if (interrupter1_control_Control == 0) {
			interrupter1_control_Control = 0b0011;      // Reset counter right away
			interrupter1_control_Control = 0b0010;      // Keep running the PWM generator (reloads counter after each PWM cycle)
		}
	}
	CyGlobalIntEnable;
}

/*****************************************************************************
* Callback if a transient mode parameter is changed (param.pw or param.pwd)
* Updates the interrupter hardware
******************************************************************************/
uint8_t callback_TRFunction(parameter_entry * params, uint8_t index, TERMINAL_HANDLE * handle) {

    // Keep the pulse width percent in sync with the pulse width.
    // pw is the pulse width specified in microseconds.  pwp is the pulse width percent * 10.
    // configuration.max_tr_pw is the maximum transient mode pulse width in microseconds.
    // So the 1000 multiplier is 100 (to convert from proportion to percent) * 10 because 
    // the result is percent * 10 (fixed point).
    param.pwp = (1000 * param.pw) / configuration.max_tr_pw;
    
    switch(interrupter.mode){
        case INTR_MODE_OFF:
        break;
        
        case INTR_MODE_TR:
        	interrupter.pw = param.pw;
        	interrupter.prd = param.pwd;    // TODO: Bug? Hmm...  notice this assigns pwd (whose description 
                                            // says "pulsewidthdelay" to prd which is the pulse width period.
                                            // perhaps the description should read "pulse width period"?
            update_interrupter();
        break;
            
        default:
            if(configuration.ext_interrupter  && param.synth == SYNTH_OFF){
                interrupter_update_ext();
            }else if (param.synth == SYNTH_MIDI || param.synth == SYNTH_MIDI_QCW){
                SigGen_limit();
            }
        break; 
    }

	return pdPASS;
}

/*****************************************************************************
* Callback if a transient mode parameter is changed (percent ontime)
* Updates the interrupter hardware
******************************************************************************/
uint8_t callback_TRPFunction(parameter_entry * params, uint8_t index, TERMINAL_HANDLE * handle) {

    // The pulse width percent was changed.  Keep the pulse width in sync.
    param.pw = (configuration.max_tr_pw * param.pwp) / 1000;
    interrupter.pw = param.pw;
    
    if (param.synth == SYNTH_MIDI || param.synth == SYNTH_MIDI_QCW)
        SigGen_limit();
    
	if (interrupter.mode!=INTR_MODE_OFF)
		update_interrupter();

    if(configuration.ext_interrupter)
        interrupter_update_ext();
    
	return pdPASS;
}

/*****************************************************************************
* Timer callback for burst mode.  This uses the normal transient mode param.pw
* for the interrupter (which sets the pitch of the sound).  This will be done 
* for "bon" time, and then wait for the specified "boff" time before repeating.
******************************************************************************/
void vBurst_Timer_Callback(TimerHandle_t xTimer){
    if(interrupter.burst_state == BURST_ON){
        // Here if burst is on and the timer has expired.  Turn burst off and wait the specified time.
        interrupter.pw = 0;
        update_interrupter();
        interrupter.burst_state = BURST_OFF;
        uint16_t boff_lim = param.burst_off? param.burst_off: 2;
        xTimerChangePeriod( xTimer, boff_lim / portTICK_PERIOD_MS, 0 );
    }else{
        // Here if burst is off.  Turn the burst on and set the timer for the specified on time.
        interrupter.pw = param.pw;
        update_interrupter();
        interrupter.burst_state = BURST_ON;
        uint16_t bon_lim = param.burst_on? param.burst_on: 2;
        xTimerChangePeriod( xTimer, bon_lim / portTICK_PERIOD_MS, 0 );
    }
}

// Deletes the burst timer (if any).  Returns true if successful.
bool delete_burst_timer(TERMINAL_HANDLE * handle)
{
    if(!interrupter.xBurst_Timer)
        return true;
    
    if(!xTimerDelete(interrupter.xBurst_Timer, 200 / portTICK_PERIOD_MS))
    {
        ttprintf("Cannot delete burst Timer\r\n");
        return false;
    }

    interrupter.xBurst_Timer = NULL;
    return true;
}

/*****************************************************************************
* Callback if a burst mode parameter is changed
******************************************************************************/
uint8_t callback_BurstFunction(parameter_entry * params, uint8_t index, TERMINAL_HANDLE * handle) {
    if(interrupter.mode!=INTR_MODE_OFF){
        if(interrupter.xBurst_Timer==NULL && param.burst_on > 0){
            // Turn on burst timer

            // TODO: Curious this starts with the state set to BURST_ON.  When the 
            // timer expires it will toggle the state and turn it off.  End result 
            // is this will wait for 2 timeouts before finally starting burst mode.
            // Seems like this should set to BURST_OFF to only wait for 1 timeout.
            // Either that or actually enable the interrupter here instead of waiting
            // for the timer callback to do it.
            interrupter.burst_state = BURST_ON;
            
            interrupter.xBurst_Timer = xTimerCreate("Burst-Tmr", param.burst_on / portTICK_PERIOD_MS, pdFALSE,(void * ) 0, vBurst_Timer_Callback);
            if(interrupter.xBurst_Timer != NULL){
                xTimerStart(interrupter.xBurst_Timer, 0);
                ttprintf("Burst Enabled\r\n");
                interrupter.mode=INTR_MODE_BURST;
            }else{
                interrupter.pw = 0;
                ttprintf("Cannot create burst Timer\r\n");
                interrupter.mode=INTR_MODE_OFF;
            }
        }else if(interrupter.xBurst_Timer!=NULL && (param.burst_on==0 || param.burst_off==0)){
            // Turn off burst timer
            if(delete_burst_timer(handle)) {
                interrupter.pw = param.burst_on==0? 0: param.pw;    // TODO: Why is burst_on==0 treated differently than burst_off==0?
                                                                    // Seems like both should turn off the interrupter until restarted by user.
                update_interrupter();
                interrupter.mode=INTR_MODE_TR;
                ttprintf("\r\nBurst Disabled\r\n");
            }
        }
    }
	return pdPASS;
}

/*****************************************************************************
* starts or stops the transient mode (classic mode)
* also spawns a timer for the burst mode.
******************************************************************************/
uint8_t CMD_tr(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args) {
    
    if(argCount==0 || strcmp(args[0], "-?") == 0){
        ttprintf("Transient [start/stop]");
        return TERM_CMD_EXIT_SUCCESS;
    }
    
    if(strcmp(args[0], "start") == 0){
        interrupter_DMA_mode(INTR_DMA_TR);  // transient mode
        
        interrupter.pw = param.pw;          // Pulse width
		interrupter.prd = param.pwd;        // Period
        update_interrupter();

		interrupter.mode=INTR_MODE_TR;
        callback_BurstFunction(NULL, 0, handle);    // Disable burst mode
		ttprintf("Transient Enabled\r\n");

        return TERM_CMD_EXIT_SUCCESS;
    }

	if(strcmp(args[0], "stop") == 0){
        SigGen_switch_synth(param.synth);
        if (interrupter.xBurst_Timer != NULL)
            delete_burst_timer(handle);
        
		interrupter.pw = 0;
		update_interrupter();
        if(configuration.ext_interrupter) 
            interrupter_update_ext();
		interrupter.mode=INTR_MODE_OFF;

        ttprintf("Transient Disabled\r\n");    
		return TERM_CMD_EXIT_SUCCESS;
	}
    return TERM_CMD_EXIT_SUCCESS;
}

