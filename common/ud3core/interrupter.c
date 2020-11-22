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

#include "interrupter.h"
#include "hardware.h"
#include "ZCDtoPWM.h"
#include "autotune.h"
#include "cli_common.h"
#include "qcw.h"
#include "tasks/tsk_fault.h"

#include <device.h>
#include <math.h>

uint16 int1_prd, int1_cmp;


uint16 ch1_prd, ch1_cmp;
uint16 ch2_prd, ch2_cmp;

uint8_t tr_running = 0;


//CY_ISR(OCD_ISR)
//{
//    //QCW_enable_Control = 0;
//}


void interrupter_kill(void){
    sysfault.interlock = 1;
    tr_running=0;
    interrupter.pw =0;
    param.pw=0;
    update_interrupter();
}

void interrupter_unkill(void){
    sysfault.interlock=0;
}

void initialize_interrupter(void) {

	//over current detection
	//    OCD_StartEx(OCD_ISR);

	//initialize the PWM generators for safe PW and PRD
	interrupter1_WritePeriod(65000);
	interrupter1_WriteCompare2(65000);
	interrupter1_WriteCompare1(64999);

	int1_prd = 65000;
	int1_cmp = 64999;

	//Start up timers
	interrupter1_Start();

	params.min_tr_prd = INTERRUPTER_CLK_FREQ / configuration.max_tr_prf;
	/* Variable declarations for int1_dma */
/*	uint8 int1_dma_Chan;
	uint8 int1_dma_TD[4];
	int1_dma_Chan = int1_dma_DmaInitialize(int1_dma_BYTES_PER_BURST, int1_dma_REQUEST_PER_BURST,
										   HI16(int1_dma_SRC_BASE), HI16(int1_dma_DST_BASE));
	int1_dma_TD[0] = CyDmaTdAllocate();
	int1_dma_TD[1] = CyDmaTdAllocate();
	int1_dma_TD[2] = CyDmaTdAllocate();
	int1_dma_TD[3] = CyDmaTdAllocate();
	CyDmaTdSetConfiguration(int1_dma_TD[0], 2, int1_dma_TD[1], int1_dma__TD_TERMOUT_EN | TD_AUTO_EXEC_NEXT);
	CyDmaTdSetConfiguration(int1_dma_TD[1], 2, int1_dma_TD[2], int1_dma__TD_TERMOUT_EN | TD_AUTO_EXEC_NEXT);
	CyDmaTdSetConfiguration(int1_dma_TD[2], 2, int1_dma_TD[3], int1_dma__TD_TERMOUT_EN | TD_AUTO_EXEC_NEXT);
	CyDmaTdSetConfiguration(int1_dma_TD[3], 2, int1_dma_TD[0], int1_dma__TD_TERMOUT_EN);
	CyDmaTdSetAddress(int1_dma_TD[0], LO16((uint32)&int1_prd), LO16((uint32)interrupter1_PERIOD_LSB_PTR));
	CyDmaTdSetAddress(int1_dma_TD[1], LO16((uint32)&int1_cmp), LO16((uint32)interrupter1_COMPARE1_LSB_PTR));
	CyDmaTdSetAddress(int1_dma_TD[2], LO16((uint32)&int1_prd), LO16((uint32)interrupter1_COMPARE2_LSB_PTR));
	CyDmaTdSetAddress(int1_dma_TD[3], LO16((uint32)&int1_prd), LO16((uint32)interrupter1_COUNTER_LSB_PTR));
	CyDmaChSetInitialTd(int1_dma_Chan, int1_dma_TD[0]);
	CyDmaChEnable(int1_dma_Chan, 1);
    
   */
    uint8 ch1_dma_Chan;
	uint8 ch1_dma_TD[4];
	ch1_dma_Chan = Ch1_DMA_DmaInitialize(int1_dma_BYTES_PER_BURST, int1_dma_REQUEST_PER_BURST,
										   HI16(int1_dma_SRC_BASE), HI16(int1_dma_DST_BASE));
	ch1_dma_TD[0] = CyDmaTdAllocate();
	ch1_dma_TD[1] = CyDmaTdAllocate();
	ch1_dma_TD[2] = CyDmaTdAllocate();
	ch1_dma_TD[3] = CyDmaTdAllocate();
	CyDmaTdSetConfiguration(ch1_dma_TD[0], 2, ch1_dma_TD[1], Ch1_DMA__TD_TERMOUT_EN | TD_AUTO_EXEC_NEXT);
	CyDmaTdSetConfiguration(ch1_dma_TD[1], 2, ch1_dma_TD[2], Ch1_DMA__TD_TERMOUT_EN | TD_AUTO_EXEC_NEXT);
	CyDmaTdSetConfiguration(ch1_dma_TD[2], 2, ch1_dma_TD[3], Ch1_DMA__TD_TERMOUT_EN | TD_AUTO_EXEC_NEXT);
	CyDmaTdSetConfiguration(ch1_dma_TD[3], 2, ch1_dma_TD[0], Ch1_DMA__TD_TERMOUT_EN);
	CyDmaTdSetAddress(ch1_dma_TD[0], LO16((uint32)&ch1_prd), LO16((uint32)interrupter1_PERIOD_LSB_PTR));
	CyDmaTdSetAddress(ch1_dma_TD[1], LO16((uint32)&ch1_cmp), LO16((uint32)interrupter1_COMPARE1_LSB_PTR));
	CyDmaTdSetAddress(ch1_dma_TD[2], LO16((uint32)&ch1_prd), LO16((uint32)interrupter1_COMPARE2_LSB_PTR));
	CyDmaTdSetAddress(ch1_dma_TD[3], LO16((uint32)&ch1_prd), LO16((uint32)interrupter1_COUNTER_LSB_PTR));
	CyDmaChSetInitialTd(ch1_dma_Chan, ch1_dma_TD[0]);
	CyDmaChEnable(ch1_dma_Chan, 1);
    
    uint8 ch2_dma_Chan;
	uint8 ch2_dma_TD[4];
	ch2_dma_Chan = Ch2_DMA_DmaInitialize(int1_dma_BYTES_PER_BURST, int1_dma_REQUEST_PER_BURST,
										   HI16(int1_dma_SRC_BASE), HI16(int1_dma_DST_BASE));
	ch2_dma_TD[0] = CyDmaTdAllocate();
	ch2_dma_TD[1] = CyDmaTdAllocate();
	ch2_dma_TD[2] = CyDmaTdAllocate();
	ch2_dma_TD[3] = CyDmaTdAllocate();
	CyDmaTdSetConfiguration(ch2_dma_TD[0], 2, ch2_dma_TD[1], Ch2_DMA__TD_TERMOUT_EN | TD_AUTO_EXEC_NEXT);
	CyDmaTdSetConfiguration(ch2_dma_TD[1], 2, ch2_dma_TD[2], Ch2_DMA__TD_TERMOUT_EN | TD_AUTO_EXEC_NEXT);
	CyDmaTdSetConfiguration(ch2_dma_TD[2], 2, ch2_dma_TD[3], Ch2_DMA__TD_TERMOUT_EN | TD_AUTO_EXEC_NEXT);
	CyDmaTdSetConfiguration(ch2_dma_TD[3], 2, ch2_dma_TD[0], Ch2_DMA__TD_TERMOUT_EN);
	CyDmaTdSetAddress(ch2_dma_TD[0], LO16((uint32)&ch2_prd), LO16((uint32)interrupter1_PERIOD_LSB_PTR));
	CyDmaTdSetAddress(ch2_dma_TD[1], LO16((uint32)&ch2_cmp), LO16((uint32)interrupter1_COMPARE1_LSB_PTR));
	CyDmaTdSetAddress(ch2_dma_TD[2], LO16((uint32)&ch2_prd), LO16((uint32)interrupter1_COMPARE2_LSB_PTR));
	CyDmaTdSetAddress(ch2_dma_TD[3], LO16((uint32)&ch2_prd), LO16((uint32)interrupter1_COUNTER_LSB_PTR));
	CyDmaChSetInitialTd(ch2_dma_Chan, ch2_dma_TD[0]);
	CyDmaChEnable(ch2_dma_Chan, 1);

   
    DDS32_1_Init();
    DDS32_2_Init();
    
}





void interrupter_oneshot(uint32_t pw, uint8_t vol) {
    if(sysfault.interlock) return;
    
	if (vol < 128) {
		ct1_dac_val[0] = params.min_tr_cl_dac_val + ((vol * params.diff_tr_cl_dac_val) >> 7);
	} else {
		ct1_dac_val[0] = params.max_tr_cl_dac_val;
	}
	uint16_t prd;
	if (pw > configuration.max_tr_pw) {
		pw = configuration.max_tr_pw;
	}
    prd = param.offtime + pw;
	/* Update Interrupter PWMs with new period/pw */
	CyGlobalIntDisable;
	int1_prd = prd - 3;
	int1_cmp = prd - pw - 3;
	interrupter1_control_Control = 0b0001;
	interrupter1_control_Control = 0b0000;
    CyGlobalIntEnable;
}


void interrupter_update_ext() {

	ct1_dac_val[0] = params.max_tr_cl_dac_val;
    uint32_t pw = param.pw;
    
    if (pw > configuration.max_tr_pw) {
		pw = configuration.max_tr_pw;
	}

    uint16_t prd = param.offtime + pw;
	/* Update Interrupter PWMs with new period/pw */
	CyGlobalIntDisable;
	int1_prd = prd - 3;
	int1_cmp = prd - pw - 3;
    
    if(configuration.ext_interrupter==1){
	    interrupter1_control_Control = 0b1100;
    }else{
        interrupter1_control_Control = 0b0100;
    }
    CyGlobalIntEnable;
}

uint8_t callback_ext_interrupter(parameter_entry * params, uint8_t index, TERMINAL_HANDLE * handle){
    if(configuration.ext_interrupter){
        alarm_push(ALM_PRIO_WARN,warn_interrupter_ext, configuration.ext_interrupter);
        interrupter_update_ext();
    }else{
        interrupter1_control_Control = 0b0000;
    }
    return pdPASS;
}

void update_interrupter() {
    
	/* Check if PW = 0, this indicates that the interrupter should be shut off */
	if (interrupter.pw == 0 || sysfault.interlock) {
		interrupter1_control_Control = 0b0000;
        return;
	}
    uint16_t limited_pw;
    
	/* Check the pulsewidth command */
	if (interrupter.pw > configuration.max_tr_pw) {
		interrupter.pw = configuration.max_tr_pw;
	}

	/* Check that the period is long enough to meet restrictions, if not, scale it */
	if (interrupter.prd < params.min_tr_prd) {
		interrupter.prd = params.min_tr_prd;
	}
	/* Compute the duty cycle and mod the PW if required */
	limited_pw = (uint32)((uint32)interrupter.pw * 1000ul) / interrupter.prd; //gives duty cycle as 0.1% increment
	if (limited_pw > configuration.max_tr_duty - param.temp_duty) {
		limited_pw = (uint32)(configuration.max_tr_duty - param.temp_duty) * (uint32)interrupter.prd / 1000ul;
	} else {
		limited_pw = interrupter.pw;
	}
	ct1_dac_val[0] = params.max_tr_cl_dac_val;

	/* Update interrupter registers */
	CyGlobalIntDisable;
	if (interrupter.pw != 0) {
		int1_prd = interrupter.prd - 3;
		int1_cmp = interrupter.prd - limited_pw - 3;
		if (interrupter1_control_Control == 0) {
			interrupter1_control_Control = 0b0011;
			interrupter1_control_Control = 0b0010;
		}
	}
	CyGlobalIntEnable;
}

void update_interrupter_new(uint16_t pw1, uint16_t pw2) {
    
    //if(sysfault.interlock) return;
    

	if (pw1 > configuration.max_tr_pw) {
		pw1 = configuration.max_tr_pw;
	}
    if (pw2 > configuration.max_tr_pw) {
		pw2 = configuration.max_tr_pw;
	}
    uint16_t prd1 = param.offtime + pw1;
    uint16_t prd2 = param.offtime + pw2;
	/* Update Interrupter PWMs with new period/pw */
	CyGlobalIntDisable;
	ch1_prd = prd1 - 3;
	ch1_cmp = prd1 - pw1 - 3;
    
    ch2_prd = prd2 - 3;
	ch2_cmp = prd2 - pw2 - 3;
	//interrupter1_control_Control = 0b0001;
	//interrupter1_control_Control = 0b0000;
    CyGlobalIntEnable;
}
