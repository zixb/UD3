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

#include "cyapicallbacks.h"
#include <cytypes.h>

#include "tsk_analog.h"
#include "tsk_fault.h"
#include "alarmevent.h"
#include "config.h"

/* RTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"
#include "interrupter.h"
#include "helper/FastPID.h"

#include <stdlib.h>
#include "math.h"


xTaskHandle tsk_analog_TaskHandle;
uint8 tsk_analog_initVar = 0u;

SemaphoreHandle_t adc_ready_Semaphore;

xQueueHandle adc_data;

TimerHandle_t xCharge_Timer;





/* ------------------------------------------------------------------------ */
/*
 * Place user included headers, defines and task global data in the
 * below merge region section.
 */
/* `#START USER_INCLUDE SECTION` */
#include "ZCDtoPWM.h"
#include "cli_common.h"
#include "telemetry.h"
#include "tsk_priority.h"
#include "tsk_midi.h"
#include <device.h>

/* Defines for MUX_DMA */
#define MUX_DMA_BYTES_PER_BURST 1
#define MUX_DMA_REQUEST_PER_BURST 1
#define MUX_DMA_SRC_BASE (CYDEV_SRAM_BASE)
#define MUX_DMA_DST_BASE (CYDEV_PERIPH_BASE)

/* DMA Configuration for ADC_DMA */
#define ADC_DMA_BYTES_PER_BURST 2
#define ADC_DMA_REQUEST_PER_BURST 1
#define ADC_DMA_SRC_BASE (CYDEV_PERIPH_BASE)
#define ADC_DMA_DST_BASE (CYDEV_SRAM_BASE)

#define INITIAL 0 /* Initial value of the filter memory. */

typedef struct
{
	uint16_t rms;
	uint64_t sum_squares;
} rms_t;


adc_sample_t ADC_sample_buf_0[ADC_BUFFER_CNT];
adc_sample_t ADC_sample_buf_1[ADC_BUFFER_CNT];
adc_sample_t *ADC_active_sample_buf = ADC_sample_buf_0;

FastPID pid_current;

rms_t current_idc;
uint8_t ADC_mux_ctl[4] = {0x05, 0x02, 0x03, 0x00};

uint16_t vdriver_lut[9] = {0,3500,7000,10430,13840,17310,20740,24200,27657};

typedef struct
{
    uint32_t limit;
    uint32_t warning;
    uint8_t warning_level;
    uint32_t leak;
    uint32_t integral;
} _i2t;

_i2t i2t;

void i2t_set_limit(uint32_t const_current, uint32_t ovr_current, uint32_t limit_ms){
    i2t.leak = const_current * const_current;
    i2t.limit= floor((float)((float)limit_ms/NEW_DATA_RATE_MS)*(float)((ovr_current*ovr_current)-i2t.leak));
    i2t_set_warning(i2t.warning_level);
}

void i2t_set_warning(uint8_t percent){
    i2t.warning_level = percent;
    i2t.warning = floor((float)i2t.limit*((float)percent/100.0));
}

void i2t_reset(){
    i2t.integral=0;
}

void i2t_init(){
    i2t.integral=0;
    i2t.warning=0;
    i2t.leak=0;
    i2t.warning_level=60;
}

uint8_t i2t_calculate(){
    uint32_t squaredCurrent = (uint32_t)tt.n.batt_i.value * (uint32_t)tt.n.batt_i.value;
    i2t.integral = i2t.integral + squaredCurrent;
	if(i2t.integral > i2t.leak)
	{
		i2t.integral -= i2t.leak;
	}
	else
	{
		i2t.integral = 0;
	}
    
    tt.n.i2t_i.value=(100*(i2t.integral>>8))/(i2t.limit>>8);
    
    if(i2t.integral < i2t.warning)
	{
		return I2T_NORMAL;
	}
	else
	{
		if(i2t.integral < i2t.limit)
		{
			return I2T_WARNING;
		}
		else
		{
            i2t.integral=i2t.limit;
			return I2T_LIMIT;
		}
	}
  
}

/* `#END` */
/* ------------------------------------------------------------------------ */
/*
 * User defined task-local code that is used to process commands, control
 * operations, and/or generrally do stuff to make the taks do something
 * meaningful.
 */
/* `#START USER_TASK_LOCAL_CODE` */

CY_ISR(ADC_data_ready_ISR) {
    if(ADC_active_sample_buf==ADC_sample_buf_0 ){
        ADC_active_sample_buf = ADC_sample_buf_1;
    } else {
        ADC_active_sample_buf = ADC_sample_buf_0;
    }
    // DS: Unblock the analog task
	xSemaphoreGiveFromISR(adc_ready_Semaphore, NULL);
}

// DS: configuration.r_top is the value of the external resistors connected to 
// the bus "voltage -" and "bus voltage +" pins (r_bus parameter).  By default
// this value is 500000.  BUSV_R_BOT is equal to 5000 which is equal to the value 
// of the feedback resistor (r32 for the bus voltage sensor, r39 for the battery 
// voltage sensor).
//
// In the code below, 819 equals 4095 ADC / 5V which is the conversion from volts to 
// ADC units.  Since the code below divides by 819, it is actually converting from 
// ADC to volts.  The 1000 is the conversion from volts to millivolts.  So the code
// below is equivalent to:
// float AdcToMv = 5.0f * 1000.0f / 4095.0f;
// bus_voltage = (configuration.r_top + BUSV_R_BOT) * raw_adc * AdcToMv / BUSV_R_BOT;
//
// Here is a standard differential amplifier circuit:
//             +----------------/\/\---+
//             |                 R3    |
//             |             |\        |
// v1---/\/\---+-------------|-\       |
//       R1                  |  \______|___vOut
//                           |  /
// v2---/\/\---+-------------|+/
//       R2    |             |/
//             |    R4
//             +---/\/\---+ Gnd
//
// And here is a simplified version of the one used on the UD3B (without the filtering caps)
//             +----------------/\/\---+
//             |                 R3    |
//             |             |\        |
// v1---/\/\---+-------------|-\       |
//       R1    |__/\/\__+Gnd |  \______|___vOut
//                 R5        |  /
// v2---/\/\---+-------------|+/
//       R2    |             |/
//             |    R4
//             +---/\/\---+ Gnd
//
// So it's basically the same circuit except for the addition of R5.  But note that r5 and 
// r3 form a voltage divider and the combination is equal to the value of R4.
//
// The equation for a differential op amp when r1=r2 (the external 500k isolation resistors) 
// and r3=r4 is:
// v2-v1 = vOut * r1 / r3
// 
// Therefore the final equation would be:
// bus_voltage = configuration.r_top * raw_adc * AdcToMv / BUSV_R_BOT;
//
// For some reason the code below adds BUSV_R_BOT to configuration.r_top and I don't know why.
// Note that this makes very little difference since BUSV_R_TOP is 100 times smaller, but I 
// wish I knew why he was doing this.
//
// Coyote on the forums says this is a differential HV probe with 100:1 attenuation.
//
// Steve Ward who initially designed this circuit (which remains unchanged as far
// as hardware goes) calculates the bus voltage like so:
// telemetry.bus_v = (ADC_sample[1]*OUTPUT_V_FS)>>12;
// OUTPUT_V_FS equals 500, which is the maximum voltage Steve assumes will be on the bus (output voltage full swing?).  
// The ADC has 12 bits of resolution, and maps the incoming voltage from VSSA to VDDA (0 to 5v).
// Steve compares telemetry.bus_v to config.PFC_PCV which has the comment "precharge voltage in volts"
// which implies that telemetry.bus_v is also in volts.
// Therefore, Steve's circuit must output a voltage from 0 to 5 volts.  The ADC maps this to the range 0 - 2^12.
// Assume 500v is present on the bus - Steve's circuit reduces it to 5v, the ADC maps it to 4096 (2^12).  
// Steve's equation will then be telemetry.bus_v = (4096 * 500) >> 12 = 500 so that all makes sense.
// This means that Steve's circuit outputs 0 to 5v for inputs of 0 to 500v.  It has a gain of .01 and
// therefore outputs millivolts.
//
// Steve has this to say in his documentation:
// VSNS: These inputs are “fully differential amplifier” circuits for measuring 
// voltages in the 0-500VDC range.  They require a 500K resistor on each wire to 
// “isolate” the UD3 from bus voltages.  The UD3 can have up to 500V common mode 
// voltage difference with respect to either +/- input, which makes it fine to 
// have a grounded UD monitoring a 240V line-fed DC bus.  For tesla gun use, the 
// Vin measures the battery supply voltage while Vbus measures the voltage on the 
// h-bridge for pre-charge functionality.
//
// A "fully differentiual amplifier" is defined as having a differential output in  
// addition to a differential input.  But in this case the output is relative to 
// ground which makes this a simple "differential amplifier".  I'm not sure why Steve 
// called it "fully differential".
//
// now consider the code below (from Netzpuscher) for the same case of 500v input:
// bus_voltage = ((500000 + 5000) * 4095) / (5000 * 819 / 1000) = 505000 (in millivolts).  This is off by 5v.
// but if BUSV_R_BOT is NOT added in the numerator we get:
// bus_voltage = (500000 * 4095) / (5000 * 819 / 1000) = 500000 (in millivolts).  
// This is exact.  In short, I don't understand why he is adding BUSV_R_BOT in the numerator.
//
// The "signals" command outputs the bus voltage using:
// ADC_CountsTo_mVolts(ADC_active_sample_buf[0].v_bus)
// and this displays "94 mv" when connected to a 9v battery outputting 9.25v according 
// to my multimeter 1 or 9.34v according to my multimeter 2.  ADC_CountsTo_mVolts is a
// generated function from PSoC Creator and is accurately reporting the actual millivolts
// on the pin to the ADC (I measured the output going to the the pin on the MCU and it 
// was 94.9mv).  So bottom line is the "signals" command is displaying the raw input to
// the ADC - not the bus voltage.
//
// TLDR:
// Assuming Steve's circuit functions as intended with a 100:1 attenuation.  If we apply
// 10v to the input pins, we should see 100mv at vOut.  Given 100mv at the ADC, the 
// equation below would give (500000 + 5000) * 100 / 5000 = 10100 = 10.1v which is NOT the expected 10.0v.
// The equation that I think should be used would give 500000 * 100 / 5000 = 10000 = 10.0v which is exact.
//
// I doubled the isolation resistors to support a 1000v bus, applied 10v to the input and
// measured 53.1mv on the output.    Using the code below we get (1000000 + 5000) * 53.1 / 5000 = 10673.1 = 10.673v
// using my equation, we get 1000000 * 53.1 / 5000 = 10620 = 10.620v.  Not much difference...
// 
uint32_t read_bus_mv(uint16_t raw_adc) {
	uint32_t bus_voltage;
    // DS: I could be wrong, but I don't think BUSV_R_BOT should be added to configuration.rtop 
    // on the next line.  The bus voltage is read by a differential amplifier circuit with .01 
    // gain.  If 10v is applied, it will output 10000mv, but the equation below would output:
    // (500000 + 5000) * 100 / 5000 = 10100 = 10.1v which is NOT the expected 10.0v.
    //
    // On the other hand, if we use the standard differential amplifier equation we have:
    // v2-v1 = vOut * r1 / r3
    // v2-v1 = 100 * 500000 / 5000 = 10000 = 10.0v which is exact.
    // The error isn't much so I'm going to leave this for now - I'm probably not understanding something.
	bus_voltage = ((configuration.r_top + BUSV_R_BOT) * raw_adc) / (BUSV_R_BOT * 819 / 1000);
	return bus_voltage;
}

uint16_t read_driver_mv(uint16_t raw_adc){
    uint32_t driver_voltage;
    uint8_t lut = raw_adc / 256;
    uint16_t span = vdriver_lut[lut+1]-vdriver_lut[lut];
    
    driver_voltage = vdriver_lut[lut]+(span * (raw_adc%256) / 256);
    
    return driver_voltage;
}

uint32_t CT1_Get_Current(uint8_t channel) {
    int32_t counts = ADC_peak_GetResult16();
	if (channel == CT_PRIMARY) {
		return ((ADC_peak_CountsTo_mVolts(counts) * configuration.ct1_ratio) / configuration.ct1_burden) / 100;
	} else {
		return ((ADC_peak_CountsTo_mVolts(counts) * configuration.ct3_ratio) / configuration.ct3_burden) / 100;
	}
}

float CT1_Get_Current_f(uint8_t channel) {
    int32_t counts = ADC_peak_GetResult16();
	if (channel == CT_PRIMARY) {
		return ((float)(ADC_peak_CountsTo_Volts(counts) * 10) / (float)(configuration.ct1_burden) * configuration.ct1_ratio);
	} else {
		return ((float)(ADC_peak_CountsTo_Volts(counts) * 10) / (float)(configuration.ct3_burden) * configuration.ct3_ratio);
	}
}

void init_rms_filter(rms_t *ptr, uint16_t init_val) {
	ptr->rms = init_val;
	ptr->sum_squares = 1UL * SAMPLES_COUNT * init_val * init_val;
}

uint16_t rms_filter(rms_t *ptr, uint16_t sample) {
	ptr->sum_squares -= ptr->sum_squares / SAMPLES_COUNT;
	ptr->sum_squares += (uint32_t)sample * sample;
	if (ptr->rms == 0)
		ptr->rms = 1; /* do not divide by zero */
	ptr->rms = (ptr->rms + ptr->sum_squares / SAMPLES_COUNT / ptr->rms) / 2;
	return ptr->rms;
}

void init_average_filter(uint32_t *ptr, uint16_t init_val) {
	*ptr = 1UL * SAMPLES_COUNT * init_val;
}

uint16_t average_filter(uint32_t *ptr, uint16_t sample) {
	*ptr -= *ptr / SAMPLES_COUNT;
	*ptr += (uint32_t)sample;
	*ptr = *ptr / SAMPLES_COUNT;
	return *ptr;
}

void calculate_rms(void) {
    
    static uint16_t old_curr_setpoint=0;
    for(uint8_t i=0;i<ADC_BUFFER_CNT;i++){

		// read the battery voltage
		tt.n.batt_v.value = read_bus_mv(ADC_active_sample_buf[i].v_batt) / 1000;

		// read the bus voltage
		tt.n.bus_v.value = read_bus_mv(ADC_active_sample_buf[i].v_bus) / 1000;

		// read the battery current
        if(configuration.ct2_type==CT2_TYPE_CURRENT){
		    tt.n.batt_i.value = (((uint32_t)rms_filter(&current_idc, ADC_active_sample_buf[i].i_bus) * params.idc_ma_count) / 100);
        }else{
            tt.n.batt_i.value = ((((int32_t)rms_filter(&current_idc, ADC_active_sample_buf[i].i_bus)-params.ct2_offset_cnt) * params.idc_ma_count) / 100);
        }

		tt.n.avg_power.value = tt.n.batt_i.value * tt.n.bus_v.value / 10;
	}
    
    // read the driver voltage
    tt.n.driver_v.value = read_driver_mv(ADC_active_sample_buf[0].v_driver);
    tt.n.primary_i.value = CT1_Get_Current(CT_PRIMARY);
    
    if(configuration.max_dc_curr){
        param.temp_duty = configuration.max_tr_duty-pid_step(&pid_current,configuration.max_dc_curr,tt.n.batt_i.value);
        if(param.temp_duty != old_curr_setpoint){
            if(interrupter.mode == INTR_MODE_TR){
                update_interrupter();
            }else{
                SigGen_limit();
            }
        }
        old_curr_setpoint = param.temp_duty;
    }
    
    if(configuration.max_const_i){  //Only do i2t calculation if enabled
        switch (i2t_calculate()){
            case I2T_LIMIT:
                sysfault.fuse=1;
                interrupter_kill();   
                break;
            case I2T_WARNING:
                sysfault.fuse=0;
                break;
            case I2T_NORMAL:
                sysfault.fuse=0;
                break;      
        }
    }
    
	control_precharge();

}



void initialize_analogs(void) {
	
	CT_MUX_Start();
    ADC_peak_Start();
	Sample_Hold_1_Start();
	Comp_1_Start();

	/* Variable declarations for ADC_DMA */
	/* Move these variable declarations to the top of the function */
	uint8 ADC_DMA_Chan;
	uint8 ADC_DMA_TD[2];

	ADC_DMA_Chan = ADC_DMA_DmaInitialize(ADC_DMA_BYTES_PER_BURST, ADC_DMA_REQUEST_PER_BURST, HI16(ADC_DMA_SRC_BASE), HI16(ADC_DMA_DST_BASE));
	ADC_DMA_TD[0] = CyDmaTdAllocate();
    ADC_DMA_TD[1] = CyDmaTdAllocate();
	CyDmaTdSetConfiguration(ADC_DMA_TD[0], 8*ADC_BUFFER_CNT, ADC_DMA_TD[1], ADC_DMA__TD_TERMOUT_EN | TD_INC_DST_ADR);
    CyDmaTdSetConfiguration(ADC_DMA_TD[1], 8*ADC_BUFFER_CNT, ADC_DMA_TD[0], ADC_DMA__TD_TERMOUT_EN | TD_INC_DST_ADR);
	CyDmaTdSetAddress(ADC_DMA_TD[0], LO16((uint32)ADC_SAR_WRK0_PTR), LO16((uint32)ADC_sample_buf_0));
    CyDmaTdSetAddress(ADC_DMA_TD[1], LO16((uint32)ADC_SAR_WRK0_PTR), LO16((uint32)ADC_sample_buf_1));
	CyDmaChSetInitialTd(ADC_DMA_Chan, ADC_DMA_TD[0]);
	CyDmaChEnable(ADC_DMA_Chan, 1);

	/* Variable declarations for MUX_DMA */
	/* Move these variable declarations to the top of the function */
	uint8 MUX_DMA_Chan;
	uint8 MUX_DMA_TD[1];

	/* DMA Configuration for MUX_DMA */
	MUX_DMA_Chan = MUX_DMA_DmaInitialize(MUX_DMA_BYTES_PER_BURST, MUX_DMA_REQUEST_PER_BURST, HI16(MUX_DMA_SRC_BASE), HI16(MUX_DMA_DST_BASE));
	MUX_DMA_TD[0] = CyDmaTdAllocate();
	CyDmaTdSetConfiguration(MUX_DMA_TD[0], 4, MUX_DMA_TD[0], CY_DMA_TD_INC_SRC_ADR);
	CyDmaTdSetAddress(MUX_DMA_TD[0], LO16((uint32)ADC_mux_ctl), LO16((uint32)Amux_Ctrl_Control_PTR));
	CyDmaChSetInitialTd(MUX_DMA_Chan, MUX_DMA_TD[0]);
	CyDmaChEnable(MUX_DMA_Chan, 1);
    

	ADC_data_ready_StartEx(ADC_data_ready_ISR);

	CT_MUX_Select(CT_PRIMARY);

	init_rms_filter(&current_idc, INITIAL);
    
    ADC_Start();
}

int16 initial_vbus, final_vbus, delta_vbus;
uint32 charging_counter;
uint8_t timer_triggerd=0;


void initialize_charging(void) {
	tt.n.bus_status.value = BUS_OFF;
	initial_vbus = 0;
	final_vbus = 0;
	charging_counter = 0;
}


// DS: This is called every time the analog task is run (which waits on adc_ready_Semaphore).  
// The ADC generates 100000 samples per second (I think).

void ac_precharge_bus_scheme(){
	//we cant know the AC line voltage so we will watch the bus voltage climb and infer when its charged by it not increasing fast enough
	//this logic is part of a charging counter
    if(tt.n.bus_status.value == BUS_READY && tt.n.bus_v.value <20){
        tt.n.bus_status.value=BUS_BATT_UV_FLT;
        if(sysfault.bus_uv==0){
            alarm_push(ALM_PRIO_ALARM,warn_bus_undervoltage,tt.n.bus_v.value);
        }
        sysfault.bus_uv=1;
        bus_command=BUS_COMMAND_FAULT;
    }
    // The charging_counter stuff runs this once every AC_PRECHARGE_TIMEOUT (100) times.
	if (charging_counter == 0)
		initial_vbus = tt.n.bus_v.value;
	charging_counter++;
	if (charging_counter > AC_PRECHARGE_TIMEOUT) {
		final_vbus = tt.n.bus_v.value;
		delta_vbus = final_vbus - initial_vbus;
        // DS: I verified that with a bus voltage of 18v, the charge_end relay is never triggered.  Apparently
        // the next line requires the voltage to be at least 20 volts.  If I set the bus to 24v then the precharge
        // relay turns on after the charge_delay timeout.
		if ((delta_vbus < 4) && (tt.n.bus_v.value > 20) && tt.n.bus_status.value == BUS_CHARGING) {
            // DS: So this waits until the bus isn't rising much anymore and _then_ waits for the charge_delay?
            // Since the bus isn't rising much anymore, then the caps must be charged.  Why also wait for the
            // charge_delay - which exists solely to provide time for the caps to charge?
            if(!timer_triggerd){
                timer_triggerd=1;
			    xTimerStart(xCharge_Timer,0);
            }
		} else if (tt.n.bus_status.value != BUS_READY) {
            if(tt.n.bus_status.value != BUS_CHARGING){
                alarm_push(ALM_PRIO_INFO,warn_bus_charging, ALM_NO_VALUE);
            }
            sysfault.charge=1;
			relay_write_bus(1);
			tt.n.bus_status.value = BUS_CHARGING;
		}
		charging_counter = 0;
	}
}

void ac_dual_meas_scheme(){
}

void ac_precharge_fixed_delay(){
    if(!relay_read_bus() && !relay_read_charge_end()){  // DS: bugfix: The if logic was inverted and the bus remained off forever
        alarm_push(ALM_PRIO_INFO,warn_bus_charging, ALM_NO_VALUE);
        sysfault.charge=1;
        xTimerStart(xCharge_Timer,0);
        relay_write_bus(1);
        tt.n.bus_status.value = BUS_CHARGING;
    }    
}

// DS: This is called after the precharge period has expired.  
void vCharge_Timer_Callback(TimerHandle_t xTimer){
    timer_triggerd=0;
    if(bus_command== BUS_COMMAND_ON){
        if(relay_read_bus()){
            alarm_push(ALM_PRIO_INFO,warn_bus_ready, ALM_NO_VALUE);
            relay_write_charge_end(1);      // DS: turn on the charge end relay to bypass the inrush precharge resistors
            tt.n.bus_status.value = BUS_READY;
            sysfault.charge=0;
            sysfault.bus_uv=0;
        }
    }else{
        relay_write_bus(0);
        relay_write_charge_end(0);  // DS: bugfix - this used to be relay_read_charge_end(0) which makes no sense...
        sysfault.charge=0;
        alarm_push(ALM_PRIO_INFO,warn_bus_off, ALM_NO_VALUE);
        tt.n.bus_status.value = BUS_OFF;
    }
}

void control_precharge(void) { //this gets called from tsk_analogs.c when the ADC data set is ready, 8khz rep rate

	if (bus_command == BUS_COMMAND_ON) {
        switch(configuration.ps_scheme){
            case BAT_PRECHARGE_BUS_SCHEME:
                //Not implemented
            break;
            case BAT_BOOST_BUS_SCHEME:
                //Not implemented
            break;
            case AC_PRECHARGE_BUS_SCHEME:
                ac_precharge_bus_scheme();
            break;
            case AC_DUAL_MEAS_SCHEME:
                ac_dual_meas_scheme();      // DS: Not implemented
            break;
            case AC_PRECHARGE_FIXED_DELAY:
                ac_precharge_fixed_delay();
            break;
        } 
	} else {
        // DS: Here if bus_command is BUS_COMMAND_OFF or BUS_COMMAND_FAULT.  I don't 
        // understand what the next code is doing.  It seems to be turning the bus
        // ON even through the user may have turned it off?  It appears to turn the bus
        // back on and starts the timer.  The timer code then turns the bus off when the
        // precharge timer expires.  Not sure why all this is necessary...
        // Consider: 
        //   1. bus is charged, so the bus and charge_end relays are both enabled.
        //   2. user enters the bus off command.  We end up here.
        //   3. The if statement is true because both relays are enabled
        //   4. This will turn the precharge relay off, keep the bus relay on and start the timer
        //   5. Timer expires and the bus is finally turned off - but only after waiting for the
        //      precharge time.
        // I checked this out with a multimeter and verified that the bus relay is not turned off
        // until the precharge timeout expires.
		if ((relay_read_charge_end() || relay_read_bus()) && timer_triggerd==0){
            sysfault.charge=1;
			relay_write_bus(1);
            relay_write_charge_end(0);
            timer_triggerd=1;
            xTimerStart(xCharge_Timer,0);
		}
	}
}

void reconfig_charge_timer(){
    if(xCharge_Timer!=NULL){
        xTimerChangePeriod(xCharge_Timer,configuration.chargedelay / portTICK_PERIOD_MS,0);
    }
}

uint8_t callback_pid(parameter_entry * params, uint8_t index, TERMINAL_HANDLE * handle){
    pid_new(&pid_current,configuration.pid_curr_p,configuration.pid_curr_i,0,CURRENT_PID_HZ,10,false);
    pid_set_anti_windup(&pid_current,0,configuration.max_tr_duty);
    pid_set_limits(&pid_current,0,configuration.max_tr_duty);
    if(pid_current._cfg_err==true){
        ttprintf("PID value error\r\n");   
    }
    return pdPASS;
}

/* `#END` */
/* ------------------------------------------------------------------------ */
/*
 * This is the main procedure that comprises the task.  Place the code required
 * to preform the desired function within the merge regions of the task procedure
 * to add functionality to the task.
 */
void tsk_analog_TaskProc(void *pvParameters) {
	/*
	 * Add and initialize local variables that are allocated on the Task stack
	 * the the section below.
	 */
	/* `#START TASK_VARIABLES` */

	//adc_data = xQueueCreate(128, sizeof(ADC_sample));
    
    // DS: configuration.chargedelay is the delay for the charge delay in ms.  Dividing by portTICK_PERIOD_MS
    // converts from ms to ticks.  Default charge timer delay is 1000 (1 sec).  
    xCharge_Timer = xTimerCreate("Chrg-Tmr", configuration.chargedelay / portTICK_PERIOD_MS, pdFALSE,(void * ) 0, vCharge_Timer_Callback);

    /* `#END` */

	/*
	 * Add the task initialzation code in the below merge region to be included
	 * in the task.
	 */
	/* `#START TASK_INIT_CODE` */
    
	adc_ready_Semaphore = xSemaphoreCreateBinary();
    
	initialize_analogs();
    
    CyGlobalIntEnable;

    
    alarm_push(ALM_PRIO_INFO,warn_task_analog, ALM_NO_VALUE);

	/* `#END` */

	for (;;) {
		/* `#START TASK_LOOP_CODE` */
        // DS: This blocks until adc_ready_Semaphore is available (which happens above in ADC_data_ready_ISR)
		xSemaphoreTake(adc_ready_Semaphore, portMAX_DELAY);
		calculate_rms();

		/* `#END` */
	}
}
/* ------------------------------------------------------------------------ */
void tsk_analog_Start(void) {
	/*
	 * Insert task global memeory initialization here. Since the OS does not
	 * initialize ANY global memory, execute the initialization here to make
	 * sure that your task data is properly 
	 */
	/* `#START TASK_GLOBAL_INIT` */

	/* `#END` */

	if (tsk_analog_initVar != 1) {

		/*
	 	* Create the task and then leave. When FreeRTOS starts up the scheduler
	 	* will call the task procedure and start execution of the task.
	 	*/
		xTaskCreate(tsk_analog_TaskProc, "Analog", STACK_ANALOG, NULL, PRIO_ANALOG, &tsk_analog_TaskHandle);
		tsk_analog_initVar = 1;
	}
}
/* ------------------------------------------------------------------------ */
/* ======================================================================== */
/* [] END OF FILE */
