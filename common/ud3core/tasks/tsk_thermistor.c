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
#include <math.h>

#include "tsk_thermistor.h"
#include "tsk_fault.h"
#include "alarmevent.h"

/* RTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

xTaskHandle tsk_thermistor_TaskHandle;
uint8 tsk_thermistor_initVar = 0u;

SemaphoreHandle_t adc_sem;

enum fan_mode{
    FAN_NORMAL = 0,
    FAN_TEMP1_TEMP2 = 1,
    FAN_NOT_USED = 2,
    FAN_TEMP2_RELAY3 = 3,
    FAN_TEMP2_RELAY4 = 4
};

typedef struct{
    uint16_t fault1_cnt;
    uint16_t fault2_cnt;
}TEMP_FAULT;

#define THERM_1 0
#define THERM_2 1
#define THERM_GND 2

/* ------------------------------------------------------------------------ */
/*
 * Place user included headers, defines and task global data in the
 * below merge region section.
 */
/* `#START USER_INCLUDE SECTION` */
#include "tsk_analog.h"
#include "cli_common.h"
#include "telemetry.h"
#include "tsk_priority.h"

#define THERM_DAC_VAL 23

#define TEMP1_FAULT 0x00FF
#define TEMP2_FAULT 0xFF00

#define TEMP_FAULT_LOW -32640

#define TEMP_FAULT_COUNTER_MAX 5

#define TEMP_TABLE_SIZE 32

//temperature *128
int16_t count_temp_table[TEMP_TABLE_SIZE];

// bits is 12 which is the number of bits in the ADC that converts the thermistor voltage to a digital value.
// The ADC maps 12 bits to a voltage of 0 to 5 volts (maybe -.1 to 500.1) not sure...
void calc_table_128(int16_t t_table[], uint8_t bits, uint16_t table_size, uint32_t B, uint32_t RN, uint16_t meas_curr_uA){
	int32_t max_cnt = 1 << bits;

	uint32_t steps =  max_cnt / table_size;     // steps = 4096 / TEMP_TABLE_SIZE (32) = 128

	float meas_current = meas_curr_uA * 1e-6;   // the current from the IDAC: 184 / 1000000 = .000184 amps

	float u_ref_mv = 5.0;   // I think the mv stands for measure voltage, not millivolts
	float r_cnt;
	float temp_cnt;
	

	int32_t i;
	uint16_t w=0;
	for(i=0;i<=max_cnt;i+=steps){
        // Convert voltage dac units to resistance.  u_ref_mv / meas_current is the
        // total possible resistance, which is divided by the ADC range to get ohms per ADC
        // unit, which is finally multiplied by i (the number of ADC units being calculated
        // for this LUT node).
		if(i>0){
			r_cnt = u_ref_mv / max_cnt * i / meas_current;  // DS: 
		}else{
			r_cnt = 0;
		}
		
        // Regarding the magic numbers below:
        // 298.15 = 25 degrees Centigrade in Kelvin.  This is the standard temp for NTC's R25 measurement.
        // -273.15 = absolute zero.  Subtracting this from degrees Kelvin converts to Centigrade.
        // The equation itself is documented here: https://www.jameco.com/Jameco/workshop/TechTip/temperature-measurement-ntc-thermistors.html
        // 1/T = 1/T0 + 1/B * ln(R/R0)
        
        // Rearranged the following code to avoid a divide by 0 when r_cnt is set to 0 above.
        // note that log(a/b) = -log(b/a)...
		temp_cnt = (298.15/(1+(log(((float)r_cnt/RN))*(298.15/(float)B))))-273.15; 
		temp_cnt *= 128;
		
		t_table[w] = temp_cnt;
        if(w<table_size) w++;
		
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

void initialize_thermistor(void) {
	IDAC_therm_Start();
	ADC_therm_Start();
    Therm_Mux_Start();
	
}

int32_t get_temp_128(int32_t counts) {
	uint16_t counts_div = counts / 128;
	if (counts_div == TEMP_TABLE_SIZE-1){
		return TEMP_FAULT_LOW;
    }
	uint32_t counts_window = counts_div * 128;

    // DS: The following code is just doing a linear interpolation.  I think it could be rewritten as:
    // int32_t a = count_temp_table[counts_div];
    // int32_t b = count_temp_table[counts_div+1];
    // return a + ((b - a) * (counts %128)) / 128;
	return count_temp_table[counts_div] - (((int32_t)(count_temp_table[counts_div] - count_temp_table[counts_div+1]) * ((uint32_t)counts - counts_window)) / 128);

}

int32_t get_temp_counts(uint8_t channel){
    Therm_Mux_Select(channel);
    vTaskDelay(50);     // DS: bugfix - this used to delay just 20 ticks which wasn't long enough on my board.  The temps were all off by 10 degrees or so.  Once I changed this to 50 the temps were all within a degree or so.
    ADC_therm_StartConvert();
    vTaskDelay(50);
    int16_t temp = ADC_therm_GetResult16()-ADC_therm_Offset;
    if(temp<0)temp=0;
    return temp;
}

void run_temp_check(TEMP_FAULT * ret) {
	//this function looks at all the thermistor temperatures, compares them against limits and returns any faults

    tt.n.temp1.value = get_temp_128(get_temp_counts(THERM_1)) / 128;
    tt.n.temp2.value = get_temp_128(get_temp_counts(THERM_2)) / 128;

    // DS: TODO: The following lines are comparing a signed to an unsigned.  The values will be negative if
    // the thermistor is not installed (usual case for temp2).  C will implicitly convert the signed value
    // to a huge unsigned value for the comparison which causes the result to be true.  End result is that
    // temp2_high will be true if it does not exist and will set an alarm.  TLDR: Add a (int32_t) cast to the configuration vars below

	// check for faults
	if (tt.n.temp1.value > configuration.temp1_max && configuration.temp1_max) {
        if(ret->fault1_cnt){
            ret->fault1_cnt--;
        }
	}else{
        ret->fault1_cnt = TEMP_FAULT_COUNTER_MAX;
    }
	if (tt.n.temp2.value > configuration.temp2_max && configuration.temp2_max) {
		if(ret->fault2_cnt){
            ret->fault2_cnt--;
        }
	}else{
        ret->fault2_cnt = TEMP_FAULT_COUNTER_MAX;
    }
    
    uint_fast8_t temp1_high = (tt.n.temp1.value > configuration.temp1_setpoint);
    uint_fast8_t temp2_high = (tt.n.temp2.value > configuration.temp2_setpoint);

    switch(configuration.temp2_mode){
        case FAN_NORMAL:
            Fan_Write(temp1_high);
        break;
        case FAN_TEMP1_TEMP2:
            Fan_Write( (temp1_high || temp2_high) ? 1 : 0);
        break;
        case FAN_NOT_USED:
            
        break;
        case FAN_TEMP2_RELAY3:
            Fan_Write(temp1_high);
            Relay3_Write(temp2_high);
        break;
        case FAN_TEMP2_RELAY4:
            Fan_Write(temp1_high);
            Relay4_Write(temp2_high);
        break;
    }

	return;
}

// DS: configuration.idac is initialized to 185.  I think this comes from THERM_DAC_VAL (23)
// multiplied by the IDAC's 8 ua per DAC count.  23 counts * 8ua per count = 184.  The IDAC is
// configured at initialization to output a current of 23 DAC units which equals 184 micro amps.
// configuration.ntc_r25 is the resistance of the thermistor at 25 degrees centigrade (10000 ohms).
// configuration.ntc_b comes from the thermistor datasheet and is 3977 by default.  The 
// recommended B57875S0103J002 thermistor has a B of 3460.  The B57867S0103J140 thermistor that 
// I bought has a B of 3988.
uint8_t callback_ntc(parameter_entry * params, uint8_t index, TERMINAL_HANDLE * handle){
    calc_table_128(count_temp_table, 12 , sizeof(count_temp_table) / sizeof(int16_t),
        configuration.ntc_b, configuration.ntc_r25, configuration.idac);
    return 1;
}

uint8_t CMD_ntc(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args) {
    xSemaphoreTake(adc_sem, portMAX_DELAY);
    
    ttprintf("Connect a precision 10k to therm 1\r\n");
    ttprintf("Press [y] to proceed\r\n");
   
    if(getch(handle, portMAX_DELAY) != 'y'){
        ttprintf("Calibration aborted\r\n");
        return TERM_CMD_EXIT_SUCCESS;
    }
    
    Therm_Mux_Select(THERM_1);
    vTaskDelay(100);
    ADC_therm_StartConvert();
    vTaskDelay(100);
    float temp = ADC_therm_CountsTo_Volts(ADC_therm_GetResult16())*100;
    configuration.idac = temp;
    ttprintf("Calibration finished: %u uA\r\n", configuration.idac);
    ttprintf("Recalculate NTC LUT...\r\n");
    callback_ntc(NULL, 0, handle);
    ttprintf("Finished... Please save to eeprom.\r\n");
    xSemaphoreGive(adc_sem);
    return TERM_CMD_EXIT_SUCCESS;
}

void calib_adc(){
    
    IDAC_therm_SetValue(THERM_DAC_VAL*3);
    Therm_Mux_Select(THERM_GND);
    vTaskDelay(50);
    int32_t cnt=0;
    for(uint8_t i = 0;i<4;i++){
        ADC_therm_StartConvert();
        vTaskDelay(50);
        cnt += ADC_therm_GetResult16();
        // DS: The following "if" is not necessary.  Just do it after the loop.
        if(i==3){
            cnt /= 4;
            ADC_therm_SetOffset(cnt);
            alarm_push(ALM_PRIO_INFO, "ADC: Temperature ADC offset", cnt);
        }
    }    
}

/* `#END` */
/* ------------------------------------------------------------------------ */
/*
 * This is the main procedure that comprises the task.  Place the code required
 * to preform the desired function within the merge regions of the task procedure
 * to add functionality to the task.
 */
void tsk_thermistor_TaskProc(void *pvParameters) {
	/*
	 * Add and initialize local variables that are allocated on the Task stack
	 * the the section below.
	 */
	/* `#START TASK_VARIABLES` */

	/* `#END` */

	/*
	 * Add the task initialzation code in the below merge region to be included
	 * in the task.
	 */
	/* `#START TASK_INIT_CODE` */
    
    adc_sem = xSemaphoreCreateBinary();
    xSemaphoreGive(adc_sem);
    
	initialize_thermistor();
    calib_adc();

    IDAC_therm_SetValue(THERM_DAC_VAL);
    
    callback_ntc(NULL,0,NULL);
    
    TEMP_FAULT temp;
    temp.fault1_cnt = TEMP_FAULT_COUNTER_MAX;
    temp.fault2_cnt = TEMP_FAULT_COUNTER_MAX;
    

	/* `#END` */
    alarm_push(ALM_PRIO_INFO, "TASK: Thermistor started", ALM_NO_VALUE);
	for (;;) {
        xSemaphoreTake(adc_sem, portMAX_DELAY);
		/* `#START TASK_LOOP_CODE` */
        run_temp_check(&temp);
        
                
        if(temp.fault1_cnt == 0){
            if(sysfault.temp1==0){
                alarm_push(ALM_PRIO_CRITICAL, "NTC: Temperature Therm1 high", tt.n.temp1.value);
            }
            sysfault.temp1 = 1;
        }
        
        if(temp.fault2_cnt == 0){
            if(sysfault.temp2==0){
                alarm_push(ALM_PRIO_CRITICAL, "NTC: Temperature Therm2 high", tt.n.temp2.value);
            }
            sysfault.temp2 = 1;
        }
        xSemaphoreGive(adc_sem);
      
		/* `#END` */

		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}
/* ------------------------------------------------------------------------ */
void tsk_thermistor_Start(void) {
	/*
	 * Insert task global memeory initialization here. Since the OS does not
	 * initialize ANY global memory, execute the initialization here to make
	 * sure that your task data is properly 
	 */
	/* `#START TASK_GLOBAL_INIT` */

	/* `#END` */

	if (tsk_thermistor_initVar != 1) {

		/*
	 	* Create the task and then leave. When FreeRTOS starts up the scheduler
	 	* will call the task procedure and start execution of the task.
	 	*/
		xTaskCreate(tsk_thermistor_TaskProc, "Therm", STACK_THERMISTOR, NULL, PRIO_THERMISTOR, &tsk_thermistor_TaskHandle);
		tsk_thermistor_initVar = 1;
	}
}
/* ------------------------------------------------------------------------ */
/* ======================================================================== */
/* [] END OF FILE */
