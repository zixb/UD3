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

#define TEMP_FAULT_LOW 0xFFFF

#define TEMP_FAULT_COUNTER_MAX 5

//temperature *128
const uint16 count_temp_table[] = {
	15508, 10939, 8859, 7536, 6579, 5826, 5220, 4715, 4275, 3879, 3551, 3229, 2965, 2706, 2469, 2261,
	2054, 1861, 1695, 1530, 1364, 1215, 1084, 953, 822, 690, 576, 473, 369, 266, 163, 59};

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
	IDAC_therm_SetValue(THERM_DAC_VAL);
}

uint16_t get_temp_128(uint16_t counts) {
	uint16_t counts_div = counts / 128;
	if (!counts_div)
		return TEMP_FAULT_LOW;
	uint32_t counts_window = counts_div * 128;

	counts_div--;
	if (counts > counts_window) {
		return count_temp_table[counts_div] - (((uint32_t)(count_temp_table[counts_div] - count_temp_table[counts_div + 1]) * ((uint32_t)counts - counts_window)) / 128);
	} else {
		return count_temp_table[counts_div] - (((uint32_t)(count_temp_table[counts_div - 1] - count_temp_table[counts_div]) * ((uint32_t)counts - counts_window)) / 128);
	}
}
int16_t get_temp_counts(uint8_t channel){
    Therm_Mux_Select(channel);
    vTaskDelay(20);
    ADC_therm_StartConvert();
    vTaskDelay(20);
    return ADC_therm_GetResult16()-80;  //compensate for 100mV Offset
}

void run_temp_check(TEMP_FAULT * ret) {
	//this function looks at all the thermistor temperatures, compares them against limits and returns any faults
    
	tt.n.temp1.value = get_temp_128(get_temp_counts(0)) / 128;
	tt.n.temp2.value = get_temp_128(get_temp_counts(1)) / 128;

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
    
	initialize_thermistor();
    
    TEMP_FAULT temp;
    temp.fault1_cnt = TEMP_FAULT_COUNTER_MAX;
    temp.fault2_cnt = TEMP_FAULT_COUNTER_MAX;
    

	/* `#END` */
    alarm_push(ALM_PRIO_INFO,warn_task_thermistor, ALM_NO_VALUE);
	for (;;) {
		/* `#START TASK_LOOP_CODE` */
        run_temp_check(&temp);
        
        if(temp.fault1_cnt == 0){
            if(sysfault.temp1==0){
                alarm_push(ALM_PRIO_CRITICAL, warn_temp1_fault, tt.n.temp1.value);
            }
            sysfault.temp1 = 1;
        }
        
        if(temp.fault1_cnt == 0){
            if(sysfault.temp2==0){
                alarm_push(ALM_PRIO_CRITICAL, warn_temp2_fault, tt.n.temp2.value);
            }
            sysfault.temp2 = 1;
        }
      
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
