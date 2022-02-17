#include "cyapicallbacks.h"
#include <cytypes.h>

/* RTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "cli_common.h"
#include "tsk_cli.h"
#include "telemetry.h"
#include "tsk_priority.h"

void tsk_duty_task(void *pvParameters) {
	OnTimeCounter_Start();
    uint32_t lastTickCount = SG_Timer_ReadCounter();
    
    while(1){
        vTaskDelay(pdMS_TO_TICKS(250));
        
        //read and reset counters
        // OnTimeCounter is a timer component.  It is connected to the interrupter signal ANDed  
        // with the 1Mhz interrupter clock.  The timer is a 24 bit count DOWN counter - it decrements on
        // each tick.  That is why they subtract from the max value 0xffffff.
        // SG_Timer is another timer component connected to the Synth_Clock signal
        uint32_t onTime = 0xffffff - OnTimeCounter_ReadCounter();
        uint32_t period = lastTickCount - SG_Timer_ReadCounter();
        
        OnTimeCounter_WriteCounter(0xfffffff);  // DS: Bug: Note this is writing 7 'f's.  But the timer is only 24 bits (6 f'fs).
        lastTickCount = SG_Timer_ReadCounter();
        // The interrupter clock is 1Mhz.  The SG_Timer is 320K.  1Mhz/320K = 3.125.
        
        // dutycycle := onTime / period
        // the period is 3,125 times slower
        // to get the scaling (0,1% resolution) we do dutycycle * 1000
        // together with fixed point division that results in
        //      
        //      dutycycleScaled := (onTime * 1000) / ((period * 3125) / 1000)
        
        tt.n.dutycycle.value = (onTime * 1000) / ((period * 3125) / 1000);
    }
}

void tsk_duty_Start(void) {
	xTaskCreate(tsk_duty_task, "Duty", configMINIMAL_STACK_SIZE, NULL, PRIO_DUTY, NULL);
}