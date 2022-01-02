
#include "system.h"

// DS: Returns the total non-idle time as percent of total time * 10
uint32_t SYS_getCPULoadFine(TaskStatus_t * taskStats, uint32_t taskCount, uint32_t sysTime){
    // DS: TODO: I think the next line should be if(sysTime < configTICK_RATE_HZ) to 
    // avoid a divide by 0 below.  configTICK_RATE_HZ is 1000 so if sysTime is 500 to 999 then
    // (sysTime/configTICK_RATE_HZ) will be 0 which is then used as the denominator below.
    if(sysTime<500) return 0;
    uint32_t currTask = 0;
    for(;currTask < taskCount; currTask++){
        if(strlen(taskStats[currTask].pcTaskName) == 4 && strcmp(taskStats[currTask].pcTaskName, "IDLE") == 0){
            return configTICK_RATE_HZ - ((taskStats[currTask].ulRunTimeCounter) / (sysTime/configTICK_RATE_HZ));
        }
    }
    return -1;
}

const char * SYS_getTaskStateString(eTaskState state){
    switch(state){
        case eRunning:
            return "running";
        case eReady:
            return "ready";
        case eBlocked:
            return "blocked";
        case eSuspended:
            return "suspended";
        case eDeleted:
            return "deleted";
        default:
            return "invalid";
    }
}