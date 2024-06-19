/*
 * File:   scheduler.c
 * Author: Chiappe, Di Donna, Guelfi, Utegaliyeva
 *
 * Created on 14 giugno 2024, 9.37
 */


#include "scheduler.h"

void scheduler(heartbeat schedInfo[], int nTasks) 
{
    int i;
    for (i = 0; i < nTasks; i++) {
        if(schedInfo[i].enable == 1)
        {
            schedInfo[i].n++;
            if (schedInfo[i].n >= schedInfo[i].N) {
                schedInfo[i].f(schedInfo[i].params);            
                schedInfo[i].n = 0;
            }
        }
        else
        {
            schedInfo[i].n = 0;
        }
    }
}
