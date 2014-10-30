//Jayaraj Poroor: UIP Task header file
#ifndef UIPTASK_H_
#define UIPTASK_H_

#include <eventq.h>
#include <stream_eventq.h>

#define UIP_TASK_YIELD_TIME_MS			10
    
void uip_task(void *p);

void uip_task_init(const uint8 *ipbuf, const uint8 *netmask, const uint8 *mac);
          
#endif /*UIPTASK_H_*/
