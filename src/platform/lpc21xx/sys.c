#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <typedefs.h>
#include <sys.h>

void sys_sleep(uint32 seconds)
{
	sys_msleep(1000*seconds);
}

void sys_msleep(uint32 millis)
{
	uint32 nticks = millis/portTICK_RATE_MS;
	
	xQueueReceive( ((tskTCB*)pxCurrentTCB)->ws.wait_q, (signed portCHAR *) 0UL, nticks );
}
