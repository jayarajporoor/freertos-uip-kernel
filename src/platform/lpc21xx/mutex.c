#include <mutex_api.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <mutex_api.h>
#include <errno.h>


#define MUTEX_MAX		2

#define MUTEX_FREE		0
#define MUTEX_ALLOC		1
#define MUTEX_LOCKED 	2

typedef struct
{
	uint8				state;
	wait_list_t			wl;//wait list for tasks to block on mutex
}mutex_ctx_t;

#define MUTEX_CTX(p) ((mutex_ctx_t*)(p))
mutex_ctx_t mutexes[MUTEX_MAX];

STATUS mutex_create	( HMUTEX *h )
{
	int i;
	
	taskENTER_CRITICAL();
	{
		for(i=0;i<MUTEX_MAX;i++)
		{
			if(MUTEX_FREE == mutexes[i].state)
			{
				mutexes[i].state = MUTEX_ALLOC;
				task_wait_list_init(&mutexes[i].wl);
				break;
			}
		}
	}
	taskEXIT_CRITICAL();
	
	if(i < MUTEX_MAX)
	{
		h->mutex_ctx = &mutexes[i];
		return 0;
	}
	return -ENOBUFS;
}

STATUS mutex_lock	( HMUTEX h )
{
	uint8 prev_state;
	int32 status=0;
	
	taskENTER_CRITICAL();
	{
		prev_state = MUTEX_CTX(h.mutex_ctx)->state;
		MUTEX_CTX(h.mutex_ctx)->state|= MUTEX_LOCKED;
		if(prev_state&MUTEX_LOCKED)
		{
			task_wait_list_enqueue_(&(MUTEX_CTX(h.mutex_ctx)->wl), CURR_WAIT_STRUCT());
		}
	}
	taskEXIT_CRITICAL();

	if(prev_state&MUTEX_LOCKED)
	{	
		TASK_WAIT(CURR_WAIT_STRUCT(), &status);
	}
	return status;
}

STATUS mutex_unlock	( HMUTEX h )
{
	wait_struct_t *w=(wait_struct_t*)0UL;
	int32 status=0;
	
	taskENTER_CRITICAL();
	{
		if(TASK_WAIT_LIST_IS_EMPTY(&(MUTEX_CTX(h.mutex_ctx)->wl)))
		{
			MUTEX_CTX(h.mutex_ctx)->state&= ~MUTEX_LOCKED;	
		}else
		{
			w = task_wait_list_dequeue_(&(MUTEX_CTX(h.mutex_ctx)->wl));
		}
	}
	taskEXIT_CRITICAL();

	if(w)
	{	
		TASK_NOTIFY(w, &status);
	}
	return 0;	
}
	
STATUS mutex_destroy(HMUTEX h )
{
	//TODO
	return 0;
}

