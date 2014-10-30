/*
Socket-layer interface for UIP by Jayaraj Poroor
*/

#include <typedefs.h>
#include <FreeRTOS.h>
#include <timer.h>
#include <task.h>
#include <queue.h>
#include <uip.h>
#include <mutex_api.h>
#include <socket.h>
#include <uip_task.h>
#include <errno.h>
#include <klog.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define SOCKET_CLOSED 				0x000
#define SOCKET_OPEN					0x001
#define SOCKET_LISTEN				0x002
#define SOCKET_ACTIVE				0x004
#define SOCKET_READ_PENDING	 		0x008
#define SOCKET_IBUF_PENDING 	 	0x010
#define SOCKET_WRITE_PENDING  		0x020
#define SOCKET_CLOSE_PENDING		0x040
#define SOCKET_PENDING_UIP_CONNECT  0x080
#define SOCKET_PENDING_UIP_LISTEN   0x100
#define SOCKET_ZOMBIE				0x200
#define SOCKET_WRITE_ACK_WAIT       0x400

extern struct uip_eth_addr uip_ethaddr;

static socket_t sockets[SOCKET_MAX];

static socket_t *lh_open;

#define SOCKET_CHECK_BIND	 0x01
#define SOCKET_CHECK_ZOMBIE  0x02

int32 _socket_lock_and_check(int fd, uint32 check);
void _socket_list_add(socket_t **list, socket_t *p_list_head);
void socket_list_remove_(socket_t **list, socket_t *p_list_head);
void socket_ip_aton_(uint8 *ipbuf, int ipbuf_sz, const char *ipstr);
void socket_ip_ntoa_(char *ipstr, int ipstr_sz, const uint8 *ipbuf);

int32 _socket_buf_copy(void *buf, socket_buf_t *sbuf, uint32 len);

/*
 * Note: functions ending in '_' are called by the uip_task 
 * functions starting with '_' are called by all application tasks
 * functions starting and ending with '_' are called by uip_task as well as
 * all application tasks
 */

int32 socket_init(const char *ip, const char *netmask, const uint8 *mac)
{
	int i;
	uint8 	ipbuf[4];
	uint8	netmaskbuf[4];
	int status;

	socket_ip_aton_(ipbuf, sizeof(ipbuf), ip);

	socket_ip_aton_(netmaskbuf, sizeof(netmaskbuf), netmask);

	uip_task_init(ipbuf, netmaskbuf, mac);
						
	for(i=0;i<SOCKET_MAX;i++)
	{
		 if((status=mutex_create(&sockets[i].mutex)) < 0)
		 {
		 	klog_error("%s:%d> Can't create mutex for socket: %d. Error: %ld\n"
		 			, __FILE__, __LINE__, i, status); 
		 }
	}
	return 0;
}


int32 _socket_open()
{
	int32 i;
	int   got_socket=0;

	for(i=0;i<SOCKET_MAX;i++)
	{
		if(SOCKET_CLOSED == sockets[i].state)
		{
			//we have a closed socket
			//now lock and check again
			//to avoid race condition
			mutex_lock(sockets[i].mutex);
			if(SOCKET_CLOSED == sockets[i].state)
			{
				//ok now we're sure
				got_socket=1;
				sockets[i].state = SOCKET_OPEN;
				sockets[i].rw_timer.interval = (clock_time_t)-1;
				taskENTER_CRITICAL();
				{				
					_socket_list_add(&lh_open, &sockets[i]);
				}
				taskEXIT_CRITICAL();
			}
			mutex_unlock(sockets[i].mutex);
			if(got_socket)
			{
				break;
			}
		}
	}

	if( i >= SOCKET_MAX )
	{
		i = -1;
	}
	
	return i;
}

int32 _socket_bind(int fd, uint16 port)
{
	int32 status;
	status = _socket_lock_and_check(fd, SOCKET_CHECK_ZOMBIE);
	
	if(status < 0)
	{
		mutex_unlock(sockets[fd].mutex);		
		return status;
	}

	sockets[fd].lport = port;
	
	mutex_unlock(sockets[fd].mutex);	
	return 0;
}

int32 _socket_listen(int fd)
{
	int32 status;
	status = _socket_lock_and_check(fd, SOCKET_CHECK_ZOMBIE|SOCKET_CHECK_BIND);
		
	if(status < 0)
	{
		mutex_unlock(sockets[fd].mutex);		
		return status;
	}
	
	sockets[fd].state |= SOCKET_LISTEN;
	
	mutex_unlock(sockets[fd].mutex);
	return 0;	
}

int32 _socket_accept(int fd, stream_param_t *buf)
{
	int32 status;
	status = _socket_lock_and_check(fd, SOCKET_CHECK_ZOMBIE|SOCKET_CHECK_BIND);
	
	if(status < 0)
	{
		mutex_unlock(sockets[fd].mutex);		
		return status;
	}
	
	sockets[fd].req_ws = CURR_WAIT_STRUCT();
	sockets[fd].req_buf = buf;
	
	sockets[fd].state |= SOCKET_LISTEN;
	sockets[fd].state |= SOCKET_PENDING_UIP_LISTEN;
	
	mutex_unlock(sockets[fd].mutex);	
	TASK_WAIT(CURR_WAIT_STRUCT(), &status);
	
	return status;
}

int32 _socket_connect(int fd, stream_param_t *buf)
{
	int32 status;
	
	status = _socket_lock_and_check(fd, SOCKET_CHECK_ZOMBIE);
	if(status < 0)
	{
		mutex_unlock(sockets[fd].mutex);		
		return status;
	}
	
	sockets[fd].req_ws = CURR_WAIT_STRUCT();
	sockets[fd].req_buf = buf;
	
	sockets[fd].state |= SOCKET_PENDING_UIP_CONNECT;	
	
	mutex_unlock(sockets[fd].mutex);
	TASK_WAIT(CURR_WAIT_STRUCT(), &status);
	
	return status;
		
}

int32 _socket_read(int fd, uint8 *buf, int32 len)
{
	int32 status;
	
	//for reading, we don't mind if it's a zombie - we
	//can read whatever is in the ibuf - if any
	status = _socket_lock_and_check(fd, 0);
	
	if(status < 0)
	{
		mutex_unlock(sockets[fd].mutex);
		return status;
	}
	
	if(!(sockets[fd].state&SOCKET_IBUF_PENDING))
	{
		//if no ibufs are pending and if this is already a zombie 
		//we'll not try blocking read
		if(sockets[fd].state&SOCKET_ZOMBIE)
		{
			mutex_unlock(sockets[fd].mutex);
			return -EPIPE;			
		}
		
		sockets[fd].req_buf = buf;
		sockets[fd].req_count = len;
		sockets[fd].req_ws = CURR_WAIT_STRUCT();
		if(sockets[fd].rw_timer.interval != (clock_time_t)-1)
		{
			timer_restart(&sockets[fd].rw_timer);
		}	
		sockets[fd].state |= SOCKET_READ_PENDING;
		mutex_unlock(sockets[fd].mutex);		
		TASK_WAIT(CURR_WAIT_STRUCT(), &status);
		//again lock the socket
		if(status >= 0)
		{
			status = _socket_lock_and_check(fd, 0);
		}else
		{
			return status;//read failed - so go back
		}		
	}
		
	if(status >= 0)
	{
		status = -EPIPE;
		if(sockets[fd].state&SOCKET_IBUF_PENDING)
		{
			status = _socket_buf_copy(buf, &sockets[fd].ibuf, len);
			if(sockets[fd].ibuf.count == 0)
			{	
				sockets[fd].state &= ~SOCKET_IBUF_PENDING;		
			}
		}
	}
	mutex_unlock(sockets[fd].mutex);
		
	return status;
}

int32 _socket_write(int fd, uint8 *buf, int32 len)
{
	int32 status;
	
	status = _socket_lock_and_check(fd, SOCKET_CHECK_ZOMBIE);
	
	if(status < 0)
	{
		mutex_unlock(sockets[fd].mutex);
		return status;
	}

	sockets[fd].req_buf = buf;
	sockets[fd].req_count = len;
	sockets[fd].close_status = 0;
	sockets[fd].req_ws = CURR_WAIT_STRUCT();		
	sockets[fd].state |= SOCKET_WRITE_PENDING;

	if(sockets[fd].rw_timer.interval != (clock_time_t)-1)
	{
		timer_restart(&sockets[fd].rw_timer);
	}	
	
	mutex_unlock(sockets[fd].mutex);	
	TASK_WAIT(CURR_WAIT_STRUCT(), &status);
	
	return status;
	
}

int32 _socket_setopt(int fd, int32 opt, void *val)
{
	int32 status=EINVAL;
	status = _socket_lock_and_check(fd, SOCKET_CHECK_ZOMBIE);
	
	if(status < 0)
	{
		mutex_unlock(sockets[fd].mutex);
		return status;
	}
	
	switch(opt)
	{
		case STREAM_OPT_ADDR_REUSE:		
			break;
		case STREAM_OPT_READ_TIMEOUT:
			timer_set(&sockets[fd].rw_timer, *((clock_time_t*)val));
			break; 
	}
	
	mutex_unlock(sockets[fd].mutex);
			
	return status;
}

/*
 * This is the socket_close method called by application tasks
 */
int32 _socket_close(int fd)
{
	int32 status;

	if(fd < 0 || fd >= SOCKET_MAX)
	{
		return -EBADFD;
	}
	
	mutex_lock(sockets[fd].mutex);
	
	if(!(sockets[fd].state&SOCKET_ZOMBIE))
	{	
		sockets[fd].req_ws = CURR_WAIT_STRUCT();	
		sockets[fd].state |= SOCKET_CLOSE_PENDING;
	
		mutex_unlock(sockets[fd].mutex);
	
		TASK_WAIT(CURR_WAIT_STRUCT(), &status);		
		//clear ibuf and release the socket
		sockets[fd].ibuf.pos = 0;
		sockets[fd].ibuf.count = 0;		
		sockets[fd].state = SOCKET_CLOSED;	
	}else
	{
		//this is already a zombie - so clear ibuf and release it
		sockets[fd].ibuf.pos = 0;
		sockets[fd].ibuf.count = 0;				
		sockets[fd].state = SOCKET_CLOSED;
		mutex_unlock(sockets[fd].mutex);
	}
		
	return 0;	
}

/*
 * Lock the socket mutex and do basic validation 
 */
int32 _socket_lock_and_check(int fd, uint32 check)
{
	int32 status;
	
	if(fd < 0 || fd >= SOCKET_MAX)
	{
		return -EBADFD;
	}
	
	mutex_lock(sockets[fd].mutex);
	
	if(!(sockets[fd].state&SOCKET_OPEN))
	{
		status = -EBADFD;
		return status;
	}
	
	if((check&SOCKET_CHECK_ZOMBIE) && 
			(sockets[fd].state&SOCKET_ZOMBIE) )
	{
		//the socket was closed owing to some internal error			
		//socket status was preserved so that it can be given to the application
		//save the status on the local stack		
		status = sockets[fd].close_status;
		
		if(status >= 0)//no proper error code
		{
			status = -ESTALE;
		}
		
		return status;
		
	}		
	
	if((check&SOCKET_CHECK_BIND) && (sockets[fd].lport == 0))
	{
		return -EINVAL;//TODO - not bound error
	}
	return 0;	
}

/*
 * This is the actual socket_close method - invoked by the uip_task
 */
void socket_close_(socket_t *sock, int32 status)
{
	wait_struct_t *saved_ws;
	
	//this is always called when socket mutex is locked
	//so if we do it again here it'll cause deadlock 
	
	sock->req_buf = (void*)0UL;//can't do any more writes
	sock->req_count = 0;
	sock->close_status = status;
	if(sock->conn != (struct uip_conn*)0UL)
	{
		sock->conn->appstate = (socket_t*)0UL;
		sock->conn=(struct uip_conn *)0UL;
	}
	
	//the socket can't be reallocated till application
	//gets the close status
	saved_ws = sock->req_ws;
	sock->req_ws = (wait_struct_t*)0UL;
	sock->state |= SOCKET_ZOMBIE;
	
	taskENTER_CRITICAL();
	{	
		socket_list_remove_(&lh_open, sock);
	}
	taskEXIT_CRITICAL();
	
	if(sock->state&(SOCKET_READ_PENDING|SOCKET_WRITE_PENDING|SOCKET_CLOSE_PENDING ))
	{
		TASK_NOTIFY(saved_ws, &status);	
	}
	
}

void socket_process_pending_lists_()
{	
	stream_param_t *param;
	uint8 			ipbuf[4];
	uip_ipaddr_t	ipaddr;	
	socket_t 		*sock=lh_open;
	int32			status;
	struct uip_conn *conn;
	
	while(sock)
	{
		mutex_lock(sock->mutex);
		
		if(sock->state&SOCKET_CLOSE_PENDING)
		{
			socket_close_(sock, 0);//proper close
		}else
		if(sock->state&SOCKET_PENDING_UIP_LISTEN)
		{
			sock->state &= ~SOCKET_PENDING_UIP_LISTEN;			
			uip_listen(HTONS(sock->lport));
		}else
		if(sock->state&SOCKET_PENDING_UIP_CONNECT)
		{
			sock->state &= ~SOCKET_PENDING_UIP_CONNECT;
			param = (stream_param_t *)sock->req_buf;
			socket_ip_aton_(ipbuf, sizeof(ipbuf), param->ip);
			uip_ipaddr(ipaddr, ipbuf[0], ipbuf[1], ipbuf[2], ipbuf[3]);
			conn = uip_connect(&ipaddr, htons(param->port));
			if(conn	== (struct uip_conn*)0UL)
			{
				status = -EINVAL;
				TASK_NOTIFY(sock->req_ws, &status);
			}else
			{
				conn->appstate = sock; 
				sock->conn = conn;
				sock->lport = htons(conn->lport);
				sock->state |= SOCKET_ACTIVE;//we're active now				
			}		
		}else
		if(sock->state&SOCKET_READ_PENDING)
		{
			if( 	(sock->rw_timer.interval != (clock_time_t)-1) 
			   &&   timer_expired(&sock->rw_timer) )
			{
				status = -ETIMEDOUT;
				sock->state &= ~SOCKET_READ_PENDING;
				TASK_NOTIFY(sock->req_ws, &status);
			}else
			if(sock->state&SOCKET_IBUF_PENDING)
			{
				//This task blocked on read_pending just before it got a buffer
				//so we need to wake up the task
				status = 0;
				sock->state &= ~SOCKET_READ_PENDING;
				TASK_NOTIFY(sock->req_ws, &status);
			}
		}else
		if(sock->state&SOCKET_WRITE_PENDING)
		{
			if( (sock->rw_timer.interval != (clock_time_t)-1) 
			   &&   timer_expired(&sock->rw_timer) )
			{
				status = -ETIMEDOUT;
				sock->state &= ~SOCKET_WRITE_PENDING;
				TASK_NOTIFY(sock->req_ws, &status);
			}			
		}	
		
		mutex_unlock(sock->mutex);
		
		sock=sock->next;
	}
	
}

void socket_process_new_connect_()
{
	stream_param_t  *p;
	int32 			status;
	uint8 			ipbuf[4];	
	socket_t *sock=(socket_t*)uip_conn->appstate;
	
/*TODO: For Active Sockets:
 	if(sock != (socket_t *)0UL)
	{
		mutex_lock(sock->mutex);
		status = 0;
		TASK_NOTIFY(sock->req_ws, &status);
		mutex_unlock(sock->mutex);
		return;
	}*/
	
	//Not an active connect.
	//So check for matching passive listening socket
	
	sock=lh_open;
		
	while(sock)
	{
		mutex_lock(sock->mutex);
		if( (!(sock->state&SOCKET_ACTIVE)) &&
		    (sock->state&SOCKET_LISTEN) &&
			(htons(sock->lport) == uip_conn->lport)     )
		{
			uip_conn->appstate = sock;
			sock->conn = uip_conn;
			if(sock->state&SOCKET_LISTEN)
			{
				p = (stream_param_t*)sock->req_buf;
				ipbuf[0] = uip_ipaddr1(uip_conn->ripaddr);
				ipbuf[1] = uip_ipaddr2(uip_conn->ripaddr);
				ipbuf[2] = uip_ipaddr3(uip_conn->ripaddr);
				ipbuf[3] = uip_ipaddr4(uip_conn->ripaddr);
				
				socket_ip_ntoa_( p->ip, sizeof(p->ip), ipbuf);
				p->port = ntohs(uip_conn->rport);
			}					
			
			sock->state |= SOCKET_ACTIVE;
			status = 0;
			TASK_NOTIFY(sock->req_ws, &status);
			mutex_unlock(sock->mutex);			
			break;
		}
		mutex_unlock(sock->mutex);		
		sock = sock->next;
	}
	
}

void socket_process_close_()
{
	socket_t *sock=(socket_t*)uip_conn->appstate;
	
	if(sock)
	{
		mutex_lock(sock->mutex);
	}
	
	if(sock)
	{
		socket_close_(sock, -ECONNRESET);
	}
	
	if(sock)
	{
		mutex_unlock(sock->mutex);
	}		
}

void socket_process_timeout_()
{
	socket_t *sock=(socket_t*)uip_conn->appstate;
	
	if(sock)
	{
		mutex_lock(sock->mutex);
	}
	
	if(sock && (sock->state&SOCKET_OPEN))
	{
		socket_close_(sock, -ETIMEDOUT);
	}	
	
	if(sock)
	{
		mutex_unlock(sock->mutex);
	}
}

void socket_process_try_restart_()
{
	socket_t *sock=(socket_t*)uip_conn->appstate;
	
	if(sock)
	{
		mutex_lock(sock->mutex);
	}
	
	if(sock && (sock->state&SOCKET_ACTIVE))
	{
		if(!(sock->state&SOCKET_IBUF_PENDING))
		{
			uip_restart();
		}
	}else
	{
		uip_close();
	}
	
	if(sock)
	{
		mutex_unlock(sock->mutex);
	}
		
}

void socket_process_write_()
{
	int32 status;
	socket_t *sock=(socket_t*)uip_conn->appstate;
	
	if(sock)
	{
		mutex_lock(sock->mutex);
	}
	
	if(sock && (sock->state&SOCKET_ACTIVE))
	{
		if(sock->state&SOCKET_WRITE_PENDING)
		{
			if(uip_acked())
			{
				sock->req_count -= sock->write_count;
				sock->req_buf += sock->write_count;
				sock->state &= ~SOCKET_WRITE_ACK_WAIT;
			}else
			if(uip_rexmit())
			{
				uip_send(sock->req_buf, sock->req_count);				
			}

			if(sock->req_count > 0)
			{
			    if(!(sock->state&SOCKET_WRITE_ACK_WAIT))
			    { 								
					sock->state|= SOCKET_WRITE_ACK_WAIT;					
					if(sock->req_count < uip_mss())
					{
						sock->write_count = sock->req_count;						
					}else
					{
						sock->write_count = uip_mss();
					}
					uip_send(sock->req_buf, sock->write_count);
			    }					
			}else
			{
				status = 0;
				sock->state &= ~SOCKET_WRITE_PENDING;
				TASK_NOTIFY(sock->req_ws, &status);
			}
			
		}		
	}else
	{
		uip_close();
	}
	
	if(sock)
	{
		mutex_unlock(sock->mutex);
	}
	
}


void socket_process_new_data_()
{
	socket_t *sock=(socket_t*)uip_conn->appstate;
	int32 status;
	
	if(sock)
	{
		mutex_lock(sock->mutex);
	}
	
	if(sock && (sock->state&SOCKET_ACTIVE))
	{
		if(!(sock->state&SOCKET_IBUF_PENDING))
		{		
			memcpy(sock->ibuf.buf, uip_appdata, uip_datalen());
			sock->ibuf.pos = 0;
			sock->ibuf.count = uip_datalen();
			sock->state |= SOCKET_IBUF_PENDING;
			uip_stop();//we can't process any further data till this buffer is consumed
			if(sock->state&SOCKET_READ_PENDING)
			{
				status = 0;
				sock->state &= ~SOCKET_READ_PENDING;
				TASK_NOTIFY(sock->req_ws, &status);
			}
		}else
		{
			//This condition should not happen - since as soon as we
			//get data and move it to the ibuf we would've done uip_stop
			//till sbuf is free again		
			klog_error("%s:%d> Socket read out of sync. Received socket data, but buf is not free!"
			__FILE__, __LINE__); 
			uip_close();
			socket_close_(sock, -ENOBUFS);
		}
	}else
	{
		uip_close();
	}	
	
	if(sock)
	{
		mutex_unlock(sock->mutex);
	}
	
}

void socket_ip_aton_(uint8 *ipbuf, int ipbuf_sz, const char *ipstr)
{
	if(ipbuf_sz < 4)
	{
		return;
	}
	
	ipbuf[0] = atoi(ipstr);
	ipbuf[1] = ipbuf[2]= ipbuf[3] = 0;
	
	while(*ipstr && *ipstr != '.') ipstr++;
	if(*ipstr) ipstr++;
	
	if(*ipstr) ipbuf[1] = atoi(ipstr);

	while(*ipstr && *ipstr != '.') ipstr++;
	if(*ipstr) ipstr++;
	
	if(*ipstr) ipbuf[2] = atoi(ipstr);

	while(*ipstr && *ipstr != '.') ipstr++;
	if(*ipstr) ipstr++;
	
	if(*ipstr) ipbuf[3] = atoi(ipstr);	
	
}

void socket_ip_ntoa_(char *ipstr, int ipstr_sz, const uint8 *ipbuf)
{
	uint8 val;
	int i;
	
	if(ipstr_sz <= 1)
	{
		return;
	}
	
	for(i=0;i<4;i++)
	{
		
		val = ipbuf[i]/100U;
		if(val != 0)
		{
			*ipstr = val + '0';
			ipstr++;
			ipstr_sz--;
			if(ipstr_sz <= 0)
			{
				break;
			}			
		}
			
		val = (ipbuf[i]-val*100)/10;
	
		if(val != 0)
		{
			*ipstr = val + '0';
			ipstr++;
			ipstr_sz--;
			if(ipstr_sz <= 0)
			{
				break;
			}						
		}
		
		val = ipbuf[i]%10;
		
		*ipstr = val + '0';
		ipstr++;
		ipstr_sz--;
		if(ipstr_sz <= 0)
		{
			break;
		}			
		
		if(i < 3)
		{
			*ipstr = '.';
			ipstr++;
			ipstr_sz--;
			if(ipstr_sz <= 0)
			{
				break;
			}						
		}
	}
	
	*ipstr= '\0';
	
}

int32 _socket_buf_copy(void *buf, socket_buf_t *sbuf, uint32 len)
{
	int32 len_to_copy;
	
	if(sbuf->count == 0 || len == 0)
	{
		return 0;
	}
	len_to_copy = (len > sbuf->count)? sbuf->count : len ; 
	
	memcpy(buf, &sbuf->buf[sbuf->pos], len_to_copy);
	
	sbuf->pos += len_to_copy;
	sbuf->count -= len_to_copy;
	
	return len_to_copy;
	
}


void _socket_list_add(socket_t **p_list_head, socket_t *item)
{
	item->next = *p_list_head;
	*p_list_head = item;	
}

void socket_list_remove_(socket_t **p_list_head, socket_t *item)
{
	socket_t *prev;
	socket_t *curr;
	
	if(*p_list_head == item)
	{
		*p_list_head = item->next;
		item->next = (socket_t*)0UL;
	}else
	{
		prev = *p_list_head;
		curr = (*p_list_head)->next;
		while(curr)
		{	
			if(curr == item)
			{
				prev->next = curr->next;
				curr->next = (socket_t*)0UL;
				break;				
			}
			prev=curr;
			curr=curr->next;
		}
	}
}
