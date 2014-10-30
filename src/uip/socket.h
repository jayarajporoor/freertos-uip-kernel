#ifndef SOCKET_H_
#define SOCKET_H_

#include <stream_eventq.h>
#include <uip-conf.h>

#define PROTOCOL_HDR_SIZE (14+32)

typedef struct tag_socket_buf
{
	uint8 					buf[UIP_CONF_BUFFER_SIZE-PROTOCOL_HDR_SIZE];
	uint32					pos;
	uint32					count;				
}socket_buf_t;

struct uip_conn;

typedef struct tag_socket
{
	uint16					lport;
	socket_buf_t			ibuf;/*input buffer*/
	uint32					state;
	int32					close_status;
	int32					write_count;
	struct timer			rw_timer;
	void 					*req_buf;	
	int32					req_count;
	wait_struct_t			*req_ws;//wait struct for tasks to block on the socket	
	struct uip_conn			*conn;//back-ref to associate uip_conn for active sockets							
	struct 	tag_socket 		*next;//for lists
	HMUTEX					mutex;
}socket_t;

#define	SOCKET_MAX				2

int32 socket_init(const char *ip, const char *netmask, const uint8 *mac);

int32 _socket_open();

int32 _socket_bind(int fd, uint16 port);

int32 _socket_listen(int fd);

int32 _socket_accept(int fd, stream_param_t *buf);

int32 _socket_connect(int fd, stream_param_t *buf);

int32 _socket_read(int fd, uint8 *buf, int32 len);

int32 _socket_write(int fd, uint8 *buf, int32 len);

int32 _socket_setopt(int fd, int32 opt, void *val);

int32 _socket_close(int fd);


void socket_process_pending_lists_();

void socket_process_new_connect_();

void socket_process_close_();

void socket_process_timeout_();

void socket_process_try_restart_();

void socket_process_write_();

void socket_process_new_data_();

 
#endif /*SOCKET_H_*/
