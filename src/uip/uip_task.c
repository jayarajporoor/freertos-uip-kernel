/*
FreeRTOS-UIP Integration: UIP task by Jayaraj Poroor
*/
#include <FreeRTOS.h>
#include <uip.h>
#include <uip_arp.h>
#include <enc28j60.h>
#include <timer.h>
#include <klog.h>
#include <uip_task.h>
#include <sys.h>
#include <task.h>
#include <mutex_api.h>
#include <socket.h>

#define BUF ((struct uip_eth_hdr *)&uip_buf[0])

void uip_task_appcall();

void uip_task_init(const uint8 *ipbuf, const uint8 *netmask, const uint8 *mac)
{
  	uip_ipaddr_t ipaddr;
  		
  	uip_init();
  	uip_arp_init();
  	
  	uip_ipaddr(ipaddr, ipbuf[0], ipbuf[1], ipbuf[2], ipbuf[3]);
  	uip_sethostaddr(ipaddr);
  	
  	uip_ipaddr(ipaddr, netmask[0], netmask[1], netmask[2], netmask[3]);
  	uip_setnetmask(ipaddr);  		

	uip_ethaddr.addr[0] = mac[0];
	uip_ethaddr.addr[1] = mac[1];
	uip_ethaddr.addr[2] = mac[2];
	uip_ethaddr.addr[3] = mac[3];
	uip_ethaddr.addr[4] = mac[4];
	uip_ethaddr.addr[5] = mac[5];
  		
}


void uip_task(void *p)
{
	int i;
  	struct timer periodic_timer, arp_timer;

  	enc28j60_open();	

  	timer_set(&periodic_timer, CLOCK_SECOND / 2);
  	timer_set(&arp_timer, CLOCK_SECOND * 10);  
  	
  	
  	while(1) 
  	{
  		socket_process_pending_lists_();
  		
    	uip_len = enc28j60_read();
    	
    	if(uip_len > 0) 
    	{
      		if(BUF->type == htons(UIP_ETHTYPE_IP)) 
      		{
        		uip_input();
        		/* If the above function invocation resulted in data that
           		should be sent out on the network, the global variable
           		uip_len is set to a value > 0. */
        		if(uip_len > 0) 
        		{
          			uip_arp_out();
          			enc28j60_write();
        		}
      		} 
      		else if(BUF->type == htons(UIP_ETHTYPE_ARP)) 
      		{
        		uip_arp_arpin();
        		/* If the above function invocation resulted in data that
           		should be sent out on the network, the global variable
           		uip_len is set to a value > 0. */
        		if(uip_len > 0) 
        		{
          			enc28j60_write();
        		}
      		}

		} 
		
		if(timer_expired(&periodic_timer)) 
		{
      		timer_reset(&periodic_timer);
      		for(i = 0; i < UIP_CONNS; i++) 
      		{
        		uip_periodic(i);
        		/* If the above function invocation resulted in data that
           		should be sent out on the network, the global variable
           		uip_len is set to a value > 0. */
        		if(uip_len > 0) 
        		{
          			uip_arp_out();
          			enc28j60_write();
        		}
      		}

#if UIP_UDP
      		for(i = 0; i < UIP_UDP_CONNS; i++) 
      		{
        		uip_udp_periodic(i);
        		/* If the above function invocation resulted in data that
           		should be sent out on the network, the global variable
           		uip_len is set to a value > 0. */
        		if(uip_len > 0) 
        		{
          			uip_arp_out();
          			enc28j60_write();
        		}
        	}
#endif /* UIP_UDP */           
      		/* Call the ARP timer function every 10 seconds. */
      		if(timer_expired(&arp_timer)) 
      		{
      			timer_reset(&arp_timer);
      			uip_arp_timer();
      		}
    	}
    	
    	//yield so that other tasks can do something
    	sys_msleep(UIP_TASK_YIELD_TIME_MS);
	}
  	
}


void uip_log(char *msg)
{
	klog_info("UIP: %s", msg);
}


void uip_task_appcall()
{  
	if(uip_connected()) 
	{
		socket_process_new_connect_();
	}else
	if(uip_poll())
	{
		if(uip_stopped(uip_conn))
		{
			socket_process_try_restart_();
		}
		
		socket_process_write_();
	}else
	if(uip_newdata())
	{
		socket_process_new_data_();
		socket_process_write_();
	}else
	if(uip_aborted() || uip_closed())
	{
		socket_process_close_();
	}else
	if(uip_timedout())
	{
		socket_process_timeout_();
		uip_close();
	}else
	if(uip_acked() || uip_rexmit())
	{
		socket_process_write_();
	}
}

