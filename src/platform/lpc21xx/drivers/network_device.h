#ifndef NETWORK_DEVICE_H_
#define NETWORK_DEVICE_H_

#include "typedefs.h"

void ethernetInit (void);
u16 ethernetReceive(void);
void ethernetSend(void);

#define network_device_init ethernetInit
#define network_device_send ethernetSend
#define network_device_read ethernetReceive
 
#endif /*NETWORK_DEVICE_H_*/
