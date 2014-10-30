#ifndef LPC21XX_RES_H_
#define LPC21XX_RES_H_

#include <typedefs.h>

#define RES_VIC_VECT_MAX 15

uint32 res_port0_acquire_(const char *user, uint32 pins); 
uint32 res_port0_release_(const char *user, uint32 pins);

uint32 res_port1_acquire_(const char *user, uint32 pins); 
uint32 res_port1_release_(const char *user, uint32 pins);

uint32 res_fiq_acquire_(const char *user);
uint32 res_fiq_release_(const char *user);

uint32 res_vic_vect_acquire_(const char *user, uint32 vect);
uint32 res_vic_vect_release_(const char *user, uint32 vect);
uint32 res_vic_vect_acquire_any_(const char *user);

#endif /*LPC21XX_RES_H_*/
