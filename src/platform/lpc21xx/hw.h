#ifndef HW_H_
#define HW_H_

int hw_init(int run_from_ram);

void hw_fiq_handler( void ) __attribute__ ((naked));

#endif /*HW_H_*/
