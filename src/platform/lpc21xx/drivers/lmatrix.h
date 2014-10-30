#ifndef __LMATRIX_H__

#define __LMATRIX_H__ 1

#define LMATRIX_NUM_OF_BOARDS 6
#define LMATRIX_NUM_OF_LEDS (LMATRIX_NUM_OF_BOARDS * 4)
#define LMATRIX_MAX_STR_LEN (LMATRIX_NUM_OF_LEDS * 2)

#define LMATRIX_INTENSITY_MAX 0xFF
#define LMATRIX_INTENSITY_HIGH 0xAA 
#define LMATRIX_INTENSITY_LOW 0x55

int32 lmatrix_open(uint8 intensity);
void lmatrix_test();
int32 lmatrix_write(char *buf, int len);
int32 lmatrix_close();

#endif
