#ifndef __SSP_H__
#define __SSP_H__

#define SSP_MAX_WORDS 8

int32 ssp_open();
int32 ssp_write_word(uint16* text, int32 num_of_words);

#endif
