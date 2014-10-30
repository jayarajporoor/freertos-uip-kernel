#ifndef BOARD_CONFIG_H_
#define BOARD_CONFIG_H_

/*
 * Board-configuration for embedded artists edu board
 */

#define BOARD_CONFIG_FOSC_MHZ				12

#define BOARD_CONFIG_PORT0_SPI0_SSEL_PIN	PIN_SPI0_SSEL 
#define BOARD_CONFIG_PORT0_SSP_SSEL			PIN_SSP_SSEL
#define BOARD_CONFIG_NUART					2


#define BOARD_CONFIG_NUART					2

void board_config();

#endif /*BOARD_CONFIG_H_*/
