
#ifndef __SPI_H__
#define __SPI_H__

#define SPI_OP_WRITE 1
#define SPI_OP_READ  2
#define SPI_OP_HAS_CMD 4

#define SPI_ST_ABRT			(1UL << 3)
#define SPI_ST_MODF			(1UL << 4)
#define SPI_ST_ROVR			(1UL << 5)
#define SPI_ST_WCOL			(1UL << 6)
#define SPI_ST_DONE			(1UL << 7)

int32 spi_open();

#define spi_read(buf, len) spi_op((buf), (len), SPI_OP_READ, 0)
#define spi_write(buf, len) spi_op((buf), (len), SPI_OP_WRITE, 0)
#define spi_write_read(buf, len) spi_op((buf), (len), SPI_OP_WRITE|SPI_OP_READ, 0)

#define spi_cmd_read(buf, len, cmd) spi_op((buf), (len), SPI_OP_READ|SPI_OP_HAS_CMD, (cmd))
#define spi_cmd_write(buf, len, cmd) spi_op((buf), (len), SPI_OP_WRITE|SPI_OP_HAS_CMD, (cmd))

int32 spi_op(uint8 *buf, int32 len, int8 op, uint8 first_byte);

/*
#define SPI_SLAVE_SELECT\
	portENTER_CRITICAL();\
	PORT0_CLR(BOARD_CONFIG_PORT0_SPI0_SSEL_PIN)

#define SPI_SLAVE_UNSELECT\
	PORT0_SET(BOARD_CONFIG_PORT0_SPI0_SSEL_PIN);\
	portEXIT_CRITICAL()

uint8 spi_put (uint8 valueIn);
*/	

#endif
