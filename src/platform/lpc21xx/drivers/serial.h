

/*! \file serial.h
    \brief Serial Library Routines Header File.
*/   
#ifndef SERIAL_H_
#define SERIAL_H_


/*! \brief Data Link Escape CHaracter.*/
#define SER_DLE_CHAR				0x10

/*! \brief Connection Close CHaracter.*/
#define SER_CLS_CHAR				0x03


/*! \brief Serial Device Odd Parity Flag.*/
#define SER_PARITY_O				'O'

/*! \brief Serial Device Even Parity Flag.*/
#define SER_PARITY_E				'E'

/*!	\brief Serial Device No Parity Flag.*/
#define SER_PARITY_N				'N'

/*! \brief Serial Device in Synchronous Mode.*/
#define SER_SYNC_MODE 				'S'

/*!	\brief Serial Device in Asynchronous Mode.*/
#define SER_ASYNC_MODE 				'A'

/*! \brief Serial Read Timeout.*/
#define SER_READ_TIMEOUT			0

/*! \brief Serial DLE Escape.*/
#define SER_DLE_RECVD				2

/*!	\brief Serial Buffer Maximum Size.*/
#define SER_MAX_BUF					32


/*! \brief Opens serial port. Returns handle >= 0 if success or error code < 0 if error. */
int32 serial_open (uint8 port, uint32 mode);
int32 serial_close( int32 port);

/* Synchronous Routines*/
int32 serial_read(int32 port, int8 *buf, int32 nread);
int32 serial_write(int32  port, const int8 *buf, int32 nwrite);
int32 serial_timed_read(int32 port, int8 *buf, int32 nread, int32 msecs);


#endif /*SERIAL_H_*/
