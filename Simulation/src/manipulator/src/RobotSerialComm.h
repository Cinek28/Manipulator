// RobotSerialComm.h

#ifndef __ROBOTSERIALCOMM_H__
#define ____ROBOTSERIALCOMM_H__

#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#include "CyclicBuffer.h"

#define TIMEOUT 1
#define BAUDRATE B9600
#define MAX_PARAMS 29

typedef enum ManipulatorCmd
{
	JOINT = 0,
	TOOL = 1,
	TEST = 2,
	IDLE = 3
} ManipulatorCmd;

struct ManipulatorMsg
{
	ManipulatorCmd type;
	unsigned char length;
	unsigned char params[MAX_PARAMS];
	unsigned char checksum;
}

class RobotSerialComm
{

public:
	RobotSerialComm();
	~RobotSerialComm();

	bool openPort( int port = 0, int baudrate = 9600 );
	bool closePort( void );

	int sendData( const char *, int );
	int readData( void );

	bool isOpened( void ){ return( portOpened ); }

protected:
	struct termios tty;
	int serialPort;

	bool portOpened = false;

	cyclicBuffer buffer;

	bool writeByte( unsigned char );
	int readByte( void *, int );
};

#endif
