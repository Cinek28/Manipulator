// RobotSerialComm.cpp

#include "RobotSerialComm.h"

RobotSerialComm::RobotSerialComm()
{
	memset(&tty, 0, sizeof(tty));
}

RobotSerialComm::~RobotSerialComm()
{
	close();
}

bool RobotSerialComm::openPort( int nPort, int nBaud )
{
	serialPort = open("/dev/tty0", O_RDWR);

	if (serialPort < 0) {
    	printf("Error %i from open: %s\n", errno, strerror(errno));
		return false;
	}

	if(tcgetattr(serialPort, &tty) != 0) 
	{
    	printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
		return false;
	}

	tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity
	tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used
	tty.c_cflag |= CS8; // 8 bits per byte
	tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control
	tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines
	tty.c_lflag &= ~ICANON; // Disable canonical mode
	tty.c_lflag &= ~ECHO; // Disable echo
	tty.c_lflag &= ~ECHOE; // Disable erasure
	tty.c_lflag &= ~ECHONL; // Disable new-line echo
	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off sw flow ctrl
	// Disable any special handling of received bytes:
	tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); 
	tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes
	tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return
	// Wait for up to 100ms, return as soon as any data received:
	tty.c_cc[VTIME] = TIMEOUT;
	tty.c_cc[VMIN] = 1;
	// Set in/out baud rate to be 9600
	cfsetispeed(&tty, B9600);
	cfsetospeed(&tty, B9600);

	// Save tty settings, also checking for error
	if (tcsetattr(serialPort, TCSANOW, &tty) != 0) 
	{
    	printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
		return false;
	}
	isOpened = true;

	return true;

}

bool RobotSerialComm::closePort( void )
{
	isOpened = false;
	close(serialPort);
}

bool RobotSerialComm::writeByte( unsigned char ucByte )
{
	BOOL bWriteStat;
	DWORD dwBytesWritten;

	bWriteStat = WriteFile( m_hIDComDev, (LPSTR) &ucByte, 1, &dwBytesWritten, &m_OverlappedWrite );
	if( !bWriteStat && ( GetLastError() == ERROR_IO_PENDING ) ){
		if( WaitForSingleObject( m_OverlappedWrite.hEvent, 1000 ) ) dwBytesWritten = 0;
		else{
			GetOverlappedResult( m_hIDComDev, &m_OverlappedWrite, &dwBytesWritten, FALSE );
			m_OverlappedWrite.Offset += dwBytesWritten;
			}
		}

	return( TRUE );

}

int RobotSerialComm::sendData( ManipulatorMsg *msg)
{
	unsigned char byte = (unsigned char) msg->length;
	write(serialPort, &byte, sizeof(unsigned char)*(msg->length+3));
}

bool RobotSerialComm::readByte(unsigned char* byte)
{
	int num_bytes = read(serialPort, byte, sizeof(unsigned char));

	if(num_bytes == 1)
		return true;

	return false;
}

bool RobotSerialComm::readData( ManipulatorMsg *msg)
{
	unsigned char byte = 0;
	if(!readByte(&byte))
		return false;
	setBufferByte(&buffer, byte);
	int size = (int)getBufferOccupiedSize(&buffer);

	if(size == 2)
	{
	    if(peekBufferByte(&buffer, 0) != 0xff ||
	    	peekBufferByte(&buffer, 1) != 0xff)
	    {
	    	clearBuffer(&buffer);
	    }
	}
	else if(size == 3)
	{
	    msg->type = *(buffer.end-1);
	}
	else if(size == 4)
	{
	    msg->length = *(buffer.end-1);
		if(msg->length > MAX_PARAMS)
	    {
	    	msg->length = 0;
	    	clearBuffer(&buffer);
	    }
	}
	else if(size > 4 && size <= 4 + msg->length)
	{
	    msg->params[size-5] = *(buffer.end-1);
	}
	else if(size > 4 + msg->length)
	{
	    msg->checksum = *(buffer.end-1);
	    uint8_t calcChecksum = msg->type + msg->length;
	    for(int i = 0; i < msg->length; ++i)
	    {
	    	calcChecksum += msg->params[i];
	    }
	    calcChecksum = ~calcChecksum;
	    if(calcChecksum == msg->checksum)
	    {
	    	return true;
	    }

	    clearBuffer(&buffer);
	}
	return false;
}

