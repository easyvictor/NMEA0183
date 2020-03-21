/*
NMEA0183LinuxStream.cpp

2018 Copyright (c) Timo Lappalainen   All rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.


THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

Inherited tNMEA0183Stream object for Linux
*/

#if defined(__linux__)||defined(__linux)||defined(linux)

#include <iostream>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions
#include "NMEA0183LinuxStream.h"

using namespace std;

//*****************************************************************************
tNMEA0183LinuxStream::tNMEA0183LinuxStream(const char *_stream, uint32_t _baud) : port(-1) {
  // If baud is nonzero, we assume it is a serial port. Else we will assume it's a generic
  // Linux FIFO or other file handle.
  if ( _stream!=0 ) {
    port=open(_stream, O_RDWR | O_NOCTTY);
  }
  if ( port!=-1 ) {
    cout << _stream << " Opened" << endl;
  } else {
    if ( _stream!=0 ) cout << "Failed to open " << _stream << endl;
  }

  // Handle serial port
  if (_baud > 0) {
    struct termios tty;
    struct termios tty_old;
    memset (&tty, 0, sizeof tty);

    /* Error Handling */
    if ( tcgetattr ( port, &tty ) != 0 ) {
      cerr << "Error " << errno << " from tcgetattr: " << strerror(errno) << endl;
    }

    /* Save old tty parameters */
    tty_old = tty;

    /* Set Baud Rate */
    speed_t speed;
    switch (_baud) {
      case 4800: speed = B4800; break;
      case 9600: speed = B9600; break;
      case 19200: speed = B19200; break;
      case 38400: speed = B38400; break;
      default: cerr << "Baud " << _baud << " is not supported.\n";
      exit(1);
    }
    cfsetospeed (&tty, speed);
    cfsetispeed (&tty, speed);

    /* Setting other Port Stuff */
    tty.c_cflag     &= ~PARENB;            // Make 8n1
    tty.c_cflag     &= ~CSTOPB;
    tty.c_cflag     &= ~CSIZE;
    tty.c_cflag     |= CS8;
    tty.c_cflag     |= CREAD;
    tty.c_cflag     |= CLOCAL;
    tty.c_cflag     |= CRTSCTS;
    tty.c_cflag     |= CREAD;
    tty.c_cflag     |= CLOCAL;
    tty.c_cc[VMIN]   =  0;                  // non blocking
    tty.c_cc[VTIME]  =  0;                  // non blocking

    /* Flush Port, then applies attributes */
    tcflush(port, TCIFLUSH);
    if (tcsetattr(port, TCSANOW, &tty) != 0) {
       cout << "Error " << errno << " from tcsetattr" << endl;
    }
  }

}

//*****************************************************************************
tNMEA0183LinuxStream::~tNMEA0183LinuxStream() {
  if ( port!=-1 ) {
    close(port);
  }
}

/********************************************************************
*	Other 'Bridge' functions and classes
*
*
*
**********************************************************************/
int tNMEA0183LinuxStream:: read() {
  if ( port!=-1 ) {
    char buf;
    auto nBytes = ::read(port, &buf, 1);
    if (nBytes == 1) {
      return (int)buf;
    } else {
      return -1;
    }
  } else {
    // Serial stream bridge -- Returns first byte if incoming data, or -1 on no available data.
    struct timeval tv = { 0L, 0L };
    fd_set fds;

    FD_ZERO(&fds);
    FD_SET(0, &fds);
    if (select(1, &fds, NULL, NULL, &tv) < 0)                                   // Check fd=0 (stdin) to see if anything is there (timeout=0)
        return -1;                                                              // Nothing is waiting for us.

   return (getc(stdin));                                                         // Something is there, go get one char of it.
  }
}

//*****************************************************************************
size_t tNMEA0183LinuxStream:: write(const uint8_t* data, size_t size) {                // Serial Stream bridge -- Write data to stream.
  if ( port!=-1 ) {
    return ::write(port,data,size);
  } else {
    size_t i;

    for (i=0; (i<size) && data[i];  i++)                                        // send chars to stdout for 'size' or until null is found.
        putc(data[i],stdout);

    return(i);
  }
}

//*****************************************************************************
int tNMEA0183LinuxStream::available() {
  fd_set readfs;
  int status = 0;
  timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 0;
  FD_ZERO(&readfs);
  FD_SET(port, &readfs);
  status = select(port+1, &readfs, NULL, NULL, &tv);
  if (status > 0 && FD_ISSET(port, &readfs)) {
    return 1;
  } else {
    return 0;  
  }
}

#endif
