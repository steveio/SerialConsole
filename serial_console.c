/**
* serial_console.cpp
*
* A simple linux cmd line cat tool to display char ascii output from an (Arduino) serial device
*
* Todo -
* implement stream API - read(), find, findUntil(), filter()
*	handle > 256 char buffer length, cyclic buffer, realloc
*
* @see https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/
*/

// C library headers
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <signal.h>
#include <stdint.h>


// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()


#define BUFFER_SIZE 8

// Define the function to be called when ctrl-c (SIGINT) signal is sent to process
void signal_callback_handler(int signum)
{
   printf("Caught signal %d\n",signum);
   exit(signum);
}


void delay(int number_of_seconds)
{
    // Converting time into milli_seconds
    int milli_seconds = 1000 * number_of_seconds;

    // Storing start time
    clock_t start_time = clock();

    // looping till required time is not achieved
    while (clock() < start_time + milli_seconds)
        ;
}

// @see http://www.mit.edu/afs.new/sipb/project/merakidev/include/bits/termios.h
void setBaud(uint32_t baud_input, uint32_t * baud)
{
  switch(baud_input)
  {
    case 9600:
      (*baud) = B9600;
      break;
    case 19200:
      (*baud) = B19200;
      break;
    case 38400:
      (*baud) = B38400;
      break;
    case 57600:
      (*baud) = B57600;
      break;
    case 115200:
      (*baud) = B115200;
      break;
    case 230400:
      (*baud) = B230400;
      break;
    default:
      (*baud) = B9600;
  }

}


int main(int argc, char const *argv[])
{

	signal(SIGINT, signal_callback_handler);

  if (argc < 3)
	{
		printf("Usage: ./serial_console <port> <baud> \n");
		return 1;
	}

  char port[32];
	strcpy(port, argv[1]);

  char const *baud_input = argv[2];
  int n = 0;
  int i;
  for(i =0; i< strlen(baud_input); i++) {
      n = 10 * n + baud_input[i] - '0';
  }

  uint32_t baud;
  int * ptr = &baud;
  setBaud(n, ptr);
  //uint16_t baud_input;
	//strcpy(baud_input, argv[2]);
  //uint16_t baud;
  //setBaud(baud_input, baud);

	// Open the serial port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)
	int serial_port = open(port, O_RDWR);

	// Create new termios struc, we call it 'tty' for convention
	struct termios tty;
	memset(&tty, 0, sizeof tty);

	// Read in existing settings, and handle any error
	if(tcgetattr(serial_port, &tty) != 0) {
	    printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
	}

	tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
	tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
	tty.c_cflag |= CS8; // 8 bits per byte (most common)
	tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
	tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

	tty.c_lflag &= ~ICANON;
	tty.c_lflag &= ~ECHO; // Disable echo
	tty.c_lflag &= ~ECHOE; // Disable erasure
	tty.c_lflag &= ~ECHONL; // Disable new-line echo
	tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
	tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

	tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
	tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
	// tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
	// tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

	tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
	tty.c_cc[VMIN] = 0;

	// Set in/out baud rate
	cfsetispeed(&tty, baud);
	cfsetospeed(&tty, baud);

	// Save tty settings, also checking for error
	if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
	    printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
	}


	while(1)
	{
		// Allocate memory for read buffer, set size according to your needs
		uint8_t read_buf [BUFFER_SIZE];
		memset(&read_buf, '\0', sizeof(read_buf));

		// Read bytes. The behaviour of read() (e.g. does it block?,
		// how long does it block for?) depends on the configuration
		// settings above, specifically VMIN and VTIME
		int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));

    //printf("%s \n", read_buf);
    //printf("%d \n", num_bytes);

		// n is the number of bytes read. n may be 0 if no bytes were received, and can also be -1 to signal an error.
		if (num_bytes < 0) {
		    printf("Error reading: %s", strerror(errno));
		}

    for(int i=0; i<num_bytes; i++)
    {

      if (read_buf[i] > 0x20)
      {
        printf("%c", read_buf[i]);
      }
      if (read_buf[i] == 0x0A)
      {
        printf("\n");
      }
      fflush(stdout);

    }


	}

	close(serial_port);

}
