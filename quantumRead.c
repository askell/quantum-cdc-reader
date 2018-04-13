#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>

int set_interface_attribs (int fd, int speed, int parity)
{
		struct termios tty;
		memset (&tty, 0, sizeof tty);
		if (tcgetattr (fd, &tty) != 0)
		{
				printf("error %d from tcgetattr\n", errno);
				return -1;
		}

		cfsetospeed (&tty, speed);
		cfsetispeed (&tty, speed);

		tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
		// disable IGNBRK for mismatched speed tests; otherwise receive break
		// as \000 chars
		tty.c_iflag &= ~IGNBRK;         // disable break processing
		tty.c_lflag = 0;                // no signaling chars, no echo,
										// no canonical processing
		tty.c_oflag = 0;                // no remapping, no delays
		tty.c_cc[VMIN]  = 0;            // read doesn't block
		tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

		tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

		tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
										// enable reading
		tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
		tty.c_cflag |= parity;
		tty.c_cflag &= ~CSTOPB;
		tty.c_cflag &= ~CRTSCTS;

		if (tcsetattr (fd, TCSANOW, &tty) != 0)
		{
				printf("error %d from tcsetattr\n", errno);
				return -1;
		}
		return 0;
}

int main(int argc, char *argv[])
{
	if(argc != 2)
	{
		printf("Invalid arguments supplied.\n");
		return -1;
	}

	char *portname = argv[1];

	int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0)
	{
		printf("error %d opening %s: %s\n", errno, portname, strerror (errno));
		return -1;
	}

	set_interface_attribs (fd, B9600, 0);

	write (fd, "\xC0\x00\xE1\xF0\xC0", 5);
	usleep (5 * 2000);
	write (fd, "\xC0\xDB\xDC\x20\x2F\x39\xC0", 7);
	usleep ((7 + 32) * 2000);             
	char buf [32];
	int n = read (fd, buf, sizeof buf);
	if (n != 32)
	{
		printf("Invalid bytes read: %d\n", n);
		return -1;
	}
	//unsigned long left = buf[12] | (buf[11] << 8) | (buf[10] << 16) | (buf[9] << 24);
	//unsigned long right = buf[16] | (buf[15] << 8) | (buf[14] << 16) | (buf[13] << 24);
	unsigned long average = buf[28] | (buf[27] << 8) | (buf[26] << 16) | (buf[25] << 24);
	printf("%lu", average);

	return 0;
}

