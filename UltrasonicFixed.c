#include <termios.h>                                                         
#include <stdio.h>
#include <stdlib.h>	
#include <string.h>
#include <unistd.h>                                                          
#include <fcntl.h>                                                                                                               
#include <sys/types.h> 
#include <stdint.h>
#include <fcntl.h>
#include <sys/signal.h>
#include <sys/types.h>
#include <termios.h>
#include <time.h>
#include <stdbool.h>
#include <stropts.h>
#include <poll.h>	
#include <wiringPi.h>	
#include <errno.h>

#define BAUDRATE B9600                                                      
#define MODEMDEVICE "/dev/ttyS0"


int counters = 0;
int fd;                                                             
char buf[255];  
int variable;
struct pollfd fds[1];
int ret, res;

int set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf ("error %d from tcgetattr", errno);
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
                return -1;
        }
        return 0;
}

void set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf("error %d from tggetattr", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                printf("error %d setting term attributes", errno);
}

void OpenUltrasonicDevice(){
		/* open the device */
	fd = open(MODEMDEVICE, O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (fd == 0)
	{
		perror(MODEMDEVICE);
		printf("Failed to open MODEMDEVICE \"/dev/ttyAMA0\"\n");
		exit(-1); 
	}

	set_interface_attribs (fd, BAUDRATE, 0);  // set speed to 19200 bps, 8n1 (no parity)
	set_blocking (fd, 0);                // set no blocking

	/* Open STREAMS device. */
	fds[0].fd = fd;
	fds[0].events = POLLRDNORM;
	}

//Konversi dari data serial menjadi jarak asli
int getJarak(){
		ret = poll(fds, 1, 150);
		//wait for response
		if (ret > 0)
		{
			counters = 0;
			/* An event on one of the fds has occurred. */
			if (fds[0].revents & POLLHUP)
			{
				return -1;
				printf("Hangup\n");
			}
			if (fds[0].revents & POLLRDNORM)
			{
				//jika ada data masuk, maka jarak mulai dihitung hitung.
					int jarak = 0;
					res = read(fd,buf,255);
					buf[res]=0;
					//jika data yang masuk nilainya dibawah 100, nilai 0 dibuang
					if((buf[1]-'0') == 0){
						jarak = (10*(buf[2] - 48)) + (buf[3]-48);
						printf("%i\n", counters);
						return jarak;
						}
					//jika data yang masuk lebih dari 100 maka digit pertama disimpan
					else{
						jarak = (100*(buf[1] - 48))+ (10*(buf[2] - 48)) + (buf[3]-48);
						}
			printf("%i\n", counters);
			return jarak;//data yang keluar berupa satuan Centi meter
			}
		}
		//Power Checker, jika sistem mati maka counter akan bertambah untuk menghitung value error dari sistem.
		else{
			counters += 1;
			printf("%i\n", counters);
			if(counters == 200){
					counters = 0;
					return -1;
					}
			return 0;
			}
	return -1;
	}

int main(void)
{ 	
	OpenUltrasonicDevice();
	for (;;)
	{
		int a = getJarak();
	}
}
