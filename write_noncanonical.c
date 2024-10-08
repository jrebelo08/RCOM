// Write to serial port in non-canonical mode
//
// Modified by: Eduardo Nuno Almeida [enalmeida@fe.up.pt]

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include <signal.h>

// Baudrate settings are defined in <asm/termbits.h>, which is
// included by <termios.h>
#define BAUDRATE B38400
#define _POSIX_SOURCE 1 // POSIX compliant source

#define FALSE 0
#define TRUE 1

#define BUF_SIZE 5
#define MAX_RETRIES 4
#define TIMEOUT_SEC 3

volatile int STOP = FALSE;
volatile int alarmEnabled = FALSE;
volatile int alarmCount = 0;

void alarmHandler(int signal){
    alarmEnabled = FALSE;
    alarmCount++;
    printf("Alarm #%d\n",alarmCount);
}

int main(int argc, char *argv[])
{ 
    // Program usage: Uses either COM1 or COM2
    const char *serialPortName = argv[1];

    if (argc < 2)
    {
        printf("Incorrect program usage\n"
               "Usage: %s <SerialPort>\n"
               "Example: %s /dev/ttyS1\n",
               argv[0],
               argv[0]);
        exit(1);
    }

    // Open serial port device for reading and writing, and not as controlling tty
    // because we don't want to get killed if linenoise sends CTRL-C.
    int fd = open(serialPortName, O_RDWR | O_NOCTTY);

    if (fd < 0)
    {
        perror(serialPortName);
        exit(-1);
    }

    struct termios oldtio;
    struct termios newtio;

    // Save current port settings
    if (tcgetattr(fd, &oldtio) == -1)
    {
        perror("tcgetattr");
        exit(-1);
    }

    // Clear struct for new port settings
    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    // Set input mode (non-canonical, no echo,...)
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0; // Inter-character timer unused
    newtio.c_cc[VMIN] = 0;  // Blocking read until 5 chars received

    // VTIME e VMIN should be changed in order to protect with a
    // timeout the reception of the following character(s)

    // Now clean the line and activate the settings for the port
    // tcflush() discards data written to the object referred to
    // by fd but not transmitted, or data received but not read,
    // depending on the value of queue_selector:
    //   TCIFLUSH - flushes data received but not read.
    tcflush(fd, TCIOFLUSH);

    // Set new port settings
    if (tcsetattr(fd, TCSANOW, &newtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    printf("New termios structure set\n");

    // Create string to send
    unsigned char buf[BUF_SIZE] = {0};

	buf[0] = 0x7E;
	buf[1] = 0x03;
	buf[2] = 0x03;
	buf[3] = 0x00;
	buf[4] = 0x7E;   

    // In non-canonical mode, '\n' does not end the writing.
    // Test this condition by placing a '\n' in the middle of the buffer.
    // The whole buffer must be sent even with the '\n'.
    buf[5] = '\n';

    // bytes written and bytes_read
    int bytes, bytes_r;

    //Create a buffer to receive UA
    unsigned char buf_r[BUF_SIZE] = {0};

    (void)signal(SIGALRM, alarmHandler);
    
    while(alarmCount < MAX_RETRIES){

        bytes = write(fd, buf, BUF_SIZE);
        printf("%d bytes written\n", bytes);

        // Alarm para 3 secs 
        alarm(TIMEOUT_SEC);
        alarmEnabled = TRUE;

        while(alarmEnabled){
                bytes_r = read(fd, buf_r, BUF_SIZE);
                if(bytes_r > 0){
                    if(buf_r[0] == 0x7E && buf_r[4] == 0x7E && buf_r[1] == 0x03 && buf_r[2] == 0x07 && buf_r[3] == 0x04){
                        printf("Received the UA from the receiver \n");
                        alarm(0);
                        alarmEnabled = FALSE;
                        STOP = TRUE;
                        break;
                    }
                }
            }
        
        if(STOP)
            break;
    }
    
    if (alarmCount >= MAX_RETRIES)
    {
        printf("Failed to receive UA after %d retries\n", MAX_RETRIES);
    }

    // Restore the old port settings
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    close(fd);

    return 0;
}
