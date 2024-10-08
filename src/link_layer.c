// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source
#define BUF_SIZE 8

volatile int STOP = false;
////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    if (openSerialPort(connectionParameters.serialPort,
                       connectionParameters.baudRate) < 0)
    {
        return -1;
    }
    enum State { START, FLAG_RCV, A_RCV, C_RCV, BCC_OK, STOP_STATE };
    enum State state = START;
    
    unsigned char buf[BUF_SIZE + 1] = {0}; // +1 space for the final '\0' char

    if(connectionParameters.role == LlRx){
            // Loop for input



        while (STOP == FALSE)
        {
            int bytes = readByte(buf); 
            if (bytes > 0)
            {
                switch (state)
                {
                case START:
                    if (buf[0] == 0x7E) 
                        state = FLAG_RCV;
                    break;
                case FLAG_RCV:
                    if (buf[0] == 0x03) 
                        state = A_RCV;
                    else if (buf[0] != 0x7E)
                        state = START;
                    break;
                case A_RCV:
                    if (buf[0] == 0x03) 
                        state = C_RCV;
                    else if (buf[0] == 0x7E)
                        state = FLAG_RCV;
                    else
                        state = START;
                    break;
                case C_RCV:
                    if (buf[0] == (0x03 ^ 0x03)) 
                        state = BCC_OK;
                    else if (buf[0] == 0x7E)
                        state = FLAG_RCV;
                    else
                        state = START;
                    break;
                case BCC_OK:
                    if (buf[0] == 0x7E) 
                        state = STOP_STATE;
                    else
                        state = START;
                    break;
                case STOP_STATE:
                    STOP = TRUE;
                    break;
                }
            }
            sleep(10);
        }

        // Sending UA Frame 
        if(state == STOP_STATE){
            unsigned char buf_s[BUF_SIZE] = {0};

            // UA Frame
            buf_s[0] = 0x7E;
            buf_s[1] = 0x03;
            buf_s[2] = 0x07;
            buf_s[3] = 0x04;
            buf_s[4] = 0x7E;  
            int bytes_s = writeBytes(buf_s,BUF_SIZE);
            printf("%d bytes written\n",bytes_s);
        }
    }

    if(connectionParameters.role == LlTx){
        if(state == START){
            unsigned char buf_s[BUF_SIZE] = {0};

            // SET Frame
            buf_s[0] = 0x7E;
            buf_s[1] = 0x03;
            buf_s[2] = 0x03;
            buf_s[3] = 0x00;
            buf_s[4] = 0x7E;  
            int bytes_s = writeBytes(buf_s,BUF_SIZE);
            printf("%d bytes written\n",bytes_s);
        }
        while (STOP == FALSE)
        {
            int bytes = readByte(buf); 
            if (bytes > 0)
            {
                switch (state)
                {
                case START:
                    if (buf[0] == 0x7E) 
                        state = FLAG_RCV;
                    break;
                case FLAG_RCV:
                    if (buf[0] == 0x03) 
                        state = A_RCV;
                    else if (buf[0] != 0x7E)
                        state = START;
                    break;
                case A_RCV:
                    if (buf[0] == 0x07) 
                        state = C_RCV;
                    else if (buf[0] == 0x7E)
                        state = FLAG_RCV;
                    else
                        state = START;
                    break;
                case C_RCV:
                    if (buf[0] == (0x03 ^ 0x07)) 
                        state = BCC_OK;
                    else if (buf[0] == 0x7E)
                        state = FLAG_RCV;
                    else
                        state = START;
                    break;
                case BCC_OK:
                    if (buf[0] == 0x7E) 
                        state = STOP_STATE;
                    else
                        state = START;
                    break;
                case STOP_STATE:
                    STOP = TRUE;
                    break;
                }
            }
            sleep(10);
        }
    }

    return 1;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    // TODO

    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    // TODO

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{
    // TODO

    int clstat = closeSerialPort();
    return clstat;
}