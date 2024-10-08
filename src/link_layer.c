// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source
#define BUF_SIZE 8
#define FALSE 0
#define TRUE 1
#define MAX_RETRIES 3

volatile int STOP = FALSE;
int alarmEnabled = FALSE; 
int alarmCount = 0;      

void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;
    printf("Alarm #%d\n", alarmCount);
}

void initializeAlarm() {
    struct sigaction act = {0};
    act.sa_handler = &alarmHandler;
    if (sigaction(SIGALRM, &act, NULL) == -1) {
        perror("sigaction");
        exit(EXIT_FAILURE);
    }
}

void sendUAFrame() {
    char buf_s[BUF_SIZE] = {0};

    // UA Frame
    buf_s[0] = 0x7E; // Flag
    buf_s[1] = 0x03; // Address
    buf_s[2] = 0x07; // Control
    buf_s[3] = 0x04; // BCC
    buf_s[4] = 0x7E; // Flag

    int bytes_s = writeBytes(buf_s, BUF_SIZE);
    printf("%d bytes written (UA Frame)\n", bytes_s);
}

void sendSETFrame() {
    char buf_s[BUF_SIZE] = {0};

    // SET Frame
    buf_s[0] = 0x7E; // Flag
    buf_s[1] = 0x03; // Address
    buf_s[2] = 0x03; // Control
    buf_s[3] = 0x00; // BCC
    buf_s[4] = 0x7E; // Flag

    int bytes_s = writeBytes(buf_s, BUF_SIZE);
    printf("%d bytes written (SET Frame)\n", bytes_s);
}

int llOpenRxStateMachine() {
    enum State { START, FLAG_RCV, A_RCV, C_RCV, BCC_OK, STOP_STATE };
    enum State state = START;
    char buf[BUF_SIZE + 1] = {0}; // +1 for the final '\0' char

    while (STOP == FALSE) {
        int bytes = readByte(buf); 
        if (bytes > 0) {
            switch (state) {
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
        usleep(100000); // microsecond precision
    }

    // Sending UA Frame 
    if (state == STOP_STATE) {
        sendUAFrame();
        return 1; // Success
    }
    return -1; // Failure
}

int llOpenTxStateMachine() {
    enum State { START, FLAG_RCV, A_RCV, C_RCV, BCC_OK, STOP_STATE };
    enum State state = START;
    char buf[BUF_SIZE + 1] = {0}; // +1 for the final '\0' char
    int alarmCount = 0;

    while (STOP == FALSE && alarmCount < MAX_RETRIES) {
        if (!alarmEnabled) {
            sendSETFrame(); // Sending SET Frame
            alarm(3); // Set alarm for 3 seconds
            alarmEnabled = TRUE;
        }

        int bytes = readByte(buf);
        if (bytes > 0) {
            alarm(0); 
            alarmEnabled = FALSE; // Reset the alarm status
            alarmCount = 0; // Reset on successful read

            switch (state) {
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
        } else if (alarmEnabled == FALSE) {
            alarmCount++; // Increment alarm count if no bytes are read
            if (alarmCount >= MAX_RETRIES) {
                printf("Max alarms reached. Aborting.\n");
                return -1; 
            }
        }
    }
    return 1; // Success
}

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

    initializeAlarm();

    if (connectionParameters.role == LlRx) {
        return llOpenRxStateMachine(); 
    } else if (connectionParameters.role == LlTx) {
        return llOpenTxStateMachine(); 
    }

    return -1;
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