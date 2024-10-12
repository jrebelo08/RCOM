// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source
#define BUF_SIZE 8
#define FALSE 0
#define TRUE 1
#define MAX_RETRIES 3

// Frame Control Constants
#define FLAG 0x7E                     // Flag byte
#define ADDR_TX 0x03 // Address field for transmitter to receiver
#define ADDR_RX 0x01 // Address field for receiver to transmitter

// Control Field Constants
#define CTRL_SET 0x03      // Control field for SET frame
#define CTRL_UA 0x07       // Control field for UA frame
#define CTRL_RR1 0x05       // Control field for RR (Receiver Ready)
#define CTRL_REJ0 0x01      // Control field for REJ (Reject)
#define CTRL_DISC 0x0B     // Control field for DISC (Disconnect)
#define CTRL_I_0 0x00      // Control field for I-frame with sequence number 0
#define CTRL_I_1 0x40      // Control field for I-frame with sequence number 1

// Byte stuffing
#define ESC 0x7d

volatile int STOP = FALSE;
int alarmEnabled = FALSE; 
int alarmCount = 0;
int frame_number = 0;      

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
    buf_s[0] = FLAG; // Flag
    buf_s[1] = ADDR_TX; // Address
    buf_s[2] = CTRL_UA; // Control
    buf_s[3] = ADDR_TX ^ CTRL_UA; // BCC
    buf_s[4] = FLAG; // Flag

    int bytes_s = writeBytes(buf_s, BUF_SIZE);
    printf("%d bytes written (UA Frame)\n", bytes_s);
}

void sendSETFrame() {
    char buf_s[BUF_SIZE] = {0};

    // SET Frame
    buf_s[0] = FLAG; // Flag
    buf_s[1] = ADDR_TX; // Address
    buf_s[2] = CTRL_SET; // Control
    buf_s[3] = ADDR_TX ^ CTRL_SET; // BCC
    buf_s[4] = FLAG; // Flag

    int bytes_s = writeBytes(buf_s, BUF_SIZE);
    printf("%d bytes written (SET Frame)\n", bytes_s);
}

int llOpenRxStateMachine() {
    enum State { START, FLAG_RCV, A_RCV, C_RCV, BCC_OK, STOP_STATE };
    enum State state = START;
    char buf[BUF_SIZE + 1] = {0}; // +1 for the final '\0' char

    STOP = FALSE;

    while (STOP == FALSE) {
        int bytes = readByte(buf); 
        if (bytes > 0) {
            switch (state) {
            case START:
                if (buf[0] == FLAG) 
                    state = FLAG_RCV;
                break;
            case FLAG_RCV:
                if (buf[0] == ADDR_TX) 
                    state = A_RCV;
                else if (buf[0] != FLAG)
                    state = START;
                break;
            case A_RCV:
                if (buf[0] == CTRL_SET) 
                    state = C_RCV;
                else if (buf[0] == FLAG)
                    state = FLAG_RCV;
                else
                    state = START;
                break;
            case C_RCV:
                if (buf[0] == (ADDR_TX ^ CTRL_SET)) 
                    state = BCC_OK;
                else if (buf[0] == FLAG)
                    state = FLAG_RCV;
                else
                    state = START;
                break;
            case BCC_OK:
                if (buf[0] == FLAG) 
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

    STOP = FALSE;

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
                if (buf[0] == FLAG) 
                    state = FLAG_RCV;
                break;
            case FLAG_RCV:
                if (buf[0] == ADDR_TX) 
                    state = A_RCV;
                else if (buf[0] != FLAG)
                    state = START;
                break;
            case A_RCV:
                if (buf[0] == CTRL_UA) 
                    state = C_RCV;
                else if (buf[0] == FLAG)
                    state = FLAG_RCV;
                else
                    state = START;
                break;
            case C_RCV:
                if (buf[0] == (ADDR_TX ^ CTRL_UA)) 
                    state = BCC_OK;
                else if (buf[0] == FLAG)
                    state = FLAG_RCV;
                else
                    state = START;
                break;
            case BCC_OK:
                if (buf[0] == FLAG) 
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

unsigned char* buildFrame(const unsigned char *buf, int bufSize, int *frameSize) {
    *frameSize = bufSize + 6;
    unsigned char *frame = (unsigned char *)malloc(*frameSize);

    frame[0] = FLAG; // Start flag
    frame[1] = ADDR_TX; // Address field
    frame[2] = (frame_number == 0) ? CTRL_I_0 : CTRL_I_1; // Control field
    frame[3] = frame[1] ^ frame[2]; // BCC1

    unsigned char bcc2 = 0;
    for (int i = 0; i < bufSize; i++) {
        frame[i + 4] = buf[i];
        bcc2 ^= buf[i];
    }
    frame[bufSize + 4] = bcc2; // BCC2
    frame[bufSize + 5] = FLAG; // End flag

    return frame;
}

unsigned char* byteStuffing(const unsigned char *frame, int frameSize, int *stuffedSize) {
    unsigned char *stuffedFrame = (unsigned char *)malloc(frameSize * 2); 
    *stuffedSize = 0;

    for (int i = 0; i < frameSize; i++) {
        if (frame[i] == FLAG) {
            stuffedFrame[(*stuffedSize)++] = ESC;
            stuffedFrame[(*stuffedSize)++] = 0x5e; // FLAG substitution
        } else if (frame[i] == ESC) {
            stuffedFrame[(*stuffedSize)++] = ESC;
            stuffedFrame[(*stuffedSize)++] = 0x5d; // ESC substitution
        } else {
            stuffedFrame[(*stuffedSize)++] = frame[i];
        }
    }
    return stuffedFrame;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize) {
    
    int frameSize;
    unsigned char *frame = buildFrame(buf, bufSize, &frameSize);

    int stuffedSize;
    unsigned char *stuffedFrame = byteStuffing(frame, frameSize, &stuffedSize);
    free(frame); // Free original frame memory

    initializeAlarm(); // Initialize the alarm handler
    alarmCount = 0;

    STOP = FALSE;

    while (STOP == FALSE && alarmCount < MAX_RETRIES) {
        // Transmit the frame
        writeBytes((const char *)stuffedFrame, stuffedSize); // Send the stuffed frame
        alarm(3); // Set alarm for 3 seconds
        alarmEnabled = TRUE;

        // Acknowledgment state machine
        enum { START, FLAG_RCV, A_RCV, C_RCV, BCC_OK, STOP_STATE } state = START;
        unsigned char byte;
        int reject = 0;

        while (alarmEnabled) { // Loop until alarm goes off or acknowledgment is received
            if (readByte((char *) &byte) > 0) {
                switch (state) {
                    case START:
                        if (byte == FLAG) state = FLAG_RCV;
                        break;
                    case FLAG_RCV:
                        if (byte == ADDR_TX) state = A_RCV;
                        else if (byte != FLAG) state = START;
                        break;
                    case A_RCV:
                        if (frame_number == 0 && byte == CTRL_RR1) {
                            state = C_RCV;
                            reject = 0; // Positive acknowledgment
                        } else if (frame_number == 1 && byte == CTRL_REJ0) {
                            state = C_RCV;
                            reject = 1; // Negative acknowledgment (REJ)
                        } else if (byte == FLAG) {
                            state = FLAG_RCV;
                        } else {
                            state = START;
                        }
                        break;
                    case C_RCV:
                        if (byte == (ADDR_TX ^ ((reject == 0) ? CTRL_RR1 : CTRL_REJ0))) {
                            state = BCC_OK;
                        } else if (byte == FLAG) {
                            state = FLAG_RCV;
                        } else {
                            state = START;
                        }
                        break;
                    case BCC_OK:
                        if (byte == FLAG) {
                            if (reject == 0) { // Positive acknowledgment (RR)
                                alarm(0); // Reset the alarm
                                alarmEnabled = FALSE; // Acknowledgment received
                                free(stuffedFrame); // Free allocated memory
                                frame_number = 1 - frame_number; // Toggle frame number for next transmission
                                state = STOP_STATE;
                                return bufSize; // Successful write
                            } else { // Negative acknowledgment (REJ)
                                state = START; // Restart the loop for resending
                            }
                        } else {
                            state = START; // Unexpected byte, reset state
                        }
                        break;
                        case STOP_STATE:
                            STOP = TRUE;
                        break;
                }
            }
        }

        // Alarm triggered or maximum retries reached, retry transmission
        if (alarmCount >= MAX_RETRIES) {
            printf("Max retries reached. Transmission failed.\n");
            break; // Exit if maximum retries are reached
        }
    }

    free(stuffedFrame); // Free allocated memory after all attempts
    return -1; // Failed to send after max retries
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