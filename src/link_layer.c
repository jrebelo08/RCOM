// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source
#define BUF_SIZE 5
#define FALSE 0
#define TRUE 1

// Frame Control Constants
#define FLAG 0x7E                     // Flag byte
#define ADDR_TX 0x03 // Address field for transmitter to receiver
#define ADDR_RX 0x01 // Address field for receiver to transmitter

// Control Field Constants
#define CTRL_SET 0x03      // Control field for SET frame
#define CTRL_UA 0x07       // Control field for UA frame
#define CTRL_RR0 0XAA      // Control fiel for RR0 frame 0
#define CTRL_RR1 0xAB       // Control field for RR1 frame 1 
#define CTRL_REJ0 0x54      // Control field for REJ0 frame 0 
#define CTRL_REJ1 0x55      // Control field for REJ1 frame 1 
#define CTRL_DISC 0x0B     // Control field for DISC (Disconnect)
#define CTRL_I_0 0x00      // Control field for I-frame with sequence number 0
#define CTRL_I_1 0x40      // Control field for I-frame with sequence number 1

// Byte stuffing
#define ESC 0x7d

volatile int STOP = FALSE;
int alarmEnabled = FALSE; 
int alarmCount = 0;
int frame_number = 0;
LinkLayerRole role;      
int timeout = 0;
int retransmissions = 0;
static int numFramesSent = 0;
static int numRetransmissions = 0;
static int numTimeouts = 0;

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

void handleAlarm() {
    alarmCount++;
    if (alarmCount >= retransmissions) {
        numTimeouts++;
    }
    initializeAlarm();
    alarm(timeout);
    alarmEnabled = TRUE;
    if (alarmCount > 0) {
        numRetransmissions++;
    }
}

void sendFrame(unsigned char controlByte, const char *frameType) {
    char buf_s[BUF_SIZE] = {0};

    // Constructing the frame
    buf_s[0] = FLAG;                  // Flag
    buf_s[1] = ADDR_TX;               // Address
    buf_s[2] = controlByte;           // Control
    buf_s[3] = ADDR_TX ^ controlByte; // BCC
    buf_s[4] = FLAG;                  // Flag

    int bytes_s = writeBytes(buf_s, BUF_SIZE);
    numFramesSent++;
    printf("%d bytes written (%s Frame)\n", bytes_s, frameType);
}

void sendUAFrame() {
    sendFrame(CTRL_UA, "UA");
}

void sendSETFrame() {
    sendFrame(CTRL_SET, "SET");
}

void sendDISCFrame() {
    sendFrame(CTRL_DISC, "DISC");
}

void sendRRFrame(int seq) {
    if (seq == 0) {
        sendFrame(CTRL_RR0, "RR0");
    } else if (seq == 1) {
        sendFrame(CTRL_RR1, "RR1");
    } 
}

void sendREJFrame(int seq) {
    if (seq == 0) {
        sendFrame(CTRL_REJ0, "REJ0");
    } else if (seq == 1) {
        sendFrame(CTRL_REJ1, "REJ1");
    } 
}

void sendIFrame(int seq) {
    if (seq == 0) {
        sendFrame(CTRL_I_0, "I0");
    } else if (seq == 1) {
        sendFrame(CTRL_I_1, "I1");
    }
}

int handleStateMachine(LinkLayerState *state, unsigned char byte, unsigned char expectedAddr, unsigned char expectedCtrl, unsigned char expectedBCC) {
    switch (*state) {
        case START:
            if (byte == FLAG) *state = FLAG_RCV;
            break;
        case FLAG_RCV:
            if (byte == expectedAddr) *state = A_RCV;
            else if (byte != FLAG) *state = START;
            break;
        case A_RCV:
            if (byte == expectedCtrl) *state = C_RCV;
            else if (byte == FLAG) *state = FLAG_RCV;
            else *state = START;
            break;
        case C_RCV:
            if (byte == expectedBCC) *state = BCC_OK;
            else if (byte == FLAG) *state = FLAG_RCV;
            else *state = START;
            break;
        case BCC_OK:
            if (byte == FLAG) {
                *state = STOP_STATE;
                STOP = TRUE;
            } else {
                *state = START;
            }
            break;
    }
    return (*state == STOP_STATE) ? 1 : -1;
}

int llOpenRx() {
    LinkLayerState state = START;
    unsigned char byte;
    STOP = FALSE;

    while (STOP == FALSE) {
        if (readByte((char *)&byte) > 0) {
            if (handleStateMachine(&state, byte, ADDR_TX, CTRL_SET, ADDR_TX ^ CTRL_SET) == 1) {
                sendUAFrame();
                return 1;
            }
        }
    }
    return -1;
}

int llOpenTx() {
    LinkLayerState state = START;
    unsigned char byte;
    STOP = FALSE;

    sendSETFrame();
    initializeAlarm();
    alarm(timeout);
    alarmEnabled = TRUE;

    while (STOP == FALSE && alarmCount < retransmissions) {
        if (!alarmEnabled) {
            handleAlarm();
            if (alarmCount >= retransmissions) return -1;
            sendSETFrame();
        }

        if (readByte((char *)&byte) > 0) {
            if (handleStateMachine(&state, byte, ADDR_TX, CTRL_UA, ADDR_TX ^ CTRL_UA) == 1) {
                alarm(0);
                alarmEnabled = FALSE;
                alarmCount = 0;
                return 1;
            }
        }
    }
    return -1;
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters) {
    role = connectionParameters.role;
    retransmissions = connectionParameters.nRetransmissions;
    timeout = connectionParameters.timeout;

    if (openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate) < 0) {
        return -1;
    }

    if (connectionParameters.role == LlRx) {
        return llOpenRx();
    } else if (connectionParameters.role == LlTx) {
        return llOpenTx();
    }

    return -1;
}


int handleLlwriteStateTransition(LinkLayerState *state, unsigned char byte, unsigned char *cField, int *reject) {
    switch (*state) {
        case START:
            if (byte == FLAG) *state = FLAG_RCV;
            break;
        case FLAG_RCV:
            if (byte == ADDR_TX) *state = A_RCV;
            else if (byte != FLAG) *state = START;
            break;
        case A_RCV:
            if (byte == CTRL_RR0 || byte == CTRL_RR1 || byte == CTRL_REJ0 || byte == CTRL_REJ1 || byte == CTRL_DISC) {
                *state = C_RCV;
                *cField = byte;
            } else if (byte == FLAG) {
                *state = FLAG_RCV;
            } else {
                *state = START;
            }
            break;
        case C_RCV:
            if (byte == (ADDR_TX ^ *cField)) {
                *state = BCC_OK;
                *reject = (byte == CTRL_REJ0 || byte == CTRL_REJ1) ? 1 : 0;
            } else if (byte == FLAG) {
                *state = FLAG_RCV;
            } else {
                *state = START;
            }
            break;
        case BCC_OK:
            return 1; 
        default:
            break;
    }
    return 0; 
}


unsigned char* buildFrame(const unsigned char *buf, int bufSize, int *frameSize) {
    int oldFrameSize = bufSize + 6;
    unsigned char *oldFrame = (unsigned char*) malloc(oldFrameSize);

    oldFrame[0] = FLAG;
    oldFrame[1] = ADDR_TX;
    oldFrame[2] = frame_number == 0 ? CTRL_I_0 : CTRL_I_1;
    oldFrame[3] = ADDR_TX ^ (frame_number == 0 ? CTRL_I_0 : CTRL_I_1);

    unsigned char BCC2 = 0;
    for (int i = 0; i < bufSize; i++) {
        oldFrame[i + 4] = buf[i];
        BCC2 ^= buf[i];
    }

    oldFrame[bufSize + 4] = BCC2;
    oldFrame[bufSize + 5] = FLAG;

    *frameSize = oldFrameSize;
    return oldFrame;
}

unsigned char* byteStuffing(const unsigned char *oldFrame, int oldFrameSize, int *newFrameSize) {
    unsigned char *newFrame = (unsigned char*) malloc(oldFrameSize * 2);

    int newSize = 0;
    for (int j = 0; j < 4; j++) {
        newFrame[newSize++] = oldFrame[j];
    }

    for (int j = 4; j < oldFrameSize - 1; j++) {
        if (oldFrame[j] == FLAG) {
            newFrame[newSize++] = ESC;
            newFrame[newSize++] = 0x5e;
        } else if (oldFrame[j] == ESC) {
            newFrame[newSize++] = ESC;
            newFrame[newSize++] = 0x5d;
        } else {
            newFrame[newSize++] = oldFrame[j];
        }
    }

    newFrame[newSize++] = FLAG;

    newFrame = realloc(newFrame, newSize);
    *newFrameSize = newSize;

    return newFrame;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize) {
    LinkLayerState state = START;
    int reject = 0;
    unsigned char byte;
    unsigned char cField;

    int oldFrameSize;
    unsigned char *oldFrame = buildFrame(buf, bufSize, &oldFrameSize);

    int newFrameSize;
    unsigned char *newFrame = byteStuffing(oldFrame, oldFrameSize, &newFrameSize);

    free(oldFrame);
    STOP = FALSE;

    initializeAlarm();
    alarm(timeout);
    alarmEnabled = TRUE;

    writeBytes((const char *)newFrame, newFrameSize);
    numFramesSent++;

    printf("llwrite: Frame sent, size = %d, frame_number = %d\n", newFrameSize, frame_number);

    while (STOP == FALSE && alarmCount < retransmissions) {
        if (!alarmEnabled) {
            handleAlarm();
            if (alarmCount >= retransmissions) {
                free(newFrame);
                return -1;
            }
            writeBytes((const char *)newFrame, newFrameSize);
            numFramesSent++;
            printf("llwrite: Retransmitted frame, size = %d\n", newFrameSize);
        }

        if (readByte((char *)&byte) > 0) {
            printf("llwrite: Byte received = 0x%02X, current state = %d\n", byte, state);
            if (handleLlwriteStateTransition(&state, byte, &cField, &reject)) {
                alarm(0);
                alarmEnabled = FALSE;
                free(newFrame);
                frame_number = 1 - frame_number;
                STOP = TRUE;
                printf("llwrite: Frame acknowledged, transmission successful.\n");
                return bufSize;
            }
        }

        if (alarmCount >= retransmissions) {
            STOP = TRUE;
            printf("llwrite: Maximum retransmissions reached, transmission failed.\n");
            break;
        }
    }

    if (reject) {
        handleAlarm();
        writeBytes((const char *)newFrame, newFrameSize);
        numFramesSent++;
        printf("llwrite: Resending on Start frame, size = %d\n", newFrameSize);
    }

    free(newFrame);
    return -1;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet) {
    enum ReadFrame {
        START,          
        FLAG_RCV,       
        A_RCV,          
        C_RCV,          
        READING_DATA,   
        DATA_ESCAPED,   
        STOP_STATE      
    } state = START;
    
    unsigned char byte;
    unsigned char controlField;
    int dataIdx = 0;

    printf("llread: Waiting to receive frame...\n");

    while (state != STOP_STATE) {
        if (readByte((char *)&byte)) {
            printf("llread: Byte received = 0x%02X, current state = %d\n", byte, state);
            switch (state) {
                case START:
                    if (byte == FLAG) 
                        state = FLAG_RCV;
                    break;

                case FLAG_RCV:
                    if (byte == ADDR_TX) {
                        state = A_RCV;
                    } else if (byte != FLAG) {
                        state = START;
                    }
                    break;

                case A_RCV:
                    if (byte == CTRL_I_0 || byte == CTRL_I_1) {
                        controlField = byte;
                        state = C_RCV;
                        printf("llread: Control field detected = 0x%02X\n", controlField);
                    } else if (byte == FLAG) {
                        state = FLAG_RCV;
                    } else if (byte == CTRL_DISC) {
                        sendDISCFrame();
                        printf("llread: DISC frame received, closing connection.\n");
                        return 0;
                    } else {
                        state = START;
                    }
                    break;

                case C_RCV:
                    if (byte == (ADDR_TX ^ controlField)) {
                        state = READING_DATA;
                        printf("llread: BCC1 check passed.\n");
                    } else if (byte == FLAG) {
                        state = FLAG_RCV;
                    } else {
                        state = START;
                    }
                    break;

                case READING_DATA:
                    if (byte == FLAG) {
                        unsigned char bcc2 = packet[dataIdx - 1];  
                        dataIdx--;  
                        packet[dataIdx] = '\0';  

                        unsigned char acc = packet[0];  
                        for (int j = 1; j < dataIdx; j++)
                            acc ^= packet[j];  

                        if (bcc2 == acc) {
                            state = STOP_STATE;
                            sendRRFrame(frame_number);
                            frame_number = (frame_number + 1) % 2;
                            return dataIdx;  
                        } else {
                            printf("Error: BCC2 check failed, retransmission needed.\n");
                            sendREJFrame(frame_number);
                            return -1;  
                        }
                    } else if (byte == ESC) {
                        state = DATA_ESCAPED;  
                    } else {
                        packet[dataIdx++] = byte; 
                    }
                    break;

                case DATA_ESCAPED:
                    state = READING_DATA; 
                    if (byte == (FLAG ^ 0x20)) {
                        packet[dataIdx++] = FLAG;  
                    } else if (byte == (ESC ^ 0x20)) {
                        packet[dataIdx++] = ESC; 
                    } else {
                        packet[dataIdx++] = ESC;  
                        packet[dataIdx++] = byte; 
                    }
                    break;

                default:
                    state = START;
                    break;
            }
        }
    }
    return -1;  
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics) {

    if (role == LlTx) {
        sendDISCFrame(); 

        LinkLayerState state = START;
        unsigned char byte;
        alarmCount = 0;

        initializeAlarm();
        alarm(timeout); 
        alarmEnabled = TRUE;

        while (STOP == FALSE) {
            if (!alarmEnabled) {  
                handleAlarm();
                if (alarmCount >= retransmissions) {
                    return -1;
                }
                sendDISCFrame();
            }

            if (readByte((char*)&byte) > 0) {
                if (handleStateMachine(&state, byte, ADDR_RX, CTRL_DISC, ADDR_RX ^ CTRL_DISC) == 1) {
                    alarm(0);
                    alarmEnabled = FALSE;
                    STOP = TRUE;
                }
            }
        }

        sendUAFrame();

    } else if (role == LlRx) {
        LinkLayerState state = START;
        unsigned char byte;
        STOP = FALSE;

        while (STOP == FALSE) {
            if (readByte((char*)&byte) > 0) {
                if (handleStateMachine(&state, byte, ADDR_TX, CTRL_DISC, ADDR_TX ^ CTRL_DISC) == 1) {
                    STOP = TRUE;
                }
            }
        }

        sendDISCFrame();

        state = START;
        handleAlarm();

        while (STOP == FALSE) {
            if (!alarmEnabled) {  
                handleAlarm();
                if (alarmCount >= retransmissions) {
                    return -1;
                }
                sendDISCFrame();
            }

            if (readByte((char*)&byte) > 0) {
                if (handleStateMachine(&state, byte, ADDR_RX, CTRL_UA, ADDR_RX ^ CTRL_UA) == 1) {
                    alarm(0); 
                    alarmEnabled = FALSE;
                    STOP = TRUE;
                }
            }
        }
    }

    closeSerialPort();

    if (showStatistics) {
        printf("Statistics:\n");
        printf("Frames Sent: %d\n", numFramesSent);
        printf("Retransmissions: %d\n", numRetransmissions);
        printf("Timeouts: %d\n", numTimeouts);
    }

    return 1;
}
