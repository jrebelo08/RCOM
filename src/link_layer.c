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
#define CTRL_RR0 0X05      // Control fiel for RR0 frame 0
#define CTRL_RR1 0x85       // Control field for RR1 frame 1 
#define CTRL_REJ0 0x01      // Control field for REJ0 frame 0 
#define CTRL_REJ1 0x81      // Control field for REJ1 frame 1 
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

int llOpenRxStateMachine() {
    LinkLayerState state = START;
    char buf[BUF_SIZE + 1] = {0}; 

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
    }

    if (state == STOP_STATE) {
        sendUAFrame();
        return 1; 
    }
    return -1; 
}

int llOpenTxStateMachine() {
    LinkLayerState state = START;
    char buf[BUF_SIZE + 1] = {0}; // +1 for the final '\0' char
    int alarmCount = 0;

    STOP = FALSE;

    sendSETFrame(); 

    initializeAlarm();
    alarm(timeout);       
    alarmEnabled = TRUE;

    while (STOP == FALSE && alarmCount < retransmissions) {
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
                alarm(0);
                alarmEnabled = FALSE;
                alarmCount = 0;
                break;
            }
        } else if (!alarmEnabled) {
            alarmCount++;
            if (alarmCount >= retransmissions) {
                numTimeouts++;
                return -1; 
            }
            sendSETFrame();
            initializeAlarm();
            alarm(timeout);
            alarmEnabled = TRUE; 
            if(alarmCount > 0){
                numRetransmissions++;
            }
        }
    }
    return 1;
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    role = connectionParameters.role;
    retransmissions = connectionParameters.nRetransmissions;
    timeout = connectionParameters.timeout;
    
    if (openSerialPort(connectionParameters.serialPort,
                       connectionParameters.baudRate) < 0)
    {
        return -1;
    }

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

    STOP = FALSE;

    while (STOP == FALSE && alarmCount < retransmissions) {
        // Transmit the frame to rx
        writeBytes((const char *)stuffedFrame, stuffedSize); // Send the stuffed frame
        numFramesSent++;
        initializeAlarm();
        alarm(timeout); 
        alarmEnabled = TRUE;

        LinkLayerState state = START;
        unsigned char byte;
        int reject = 0;

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
                    if (frame_number == 0 && byte == CTRL_RR0) {
                        state = C_RCV;
                        reject = 0; 
                    }
                    else if (frame_number == 0 && byte == CTRL_REJ0) {
                        state = C_RCV;
                        reject = 1; 
                        if(alarmCount > 0){
                            numRetransmissions++;
                        }
                    }
                    else if (frame_number == 1 && byte == CTRL_RR1) {
                        state = C_RCV;
                        reject = 0; 
                    }
                    else if (frame_number == 1 && byte == CTRL_REJ1) {
                        state = C_RCV;
                        reject = 1;
                        if(alarmCount > 0){
                            numRetransmissions++;
                        }
                    }
                    else if (byte == FLAG) {
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
                        if (reject == 0) {
                            alarm(0); 
                            alarmEnabled = FALSE; 
                            free(stuffedFrame); 
                            frame_number = 1 - frame_number; 
                            state = STOP_STATE;
                            return bufSize; // Successful write
                        } else { 
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
        if (alarmCount >= retransmissions) {
            numTimeouts++;
            STOP = TRUE;
            break; 
        }
    }

    free(stuffedFrame); 
    return -1; 
}

static int checkBCC2(const unsigned char *data, int length, unsigned char bcc2) {
    unsigned char calculatedBCC2 = data[0];
    for (int j = 1; j < length; j++) {
        calculatedBCC2 ^= data[j];
    }
    return calculatedBCC2 == bcc2;
}
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

    while (state != STOP_STATE) {
        if(readByte((char *)&byte)){

            switch (state) {
                case START:
                    if (byte == FLAG) state = FLAG_RCV; 
                    break;

                case FLAG_RCV:
                    if (byte == ADDR_RX) {
                        state = A_RCV;  
                    } else if (byte != FLAG) {
                        state = START;  
                    }
                    break;

                case A_RCV:
                    if (byte == CTRL_I_0 || byte == CTRL_I_1) {
                        controlField = byte; 
                        state = C_RCV;
                    } else if (byte == FLAG) {
                        state = FLAG_RCV; 
                    } else if (byte == CTRL_DISC) {
                        sendDISCFrame();
                        return 0;  
                    } else {
                        state = START;  
                    }
                    break;

                case C_RCV:
                    if (byte == (ADDR_RX ^ controlField)) {
                        state = READING_DATA; 
                    } else if (byte == FLAG) {
                        state = FLAG_RCV; 
                    } else {
                        state = START;  
                    }
                    break;

                case READING_DATA:
                    if (byte == ESC) {
                        state = DATA_ESCAPED;  
                    } else if (byte == FLAG) {
                    
                        unsigned char bcc2 = packet[dataIdx - 1];
                        dataIdx--;  

                        if (checkBCC2(packet, dataIdx, bcc2)) {
                            sendRRFrame((controlField == CTRL_I_0) ? 0 : 1);
                            frame_number = 1 - frame_number; 
                            return dataIdx; 
                        } else {
                            sendREJFrame((controlField == CTRL_I_0) ? 0 : 1);
                            return -1;  
                        }
                    } else {
                        packet[dataIdx++] = byte;
                    }
                    break;

                case DATA_ESCAPED:
                    state = READING_DATA;  
                    if (byte == ESC || byte == FLAG) {
                        packet[dataIdx++] = byte;  
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
                alarmCount++;
                if (alarmCount >= retransmissions) {
                    numTimeouts++;
                    return -1;
                }
                
                sendDISCFrame();
                if(alarmCount > 0){
                    numRetransmissions++;
                }

                initializeAlarm();
                alarm(timeout);
                alarmEnabled = TRUE;
            }

            if (readByte((char*)&byte) > 0) {

                switch (state) {
                    case START:
                        if (byte == FLAG) state = FLAG_RCV;
                        break;
                    case FLAG_RCV:
                        if (byte == ADDR_RX) state = A_RCV;
                        else if (byte != FLAG) state = START;
                        break;
                    case A_RCV:
                        if (byte == CTRL_DISC) state = C_RCV;
                        else if (byte == FLAG) state = FLAG_RCV;
                        else state = START;
                        break;
                    case C_RCV:
                        if (byte == (ADDR_RX ^ CTRL_DISC)) state = BCC_OK;
                        else if (byte == FLAG) state = FLAG_RCV;
                        else state = START;
                        break;
                    case BCC_OK:
                        if (byte == FLAG) {
                            state = STOP_STATE;
                        } else {
                            state = START;
                        }
                        break;
                    case STOP_STATE:
                        alarm(0);
                        alarmEnabled = FALSE;
                        alarmCount = 0;
                        STOP = TRUE;
                        break;
                }
            }
        }

        sendUAFrame();

    } else if (role == LlRx) {
        LinkLayerState state = START;
        unsigned char byte;
        alarmCount = 0;

        while (STOP == FALSE) {
            if (readByte((char*)&byte) > 0) {
                switch (state) {
                    case START:
                        if (byte == FLAG) state = FLAG_RCV;
                        break;
                    case FLAG_RCV:
                        if (byte == ADDR_TX) state = A_RCV;
                        else if (byte != FLAG) state = START;
                        break;
                    case A_RCV:
                        if (byte == CTRL_DISC) state = C_RCV;
                        else if (byte == FLAG) state = FLAG_RCV;
                        else state = START;
                        break;
                    case C_RCV:
                        if (byte == (ADDR_TX ^ CTRL_DISC)) state = BCC_OK;
                        else if (byte == FLAG) state = FLAG_RCV;
                        else state = START;
                        break;
                    case BCC_OK:
                        if (byte == FLAG) {
                            state = STOP_STATE;
                        } else {
                            state = START;
                        }
                        break;
                    case STOP_STATE:
                        STOP = TRUE;
                        break;
                }
            }
        }

        sendDISCFrame();
        numFramesSent++;

        state = START;
        initializeAlarm();
        alarm(timeout); 
        alarmEnabled = TRUE;

        while (STOP == FALSE) {
            if (!alarmEnabled) {  
                alarmCount++;
                if (alarmCount >= retransmissions) {
                    numTimeouts++;
                    return -1;
                }
                
                sendDISCFrame();
                numFramesSent++;
                if(alarmCount > 0){
                    numRetransmissions++;
                }
                initializeAlarm();
                alarm(timeout); 
                alarmEnabled = TRUE;
            }

            if (readByte((char*)&byte) > 0) {

                switch (state) {
                    case START:
                        if (byte == FLAG) state = FLAG_RCV;
                        break;
                    case FLAG_RCV:
                        if (byte == ADDR_RX) state = A_RCV;
                        else if (byte != FLAG) state = START;
                        break;
                    case A_RCV:
                        if (byte == CTRL_UA) state = C_RCV;
                        else if (byte == FLAG) state = FLAG_RCV;
                        else state = START;
                        break;
                    case C_RCV:
                        if (byte == (ADDR_RX ^ CTRL_UA)) state = BCC_OK;
                        else if (byte == FLAG) state = FLAG_RCV;
                        else state = START;
                        break;
                    case BCC_OK:
                        if (byte == FLAG) {
                            state = STOP_STATE;
                        } else {
                            state = START;
                        }
                        break;
                    case STOP_STATE:
                        alarm(0); 
                        alarmEnabled = FALSE;
                        alarmCount = 0;
                        STOP = TRUE;
                        break;
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
