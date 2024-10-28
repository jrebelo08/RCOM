#include "../include/application_layer.h"
#include "../include/link_layer.h"

#define C_DATA 1
#define C_START 2
#define C_END 3

// Helper function to initialize link layer connection parameters
LinkLayer initializeLinkLayer(const char* serialPort, LinkLayerRole role, int baudRate, int nTries, int timeout) {
    LinkLayer connectionParams;
    strncpy(connectionParams.serialPort, serialPort, sizeof(connectionParams.serialPort) - 1);
    connectionParams.serialPort[sizeof(connectionParams.serialPort) - 1] = '\0';
    connectionParams.role = role;
    connectionParams.baudRate = baudRate;
    connectionParams.nRetransmissions = nTries;
    connectionParams.timeout = timeout;
    return connectionParams;
}

// Helper function to construct START or END control packets
unsigned char* constructControlPacket(unsigned char controlType, unsigned long fileSize) {
    unsigned char* controlPacket = (unsigned char*) calloc(11, sizeof(unsigned char));
    controlPacket[0] = controlType;
    controlPacket[1] = 0; // File size TLV type
    controlPacket[2] = 8; // File size length

    for (int i = 0; i < 8; i++) {
        controlPacket[10 - i] = (unsigned char)(fileSize & 0xFF);
        fileSize >>= 8;
    }
    return controlPacket;
}

// Transmitter: sends data in packets along with START and END control packets
void transmitFileData(int fd, unsigned long fileSize) {
    unsigned char* startPacket = constructControlPacket(C_START, fileSize);
    if (llwrite(startPacket, 11) == -1) {
        perror("Error sending START packet");
        free(startPacket);
        return;
    }
    free(startPacket);

    unsigned char* dataPacket = (unsigned char*) calloc(MAX_PAYLOAD_SIZE, sizeof(unsigned char));
    unsigned int maxDataSize = MAX_PAYLOAD_SIZE - 3;
    unsigned long bytesRemaining = fileSize;

    while (bytesRemaining > 0) {
        unsigned int chunkSize = (bytesRemaining < maxDataSize) ? bytesRemaining : maxDataSize;

        dataPacket[0] = C_DATA;
        dataPacket[1] = (chunkSize >> 8) & 0xFF;
        dataPacket[2] = chunkSize & 0xFF;

        if (read(fd, dataPacket + 3, chunkSize) < chunkSize) {
            perror("Error reading from file");
            break;
        }

        if (llwrite(dataPacket, chunkSize + 3) != (int)(chunkSize + 3)) {
            perror("Error sending data packet");
            break;
        }

        printf("Sent data packet of size %u bytes\n", chunkSize);
        bytesRemaining -= chunkSize;
    }

    free(dataPacket);

    unsigned char* endPacket = constructControlPacket(C_END, fileSize);
    if (llwrite(endPacket, 11) == -1) {
        perror("Error sending END packet");
    }
    free(endPacket);
}

// Receiver: receives data packets and saves them to a file
int receiveFileData(int fd) {
    unsigned char* buffer = (unsigned char*) calloc(MAX_PAYLOAD_SIZE, sizeof(unsigned char));

    // Receive and parse START packet
    if (llread(buffer) == -1 || buffer[0] != C_START) {
        perror("Error receiving START packet");
        free(buffer);
        return -1;
    }

    unsigned long expectedFileSize = 0;
    for (int i = 3; i < 11; i++) {
        expectedFileSize = (expectedFileSize << 8) | buffer[i];
    }

    unsigned long receivedBytes = 0;
    while (1) {
        if (llread(buffer) == -1) {
            perror("Error receiving data packet, retrying...");
            continue;
        }
        if (buffer[0] == C_END) break;

        unsigned int packetSize = (buffer[1] << 8) | buffer[2];
        if (write(fd, buffer + 3, packetSize) != (ssize_t) packetSize) {
            perror("Error writing data to file");
            break;
        }

        receivedBytes += packetSize;
        printf("Received and wrote %u bytes of data\n", packetSize);
    }

    // Validate END packet's file size
    unsigned long endFileSize = 0;
    for (int i = 3; i < 11; i++) {
        endFileSize = (endFileSize << 8) | buffer[i];
    }

    free(buffer);
    return (expectedFileSize == endFileSize) ? 0 : -1;
}

// Main application layer function to handle transmitter and receiver roles
void applicationLayer(const char *serialPort, const char *role, int baudRate, int nTries, int timeout, const char *filename) {
    LinkLayerRole appRole = (strcmp(role, "tx") == 0) ? LlTx : LlRx;
    LinkLayer connectionParams = initializeLinkLayer(serialPort, appRole, baudRate, nTries, timeout);

    if (llopen(connectionParams) == -1) {
        perror("Failed to open link layer connection");
        return;
    }

    if (appRole == LlTx) {
        int fd = open(filename, O_RDONLY);
        if (fd == -1) {
            perror("Error opening file for transmission");
            return;
        }

        struct stat fileStats;
        if (fstat(fd, &fileStats) == -1) {
            perror("Error obtaining file statistics");
            close(fd);
            return;
        }

        printf("Starting file transmission...\n");
        transmitFileData(fd, (unsigned long) fileStats.st_size);
        close(fd);

    } else {
        int fd = open(filename, O_WRONLY | O_CREAT | O_TRUNC, 0666);
        if (fd == -1) {
            perror("Error opening file for reception");
            return;
        }

        printf("Starting file reception...\n");
        if (receiveFileData(fd) == -1) {
            perror("File reception failed");
        }
        close(fd);
    }

    if (llclose(0) == -1) {
        perror("Error closing link layer connection");
    }

    printf("Transmission completed successfully.\n");
}
