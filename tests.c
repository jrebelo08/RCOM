#include "link_layer.h"
#include <stdio.h>
#include <string.h>

int main(int argc, char *argv[]) {
    if (argc != 2) {
        printf("Usage: %s <tx|rx>\n", argv[0]);
        return 1;
    }

    LinkLayer connectionParameters;
    connectionParameters.baudRate = 9600;           
    connectionParameters.nRetransmissions = 3;      
    connectionParameters.timeout = 3;               

    if (strcmp(argv[1], "tx") == 0) {
        strncpy(connectionParameters.serialPort, "/dev/ttyS10", sizeof(connectionParameters.serialPort) - 1);
        connectionParameters.role = LlTx;
        printf("Running as transmitter on /dev/ttyS10\n");
    } else if (strcmp(argv[1], "rx") == 0) {
        strncpy(connectionParameters.serialPort, "/dev/ttyS11", sizeof(connectionParameters.serialPort) - 1);
        connectionParameters.role = LlRx;
        printf("Running as receiver on /dev/ttyS11\n");
    } else {
        printf("Invalid argument. Use 'tx' for transmitter or 'rx' for receiver.\n");
        return 1;
    }

    int result = llopen(connectionParameters);
    if (result == -1) {
        printf("Failed to open the link layer connection.\n");
        return 1;
    } else {
        printf("Link layer connection established.\n");
    }

    if (connectionParameters.role == LlTx) {
        printf("Transmitting data...\n");
    } else if (connectionParameters.role == LlRx) {
        printf("Receiving data...\n");
    }

    if (llclose(1) == -1) {
        printf("Failed to close the link layer connection.\n");
        return 1;
    } else {
        printf("Link layer connection closed successfully.\n");
    }

    return 0;
}
