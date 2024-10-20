// Application layer protocol implementation

#include "application_layer.h"



void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    if (strcmp(role, "tx") == 0) {
    // Open the file for reading
    // Prepare the control packet for file size
    // Establish a link layer connection
    // Transmit the control packet
    // Read and transmit file data in chunks
    // Transmit the end control packet
    // Close the connection
    int fd = open(filename, O_RDONLY);
        if (fd == -1) {
            printf("Error opening \"%s\".\n", filename);
            return;
        }

        struct stat st;
        stat(filename, &st);


        unsigned char *control_packet = (unsigned char*) malloc (11); // 11?
        control_packet[0] = C_START;
        control_packet[1] = 0;
        control_packet[2] = 8;

          unsigned long size_aux = (unsigned long) st.st_size;   
        for (int i = 7; i >= 0; i--) {
            control_packet[i + 3] = (unsigned char) (0xff & size_aux);
            size_aux >>= 8;
        }

    }
    else if (strcmp(role, "rx") == 0) {
        // Open the file for writing
        // Establish a link layer connection
        // Receive the control packet to get the file size
        // Receive and write data packets until the end control packet is received
        // Close the connection
    }

}
