// RCOM 2026/2027
//
// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

// Control field (C) of the packets
#define C_START 1
#define C_DATA 2
#define C_END 3
// Types (T) of the TLV parameters of the control packets
#define TLV_FILE_SIZE 0
#define TLV_FILE_NAME 1
// A DATA packet is: C, L2, L1, then K bytes of the file (K = 256*L2 + L1)
#define DATA_HEADER 3

static double secondsSince(const struct timespec *start)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (now.tv_sec - start->tv_sec) + (now.tv_nsec - start->tv_nsec) / 1e9;
}

// Prints how fast the file went through the link. S = R/C is the "efficiency" used in the report
// (R = received bit rate, C = link capacity = baud rate).
static void printThroughput(long fileSize, double seconds, int baudRate)
{
    if (seconds <= 0) return;
    double R = fileSize * 8.0 / seconds;
    printf("%ld bytes in %.2f s -> R = %.0f bit/s, C = %d bit/s, S = R/C = %.3f\n",
           fileSize, seconds, R, baudRate, R / baudRate);
}

// Builds a control packet: C | T=0 L size | T=1 L name
// The size is written with as few bytes as needed, most significant first.
// Returns the number of bytes of the packet.
static int buildControlPacket(unsigned char *packet, unsigned char c, long fileSize, const char *fileName)
{
    int n = 0;
    packet[n++] = c;

    int sizeBytes = 0;
    for (long tmp = fileSize; tmp > 0; tmp >>= 8) sizeBytes++;
    if (sizeBytes == 0) sizeBytes = 1;

    packet[n++] = TLV_FILE_SIZE;
    packet[n++] = sizeBytes;
    for (int i = sizeBytes - 1; i >= 0; i--) packet[n++] = (fileSize >> (8 * i)) & 0xFF;

    int nameLen = strlen(fileName);
    if (nameLen > 255) nameLen = 255;
    packet[n++] = TLV_FILE_NAME;
    packet[n++] = nameLen;
    memcpy(packet + n, fileName, nameLen);
    n += nameLen;

    return n;
}

// Reads the TLV parameters of a control packet. fileName must have room for 256 bytes.
// Returns 0 on success, -1 if the packet is malformed or has no file size.
static int parseControlPacket(const unsigned char *packet, int len, long *fileSize, char *fileName)
{
    int i = 1; // packet[0] is C
    int gotSize = FALSE;
    fileName[0] = '\0';

    while (i + 2 <= len)
    {
        unsigned char t = packet[i];
        unsigned char l = packet[i + 1];
        i += 2;
        if (i + l > len) return -1; // value goes past the end of the packet

        if (t == TLV_FILE_SIZE)
        {
            if (l == 0 || l > sizeof(long)) return -1;
            long v = 0;
            for (int k = 0; k < l; k++) v = (v << 8) | packet[i + k];
            *fileSize = v;
            gotSize = TRUE;
        }
        else if (t == TLV_FILE_NAME)
        {
            memcpy(fileName, packet + i, l);
            fileName[l] = '\0';
        }
        // other types: skipped
        i += l;
    }
    return gotSize ? 0 : -1;
}

// START packet -> DATA packets -> END packet. The link must already be open.
static int sendFile(const char *filename, int baudRate)
{
    FILE *file = fopen(filename, "rb");
    if (file == NULL)
    {
        perror("fopen");
        return -1;
    }
    fseek(file, 0, SEEK_END);
    long fileSize = ftell(file);
    rewind(file);

    // only the name goes in the packet, not the directories
    const char *baseName = strrchr(filename, '/');
    baseName = (baseName == NULL) ? filename : baseName + 1;

    struct timespec start;
    clock_gettime(CLOCK_MONOTONIC, &start);

    unsigned char packet[MAX_PAYLOAD_SIZE];

    int len = buildControlPacket(packet, C_START, fileSize, baseName);
    if (llSend(packet, len) < 0)
    {
        printf("Could not send the START packet\n");
        fclose(file);
        return -1;
    }

    long sent = 0;
    size_t n;
    // read the next piece of the file straight after the 3 header bytes
    while ((n = fread(packet + DATA_HEADER, 1, MAX_PAYLOAD_SIZE - DATA_HEADER, file)) > 0)
    {
        packet[0] = C_DATA;
        packet[1] = (n >> 8) & 0xFF; // L2
        packet[2] = n & 0xFF;        // L1
        if (llSend(packet, n + DATA_HEADER) < 0)
        {
            printf("Could not send a DATA packet (%ld of %ld bytes sent)\n", sent, fileSize);
            fclose(file);
            return -1;
        }
        sent += n;
    }
    if (ferror(file))
    {
        printf("Error reading the file\n");
        fclose(file);
        return -1;
    }
    fclose(file);

    len = buildControlPacket(packet, C_END, fileSize, baseName);
    if (llSend(packet, len) < 0)
    {
        printf("Could not send the END packet\n");
        return -1;
    }

    printf("File \"%s\" sent. ", baseName);
    printThroughput(fileSize, secondsSince(&start), baudRate);
    return 0;
}

// Reads packets until the END packet, writing the data to "filename". The link must already be open.
static int receiveFile(const char *filename, int baudRate)
{
    FILE *file = NULL;
    long expectedSize = 0, received = 0;
    char nameInPacket[256];
    struct timespec start;
    unsigned char packet[MAX_PAYLOAD_SIZE];

    while (TRUE)
    {
        int len = llReceive(packet);
        if (len < 0)
        {
            printf("llReceive failed\n");
            if (file) fclose(file);
            return -1;
        }
        if (len == 0) continue;

        if (packet[0] == C_START)
        {
            if (file != NULL || parseControlPacket(packet, len, &expectedSize, nameInPacket) < 0)
            {
                printf("Invalid START packet\n");
                if (file) fclose(file);
                return -1;
            }
            file = fopen(filename, "wb");
            if (file == NULL)
            {
                perror("fopen");
                return -1;
            }
            clock_gettime(CLOCK_MONOTONIC, &start);
            printf("Receiving \"%s\" (%ld bytes) into \"%s\"\n", nameInPacket, expectedSize, filename);
        }
        else if (packet[0] == C_DATA)
        {
            // the DATA packet has its own length field: use it to double-check the packet
            int k = 256 * packet[1] + packet[2];
            if (file == NULL || len < DATA_HEADER || k != len - DATA_HEADER)
            {
                printf("Invalid DATA packet\n");
                if (file) fclose(file);
                return -1;
            }
            if (fwrite(packet + DATA_HEADER, 1, k, file) != (size_t)k)
            {
                perror("fwrite");
                fclose(file);
                return -1;
            }
            received += k;
        }
        else if (packet[0] == C_END)
        {
            long endSize = 0;
            char endName[256];
            if (file == NULL || parseControlPacket(packet, len, &endSize, endName) < 0)
            {
                printf("Invalid END packet\n");
                if (file) fclose(file);
                return -1;
            }
            fclose(file);

            // END repeats the START information, and the file must have the announced size
            if (endSize != expectedSize || received != expectedSize)
            {
                printf("Size mismatch: START said %ld, END said %ld, received %ld bytes\n",
                       expectedSize, endSize, received);
                return -1;
            }
            printf("File \"%s\" received correctly. ", filename);
            printThroughput(received, secondsSince(&start), baudRate);
            return 0;
        }
        else
        {
            printf("Unknown packet type %d\n", packet[0]);
            if (file) fclose(file);
            return -1;
        }
    }
}

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayer llParameters = {
        .baudRate = baudRate,
        .nRetransmissions = nTries,
        .timeout = timeout,
    };
    strcpy(llParameters.serialPort, serialPort);

    if (strcmp(role, "tx") == 0)
    {
        if (llOpenTx(llParameters) < 0)
        {
            printf("llOpenTx failed\n");
            return;
        }

        int result = sendFile(filename, baudRate);

        // the link is closed even if the transfer failed
        if (llCloseTx() < 0) printf("llCloseTx failed\n");
        printf(result == 0 ? "Transfer finished successfully\n" : "Transfer FAILED\n");
    }
    else if (strcmp(role, "rx") == 0)
    {
        if (llOpenRx(llParameters) < 0)
        {
            printf("llOpenRx failed\n");
            return;
        }

        int result = receiveFile(filename, baudRate);

        if (llCloseRx() < 0) printf("llCloseRx failed\n");
        printf(result == 0 ? "Transfer finished successfully\n" : "Transfer FAILED\n");
    }
    else
    {
        printf("Invalid role: %s. Must be 'tx' or 'rx'.\n", role);
        return;
    }
}
