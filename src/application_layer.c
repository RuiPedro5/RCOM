

#include "application_layer.h"
#include "link_layer.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#define C_START 1
#define C_DATA 2
#define C_END 3

#define TLV_FILE_SIZE 0
#define TLV_FILE_NAME 1

#define DATA_HEADER 3
static int ConstruirPacket(unsigned char *packet, unsigned char c, long fileSize, const char *fileName)
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


static int ControlPacketPArse(const unsigned char *packet, int len, long *fileSize, char *fileName)
{
    int i = 1; 
    int gotSize = FALSE;
    fileName[0] = '\0';

    while (i + 2 <= len)
    {
        unsigned char t = packet[i];
        unsigned char l = packet[i + 1];
        i += 2;
        if (i + l > len) return -1;

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
   
        i += l;
    }
    return gotSize ? 0 : -1;
}


static int mandarFile(const char *filename)
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

   
    const char *baseName = strrchr(filename, '/');
    baseName = (baseName == NULL) ? filename : baseName + 1;
    unsigned char packet[MAX_PAYLOAD_SIZE];

    int len = ConstruirPacket(packet, C_START, fileSize, baseName);
    if (llSend(packet, len) < 0)
    {
        printf("Could not send the START packet\n");
        fclose(file);
        return -1;
    }

    long sent = 0;
    size_t n;
    
    while ((n = fread(packet + DATA_HEADER, 1, MAX_PAYLOAD_SIZE - DATA_HEADER, file)) > 0)
    {
        packet[0] = C_DATA;
        packet[1] = (n >> 8) & 0xFF; 
        packet[2] = n & 0xFF;       
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

    len = ConstruirPacket(packet, C_END, fileSize, baseName);
    if (llSend(packet, len) < 0)
    {
        printf("Could not send the END packet\n");
        return -1;
    }

    printf("File \"%s\" sent. \n", baseName);
    return 0;
}


static int receberFile(const char *filename)
{
    FILE *file = NULL;
    long expectedSize = 0, received = 0;
    char nameInPacket[256];
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
            if (file != NULL || ControlPacketPArse(packet, len, &expectedSize, nameInPacket) < 0)
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
            printf("Receiving \"%s\" (%ld bytes) into \"%s\"\n", nameInPacket, expectedSize, filename);
        }
        else if (packet[0] == C_DATA)
        {
        
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
            if (file == NULL || ControlPacketPArse(packet, len, &endSize, endName) < 0)
            {
                printf("Invalid END packet\n");
                if (file) fclose(file);
                return -1;
            }
            fclose(file);

           
            if (endSize != expectedSize || received != expectedSize)
            {
                printf("Size mismatch: START said %ld, END said %ld, received %ld bytes\n",
                       expectedSize, endSize, received);
                return -1;
            }
            printf("File \"%s\" received correctly. \n", filename);
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

        int result = mandarFile(filename);

        
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

        int result = receberFile(filename);

        if (llCloseRx() < 0) printf("llCloseRx failed\n");
        printf(result == 0 ? "Transfer finished successfully\n" : "Transfer FAILED\n");
    }
    else
    {
        printf("Invalid role: %s. Must be 'tx' or 'rx'.\n", role);
        return;
    }
}