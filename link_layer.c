// RCOM 2026/2027
//
// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source
#define BUF_SIZE 5
int alarmEnabled = FALSE;
int alarmCount= 0;

//N(s)
static int module_counter=0;

//maquinadeestados
// Receiver state machine for supervision frames: F A C BCC1 F
// It reads the serial port byte by byte and only finishes when a frame
// with the expected A and C fields arrives (BCC1 = A^C is checked here too).
// useAlarm == TRUE : stops waiting when the alarm fires (Tx waiting for UA)
// useAlarm == FALSE: waits forever (Rx waiting for SET)
// Returns 0 if the frame was received, 1 if the alarm fired first.
typedef enum { ST_START, ST_FLAG_RCV, ST_A_RCV, ST_C_RCV, ST_BCC_OK, ST_STOP } State;

int maquinadeestados(unsigned char expectedA, unsigned char expectedC, int useAlarm){

    State state = ST_START;
    unsigned char a = 0, c = 0;

    while (state != ST_STOP){
        //paro de tentar ler apenas quando o alarm acabar e for reposto.
        if (useAlarm && alarmEnabled == FALSE) return 1;

        unsigned char byte;
        int nbytes = readByteSerialPort(&byte);

        printf ("saiu da funçao readByteSerialPort tendo ido %d bytes\n", nbytes);
        //-1 (read interrupted by the alarm) or 0 (nothing read): try again,
        //the check above decides if we must stop
        if (nbytes<=0){continue;}

        printf("Byte received: 0x%02X\n", byte) ;

        switch (state){
        case ST_START:
            if (byte == 0x7E) state = ST_FLAG_RCV;
            break;
        case ST_FLAG_RCV:
            if (byte == expectedA){ a = byte; state = ST_A_RCV; }
            else if (byte != 0x7E) state = ST_START; //another FLAG: stay here
            break;
        case ST_A_RCV:
            if (byte == expectedC){ c = byte; state = ST_C_RCV; }
            else if (byte == 0x7E) state = ST_FLAG_RCV;
            else state = ST_START;
            break;
        case ST_C_RCV:
            if (byte == (a ^ c)) state = ST_BCC_OK;
            else if (byte == 0x7E) state = ST_FLAG_RCV;
            else state = ST_START;
            break;
        case ST_BCC_OK:
            if (byte == 0x7E) state = ST_STOP;
            else state = ST_START;
            break;
        default:
            state = ST_START;
            break;
        }
    }
    printf("Frame received. Stop reading from serial port.\n");
    return 0;
}

//alarmhandler
void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;

    printf("Alarm #%d received\n", alarmCount);
}


////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llOpenTx(LinkLayer llParameters)
{
// ----------------------------------------------------
// This example code shows how to open the serial port and send a string.
// TODO: Adapt and extend this code according to the specifications of the project.
// ----------------------------------------------------

if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0)
{
perror("openSerialPort");
return -1;
}

printf("Serial port %s opened\n", llParameters.serialPort);

// Create string to send
unsigned char buf[BUF_SIZE] = {0};


buf[0] = 0X7E;
buf[1] = 0X03;
buf[2] = 0X03;
buf[3]= buf[1]^buf[2];
buf[4]= 0X7E;

// In non-canonical mode, '\n' does not end the writing.
// Test this condition by placing a '\n' in the middle of the buffer.
// The whole buffer must be sent even with the '\n'.


struct sigaction act = {0};
act.sa_handler = &alarmHandler;
if (sigaction(SIGALRM, &act, NULL) == -1){
    perror("sigaction");
    exit(1);
}

printf("Alarm configured\n");

alarmCount = 0;
alarmEnabled = FALSE;

while (alarmCount <= llParameters.nRetransmissions)
{ 
  int bytes=0;
  if (alarmEnabled == FALSE)
        {
            bytes = writeBytesSerialPort(buf, BUF_SIZE);
            printf("%d bytes written to serial port\n", bytes);
            //escreve uma vez a cada alarme

            alarm(llParameters.timeout); // Set alarm to be triggered in timeout seconds
            alarmEnabled = TRUE;
            
        }  
    
    if (maquinadeestados(0x03, 0x07, TRUE)==0){
        alarm(0);
        alarmEnabled = FALSE; // otherwise the next use of the alarm would never write
        return 0; 
    }
    

}


// Wait until all bytes have been written to the serial port
sleep(1);

// Close serial port
if (closeSerialPort() < 0)
{
perror("closeSerialPort");
return -1;
}

printf("Serial port %s closed\n", llParameters.serialPort);

return -1;
}


int llOpenRx(LinkLayer llParameters)
{
    // ----------------------------------------------------
    // This example code shows how to open the serial port and receive a string.
    // TODO: Adapt and extend this code according to the specifications of the project.
    // ----------------------------------------------------

    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0)
    {
        perror("openSerialPort");
        return -1;
    }

    printf("Serial port %s opened\n", llParameters.serialPort);

    // Wait (no alarm, no limit) for a valid SET frame: A=0x03, C=0x03
    maquinadeestados(0x03, 0x03, FALSE);

    printf("Received SET. Will send UA.\n");

    // UA is a reply sent by the Receiver, so A = 0x03 (it was 0x01)
    unsigned char buf[BUF_SIZE] = {0};

    buf[0] = 0X7E;
    buf[1] = 0X03;
    buf[2] = 0X07;
    buf[3]= buf[1]^buf[2];
    buf[4]= 0X7E;

    int bytes = writeBytesSerialPort(buf, BUF_SIZE);
    printf("%d bytes written to serial port\n", bytes);

    // Close serial port
    if (closeSerialPort() < 0)
    {
        perror("closeSerialPort");
        return -1;
    }

    printf("Serial port %s closed\n", llParameters.serialPort);

    return 0;
}

////////////////////////////////////////////////
// LLSEND
////////////////////////////////////////////////

int llSend(const unsigned char *buf, int bufSize)
{
    // TODO: Implement this function
    //create frame with size 0, we dinamically increase it
    int frame_size=0;
    unsigned char frame[2*MAX_PAYLOAD_SIZE + 7];
    //calculate bcc2 first
    unsigned char bcc2=0x00;
    for (int i=0; i<bufSize; i++){
        bcc2^=buf[i];
    }
    //add flags to frame
    frame[0]=0x7E;
    frame[1]=0x03;
    if (module_counter==0){frame[2]=0x00;}else frame[2]=0x80;
    frame[3]=frame[1]^frame[2];
    frame_size= 4;

    //stuff data
    for (int i=0; i<bufSize; i++){
        if (buf[i]==0x7E){
            frame[frame_size]=0x7D;
            frame_size++;
            frame[frame_size]=0x5E;
            frame_size++;
        }
        else if (buf[i]==0x7D){
            frame[frame_size]=0x7D;
            frame_size++;
            frame[frame_size]=0x5D;
            frame_size++;
        }
        else{
            frame[frame_size]=buf[i];
            frame_size++;
        }
    }

    //add bcc2
    if(bcc2==0x7D){
        frame[frame_size]=0x7d;
        frame_size++;
        frame[frame_size]=0x5D;
        frame_size++;
    }
    else if (bcc2==0x7E){
        frame[frame_size]=0x7d;
        frame_size++;
        frame[frame_size]=0x5E;
        frame_size++;

    }
    else{
        frame[frame_size]= bcc2;
        frame_size++;
    }
    //flag final
    frame[frame_size]= 0x7E;
       

    return 0;
}


////////////////////////////////////////////////
// LLRECEIVE
////////////////////////////////////////////////
int llReceive(unsigned char *packet)
{
    // TODO: Implement this function

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llCloseTx()
{
    // TODO: Implement this function

    return 0;
}

int llCloseRx()
{
    // TODO: Implement this function

    return 0;
}