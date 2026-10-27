// RCOM 2026/2027
//
// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source
#define BUF_SIZE 5
int alarmEnabled = FALSE;
int alarmCount= 0;

//N(s)
// On the Tx: number (0 or 1) of the I frame we are going to send next.
// On the Rx: number (0 or 1) of the I frame we expect to receive next.
// (Tx and Rx are different programs, so each one has its own copy.)
static int module_counter=0;

//link layer parameters (timeout, nRetransmissions...) saved by llOpenTx/llOpenRx so llSend can use them
static LinkLayer ll;

//C field of the last supervision frame accepted by maquinadeestados
static unsigned char lastC = 0;

//protocol statistics (printed when the link is closed)
static int statISent = 0;    //I frames sent (first transmission)
static int statRetrans = 0;  //I frames retransmitted
static int statTimeouts = 0; //time-outs (SET, I and DISC frames)
static int statREJRcv = 0;   //REJ received (Tx)
static int statIRcv = 0;     //I frames accepted (Rx)
static int statDup = 0;      //duplicate I frames received (Rx)
static int statREJSent = 0;  //REJ sent (Rx)

static void printStatistics(int isTx){
    printf("---- Link layer statistics ----\n");
    if (isTx)
        printf("I frames sent: %d | retransmitted: %d | time-outs: %d | REJ received: %d\n",
               statISent, statRetrans, statTimeouts, statREJRcv);
    else
        printf("I frames accepted: %d | duplicates: %d | REJ sent: %d | time-outs: %d\n",
               statIRcv, statDup, statREJSent, statTimeouts);
}

//expectedC value meaning "accept any of RR0, RR1, REJ0, REJ1"
#define C_ANY_REPLY 0xFF
//C field of RR / REJ for a given N(r)
#define C_RR(n)  (0xAA | (n))
#define C_REJ(n) (0x54 | (n))

static int isExpectedC(unsigned char byte, unsigned char expectedC){
    if (expectedC == C_ANY_REPLY)
        return byte == C_RR(0) || byte == C_RR(1) || byte == C_REJ(0) || byte == C_REJ(1);
    return byte == expectedC;
}

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

#ifdef DEBUG_BYTES
        printf ("saiu da funçao readByteSerialPort tendo ido %d bytes\n", nbytes);
#endif
        //-1 (read interrupted by the alarm) or 0 (nothing read): try again,
        //the check above decides if we must stop
        if (nbytes<=0){continue;}

#ifdef DEBUG_BYTES
        printf("Byte received: 0x%02X\n", byte) ;
#endif

        switch (state){
        case ST_START:
            if (byte == 0x7E) state = ST_FLAG_RCV;
            break;
        case ST_FLAG_RCV:
            if (byte == expectedA){ a = byte; state = ST_A_RCV; }
            else if (byte != 0x7E) state = ST_START; //another FLAG: stay here
            break;
        case ST_A_RCV:
            if (isExpectedC(byte, expectedC)){ c = byte; lastC = byte; state = ST_C_RCV; }
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
ll = llParameters;
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
    ll = llParameters;
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

    // The Rx also uses the alarm: in llCloseRx the DISC it sends is protected by a timer
    struct sigaction act = {0};
    act.sa_handler = &alarmHandler;
    if (sigaction(SIGALRM, &act, NULL) == -1){
        perror("sigaction");
        exit(1);
    }

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

    // The serial port stays open: llReceive() still needs it. It is closed in llCloseRx().
    return 0;
}

////////////////////////////////////////////////
// LLSEND
////////////////////////////////////////////////

int llSend(const unsigned char *buf, int bufSize)
{
    // the frame buffer below only has room for MAX_PAYLOAD_SIZE bytes of data (even after stuffing)
    if (bufSize < 0 || bufSize > MAX_PAYLOAD_SIZE) return -1;

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
    frame_size++;

    // ---- Stop & Wait: send the frame, then wait for the Rx answer (RR or REJ) ----
    int next = 1 - module_counter; // the N(r) the Rx answers with when it accepts our frame

    // 1 first transmission + up to nRetransmissions retransmissions
    for (int attempt = 0; attempt <= ll.nRetransmissions; attempt++)
    {
        if (attempt > 0)
        {
            statRetrans++;
            printf("Retransmitting I frame Ns=%d (%d/%d)\n", module_counter, attempt, ll.nRetransmissions);
        }
        else statISent++;

        if (writeBytesSerialPort(frame, frame_size) != frame_size)
        {
            perror("writeBytesSerialPort");
            return -1;
        }
        printf("I frame Ns=%d sent (%d bytes on the line)\n", module_counter, frame_size);

        alarmEnabled = TRUE;
        alarm(ll.timeout);

        int rejected = FALSE;
        while (alarmEnabled == TRUE && rejected == FALSE)
        {
            // returns 1 if the alarm fired (time-out), 0 if an RR/REJ arrived
            if (maquinadeestados(0x03, C_ANY_REPLY, TRUE) == 1)
            {
                statTimeouts++;
                printf("Time-out waiting for RR/REJ\n");
                break;
            }

            if (lastC == C_RR(next))
            {
                // positive ACK: the Rx accepted the frame and now expects "next"
                alarm(0);
                alarmEnabled = FALSE;
                module_counter = next;
                printf("RR%d received. Frame accepted\n", next);
                return bufSize;
            }
            if (lastC == C_REJ(module_counter))
            {
                // negative ACK: the Rx saw errors in the data, send it again
                printf("REJ%d received\n", module_counter);
                statREJRcv++;
                rejected = TRUE;
            }
            // anything else (e.g. a late, old RR): ignore it and keep waiting
        }

        alarm(0);
        alarmEnabled = FALSE;
        // time-out or REJ: next iteration of the for loop retransmits
    }

    printf("No valid answer after %d retransmissions. Giving up\n", ll.nRetransmissions);
    return -1;
}

//sends a supervision frame (RR/REJ/...): F A C BCC1 F
static int sendSupervisionFrame(unsigned char a, unsigned char c)
{
    unsigned char f[5] = {0x7E, a, c, a ^ c, 0x7E};
    return writeBytesSerialPort(f, 5);
}

////////////////////////////////////////////////
// LLRECEIVE
////////////////////////////////////////////////

//maquinadeestadosI
// Same idea as maquinadeestados, but for frames sent BY THE TRANSMITTER (A=0x03):
//   I frames:   F A C BCC1 D1...DN BCC2 F   (C = 0x00 / 0x80)
//   SET / DISC: F A C BCC1 F                (C = 0x03 / 0x0B, no data)
// It waits (forever) for one complete frame with a correct header (BCC1).
// Out: data[] = the data field already de-stuffed, WITH the BCC2 as its last byte
//      *size  = how many bytes are in data[] (BCC2 included; 0 for SET / DISC)
//      *ns    = N(s) of the frame (0 or 1), only meaningful for I frames
//      *type  = T_I, T_SET or T_DISC
// Frames with a wrong header are ignored, as the spec says.
#define T_I 0
#define T_SET 1
#define T_DISC 2
typedef enum { I_START, I_FLAG_RCV, I_A_RCV, I_C_RCV, I_DATA, I_STOP } IState;

static void maquinadeestadosI(unsigned char *data, int *size, int *ns, int *type)
{
    IState state = I_START;
    unsigned char c = 0;
    int escaped = FALSE; // TRUE right after a 0x7D (next byte must be de-stuffed)
    *size = 0;
    *ns = 0;
    *type = T_I;

    while (state != I_STOP)
    {
        unsigned char byte;
        int nbytes = readByteSerialPort(&byte);
        if (nbytes <= 0) continue;

        switch (state)
        {
        case I_START:
            if (byte == 0x7E) state = I_FLAG_RCV;
            break;
        case I_FLAG_RCV:
            if (byte == 0x03) state = I_A_RCV;
            else if (byte != 0x7E) state = I_START; //another FLAG: stay here
            break;
        case I_A_RCV:
            if (byte == 0x00 || byte == 0x80 || byte == 0x03 || byte == 0x0B){ c = byte; state = I_C_RCV; } //I, SET, DISC
            else if (byte == 0x7E) state = I_FLAG_RCV;
            else state = I_START;
            break;
        case I_C_RCV:
            if (byte == (0x03 ^ c))
            {
                //header is correct: from now on, everything until the next flag is data
                *ns = (c == 0x80);
                *type = (c == 0x03) ? T_SET : (c == 0x0B) ? T_DISC : T_I;
                *size = 0;
                escaped = FALSE;
                state = I_DATA;
            }
            else if (byte == 0x7E) state = I_FLAG_RCV;
            else state = I_START;
            break;
        case I_DATA:
        {
            int store = FALSE;
            if (escaped)
            {
                escaped = FALSE;
                if (byte == 0x7E){ state = I_FLAG_RCV; break; } //flag after 0x7D: broken frame
                byte ^= 0x20; //de-stuffing: 0x5E -> 0x7E, 0x5D -> 0x7D
                store = TRUE;
            }
            else if (byte == 0x7D) escaped = TRUE;       //next byte is the stuffed one
            else if (byte == 0x7E) state = I_STOP;       //closing flag: frame complete
            else store = TRUE;

            if (store)
            {
                if (*size >= MAX_PAYLOAD_SIZE + 1) state = I_START; //too big: not a valid frame
                else data[(*size)++] = byte;
            }
            break;
        }
        default:
            state = I_START;
            break;
        }
    }
}

////////////////////////////////////////////////
// LLRECEIVE
////////////////////////////////////////////////

static int discReceived = FALSE; //a DISC arrived while llReceive was waiting for data

int llReceive(unsigned char *packet)
{
    unsigned char data[MAX_PAYLOAD_SIZE + 1]; // data + BCC2
    int size, ns, type;

    while (TRUE)
    {
        maquinadeestadosI(data, &size, &ns, &type);

        if (type == T_SET && size == 0)
        {
            //our UA got lost: the Tx did not notice the connection is open and sent SET again
            printf("SET received again. Sending UA again\n");
            if (sendSupervisionFrame(0x03, 0x07) < 0) return -1;
            continue;
        }
        if (type == T_DISC && size == 0)
        {
            //the Tx is closing the link although we did not get the END packet: tell the app
            printf("DISC received while waiting for data\n");
            discReceived = TRUE;
            return -1;
        }
        if (type != T_I || size < 1) continue; // not a real I frame (no room even for the BCC2)

        //BCC2 is the last byte; the data is everything before it
        int dataLen = size - 1;
        unsigned char bcc2 = 0x00;
        for (int i = 0; i < dataLen; i++) bcc2 ^= data[i];
        int bcc2Ok = (bcc2 == data[dataLen]);

        if (ns != module_counter)
        {
            //duplicate (we already have it, our RR got lost): throw the data away, but confirm again
            printf("Duplicate I frame Ns=%d. Sending RR%d again\n", ns, module_counter);
            statDup++;
            if (sendSupervisionFrame(0x03, C_RR(module_counter)) < 0) return -1;
            continue;
        }

        if (!bcc2Ok)
        {
            //right frame but corrupted data: ask for it again
            printf("I frame Ns=%d with errors in the data. Sending REJ%d\n", ns, ns);
            statREJSent++;
            if (sendSupervisionFrame(0x03, C_REJ(ns)) < 0) return -1;
            continue;
        }

        //new frame, data OK: hand it to the application and confirm
        memcpy(packet, data, dataLen);
        module_counter = 1 - ns; //now we expect the other number
        statIRcv++;
        printf("I frame Ns=%d accepted (%d bytes). Sending RR%d\n", ns, dataLen, module_counter);
        if (sendSupervisionFrame(0x03, C_RR(module_counter)) < 0) return -1;
        return dataLen;
    }
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
//  Tx                      Rx
//  DISC (A=0x03) ------->            command sent by the Tx
//        <------- DISC (A=0x01)      command sent by the Rx
//  UA   (A=0x01) ------->            reply sent by the Tx
int llCloseTx()
{
    int closed = FALSE;
    alarmCount = 0;
    alarmEnabled = FALSE;

    // 1 first transmission + up to nRetransmissions retransmissions (DISC is protected by the timer)
    for (int attempt = 0; attempt <= ll.nRetransmissions && !closed; attempt++)
    {
        if (attempt > 0) printf("Retransmitting DISC (%d/%d)\n", attempt, ll.nRetransmissions);

        if (sendSupervisionFrame(0x03, 0x0B) < 0)
        {
            perror("writeBytesSerialPort");
            break;
        }
        printf("DISC sent\n");

        alarmEnabled = TRUE;
        alarm(ll.timeout);
        int r = maquinadeestados(0x01, 0x0B, TRUE); // DISC from the Rx: A=0x01
        alarm(0);
        alarmEnabled = FALSE;

        if (r == 0) closed = TRUE;
        else { statTimeouts++; printf("Time-out waiting for DISC\n"); }
    }

    if (closed)
    {
        // final UA: a reply sent by the Tx, so A = 0x01
        if (sendSupervisionFrame(0x01, 0x07) < 0) perror("writeBytesSerialPort");
        else printf("DISC received. UA sent. Connection closed\n");
    }
    else printf("No DISC received from the Rx. Closing anyway\n");

    printStatistics(TRUE);

    // Wait until all bytes have been written to the serial port
    sleep(1);
    if (closeSerialPort() < 0)
    {
        perror("closeSerialPort");
        return -1;
    }
    printf("Serial port %s closed\n", ll.serialPort);

    return closed ? 0 : -1;
}

int llCloseRx()
{
    unsigned char data[MAX_PAYLOAD_SIZE + 1];
    int size, ns, type;

    // 1) wait for the Tx DISC (unless llReceive already got it)
    while (discReceived == FALSE)
    {
        maquinadeestadosI(data, &size, &ns, &type);

        if (type == T_DISC && size == 0) break;

        if (type == T_I && size >= 1 && ns != module_counter)
        {
            //the Tx did not get our last RR and sent the last I frame again: confirm it again
            printf("Duplicate I frame Ns=%d while closing. Sending RR%d again\n", ns, module_counter);
            statDup++;
            sendSupervisionFrame(0x03, C_RR(module_counter));
        }
    }
    printf("DISC received\n");

    // 2) send our DISC (command sent by the Rx: A=0x01) and wait for the Tx UA (A=0x01)
    int closed = FALSE;
    alarmCount = 0;
    alarmEnabled = FALSE;

    for (int attempt = 0; attempt <= ll.nRetransmissions && !closed; attempt++)
    {
        if (attempt > 0) printf("Retransmitting DISC (%d/%d)\n", attempt, ll.nRetransmissions);

        if (sendSupervisionFrame(0x01, 0x0B) < 0)
        {
            perror("writeBytesSerialPort");
            break;
        }
        printf("DISC sent\n");

        alarmEnabled = TRUE;
        alarm(ll.timeout);
        int r = maquinadeestados(0x01, 0x07, TRUE); // UA from the Tx: A=0x01
        alarm(0);
        alarmEnabled = FALSE;

        if (r == 0) closed = TRUE;
        else { statTimeouts++; printf("Time-out waiting for UA\n"); }
    }

    if (closed) printf("UA received. Connection closed\n");
    else printf("No UA received from the Tx. Closing anyway\n");

    printStatistics(FALSE);

    sleep(1);
    if (closeSerialPort() < 0)
    {
        perror("closeSerialPort");
        return -1;
    }
    printf("Serial port %s closed\n", ll.serialPort);

    return closed ? 0 : -1;
}
