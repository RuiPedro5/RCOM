// RCOM 2026/2027
//
// Implementação do protocolo de ligação de dados

#include "link_layer.h"
#include "serial_port.h"

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>

#define _POSIX_SOURCE 1
#define BUF_SIZE 5
int alarmEnabled = FALSE;
int alarmCount= 0;

// N(s): no Tx é o número da próxima trama I a enviar; no Rx é o número da próxima trama I esperada
static int module_counter=0;

// parâmetros guardados por llOpenTx/llOpenRx para o llSend os usar
static LinkLayer ll;

// campo C da última trama de supervisão aceite pela maquinadeestados
static unsigned char lastC = 0;

// estatísticas (mostradas ao fechar a ligação)
static int statISent = 0;
static int statRetrans = 0;
static int statTimeouts = 0;
static int statREJRcv = 0;
static int statIRcv = 0;
static int statDup = 0;
static int statREJSent = 0;

static void printStatistics(int isTx){
    printf("---- Link layer statistics ----\n");
    if (isTx)
        printf("I frames sent: %d | retransmitted: %d | time-outs: %d | REJ received: %d\n",
               statISent, statRetrans, statTimeouts, statREJRcv);
    else
        printf("I frames accepted: %d | duplicates: %d | REJ sent: %d | time-outs: %d\n",
               statIRcv, statDup, statREJSent, statTimeouts);
}

// valor de expectedC que aceita RR0, RR1, REJ0 ou REJ1
#define C_ANY_REPLY 0xFF
#define C_RR(n)  (0xAA | (n))
#define C_REJ(n) (0x54 | (n))

static int isExpectedC(unsigned char byte, unsigned char expectedC){
    if (expectedC == C_ANY_REPLY)
        return byte == C_RR(0) || byte == C_RR(1) || byte == C_REJ(0) || byte == C_REJ(1);
    return byte == expectedC;
}

// Máquina de estados para tramas de supervisão: F A C BCC1 F
// useAlarm == TRUE : pára quando o alarme dispara
// useAlarm == FALSE: espera indefinidamente
// Devolve 0 se a trama foi recebida, 1 se o alarme disparou primeiro.
typedef enum { ST_START, ST_FLAG_RCV, ST_A_RCV, ST_C_RCV, ST_BCC_OK, ST_STOP } State;

int maquinadeestados(unsigned char expectedA, unsigned char expectedC, int useAlarm){

    State state = ST_START;
    unsigned char a = 0, c = 0;

    while (state != ST_STOP){
        if (useAlarm && alarmEnabled == FALSE) return 1;

        unsigned char byte;
        int nbytes = readByteSerialPort(&byte);

#ifdef DEBUG_BYTES
        printf ("saiu da funçao readByteSerialPort tendo ido %d bytes\n", nbytes);
#endif
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
            else if (byte != 0x7E) state = ST_START;
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

if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0)
{
perror("openSerialPort");
return -1;
}

printf("Serial port %s opened\n", llParameters.serialPort);

unsigned char buf[BUF_SIZE] = {0};


buf[0] = 0X7E;
buf[1] = 0X03;
buf[2] = 0X03;
buf[3]= buf[1]^buf[2];
buf[4]= 0X7E;


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

            alarm(llParameters.timeout);
            alarmEnabled = TRUE;
            
        }  
    
    if (maquinadeestados(0x03, 0x07, TRUE)==0){
        alarm(0);
        alarmEnabled = FALSE;
        return 0; 
    }
    

}


sleep(1);

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

    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0)
    {
        perror("openSerialPort");
        return -1;
    }

    printf("Serial port %s opened\n", llParameters.serialPort);

    // o Rx também usa o alarme (o DISC enviado em llCloseRx é protegido por temporizador)
    struct sigaction act = {0};
    act.sa_handler = &alarmHandler;
    if (sigaction(SIGALRM, &act, NULL) == -1){
        perror("sigaction");
        exit(1);
    }

    // espera, sem limite, por uma trama SET válida: A=0x03, C=0x03
    maquinadeestados(0x03, 0x03, FALSE);

    printf("Received SET. Will send UA.\n");

    unsigned char buf[BUF_SIZE] = {0};

    buf[0] = 0X7E;
    buf[1] = 0X03;
    buf[2] = 0X07;
    buf[3]= buf[1]^buf[2];
    buf[4]= 0X7E;

    int bytes = writeBytesSerialPort(buf, BUF_SIZE);
    printf("%d bytes written to serial port\n", bytes);

    // a porta fica aberta: é fechada em llCloseRx()
    return 0;
}

////////////////////////////////////////////////
// LLSEND
////////////////////////////////////////////////

int llSend(const unsigned char *buf, int bufSize)
{
    if (bufSize < 0 || bufSize > MAX_PAYLOAD_SIZE) return -1;

    int frame_size=0;
    unsigned char frame[2*MAX_PAYLOAD_SIZE + 7];
    unsigned char bcc2=0x00;
    for (int i=0; i<bufSize; i++){
        bcc2^=buf[i];
    }
    frame[0]=0x7E;
    frame[1]=0x03;
    if (module_counter==0){frame[2]=0x00;}else frame[2]=0x80;
    frame[3]=frame[1]^frame[2];
    frame_size= 4;

    // byte stuffing dos dados
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

    // BCC2 (também com stuffing)
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
    frame[frame_size]= 0x7E;
    frame_size++;

    // Stop & Wait: envia a trama e espera a resposta do Rx (RR ou REJ)
    int next = 1 - module_counter;

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
            // devolve 1 se houve time-out, 0 se chegou RR/REJ
            if (maquinadeestados(0x03, C_ANY_REPLY, TRUE) == 1)
            {
                statTimeouts++;
                printf("Time-out waiting for RR/REJ\n");
                break;
            }

            if (lastC == C_RR(next))
            {
                alarm(0);
                alarmEnabled = FALSE;
                module_counter = next;
                printf("RR%d received. Frame accepted\n", next);
                return bufSize;
            }
            if (lastC == C_REJ(module_counter))
            {
                printf("REJ%d received\n", module_counter);
                statREJRcv++;
                rejected = TRUE;
            }
            // qualquer outra resposta (ex.: RR antigo atrasado) é ignorada
        }

        alarm(0);
        alarmEnabled = FALSE;
    }

    printf("No valid answer after %d retransmissions. Giving up\n", ll.nRetransmissions);
    return -1;
}

static int sendSupervisionFrame(unsigned char a, unsigned char c)
{
    unsigned char f[5] = {0x7E, a, c, a ^ c, 0x7E};
    return writeBytesSerialPort(f, 5);
}

////////////////////////////////////////////////
// LLRECEIVE
////////////////////////////////////////////////

// Máquina de estados para tramas enviadas pelo transmissor (A=0x03):
//   trama I:    F A C BCC1 D1...DN BCC2 F   (C = 0x00 / 0x80)
//   SET / DISC: F A C BCC1 F                (C = 0x03 / 0x0B)
// Saídas:
//   data[] = campo de dados já sem stuffing, com o BCC2 como último byte
//   *size  = número de bytes em data[] (0 para SET / DISC)
//   *ns    = N(s) da trama (só relevante para tramas I)
//   *type  = T_I, T_SET ou T_DISC
// Tramas com cabeçalho errado são ignoradas.
#define T_I 0
#define T_SET 1
#define T_DISC 2
typedef enum { I_START, I_FLAG_RCV, I_A_RCV, I_C_RCV, I_DATA, I_STOP } IState;

static void maquinadeestadosI(unsigned char *data, int *size, int *ns, int *type)
{
    IState state = I_START;
    unsigned char c = 0;
    int escaped = FALSE;
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
            else if (byte != 0x7E) state = I_START;
            break;
        case I_A_RCV:
            if (byte == 0x00 || byte == 0x80 || byte == 0x03 || byte == 0x0B){ c = byte; state = I_C_RCV; }
            else if (byte == 0x7E) state = I_FLAG_RCV;
            else state = I_START;
            break;
        case I_C_RCV:
            if (byte == (0x03 ^ c))
            {
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
                if (byte == 0x7E){ state = I_FLAG_RCV; break; }
                byte ^= 0x20; // 0x5E -> 0x7E, 0x5D -> 0x7D
                store = TRUE;
            }
            else if (byte == 0x7D) escaped = TRUE;
            else if (byte == 0x7E) state = I_STOP;
            else store = TRUE;

            if (store)
            {
                if (*size >= MAX_PAYLOAD_SIZE + 1) state = I_START;
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

// TRUE se chegou um DISC enquanto o llReceive esperava dados
static int discReceived = FALSE;

int llReceive(unsigned char *packet)
{
    unsigned char data[MAX_PAYLOAD_SIZE + 1];
    int size, ns, type;

    while (TRUE)
    {
        maquinadeestadosI(data, &size, &ns, &type);

        if (type == T_SET && size == 0)
        {
            // o UA perdeu-se: o Tx reenviou o SET
            printf("SET received again. Sending UA again\n");
            if (sendSupervisionFrame(0x03, 0x07) < 0) return -1;
            continue;
        }
        if (type == T_DISC && size == 0)
        {
            printf("DISC received while waiting for data\n");
            discReceived = TRUE;
            return -1;
        }
        if (type != T_I || size < 1) continue;

        int dataLen = size - 1;
        unsigned char bcc2 = 0x00;
        for (int i = 0; i < dataLen; i++) bcc2 ^= data[i];
        int bcc2Ok = (bcc2 == data[dataLen]);

        if (ns != module_counter)
        {
            // duplicado (o RR perdeu-se): descarta os dados mas volta a confirmar
            printf("Duplicate I frame Ns=%d. Sending RR%d again\n", ns, module_counter);
            statDup++;
            if (sendSupervisionFrame(0x03, C_RR(module_counter)) < 0) return -1;
            continue;
        }

        if (!bcc2Ok)
        {
            // trama esperada mas com dados corrompidos: pede reenvio
            printf("I frame Ns=%d with errors in the data. Sending REJ%d\n", ns, ns);
            statREJSent++;
            if (sendSupervisionFrame(0x03, C_REJ(ns)) < 0) return -1;
            continue;
        }

        memcpy(packet, data, dataLen);
        module_counter = 1 - ns;
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
//  DISC (A=0x03) ------->            comando enviado pelo Tx
//        <------- DISC (A=0x01)      comando enviado pelo Rx
//  UA   (A=0x01) ------->            resposta enviada pelo Tx
int llCloseTx()
{
    int closed = FALSE;
    alarmCount = 0;
    alarmEnabled = FALSE;

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
        int r = maquinadeestados(0x01, 0x0B, TRUE);
        alarm(0);
        alarmEnabled = FALSE;

        if (r == 0) closed = TRUE;
        else { statTimeouts++; printf("Time-out waiting for DISC\n"); }
    }

    if (closed)
    {
        if (sendSupervisionFrame(0x01, 0x07) < 0) perror("writeBytesSerialPort");
        else printf("DISC received. UA sent. Connection closed\n");
    }
    else printf("No DISC received from the Rx. Closing anyway\n");

    printStatistics(TRUE);

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

    // 1) espera o DISC do Tx (a não ser que o llReceive já o tenha recebido)
    while (discReceived == FALSE)
    {
        maquinadeestadosI(data, &size, &ns, &type);

        if (type == T_DISC && size == 0) break;

        if (type == T_I && size >= 1 && ns != module_counter)
        {
            // o Tx não recebeu o último RR e reenviou a última trama I
            printf("Duplicate I frame Ns=%d while closing. Sending RR%d again\n", ns, module_counter);
            statDup++;
            sendSupervisionFrame(0x03, C_RR(module_counter));
        }
    }
    printf("DISC received\n");

    // 2) envia o DISC do Rx (A=0x01) e espera o UA do Tx (A=0x01)
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
        int r = maquinadeestados(0x01, 0x07, TRUE);
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