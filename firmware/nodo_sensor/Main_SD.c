// *****************************************************************************
// Nodo Sensor - Fase 4: Doble bufer (Ping-Pong) y trama estructurada de 512 bytes
// dsPIC33EP256MC202, Fosc = 80 MHz (FRC+PLL), Fcy = 40 MHz
//
// ARQUITECTURA
//   INT1 (pulso fisico, prioridad 5)  -> LLENA el bufer activo (cabecera, ID, timestamp,
//                                        carga util sintetica) y conmuta al bufer
//                                        complementario. Nunca toca la SD.
//   main()                            -> detecta el bufer LLENO, calcula el CRC16 y ejecuta
//                                        SD_Write_Block() sobre ese bufer.
//   urx_1 (UART1 RX, prioridad 7)     -> valida la trama de tiempo del Concentrador y
//                                        resincroniza timestamp_local (como en la Fase 3).
//
//   Estados de cada bufer:  LIBRE -> LLENO (lo marca INT1) -> ESCRIBIENDO (main) -> LIBRE
//   Proteccion contra desbordamiento: INT1 solo escribe en un bufer LIBRE. Si el bufer
//   activo no esta libre, NO lo sobrescribe: cuenta overflowBuf y descarta ese pulso
//   (en la SD se vera como un salto de timestamp).
//
// TRAMA DE 512 BYTES (sectores de datos, desde SECTOR_INICIAL)
//   0-3     : AA 55 AA 55            cabecera fija
//   4-5     : 00 ID                  ID del nodo (0x0001 / 0x0002)
//   6-9     : timestamp uint32       big-endian (contador de sincronismo, +1 por pulso)
//   10-507  : 498 bytes de carga util sintetica, patron 0-255 repetido
//             PATRON_CONTINUO = 0 -> cada sector reinicia en 0x00 (termina en 0xF1)
//             PATRON_CONTINUO = 1 -> el patron continua entre sectores
//   508-509 : CRC16-CCITT-FALSE (poly 0x1021, init 0xFFFF) de los bytes 0..507, big-endian
//   510-511 : 55 AA                  marcador de fin de bloque
//
// SECTOR DE DIAGNOSTICO (SECTOR_DIAG = SECTOR_INICIAL-1), todo big-endian:
//   0-3 'D''I''A''G' | 4-5 ID | 6-9 sector_actual | 10-13 pulsos INT1 | 14-17 sectores llenados
//   18-21 sectores escritos | 22-25 overflow | 26-29 errores SD | 30-33 tramas validas
//   34-37 errores checksum | 38-41 discrepancias pulso/trama | 42-45 saltos de ts en tramas
//   46-49 bytes fuera de trama | 50-53 tramas truncadas | 54-57 errores UART
//   58-61 ocupacion maxima de bufers (1..2) | 62-65 t escritura min (us) | 66-69 max (us)
//   70-73 promedio (us) | 74-77 ultimo (us) | 78-81 escrituras medidas | 510-511 55 AA
//   Se escribe cuando los pulsos se detienen (DIAG_REPOSO_DECIMAS) y, opcionalmente,
//   cada DIAG_CADA_N sectores.
//
// MEDICION DEL TIEMPO DE ESCRITURA
//   - Osciloscopio: el pin SD_ACTIVO (RA1) sube durante SD_Write_Block().
//   - Temporizador: Timer1 (1:256 -> 6.4 us por cuenta) mide cada escritura exitosa.
//
// Proyecto MikroC: Oscillator Frequency = 80.000000 MHz, Oscillator Source = FRCPLL.
// Archivos: Main_SD_Fase4.c, sdcard.c, spiSD.c (+ .h). rs485.c NO va en el nodo.
// *****************************************************************************

#include <spiSD.h>
#include <sdcard.h>
#include <stdbool.h>
#include <stdlib.h>

// RP41 = RB9: entrada SDI1 de SPI1 (MikroC no define RPINR20bits para este chip)
#define RPINR20 (*((volatile unsigned int*)0x06C8))
// RPINR0 (INT1R en los bits 14:8)
#define RPINR0  (*((volatile unsigned int*)0x06A0))

// ------------------------------ CONFIGURACION --------------------------------
#define ID_NODO              1        // 1 en el primer nodo, 2 en el segundo
#define SECTOR_INICIAL       2048UL   // OJO: sobrescribe datos de la tarjeta desde ese sector
#define SECTOR_DIAG          (SECTOR_INICIAL - 1)
#define REINTENTOS_SD        3

#define PATRON_CONTINUO      0        // 0 = reinicia en 0 en cada sector, 1 = continua entre sectores
#define DIAG_CADA_N          0        // 0 = solo al detenerse los pulsos; N = tambien cada N sectores
#define DIAG_REPOSO_DECIMAS  15       // 1.5 s sin pulsos -> escribe el sector de diagnostico

#define OSC_TUN              0
#define BAUD_RS485           500000
#define U1RX_PRIORIDAD       7
#define SILENCIO_US          2000     // silencio que separa tramas (Timer3, 1:8 = 5 MHz)

#define RS485_BYTE_INICIO    0x3A
#define RS485_DIR_BCAST      0xFF
#define RS485_FUNC_TIEMPO    0x54

#define PPS_PIN_RP           46       // RB14 / RP46 -> INT1
// Flanco de INT1: 0 = automatico, 1 = bajada, 2 = subida.
// En la Fase 3 la polaridad detectada fue siempre 0 (flanco de subida): se fuerza.
#define PPS_FLANCO           2

// Estados de los bufers
#define BUF_LIBRE            0
#define BUF_LLENO            1
#define BUF_ESCRIBIENDO      2
// -----------------------------------------------------------------------------

// Pines
sbit TEST at LATA2_bit;                    // LED D1
sbit TEST_Direction at TRISA2_bit;
sbit CsADXL at LATA3_bit;
sbit CsADXL_Direction at TRISA3_bit;
sbit sd_CS_lat at LATB0_bit;
sbit sd_CS_tris at TRISB0_bit;
sbit sd_detect_port at PORTA.B4;
sbit sd_detect_tris at TRISA4_bit;
sbit MSRS485 at LATB12_bit;                // Control MAX485 (DE/RE)
sbit MSRS485_Direction at TRISB12_bit;
sbit SD_ACTIVO at LATA1_bit;               // En alto durante SD_Write_Block (osciloscopio). Verifica que RA1 este libre
sbit SD_ACTIVO_Direction at TRISA1_bit;

// Requerido por sdcard.c
struct sdflags sdflags;

// CRC16-CCITT-FALSE por nibbles (16 entradas en flash)
const unsigned int crcTab4[16] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF
};

// ------------------------------ VARIABLES ------------------------------------
// Doble bufer
unsigned char Buffer_A[512];
unsigned char Buffer_B[512];
unsigned char bufferDiag[512];
unsigned char *bufPtr[2];                  // bufPtr[0] = Buffer_A, bufPtr[1] = Buffer_B
volatile unsigned char estadoBuf[2];       // BUF_LIBRE / BUF_LLENO / BUF_ESCRIBIENDO
unsigned char idxActivo;                   // bufer que llena INT1 (solo lo toca la ISR)
unsigned char idxEscritura;                // bufer que escribe main (solo lo toca main)

// Reloj y sincronismo
volatile unsigned long  timestamp_local;   // contador local: +1 por pulso INT1, resincronizado por trama
volatile unsigned char  sincronizado;      // 1 tras recibir la primera trama valida
volatile unsigned char  banPulso;          // INT1 lo sube; main lo baja (deteccion de reposo)

// Contadores (ISR -> main)
volatile unsigned long  pulsosPPS;
volatile unsigned long  sectoresLlenados;
volatile unsigned long  overflowBuf;
volatile unsigned char  maxOcupacion;
volatile unsigned long  tramasValidas;
volatile unsigned long  discrepanciasPPS;
volatile unsigned long  saltosTiempo;
volatile unsigned long  erroresChecksum;
volatile unsigned long  cntFueraTrama;
volatile unsigned long  cntSilencioMedio;
volatile unsigned long  cntErrorUART;

// Recepcion de la trama de tiempo
unsigned char rxBuf[8];
unsigned char rxIdx;
unsigned char rxChk;
volatile unsigned long  tsAnterior;
volatile unsigned char  hayAnterior;
volatile unsigned char  rxDescarte;

// Registro SD (solo main)
unsigned long sector_actual;
unsigned long sectoresEscritos;
unsigned long erroresSD;
unsigned char sd_ultimo_wr;
unsigned long tWrMinUs, tWrMaxUs, tWrSumaUs, tWrUltimoUs, tWrN;

// Prototipos
void ConfiguracionPrincipal();
void ConfigurarPPS_SPI1();
void ConfigurarPPS_UART1();
void ConfigurarPPS_INT1();
void ConfigurarTimers();
void ErrorFatal();
void VaciarUART();
unsigned int CRC16(unsigned char *d, unsigned int n);
void ProcesarBuffer(unsigned char *buf);
unsigned char GuardarBloque(unsigned char *buf, unsigned long sector, unsigned char medir);
void EscribirDiag();
void PonerU32(unsigned char *b, unsigned int pos, unsigned long v);

// *****************************************************************************
void main() {
    unsigned int  k;
    unsigned char estadoSD;
    unsigned char decimas;
    unsigned char diagHecho;

    TRISAbits.TRISA2 = 0;
    LATAbits.LATA2 = 1;                   // marca de vida

    ConfiguracionPrincipal();

    for (k = 0; k < 3; k++) {             // 3 parpadeos: configuracion terminada
        TEST = 1;  Delay_ms(200);
        TEST = 0;  Delay_ms(200);
    }
    Delay_ms(1000);

    estadoSD = SD_Init_Try(5);
    if (estadoSD != SUCCESSFUL_INIT) {
        ErrorFatal();
    }

    // Doble bufer
    bufPtr[0] = Buffer_A;
    bufPtr[1] = Buffer_B;
    for (k = 0; k < 512; k++) { Buffer_A[k] = 0; Buffer_B[k] = 0; }
    estadoBuf[0] = BUF_LIBRE;
    estadoBuf[1] = BUF_LIBRE;
    idxActivo = 0;
    idxEscritura = 0;

    sector_actual     = SECTOR_INICIAL;
    sectoresEscritos  = 0;
    erroresSD         = 0;
    tWrMinUs = 0xFFFFFFFFUL;  tWrMaxUs = 0;  tWrSumaUs = 0;  tWrUltimoUs = 0;  tWrN = 0;

    sectoresLlenados = 0;  overflowBuf = 0;  maxOcupacion = 0;
    tramasValidas = 0;  discrepanciasPPS = 0;  saltosTiempo = 0;  erroresChecksum = 0;
    cntFueraTrama = 0;  cntSilencioMedio = 0;  cntErrorUART = 0;
    hayAnterior = 0;  tsAnterior = 0;
    pulsosPPS = 0;  timestamp_local = 0;
    sincronizado = 0;  banPulso = 0;

    VaciarUART();
    rxIdx = 0;
    rxChk = 0;
    T3IF_bit = 0;  TMR3 = 0;
    T2IF_bit = 0;  TMR2 = 0;
    U1RXIF_bit = 0;
    INT1IF_bit = 0;

    decimas = 0;
    diagHecho = 0;
    T2CON.TON = 1;                        // base de 100 ms para detectar reposo

    U1RXIE_bit = 1;
    INT1IE_bit = 1;

    while (1) {
        asm CLRWDT;

        if (U1STAbits.OERR == 1) {        // proteccion contra OERR
            U1RXIE_bit = 0;
            U1STAbits.OERR = 0;
            rxIdx = 0;
            erroresChecksum++;
            U1RXIE_bit = 1;
        }

        // ---- Bufer lleno: CRC + escritura en SD. INT1 sigue llenando el otro bufer ----
        if (estadoBuf[idxEscritura] == BUF_LLENO) {
            estadoBuf[idxEscritura] = BUF_ESCRIBIENDO;
            ProcesarBuffer(bufPtr[idxEscritura]);
            estadoBuf[idxEscritura] = BUF_LIBRE;
            idxEscritura ^= 1;
            continue;                     // prioridad al vaciado de bufers
        }

        // ---- Reposo: sin pulsos durante DIAG_REPOSO_DECIMAS -> sector de diagnostico ----
        if (T2IF_bit) {
            T2IF_bit = 0;
            if (banPulso) {
                banPulso = 0;
                decimas = 0;
                diagHecho = 0;
            } else if (decimas < 255) {
                decimas++;
            }
            if ((decimas >= DIAG_REPOSO_DECIMAS) && (diagHecho == 0) && (sincronizado == 1)) {
                EscribirDiag();
                diagHecho = 1;
            }
        }
    }
}

// *****************************************************************************
// CRC16-CCITT-FALSE (poly 0x1021, init 0xFFFF), calculo por nibbles
// *****************************************************************************
unsigned int CRC16(unsigned char *d, unsigned int n) {
    unsigned int crc;
    unsigned int i;
    crc = 0xFFFF;
    for (i = 0; i < n; i++) {
        crc = crc ^ ((unsigned int)d[i] << 8);
        crc = (crc << 4) ^ crcTab4[crc >> 12];
        crc = (crc << 4) ^ crcTab4[crc >> 12];
    }
    return crc;
}

// *****************************************************************************
// Completa el CRC del bufer lleno y lo escribe en la SD
// *****************************************************************************
void ProcesarBuffer(unsigned char *buf) {
    unsigned int crc;

    crc = CRC16(buf, 508);
    buf[508] = (unsigned char)(crc >> 8);
    buf[509] = (unsigned char)(crc);

    if (GuardarBloque(buf, sector_actual, 1)) {
        sectoresEscritos++;
        sector_actual++;
#if DIAG_CADA_N > 0
        if ((sectoresEscritos % DIAG_CADA_N) == 0) EscribirDiag();
#endif
    } else {
        erroresSD++;                      // el sector se pierde; el siguiente reutiliza el mismo numero
    }
}

// *****************************************************************************
// Escribe un bloque con reintentos. Si medir = 1 mide el tiempo de cada escritura
// exitosa con Timer1 (6.4 us por cuenta) y levanta SD_ACTIVO para el osciloscopio.
// *****************************************************************************
unsigned char GuardarBloque(unsigned char *buf, unsigned long sector, unsigned char medir) {
    unsigned char n, r;
    unsigned int  t;
    unsigned long tUs;

    for (n = 0; n < REINTENTOS_SD; n++) {
        if (medir) { SD_ACTIVO = 1; TMR1 = 0; T1IF_bit = 0; }
        r = SD_Write_Block(buf, sector);
        if (medir) {
            t = TMR1;
            if (T1IF_bit) t = 0xFFFF;     // saturado: > 419 ms
            SD_ACTIVO = 0;
        }
        sd_ultimo_wr = r;
        if (r == DATA_ACCEPTED) {
            if (medir) {
                tUs = ((unsigned long)t * 32UL) / 5UL;     // cuentas de 6.4 us -> us
                tWrUltimoUs = tUs;
                if (tUs < tWrMinUs) tWrMinUs = tUs;
                if (tUs > tWrMaxUs) tWrMaxUs = tUs;
                tWrSumaUs += tUs;
                tWrN++;
            }
            return 1;
        }
        if (n == 1) SD_Init_Try(3);       // tras el segundo fallo reinicia la tarjeta
    }
    return 0;
}

// *****************************************************************************
// Sector de diagnostico: contadores y estadistica de tiempos de escritura
// *****************************************************************************
void EscribirDiag() {
    unsigned int  k;
    unsigned long sPulsos, sLlen, sOvf, sTramas, sChk, sDisc, sSalt, sFuera, sSil, sUart;
    unsigned char sOcup;
    unsigned long prom, tmin;

    U1RXIE_bit = 0;                       // instantanea coherente de los contadores
    INT1IE_bit = 0;
    sPulsos = pulsosPPS;   sLlen = sectoresLlenados;  sOvf = overflowBuf;   sOcup = maxOcupacion;
    sTramas = tramasValidas;  sChk = erroresChecksum;  sDisc = discrepanciasPPS;
    sSalt = saltosTiempo;  sFuera = cntFueraTrama;  sSil = cntSilencioMedio;  sUart = cntErrorUART;
    INT1IE_bit = 1;
    U1RXIE_bit = 1;

    prom = 0;
    tmin = tWrMinUs;
    if (tWrN > 0) prom = tWrSumaUs / tWrN; else tmin = 0;

    for (k = 0; k < 512; k++) bufferDiag[k] = 0;
    bufferDiag[0] = 'D';  bufferDiag[1] = 'I';  bufferDiag[2] = 'A';  bufferDiag[3] = 'G';
    bufferDiag[4] = 0x00; bufferDiag[5] = ID_NODO;
    PonerU32(bufferDiag, 6,  sector_actual);
    PonerU32(bufferDiag, 10, sPulsos);
    PonerU32(bufferDiag, 14, sLlen);
    PonerU32(bufferDiag, 18, sectoresEscritos);
    PonerU32(bufferDiag, 22, sOvf);
    PonerU32(bufferDiag, 26, erroresSD);
    PonerU32(bufferDiag, 30, sTramas);
    PonerU32(bufferDiag, 34, sChk);
    PonerU32(bufferDiag, 38, sDisc);
    PonerU32(bufferDiag, 42, sSalt);
    PonerU32(bufferDiag, 46, sFuera);
    PonerU32(bufferDiag, 50, sSil);
    PonerU32(bufferDiag, 54, sUart);
    PonerU32(bufferDiag, 58, (unsigned long)sOcup);
    PonerU32(bufferDiag, 62, tmin);
    PonerU32(bufferDiag, 66, tWrMaxUs);
    PonerU32(bufferDiag, 70, prom);
    PonerU32(bufferDiag, 74, tWrUltimoUs);
    PonerU32(bufferDiag, 78, tWrN);
    bufferDiag[510] = 0x55;
    bufferDiag[511] = 0xAA;

    GuardarBloque(bufferDiag, SECTOR_DIAG, 0);
}

void PonerU32(unsigned char *b, unsigned int pos, unsigned long v) {
    b[pos]     = (unsigned char)(v >> 24);
    b[pos + 1] = (unsigned char)(v >> 16);
    b[pos + 2] = (unsigned char)(v >> 8);
    b[pos + 3] = (unsigned char)(v);
}

// Vacia el FIFO del UART1 y limpia OERR (destino volatile para que no se elimine la lectura)
void VaciarUART() {
    unsigned char n;
    for (n = 0; (n < 16) && (U1STAbits.URXDA == 1); n++) {
        rxDescarte = U1RXREG;
    }
    U1STAbits.OERR = 0;
}

void ErrorFatal() {
    while (1) {
        asm CLRWDT;
        TEST = ~TEST;
        Delay_ms(100);
    }
}

// *****************************************************************************
// UART1 RX por interrupcion: maquina de estados de la trama de tiempo (Fase 3)
// Aqui NO se escribe en los bufers: solo se valida y se resincroniza el contador local.
// *****************************************************************************
void urx_1() org IVT_ADDR_U1RXINTERRUPT {
    unsigned char b;
    unsigned long ts;

    U1RXIF_bit = 0;

    if (U1STAbits.OERR == 1) {
        U1STAbits.OERR = 0;
        rxIdx = 0;
        erroresChecksum++;
    }

    while (U1STAbits.URXDA) {
        if ((U1STAbits.FERR == 1) || (U1STAbits.PERR == 1)) cntErrorUART++;
        b = U1RXREG;

        if (T3IF_bit) {                   // hubo silencio: nueva trama
            if (rxIdx != 0) cntSilencioMedio++;
            rxIdx = 0;
        }
        T3IF_bit = 0;
        TMR3 = 0;

        if (rxIdx == 0) {
            if (b == RS485_BYTE_INICIO) {
                rxBuf[0] = b;
                rxChk = b;
                rxIdx = 1;
            } else {
                cntFueraTrama++;
            }
        }
        else if (rxIdx < 7) {
            rxBuf[rxIdx] = b;
            rxChk = rxChk + b;
            rxIdx++;
        }
        else {                            // rxIdx == 7: checksum
            rxIdx = 0;
            if ((b == rxChk) &&
                (rxBuf[2] == RS485_FUNC_TIEMPO) &&
                ((rxBuf[1] == RS485_DIR_BCAST) || (rxBuf[1] == ID_NODO))) {

                ts = ((unsigned long)rxBuf[3] << 24) | ((unsigned long)rxBuf[4] << 16) |
                     ((unsigned long)rxBuf[5] << 8)  |  (unsigned long)rxBuf[6];

                tramasValidas++;
                if (hayAnterior && (ts != (tsAnterior + 1))) saltosTiempo++;
                tsAnterior = ts;
                hayAnterior = 1;

                // El pulso llega antes que la trama: timestamp_local ya debe valer ts.
                // Solo se escribe si discrepa (evita carreras con INT1, que no puede
                // interrumpir esta ISR pero si ser interrumpida por ella).
                if (timestamp_local != ts) {
                    if (sincronizado) discrepanciasPPS++;
                    timestamp_local = ts;
                }
                sincronizado = 1;

                if ((ts & 1) == 0) TEST = 1; else TEST = 0;    // CP 3.1: LED segun paridad
            } else {
                erroresChecksum++;
            }
        }
    }
}

// *****************************************************************************
// INT1: pulso fisico de sincronismo. Alimenta de forma determinista el bufer activo.
// Duracion tipica: ~100-150 us (llenado de 498 bytes). No usa la SD ni delays.
// *****************************************************************************
void int_1() org IVT_ADDR_INT1INTERRUPT {
    unsigned char *p;
    unsigned char v;
    unsigned int  i;
    unsigned long ts;
    unsigned char ocupados;

    INT1IF_bit = 0;
    timestamp_local++;
    pulsosPPS++;
    banPulso = 1;

    if (sincronizado == 0) return;        // aun sin trama de tiempo valida: no se registra

    // Proteccion contra desbordamiento: nunca sobrescribir un bufer pendiente
    if (estadoBuf[idxActivo] != BUF_LIBRE) {
        overflowBuf++;
        return;
    }

    ts = timestamp_local;
    p  = bufPtr[idxActivo];

    p[0] = 0xAA;  p[1] = 0x55;  p[2] = 0xAA;  p[3] = 0x55;     // cabecera
    p[4] = 0x00;  p[5] = ID_NODO;                              // ID del nodo
    p[6] = (unsigned char)(ts >> 24);                          // timestamp big-endian
    p[7] = (unsigned char)(ts >> 16);
    p[8] = (unsigned char)(ts >> 8);
    p[9] = (unsigned char)(ts);

#if PATRON_CONTINUO == 1
    v = (unsigned char)((sectoresLlenados * 498UL) & 0xFFUL);
#else
    v = 0;
#endif
    for (i = 0; i < 498; i++) {                                // carga util sintetica 0-255
        p[10 + i] = v;
        v++;
    }

    p[508] = 0x00;  p[509] = 0x00;                             // CRC: lo completa main
    p[510] = 0x55;  p[511] = 0xAA;                             // fin de bloque

    sectoresLlenados++;
    estadoBuf[idxActivo] = BUF_LLENO;                          // bandera de escritura
    idxActivo ^= 1;                                            // conmuta al bufer complementario

    ocupados = 0;
    if (estadoBuf[0] != BUF_LIBRE) ocupados++;
    if (estadoBuf[1] != BUF_LIBRE) ocupados++;
    if (ocupados > maxOcupacion) maxOcupacion = ocupados;
}

// *****************************************************************************
void ConfiguracionPrincipal() {
    unsigned long guardiaPLL;

    CLKDIVbits.FRCDIV = 0;
    CLKDIVbits.PLLPOST = 0;
    CLKDIVbits.PLLPRE = 5;
    PLLFBDbits.PLLDIV = 150;
    OSCTUN = (OSC_TUN & 0x3F);
    guardiaPLL = 400000;
    while ((OSCCONbits.LOCK != 1) && (guardiaPLL > 0)) guardiaPLL--;
    if (OSCCONbits.LOCK != 1) {
        TRISAbits.TRISA2 = 0;
        while (1) { LATAbits.LATA2 = ~LATAbits.LATA2; Delay_ms(50); }
    }
    Delay_ms(100);
    LATAbits.LATA2 = 0;

    ANSELA = 0;
    ANSELB = 0;

    ConfigurarPPS_SPI1();
    ConfigurarPPS_UART1();
    ConfigurarPPS_INT1();

    TEST_Direction = 0;
    TEST = 0;
    SD_ACTIVO_Direction = 0;
    SD_ACTIVO = 0;

    CsADXL_Direction = 0;
    CsADXL = 1;
    sd_CS_tris = 0;
    sd_CS_lat = 1;
    sd_detect_tris = 1;
    CNPUAbits.CNPUA4 = 1;

    MSRS485_Direction = 0;
    MSRS485 = 0;                          // MAX485 siempre en recepcion

    UART1_Init_Advanced(BAUD_RS485, _UART_8BIT_NOPARITY, _UART_ONE_STOPBIT, _UART_HI_SPEED);
    U1STAbits.URXISEL = 0x00;
    IPC2 = (IPC2 & 0x8FFF) | ((unsigned int)U1RX_PRIORIDAD << 12);
    U1RXIF_bit = 0;
    U1RXIE_bit = 0;

    ConfigurarTimers();

    TRISBbits.TRISB14 = 1;
    Delay_ms(5);
#if PPS_FLANCO == 1
    INTCON2.INT1EP = 1;
#elif PPS_FLANCO == 2
    INTCON2.INT1EP = 0;
#else
    if (PORTBbits.RB14 == 1) INTCON2.INT1EP = 1; else INTCON2.INT1EP = 0;
#endif
    IPC5bits.INT1IP = 0x05;
    INT1IF_bit = 0;
    INT1IE_bit = 0;                       // se habilita en main() tras iniciar la SD

    INTCON2.GIE = 1;
}

// Timer3: silencio entre tramas (1:8 -> 5 MHz). Timer2: base de 100 ms (1:256). Timer1: mide la escritura SD.
void ConfigurarTimers() {
    T3CON = 0x0010;
    TMR3 = 0;
    PR3 = (unsigned int)(SILENCIO_US * 5UL);
    T3IF_bit = 0;
    T3IE_bit = 0;
    T3CON.TON = 1;

    T2CON = 0x0030;                       // 1:256 -> 156250 Hz, apagado hasta main()
    TMR2 = 0;
    PR2 = 15624;                          // 100 ms exactos
    T2IF_bit = 0;
    T2IE_bit = 0;

    T1CON = 0x0030;                       // 1:256 -> 6.4 us por cuenta
    TMR1 = 0;
    PR1 = 0xFFFF;
    T1IF_bit = 0;
    T1IE_bit = 0;
    T1CON.TON = 1;
}

void ConfigurarPPS_SPI1() {
    asm {
        MOV #0x46, W0
        MOV #0x57, W1
        MOV #0x0742, W2
        MOV.b W0, [W2]
        MOV.b W1, [W2]
        BCLR OSCCON, #6
    }
    RPOR2 = (RPOR2 & 0x00FF) | 0x0600;    // RP39 (RB7) -> SCK1
    RPOR3 = (RPOR3 & 0xFF00) | 0x0005;    // RP40 (RB8) -> SDO1
    RPINR20 = (RPINR20 & 0xFF00) | 41;    // RP41 (RB9) <- SDI1
    TRISBbits.TRISB7 = 0;
    TRISBbits.TRISB8 = 0;
    TRISBbits.TRISB9 = 1;
}

void ConfigurarPPS_UART1() {
    asm {
        MOV #0x46, W0
        MOV #0x57, W1
        MOV #0x0742, W2
        MOV.b W0, [W2]
        MOV.b W1, [W2]
        BCLR OSCCON, #6
    }
    RPINR18bits.U1RXR = 47;  // RX en RB15
    RPOR1bits.RP36R = 1;     // TX en RB4
    TRISBbits.TRISB15 = 1;
    TRISBbits.TRISB4 = 0;
}

void ConfigurarPPS_INT1() {
    asm {
        MOV #0x46, W0
        MOV #0x57, W1
        MOV #0x0742, W2
        MOV.b W0, [W2]
        MOV.b W1, [W2]
        BCLR OSCCON, #6
    }
    TRISBbits.TRISB14 = 1;
    RPINR0 = (RPINR0 & 0x80FF) | ((unsigned int)PPS_PIN_RP << 8);   // INT1 <- RP46
}