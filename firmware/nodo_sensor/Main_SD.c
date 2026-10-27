// *****************************************************************************
// Nodo Sensor - Fase 3: recepcion de tiempo por RS485 y registro en MicroSD
// dsPIC33EP256MC202, Fosc = 80 MHz (FRC+PLL), Fcy = 40 MHz
//
// Recepcion de la trama por INTERRUPCION del UART1 (urx_1) y conteo de segundos
// por la interrupcion fisica INT1 (pulso INT_SINC del Concentrador).
//
// Funcionamiento:
//  - urx_1 decodifica con una maquina de estados la trama:
//      [0x3A][DIR][0x54][TS3][TS2][TS1][TS0][CHK]     CHK = suma bytes 0..6 (8 bits)
//    Si checksum, funcion y direccion (255 o ID_NODO) son validos: guarda el
//    timestamp, resincroniza el contador local, levanta banTramaValida y cambia el
//    LED TEST segun la paridad del timestamp (CP 3.1).
//  - Cuando termina el silencio posterior a la trama (Timer2, SILENCIO_US) el bucle
//    principal arma un sector de 512 bytes y lo escribe en la SD (CP 3.2).
//  - INT1 (RB14/RP46) incrementa timestamp_local y pulsosPPS una vez por segundo.
//    Prioridades: urx_1 = 7, INT1 = 5.
//  - Solo se escriben timestamps RECIBIDOS por RS485 (no se reconstruye ninguno). Los
//    contadores de diagnostico (bytes 28-33) indican por que se perderia una trama.
//
// Formato del sector (todo big-endian):
//   0-1   : A5 5A (marca)
//   2     : ID del nodo
//   3     : 0x45 | polaridad<<1  (bit2 = recepcion por urx_1; bit1 = reposo alto)
//   4-7   : timestamp del sector
//   8-11  : contador de paquetes (1, 2, 3, ...)
//   12-15 : numero de sector donde se escribe
//   16-17 : errores de checksum / formato / OERR acumulados
//   18-19 : saltos de timestamp en tramas RECIBIDAS (ts != ts_anterior + 1)
//   20-21 : errores de escritura en SD acumulados
//   22-23 : tramas descartadas (llego otra trama valida antes de guardar la anterior)
//   24-25 : pulsos fisicos contados por INT1
//   26-27 : discrepancias entre el contador de pulsos y el timestamp de la trama
//   28-29 : bytes descartados fuera de trama (primer byte distinto de 0x3A)
//   30-31 : tramas truncadas por silencio a mitad de trama (> SILENCIO_US entre bytes)
//   32-33 : errores de UART (FERR/PERR) en bytes recibidos
//   34-511: 00
//
// Proyecto MikroC: Oscillator Frequency = 80.000000 MHz, Oscillator Source = FRCPLL.
// Archivos del proyecto: Main_SD.c, sdcard.c, spiSD.c (+ sus .h).
// *****************************************************************************

#include <spiSD.h>
#include <sdcard.h>
#include <stdbool.h>
#include <stdlib.h>

// RP41 = RB9: entrada SDI1 de SPI1 (MikroC no define RPINR20bits para este chip)
#define RPINR20 (*((volatile unsigned int*)0x06C8))

// RPINR0 (INT1R en los bits 14:8). En esta familia RPINRn esta en 0x06A0 + 2*n.
#define RPINR0  (*((volatile unsigned int*)0x06A0))

// ------------------------------ CONFIGURACION --------------------------------
#define ID_NODO            2        // Poner 2 en el segundo nodo
#define SECTOR_INICIAL     2048UL   // OJO: sobrescribe datos de la tarjeta desde ese sector
#define REINTENTOS_SD      3

// Ajuste fino del oscilador interno (FRC): -32 a +31, cada paso = 0.375 %.
#define OSC_TUN            0

// Baud del bus RS485. Debe ser IGUAL en el Concentrador y en los nodos.
#define BAUD_RS485         500000

// Prioridad de la interrupcion del UART1 RX (1 a 7).
#define U1RX_PRIORIDAD     7

// Silencio (us) que separa una trama de la siguiente. Si pasa este tiempo entre dos bytes
// se descarta la trama en curso. Timer2 con prescaler 1:8 (5 MHz): maximo ~13000 us.
// Antes eran 400 us; 2000 us tolera pausas entre bytes del Concentrador y sigue siendo
// mucho menor que el segundo entre tramas.
#define SILENCIO_US        2000

#define RS485_BYTE_INICIO  0x3A
#define RS485_DIR_BCAST    0xFF
#define RS485_FUNC_TIEMPO  0x54

// Pulso pps fisico: RB14 / RP46 (salida RO del MAX485 U5, red INT_SINC) -> INT1
#define PPS_PIN_RP         46

// Flanco de INT1: 0 = automatico (segun el nivel de reposo), 1 = bajada, 2 = subida.
#define PPS_FLANCO         0

// Byte 3 de cada sector: 0x45 = version con INT1 y recepcion por urx_1
#define FW_FLAGS           0x45
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

// Requerido por sdcard.c (extern struct sdflags sdflags)
struct sdflags sdflags;

// ------------------------------ VARIABLES ------------------------------------
// Compartidas con las ISR (urx_1 e INT1)
volatile unsigned long  timestamp_local;    // Contador local (trama y pulso pps)
volatile unsigned int   pulsosPPS;          // Pulsos fisicos contados por INT1

// Recepcion de la trama (maquina de estados de urx_1)
unsigned char rxBuf[8];
unsigned char rxIdx;
unsigned char rxChk;
volatile unsigned short banTramaValida;     // 1 = hay una trama valida pendiente de guardar
volatile unsigned long  tsRecibido;         // ultimo timestamp valido recibido

// Diagnostico (los incrementa urx_1; main solo los lee)
volatile unsigned long  tsAnterior;
volatile unsigned short hayAnterior;
volatile unsigned int   erroresChecksum;
volatile unsigned int   saltosTiempo;
volatile unsigned int   tramasPerdidas;
volatile unsigned int   discrepanciasPPS;   // tramas cuyo timestamp no coincide con el contador de pulsos
volatile unsigned int   cntFueraTrama;      // bytes descartados fuera de trama
volatile unsigned int   cntSilencioMedio;   // tramas truncadas por silencio a mitad de trama
volatile unsigned int   cntErrorUART;       // FERR / PERR

// Registro SD (solo main)
unsigned char bufferSector[512];
unsigned long sector_actual;
unsigned long contadorPaquetes;
unsigned int  erroresSD;
unsigned char sd_ultimo_wr;                 // ultimo codigo devuelto por SD_Write_Block
unsigned char polaridadPPS = 0;             // 1 = reposo alto (flanco de bajada), 0 = reposo bajo
volatile unsigned char rxDescarte;          // destino de bytes descartados (volatile: no se elimina la lectura)

// Prototipos
void ConfiguracionPrincipal();
void ConfigurarPPS_SPI1();
void ConfigurarPPS_UART1();
void ConfigurarPPS_INT1();
void ConfigurarTimeoutRx();
void ErrorFatal();
void VaciarUART();
void EscribirSector(unsigned long ts);
unsigned char GuardarSector(unsigned long sector);
void PonerU16(unsigned int pos, unsigned int v);
void PonerU32(unsigned int pos, unsigned long v);

// *****************************************************************************
void main() {
    unsigned long ts;
    unsigned int  k;
    unsigned char estadoSD;

    // Marca de vida: LED encendido al arrancar, antes de configurar el reloj.
    TRISAbits.TRISA2 = 0;
    LATAbits.LATA2 = 1;

    ConfiguracionPrincipal();

    // Marca C: la configuracion termino. Tres parpadeos y 1 s apagado.
    for (k = 0; k < 3; k++) {
        TEST = 1;
        Delay_ms(200);
        TEST = 0;
        Delay_ms(200);
    }
    Delay_ms(1000);

    // Inicializacion de la SD. Si falla: parpadeo rapido continuo.
    estadoSD = SD_Init_Try(5);
    if (estadoSD != SUCCESSFUL_INIT) {
        ErrorFatal();
    }

    for (k = 0; k < 512; k++) bufferSector[k] = 0x00;
    sector_actual       = SECTOR_INICIAL;
    contadorPaquetes    = 0;
    erroresSD           = 0;
    erroresChecksum     = 0;
    saltosTiempo        = 0;
    tramasPerdidas      = 0;
    discrepanciasPPS    = 0;
    cntFueraTrama       = 0;
    cntSilencioMedio    = 0;
    cntErrorUART        = 0;
    hayAnterior         = 0;
    tsAnterior          = 0;
    pulsosPPS           = 0;
    timestamp_local     = 0;

    // Durante el arranque de la SD llegaron bytes con la interrupcion apagada: se vacia el FIFO
    // (con OERR = 1 el UART no recibe ni interrumpe hasta limpiarlo) y se reinicia la trama.
    VaciarUART();
    rxIdx = 0;
    rxChk = 0;
    banTramaValida = 0;
    tsRecibido = 0;
    T2IF_bit = 0;
    TMR2 = 0;
    U1RXIF_bit = 0;
    INT1IF_bit = 0;

    U1RXIE_bit = 1;                   // la trama entra por la interrupcion urx_1
    INT1IE_bit = 1;                   // el pulso fisico cuenta los segundos

    while (1) {
        asm CLRWDT;

        // Proteccion: con OERR = 1 el UART deja de recibir y no vuelve a interrumpir.
        if (U1STAbits.OERR == 1) {
            U1RXIE_bit = 0;
            U1STAbits.OERR = 0;
            rxIdx = 0;
            erroresChecksum++;
            U1RXIE_bit = 1;
        }

        if (banTramaValida && (T2IF_bit == 1)) {
            // Trama valida y silencio posterior: se guarda. Copia atomica del timestamp.
            U1RXIE_bit = 0;
            ts = tsRecibido;
            banTramaValida = 0;
            U1RXIE_bit = 1;

            EscribirSector(ts);
        }
    }
}

// *****************************************************************************
// Arma y escribe un sector con el timestamp recibido.
// *****************************************************************************
void EscribirSector(unsigned long ts) {
    unsigned long paquete;

    paquete = contadorPaquetes + 1;

    bufferSector[0] = 0xA5;
    bufferSector[1] = 0x5A;
    bufferSector[2] = ID_NODO;
    bufferSector[3] = FW_FLAGS | (polaridadPPS << 1);
    PonerU32(4,  ts);
    PonerU32(8,  paquete);
    PonerU32(12, sector_actual);
    PonerU16(16, erroresChecksum);
    PonerU16(18, saltosTiempo);
    PonerU16(20, erroresSD);
    PonerU16(22, tramasPerdidas);
    PonerU16(24, pulsosPPS);
    PonerU16(26, discrepanciasPPS);
    PonerU16(28, cntFueraTrama);
    PonerU16(30, cntSilencioMedio);
    PonerU16(32, cntErrorUART);

    if (GuardarSector(sector_actual)) {
        contadorPaquetes = paquete;
        sector_actual++;
    } else {
        erroresSD++;                    // el mismo sector se reintenta en la siguiente escritura
    }
}

// *****************************************************************************
// Escribe bufferSector en 'sector' con reintentos y reinicializacion de la SD.
// Devuelve 1 si la escritura fue aceptada.
// *****************************************************************************
unsigned char GuardarSector(unsigned long sector) {
    unsigned char n, r;
    for (n = 0; n < REINTENTOS_SD; n++) {
        r = SD_Write_Block(bufferSector, sector);
        sd_ultimo_wr = r;
        if (r == DATA_ACCEPTED) return 1;
        if (n == 1) SD_Init_Try(3);      // Tras el segundo fallo, reinicia la tarjeta
    }
    return 0;
}

// Vacia el FIFO del UART1 (4 bytes) y limpia OERR.
// El destino es una variable volatile y el bucle tiene tope: si solo se asignara a una local
// sin uso, el optimizador de MikroC elimina la lectura de U1RXREG y el bucle se cuelga.
void VaciarUART() {
    unsigned char n;
    for (n = 0; (n < 16) && (U1STAbits.URXDA == 1); n++) {
        rxDescarte = U1RXREG;
    }
    U1STAbits.OERR = 0;
}

void PonerU16(unsigned int pos, unsigned int v) {
    bufferSector[pos]     = (unsigned char)(v >> 8);
    bufferSector[pos + 1] = (unsigned char)(v);
}

void PonerU32(unsigned int pos, unsigned long v) {
    bufferSector[pos]     = (unsigned char)(v >> 24);
    bufferSector[pos + 1] = (unsigned char)(v >> 16);
    bufferSector[pos + 2] = (unsigned char)(v >> 8);
    bufferSector[pos + 3] = (unsigned char)(v);
}

void ErrorFatal() {
    while (1) {
        asm CLRWDT;
        TEST = ~TEST;
        Delay_ms(100);
    }
}

// *****************************************************************************
// Recepcion RS485 (UART1) por interrupcion - maquina de estados de la trama.
// Timer2 sirve de deteccion de silencio: si pasan > SILENCIO_US sin bytes, T2IF queda en 1
// y la trama siguiente empieza desde cero.
// *****************************************************************************
void urx_1() org IVT_ADDR_U1RXINTERRUPT {
    unsigned char b;
    unsigned long ts;

    U1RXIF_bit = 0;                       // Limpiar al inicio, no al final

    if (U1STAbits.OERR == 1) {            // Desbordamiento de FIFO
        U1STAbits.OERR = 0;
        rxIdx = 0;
        erroresChecksum++;
    }

    while (U1STAbits.URXDA) {
        // Los flags de error corresponden al byte que esta al frente del FIFO: se leen antes
        if ((U1STAbits.FERR == 1) || (U1STAbits.PERR == 1)) cntErrorUART++;
        b = U1RXREG;

        if (T2IF_bit) {                   // Hubo silencio: nueva trama
            if (rxIdx != 0) cntSilencioMedio++;   // ...pero la anterior no estaba completa
            rxIdx = 0;
        }
        T2IF_bit = 0;
        TMR2 = 0;

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
        else {                            // rxIdx == 7: byte de checksum
            rxIdx = 0;
            if ((b == rxChk) &&
                (rxBuf[2] == RS485_FUNC_TIEMPO) &&
                ((rxBuf[1] == RS485_DIR_BCAST) || (rxBuf[1] == ID_NODO))) {

                ts = ((unsigned long)rxBuf[3] << 24) | ((unsigned long)rxBuf[4] << 16) |
                     ((unsigned long)rxBuf[5] << 8)  |  (unsigned long)rxBuf[6];

                if (hayAnterior && (ts != (tsAnterior + 1))) saltosTiempo++;
                // El pulso llega antes que la trama: el contador de pulsos ya debe valer ts.
                // INT1 (prioridad 5) no puede interrumpir esta ISR (prioridad 7): lectura atomica.
                if (hayAnterior && (timestamp_local != ts)) discrepanciasPPS++;
                tsAnterior = ts;
                hayAnterior = 1;

                if (banTramaValida) tramasPerdidas++;   // main no alcanzo a guardar la anterior

                tsRecibido = ts;
                timestamp_local = ts;                   // Resincroniza el contador local
                banTramaValida = 1;

                // CP 3.1: LED segun la paridad del timestamp (encendido con ts par)
                if ((ts & 1) == 0) TEST = 1; else TEST = 0;
            } else {
                erroresChecksum++;
            }
        }
    }
}

// *****************************************************************************
// Pulso fisico de sincronismo (INT1): avanza el contador local
// *****************************************************************************
void int_1() org IVT_ADDR_INT1INTERRUPT {
    INT1IF_bit = 0;
    timestamp_local++;
    pulsosPPS++;
}

// *****************************************************************************
void ConfiguracionPrincipal() {
    unsigned long guardiaPLL;

    // Mismo reloj que el Concentrador: FRC/7 * 152 / 2 = ~80 MHz
    CLKDIVbits.FRCDIV = 0;
    CLKDIVbits.PLLPOST = 0;
    CLKDIVbits.PLLPRE = 5;
    PLLFBDbits.PLLDIV = 150;
    OSCTUN = (OSC_TUN & 0x3F);            // Ajuste fino del FRC
    guardiaPLL = 400000;
    while ((OSCCONbits.LOCK != 1) && (guardiaPLL > 0)) guardiaPLL--;
    if (OSCCONbits.LOCK != 1) {
        // El PLL no engancha: casi siempre es Oscillator Source distinto de FRCPLL en Edit Project.
        TRISAbits.TRISA2 = 0;
        while (1) { LATAbits.LATA2 = ~LATAbits.LATA2; Delay_ms(50); }
    }
    Delay_ms(100);                        // Estabilizacion antes de configurar el UART
    LATAbits.LATA2 = 0;                   // Marca B: el PLL engancho, LED apagado

    ANSELA = 0;
    ANSELB = 0;

    ConfigurarPPS_SPI1();
    ConfigurarPPS_UART1();
    ConfigurarPPS_INT1();                 // INT1 <- RB14, en la misma ventana de PPS que SPI1 y UART1

    TEST_Direction = 0;
    TEST = 0;

    CsADXL_Direction = 0;
    CsADXL = 1;
    sd_CS_tris = 0;
    sd_CS_lat = 1;
    sd_detect_tris = 1;
    CNPUAbits.CNPUA4 = 1;

    // MAX485 siempre en recepcion
    MSRS485_Direction = 0;
    MSRS485 = 0;

    // UART1: la interrupcion se habilita en main() despues de iniciar la SD
    UART1_Init_Advanced(BAUD_RS485, _UART_8BIT_NOPARITY, _UART_ONE_STOPBIT, _UART_HI_SPEED);
    U1STAbits.URXISEL = 0x00;             // interrupcion con cada byte recibido
    IPC2 = (IPC2 & 0x8FFF) | ((unsigned int)U1RX_PRIORIDAD << 12);   // U1RXIP<2:0> = bits 14:12
    U1RXIF_bit = 0;
    U1RXIE_bit = 0;

    ConfigurarTimeoutRx();

    TRISBbits.TRISB14 = 1;                // RB14 (RP46) entrada: pulso de sincronismo
    // Polaridad automatica: se lee el nivel de reposo de RB14 y se dispara en el flanco que SALE
    // del reposo (inicio del pulso). Reposo alto -> flanco de bajada; reposo bajo -> flanco de subida.
    Delay_ms(5);
#if PPS_FLANCO == 1
    INTCON2.INT1EP = 1; polaridadPPS = 1;                                // forzado: flanco de bajada
#elif PPS_FLANCO == 2
    INTCON2.INT1EP = 0; polaridadPPS = 0;                                // forzado: flanco de subida
#else
    if (PORTBbits.RB14 == 1) { INTCON2.INT1EP = 1; polaridadPPS = 1; }   // flanco de bajada
    else                     { INTCON2.INT1EP = 0; polaridadPPS = 0; }   // flanco de subida
#endif
    IPC5bits.INT1IP = 0x05;
    INT1IF_bit = 0;
    INT1IE_bit = 0;                       // se habilita en main() tras iniciar la SD

    INTCON2.GIE = 1;
}

// Timer2 libre (sin interrupcion): periodo SILENCIO_US (Fcy = 40 MHz, prescaler 1:8 -> 5 MHz)
void ConfigurarTimeoutRx() {
    T2CON = 0x0010;                       // TCKPS<1:0> = 01 (1:8), apagado
    TMR2 = 0;
    PR2 = (unsigned int)(SILENCIO_US * 5UL);
    T2IF_bit = 0;
    T2IE_bit = 0;
    T2CON.TON = 1;
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
    RPOR2 = (RPOR2 & 0x00FF) | 0x0600;    // RP39 (RB7)  -> SCK1
    RPOR3 = (RPOR3 & 0xFF00) | 0x0005;    // RP40 (RB8)  -> SDO1
    RPINR20 = (RPINR20 & 0xFF00) | 41;    // RP41 (RB9)  <- SDI1
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

// Mapea INT1 al pin RB14 (RP46): salida RO del MAX485 U5 (red INT_SINC).
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
    RPINR0 = (RPINR0 & 0x80FF) | ((unsigned int)PPS_PIN_RP << 8);   // INT1 <- RP46 (por direccion)
}