// *****************************************************************************
// Fase 2 - Validacion de inicializacion, escritura y lectura cruda en microSD
// dsPIC33EP256MC202 - SPI1 (SCK=RB7, SDO=RB8, SDI=RB9) - CS=RB0 - Detect=RA4
//
// Secuencia:
//   1. Espera tarjeta (card-detect RA4, activo en bajo, con antirrebote)
//   2. SD_Init_Try (SLOW -> FAST)
//   3. Escribe patron 00..FF x2 en sector 2500
//   4. Lee el sector 2500 a RAM
//   5. Compara byte a byte
//   6. Copia lo leido al sector 2501
//   7. LED fijo = todo correcto
//
// Codigos de error: grupo 1 = codigo (parpadeos de 300 ms), pausa 1.5 s,
// grupo 2 = detalle (parpadeos MUY rapidos de 100 ms), pausa 4 s, se repite.
//   Grupo 1: 1 = fallo SD_Init            2 = fallo escritura sector 2500
//            3 = fallo lectura 2500       4 = datos leidos distintos
//            5 = fallo escritura sector 2501
//   Grupo 2 (detalle del estado devuelto por la libreria):
//            0 = sin detalle   1 = tarjeta ocupada/timeout (SD_NOT_READY)
//            2 = token de datos no recibido   3 = sin respuesta R1 (0xFF)
//            4 = CRC rechazado  5 = error de escritura  6 = token inesperado
//            7 = R1 con bits de error u otro
//
// NOTA: escribe sectores crudos (2500 y 2501). Usar SD sin datos importantes.
// *****************************************************************************

#include <spiSD.h>
#include <sdcard.h>
#include <stdbool.h>
#include <stdlib.h>

// RP41 = RB9: entrada SDI1 de SPI1.
#define RPINR20 (*((volatile unsigned int*)0x06C8))

// Parametros de la aplicacion.
#define IDNODO 1
#define SIZESD 16                    // Capacidad de la SD en GB.
#define DELTASECTOR 97952

// Configuracion.
#define SD_DETECCION_HARDWARE 1      // 1 = usa el contacto del zocalo (RA4).
#define FP 80000000

// Prueba unitaria.
#define SECTOR_PRUEBA 2500UL
#define INTENTOS_INIT 10

// Codigos de error por parpadeos.
#define ERR_INIT        1
#define ERR_ESCRITURA   2
#define ERR_LECTURA     3
#define ERR_COMPARACION 4
#define ERR_COPIA       5

// Indices de trabajo.
unsigned int i, j, x, y;

// Pines y estado de la tarjeta.
struct sdflags sdflags;
sbit TEST at LATA2_bit;                    // LED de prueba.
sbit TEST_Direction at TRISA2_bit;
sbit CsADXL at LATA3_bit;                  // Chip select del acelerometro.
sbit CsADXL_Direction at TRISA3_bit;
sbit sd_CS_lat at LATB0_bit;               // Chip select de la SD.
sbit sd_CS_tris at TRISB0_bit;
sbit sd_detect_port at PORTA.B4;           // Card-detect: se lee PORT, no LAT.
sbit sd_detect_tris at TRISA4_bit;
sbit MSRS485 at LATB12_bit;                // Modo del transceptor RS485.
sbit MSRS485_Direction at TRISB12_bit;

// Estado general.
unsigned short inicioSistema;
unsigned long horaSistema, fechaSistema;

// Sectores de la SD.
unsigned long PSF;
unsigned long PSE;
unsigned long USF;
unsigned long PSEC;
unsigned long sectorSD;
unsigned long sectorLec;
const unsigned int clusterSizeSD = 512;
unsigned long infoPrimerSector;
unsigned long infoUltimoSector;
unsigned char cabeceraSD[6] = {255, 253, 251, 10, 0, 250};
unsigned char bufferSD[clusterSizeSD];
unsigned char checkEscSD;
unsigned char checkLecSD;
unsigned short banInsSec;

// Prototipos.
void ConfiguracionPrincipal();
void ConfigurarPPS_SPI1();
void EsperarTarjeta();
void Ejemplo_uso_SD();
void GuardarTramaSD();
void GuardarBufferSD(unsigned char* bufferLleno, unsigned long sector);
void GuardarInfoSector(unsigned long datoSector, unsigned long localizacionSector);
void LED(int veces, unsigned char tiempo_seg);
void LED_Error(unsigned char codigo, unsigned char detalle);
unsigned char Detalle(unsigned char estado);

// *****************************************************************************
void main() {
    ConfiguracionPrincipal();

    // Indicador de arranque: 1 s encendido.
    TEST = 1;
    Delay_ms(1000);
    TEST = 0;

    // Inicializa variables.
    i = 0; j = 0; x = 0; y = 0;
    inicioSistema = 0;
    horaSistema = 0;
    fechaSistema = 0;
    PSEC = 0;
    sectorSD = 0;
    sectorLec = 0;
    checkEscSD = 0;
    checkLecSD = 0;
    banInsSec = 0;
    MSRS485 = 0;                           // RS485 en modo lectura.

    // Ultimo sector fisico segun capacidad.
    PSF = 2048;
    switch (SIZESD) {
        case 2:  USF = 3911679;  break;
        case 4:  USF = 7772160;  break;
        case 8:  USF = 16779263; break;
        case 16: USF = 31115263; break;
        default: USF = 31115263; break;
    }
    infoPrimerSector = PSF + DELTASECTOR - 2;
    infoUltimoSector = PSF + DELTASECTOR - 1;
    PSE = PSF + DELTASECTOR;

    horaSistema = 86100;                   // 23:55:00 (prueba).
    fechaSistema = 200228;                 // 20/02/28 (prueba).

    // 1. Espera la tarjeta (o la da por presente si no hay card-detect).
    EsperarTarjeta();

    // 2. Inicializa la SD (SLOW durante comandos, FAST al terminar).
    if (sdflags.detected && !sdflags.init_ok) {
        checkEscSD = SD_Init_Try(INTENTOS_INIT);
        if (checkEscSD == SUCCESSFUL_INIT) {
            sdflags.init_ok = true;
            inicioSistema = 1;
        } else {
            sdflags.init_ok = false;
            inicioSistema = 0;
            LED_Error(ERR_INIT, Detalle(checkEscSD));   // No retorna.
        }
    }

    // 3-6. Prueba unitaria de escritura / lectura / comparacion / copia.
    Ejemplo_uso_SD();

    // Todo correcto: LED fijo.
    TEST = 1;
    while (1) {
        asm CLRWDT;
    }
}

// *****************************************************************************
// Espera a que la tarjeta este insertada y estable.
// *****************************************************************************
void EsperarTarjeta() {
#if SD_DETECCION_HARDWARE
    while (1) {
        if (SD_Detect() == DETECTED) {
            Delay_ms(100);                 // Antirrebote.
            if (SD_Detect() == DETECTED) {
                Delay_ms(200);             // Deja asentar la tarjeta.
                sdflags.detected = true;
                break;
            }
        } else {
            sdflags.detected = false;
            sdflags.init_ok = false;
        }
        Delay_ms(100);
    }
#else
    sdflags.detected = true;
#endif
}

// *****************************************************************************
// Prueba unitaria: escribe, lee, compara byte a byte y copia al sector +1.
// *****************************************************************************
void Ejemplo_uso_SD() {
    unsigned char data_to_write[512];
    unsigned char buffer[512];
    unsigned int k;
    unsigned long sector;

    // Patron 00 01 02 ... FF repetido dos veces.
    for (k = 0; k < 512; k++) {
        data_to_write[k] = (unsigned char)(k & 0xFF);
        buffer[k] = 0;
    }

    sector = SECTOR_PRUEBA;

    // Escritura.
    checkEscSD = SD_Write_Block(data_to_write, sector);
    if (checkEscSD != DATA_ACCEPTED) {
        LED_Error(ERR_ESCRITURA, Detalle(checkEscSD));
    }
    LED(1, 1);

    // Lectura a RAM.
    checkLecSD = SD_Read_Block(buffer, sector);
    if (checkLecSD != 0) {
        LED_Error(ERR_LECTURA, Detalle(checkLecSD));
    }
    LED(1, 1);

    // Comparacion byte a byte.
    for (k = 0; k < 512; k++) {
        if (buffer[k] != data_to_write[k]) {
            LED_Error(ERR_COMPARACION, 0);
        }
    }
    LED(1, 1);

    // Copia lo leido al sector siguiente.
    checkEscSD = SD_Write_Block(buffer, sector + 1);
    if (checkEscSD != DATA_ACCEPTED) {
        LED_Error(ERR_COPIA, Detalle(checkEscSD));
    }
    LED(1, 1);
}

// *****************************************************************************
// Parpadeos de 1 o 2 segundos (tiempo_seg = 1 o 2).
// *****************************************************************************
void LED(int veces, unsigned char tiempo_seg) {
    unsigned int n;
    TEST_Direction = 0;
    if (tiempo_seg == 1) {
        for (n = 0; n < veces; n++) {
            TEST = 1; Delay_ms(1000);
            TEST = 0; Delay_ms(1000);
        }
    } else if (tiempo_seg == 2) {
        for (n = 0; n < veces; n++) {
            TEST = 1; Delay_ms(2000);
            TEST = 0; Delay_ms(2000);
        }
    }
}

// *****************************************************************************
// Error en dos grupos de parpadeos (ver cabecera). No retorna.
// *****************************************************************************
void LED_Error(unsigned char codigo, unsigned char detalle) {
    unsigned char n;
    while (1) {
        for (n = 0; n < codigo; n++) {
            TEST = 1; Delay_ms(300);
            TEST = 0; Delay_ms(300);
        }
        Delay_ms(1500);
        for (n = 0; n < detalle; n++) {
            TEST = 1; Delay_ms(100);
            TEST = 0; Delay_ms(200);
        }
        Delay_ms(4000);
    }
}

// Convierte el estado de la libreria SD en un numero pequeno de parpadeos.
unsigned char Detalle(unsigned char estado) {
    switch (estado) {
        case SD_NOT_READY:            return 1;
        case TOKEN_NOT_RECEIVED:      return 2;
        case 0xFF:                    return 3;
        case DATA_REJECTED_CRC_ERROR: return 4;
        case DATA_REJECTED_WR_ERROR:  return 5;
        case ERROR:                   return 6;
        default:                      return 7;
    }
}

// *****************************************************************************
void ConfiguracionPrincipal() {

    // Configuración para Fosc ~= 80 MHz usando FRC interno (7.37 MHz)
    // Fosc = Fin * (M / (N1 * N2)) = 7.37 MHz * (43 / (2 * 2)) = 79.22 MHz
    PLLFBD = 41;             // M = 43 (El registro requiere el valor M - 2)
    CLKDIVbits.PLLPOST = 0;  // N2 = 2 (El valor 0 significa divisor de 2)
    CLKDIVbits.PLLPRE = 0;   // N1 = 2 (El valor 0 significa divisor de 2)

    // Espera obligatoria hasta que el PLL se estabilice
    while (OSCCONbits.LOCK != 1) {};

    Delay_ms(4000);                        // Estabilizacion inicial.

    ANSELA = 0;
    ANSELB = 0;
    ConfigurarPPS_SPI1();

    TEST_Direction = 0;
    TEST = 0;

    // CS del acelerometro inactivo para que no use el bus SPI.
    CsADXL_Direction = 0;
    CsADXL = 1;

    // CS de la SD inactivo.
    sd_CS_tris = 0;
    sd_CS_lat = 1;

    // Card-detect: entrada con pull-up interno (contacto a GND con tarjeta).
    sd_detect_tris = 1;
    CNPUAbits.CNPUA4 = 1;

    MSRS485_Direction = 0;

    sdflags.detected = false;
    sdflags.init_ok = false;
    sdflags.saving = false;

    Delay_ms(200);
}

// SCK1OUT en RB7, SDO1 en RB8 y SDI1 en RB9.
void ConfigurarPPS_SPI1() {
    asm {
        MOV #0x46, W0
        MOV #0x57, W1
        MOV #0x0742, W2
        MOV.b W0, [W2]
        MOV.b W1, [W2]
        BCLR OSCCON, #6
    }

    RPOR2 = (RPOR2 & 0x00FF) | 0x0600;
    RPOR3 = (RPOR3 & 0xFF00) | 0x0005;
    RPINR20 = (RPINR20 & 0xFF00) | 41;

    TRISBbits.TRISB7 = 0;
    TRISBbits.TRISB8 = 0;
    TRISBbits.TRISB9 = 1;
}

// *****************************************************************************
// Rutinas para fases posteriores (no se llaman en esta prueba).
// *****************************************************************************

// Escribe un buffer de 512 bytes con hasta cinco intentos.
void GuardarBufferSD(unsigned char* bufferLleno, unsigned long sector) {
    for (x = 0; x < 5; x++) {
        checkEscSD = SD_Write_Block(bufferLleno, sector);
        if (checkEscSD == DATA_ACCEPTED) {
            break;
        }
        Delay_us(10);
    }
}

// Guarda una trama de prueba en sectores consecutivos.
void GuardarTramaSD() {
    unsigned char buf[512];

    for (sectorSD = 40000; sectorSD < 40020; sectorSD++) {
        for (x = 0; x < 6; x++)   buf[x] = 1;
        for (x = 0; x < 6; x++)   buf[6 + x] = 2;
        for (x = 0; x < 500; x++) buf[12 + x] = 3;
        GuardarBufferSD(buf, sectorSD);
        sectorSD++;

        for (x = 0; x < 512; x++) buf[x] = 4;
        GuardarBufferSD(buf, sectorSD);
        sectorSD++;

        for (x = 0; x < 512; x++) buf[x] = 5;
        GuardarBufferSD(buf, sectorSD);
        sectorSD++;

        for (x = 0; x < 512; x++) buf[x] = 6;
        GuardarBufferSD(buf, sectorSD);
        sectorSD++;

        for (x = 0; x < 512; x++) buf[x] = (x < 464) ? 7 : 0;
        // Escritura del quinto sector deshabilitada durante la prueba.

        LED(3, 1);
    }
}

// Guarda un numero de sector (big-endian) dentro de un sector.
void GuardarInfoSector(unsigned long datoSector, unsigned long localizacionSector) {
    unsigned char bufferSectores[512];

    bufferSectores[0] = (datoSector >> 24) & 0xFF;
    bufferSectores[1] = (datoSector >> 16) & 0xFF;
    bufferSectores[2] = (datoSector >> 8) & 0xFF;
    bufferSectores[3] = (datoSector) & 0xFF;
    for (x = 4; x < 512; x++) {
        bufferSectores[x] = 0;
    }

    for (x = 0; x < 5; x++) {
        checkEscSD = SD_Write_Block(bufferSectores, localizacionSector);
        if (checkEscSD == DATA_ACCEPTED) {
            break;
        }
        Delay_us(10);
    }
}