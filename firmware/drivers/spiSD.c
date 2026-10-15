// *****************************************************************************
// Controlador SPI1 para la tarjeta SD
// Fcy = ~40 MHz (reloj Fosc de ~80 MHz).
// Fsck = Fcy / (prescaler primario x prescaler secundario)
// *****************************************************************************

#include "spiSD.h"

void SPISD_Init(unsigned char speed) {
    SPI1STAT.SPIEN = 0;              // Deshabilita SPI1 durante la configuracion.

    if (speed == SLOW) {
        // Fase Init: Frecuencia baja (~156 kHz asumiendo Fcy de 40 MHz)
        SPI1_Init_Advanced(_SPI_MASTER, _SPI_8_BIT,
                           _SPI_PRESCALE_SEC_4, _SPI_PRESCALE_PRI_64,
                           _SPI_SS_DISABLE, _SPI_DATA_SAMPLE_MIDDLE,
                           _SPI_CLK_IDLE_HIGH, _SPI_ACTIVE_2_IDLE);
    } else {
        // Fase Datos (FAST): Alta velocidad (~10 MHz asumiendo Fcy de 40 MHz)
        SPI1_Init_Advanced(_SPI_MASTER, _SPI_8_BIT,
                           _SPI_PRESCALE_SEC_1, _SPI_PRESCALE_PRI_4,
                           _SPI_SS_DISABLE, _SPI_DATA_SAMPLE_MIDDLE,
                           _SPI_CLK_IDLE_HIGH, _SPI_ACTIVE_2_IDLE);
    }

    SPI1STAT.SPIEN = 1;              // Habilita SPI1.
}

// Transmite un byte y devuelve el byte recibido simultaneamente.
unsigned char SPISD_Write(unsigned char datos) {
    SPI1BUF = datos;
    while (SPI1STATbits.SPITBF);
    while (SPI1STATbits.SPIRBF == 0);
    return SPI1BUF;
}