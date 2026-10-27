/*-------------------------------------------------------------------------------------------------------------------------
Nodo Concentrador - Sistema de Monitorizacion de Salud Estructural V1.7
Autor original: Geovanny Cullquicondo

Descripcion (Fase 3):
  - Timer3 genera una base de 1 s (10 interrupciones de 100 ms).
  - En cada segundo, la ISR SOLO: incrementa el timestamp, sube el pulso INT_SINC_1
    y levanta una bandera. No transmite ni usa delays dentro de la ISR.
  - El bucle principal envia la trama de tiempo por RS485 (UART2 + MAX485) y baja el pulso.

Trama de tiempo (8 bytes):
  [0x3A][0xFF broadcast][0x54 'T'][TS byte3 MSB][TS byte2][TS byte1][TS byte0 LSB][Checksum]
  Checksum = suma de los 7 bytes anteriores, truncada a 8 bits.

Hardware:
  dsPIC33EP256MC202, Fosc = 80 MHz (FRC+PLL), Fcy = 40 MHz
  RB15 = U2RX (RP47), RB4 = U2TX (RP36), RB11 = DE/RE del MAX485
  D5 (RA1)  -> Heartbeat, cambia cada segundo.
  D2 (RA3)  -> Cambia cada vez que se envia una trama RS485.
  INT_SINC_1 (RA0) -> Pulso fisico de sincronismo (1 pps).

Proyecto MikroC:
  - Edit Project: Oscillator Frequency = 80.000000 MHz (afecta Delay_ms / Delay_us).
  - Agregar rs485.c al proyecto (Source Files). NO usar #include "rs485.c".
---------------------------------------------------------------------------------------------------------------------------*/

// 0 = funcionamiento normal (1 trama por segundo)
// 1 = depuracion con osciloscopio: una trama cada ~1 ms
// 2 = depuracion con multimetro: DE fijo en alto y 0x00 continuo
// 3 = prueba de hardware sin UART: RB4 alterna cada 500 ms con DE fijo en alto

#define MODO_PRUEBA_CONTINUA   0

// Ajuste fino del oscilador interno (FRC): -32 a +31, cada paso = 0.375 %.
// Positivo = mas rapido. Se calcula midiendo el periodo del UART (ver instrucciones).
#define OSC_TUN                0

// Baud del bus RS485. Debe ser IGUAL en el Concentrador y en los nodos.
// 2 Mbps y 1 Mbps dieron bytes con bits perdidos; se prueba 500 kbps.
#define BAUD_RS485             500000

////////////////////////////////////////////////////         Pines         /////////////////////////////////////////////////

sbit RP1 at LATA4_bit;
sbit RP1_Direction at TRISA4_bit;
sbit MSRS485 at LATB11_bit;
sbit MSRS485_Direction at TRISB11_bit;

sbit INT_SINC_1 at LATA0_bit;
sbit INT_SINC_1_Direction at TRISA0_bit;

sbit INT_SINC at LATA1_bit;
sbit INT_SINC_Direction at TRISA1_bit;

sbit INT_SINC_2 at LATA3_bit;
sbit INT_SINC_2_Direction at TRISA3_bit;

//////////////////////////////////////////////  Constantes  /////////////////////////////////////////////////////////////////

#define PERIODO_SYNC_MS       1000
#define ANCHO_PULSO_SYNC_US   1000     // Tiempo en alto del pulso, contado despues de enviar la trama
#define TICK_TIMER3_MS        100

//////////////////////////////////////////////  Variables globales  /////////////////////////////////////////////////////////

// Reloj maestro de la red (segundos desde el arranque). Se modifica en ISR.
volatile unsigned long timestamp_global;
volatile unsigned short banSyncPendiente;

// SPI / RPi (logica original, sin cambios)
unsigned short banLec, banEsc;
unsigned char *ptrnumBytesSPI;
unsigned char tramaSolicitudSPI[10];
unsigned char tramaSolicitudNodo[10];
unsigned short banInicio;
unsigned short banRespuestaPi;
unsigned short banSPI0, banSPI1, banSPI2, banSPI7, banSPI8, banSPIA;
unsigned short bufferSPI;

// RS485 (recepcion original, provisional)
unsigned short banRSI, banRSC;
volatile unsigned char byteRS485;
unsigned int i_rs485;
unsigned char tramaCabeceraRS485[10];
unsigned char inputPyloadRS485[2600];
unsigned char outputPyloadRS485[15];
unsigned short direccionRS485;
unsigned short funcionRS485;
unsigned short subFuncionRS485;
unsigned int numDatosRS485;
unsigned char *ptrnumDatosRS485;

unsigned short banInicioMuestreo;

/////////////////////////////////////////////////////////  Declaracion de funciones  ///////////////////////////////////////////
void ConfiguracionPrincipal();
void InterrupcionP1(unsigned short funcionSPI, unsigned short subFuncionSPI, unsigned int numBytesSPI);
void CambiarEstadoBandera(unsigned short bandera, unsigned short estado);

// Funciones definidas en rs485.c (archivo separado del proyecto)
extern void EnviarTramaRS485(unsigned short puerto, unsigned short direccion, unsigned short funcion, unsigned int numDatos, unsigned char *payload);
extern void EnviarTramaTiempoRS485(unsigned short puerto, unsigned long timestamp);

//////////////////////////////////////////////////////////////      Main      ////////////////////////////////////////////////

void main() {
     unsigned long tsEnvio;

     ConfiguracionPrincipal();

     timestamp_global = 0;
     banSyncPendiente = 0;

     banSPI0 = 0; banSPI1 = 0; banSPI2 = 0; banSPI7 = 0; banSPI8 = 0; banSPIA = 0;
     banRespuestaPi = 0;

     banRSI = 0; banRSC = 0; byteRS485 = 0; i_rs485 = 0;
     funcionRS485 = 0; subFuncionRS485 = 0; numDatosRS485 = 0;
     ptrnumDatosRS485 = (unsigned char *) &numDatosRS485;

     banInicioMuestreo = 0;

     RP1 = 0;
     INT_SINC = 1;      // D5 encendido
     INT_SINC_1 = 0;    // Pulso fisico apagado
     INT_SINC_2 = 1;    // D2 encendido

     MSRS485 = 1;       // MAX485 en transmision permanente: el bus reposa manejado en nivel alto
                        // (el Concentrador es el unico transmisor en esta fase; ver RS485_MANTENER_DE_ALTO en rs485.c)

     SPI1BUF = 0x00;

     // Todo configurado: ahora si se habilitan las interrupciones globales y se arranca la base de tiempo.
     INTCON2.GIE = 1;

#if MODO_PRUEBA_CONTINUA == 3
     // ---- Modo 3: SIN UART. RB4 como pin normal que alterna cada 500 ms, DE fijo en alto ----
     // Prueba pura de hardware: el pin 4 del MAX485 debe ir 3.3 V / 0 V cada 0.5 s.
     RPOR1bits.RP36R = 0;       // Desconecta U2TX del pin
     TRISBbits.TRISB4 = 0;
     MSRS485 = 1;
     while(1){
          asm CLRWDT;
          LATBbits.LATB4 = 1;
          Delay_ms(500);
          LATBbits.LATB4 = 0;
          Delay_ms(500);
     }
#elif MODO_PRUEBA_CONTINUA == 2
     // ---- Modo 2: DE fijo en alto y bytes 0x00 sin parar. Se mide con multimetro ----
     // Esperado: pin 3 del MAX485 = 3.3 V fijo; pin 4 (DI) = ~0.3 V (casi siempre bajo).
     MSRS485 = 1;
     while(1){
          asm CLRWDT;
          UART2_Write(0x00);
     }
#elif MODO_PRUEBA_CONTINUA == 1
     // ---- Modo 1: una trama de tiempo cada ~1 ms, para ver DE / DI / A-B en el osciloscopio ----
     while(1){
          asm CLRWDT;
          tsEnvio = timestamp_global;
          timestamp_global++;
          EnviarTramaTiempoRS485(2, tsEnvio);
          INT_SINC_2 = ~INT_SINC_2;
          Delay_ms(1);
     }
#else
     T3CON.TON = 1;     // Inicia temporizador de sincronismo

     while(1){
          asm CLRWDT;

          if (banSyncPendiente){
               banSyncPendiente = 0;

               // Lectura atomica del timestamp de 32 bits (el dsPIC es de 16 bits)
               T3IE_bit = 0;
               tsEnvio = timestamp_global;
               T3IE_bit = 1;

               // La ISR ya subio INT_SINC_1; la trama sale inmediatamente despues del flanco.
               EnviarTramaTiempoRS485(2, tsEnvio);
               INT_SINC_2 = ~INT_SINC_2;           // D2: indica trama enviada

               Delay_us(ANCHO_PULSO_SYNC_US);
               INT_SINC_1 = 0;                     // Fin del pulso fisico
          }
     }
#endif
}

//////////////////////////////////////////////////////////////  FUNCIONES  ////////////////////////////////////////////////////

void ConfiguracionPrincipal(){
     // Reloj: FRC 7.37 MHz / (PLLPRE+2=7) * (PLLDIV+2=152) / (PLLPOST: /2) = ~80 MHz
     CLKDIVbits.FRCDIV = 0;
     CLKDIVbits.PLLPOST = 0;
     CLKDIVbits.PLLPRE = 5;
     PLLFBDbits.PLLDIV = 150;
     OSCTUN = (OSC_TUN & 0x3F);          // Ajuste fino del FRC
     while (OSCCONbits.LOCK != 1) {};    // Espera a que el PLL enganche
     Delay_ms(100);                      // Estabilizacion antes de configurar UART a 2 Mbps

     ANSELA = 0;
     ANSELB = 0;

     INT_SINC_Direction   = 0;
     INT_SINC_1_Direction = 0;
     INT_SINC_2_Direction = 0;
     RP1_Direction = 0;
     MSRS485_Direction = 0;
     TRISBbits.TRISB4 = 0;      // TX (RB4) como salida
     TRISBbits.TRISB15 = 1;     // RX (RB15) como entrada

     // RS485 a 2 Mbps (UART2): RX = RB15 (RP47), TX = RB4 (RP36)
     RPINR19bits.U2RXR = 0x2F;
     RPOR1bits.RP36R = 0x03;
     UART2_Init_Advanced(BAUD_RS485, _UART_8BIT_NOPARITY, _UART_ONE_STOPBIT, _UART_HI_SPEED);
     U2STAbits.URXISEL = 0x00;
     IPC7bits.U2RXIP = 0x04;
     U2RXIF_bit = 0;
     U2RXIE_bit = 1;

     // SPI1 esclavo hacia la Raspberry (logica original)
     SPI1STAT.SPIEN = 1;
     SPI1_Init_Advanced(_SPI_SLAVE, _SPI_8_BIT, _SPI_PRESCALE_SEC_1, _SPI_PRESCALE_PRI_1, _SPI_SS_ENABLE, _SPI_DATA_SAMPLE_END, _SPI_CLK_IDLE_HIGH, _SPI_ACTIVE_2_IDLE);
     SPI1IF_bit = 0;
     IPC2bits.SPI1IP = 0x03;
     SPI1IE_bit = 1;

     // Timer2: reservado para timeout de recepcion RS485 (apagado en esta fase)
     T2CON = 0x30;
     T2CON.TON = 0;
     T2IE_bit = 1;
     T2IF_bit = 0;
     PR2 = 46875;
     IPC1bits.T2IP = 0x02;

     // Timer3: base de tiempo. Fcy/256 = 156250 Hz; PR3 = 15625 -> 100 ms por interrupcion
     T3CON = 0x30;
     T3CON.TON = 0;
     TMR3 = 0;
     T3IF_bit = 0;
     PR3 = 15625;
     IPC2bits.T3IP = 0x02;
     T3IE_bit = 1;

     Delay_ms(100);
}

void InterrupcionP1(unsigned short funcionSPI, unsigned short subFuncionSPI, unsigned int numBytesSPI){
     if (banRespuestaPi==1){
         ptrnumBytesSPI = (unsigned char *) &numBytesSPI;
         tramaSolicitudSPI[0] = funcionSPI;
         tramaSolicitudSPI[1] = subFuncionSPI;
         tramaSolicitudSPI[2] = *(ptrnumBytesSPI);
         tramaSolicitudSPI[3] = *(ptrnumBytesSPI+1);
         RP1 = 1;
         Delay_us(20);
         RP1 = 0;
         banRespuestaPi = 0;
     }
}

void CambiarEstadoBandera(unsigned short bandera, unsigned short estado){
     if (estado==1){
         banSPI0 = 3; banSPI1 = 3; banSPI2 = 3; banSPI7 = 3; banSPI8 = 3; banSPIA = 3;
         switch (bandera){
                case 0: banSPI0 = 1; break;
                case 1: banSPI1 = 1; break;
                case 2: banSPI2 = 1; break;
                case 7: banSPI7 = 1; break;
                case 8: banSPI8 = 1; break;
                case 0x0A: banSPIA = 1; break;
         }
     }
     if (estado==0){
         banSPI0 = 0; banSPI1 = 0; banSPI2 = 0; banSPI7 = 0; banSPI8 = 0; banSPIA = 0;
     }
}

//////////////////////////////////////////////////////////////  INTERRUPCIONES  /////////////////////////////////////////////////

void spi_1() org  IVT_ADDR_SPI1INTERRUPT {
     // Logica de la Raspberry sin cambios (provisional en Fase 3)
     SPI1IF_bit = 0;
     bufferSPI = SPI1BUF;
}

//*****************************************************************************************************************************
// Base de tiempo de 1 s. SIN delays y SIN transmision: solo marca el instante y avisa al main.
//*****************************************************************************************************************************
void Timer3Int() org IVT_ADDR_T3INTERRUPT{
     static unsigned int contadorSync = 0;

     T3IF_bit = 0;
     contadorSync++;

     if (contadorSync >= (PERIODO_SYNC_MS / TICK_TIMER3_MS)){
        contadorSync = 0;

        timestamp_global++;           // Tiempo absoluto de la red
        INT_SINC = ~INT_SINC;         // D5: heartbeat
        INT_SINC_1 = 1;               // Flanco de subida del pulso de sincronismo (instante exacto)
        banSyncPendiente = 1;         // El main envia la trama y baja el pulso
     }
}
//*****************************************************************************************************************************

void Timer2Int() org IVT_ADDR_T2INTERRUPT{
     T2IF_bit = 0;
     T2CON.TON = 0;
     TMR2 = 0;
     banRSI = 0;
     banRSC = 0;
     i_rs485 = 0;
}

void urx_2() org  IVT_ADDR_U2RXINTERRUPT {
     U2RXIF_bit = 0;                  // Limpiar al inicio: no se pierden bytes que lleguen durante la ISR
     if (U2STAbits.OERR == 1) {
         U2STAbits.OERR = 0;
     }
     while (U2STAbits.URXDA) {        // Vaciar la FIFO
         byteRS485 = U2RXREG;
     }
     // Logica de recepcion original para la Raspberry: provisional, no se usa en Fase 3
}