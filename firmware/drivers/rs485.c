/*-------------------------------------------------------------------------------------------------------------------------
RS485.c V1.7
Autor original: Geovanny Cullquicondo

Modulo compartido para el bus RS485 principal de datos.
Gestiona la transmision de tramas y el control de direccion del MAX485.

Agregar este archivo al proyecto como Source File. No usar #include "rs485.c".

Trama de tiempo (8 bytes):
  [0] 0x3A  cabecera
  [1] 0xFF  direccion (broadcast)
  [2] 0x54  funcion 'T'
  [3] timestamp byte 3 (MSB)
  [4] timestamp byte 2
  [5] timestamp byte 1
  [6] timestamp byte 0 (LSB)
  [7] checksum = (suma de los bytes [0] a [6]) & 0xFF
---------------------------------------------------------------------------------------------------------------------------*/

//////////////////////////////////////////////////////////////  CONSTANTES  //////////////////////////////////////////////////////////////

#define RS485_BYTE_INICIO     0x3A
#define RS485_PUERTO_UART1    1
#define RS485_PUERTO_UART2    2
#define RS485_DIR_BROADCAST   255
#define RS485_FUNCION_TIEMPO  0x54  // 'T'

// 1 = el MAX485 se queda en transmision (DE/RE en alto) despues de cada trama, de modo que el bus
//     reposa manejado en nivel alto y no flota. Valido mientras el Concentrador sea el unico que transmite.
// 0 = DE baja tras cada trama (bus libre para respuestas de otros nodos; requiere resistencias de polarizacion).
#define RS485_MANTENER_DE_ALTO  1

// Pin de control del MAX485 (definido en el archivo principal)
extern sfr sbit MSRS485;

// Trama fija de prueba
unsigned char tramaPruebaRS485[10] = {10, 11, 12, 13, 14, 15, 16, 17, 18, 19};

//*****************************************************************************************************************************************
// Espera a que el UART termine de enviar el ultimo bit (FIFO y registro de desplazamiento vacios)
static void RS485_EsperarFinTx(unsigned short puerto){
     if (puerto==RS485_PUERTO_UART1){
        while(!U1STAbits.TRMT);
     } else {
        while(!U2STAbits.TRMT);
     }
}
//*****************************************************************************************************************************************

//*****************************************************************************************************************************************
// Escribe un byte por el UART indicado (1 o 2)
static void RS485_EscribirByte(unsigned short puerto, unsigned char dato){
     if (puerto==RS485_PUERTO_UART1){
        UART1_Write(dato);
     } else {
        UART2_Write(dato);
     }
}
//*****************************************************************************************************************************************

//*****************************************************************************************************************************************
// EnviarTramaRS485 (trama clasica con NumDatos)
void EnviarTramaRS485(unsigned short puerto, unsigned short direccion, unsigned short funcion, unsigned int numDatos, unsigned char *payload){
     unsigned int k;
     unsigned char *ptrNumDatos;
     ptrNumDatos = (unsigned char *) &numDatos;

     MSRS485 = 1;
     Delay_us(10);

     RS485_EscribirByte(puerto, RS485_BYTE_INICIO);
     RS485_EscribirByte(puerto, direccion);
     RS485_EscribirByte(puerto, funcion);
     RS485_EscribirByte(puerto, *(ptrNumDatos));
     RS485_EscribirByte(puerto, *(ptrNumDatos+1));

     for (k=0; k<numDatos; k++){
         RS485_EscribirByte(puerto, payload[k]);
     }

     RS485_EsperarFinTx(puerto);
     Delay_us(10);
#if RS485_MANTENER_DE_ALTO == 0
     MSRS485 = 0;
#endif
}
//*****************************************************************************************************************************************

//*****************************************************************************************************************************************
// EnviarTramaTiempoRS485: trama de tiempo de la Fase 3 (8 bytes, con checksum)
void EnviarTramaTiempoRS485(unsigned short puerto, unsigned long timestamp){
    unsigned char checksum;
    unsigned char b1, b2, b3, b4;

    // Descomponer el uint32 en 4 bytes (Big-Endian)
    b1 = (unsigned char)((timestamp >> 24) & 0xFF);
    b2 = (unsigned char)((timestamp >> 16) & 0xFF);
    b3 = (unsigned char)((timestamp >> 8) & 0xFF);
    b4 = (unsigned char)(timestamp & 0xFF);

    // Checksum: suma de bytes [0]..[6] truncada a 8 bits
    checksum = (unsigned char)(RS485_BYTE_INICIO + RS485_DIR_BROADCAST + RS485_FUNCION_TIEMPO + b1 + b2 + b3 + b4);

    MSRS485 = 1;                                      // Habilita transmision del MAX485
    Delay_us(10);

    RS485_EscribirByte(puerto, RS485_BYTE_INICIO);    // [0] Cabecera
    RS485_EscribirByte(puerto, RS485_DIR_BROADCAST);  // [1] Direccion broadcast
    RS485_EscribirByte(puerto, RS485_FUNCION_TIEMPO); // [2] Funcion 'T'
    RS485_EscribirByte(puerto, b1);                   // [3] Timestamp MSB
    RS485_EscribirByte(puerto, b2);                   // [4]
    RS485_EscribirByte(puerto, b3);                   // [5]
    RS485_EscribirByte(puerto, b4);                   // [6] Timestamp LSB
    RS485_EscribirByte(puerto, checksum);             // [7] Checksum

    RS485_EsperarFinTx(puerto);
    Delay_us(10);
#if RS485_MANTENER_DE_ALTO == 0
    MSRS485 = 0;                                      // MAX485 vuelve a recepcion
#endif
}
//*****************************************************************************************************************************************

//*****************************************************************************************************************************************
void EnviarTramaPruebaRS485(unsigned short direccion){
     EnviarTramaRS485(RS485_PUERTO_UART2, direccion, 0xF3, 10, tramaPruebaRS485);
}
//*****************************************************************************************************************************************