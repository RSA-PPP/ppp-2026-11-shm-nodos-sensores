# Main_SD

Firmware para un dsPIC33EP256MC202 que comunica una tarjeta SD mediante SPI y permite leer y escribir sectores físicos de 512 bytes. El proyecto está hecho para **mikroC PRO for dsPIC** y contiene una prueba de comunicación, además de rutinas preparadas para guardar tramas de aceleración.

## Qué hace el proyecto

Al arrancar, el firmware:

1. Espera 4 segundos y configura los puertos del microcontrolador.
2. Configura el Peripheral Pin Select (PPS) para SPI1.
3. Configura la tarjeta SD como insertada, porque el contacto físico de detección no está soldado en la placa.
4. Inicializa la SD con hasta 10 intentos.
5. Ejecuta `Ejemplo_uso_SD()`, que escribe un sector, lo lee y copia los datos al sector siguiente.
6. Indica el resultado con el LED de prueba conectado en RA2.

Si la inicialización falla, deshabilita la interrupción externa INT1 y el UART, deja `inicioSistema` en cero y entra en un patrón de error mediante el LED.

## Advertencia: almacenamiento por sectores

La tarjeta no se utiliza mediante archivos FAT32. El firmware accede directamente a sectores físicos de 512 bytes usando los comandos de bajo nivel de la SD. Por ello:

- No se crean archivos ni directorios.
- El contenido debe inspeccionarse con una herramienta de acceso a sectores, por ejemplo HxD.
- No se debe montar ni utilizar la tarjeta normalmente mientras el firmware esté escribiendo sectores reservados.
- Formatear la tarjeta no garantiza borrar el contenido de los sectores usados por este firmware; sí puede sobrescribirlos si el sistema de archivos los asigna.
- Es responsabilidad del usuario evitar sectores ocupados por una partición o por archivos existentes.

## Hardware y conexiones

| Señal | Microcontrolador | Configuración |
| --- | --- | --- |
| SPI1 SCK | RB7 / RP39 | Salida de reloj |
| SPI1 MOSI / SDO1 | RB8 / RP40 | Salida de datos |
| SPI1 MISO / SDI1 | RB9 / RP41 | Entrada de datos |
| Chip Select de SD | RB0 | Salida, activo en bajo |
| Detección de SD | RA4 | Entrada, actualmente no usada |
| LED de prueba | RA2 | Salida |
| Selección de modo RS485 | RB12 | Se pone en cero, modo lectura |
| CS del acelerómetro | RA3 | Declarado, no utilizado por la prueba actual |

Todos los pines analógicos de los puertos A y B se deshabilitan (`ANSELA = 0`, `ANSELB = 0`). La frecuencia de reloj configurada para el proyecto es de 80 MHz, equivalente a 40 MIPS según los comentarios del firmware.

## Organización de archivos

### `Main_SD.c`

Contiene la configuración de la placa, el punto de entrada y las rutinas de aplicación.

- `main()` inicializa variables, calcula los sectores de la tarjeta, determina la presencia de la SD, llama a `SD_Init_Try(10)` y ejecuta la prueba.
- `ConfiguracionPrincipal()` espera, configura puertos, PPS, LED, CS de SD y banderas de estado.
- `ConfigurarPPS_SPI1()` asigna SCK a RB7, SDO1/MOSI a RB8 y SDI1/MISO a RB9 mediante registros PPS y configura sus direcciones.
- `Ejemplo_uso_SD()` llena un buffer de 512 bytes con los valores `1` a `255` repetidos, lo escribe en el sector `2500`, lo lee de vuelta y escribe la copia en el sector `2501`. En cada paso correcto produce un parpadeo de un segundo.
- `GuardarBufferSD()` intenta hasta cinco veces escribir un buffer completo en un sector, esperando 10 microsegundos entre intentos fallidos.
- `GuardarTramaSD()` es una rutina de demostración para empaquetar una trama de aceleración en cinco sectores. Actualmente no se llama desde `main()`.
- `GuardarInfoSector()` guarda un número de sector como entero de 32 bits en big-endian en los cuatro primeros bytes de un sector y rellena el resto con ceros. También reintenta la escritura hasta cinco veces.
- `LED(veces, tiempo_seg)` genera parpadeos de uno o dos segundos; solo implementa los valores de tiempo `1` y `2`.
- `LED_Error(codigo)` genera `codigo` parpadeos cortos de 500 ms, espera 3 segundos y repite indefinidamente. Para códigos mayores que 9 resta 9, por lo que los códigos 10 y 1 producen el mismo número de parpadeos.

### `sdcard.c` y `sdcard.h`

Implementan el protocolo SPI de la tarjeta SD, sus comandos, respuestas y estados.

#### Inicialización

`SD_Init_Try(n)` llama a `SD_Init()` hasta `n` veces. Si se pasa cero, realiza al menos un intento.

`SD_Init()` realiza la siguiente secuencia:

1. Configura CS como salida y lo libera en nivel alto.
2. Inicializa SPI a velocidad lenta: 625 kHz con reloj de 80 MHz.
3. Envía relojes de espera con MOSI en alto antes de comenzar.
4. Envía CMD0 hasta recibir la respuesta de estado inactivo.
5. Envía CMD8 para verificar patrón `0xAA` y rango de tensión de 2.7 a 3.6 V.
6. Para tarjetas antiguas usa CMD1; para tarjetas compatibles usa CMD55 + ACMD41.
7. Lee el OCR con CMD58 y comprueba la tensión aceptada.
8. Activa y después desactiva la verificación CRC mediante CMD59.
9. Fija el tamaño de bloque en 512 bytes con CMD16.
10. Lee nuevamente el OCR y conserva el bit CCS para distinguir direccionamiento SDSC y SDHC/SDXC.
11. Libera CS y cambia SPI a velocidad rápida: 2.5 MHz.

El resultado `SUCCESSFUL_INIT` es `0xAA`. Los demás códigos están definidos en `sdcard.h`: tarjeta ausente, SD no lista, tarjeta inutilizable, error de eco, tensión incompatible, token no recibido y errores de rechazo de escritura.

#### Lectura y escritura

- `SD_Read_Block(buffer, sector)` selecciona la SD, envía CMD17, espera el token `0xFE`, lee 512 bytes y descarta los dos bytes de CRC. Devuelve cero si la lectura fue correcta.
- `SD_Write_Block(buffer, sector)` selecciona la SD, envía CMD24, transmite el token `0xFE`, los 512 bytes y un CRC ficticio de dos bytes. Interpreta la respuesta de datos y devuelve `DATA_ACCEPTED`, `DATA_REJECTED_CRC_ERROR`, `DATA_REJECTED_WR_ERROR` u otro error.
- En tarjetas SDSC (`ccs == 0x02`) la dirección recibida se multiplica por 512; en tarjetas de direccionamiento por bloque se usa directamente.
- `SD_Read(buffer, nbytes)` espera el token de datos y recibe la cantidad solicitada; esta implementación se usa con `nbytes = 512`.
- `SD_Ready()` espera hasta `SD_TIME_OUT` ciclos a que la tarjeta devuelva `0xFF`.

#### Soporte de protocolo

- `SD_Send_Command()` transmite comando, argumento de 32 bits y CRC7 en el formato SPI de SD.
- `R1_Response()`, `R2_Response()` y `Response_32b()` reciben respuestas de 8, 16 y 32 bits.
- `Select_SD()` baja CS para activar la tarjeta.
- `Release_SD()` sube CS para liberar la tarjeta.
- `SD_Detect()` devuelve `DETECTED` (`0xDE`) si el pin RA4 está en cero y cero en caso contrario. Esta función no interviene en el arranque actual porque `SD_DETECCION_HARDWARE` vale cero.
- `Detect_SD()` y `SD_Check()` aparecen declaradas en `sdcard.h`, pero no tienen implementación en los archivos fuente de este proyecto.

### `spiSD.c` y `spiSD.h`

- `SPISD_Init(FAST)` deshabilita SPI1, lo configura como maestro de 8 bits, con reloj inactivo alto, muestreo central y CS por hardware deshabilitado; finalmente vuelve a habilitarlo.
- `SPISD_Init(SLOW)` usa el divisor correspondiente a 625 kHz para la fase de inicialización.
- `SPISD_Write(dato)` escribe un byte en `SPI1BUF`, espera a que termine la transmisión y devuelve el byte recibido simultáneamente por SPI.
- `FAST` vale `1` y `SLOW` vale `0`.

## Mapa de sectores configurado

Los valores se seleccionan con `SIZESD`. Para la configuración actual, `SIZESD = 16`:

| Capacidad | Primer sector físico (`PSF`) | Último sector físico (`USF`) |
| --- | ---: | ---: |
| 2 GB | 2048 | 3911679 |
| 4 GB | 2048 | 7772160 |
| 8 GB | 2048 | 16779263 |
| 16 GB | 2048 | 31115263 |

Con `DELTASECTOR = 97952` se calculan:

- `PSE = PSF + DELTASECTOR`: primer sector reservado para escritura.
- `infoPrimerSector = PSF + DELTASECTOR - 2`: sector para guardar el primer sector escrito.
- `infoUltimoSector = PSF + DELTASECTOR - 1`: sector para guardar el último sector escrito.

La prueba de `Ejemplo_uso_SD()` usa el sector `2500`, que es independiente del mapa calculado. `GuardarTramaSD()` usa inicialmente el sector `40000` y avanza cinco sectores por trama, pero su quinto `SD_Write_Block()` está comentado en el código actual.

## Formato previsto de una trama

`GuardarTramaSD()` documenta una trama de 2512 bytes:

```text
Cabecera (6 bytes) + tiempo (6 bytes) + aceleración (2500 bytes) = 2512 bytes
```

La cabecera global `cabeceraSD` está definida como:

```text
FF FD FB 0A 00 FA
```

El esquema previsto ocupa cuatro sectores completos y un quinto sector con 464 bytes de datos y 48 ceros. Sin embargo, la rutina actual usa valores de prueba (`1` a `7`) en lugar de copiar una trama real, no copia `cabeceraSD` ni las variables de tiempo, y deja comentada la escritura del quinto sector. Debe considerarse código de prueba pendiente de integración.

## Compilación y carga

1. Abrir `Main_SD.mcpds` en **mikroC PRO for dsPIC**.
2. Verificar que el dispositivo seleccionado sea `P33EP256MC202`.
3. Mantener incluidos en el proyecto `Main_SD.c`, `sdcard.c`, `spiSD.c`, `sdcard.h` y `spiSD.h`.
4. Comprobar que la frecuencia del proyecto sea 80 MHz y que las definiciones del dispositivo correspondan al hardware real.
5. Compilar y cargar el resultado con el programador apropiado para la placa.

Los archivos `.asm`, `.lst`, `.dbg`, `.brk` y similares son artefactos generados o auxiliares del entorno de mikroC; la compilación parte de los archivos C y H indicados arriba.

## Prueba rápida

1. Insertar una SD compatible y conectar el hardware SPI respetando CS activo en bajo.
2. Encender o reiniciar el microcontrolador.
3. Esperar la configuración inicial y la inicialización de la tarjeta.
4. Leer el sector `2500` con una herramienta de acceso directo a sectores y comprobar la secuencia `01 02 03 ... FF 01 ...`.
5. Comparar el sector `2501` con el sector `2500`; ambos deben contener la misma secuencia.
6. Un fallo de escritura produce 10, 11 o 12 parpadeos según el paso; un fallo de lectura produce 11 parpadeos. Los errores de inicialización usan el código devuelto por `SD_Init_Try(10)`.

## Estado actual y consideraciones

- La detección automática de inserción está desactivada por hardware.
- No hay sistema de archivos, nombres de archivo ni gestión de espacio libre.
- No hay una tarea principal de muestreo activa: la prueba termina dentro de `Ejemplo_uso_SD()` y después `main()` retorna.
- El UART y la interrupción INT1 solo se deshabilitan cuando falla la inicialización.
- La gestión de errores de bajo nivel depende de los códigos definidos en `sdcard.h` y de los patrones del LED.
- Antes de usar el firmware para datos reales se debe integrar la trama de aceleración, activar las escrituras necesarias y reservar formalmente los sectores para evitar colisiones con FAT32.