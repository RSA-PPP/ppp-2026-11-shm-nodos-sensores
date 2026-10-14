sbit CARD_DETECT at PORTA.B4;          // Pin 12 (RA4)
sbit CARD_DETECT_Direction at TRISA4_bit;

sbit LED_ESTADO at LATA2_bit;          // Pin 9 (RA2)
sbit LED_ESTADO_Direction at TRISA2_bit;

void main() {
    ANSELA = 0x0000;
    ANSELB = 0x0000;

    CARD_DETECT_Direction = 1;
    CNPUAbits.CNPUA4 = 1;              // pull-up interno
    LED_ESTADO_Direction = 0;
    LED_ESTADO = 0;

    while(1) {
        LED_ESTADO = (CARD_DETECT == 0);   // encendido = tarjeta presente
        Delay_ms(20);
    }
}