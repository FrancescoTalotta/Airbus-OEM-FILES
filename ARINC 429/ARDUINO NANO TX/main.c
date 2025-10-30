#include <avr/io.h>
#include <stdint.h>
#include <util/delay.h>

/* USART0 pins on ATmega2560:
   XCK0 = PD4 (clock out)
   TXD0 = PE1 (data out)
   RXD0 = PE0 (unused) */


static void usart0_mspim_init(uint16_t ubrr)
{
    // XCK0, TXD0 outputs; RXD0 input
    DDRD |= _BV(PD4);
    DDRD |= _BV(PD5);

    UCSR0B = 0;  // disable TX/RX while configuring

    // UMSEL01:0 = 11 (MSPIM), UCPOL0=1 (idle HIGH), 8-bit (UCSZ01:0=11)
    UCSR0C = (1<<UMSEL01) | (1<<UMSEL00)   // MSPIM mode
           | (0<<UCPOL0)                   // CPOL=0  -> clock idles LOW
           | (1<<UCSZ01)  | (1<<UCSZ00);   // UCSZ1=1 -> MBS first, UCSZ0 -> leading or tailing


    UCSR0B = (1<<TXEN0);          // enable transmitter (RX optional)

    UBRR0L = ubrr;                //set the baud, have to split it into the two registers
    UBRR0H = (ubrr >> 8);         //high end of baud register

    PORTD &= ~(1<<PD5);      //HIGH  Speed ARINC
    //PORTD |=  (1<<PD5);        //LOW Speed ARINC 

}


static void mspim_tx_bytes(const uint8_t *p, uint8_t n)
{

    // Clear TXC before starting a new frame train
    UCSR0A = (1<<TXC0);

    // Prime first byte
    while (!(UCSR0A & (1<<UDRE0))) { }
    UDR0 = *p++;

    // Feed the rest as soon as the data register empties
    while (--n) {
        while (!(UCSR0A & (1<<UDRE0))) { }
        UDR0 = *p++;
    }

    // Wait for the last byte to fully shift out (no gap before next call)
    while (!(UCSR0A & (1<<TXC0))) { }
}


/* ---------- ARINC 429 helpers ---------- */
typedef enum {
    ARINC_PARITY_MANUAL,
    ARINC_PARITY_ODD_AUTO,
    ARINC_PARITY_EVEN_AUTO
} arinc_parity_t;


unsigned char ARINC429_Permute8Bits(unsigned char X)
{
    unsigned char result=0;

    if (X&0x80) result |= 0x01;
    if (X&0x40) result |= 0x02;
    if (X&0x20) result |= 0x04;
    if (X&0x10) result |= 0x08;
    if (X&0x08) result |= 0x10;
    if (X&0x04) result |= 0x20;
    if (X&0x02) result |= 0x40;
    if (X&0x01) result |= 0x80;

    return result;
}


// compute parity bit over bits 0..30 of w31; return 0/1 for bit31
static inline uint8_t parity_bit_0_30(uint32_t w31, uint8_t want_odd) {
    w31 &= 0x7FFFFFFFUL;
    uint32_t x = w31;
    x ^= x>>16; x ^= x>>8; x ^= x>>4; x ^= x>>2; x ^= x>>1;
    uint8_t onesOdd = (uint8_t)(x & 1U);
    // if we want odd total, parity= !onesOdd; if we want even total, parity= onesOdd
    return want_odd ? (uint8_t)(!onesOdd) : onesOdd;
}


/* Build & send one ARINC word.
   - label: 8 bits
   - sdi  : 2 bits
   - data : 19 bits
   - ssm  : 2 bits
   - parity_mode: manual / odd auto / even auto
   - parity_bit: used only if parity_mode == MANUAL (0 or 1)
*/
void arinc429_send(uint8_t label, uint8_t sdi, uint32_t data19, uint8_t ssm,
                   arinc_parity_t parity_mode, uint8_t parity_bit_manual)
{
    // ARINC: label is bit-reversed on wire (MSB first overall)
    uint8_t lbl = ARINC429_Permute8Bits(label);
 
    // pack bits 0..30
    uint32_t w31 = ((uint32_t)lbl    << 24)
                 | ((uint32_t)sdi    << 22)
                 | ((uint32_t)data19 <<  2)
                 | ((uint32_t)ssm    <<  1);

    // decide parity (bit 31)
    uint8_t pbit;
    switch (parity_mode) {
        case ARINC_PARITY_MANUAL:    pbit = (parity_bit_manual ? 1u : 0u); break;
        case ARINC_PARITY_ODD_AUTO:  pbit = parity_bit_0_30(w31, /*want_odd*/1); break;
        case ARINC_PARITY_EVEN_AUTO: pbit = parity_bit_0_30(w31, /*want_odd*/0); break;
        default:                     pbit = parity_bit_0_30(w31, 1); break;
    }

    uint32_t w32 = (w31 & 0xFFFFFFFEUL) | ((uint32_t)pbit);

    // send MSB-first as 4 bytes with no gaps
    uint8_t buf[4] = {
        (uint8_t)((w32 >> 24)),
        (uint8_t)(ARINC429_Permute8Bits(w32 >> 16)),
        (uint8_t)(ARINC429_Permute8Bits(w32 >>  8)),
        (uint8_t)(ARINC429_Permute8Bits(w32      ))
    };
    mspim_tx_bytes(buf, 4);
}



int main(void)
{
    // Exact SCK = 12.5 kHz (UBRR=639)
    //usart0_mspim_init(639);
  
    // Switch to exact 100 kHz (UBRR=79)
    usart0_mspim_init(79);



    for(;;) {
        arinc429_send(0x5B, 0x01, 0x760FF, 0x03, ARINC_PARITY_ODD_AUTO, 0);
        _delay_ms(1000);
    }
}
