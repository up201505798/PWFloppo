/*
    (c) 2019 Microchip Technology Inc. and its subsidiaries.
    Subject to your compliance with these terms, you may use Microchip software and any
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party
    license terms applicable to your use of third party software (including open source software) that
    may accompany Microchip software.
    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS
    FOR A PARTICULAR PURPOSE.
    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS
    SOFTWARE.
*/
#define F_CPU                           (4000000UL)         /* using default clock 4MHz*/
#define USART1_BAUD_RATE(BAUD_RATE)     ((float)(64 * 4000000 / (16 * (float)BAUD_RATE)) + 0.5)
#include <avr/io.h>
#include <util/delay.h>
#include<avr/interrupt.h>

void USART1_sendString(const char* string);
volatile uint8_t buffer_out[100];
volatile uint8_t buffer_size=0;

ISR(USART1_RXC_vect){
    switch(USART1.RXDATAL){
        case '0': PORTA.OUTTGL = PIN0_bm; //drive
                  (PORTA.OUT & PIN0_bm)? USART1_sendString("Drive Desligada"): USART1_sendString("Drive Ligada");
                  break;
        case '1': PORTA.OUTTGL = PIN1_bm; //MOTOR
                  (PORTA.OUT & PIN0_bm)?  USART1_sendString("Motor Desligado"):USART1_sendString("Motor Ligado")  ;
      break;
        default: break;
        
    } 
}

ISR(USART1_TXC_vect){
   
    static uint8_t position=0;
    if(position<buffer_size){
        while(!(USART1.STATUS & USART_DREIF_bm));
   
        USART1.TXDATAL = buffer_out[position];
        position++;
    }
    else{
        position=0;
        buffer_size=0;
    }
}


void IO_init(void){
    
    PORTA.DIRSET = PIN0_bm;//DRIVE
    PORTA.OUTSET = PIN0_bm;//DRIVE OFF
    
    PORTA.DIRSET = PIN1_bm;//Motor
    PORTA.OUTSET = PIN1_bm;//motor OFF
    
 

}
void USART1_init(void){
    PORTC.DIRSET = PIN0_bm;                             /* set pin 0 of PORT C (TXd) as output*/
    PORTC.DIRCLR = PIN1_bm;                             /* set pin 1 of PORT C (RXd) as input*/
    
    USART1.BAUD = (uint16_t)(USART1_BAUD_RATE(9600));   /* set the baud rate*/
    
    USART1.CTRLC = USART_CHSIZE0_bm
                 | USART_CHSIZE1_bm;                    /* set the data format to 8-bit*/
                 
    USART1.CTRLA = USART_RXCIE_bm | USART_TXCIE_bm;   
    USART1.CTRLB |= (USART_TXEN_bm |USART_RXEN_bm) ;                      /* enable transmitter*/
}

void USART1_sendString(const char* string)
{
    int i=0;
    while(buffer_size!=0);
    for(i=0;string[i]!='\0';i++)
        buffer_out[i]=string[i];
    buffer_size=i;
    while(!(USART1.STATUS & USART_DREIF_bm));
    USART1.TXDATAL = '\n';
    
}






int main (void)
{
    







    IO_init();
    USART1_init();
    sei();
    while (1) 
    {
         

        
    }
}
    
    
    
    
    
 
    