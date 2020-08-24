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
#define F_CPU                           (24000000UL)         /* using default clock 4MHz*/
#define USART1_BAUD_RATE(BAUD_RATE)     ((float)(64 * F_CPU / (16 * (float)BAUD_RATE)) + 0.5)
#define PERIOD_EXAMPLE_VALUE			(0x8C4F)
#define PERIOD_EXAMPLE_VALUE1			(0xA0)
#include <avr/io.h>
//#include <util/delay.h>
#include<avr/interrupt.h>

#define SEEK_STOP 0
#define SEEK_0 1
#define SEEK_VALUE 2
#define SEEK_PLUS 0
#define SEEK_MINUS 1
#define SEEK_INIT_VAL 255
#define SEEK_MAX 79


#define BUF_BIG 15360

void USART1_sendString(const char* string);
volatile uint8_t buffer_out[100];
volatile uint8_t buffer_size=0;

volatile uint8_t seek_mode=0;
volatile uint8_t seek_curr_pos=SEEK_INIT_VAL;
volatile uint8_t seek_next_pos=0;


volatile uint8_t debug_var=0;
//volatile uint8_t state=0;
 
volatile uint8_t buf1[BUF_BIG] __attribute__((aligned(64))); ;


volatile uint8_t write_mode=0;

#define WRITE_OFF 0
#define DELETE_TRACK 1
#define FIND_SECTOR 2
#define SAVE_SECTOR 3
volatile uint8_t curr_sector=0;



void PWM_dataout_init(void);




void CLK_Init(void){   
    _PROTECTED_WRITE (CLKCTRL.OSCHFCTRLA, CLKCTRL_FREQSEL_24M_gc);    
    _PROTECTED_WRITE (CLKCTRL.XOSC32KCTRLA, CLKCTRL_ENABLE_bm);
    _PROTECTED_WRITE (CLKCTRL.MCLKCTRLA, (CLKCTRL_CLKSEL_OSCHF_gc | CLKCTRL_CLKOUT_bm));
}

ISR(USART1_RXC_vect){
    static uint8_t first=1;
    if(first){
        if(USART1.RXDATAL=='1'){
            
            USART1.CTRLB &= ~(USART_TXEN_bm) ;  
            first=0;
            return;
        }
    }
    static uint16_t counter_rx=0;
    buf1[counter_rx]=USART1.RXDATAL;
    
    if(++counter_rx==(BUF_BIG-2)){
        counter_rx=0;
        write_mode=DELETE_TRACK;
        PORTB.OUTCLR = PIN5_bm; //Modo escrita
        PORTE.OUTSET = PIN1_bm;
        //TCA0.SINGLE.CTRLA |=TCA_SINGLE_ENABLE_bm;		/* start timer */
        
    }
        
    
}



ISR(TCA1_OVF_vect){
    static uint8_t seek_firstINT_passed=0;
   // USART1_sendString("a");
    TCA1.SINGLE.INTFLAGS|=TCA_SINGLE_OVF_bm;
   // PORTA.OUTTGL = PIN0_bm; //drive
    switch(seek_mode){
        
        case SEEK_VALUE:
            if(!(!(PORTE.IN & PIN3_bm)&& seek_curr_pos!=0)){ //Evitar encravamento no 0
                if(seek_firstINT_passed==1){
                    if((PORTE.OUT & PIN0_bm)==SEEK_PLUS){// direcao positiva
                        seek_curr_pos++;
                    }
                    if((PORTE.OUT & PIN0_bm)==SEEK_MINUS){// direcao positiva
                        seek_curr_pos--;
                    }
                }
                else
                    seek_firstINT_passed=1;
                if(seek_curr_pos==seek_next_pos){
                    TCA1.SINGLE.CTRLA &=  ~TCA_SINGLE_ENABLE_bm;		/* stop timer */
                    TCA1.SINGLE.CNT=0;
                    TCA1.SINGLE.INTFLAGS|=TCA_SINGLE_OVF_bm;
                    seek_mode=SEEK_STOP;
                    seek_firstINT_passed=0;
                    break;
                }
                break;
            }
            else ; //erro  detetado fora no 0
        case SEEK_0: 
            if(!(PORTE.IN & PIN3_bm)){ //chegou a 0
                TCA1.SINGLE.CTRLA &=  ~TCA_SINGLE_ENABLE_bm;		/* stop timer */
                seek_curr_pos=0;
                TCA1.SINGLE.CNT=0;
                TCA1.SINGLE.INTFLAGS|=TCA_SINGLE_OVF_bm;
                seek_mode=SEEK_STOP;
                seek_firstINT_passed=0;
                
            }
            break;
        
        case SEEK_STOP:
        default:
            TCA1.SINGLE.CTRLA &=  ~TCA_SINGLE_ENABLE_bm;		/* stop timer */
            TCA1.SINGLE.CNT=0;
            TCA1.SINGLE.INTFLAGS|=TCA_SINGLE_OVF_bm;
            seek_firstINT_passed=0;
            break;
    }
    
    
}




ISR (TCA0_OVF_vect) {
    asm volatile("push r16");
    asm volatile("ldi r16,16");
    asm volatile("sts 0x0425,r16");
    //PORTB.OUTSET=PIN4_bm;
    asm volatile("ldi r16,1");
    asm volatile("sts 0x0A0B,r16");
    //TCA0.SINGLE.INTFLAGS|=TCA_SINGLE_OVF_bm;
    //static uint8_t counter=0;
    //static uint16_t counter=0;
   // static  uint8_t selector=0;
    //uint16_t c=counter;
    
    static uint8_t* pointer=buf1;
    uint8_t* p=pointer;
    TCA0.SINGLE.PERBUF=*p;
    
    uint8_t comp1=(uint8_t)(((uint16_t)(++p))>>8);
    
    //TCA0.SINGLE.PERBUF=buf[selector][counter];
    //counter++;
    
   // if(counter==BUF_MAXIMO){
    if(0b01111100==comp1){
        p=(uint8_t*)0x4000;
        TCA0.SINGLE.CTRLA &=~TCA_SINGLE_ENABLE_bm;
        write_mode=WRITE_OFF;
       // PORTB.OUTSET = PIN5_bm; //Modo leitura
        //static uint8_t* pointer1=&buf1[BUF_BIG/2];
        
        //static uint8_t* pointer0=buf1;
        
        
    }
    pointer=p;
    
    asm volatile("sts 0x0A07,r16");
    //TCA0.SINGLE.CTRLFSET|=TCA_SINGLE_PERBV_bm;
    asm volatile("ldi r16,16");
    asm volatile("sts 0x0426,r16");
    asm volatile("pop r16");
    //PORTB.OUTCLR=PIN4_bm;
     
}

ISR(PORTE_PORT_vect){
    
    if(PORTE.INTFLAGS&PIN2_bm){ //INTERRUCAO INDEX
        
        PORTE.INTFLAGS|=(PIN2_bm);
        if((PORTE.IN&PIN2_bm)){
            if(write_mode==DELETE_TRACK){//comeco
                static uint8_t erase_passes=0;
                if(erase_passes)
                    erase_passes--;
                else{
                    erase_passes=0;
                    TCA0.SINGLE.PER = 250;
                    write_mode=SAVE_SECTOR;
                    TCA0.SINGLE.CTRLA |=  TCA_SINGLE_ENABLE_bm;	
                }
            }
            if(write_mode==WRITE_OFF){
                PORTB.OUTSET = PIN5_bm; //Modo leitura
            }
        }
            
    }
        /*if(!(PORTE.IN&PIN2_bm)&&(TCA0.SINGLE.CTRLA&TCA_SINGLE_ENABLE_bm)){
            TCA0.SINGLE.CTRLA&=~TCA_SINGLE_ENABLE_bm;
            PORTB.OUTSET=PIN5_bm;
        }*/
    
}
int move_head(uint8_t mode, uint8_t new_pos){
    int8_t delta=127;
    switch(mode){
        case SEEK_VALUE: 
            if (new_pos>SEEK_MAX)
                return 255;
            if(seek_curr_pos!=SEEK_INIT_VAL){
                delta=(int8_t)new_pos-(int8_t)seek_curr_pos;
                if(delta==0) //mesma posicao
                    return delta;
                if(delta>0){//avancar posçao{
                    if((PORTE.OUT & PIN0_bm)!=SEEK_PLUS) // direçao negativa __ ATENcao:SEEK_PLUS<<PIN0_bm
                        PORTE.OUTTGL=PIN0_bm;
                              
                }
                else //recuar
                    if((PORTE.OUT & PIN0_bm)!=SEEK_MINUS) // direçao positiva __ ATENcao:SEEK_PLUS<<PIN0_bm
                        PORTE.OUTTGL=PIN0_bm;
                
                seek_next_pos=new_pos;
                seek_mode=SEEK_VALUE;
                TCA1.SINGLE.CNT=0;
                TCA1.SINGLE.INTFLAGS|=TCA_SINGLE_OVF_bm;
                TCA1.SINGLE.CTRLA |=  TCA_SINGLE_ENABLE_bm;		/* start timer */
                break;
            }
            else;//posicao nao inicializada continua no proximo case
        case SEEK_0:    
           /* if(!(PORTE.OUT & PIN1_bm)){
                seek_curr_pos=0;
                return 0;
            }
            else{*/
                delta=0;
                if((PORTE.OUT & PIN0_bm)!=SEEK_MINUS) // direçao negativa __ ATENcao:SEEK_PLUS<<PIN0_bm
                        PORTE.OUTTGL=PIN0_bm;
                seek_mode=SEEK_0;
                TCA1.SINGLE.CNT=0;
                TCA1.SINGLE.INTFLAGS|=TCA_SINGLE_OVF_bm;
                TCA1.SINGLE.CTRLA |=  TCA_SINGLE_ENABLE_bm;		/* start timer */
                break;
          //  }
        default:
            return 255;
    }
        while(seek_mode!=SEEK_STOP); //esperar pelo fim do processo--> bloqueante
        
        return delta;
}
                        
                    
                    
                    
                


void PWM_stepper_init(void){
	PORTMUX.TCAROUTEA |= PORTMUX_TCA10_bm;			/* set waveform output on PORT C */

    TCA1.SINGLE.CTRLB = TCA_SINGLE_CMP2EN_bm                    /* enable compare channel 2 */
                      | TCA_SINGLE_WGMODE_DSBOTTOM_gc ;		/* single-slope PWM mode */
    
    TCA1.SINGLE.PER = PERIOD_EXAMPLE_VALUE;			/* set PWM frequency*/
    
    TCA1.SINGLE.INTCTRL= TCA_SINGLE_OVF_bm;
    
    TCA1.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc;/* set clock source (sys_clk/1) */
                      //| TCA_SINGLE_ENABLE_bm;		/* start timer */
    TCA1.SINGLE.CMP2 = PERIOD_EXAMPLE_VALUE*0.95;

}

void PWM_dataout_init(void){
    buf1[BUF_BIG-1]=185;
    buf1[BUF_BIG-2]=185;
	PORTMUX.TCAROUTEA |= PORTMUX_TCA02_bm;			/* set waveform output on PORT E */

    TCA0.SINGLE.CTRLB = TCA_SINGLE_CMP1EN_bm                    /* enable compare channel 1 */
                     // |  TCA_SINGLE_ALUPD_bm 
                      | TCA_SINGLE_WGMODE_SINGLESLOPE_gc; 		/* single-slope PWM mode */
                    //  |TCA_SINGLE_ALUPD_bm;
  
    
  // TCA0.SINGLE.CTRLESET=TCA_SINGLE_LUPD_bp;
    
    TCA0.SINGLE.PER = 250;			/* set PWM frequency*/
    
    TCA0.SINGLE.INTCTRL= TCA_SINGLE_OVF_bm ;
    
  // TCA0.SINGLE.CTRLESET=TCA_SINGLE_DIR_bm;
    TCA0.SINGLE.CMP1 = PERIOD_EXAMPLE_VALUE1*0.05;
     TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc;/* set clock source (sys_clk/1) */
                       
}

void IO_init(void){
    
    PORTA.DIRSET = PIN2_bm;//DRIVE
    //PORTA.OUTSET = PIN2_bm;//DRIVE OFF
    PORTA.OUTCLR = PIN2_bm;//DRIVE ON
    
    PORTA.DIRSET = PIN3_bm;//Motor
    PORTA.OUTSET = PIN3_bm;//motor OFF
    
    PORTC.DIRSET = PIN6_bm;//step PWM
    PORTC.OUTSET = PIN6_bm;//repouso
    
    PORTE.DIRSET = PIN0_bm;//seek_dir
    PORTE.OUTSET = PIN0_bm;//pos neg
            
    PORTE.DIRCLR = PIN3_bm;//seek0
   //PORTE.PIN1CTRL &=~(PORT_PULLUPEN_bm);
    PORTE.PIN3CTRL =0;
    
    PORTE.DIRCLR= PIN2_bm; //INDEX
    PORTE.PIN2CTRL=1; //INT BOTHEDGES & pull_up disabled
 
    PORTE.DIRSET= PIN1_bm; //data out
    PORTE.PIN1CTRL=PORT_INVEN_bm;
    
    PORTB.DIRSET= PIN5_bm; //Write gate
    PORTB.OUTSET = PIN5_bm; //Modo leitura
     
    PORTB.DIRSET= PIN4_bm;
}
 void USART1_init(void){
    PORTC.DIRSET = PIN0_bm;                             /* set pin 0 of PORT C (TXd) as output*/
    PORTC.DIRCLR = PIN1_bm;                             /* set pin 1 of PORT C (RXd) as input*/
    
    USART1.BAUD = (uint16_t)(USART1_BAUD_RATE(9600));   /* set the baud rate*/
    
    USART1.CTRLC = USART_CHSIZE0_bm
                 | USART_CHSIZE1_bm;                    /* set the data format to 8-bit*/
                 
    USART1.CTRLA = USART_RXCIE_bm ;//| USART_TXCIE_bm;   
    USART1.CTRLB |= (USART_TXEN_bm |USART_RXEN_bm) ;                      /* enable transmitter*/
    
    
    
    PORTC.DIRCLR=PIN7_bm;
    PORTC.PIN7CTRL|= PORT_PULLUPEN_bm;
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
    




    CLK_Init();


    IO_init();
    PORTA.OUTCLR= PIN3_bm;//motor on;
    PORTA.OUTCLR= PIN2_bm;//drive_on;
    
    PWM_dataout_init();
    //TCA1.SINGLE.CMP2 = dutyCycle;
    
    USART1_init();
    //PWM_stepper_init();
    //move_head(SEEK_0,0);
    sei();
    
    while(1){
      if(!(PORTC.IN &(PIN7_bm))) 
          USART1.TXDATAL = '1';
      while(!(PORTC.IN &(PIN7_bm)));
    }
    /*
    //PWM_stepper_init();
    buf_init();
    PWM_dataout_init();
    sei();
    
    while(debug_var==0);
    
   // _delay_ms(500);
    move_head(SEEK_0,0);
    PORTB.OUTCLR=PIN5_bm;    
    while(1);*/
}
    
    




    
    
 
    