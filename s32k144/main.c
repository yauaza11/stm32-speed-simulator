#include "device_registers.h"
#include "clocks_and_modes.h"
#include "ADC.h"


int lpit0_ch0_flag_counter = 0; /*< LPIT0 timeout counter */
int External_PIN = -1;
int D=0;
uint32_t adcResultInMv=0;
int led_delay = 0;
int i = -1;


void LPIT0_init (uint32_t delay)
{ 
    uint32_t timeout;
    /*! * LPIT Clocking: * ==============================
    */
    PCC->PCCn[PCC_LPIT_INDEX] = PCC_PCCn_PCS(6); /* Clock Src = 6
    (SPLL2_DIV2_CLK)*/
    PCC->PCCn[PCC_LPIT_INDEX] |= PCC_PCCn_CGC_MASK; /* Enable clk to LPIT0
    regs */
    /*! * LPIT Initialization: */
    LPIT0->MCR |= LPIT_MCR_M_CEN_MASK; /* DBG_EN-0: Timer chans stop in
    Debug mode */
    /* DOZE_EN=0: Timer chans are
    stopped in DOZE mode */
    /* SW_RST=0: SW reset does not reset
    timer chans, regs */
    /* M_CEN=1: enable module clk
    (allows writing other LPIT0 regs) */
    timeout=delay* 40; LPIT0->TMR[0].TVAL = timeout; /* Chan 0 Timeout period: 40M clocks */
    LPIT0->TMR[0].TCTRL |= LPIT_TMR_TCTRL_T_EN_MASK; /* T_EN=1: Timer channel is enabled */
    /* CHAIN=0: channel chaining is disabled */
    /* MODE=0: 32 periodic counter mode */
    /* TSOT=0: Timer decrements immediately based on
    restart */
    /* TSOI=0: Timer does not stop after timeout */
    /* TROT=0 Timer will not reload on trigger */
    /* TRG_SRC=0: External trigger soruce */
    /* TRG_SEL=0: Timer chan 0 trigger source is selected*/
}


void delay_us (volatile int us)
{
    LPIT0_init(us); /* Initialize PIT0 for 1 second timeout */
    while (0 == (LPIT0->MSR & LPIT_MSR_TIF0_MASK)) {} /* Wait for LPIT0 CH0 Flag */
    lpit0_ch0_flag_counter++; /* Increment LPIT0 timeout counter */
    LPIT0->MSR |= LPIT_MSR_TIF0_MASK; /* Clear LPIT0 timer flag 0 */
}

void FTM_init (void)
{
    //FTM0 clocking
    PCC->PCCn[PCC_FTM0_INDEX] &= ~PCC_PCCn_CGC_MASK; //Ensure clk disabled for config
    PCC->PCCn[PCC_FTM0_INDEX] |= PCC_PCCn_PCS(0b010) //Clocksrc=1, 8MHz
    SIRCDIV1_CLK|PCC_PCCn_CGC_MASK; //Enable clock for FTM regs
    //FTM0 Initialization
    FTM0->SC = FTM_SC_PWMEN1_MASK
    //Enable PWM channel 1output
    |FTM_SC_PS(0);
    //TOIE(timer overflow Interrupt Ena) = 0 (deafault)
    //CPWMS(Center aligned PWM Select) =0
    (default, up count)
    /* CLKS (Clock source) = 0 (default, no clock; FTM disabled) */
    /* PS (Prescaler factor) = 0. Prescaler = 1*/
    FTM0->MOD = 8000-1;
    //FTM0 counter final value (used for PWM mode)
    // FTM0 Period = MOD-CNTIN+0x0001~=8000 ctr clks=4ms
    //8Mhz /1 =8MHz
    FTM0->CNTIN = FTM_CNTIN_INIT(0);
    FTM0->CONTROLS[1].CnSC |=FTM_CnSC_MSB_MASK;
    FTM0->CONTROLS[1].CnSC |=FTM_CnSC_ELSA_MASK; /*FTM0 ch1: edge-aligned PWM, low true pulses */
    /* CHIE (Chan Interrupt Ena) = 0 (default)*/
    /* MSB:MSA (chan Mode Select)=0b10, Edge Align PWM */
    /* ELSB:ELSA (chan Edge/Level Select)=0b10, low true */
}

void FTM0_CH1_PWM (int i)
{
    //uint32_t i){
    FTM0->CONTROLS[1].CnV = i;//8000~0 duty; ex(7200=> Duty 0.1 / 800=>Duty 0.9)
    // start FTM0 counter with clk source = external clock (SOSCDIV1_CLK)
    FTM0->SC|=FTM_SC_CLKS(3);
}

void PORT_init (void)
{
    /*!
    * Pins definitions
    * ===================================================
    *
    * Pin number | Function
    * ----------------- |------------------
    * PTD16 | FTM0CH1
    */
    PCC->PCCn[PCC_PORTC_INDEX]|=PCC_PCCn_CGC_MASK; /* Enable clock for
    PORTC */
    PCC->PCCn[PCC_PORTD_INDEX ]|=PCC_PCCn_CGC_MASK; /* Enable clock for PORTD
    */
    PORTD->PCR[16]|=PORT_PCR_MUX(2); /* Port D16: MUX = ALT2, FTM0CH1 */
    PTD->PDDR |= 1<<12| 1<<2| 1<<3| 1<<4| 1<<5| 1<<6| 1<<7; /* Port D1-D7:
    Data Direction= output (default) */
    PORTD->PCR[12] = PORT_PCR_MUX(1); /* Port D1: MUX = GPIO */
    PORTD->PCR[2] = PORT_PCR_MUX(1); /* Port D2: MUX = GPIO */
    PORTD->PCR[3] = PORT_PCR_MUX(1); /* Port D3: MUX = GPIO */
    PORTD->PCR[4] = PORT_PCR_MUX(1); /* Port D4: MUX = GPIO */
    PORTD->PCR[5] = PORT_PCR_MUX(1); /* Port D5: MUX = GPIO */
    PORTD->PCR[6] = PORT_PCR_MUX(1); /* Port D6: MUX = GPIO */
    PORTD->PCR[7] = PORT_PCR_MUX(1); /* Port D7: MUX = GPIO */
    PTD->PDDR |=1<<8|1<<9|1<<10|1<<11;
    PORTD->PCR[8] = PORT_PCR_MUX(1); /* Port D8: MUX = GPIO */
    PORTD->PCR[9] = PORT_PCR_MUX(1); /* Port D9: MUX = GPIO */
    PORTD->PCR[10] = PORT_PCR_MUX(1); /* Port D10: MUX = GPIO */
    PORTD->PCR[11] = PORT_PCR_MUX(1); /* Port D11: MUX = GPIO */
    PTC->PDDR |= 1<<1| 1<<2| 1<<3| 1<<4| 1<<5| 1<<6| 1<<7 | 1<<8;
    /* Port C1-C8: Data Direction= output (default) */
    PORTC->PCR[1] = PORT_PCR_MUX(1); /* Port D1: MUX = GPIO */
    PORTC->PCR[2] = PORT_PCR_MUX(1); /* Port D1: MUX = GPIO */
    PORTC->PCR[3] = PORT_PCR_MUX(1); /* Port D1: MUX = GPIO */
    PORTC->PCR[4] = PORT_PCR_MUX(1); /* Port D1: MUX = GPIO */
    PORTC->PCR[5] = PORT_PCR_MUX(1); /* Port D1: MUX = GPIO */
    PORTC->PCR[6] = PORT_PCR_MUX(1); /* Port D1: MUX = GPIO */
    PORTC->PCR[7] = PORT_PCR_MUX(1); /* Port D1: MUX = GPIO */
    PORTC->PCR[8] = PORT_PCR_MUX(1); /* Port D1: MUX = GPIO */
    PTC->PDDR &= ~(1<<11);
    PORTC->PCR[11] |= PORT_PCR_MUX(1); // Port C11 mux = GPIO
    PORTC->PCR[11] |=(10<<16); // Port C11 IRQC : interrupt on Falling-edge
    PTC->PDDR &= ~(1<<12);
    PORTC->PCR[12] |= PORT_PCR_MUX(1); // Port C12 mux = GPIO
    PORTC->PCR[12] |=(10<<16); // Port C12 IRQC : interrupt on Falling-edge
    PTD->PDDR &= ~(1<<15);
    PORTD->PCR[15] |= PORT_PCR_MUX(1); // Port C12 mux = GPIO
    PORTD->PCR[15] |=(10<<16);
}
void num (int nom){
    switch(nom){ 
        case 0:
        PTD-> PSOR |= 1<<12;//PTD1; // FND A ON
        PTD-> PSOR |= 1<<2;//PTD2; // FND B ON
        PTD-> PSOR |= 1<<3;//PTD3; // FND C ON
        PTD-> PSOR |= 1<<4;//PTD4; // FND D ON
        PTD-> PSOR |= 1<<5;//PTD5; // FND E ON
        PTD-> PSOR |= 1<<6;//PTD6; // FND F ON
        PTD-> PCOR |= 1<<7;//PTD7; //
        break;
        
        case 1:
        PTD-> PCOR |= 1<<12;//PTD1; //
        PTD-> PSOR |= 1<<2;//PTD2; // FND B ON
        PTD-> PSOR |= 1<<3;//PTD3; // FND C ON
        PTD-> PCOR |= 1<<4;//PTD4; //
        PTD-> PCOR |= 1<<5;//PTD5; //
        PTD-> PCOR |= 1<<6;//PTD6; //
        PTD-> PCOR |= 1<<7;//PTD7; //
        break;
        
        case 2:
        PTD-> PSOR |= 1<<12;//PTD1; // FND A ON
        PTD-> PSOR |= 1<<2;//PTD2; // FND B ON
        PTD-> PCOR |= 1<<3;//PTD3; //
        PTD-> PSOR |= 1<<4;//PTD4; // FND D ON
        PTD-> PSOR |= 1<<5;//PTD5; // FND E ON
        PTD-> PCOR |= 1<<6;//PTD6; //
        PTD-> PSOR |= 1<<7;//PTD7; // FND G ON
        break;
        
        case 3:
        PTD-> PSOR |= 1<<12;//PTD1; // FND A ON
        PTD-> PSOR |= 1<<2;//PTD2; // FND B ON
        PTD-> PSOR |= 1<<3;//PTD3; // FND C ON
        PTD-> PSOR |= 1<<4;//PTD4; // FND D ON
        PTD-> PCOR |= 1<<5;//PTD5; //
        PTD-> PCOR |= 1<<6;//PTD6; //
        PTD-> PSOR |= 1<<7;//PTD7; // FND G ON
        break;
        
        case 4:
        PTD-> PCOR |= 1<<12;//PTD1; //
        PTD-> PSOR |= 1<<2;//PTD2; // FND B ON
        PTD-> PSOR |= 1<<3;//PTD3; // FND C ON
        PTD-> PCOR |= 1<<4;//PTD4; //
        PTD-> PCOR |= 1<<5;//PTD5; //
        PTD-> PSOR |= 1<<6;//PTD6; // FND F ON
        PTD-> PSOR |= 1<<7;//PTD7; // FND G ON
        break;
        
        case 5:
        PTD-> PSOR |= 1<<12;//PTD1; // FND A ON
        PTD-> PCOR |= 1<<2;//PTD2; //
        PTD-> PSOR |= 1<<3;//PTD3; // FND C ON
        PTD-> PSOR |= 1<<4;//PTD4; // FND D ON
        PTD-> PCOR |= 1<<5;//PTD5; //
        PTD-> PSOR |= 1<<6;//PTD6; // FND F ON
        PTD-> PSOR |= 1<<7;//PTD7; // FND G ON
        break;
        
        case 6:
        PTD-> PSOR |= 1<<12;//PTD1; // FND A ON
        PTD-> PCOR |= 1<<2;//PTD2; //
        PTD-> PSOR |= 1<<3;//PTD3; // FND C ON
        PTD-> PSOR |= 1<<4;//PTD4; // FND D ON
        PTD-> PSOR |= 1<<5;//PTD5; // FND E ON
        PTD-> PSOR |= 1<<6;//PTD6; // FND F ON
        PTD-> PSOR |= 1<<7;//PTD7; // FND G ON
        break;
        
        case 7:
        PTD-> PSOR |= 1<<12;//PTD1; // FND A ON
        PTD-> PSOR |= 1<<2;//PTD2; // FND B ON
        PTD-> PSOR |= 1<<3;//PTD3; // FND C ON
        PTD-> PCOR |= 1<<4;//PTD4; //
        PTD-> PCOR |= 1<<5;//PTD5; //
        PTD-> PSOR |= 1<<6;//PTD6; // FND F ON
        PTD-> PCOR |= 1<<7;//PTD7; //
        break;
        
        case 8:
        PTD-> PSOR |= 1<<12;//PTD1; // FND A ON
        PTD-> PSOR |= 1<<2;//PTD2; // FND B ON
        PTD-> PSOR |= 1<<3;//PTD3; // FND C ON
        PTD-> PSOR |= 1<<4;//PTD4; // FND D ON
        PTD-> PSOR |= 1<<5;//PTD5; // FND E ON
        PTD-> PSOR |= 1<<6;//PTD6; // FND F ON
        PTD-> PSOR |= 1<<7;//PTD7; // FND G ON
        break;
        
        case 9:
        PTD-> PSOR |= 1<<12;//PTD1; // FND A ON
        PTD-> PSOR |= 1<<2;//PTD2; // FND B ON
        PTD-> PSOR |= 1<<3;//PTD3; // FND C ON
        PTD-> PSOR |= 1<<4;//PTD4; // FND D ON
        PTD-> PCOR |= 1<<5;//PTD5; //
        PTD-> PSOR |= 1<<6;//PTD6; // FND F ON
        PTD-> PSOR |= 1<<7;//PTD7; // FND G ON
        break;

        case 10: // S
        PTD-> PSOR |= 1<<12;//PTD1; // FND A ON
        PTD-> PCOR |= 1<<2;//PTD2; //
        PTD-> PSOR |= 1<<3;//PTD3; // FND C ON
        PTD-> PSOR |= 1<<4;//PTD4; // FND D ON
        PTD-> PCOR |= 1<<5;//PTD5; //
        PTD-> PSOR |= 1<<6;//PTD6; // FND F ON
        PTD-> PSOR |= 1<<7;//PTD7; // FND G ON
        break;

        case 11: // T
        PTD-> PSOR |= 1<<12;//PTD1; // FND A ON
        PTD-> PSOR |= 1<<2;//PTD2; // FND B ON
        PTD-> PSOR |= 1<<3;//PTD3; // FND C ON
        PTD-> PCOR |= 1<<4;//PTD4; //
        PTD-> PCOR |= 1<<5;//PTD5; //
        PTD-> PCOR |= 1<<6;//PTD6; //
        PTD-> PCOR |= 1<<7;//PTD7; //
        break;

        case 12: // P
        PTD-> PSOR |= 1<<12;//PTD1; // FND A ON
        PTD-> PSOR |= 1<<2;//PTD2; // FND B ON
        PTD-> PCOR |= 1<<3;//PTD3; //
        PTD-> PCOR |= 1<<4;//PTD4; //
        PTD-> PSOR |= 1<<5;//PTD5; // ON
        PTD-> PSOR |= 1<<6;//PTD6; // FND F ON
        PTD-> PSOR |= 1<<7;//PTD7; // FND G ON
        break;

        case 13: // F
        PTD-> PSOR |= 1<<12;//PTD1; // FND A ON
        PTD-> PCOR |= 1<<2;//PTD2; // FND B ON
        PTD-> PCOR |= 1<<3;//PTD3; //
        PTD-> PCOR |= 1<<4;//PTD4; //
        PTD-> PSOR |= 1<<5;//PTD5; // ON
        PTD-> PSOR |= 1<<6;//PTD6; // FND F ON
        PTD-> PSOR |= 1<<7;//PTD7; // FND G ON
        break;

        case 14: // A
        PTD-> PSOR |= 1<<12;//PTD1; // FND A ON
        PTD-> PSOR |= 1<<2;//PTD2; // FND B ON
        PTD-> PSOR |= 1<<3;//PTD3; //
        PTD-> PCOR |= 1<<4;//PTD4; //
        PTD-> PSOR |= 1<<5;//PTD5; // ON
        PTD-> PSOR |= 1<<6;//PTD6; // FND F ON
        PTD-> PSOR |= 1<<7;//PTD7; // FND G ON
        break;
    }
}
void Seg_out(int number){
    int j;
    int d1000, d100, d10, d1;
    d1000 = number /1000;
    d100 = number % 1000/100;
    d10 = number % 100/10;
    d1 = number % 10;

    if(number == 2222){
        for( j = 0 ; j < 10 ; j++){ // 1000? ?? ??
            num(10)
            PTD-> PSOR |= 1<<8; PTD-> PCOR |= 1<<9; PTD-> PCOR |= 1<<10; PTD-> PCOR |= 1<<11;
            delay_us(1000);
            // 100? ?? ??
            num(11);
            PTD-> PCOR |= 1<<8; PTD-> PSOR |= 1<<9; PTD-> PCOR |= 1<<10; PTD-> PCOR |= 1<<11;
            delay_us(1000);
            //10 num(d100); num(0); PTD-> PCOR |= 1<<8; PTD-> PCOR |= 1<<9; PTD-> PSOR |= 1<<10; PTD-> PCOR |= 1<<11; delay_us(1000);
            // 1 num(d100); num(12); PTD-> PCOR |= 1<<8; PTD-> PCOR |= 1<<9; PTD-> PCOR |= 1<<10; PTD-> PSOR |= 1<<11; delay_us(1000);
        }
    }
    else if(number == 3333){
        for( j = 0 ; j < 10 ; j++){ // 1000? ?? ??
            num(13);
            PTD-> PSOR |= 1<<8;
            PTD-> PCOR |= 1<<9;
            PTD-> PCOR |= 1<<10;
            PTD-> PCOR |= 1<<11;
            delay_us(1000);
            // 100? ?? ??
            num(14);
            PTD-> PCOR |= 1<<8;
            PTD-> PSOR |= 1<<9;
            PTD-> PCOR |= 1<<10;
            PTD-> PCOR |= 1<<11;
            delay_us(1000);
            //10 num(d100); num(10); PTD-> PCOR |= 1<<8; PTD-> PCOR |= 1<<9; PTD-> PSOR |= 1<<10; PTD-> PCOR |= 1<<11; delay_us(1000);
            // 1 num(d100); num(11); PTD-> PCOR |= 1<<8; PTD-> PCOR |= 1<<9; PTD-> PCOR |= 1<<10; PTD-> PSOR |= 1<<11; delay_us(1000);
        }
    }
    else{
        for( j = 0 ; j < 10 ; j++){ // 1000? ?? ??
            num(d1000);
            PTD-> PSOR |= 1<<8;
            PTD-> PCOR |= 1<<9;
            PTD-> PCOR |= 1<<10;
            PTD-> PCOR |= 1<<11;
            delay_us(1000);
            // 100? ?? ??
            num(d100);
            PTD-> PCOR |= 1<<8;
            PTD-> PSOR |= 1<<9;
            PTD-> PCOR |= 1<<10;
            PTD-> PCOR |= 1<<11;
            delay_us(1000);
            //10 num(d100); num(d10); PTD-> PCOR |= 1<<8; PTD-> PCOR |= 1<<9; PTD-> PSOR |= 1<<10; PTD-> PCOR |= 1<<11; delay_us(1000);
            // 1 num(d100); num(d1); PTD-> PCOR |= 1<<8; PTD-> PCOR |= 1<<9; PTD-> PCOR |= 1<<10; PTD-> PSOR |= 1<<11; delay_us(1000);
        }
    }
}

void NVIC_init_IRQs(void){

    S32_NVIC->ICPR[1] |= 1<<(61%32); // Clear any pending IRQ61
    S32_NVIC->ISER[1] |= 1<<(61%32); // Enable IRQ61
    S32_NVIC->IP[61] =0xB; //Priority 11 of 15
    S32_NVIC->ICPR[1] |= 1<<(62%32); // Clear any pending IRQ61
    S32_NVIC->ISER[1] |= 1<<(62%32); // Enable IRQ61
    S32_NVIC->IP[62] =0xA; //Priority 11 of 15

}

void PORTC_IRQHandler(void){
    PORTC->PCR[11] &= ~(0x01000000); // Port Control Register ISF bit '0' set
    //PORTC_Interrupt State Flag Register Read
    if((PORTC->ISFR & (1<<11)) != 0){
        External_PIN++;
        External_PIN = External_PIN%2;
    }
    // External input Check Behavior Assignment
    switch (External_PIN){
        case 0:
        break;

        default:
        break;
    }

    PORTC->PCR[11] |= 0x01000000; // Port Control Register ISF bit '1' set
}

void PORTD_IRQHandler(void){
    PORTD->PCR[15] &= ~(0x01000000); // Port Control Register ISF bit '0' set
    if((PORTD->ISFR & (1<<15)) != 0){
        External_PIN = 1;
    }
    for (int x = 0 ; x <= 5000 ; x++){
        Seg_out(1234); FTM0_CH1_PWM(8000);
    }
    PORTD->PCR[15] |= 0x01000000; // Port Control Register ISF bit '1' set
}

int main(void)
{
    /*< ADC0 Result in miliVolts */
    SOSC_init_8MHz(); /* Initialize system oscillator for 8 MHz xtal */
    SPLL_init_160MHz(); /* Initialize SPLL to 160 MHz with 8 MHz SOSC */
    NormalRUNmode_80MHz(); /* Init clocks: 80 MHz SPLL & core, 40 MHz bus, 20 MHz flash */
    FTM_init(); ADC_init(); /* Init ADC resolution 12 bit */
    PORT_init(); /* Configure ports */
    NVIC_init_IRQs();


    for(;;) {
        convertAdcChan(12); /* Convert Channel AD12 to pot on EVB */

        while(adc_complete()==0){

        } /* Wait for conversion complete flag*/

        adcResultInMv = read_adc_chx(); /* Get channel's conversion results in mv */
        D=adcResultInMv*1.6; / *
        5000*1.6=8000*/

        if(adcResultInMv == 0){
            FTM0_CH1_PWM(8000);
            Seg_out(3333);
        }

        else if(External_PIN != 0){
                FTM0_CH1_PWM(D);
                Seg_out(5000-adcResultInMv);
                led_delay = 50*(5000-(5000-adcResultInMv) + 1);
                i++;
                i = i%8; switch (i){
                case 0: PTC->PCOR |= 1<<1;
                /* turn ON Port D1 */
                PTC->PSOR |= 1<<2|1<<3|1<<4|1<<5|1<<6|1<<7|1<<8; /* else turn OFF */
                delay_us(led_delay);
                break;

                case 1: PTC->PCOR |= 1<<2;
                /* turn ON Port D1 */
                PTC->PSOR |= 1<<1|1<<3|1<<4|1<<5|1<<6|1<<7|1<<8; /* else turn OFF */
                delay_us(led_delay);
                break;

                case 2: PTC->PCOR |= 1<<3;
                /* turn ON Port D1 */
                PTC->PSOR |= 1<<2|1<<1|1<<4|1<<5|1<<6|1<<7|1<<8; /* else turn OFF */
                delay_us(led_delay);
                break;

                case 3: PTC->PCOR |= 1<<4;
                /* turn ON Port D1 */
                PTC->PSOR |= 1<<2|1<<3|1<<1|1<<5|1<<6|1<<7|1<<8; /* else turn OFF */
                delay_us(led_delay);
                break;

                case 4: PTC->PCOR |= 1<<5;
                /* turn ON Port D1 */
                PTC->PSOR |= 1<<2|1<<3|1<<4|1<<1|1<<6|1<<7|1<<8; /* else turn OFF */
                delay_us(led_delay);
                break;

                case 5: PTC->PCOR |= 1<<6;
                /* turn ON Port D1 */
                PTC->PSOR |= 1<<2|1<<3|1<<4|1<<5|1<<1|1<<7|1<<8; /* else turn OFF */
                delay_us(led_delay);
                break;

                case 6: PTC->PCOR |= 1<<7;
                /* turn ON Port D1 */
                PTC->PSOR |= 1<<2|1<<3|1<<4|1<<5|1<<6|1<<1|1<<8; /* else turn OFF */
                delay_us(led_delay);
                break;

                case 7: PTC->PCOR |= 1<<8;
                /* turn ON Port D1 */
                PTC->PSOR |= 1<<2|1<<3|1<<4|1<<5|1<<6|1<<7|1<<1; /* else turn OFF */
                delay_us(led_delay);
                break;

                default: break;
            }
        }
        else if (External_PIN == 0){
            FTM0_CH1_PWM(8000);
            Seg_out(2222);
        }
    }
}