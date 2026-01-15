#include "device_registers.h"
#include "clocks_and_modes.h"
#include "ADC.h"

// 전역 변수 설정
int Ipit0_ch0_flag_counter = 0; /*< LPITO timeout counter */
int External_PIN = -1;
int D = 0;
uint32_t adcResultInMv = 0;
int led_delay = 0;
int i = -1;

// LPIT 초기화 함수
void LPITO_init (uint32_t delay)
{
    uint32_t timeout;
    PCC->PCCn[PCC_LPIT_INDEX] = PCC_PCCn_PCS(6);
    PCC->PCCn[PCC_LPIT_INDEX] |= PCC_PCCn_CGC_MASK; /* Enable clk to LPITO */
    LPITO->MCR |= LPIT_MCR_M_CEN_MASK; /* M_CEN = 1, enable module clk */
    timeout = delay * 40;
    LPITO->TMR[0].TVAL = timeout; /* Chan 0 Timeout period */
    LPITO->TMR[0].TCTRL |= LPIT_TMR_TCTRL_T_EN_MASK; /* T_EN=1: Timer channel is enabled */
}

// 지연 시간 함수
void delay_us (volatile int us) {
    LPITO_init(us);
    Ipit0_ch0_flag_counter++;
    while (0 == (LPITO->MSR & LPIT_MSR_TIFO_MASK)) {} /* Wait for LPITO CHO Flag */
    LPITO->MSR |= LPIT_MSR_TIFO_MASK; /* Clear LPITO timer flag 0 */
}

// FTM(PWM) 초기화 함수
void FTM_init (void) {
    PCC->PCCn[PCC_FTM0_INDEX] &= ~PCC_PCCn_CGC_MASK;
    PCC->PCCn[PCC_FTM0_INDEX] |= PCC_PCCn_PCS(0b010); // 8MHz SIRCDIV1_CLK
    PCC->PCCn[PCC_FTM0_INDEX] |= PCC_PCCn_CGC_MASK;
    FTM0->SC = FTM_SC_PWMEN1_MASK | FTM_SC_PS(0);
    FTM0->MOD = 8000 - 1;
    FTM0->CNTIN = FTM_CNTIN_INIT(0);
    FTM0->CONTROLS[1].CnSC |= FTM_CnSC_MSB_MASK;
    FTM0->CONTROLS[1].CnSC |= FTM_CnSC_ELSA_MASK;
}

// PWM 출력 함수 (모터 속도 제어)
void FTM0_CH1_PWM (int i) {
    FTM0->CONTROLS[1].CnV = i;
    FTM0->SC |= FTM_SC_CLKS(3);
}

// 포트 및 입출력 설정 함수
void PORT_init (void) {
    PCC->PCCn[PCC_PORTC_INDEX] |= PCC_PCCn_CGC_MASK;
    PCC->PCCn[PCC_PORTD_INDEX] |= PCC_PCCn_CGC_MASK;
    PORTD->PCR[16] |= PORT_PCR_MUX(2); // PTD16: FTM0CH1

    // 7-segment 및 스위치 포트 설정
    PTD->PDDR |= 1<<12 | 1<<2 | 1<<3 | 1<<4 | 1<<5 | 1<<6 | 1<<7;
    PORTD->PCR[12] = PORT_PCR_MUX(1); // D1->D12 변경 사항 반영
    PORTD->PCR[2] = PORT_PCR_MUX(1);
    PORTD->PCR[3] = PORT_PCR_MUX(1);
    PORTD->PCR[4] = PORT_PCR_MUX(1);
    PORTD->PCR[5] = PORT_PCR_MUX(1);
    PORTD->PCR[6] = PORT_PCR_MUX(1);
    PORTD->PCR[7] = PORT_PCR_MUX(1);
    PTD->PDDR |= 1<<8 | 1<<9 | 1<<10 | 1<<11;
    PORTD->PCR[8] = PORT_PCR_MUX(1);
    PORTD->PCR[9] = PORT_PCR_MUX(1);
    PORTD->PCR[10] = PORT_PCR_MUX(1);
    PORTD->PCR[11] = PORT_PCR_MUX(1);
    
    // LED 포트 설정 (C1~C8)
    PTC->PDDR |= 1<<1 | 1<<2 | 1<<3 | 1<<4 | 1<<5 | 1<<6 | 1<<7 | 1<<8;
    for(int p=1; p<=8; p++) PORTC->PCR[p] = PORT_PCR_MUX(1);

    // 인터럽트 스위치 설정 (C11, C12, D15)
    PTC->PDDR &= ~(1<<11);
    PORTC->PCR[11] |= PORT_PCR_MUX(1) | (10<<16); // Falling-edge
    PTC->PDDR &= ~(1<<12);
    PORTC->PCR[12] |= PORT_PCR_MUX(1) | (10<<16);
    PTD->PDDR &= ~(1<<15);
    PORTD->PCR[15] |= PORT_PCR_MUX(1) | (10<<16);
}

// 7-segment 숫자/문자 매핑 함수
void num (int nom) {
    switch(nom) {
        case 0: PTD->PSOR |= 1<<12|1<<2|1<<3|1<<4|1<<5|1<<6; PTD->PCOR |= 1<<7; break;
        case 1: PTD->PCOR |= 1<<12; PTD->PSOR |= 1<<2|1<<3; PTD->PCOR |= 1<<4|1<<5|1<<6|1<<7; break;
        case 2: PTD->PSOR |= 1<<12|1<<2|1<<4|1<<5|1<<7; PTD->PCOR |= 1<<3|1<<6; break;
        case 3: PTD->PSOR |= 1<<12|1<<2|1<<3|1<<4|1<<7; PTD->PCOR |= 1<<5|1<<6; break;
        case 4: PTD->PCOR |= 1<<12|1<<4|1<<5; PTD->PSOR |= 1<<2|1<<3|1<<6|1<<7; break;
        case 5: PTD->PSOR |= 1<<12|1<<3|1<<4|1<<6|1<<7; PTD->PCOR |= 1<<2|1<<5; break;
        case 6: PTD->PSOR |= 1<<12|1<<3|1<<4|1<<5|1<<6|1<<7; PTD->PCOR |= 1<<2; break;
        case 7: PTD->PSOR |= 1<<12|1<<2|1<<3|1<<6; PTD->PCOR |= 1<<4|1<<5|1<<7; break;
        case 8: PTD->PSOR |= 1<<12|1<<2|1<<3|1<<4|1<<5|1<<6|1<<7; break;
        case 9: PTD->PSOR |= 1<<12|1<<2|1<<3|1<<4|1<<6|1<<7; PTD->PCOR |= 1<<5; break;
        case 10: // S (STOP)
            PTD->PSOR |= 1<<12|1<<3|1<<4|1<<6|1<<7; PTD->PCOR |= 1<<2|1<<5; break;
        case 11: // T 
            PTD->PSOR |= 1<<12|1<<2|1<<3; PTD->PCOR |= 1<<4|1<<5|1<<6|1<<7; break;
        case 12: // P 
            PTD->PSOR |= 1<<12|1<<2|1<<5|1<<6|1<<7; PTD->PCOR |= 1<<3|1<<4; break;
        case 13: // F (FAST)
            PTD->PSOR |= 1<<12|1<<5|1<<6|1<<7; PTD->PCOR |= 1<<2|1<<3|1<<4; break;
        case 14: // A
            PTD->PSOR |= 1<<12|1<<2|1<<3|1<<5|1<<6|1<<7; PTD->PCOR |= 1<<4; break;
    }
}

// 7-segment 출력 제어 함수 [cite: 333-434]
void Seg_out(int number) {
    int j, d1000 = number / 1000, d100 = number % 1000 / 100, d10 = number % 100 / 10, d1 = number % 10;
    if(number == 2222) { // STOP 모드
        for(j=0; j<10; j++) {
            num(10); PTD->PSOR |= 1<<8; PTD->PCOR |= 1<<9|1<<10|1<<11; delay_us(1000); // S
            num(11); PTD->PCOR |= 1<<8; PTD->PSOR |= 1<<9; PTD->PCOR |= 1<<10|1<<11; delay_us(1000); // T
            num(0);  PTD->PCOR |= 1<<8|1<<9; PTD->PSOR |= 1<<10; PTD->PCOR |= 1<<11; delay_us(1000); // O
            num(12); PTD->PCOR |= 1<<8|1<<9|1<<10; PTD->PSOR |= 1<<11; delay_us(1000); // P
        }
    } else if(number == 3333) { // FAST 모드
        for(j=0; j<10; j++) {
            num(13); PTD->PSOR |= 1<<8; PTD->PCOR |= 1<<9|1<<10|1<<11; delay_us(1000); // F
            num(14); PTD->PCOR |= 1<<8; PTD->PSOR |= 1<<9; PTD->PCOR |= 1<<10|1<<11; delay_us(1000); // A
            num(10); PTD->PCOR |= 1<<8|1<<9; PTD->PSOR |= 1<<10; PTD->PCOR |= 1<<11; delay_us(1000); // S
            num(11); PTD->PCOR |= 1<<8|1<<9|1<<10; PTD->PSOR |= 1<<11; delay_us(1000); // T
        }
    } else { // 일반 전압 출력
        for(j=0; j<10; j++) {
            num(d1000); PTD->PSOR |= 1<<8; PTD->PCOR |= 1<<9|1<<10|1<<11; delay_us(1000);
            num(d100);  PTD->PCOR |= 1<<8; PTD->PSOR |= 1<<9; PTD->PCOR |= 1<<10|1<<11; delay_us(1000);
            num(d10);   PTD->PCOR |= 1<<8|1<<9; PTD->PSOR |= 1<<10; PTD->PCOR |= 1<<11; delay_us(1000);
            num(d1);    PTD->PCOR |= 1<<8|1<<9|1<<10; PTD->PSOR |= 1<<11; delay_us(1000);
        }
    }
}

// 인터럽트 설정 함수
void NVIC_init_IRQs(void) {
    S32_NVIC->ICPR[1] |= 1<<(61%32); S32_NVIC->IP[61] = 0xB; S32_NVIC->ISER[1] = 1<<(61%32);
    S32_NVIC->ICPR[1] |= 1<<(62%32); S32_NVIC->ISER[1] = 1<<(62%32); S32_NVIC->IP[62] = 0xA;
}

// 포트 C 인터럽트 핸들러
void PORTC_IRQHandler(void) {
    PORTC->PCR[11] &= ~(0x01000000);
    if((PORTC->ISFR & (1<<11)) != 0) {
        External_PIN++;
        External_PIN = External_PIN % 2; // 스위치 토글
    }
    PORTC->PCR[11] = 0x01000000;
}

// 메인 함수
int main(void) {
    SOSC_init_8MHz(); SPLL_init_160MHz(); NormalRUNmode_80MHz();
    FTM_init(); ADC_init(); PORT_init(); NVIC_init_IRQs();

    for(;;) {
        convertAdcChan(12);
        while(adc_complete() == 0){}
        adcResultInMv = read_adc_chx();
        D = adcResultInMv * 1.6; // PWM 듀티 사이클 계산

        if(adcResultInMv == 5000) { // 과속 방지 모드
            FTM0_CH1_PWM(8000); Seg_out(3333); // FAST 표시
        }
        else if(External_PIN != 0) { // 정상 주행 모드
            FTM0_CH1_PWM(D);
            Seg_out(adcResultInMv);
            led_delay = 50 * (5000 - adcResultInMv); // 속도에 따른 LED 딜레이
            i = (i + 1) % 8;
            switch (i) { // LED 순차 점멸
                case 0: PTC->PCOR |= 1<<1; PTC->PSOR |= 1<<2|1<<3|1<<4|1<<5|1<<6|1<<7|1<<8; delay_us(led_delay); break;
                case 1: PTC->PCOR |= 1<<2; PTC->PSOR |= 1<<1|1<<3|1<<4|1<<5|1<<6|1<<7|1<<8; delay_us(led_delay); break;
                case 2: PTC->PCOR |= 1<<3; PTC->PSOR |= 1<<2|1<<1|1<<4|1<<5|1<<6|1<<7|1<<8; delay_us(led_delay); break;
                case 3: PTC->PCOR |= 1<<4; PTC->PSOR |= 1<<2|1<<3|1<<1|1<<5|1<<6|1<<7|1<<8; delay_us(led_delay); break;
                case 4: PTC->PCOR |= 1<<5; PTC->PSOR |= 1<<2|1<<3|1<<4|1<<1|1<<6|1<<7|1<<8; delay_us(led_delay); break;
                case 5: PTC->PCOR |= 1<<6; PTC->PSOR |= 1<<2|1<<3|1<<4|1<<5|1<<1|1<<7|1<<8; delay_us(led_delay); break;
                case 6: PTC->PCOR |= 1<<7; PTC->PSOR |= 1<<2|1<<3|1<<4|1<<5|1<<6|1<<1|1<<8; delay_us(led_delay); break;
                case 7: PTC->PCOR |= 1<<8; PTC->PSOR |= 1<<2|1<<3|1<<4|1<<5|1<<6|1<<7|1<<1; delay_us(led_delay); break;
            }
        }
        else if (External_PIN == 0) { // BREAK 모드=
            FTM0_CH1_PWM(8000);
            Seg_out(2222); // STOP 표시
        }
    }
}
