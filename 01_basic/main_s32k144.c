#include "device_registers.h"
#include "clocks_and_modes.h"
#include "ADC.h"

/* 전역 변수 설정 */
int lpit0_ch0_flag_counter = 0;
int External_PIN = -1;  // 인터럽트 상태 제어 변수 (초기값 -1) [cite: 68]
int D = 0;
uint32_t adcResultInMv = 0;
int led_delay = 0;
int i = -1;

/* LPIT 초기화 및 지연 함수 */
void LPIT0_init (uint32_t delay) {
    uint32_t timeout;
    PCC->PCCn[PCC_LPIT_INDEX] = PCC_PCCn_PCS(6); // SPLL2_DIV2_CLK [cite: 81, 83]
    PCC->PCCn[PCC_LPIT_INDEX] |= PCC_PCCn_CGC_MASK;
    LPITO->MCR |= LPIT_MCR_M_CEN_MASK; 
    
    timeout = delay * 40;
    LPITO->TMR[0].TVAL = timeout; 
    LPITO->TMR[0].TCTRL |= LPIT_TMR_TCTRL_T_EN_MASK;
}

void delay_us (volatile int us) {
    LPIT0_init(us);
    while (0 == (LPITO->MSR & LPIT_MSR_TIFO_MASK)) {} 
    LPITO->MSR |= LPIT_MSR_TIFO_MASK;
}

/* FTM (PWM) 설정: DC 모터 속도 제어용 */
void FTM_init (void) {
    PCC->PCCn[PCC_FTM0_INDEX] &= ~PCC_PCCn_CGC_MASK;
    PCC->PCCn[PCC_FTM0_INDEX] |= PCC_PCCn_PCS(0b010); // 8MHz SIRCDIV1_CLK [cite: 121, 123]
    PCC->PCCn[PCC_FTM0_INDEX] |= PCC_PCCn_CGC_MASK;
    
    FTM0->SC = FTM_SC_PWMEN1_MASK | FTM_SC_PS(0);
    FTM0->MOD = 8000 - 1; // FTM0 counter final value [cite: 136]
    FTM0->CNTIN = FTM_CNTIN_INIT(0);
    FTM0->CONTROLS[1].CnSC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSA_MASK; // Edge-aligned PWM [cite: 142, 143, 150]
}

void FTM0_CH1_PWM (int i) {
    FTM0->CONTROLS[1].CnV = i; // 8000~0 duty 제어 [cite: 158]
    FTM0->SC |= FTM_SC_CLKS(3);
}

/* 포트 초기화 (LED, 7-Segment, Switch, Motor) */
void PORT_init (void) {
    PCC->PCCn[PCC_PORTC_INDEX] |= PCC_PCCn_CGC_MASK;
    PCC->PCCn[PCC_PORTD_INDEX] |= PCC_PCCn_CGC_MASK;
    
    PORTD->PCR[16] |= PORT_PCR_MUX(2); // PTD16: FTM0CH1 (Motor) [cite: 177, 178]
    
    // 7-segment 및 LED 포트 설정 (출력)
    PTD->PDDR |= (1<<12) | (1<<2) | (1<<3) | (1<<4) | (1<<5) | (1<<6) | (1<<7);
    PTD->PDDR |= (1<<8) | (1<<9) | (1<<10) | (1<<11);
    PTC->PDDR |= 0x1FE; // C1-C8 LED [cite: 188]
    
    // 스위치 및 인터럽트 설정 (C11, C12, D15)
    PORTC->PCR[11] |= PORT_PCR_MUX(1) | (10<<16); // Falling-edge interrupt [cite: 196, 197]
    PORTC->PCR[12] |= PORT_PCR_MUX(1) | (10<<16);
    PORTD->PCR[15] |= PORT_PCR_MUX(1) | (10<<16);
}

/* 7-Segment 숫자 및 문자 표기 함수 */
void num (int nom) {
    switch(nom) {
        case 0: /* 0 표시 로직 */ break;
        // ... (중략: 1~9 숫자 로직)
        case 10: // 'S' (STOP) [cite: 290]
            PTD->PSOR |= (1<<12)|(1<<3)|(1<<4)|(1<<6)|(1<<7); 
            PTD->PCOR |= (1<<2)|(1<<5); break;
        case 11: // 'T' [cite: 298]
            PTD->PSOR |= (1<<12)|(1<<2)|(1<<3); 
            PTD->PCOR |= (1<<4)|(1<<5)|(1<<6)|(1<<7); break;
        case 13: // 'F' (FAST) [cite: 317]
            PTD->PSOR |= (1<<12)|(1<<5)|(1<<6)|(1<<7);
            PTD->PCOR |= (1<<2)|(1<<3)|(1<<4); break;
        case 14: // 'A' [cite: 325]
            PTD->PSOR |= (1<<12)|(1<<2)|(1<<3)|(1<<5)|(1<<6)|(1<<7);
            PTD->PCOR |= (1<<4); break;
    }
}

/* 7-Segment 출력 제어 (STOP, FAST, 전압수치) */
void Seg_out(int number) {
    if(number == 2222) { // 'STOP' 표시 루틴 [cite: 341, 344, 351]
        for(int j=0; j<10; j++) {
            num(10); /* S */ // ... (Digit 제어 및 delay)
            num(11); /* T */ // ...
            num(0);  /* O */ // ...
            num(12); /* P */ // ...
        }
    } else if(number == 3333) { // 'FAST' 표시 루틴 [cite: 373, 376, 382]
        for(int j=0; j<10; j++) {
            num(13); /* F */ // ...
            num(14); /* A */ // ...
            num(10); /* S */ // ...
            num(11); /* T */ // ...
        }
    } else {
        // 일반 전압 수치 표시 (d1000, d100, d10, d1 계산 후 출력) [cite: 401, 405]
    }
}

/* 인터럽트 핸들러: BREAK 기능 구현 */
void PORTC_IRQHandler(void) {
    if((PORTC->ISFR & (1<<11)) != 0) {
        External_PIN++;
        External_PIN = External_PIN % 2; // 스위치 누를 때마다 0과 1 반복 [cite: 452, 453, 61]
    }
    PORTC->PCR[11] |= 0x01000000; // ISF 플래그 클리어 [cite: 461]
}

/* 메인 루프: ADC 기반 속도 제어 및 디스플레이 */
int main(void) {
    SOSC_init_8MHz();
    SPLL_init_160MHz();
    NormalRUNmode_80MHz();
    FTM_init();
    ADC_init();
    PORT_init();
    NVIC_init_IRQs();

    for(;;) {
        convertAdcChan(12); // 가변저항 전압 읽기 [cite: 487, 497]
        while(adc_complete() == 0){}
        adcResultInMv = read_adc_chx();
        D = adcResultInMv * 1.6; // PWM 듀티비 변환 [cite: 494, 495]

        if(adcResultInMv == 5000) { // 최대 전압 시 'FAST' [cite: 500, 502]
            FTM0_CH1_PWM(8000); // 정지
            Seg_out(3333); 
        }
        else if(External_PIN == 0) { // BREAK 발생 시 'STOP' [cite: 558, 559, 563]
            FTM0_CH1_PWM(8000); // 정지
            Seg_out(2222);
        }
        else { // 정상 주행 모드 [cite: 507, 508, 509]
            FTM0_CH1_PWM(D);
            Seg_out(adcResultInMv);
            
            // LED 순차 점멸 (속도에 비례한 지연시간 설정) [cite: 510, 513]
            led_delay = 50 * (5000 - adcResultInMv); 
            i = (i + 1) % 8;
            // 각 케이스별 LED ON/OFF 로직 실행
        }
    }
}