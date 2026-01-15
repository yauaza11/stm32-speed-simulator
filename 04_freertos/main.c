/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
//#include "usart.h"
//#include "gpio.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"  // 추가! (세마포어 쓰려면 필수)
//#include "cmsis_os.h" // 이건 맨 마지막에
#include "timers.h"
#include "portable.h"
#include "stm32f1xx.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
QueueHandle_t xADCQueue;

TaskHandle_t buttonTaskHandle;
TaskHandle_t adcTaskHandle;
TaskHandle_t xLedTaskHandle;
TaskHandle_t xPWMTaskHandle;
//TaskHandle_t xLedTaskHandle = NULL;

SemaphoreHandle_t xButtonSem;
//SemaphoreHandle_t xStopModeMutex;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define SystemCoreClock 72000000


#define RCC_APB2ENR (*(volatile unsigned int*) 0x40021018)
#define RCC_APB1ENR (*(volatile unsigned int*) 0x4002101C)
#define RCC_CFGR	(*(volatile unsigned int*) 0x40021004)
#define RCC_AHBENR	(*(volatile unsigned int*) 0x40021014)

#define GPIOA 		(*(volatile unsigned int*) 0x40010800)
#define GPIOB 		(*(volatile unsigned int*) 0x40010C00)
#define GPIOC 		(*(volatile unsigned int*) 0x40011000)
#define GPIOD 		(*(volatile unsigned int*) 0x40011400)

#define ADC1 		(*(volatile unsigned int*) 0x40012400)
#define ADC2 		(*(volatile unsigned int*) 0x40012800)

#define GPIOA_CRL 	(*(volatile unsigned int*)0x40010800)
#define GPIOA_CRH 	(*(volatile unsigned int*)0x40010804)
#define GPIOA_IDR 	(*(volatile unsigned int*)0x40010808)
#define GPIOA_ODR 	(*(volatile unsigned int*)0x4001080C)
#define GPIOA_BSRR 	(*(volatile unsigned int*)0x40010810)
#define GPIOA_BRR 	(*(volatile unsigned int*)0x40010814)

#define GPIOB_CRL 	(*(volatile unsigned int*)0x40010C00)
#define GPIOB_CRH 	(*(volatile unsigned int*)0x40010C04)
#define GPIOB_IDR 	(*(volatile unsigned int*)0x40010C08)
#define GPIOB_ODR 	(*(volatile unsigned int*)0x40010C0C)
#define GPIOB_BSRR 	(*(volatile unsigned int*)0x40010C10)
#define GPIOB_BRR 	(*(volatile unsigned int*)0x40010C14)

#define GPIOC_CRL 	(*(volatile unsigned int*)0x40011000)
#define GPIOC_CRH 	(*(volatile unsigned int*)0x40011004)
#define GPIOC_IDR 	(*(volatile unsigned int*)0x40011008)
#define GPIOC_ODR 	(*(volatile unsigned int*)0x4001100C)
#define GPIOC_BSRR 	(*(volatile unsigned int*)0x40011010)
#define GPIOC_BRR 	(*(volatile unsigned int*)0x40011014)

#define GPIOD_CRL 	(*(volatile unsigned int*)0x40011400)
#define GPIOD_CRH 	(*(volatile unsigned int*)0x40011404)
#define GPIOD_IDR 	(*(volatile unsigned int*)0x40011408)
#define GPIOD_ODR 	(*(volatile unsigned int*)0x4001140C)
#define GPIOD_BSRR 	(*(volatile unsigned int*)0x40011410)
#define GPIOD_BRR 	(*(volatile unsigned int*)0x40011414)

#define ADC1_CR2 	(*(volatile unsigned int*) 0x40012408)
#define ADC1_CR1 	(*(volatile unsigned int*) 0x40012404)
#define ADC1_SQR1 	(*(volatile unsigned int*) 0x4001242C)
#define ADC1_SQR2 	(*(volatile unsigned int*) 0x40012430)
#define ADC1_SQR3 	(*(volatile unsigned int*) 0x40012434)
#define ADC1_DR 	(*(volatile unsigned int*) 0x4001244C)
#define ADC1_SMPR2 	(*(volatile unsigned int*) 0x40012410)
#define ADC1_SR 	(*(volatile unsigned int*)0x40012400)

#define TIM2		(*(volatile unsigned int*) 0x40000000)
#define TIM2_PSC 	(*(volatile unsigned int*) 0x40000028)
#define TIM2_ARR 	(*(volatile unsigned int*) 0x4000002C)
#define TIM2_CCR2 	(*(volatile unsigned int*) 0x40000038)
#define TIM2_CCER 	(*(volatile unsigned int*) 0x40000020)
#define TIM2_CCMR1 	(*(volatile unsigned int*) 0x40000018)
#define TIM2_CR1 	(*(volatile unsigned int*) 0x40000000)
#define TIM2_EGR 	(*(volatile unsigned int*) 0x40000014)
#define TIM2_SR 	(*(volatile unsigned int*) 0x40000010)
#define TIM2_DIER	(*(volatile unsigned int*) 0x4000000C)

#define TIM3 		(*(volatile unsigned int*) 0x40000400)
#define TIM3_PSC 	(*(volatile unsigned int*) 0x40000428)
#define TIM3_ARR 	(*(volatile unsigned int*) 0x4000042C)
#define TIM3_CCR2 	(*(volatile unsigned int*) 0x40000438)
#define TIM3_CCER 	(*(volatile unsigned int*) 0x40000420)
#define TIM3_CCMR1 	(*(volatile unsigned int*) 0x40000418)
#define TIM3_CR1 	(*(volatile unsigned int*) 0x40000400)
#define TIM3_CR2 	(*(volatile unsigned int*) 0x40000404)
#define TIM3_EGR 	(*(volatile unsigned int*) 0x40000414)
#define TIM3_DIER	(*(volatile unsigned int*) 0x4000040C)

#define TIM4		(*(volatile unsigned int*) 0x40000800)
#define TIM4_PSC 	(*(volatile unsigned int*) 0x40000828)
#define TIM4_ARR 	(*(volatile unsigned int*) 0x4000082C)
#define TIM4_CCR2 	(*(volatile unsigned int*) 0x40000838)
#define TIM4_CCER 	(*(volatile unsigned int*) 0x40000820)
#define TIM4_CCMR1 	(*(volatile unsigned int*) 0x40000818)
#define TIM4_CR1 	(*(volatile unsigned int*) 0x40000800)
#define TIM4_EGR 	(*(volatile unsigned int*) 0x40000814)
#define TIM4_SR 	(*(volatile unsigned int*) 0x40000810)
#define TIM4_CR1	(*(volatile unsigned int*) 0x40000800)
#define TIM4_CR2	(*(volatile unsigned int*) 0x40000804)
#define TIM4_DIER	(*(volatile unsigned int*) 0x4000080C)
#define TIM4_CNT	(*(volatile unsigned int*) 0x40000824)

#define EXTI_IMR 	(*(volatile unsigned int*) 0x40010400)
#define EXTI_EMR	(*(volatile unsigned int*) 0x40010404)
#define EXTI_RTSR	(*(volatile unsigned int*) 0x40010408)
#define EXTI_FTSR	(*(volatile unsigned int*) 0x4001040C)
#define EXTI_SWIER	(*(volatile unsigned int*) 0x40010410)
#define EXTI_PR		(*(volatile unsigned int*) 0x40010414)

#define NVIC_ISER0 (*(volatile unsigned int*) 0xE000E100)
#define NVIC_ISER1 (*(volatile unsigned int*) 0xE000E104)

#define EXTI4_PR (*(unsigned int*) 0x40010414)
#define AFIO_EXTICR2 (*(unsigned int*) 0x4001000C)

#define DMA1_ISR (*(volatile unsigned int*) 0x40020000)
#define DMA1_IFCR (*(volatile unsigned int*) 0x40020004)
#define DMA1_CCR1 (*(volatile unsigned int*) 0x40020008)
#define DMA1_CNDTR1 (*(volatile unsigned int*) 0x4002000C)
#define DMA1_CPAR1 (*(volatile unsigned int*) 0x40020010)
#define DMA1_CMAR1 (*(volatile unsigned int*) 0x40020014)
#define DMA1_CCR2 (*(volatile unsigned int*) 0x4002001C)

#define ADC_CHECK_ON()   (GPIOC_BSRR = (1<<13))
#define ADC_CHECK_OFF()  (GPIOC_BRR  = (1<<13))

#define LED_CHECK_ON()   (GPIOA_BSRR = (1 << 1))
#define LED_CHECK_OFF()  (GPIOA_BRR  = (1 << 1))

#define SEG_CHECK_ON()   (GPIOA_BSRR = (1 << 4))
#define SEG_CHECK_OFF()  (GPIOA_BRR  = (1 << 4))

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void vADCTask(void *pvParameters);
void vPWMTask(void *pvParameters);
void vLEDTask(void *pvParameters);
void vSEGTask(void *pvParameters);
void vButtonTask(void *pvParameters);
//void vInitTask(void *pvParameters);

//void seven_seg(uint16_t adc_value);
void print_segment(uint8_t number);
void print_stop(uint8_t digit);
void print_fast(uint8_t digit);
void update_led_pattern(uint8_t idx);
void setup_interrupt();
void print_char(char c);


//void vDebounceTimerCallback(TimerHandle_t xTimer);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint16_t adc_value;
volatile uint8_t btn_pressed = 0;
volatile uint8_t debounce_active = 0;
volatile uint8_t stop_mode = 0;
volatile uint8_t seg_index = 0;
volatile uint8_t adc_state = 0; // 0: normal, 1: fast, 2: stop
volatile uint8_t led_update_flag = 0;
volatile uint8_t led_tick_counter = 0;
volatile uint8_t led_delay_ticks = 100;
volatile uint8_t led_index = 0;
volatile uint16_t adc_dma_buffer[1]; // DMA 전용 버퍼

static inline uint8_t is_fast(uint16_t v) {return (v>4000);}
static inline uint8_t is_adc_stop(uint16_t v) { return (v<500);}
static inline uint16_t calc_led_delay(uint16_t v){ return 10+(4095 - v)/45; }

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
   HAL_Init();
//  HAL_SuspendTick();
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
  /* USER CODE BEGIN SysInit */
	/**************CLK Setting**************/
	RCC_APB2ENR |= (1 << 2); // GPIOA EN
	RCC_APB2ENR |= (1 << 3); // GPIOB EN
	RCC_APB2ENR |= (1 << 4); // GPIOC EN
	RCC_APB2ENR |= (1 << 5); // GPIOD EN
	RCC_AHBENR 	|= (1 << 0); // DMA1 EN


	RCC_APB1ENR |= (1 << 3); // TIM5 EN
	RCC_APB1ENR |= (1 << 0); //TIM2 EN
	RCC_APB1ENR |= (1 << 1); // TIM3 EN
	RCC_APB1ENR |= (1 << 2); //	TIM4 EN

	RCC_APB2ENR |= (1 << 0); // AFIO EN

	// ADC CLK
	RCC_APB2ENR |= (1 << 9); // ADC1 EN
	RCC_APB2ENR |= (1 << 10); // ADC2 EN

	/******************Logic Analyzer*******/
	GPIOC_CRH &= ~(0xFFF << 20);   // Clear PC13~15
	GPIOC_CRH |=  (0x111 << 20);   // Output, 10MHz

	/* PA1, PA4 Output Push-Pull, 10MHz */
	GPIOA_CRL &= ~(0xF << (1 * 4));   // PA1 Clear
	GPIOA_CRL |=  (0x1 << (1 * 4));   // Output 10MHz, PP

	GPIOA_CRL &= ~(0xF << (4 * 4));   // PA4 Clear
	GPIOA_CRL |=  (0x1 << (4 * 4));   // Output 10MHz, PP

	GPIOA_BRR = (1 << 1) | (1 << 4);
	GPIOC_BRR = (1<<13) | (1<<14) | (1<<15);

	/******************DMA*******************/

	// 3) DMA1 Channel1 config for ADC
	DMA1_CCR1 &= ~(1<<0);           // Disable DMA channel

	DMA1_CPAR1 = (uint32_t)&ADC1_DR;             // Peripheral address
	DMA1_CMAR1 = (uint32_t)adc_dma_buffer;       // Memory address
	DMA1_CNDTR1 = 1;                              // 1 sample

	DMA1_CCR1 = 0;                // reset
	DMA1_CCR1 |= (1<<5);          // CIRC
	DMA1_CCR1 |= (0b01 << 10);    // PSIZE = 16-bit
	DMA1_CCR1 |= (0b01 << 8);     // MSIZE = 16-bit
	DMA1_CCR1 &= ~(1<<4);         // DIR = 0 (P->M)
	DMA1_CCR1 &= ~(1<<7);         // MINC=0 (only 1 buffer)
	DMA1_CCR1 |= (1<<12);         // PL=high
	DMA1_CCR1 |= (1<<1);
	DMA1_CCR1 |= (1<<0);          // Enable


	/***************LEDs GPIO SET ******************/

	/*
	- **PC5 - LED_1**
	- **PB0 - LED_2**
	- **PB1 - LED_3**
	- **PB2 - LED_4**
	- **PB10 - LED_5**
	- **PB11 - LED_6**
	*/

	GPIOA_CRL &= ~(0xF << 24); // ADC PA6 SET

	GPIOC_CRL &= ~(0xF << 20); // PC5 Clear
	GPIOC_CRL |= (1<<20);
	GPIOB_CRL &= ~(0xF << 0); // PB0 Clear
	GPIOB_CRL |= (1<<0);
	GPIOB_CRL &= ~(0xF << 4); // PB1 Clear
	GPIOB_CRL |= (1<<4);
	GPIOB_CRL &= ~(0xF << 8); // PB2 Clear
	GPIOB_CRL |= (1<<8);
	GPIOB_CRH &= ~(0xF << 8); // PB10 Clear
	GPIOB_CRH |= (1<<8);
	GPIOB_CRH &= ~(0xF << 12); // PB11 Clear
	GPIOB_CRH |= (1<<12);
	/***********************************************/

	GPIOC_BRR = (1 << 5);
	GPIOB_BRR = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 10) | (1 << 11);


	/**********************7-Segment GPIO Set**************************/

	/*
		- **PA12 - DIG_A**
		- **PA11 - DIG_B**
		- **PA10 - DIG_C**
		- **PA9 - DIG_D**
		- **PA8 - DIG_E**
		- **PC9 - DIG_F**
		- **PC8 - DIG_G**
		- **PC7 - DIG_H**
		- **PC6 - DIG_1_com**
		- **PB15 - DIG_2_com**
		- **PB14 - DIG_3_com**
		- **PB13 - DIG_4_com**
	 */
	GPIOA_CRH &= ~(0xF << 16); // PA12 Clear
	GPIOA_CRH |= (1<<16);

	GPIOA_CRH &= ~(0xF << 12); // PA11 - DIG_B
	GPIOA_CRH |= (1<<12);

	GPIOA_CRH &= ~(0xF << 8); // PA10 - DIG_C
	GPIOA_CRH |= (1<<8);

	GPIOA_CRH &= ~(0xF << 4);  //PA9 - DIG_D
	GPIOA_CRH |= (1<<4);

	GPIOA_CRH &= ~(0xF << 0); // PA8 - DIG_E
	GPIOA_CRH |= (1<<0);

	GPIOC_CRH &= ~(0xF << 4); // PC9 - DIG_F
	GPIOC_CRH |= (1<<4);

	GPIOC_CRH &= ~(0xF << 0); // PC8 - DIG_G
	GPIOC_CRH |= (1<<0);

	GPIOC_CRL &= ~(0xF << 28); // PC7 - DIG_H
	GPIOC_CRL |= (1<<28);

	GPIOC_CRL &= ~(0xF << 24); // PC6 - DIG_1_com
	GPIOC_CRL |= (1<<24);

	GPIOB_CRH &= ~(0xF << 28); // PB15 - DIG_2_com
	GPIOB_CRH |= (1<<28);

	GPIOB_CRH &= ~(0xF << 24); //PB14 - DIG_3_com
	GPIOB_CRH |= (1<<24);

	GPIOB_CRH &= ~(0xF << 20); //PB13 - DIG_4_com
	GPIOB_CRH |= (1<<20);


	/**********************ADC************************/
	// 4) ADC setup
	ADC1_CR1 = 0;                    // No scan
	ADC1_CR2 |= (1<<20);             // EXTTRIG enable
	ADC1_CR2 |= (0b100 << 17);       // EXTSEL = TIM3_TRGO
	ADC1_CR2 |= (1<<8);              // DMA enable
	ADC1_CR2 &= ~(1<<1); // CONT=0
//	ADC1_CR2 |= (1<<1);              // CONT=1
	ADC1_SQR3 = 6;                   // Channel 6
	ADC1_SMPR2 |= (7 << 18);         // 239.5 cycles

	// Calibration
	ADC1_CR2 |= (1<<0);              // ADON
	for (volatile int i=0;i<10000;i++);
	ADC1_CR2 |= (1<<3);
	while (ADC1_CR2 & (1<<3));
	ADC1_CR2 |= (1<<2);
	while (ADC1_CR2 & (1<<2));

	// 5) TIM3 Trigger Output (1kHz)
	RCC_APB1ENR |= (1<<1);           // TIM3 enable
	TIM3_PSC = 72 - 1;               // 1MHz
	TIM3_ARR = 1000 - 1;             // 1 kHz
	TIM3_CR2 |= (0b010 << 4);        // MMS = 010 → TRGO on update
	TIM3_CR1 |= (1<<0);              // Enable TIM3

	// 6) Start ADC conversion
	ADC1_CR2 |= (1<<22);             // SWSTART


	/***********************PWM***************************/
	GPIOA_CRL &= ~(0xF << 28);
	GPIOA_CRL |=  (0xB << 28);   // MODE=11 (50MHz), CNF=10 (AF-PP)

	TIM3_PSC = 72 - 1;         // 1 MHz
	TIM3_ARR = 1000 - 1;
	TIM3_CCR2 = 150;

	TIM3_CCMR1 &= ~(7<<12);
	TIM3_CCMR1 |= (6<<12);     // PWM mode 1
	TIM3_CCMR1 |= (1<<11);     // CCR preload enable

	TIM3_CCMR1 |= (1<<7);      //ARR preload enable
	TIM3_CCER   |= (1<<4);     // CH2 enable
	TIM3_CR1   |=  (1<<0);        // timer enable
	/****************************************************/



	/******************************Interrupt***********************************/
	//PC5를 gpio output으로 설정
	GPIOC_CRL &= ~(0xF << 20);
	GPIOC_CRL |= (1<<20);

	//PC4 입력
	GPIOC_CRL &= ~(0xF << 16);
	GPIOC_CRL |= (2<<18);
	GPIOC_ODR  |=  (1 << 4);     // Pull-up 활성화

	//EXTI 2초기화
	AFIO_EXTICR2 &= ~(0xF<<0);
	//EXTI PC interrupt 설정
	AFIO_EXTICR2 |= (0x2<<0);

	EXTI_IMR |= (1<<4);
	EXTI_FTSR |= (1<<4);
	EXTI_RTSR &= ~(1<<4);  // Rising OFF 이게 뭔데?

	// ㅇ
	TIM4_PSC  = 7200 - 1;
	TIM4_ARR  = 100  - 1;     // ≈10ms
	TIM4_EGR |= (1<<0);       // UG (cnt/프리스케일 적용)
	TIM4_SR  &= ~1;           // UIF 클리어
	TIM4_DIER |= (1<<0);      // **UIE=1 (업데이트 인터럽트 활성)**

//	NVIC_ISER1 |= (1 << 18); // Enable TIM5 IRQ (IRQ #50)
	/***************************************************************************/

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
	//  MX_GPIO_Init();
	//  MX_USART2_UART_Init();
	  /* USER CODE BEGIN 2 */

    NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4); // 필수

	xADCQueue = xQueueCreate(1, sizeof(uint16_t)); // Queue 만들기!!
	xButtonSem = xSemaphoreCreateBinary();

	xTaskCreate(vADCTask, "ADC", 384, NULL, 2, &adcTaskHandle);
	xTaskCreate(vPWMTask, "PWM", 384, NULL, 2, NULL);
	xTaskCreate(vLEDTask, "LED", 384, NULL, 1, NULL); // 정성적 피드백이라 낮아도 됌
	xTaskCreate(vSEGTask, "7SEG", 384, NULL,2, NULL); // 정량적 피드백이기에 정확한 숫자표기를 위해 순위를 올림
	xTaskCreate(vButtonTask, "BTN", 384, NULL, 3, NULL); // 브레이크 이기때문에 1순위

//	xTaskNotifyGive(xLedTaskHandle);

	setup_interrupt();
	vTaskStartScheduler(); // 여기서 RTOS가 시작한다
  /* USER CODE END 2 */

  /* Init scheduler */
//  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
//  MX_FREERTOS_Init();

  /* Start scheduler */
//  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  GPIOC_BSRR = (1<<5); for(volatile int i=0;i<10000;i++); GPIOC_BRR = (1<<5);
	  // 여기로 들어가서 이게 실행되면 아주 큰일 난겨 RTOS로 안넘어 갔다는 뜻이거든!
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void setup_interrupt(void){

	NVIC_SetPriority(EXTI4_IRQn, 5);
	NVIC_EnableIRQ(EXTI4_IRQn);

	NVIC_SetPriority(DMA1_Channel1_IRQn, 5);
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);


	NVIC_SetPriority(TIM4_IRQn, 6);
	NVIC_EnableIRQ(TIM4_IRQn);

	NVIC_SetPriority(ADC1_2_IRQn,7);
	NVIC_EnableIRQ(ADC1_2_IRQn);

}

//void vADCTask(void *pvParameters)
//{
//    for (;;)
//    {
//    	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
//    	ADC_CHECK_ON();
//
//    	uint16_t val = adc_dma_buffer[0];
//		adc_value = val;
//
//		if (val > 4000) {
//			adc_state = 1;
//		}
//		else if (val < 500) {
//			adc_state = 2;
//		}
//		else{
//			adc_state = 0;
//		}
//		led_delay_ticks = 10 + (4095 - val) / 45;
//		ADC_CHECK_OFF();
//    }
//}

void vADCTask(void *pvParameters)
{
    for (;;)
    {
    	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    	ADC_CHECK_ON();

    	uint16_t val = adc_dma_buffer[0];
    	xQueueOverwrite(xADCQueue, &val);
    	LED_CHECK_OFF();
    }
}

void vLEDTask(void *pvParameters){

	for(;;){
		LED_CHECK_ON();

		if(stop_mode) {
			taskYIELD();
            continue;   // LED off or skip blinking
		}

		uint16_t v;
		if(xQueuePeek(xADCQueue, &v, pdMS_TO_TICKS(5)) != pdPASS){
			vTaskDelay(pdMS_TO_TICKS(10));
			continue;
		}

		if (is_fast(v) || is_adc_stop(v)) {
			vTaskDelay(pdMS_TO_TICKS(10));
			continue;
		}


		update_led_pattern(led_index);
		led_index = (led_index+1)%6;

		LED_CHECK_OFF();
		vTaskDelay(pdMS_TO_TICKS(led_delay_ticks));
	}
}

void vPWMTask(void *pvParameters)
{
    for (;;)
    {
    	if(stop_mode) {
			TIM3_CCR2 = 0;
			vTaskDelay(pdMS_TO_TICKS(5));
			taskYIELD();
			continue;
    	}
//        uint16_t val = adc_value;

//        if (adc_state == 0)
//            TIM3_CCR2 = (val * 1000) / 4095;
//        else
//            TIM3_CCR2 = 0;

    	uint16_t v;
    	if(xQueuePeek(xADCQueue, &v, pdMS_TO_TICKS(5)) == pdPASS){
    		if(is_fast(v) || is_adc_stop(v)) TIM3_CCR2 = 0;
    		else TIM3_CCR2 = (v*1000)/4095;
    	}
        vTaskDelay(pdMS_TO_TICKS(5));  // 주기적으로 PWM 갱신
    }
}

void vButtonTask(void *pvParameters)
{
    for (;;)
    {
        // 버튼 신호 기다림 (무한 대기)
        if (xSemaphoreTake(xButtonSem, portMAX_DELAY) == pdPASS)
        {
            stop_mode = !stop_mode;
        }
    }
}

void vSEGTask(void *pvParameters)
{
    static int idx = 0;

    for(;;)
    {
    	SEG_CHECK_ON();

		GPIOC_BSRR = (1<<6);
		GPIOB_BSRR = (1<<15) | (1<<14) | (1<<13);

		uint16_t v = 0;
		xQueuePeek(xADCQueue, &v ,0);

    	if(stop_mode || is_adc_stop(v)){
            switch(idx)
            {
                case 0:
                    GPIOC_BRR = (1<<6);
                    print_char('S');
                    break;

                case 1:
                    GPIOB_BRR = (1<<15);
                    print_char('T');
                    break;

                case 2:
                    GPIOB_BRR = (1<<14);
                    print_char('O');
                    break;

                case 3:
                    GPIOB_BRR = (1<<13);
                    print_char('P');
                    break;
            }
    	}
    	// Fast Mode
		else if(is_fast(v)){
			switch(idx)
				{
					case 0:
						GPIOC_BRR = (1<<6);
						print_char('F');
						break;

					case 1:
						GPIOB_BRR = (1<<15);
						print_char('A');
						break;

					case 2:
						GPIOB_BRR = (1<<14);
						print_char('S');
						break;

					case 3:
						GPIOB_BRR = (1<<13);
						print_char('T');
						break;
				}
		}

    	else{

//			uint16_t val = adc_value;
			uint8_t d0 = v / 1000;
			uint8_t d1 = (v % 1000) / 100;
			uint8_t d2 = (v % 100) / 10;
			uint8_t d3 = v % 10;

            switch(idx)
            {
                case 0:
                    GPIOC_BRR = (1<<6); // digit1 ON
                    print_segment(d0);
                    break;

                case 1:
                    GPIOB_BRR = (1<<15);
                    print_segment(d1);
                    break;

                case 2:
                    GPIOB_BRR = (1<<14);
                    print_segment(d2);
                    break;

                case 3:
                    GPIOB_BRR = (1<<13);
                    print_segment(d3);
                    break;
            }
    	}



        idx = (idx + 1) % 4;

        SEG_CHECK_OFF();
        vTaskDelay(pdMS_TO_TICKS(4));
    }
}

void DMA1_Channel1_IRQHandler(void)
{
	BaseType_t xHPW = pdFALSE;

	if(DMA1_ISR & (1<<1)){
		DMA1_IFCR |= (1<<1);
		vTaskNotifyGiveFromISR(adcTaskHandle, &xHPW);
	}
	portYIELD_FROM_ISR(xHPW);
}

void EXTI4_IRQHandler(void)
{
    EXTI_PR |= (1<<4);          // 버튼 인터럽트 발생한거 clear
    TIM4_CNT = 0;				// TIM4의 타이머 값을 clear
    TIM4_SR &= ~1;				// TIM4를 초기화
    TIM4_CR1 |= (1<<0); 		// TIM4 시작! start debounce timer
}

void TIM4_IRQHandler(void)
{
	BaseType_t xHPW = pdFALSE;
    if (TIM4_SR & 1)
    {
        TIM4_SR &= ~1;      // clear update flag
        TIM4_CR1 &= ~(1<<0); // stop timer

        // 버튼이 아직 눌려 있는지 검사!
        if (!(GPIOC_IDR & (1<<4)))
        {
            xSemaphoreGiveFromISR(xButtonSem, &xHPW);
        }

        portYIELD_FROM_ISR(xHPW);
    }
}

void update_led_pattern(uint8_t idx)
{
    GPIOC_BRR = (1<<5);
    GPIOB_BRR = (1<<0)|(1<<1)|(1<<2)|(1<<10)|(1<<11);

    switch(idx){
        case 0: GPIOC_BSRR = (1<<5); break;
        case 1: GPIOB_BSRR = (1<<0); break;
        case 2: GPIOB_BSRR = (1<<1); break;
        case 3: GPIOB_BSRR = (1<<2); break;
        case 4: GPIOB_BSRR = (1<<10); break;
        case 5: GPIOB_BSRR = (1<<11); break;
    }
}

void print_segment(uint8_t number)
{
	/*
		- **PA12 - DIG_A**
		- **PA11 - DIG_B**
		- **PA10 - DIG_C**
		- **PA9 - DIG_D**
		- **PA8 - DIG_E**
		- **PC9 - DIG_F**
		- **PC8 - DIG_G**

		- **PC7 - DIG_H**
		- **PC6 - DIG_1_com**
		- **PB15 - DIG_2_com**
		- **PB14 - DIG_3_com**
		- **PB13 - DIG_4_com**
	 */
	GPIOA_BRR = (1<<12) | (1<<11) | (1<<10) | (1<<9) | (1<<8);
	GPIOC_BRR = (1<<9) | (1<<8);

	switch(number){
		case 0:
			GPIOA_BSRR = (1<<12); // a
			GPIOA_BSRR = (1<<11); // b
			GPIOA_BSRR = (1<<10); // c
			GPIOA_BSRR = (1<<9); // d
			GPIOA_BSRR = (1<<8); // e
			GPIOC_BSRR = (1<<9); // f
			GPIOC_BRR = (1<<8); // g
			GPIOC_BRR = (1<<7);
			break;
		case 1:
			GPIOA_BRR = (1<<12); // a
			GPIOA_BSRR = (1<<11); // b
			GPIOA_BSRR = (1<<10); // c
			GPIOA_BRR = (1<<9); // d
			GPIOA_BRR = (1<<8); // e
			GPIOC_BRR = (1<<9); // f
			GPIOC_BRR = (1<<8); // g
			GPIOC_BRR = (1<<7);
			break;
		case 2:
			GPIOA_BSRR = (1<<12); // a
			GPIOA_BSRR = (1<<11); // b
			GPIOA_BRR = (1<<10); // c
			GPIOA_BSRR = (1<<9); // d
			GPIOA_BSRR = (1<<8); // e
			GPIOC_BRR = (1<<9); // f
			GPIOC_BSRR = (1<<8); // g
			GPIOC_BRR = (1<<7);
			break;
		case 3:
			GPIOA_BSRR = (1<<12); // a
			GPIOA_BSRR = (1<<11); // b
			GPIOA_BSRR = (1<<10); // c
			GPIOA_BSRR = (1<<9); // d
			GPIOA_BRR = (1<<8); // e
			GPIOC_BRR = (1<<9); // f
			GPIOC_BSRR = (1<<8); // g
			GPIOC_BRR = (1<<7);
			break;
		case 4:
			GPIOA_BRR = (1<<12); // a
			GPIOA_BSRR = (1<<11); // b
			GPIOA_BSRR = (1<<10); // c
			GPIOA_BRR = (1<<9); // d
			GPIOA_BRR = (1<<8); // e
			GPIOC_BSRR = (1<<9); // f
			GPIOC_BSRR = (1<<8); // g
			GPIOC_BRR = (1<<7);
			break;
		case 5:
			GPIOA_BSRR = (1<<12); // a
			GPIOA_BRR = (1<<11); // b
			GPIOA_BSRR = (1<<10); // c
			GPIOA_BSRR = (1<<9); // d
			GPIOA_BRR = (1<<8); // e
			GPIOC_BSRR = (1<<9); // f
			GPIOC_BSRR = (1<<8); // g
			GPIOC_BRR = (1<<7);
			break;
		case 6:
			GPIOA_BSRR = (1<<12); // a
			GPIOA_BRR = (1<<11); // b
			GPIOA_BSRR = (1<<10); // c
			GPIOA_BSRR = (1<<9); // d
			GPIOA_BSRR = (1<<8); // e
			GPIOC_BSRR = (1<<9); // f
			GPIOC_BSRR = (1<<8); // g
			GPIOC_BRR = (1<<7);
			break;
		case 7:
			GPIOA_BSRR = (1<<12); // a
			GPIOA_BSRR = (1<<11); // b
			GPIOA_BSRR = (1<<10); // c
			GPIOA_BRR = (1<<9); // d
			GPIOA_BRR = (1<<8); // e
			GPIOC_BSRR = (1<<9); // f
			GPIOC_BRR = (1<<8); // g
			GPIOC_BRR = (1<<7);
			break;
		case 8:
			GPIOA_BSRR = (1<<12); // a
			GPIOA_BSRR = (1<<11); // b
			GPIOA_BSRR = (1<<10); // c
			GPIOA_BSRR = (1<<9); // d
			GPIOA_BSRR = (1<<8); // e
			GPIOC_BSRR = (1<<9); // f
			GPIOC_BSRR = (1<<8); // g
			GPIOC_BRR = (1<<7);
			break;
		case 9:
			GPIOA_BSRR = (1<<12); // a
			GPIOA_BSRR = (1<<11); // b
			GPIOA_BSRR = (1<<10); // c
			GPIOA_BSRR = (1<<9); // d
			GPIOA_BRR = (1<<8); // e
			GPIOC_BSRR = (1<<9); // f
			GPIOC_BSRR = (1<<8); // g
			GPIOC_BRR = (1<<7);
			break;
	}
}

void print_fast(uint8_t digit)
{
    uint8_t letters[4] = {'F','A','S','T'};

    //모든 자리 끄기
    GPIOC_BSRR = (1<<6);
    GPIOB_BSRR = (1<<15)|(1<<14)|(1<<13);

    //모든 segment 끄기
    GPIOA_BRR = (1<<12)|(1<<11)|(1<<10)|(1<<9)|(1<<8);
    GPIOC_BRR = (1<<9)|(1<<8);

	switch(digit)
	{
		case 0: GPIOC_BRR = (1<<6); break;   // DIG_1 ON
		case 1: GPIOB_BRR = (1<<15); break;  // DIG_2 ON
		case 2: GPIOB_BRR = (1<<14); break;  // DIG_3 ON
		case 3: GPIOB_BRR = (1<<13); break;  // DIG_4 ON
	}

	// 글자별 segment 패턴 표시
	switch(letters[digit])
	{
		case 'F':
			GPIOA_BSRR = (1<<12); // a
			GPIOA_BRR = (1<<11); // b
			GPIOA_BRR = (1<<10); // c
			GPIOA_BRR = (1<<9); // d
			GPIOA_BSRR = (1<<8); // e
			GPIOC_BSRR = (1<<9); // f
			GPIOC_BSRR = (1<<8); // g
			GPIOC_BRR = (1<<7);
			break; // a,f,g
		case 'A':
			GPIOA_BSRR = (1<<12); // a
			GPIOA_BSRR = (1<<11); // b
			GPIOA_BSRR = (1<<10); // c
			GPIOA_BRR = (1<<9); // d
			GPIOA_BSRR = (1<<8); // e
			GPIOC_BSRR = (1<<9); // f
			GPIOC_BSRR = (1<<8); // g
			GPIOC_BRR = (1<<7);
			break;
		case 'S':
			GPIOA_BSRR = (1<<12); // a
			GPIOA_BRR = (1<<11); // b
			GPIOA_BSRR = (1<<10); // c
			GPIOA_BSRR = (1<<9); // d
			GPIOA_BRR = (1<<8); // e
			GPIOC_BSRR = (1<<9); // f
			GPIOC_BSRR = (1<<8); // g
			GPIOC_BRR = (1<<7);
			break;
		case 'T':
			GPIOA_BSRR = (1<<12); // a
			GPIOA_BSRR = (1<<11); // b
			GPIOA_BSRR = (1<<10); // c
			GPIOA_BRR = (1<<9); // d
			GPIOA_BRR = (1<<8); // e
			GPIOC_BRR = (1<<9); // f
			GPIOC_BRR = (1<<8); // g
			GPIOC_BRR = (1<<7);
			break;
	}
}


void print_stop(uint8_t digit)
{
    uint8_t letters[4] = {'S','T','O','P'};

	//전체 off
	GPIOC_BSRR = (1<<6);
	GPIOB_BSRR = (1<<15)|(1<<14)|(1<<13);
	GPIOA_BRR = (1<<12)|(1<<11)|(1<<10)|(1<<9)|(1<<8);
	GPIOC_BRR = (1<<9)|(1<<8);

	//자리선택
	switch(digit)
	{
		case 0: GPIOC_BRR = (1<<6); break;
		case 1: GPIOB_BRR = (1<<15); break;
		case 2: GPIOB_BRR = (1<<14); break;
		case 3: GPIOB_BRR = (1<<13); break;
	}

	//글자 세그먼트 표시
	switch(letters[digit])
	{
		case 'S':
			GPIOA_BSRR = (1<<12); // a
			GPIOA_BRR = (1<<11); // b
			GPIOA_BSRR = (1<<10); // c
			GPIOA_BSRR = (1<<9); // d
			GPIOA_BRR = (1<<8); // e
			GPIOC_BSRR = (1<<9); // f
			GPIOC_BSRR = (1<<8); // g
			GPIOC_BRR = (1<<7);
			break;
		case 'T':
			GPIOA_BSRR = (1<<12); // a
			GPIOA_BSRR = (1<<11); // b
			GPIOA_BSRR = (1<<10); // c
			GPIOA_BRR = (1<<9); // d
			GPIOA_BRR = (1<<8); // e
			GPIOC_BRR = (1<<9); // f
			GPIOC_BRR = (1<<8); // g
			GPIOC_BRR = (1<<7);
			break;
		case 'O':
			GPIOA_BSRR = (1<<12); // a
			GPIOA_BSRR = (1<<11); // b
			GPIOA_BSRR = (1<<10); // c
			GPIOA_BSRR = (1<<9); // d
			GPIOA_BSRR = (1<<8); // e
			GPIOC_BSRR = (1<<9); // f
			GPIOC_BRR = (1<<8); // g
			GPIOC_BRR = (1<<7);
			break;
		case 'P':
			GPIOA_BSRR = (1<<12); // a
			GPIOA_BSRR = (1<<11); // b
			GPIOA_BRR = (1<<10); // c
			GPIOA_BRR = (1<<9); // d
			GPIOA_BSRR = (1<<8); // e
			GPIOC_BSRR = (1<<9); // f
			GPIOC_BSRR = (1<<8); // g
			GPIOC_BRR = (1<<7);
			break;
	}
}

void print_char(char c)
{

    GPIOA_BRR = (1<<12) | (1<<11) | (1<<10) | (1<<9) | (1<<8);
    GPIOC_BRR = (1<<9) | (1<<8) | (1<<7);
    switch(c)
    {
        case 'S':
			GPIOA_BSRR = (1<<12); // a
			GPIOA_BRR = (1<<11); // b
			GPIOA_BSRR = (1<<10); // c
			GPIOA_BSRR = (1<<9); // d
			GPIOA_BRR = (1<<8); // e
			GPIOC_BSRR = (1<<9); // f
			GPIOC_BSRR = (1<<8); // g
			GPIOC_BRR = (1<<7);
        	break;
        case 'T':
			GPIOA_BSRR = (1<<12); // a
			GPIOA_BSRR = (1<<11); // b
			GPIOA_BSRR = (1<<10); // c
			GPIOA_BRR = (1<<9); // d
			GPIOA_BRR = (1<<8); // e
			GPIOC_BRR = (1<<9); // f
			GPIOC_BRR = (1<<8); // g
			GPIOC_BRR = (1<<7);
        	break;
        case 'O':
			GPIOA_BSRR = (1<<12); // a
			GPIOA_BSRR = (1<<11); // b
			GPIOA_BSRR = (1<<10); // c
			GPIOA_BSRR = (1<<9); // d
			GPIOA_BSRR = (1<<8); // e
			GPIOC_BSRR = (1<<9); // f
			GPIOC_BRR = (1<<8); // g
			GPIOC_BRR = (1<<7);
        	break;
        case 'P':
			GPIOA_BSRR = (1<<12); // a
			GPIOA_BSRR = (1<<11); // b
			GPIOA_BRR = (1<<10); // c
			GPIOA_BRR = (1<<9); // d
			GPIOA_BSRR = (1<<8); // e
			GPIOC_BSRR = (1<<9); // f
			GPIOC_BSRR = (1<<8); // g
			GPIOC_BRR = (1<<7);
        	break;
        case 'F':
			GPIOA_BSRR = (1<<12); // a
			GPIOA_BRR = (1<<11); // b
			GPIOA_BRR = (1<<10); // c
			GPIOA_BRR = (1<<9); // d
			GPIOA_BSRR = (1<<8); // e
			GPIOC_BSRR = (1<<9); // f
			GPIOC_BSRR = (1<<8); // g
			GPIOC_BRR = (1<<7);
			break; // a,f,g
		case 'A':
			GPIOA_BSRR = (1<<12); // a
			GPIOA_BSRR = (1<<11); // b
			GPIOA_BSRR = (1<<10); // c
			GPIOA_BRR = (1<<9); // d
			GPIOA_BSRR = (1<<8); // e
			GPIOC_BSRR = (1<<9); // f
			GPIOC_BSRR = (1<<8); // g
			GPIOC_BRR = (1<<7);
			break;
    }
}

// Idle Task 메모리 제공
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
                                    StackType_t **ppxIdleTaskStackBuffer,
                                    uint32_t *pulIdleTaskStackSize )
{
    static StaticTask_t xIdleTaskTCB;
    static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
    *ppxIdleTaskStackBuffer = xIdleStack;
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

// Timer Task 메모리 제공
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer,
                                     StackType_t **ppxTimerTaskStackBuffer,
                                     uint32_t *pulTimerTaskStackSize )
{
    static StaticTask_t xTimerTaskTCB;
    static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
    *ppxTimerTaskStackBuffer = xTimerStack;
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}


/* ========== FreeRTOS Hook Functions ========== */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;

    // 스택 오버플로 발생 시 LED 빠르게 점멸
    // 두번째 LED
    while (1)
    {
    	GPIOB_BSRR = (1<<0);

        for (volatile int i = 0; i < 10000; i++);
        GPIOB_BRR = (1<<0);
        for (volatile int i = 0; i < 10000; i++);
    }
}

void vApplicationMallocFailedHook(void)
{
    // 힙 메모리 부족 시 LED 느리게 점멸
    while (1)
    {
    	// 세번째 LED
    	GPIOB_BSRR = (1<<1);
        for (volatile int i = 0; i < 150000; i++);
        GPIOB_BRR = (1<<1);
        for (volatile int i = 0; i < 150000; i++);
    }
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
	    GPIOC_BSRR = (1 << 5);
	    for (volatile int i = 0; i < 200000; i++);
	    GPIOC_BRR = (1 << 5);
	    for (volatile int i = 0; i < 200000; i++);
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
