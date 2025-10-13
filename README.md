# 🚗 Project Title(English)

## 📘 Overview
## 🧩 Features
## ⚙️ System Architecture
## 💻 Development Environment
## 🚀 Build & Run Instructions
## 📂 Repository Structure
---
# 🚗 Project Title(한국어)

## 📘 프로젝트 개요
이 프로젝트는 STM32F103 MCU를 이용해 차량의 속도 제어 시스템을 구현한 것이다.
가변저항(ADC)을 가속페달로 사용하고, push button(EXTI)을 브레이크 기능으로 사용한다. 
현재 속도는 7-Segment Display로 출력되며(0-4095), 가변저항 값에 비례해 모터의 속도(PWM)를 제어한다.
코드는 Bare-Metal C와 FreeRTOS 버전 두 가지 형태로 작성되어 있다.

## 🧩 주요기능
- 아날로그 가속 제어 (ADC): 가변저항 값을 읽어 가속 입력으로 사용
- 속도 표시 (7-Segment): ADC 값을 바탕으로 속도(0~4095 범위)를 표시
- 브레이크 기능 (EXTI 인터럽트): 버튼 입력으로 정지 모드 토글
- PWM 출력 (TIM3_CH2): ADC 값에 비례한 PWM 신호 생성
- LED 시퀀스 표시: 현재 속도 단계(레벨)를 LED로 시각화
- RTOS 버전 (선택): ADC / PWM / Display를 각각의 태스크로 분리
  
## ⚙️ System Architecture
- ADC (PA6)	가변저항 입력 전압을 읽어 가속도 계산
- PWM (PA7, TIM3_CH2)	가속도에 비례한 PWM 출력 생성
- EXTI (PC4)	버튼 인터럽트를 통해 브레이크 동작 제어
- GPIO (LED, 7-Seg)	속도 및 시각 정보 표시
  
## 💻 Development Environment
| 항목         | 내용                               |
| ---------- | -------------------------------- |
| **MCU**    | STM32F103RB (Nucleo-F103RB)      |
| **IDE**    | STM32CubeIDE 1.14.0              |
| **언어**     | C                                |
| **디버거**    | ST-Link V2                       |
| **시스템 클럭** | 72 MHz                           |
| **기타**     | FreeRTOS v10.x (RTOS 버전에서 사용 예정) |

## 🚀 Build & Run Instructions
## 📂 Repository Structure
