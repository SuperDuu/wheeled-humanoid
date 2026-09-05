/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct {
  float    vbus;           /* Điện áp bus DC (V) */
  float    fet_temp_raw;   /* Điện áp NTC FET (V) */
  float    fet_temp;       /* Nhiệt độ FET (°C) */
  float    vsense_a;       /* Back-EMF pha A (V) */
  float    vsense_b;       /* Back-EMF pha B (V) */
  float    vsense_c;       /* Back-EMF pha C (V) */
  uint16_t vbus_raw;
  uint16_t fet_temp_raw_adc;
  uint16_t vsense_a_raw;
  uint16_t vsense_b_raw;
  uint16_t vsense_c_raw;

  uint16_t drv_fault1;
  uint16_t drv_fault2;
  uint8_t  drv_otw;
  uint8_t  drv_otsd;
  uint8_t  drv_has_fault;
} ADC_Readings_t;

typedef struct {
  float zero_electric_angle;  /* Góc điện offset (rad) */
  float coarse_electric_angle;/* Offset from the quasi-static sweep */
  float phase_correction;     /* Closed-loop torque-axis correction (rad) */
  float encoder_rad;          /* Góc encoder tại thời điểm lock (rad) */
  float vbus;                 /* VBUS khi alignment */
  float concentration;        /* Circular concentration of static zero samples */
  int32_t forward_counts;     /* Encoder travel during forward sweep */
  int32_t backward_counts;    /* Encoder travel during backward sweep */
  float torque_score_neg90;   /* Signed encoder response at correction -pi/2 */
  float torque_score_zero;    /* Signed encoder response at correction 0 */
  float torque_score_pos90;   /* Signed encoder response at correction +pi/2 */
  float torque_score_final;   /* Signed response after applying correction */
  float current_a;            /* Dòng pha A khi lock (A) */
  float current_b;            /* Dòng pha B khi lock (A) */
  float current_c;            /* Dòng pha C khi lock (A) */
  float vd_applied;           /* Điện áp Vd đã áp dụng (V) */
  uint16_t raw_angle;         /* Raw encoder angle */
  uint8_t aligned;            /* 1 = alignment thành công */
  int8_t enc_dir;             /* encoder_direction đang dùng */
} Align_Debug_t;

extern volatile Align_Debug_t g_dbg_align;
extern volatile ADC_Readings_t g_adc_readings;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
/* Secondary Output Link Encoder (AS5600 on I2C3) Hardware Flag:
 * 0 = Disabled (Only primary AS5048A on SPI3 is present)
 * 1 = Enabled (Secondary AS5600 on I2C3 connected to PC8/PC9) */
#define USE_AS5600_OUTPUT_ENCODER  0
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

void TIM1_EnsureMoeEnabled(void);
void Run_EncoderCalibration(void);
void Run_EncoderAlignment(void);
extern volatile int run_calibration;
extern volatile int run_alignment;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_1_Pin GPIO_PIN_14
#define LED_1_GPIO_Port GPIOC
#define LED_2_Pin GPIO_PIN_15
#define LED_2_GPIO_Port GPIOC
#define VSENSE_C_Pin GPIO_PIN_0
#define VSENSE_C_GPIO_Port GPIOC
#define VBUS_SENSE_Pin GPIO_PIN_2
#define VBUS_SENSE_GPIO_Port GPIOC
#define FET_TEMP_Pin GPIO_PIN_0
#define FET_TEMP_GPIO_Port GPIOA
#define VSENSE_A_Pin GPIO_PIN_3
#define VSENSE_A_GPIO_Port GPIOA
#define DRV_SCK_Pin GPIO_PIN_5
#define DRV_SCK_GPIO_Port GPIOA
#define DRV_MISO_Pin GPIO_PIN_6
#define DRV_MISO_GPIO_Port GPIOA
#define DRV_BKIN_Pin GPIO_PIN_10
#define DRV_BKIN_GPIO_Port GPIOB
#define CAN_RX_Pin GPIO_PIN_12
#define CAN_RX_GPIO_Port GPIOB
#define CAN_TX_Pin GPIO_PIN_13
#define CAN_TX_GPIO_Port GPIOB
#define VSENSE_B_Pin GPIO_PIN_14
#define VSENSE_B_GPIO_Port GPIOB
#define DRV_CS_Pin GPIO_PIN_6
#define DRV_CS_GPIO_Port GPIOC
#define DRV_EN_Pin GPIO_PIN_7
#define DRV_EN_GPIO_Port GPIOC
#define ENC_SCL_Pin GPIO_PIN_8
#define ENC_SCL_GPIO_Port GPIOC
#define ENC_SDA_Pin GPIO_PIN_9
#define ENC_SDA_GPIO_Port GPIOC
#define ENC_SCK_Pin GPIO_PIN_10
#define ENC_SCK_GPIO_Port GPIOC
#define ENC_MISO_Pin GPIO_PIN_11
#define ENC_MISO_GPIO_Port GPIOC
#define ENC_MOSI_Pin GPIO_PIN_12
#define ENC_MOSI_GPIO_Port GPIOC
#define DRV_MOSI_Pin GPIO_PIN_5
#define DRV_MOSI_GPIO_Port GPIOB
#define ENC_EN_Pin GPIO_PIN_6
#define ENC_EN_GPIO_Port GPIOB
#define ENC_CS_Pin GPIO_PIN_7
#define ENC_CS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
