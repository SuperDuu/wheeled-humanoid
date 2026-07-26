/**
  ******************************************************************************
  * @file           : drv8353.h
  * @brief          : Header for DRV8353RS Gate Driver SPI Library
  ******************************************************************************
  */

#ifndef __DRV8353_H__
#define __DRV8353_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"

/* DRV8353 Register Addresses */
#define DRV8353_REG_FAULT_STATUS_1   0x00
#define DRV8353_REG_VGS_STATUS_2     0x01
#define DRV8353_REG_DRIVER_CONTROL   0x02
#define DRV8353_REG_GATE_DRIVE_HS    0x03
#define DRV8353_REG_GATE_DRIVE_LS    0x04
#define DRV8353_REG_OCP_CONTROL      0x05
#define DRV8353_REG_CSA_CONTROL      0x06

/* CSA Gain Options */
typedef enum {
    DRV8353_CSA_GAIN_5V   = 0x00,
    DRV8353_CSA_GAIN_10V  = 0x01,
    DRV8353_CSA_GAIN_20V  = 0x02,
    DRV8353_CSA_GAIN_40V  = 0x03
} DRV8353_CSA_Gain_t;

/* DRV8353 Handle Structure */
typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *cs_port;
    uint16_t           cs_pin;
    GPIO_TypeDef      *en_port;
    uint16_t           en_pin;
    DRV8353_CSA_Gain_t csa_gain;
} DRV8353_t;

/* Function Prototypes */
HAL_StatusTypeDef DRV8353_Init(DRV8353_t *drv, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin, GPIO_TypeDef *en_port, uint16_t en_pin);
void DRV8353_Enable(DRV8353_t *drv);
void DRV8353_Disable(DRV8353_t *drv);
HAL_StatusTypeDef DRV8353_ReadRegister(DRV8353_t *drv, uint8_t regAddr, uint16_t *regVal);
HAL_StatusTypeDef DRV8353_WriteRegister(DRV8353_t *drv, uint8_t regAddr, uint16_t regVal);
HAL_StatusTypeDef DRV8353_SetCSAGain(DRV8353_t *drv, DRV8353_CSA_Gain_t gain);
HAL_StatusTypeDef DRV8353_ReadFaults(DRV8353_t *drv, uint16_t *fault1, uint16_t *fault2);

#ifdef __cplusplus
}
#endif

#endif /* __DRV8353_H__ */
