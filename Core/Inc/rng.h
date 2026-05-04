/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    rng.h
  * @brief   This file contains all the function prototypes for
  *          the rng.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#ifndef __RNG_H__
#define __RNG_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

#ifdef HAL_RNG_MODULE_ENABLED
extern RNG_HandleTypeDef hrng;
#endif

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef HAL_RNG_MODULE_ENABLED
void MX_RNG_Init(void);
#endif

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __RNG_H__ */

