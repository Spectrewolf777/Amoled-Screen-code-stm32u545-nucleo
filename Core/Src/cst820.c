/**
  * @file    cst820.c
  * @brief   CST820 capacitive touch controller driver (I2C1, polled).
  */
#include "cst820.h"

extern I2C_HandleTypeDef hi2c1;

#define CST820_ADDR_8BIT        (CST820_I2C_ADDR << 1)
#define CST820_TIMEOUT_MS       20U

/* registers */
#define REG_GESTURE             0x01U   /* 01h gesture, 02h finger count, 03h..06h XH XL YH YL */
#define REG_CHIP_ID             0xA7U
#define REG_DIS_AUTO_SLEEP      0xFEU

HAL_StatusTypeDef CST820_ReadChipID(uint8_t *id)
{
  return HAL_I2C_Mem_Read(&hi2c1, CST820_ADDR_8BIT, REG_CHIP_ID, I2C_MEMADD_SIZE_8BIT,
                          id, 1U, CST820_TIMEOUT_MS);
}

HAL_StatusTypeDef CST820_Init(void)
{
  HAL_StatusTypeDef st;
  uint8_t value = 0xFFU;

#if defined(TP_RST_PORT) && defined(TP_RST_PIN)
  AMOLED_GpioOutputInit(TP_RST_PORT, TP_RST_PIN, GPIO_PIN_SET);
  HAL_Delay(5U);
  HAL_GPIO_WritePin(TP_RST_PORT, TP_RST_PIN, GPIO_PIN_RESET);
  HAL_Delay(10U);
  HAL_GPIO_WritePin(TP_RST_PORT, TP_RST_PIN, GPIO_PIN_SET);
#endif
  HAL_Delay(100U);

  st = HAL_I2C_IsDeviceReady(&hi2c1, CST820_ADDR_8BIT, 3U, CST820_TIMEOUT_MS);
  if (st != HAL_OK)
  {
    return st;
  }

  /* Without this the controller dozes off after a few seconds without touch
     and stops answering I2C polls. */
  return HAL_I2C_Mem_Write(&hi2c1, CST820_ADDR_8BIT, REG_DIS_AUTO_SLEEP, I2C_MEMADD_SIZE_8BIT,
                           &value, 1U, CST820_TIMEOUT_MS);
}

HAL_StatusTypeDef CST820_Read(CST820_Touch *touch)
{
  HAL_StatusTypeDef st;
  uint8_t  buf[6];
  uint16_t x;
  uint16_t y;

  touch->pressed = false;
  touch->gesture = CST820_GESTURE_NONE;

  st = HAL_I2C_Mem_Read(&hi2c1, CST820_ADDR_8BIT, REG_GESTURE, I2C_MEMADD_SIZE_8BIT,
                        buf, sizeof(buf), CST820_TIMEOUT_MS);
  if (st != HAL_OK)
  {
    return st;
  }

  touch->gesture = buf[0];
  if ((buf[1] & 0x0FU) == 0U)
  {
    return HAL_OK;
  }

  x = (uint16_t)(((uint16_t)(buf[2] & 0x0FU) << 8) | buf[3]);
  y = (uint16_t)(((uint16_t)(buf[4] & 0x0FU) << 8) | buf[5]);

#if TP_SWAP_XY
  { uint16_t t = x; x = y; y = t; }
#endif
  if (x >= AMOLED_WIDTH)  { x = AMOLED_WIDTH - 1U; }
  if (y >= AMOLED_HEIGHT) { y = AMOLED_HEIGHT - 1U; }
#if TP_INVERT_X
  x = (uint16_t)(AMOLED_WIDTH - 1U - x);
#endif
#if TP_INVERT_Y
  y = (uint16_t)(AMOLED_HEIGHT - 1U - y);
#endif

  touch->x = x;
  touch->y = y;
  touch->pressed = true;
  return HAL_OK;
}
