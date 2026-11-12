/**
  * @file    co5300.c
  * @brief   CO5300 AMOLED driver (QSPI, RGB565) for STM32U5 OCTOSPI1.
  *
  * QSPI framing used by the CO5300 (datasheet 5.2):
  *   register write : [02h] [00 CMD 00] [params...]         all on 1 wire
  *   register read  : [03h] [00 CMD 00] [data...]           all on 1 wire
  *   pixel write    : [32h] [00 2C 00] [pixels...]          cmd/addr 1 wire, pixels 4 wires
  * RGB565 pixels go out high byte first.
  *
  * All pixels of one window are sent in ONE frame (chip select stays low for
  * the whole stream), which is what the panel expects. The data is pushed
  * into the OCTOSPI FIFO by polling; if the CPU is late the peripheral simply
  * pauses the clock.
  */
#include "co5300.h"
#include <string.h>

extern OSPI_HandleTypeDef hospi1;

/* QSPI instruction codes */
#define QSPI_INST_WRITE         0x02U
#define QSPI_INST_READ          0x03U
#define QSPI_INST_PIXEL_4WIRE   0x32U

/* CO5300 commands */
#define CMD_SWRESET             0x01U
#define CMD_RDDID               0x04U
#define CMD_SLPIN               0x10U
#define CMD_SLPOUT              0x11U
#define CMD_DISPOFF             0x28U
#define CMD_DISPON              0x29U
#define CMD_CASET               0x2AU
#define CMD_RASET               0x2BU
#define CMD_RAMWR               0x2CU
#define CMD_TEON                0x35U
#define CMD_COLMOD              0x3AU
#define CMD_RAMWRC              0x3CU
#define CMD_WRDISBV             0x51U
#define CMD_WRCTRLD             0x53U
#define CMD_WRIMGEHCCTR         0x58U
#define CMD_WRHBMDISBV          0x63U
#define CMD_SETSPIMODE          0xC4U
#define CMD_PAGE                0xFEU

#define CO5300_TIMEOUT_MS       1000U

/* Scratch buffer for byte swapped pixel data */
#define TX_BUF_PIXELS           1024U

static uint16_t s_txBuf[TX_BUF_PIXELS];
static uint32_t s_pendingBytes;         /* pixel bytes the current window still takes */
static bool     s_frameOpen;            /* a pixel frame is in progress (CS low) */

static HAL_StatusTypeDef StreamAbort(void);

/* -------------------------------------------------------------- low level */

/* Configure one QSPI frame. If dataMode != NONE the frame is completed by the
   following transmit/receive call. */
static HAL_StatusTypeDef QspiCommand(uint8_t instruction, uint8_t cmd, uint32_t dataMode, uint32_t nbData)
{
  OSPI_RegularCmdTypeDef c = {0};

  if (s_frameOpen)
  {
    (void)StreamAbort();   /* previous window was not filled completely */
  }

  c.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
  c.FlashId            = HAL_OSPI_FLASH_ID_1;
  c.Instruction        = instruction;
  c.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;
  c.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
  c.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  c.Address            = (uint32_t)cmd << 8;
  c.AddressMode        = HAL_OSPI_ADDRESS_1_LINE;
  c.AddressSize        = HAL_OSPI_ADDRESS_24_BITS;
  c.AddressDtrMode     = HAL_OSPI_ADDRESS_DTR_DISABLE;
  c.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
  c.DataMode           = dataMode;
  c.NbData             = nbData;
  c.DataDtrMode        = HAL_OSPI_DATA_DTR_DISABLE;
  c.DummyCycles        = 0U;
  c.DQSMode            = HAL_OSPI_DQS_DISABLE;
  c.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;

  return HAL_OSPI_Command(&hospi1, &c, CO5300_TIMEOUT_MS);
}

/* Write a register with 0..n parameter bytes (1-wire). */
static HAL_StatusTypeDef WriteReg(uint8_t cmd, const uint8_t *params, uint32_t len)
{
  HAL_StatusTypeDef st;

  if (len == 0U)
  {
    return QspiCommand(QSPI_INST_WRITE, cmd, HAL_OSPI_DATA_NONE, 0U);
  }

  st = QspiCommand(QSPI_INST_WRITE, cmd, HAL_OSPI_DATA_1_LINE, len);
  if (st == HAL_OK)
  {
    st = HAL_OSPI_Transmit(&hospi1, (uint8_t *)params, CO5300_TIMEOUT_MS);
  }
  return st;
}

static HAL_StatusTypeDef WriteReg8(uint8_t cmd, uint8_t value)
{
  return WriteReg(cmd, &value, 1U);
}

/* ------------------------------------------------------ pixel streaming -- */

/* Open the pixel frame: header out, then the peripheral waits for data. */
static HAL_StatusTypeDef StreamBegin(uint32_t totalBytes)
{
  HAL_StatusTypeDef st = QspiCommand(QSPI_INST_PIXEL_4WIRE, CMD_RAMWR, HAL_OSPI_DATA_4_LINES, totalBytes);

  if (st == HAL_OK)
  {
    /* same as HAL_OSPI_Transmit(): indirect write mode, data is fed below */
    MODIFY_REG(hospi1.Instance->CR, OCTOSPI_CR_FMODE, 0U);
    s_frameOpen = true;
  }
  return st;
}

/* Feed bytes of the open frame into the FIFO. */
__attribute__((optimize("O2")))
static HAL_StatusTypeDef StreamPush(const uint8_t *data, uint32_t len)
{
  OCTOSPI_TypeDef *ospi = hospi1.Instance;
  const bool wordWrites = (hospi1.Init.FifoThreshold >= 4U);   /* FTF then means >= 4 bytes free */
  uint32_t t0 = HAL_GetTick();

  while (len > 0U)
  {
    uint32_t sr = ospi->SR;

    if ((sr & OCTOSPI_SR_TEF) != 0U)
    {
      return HAL_ERROR;
    }
    if ((sr & OCTOSPI_SR_FTF) != 0U)
    {
      if (wordWrites && (len >= 4U))
      {
        uint32_t w;

        memcpy(&w, data, 4U);
        ospi->DR = w;
        data += 4U;
        len  -= 4U;
      }
      else
      {
        *((__IO uint8_t *)&ospi->DR) = *data;
        data++;
        len--;
      }
    }
    else if ((HAL_GetTick() - t0) > CO5300_TIMEOUT_MS)
    {
      return HAL_TIMEOUT;
    }
  }
  return HAL_OK;
}

/* Close the frame after its last byte: wait until everything is shifted out. */
static HAL_StatusTypeDef StreamEnd(void)
{
  uint32_t t0 = HAL_GetTick();

  while ((hospi1.Instance->SR & OCTOSPI_SR_TCF) == 0U)
  {
    if ((HAL_GetTick() - t0) > CO5300_TIMEOUT_MS)
    {
      (void)StreamAbort();
      return HAL_TIMEOUT;
    }
  }
  WRITE_REG(hospi1.Instance->FCR, OCTOSPI_FCR_CTCF);
  hospi1.State = HAL_OSPI_STATE_READY;
  s_frameOpen = false;
  return HAL_OK;
}

static HAL_StatusTypeDef StreamAbort(void)
{
  s_frameOpen    = false;
  s_pendingBytes = 0U;
  hospi1.State   = HAL_OSPI_STATE_BUSY_TX;   /* lets HAL_OSPI_Abort() do its work */
  return HAL_OSPI_Abort(&hospi1);
}

/* Send wire-order bytes belonging to the current window. */
static HAL_StatusTypeDef WriteWindowBytes(const uint8_t *data, uint32_t len)
{
  HAL_StatusTypeDef st = HAL_OK;

  if ((len == 0U) || (len > s_pendingBytes))
  {
    return HAL_ERROR;   /* more data than the window holds, or no window set */
  }
  if (!s_frameOpen)
  {
    st = StreamBegin(s_pendingBytes);
  }
  if (st == HAL_OK)
  {
    st = StreamPush(data, len);
  }
  if (st != HAL_OK)
  {
    (void)StreamAbort();
    return st;
  }
  s_pendingBytes -= len;
  if (s_pendingBytes == 0U)
  {
    st = StreamEnd();
  }
  return st;
}

/* -------------------------------------------------------------------- API */

HAL_StatusTypeDef CO5300_SetWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
  HAL_StatusTypeDef st;
  uint8_t  p[4];
  uint16_t x0 = (uint16_t)(x + AMOLED_COL_OFFSET);
  uint16_t x1 = (uint16_t)(x0 + w - 1U);
  uint16_t y0 = (uint16_t)(y + AMOLED_ROW_OFFSET);
  uint16_t y1 = (uint16_t)(y0 + h - 1U);

  if ((w == 0U) || (h == 0U) || ((x + w) > CO5300_WIDTH) || ((y + h) > CO5300_HEIGHT))
  {
    return HAL_ERROR;
  }

  p[0] = (uint8_t)(x0 >> 8); p[1] = (uint8_t)x0;
  p[2] = (uint8_t)(x1 >> 8); p[3] = (uint8_t)x1;
  st = WriteReg(CMD_CASET, p, 4U);
  if (st != HAL_OK)
  {
    return st;
  }

  p[0] = (uint8_t)(y0 >> 8); p[1] = (uint8_t)y0;
  p[2] = (uint8_t)(y1 >> 8); p[3] = (uint8_t)y1;
  st = WriteReg(CMD_RASET, p, 4U);

  s_pendingBytes = (st == HAL_OK) ? ((uint32_t)w * h * 2U) : 0U;
  return st;
}

HAL_StatusTypeDef CO5300_WritePixels(const uint16_t *pixels, uint32_t count)
{
  HAL_StatusTypeDef st = HAL_OK;

  while ((count > 0U) && (st == HAL_OK))
  {
    uint32_t n = (count > TX_BUF_PIXELS) ? TX_BUF_PIXELS : count;

    for (uint32_t i = 0U; i < n; i++)
    {
      s_txBuf[i] = (uint16_t)__REV16(pixels[i]);
    }
    st = WriteWindowBytes((const uint8_t *)s_txBuf, n * 2U);
    pixels += n;
    count  -= n;
  }
  return st;
}

HAL_StatusTypeDef CO5300_DrawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *pixels)
{
  HAL_StatusTypeDef st;

  if (((x & 1U) != 0U) || ((w & 1U) != 0U))
  {
    return HAL_ERROR;
  }
  st = CO5300_SetWindow(x, y, w, h);
  if (st == HAL_OK)
  {
    st = CO5300_WritePixels(pixels, (uint32_t)w * h);
  }
  return st;
}

HAL_StatusTypeDef CO5300_FillRect(int32_t x, int32_t y, int32_t w, int32_t h, uint16_t color)
{
  HAL_StatusTypeDef st;
  uint32_t count;
  uint32_t fill;
  uint16_t wire = (uint16_t)__REV16(color);
  int32_t  x1 = x + w;
  int32_t  y1 = y + h;

  /* clip */
  if (x < 0)  { x = 0; }
  if (y < 0)  { y = 0; }
  if (x1 > (int32_t)CO5300_WIDTH)  { x1 = (int32_t)CO5300_WIDTH; }
  if (y1 > (int32_t)CO5300_HEIGHT) { y1 = (int32_t)CO5300_HEIGHT; }
  if ((x >= x1) || (y >= y1))
  {
    return HAL_OK;
  }

  /* CO5300: start column and column count must be even */
  x  &= ~1;
  x1  = (x1 + 1) & ~1;

  st = CO5300_SetWindow((uint16_t)x, (uint16_t)y, (uint16_t)(x1 - x), (uint16_t)(y1 - y));
  if (st != HAL_OK)
  {
    return st;
  }

  count = (uint32_t)(x1 - x) * (uint32_t)(y1 - y);
  fill  = (count > TX_BUF_PIXELS) ? TX_BUF_PIXELS : count;
  for (uint32_t i = 0U; i < fill; i++)
  {
    s_txBuf[i] = wire;
  }

  while ((count > 0U) && (st == HAL_OK))
  {
    uint32_t n = (count > TX_BUF_PIXELS) ? TX_BUF_PIXELS : count;

    st = WriteWindowBytes((const uint8_t *)s_txBuf, n * 2U);
    count -= n;
  }
  return st;
}

HAL_StatusTypeDef CO5300_FillScreen(uint16_t color)
{
  return CO5300_FillRect(0, 0, (int32_t)CO5300_WIDTH, (int32_t)CO5300_HEIGHT, color);
}

HAL_StatusTypeDef CO5300_SetBrightness(uint8_t level)
{
  return WriteReg8(CMD_WRDISBV, level);
}

HAL_StatusTypeDef CO5300_DisplayOn(void)
{
  return WriteReg(CMD_DISPON, NULL, 0U);
}

HAL_StatusTypeDef CO5300_DisplayOff(void)
{
  return WriteReg(CMD_DISPOFF, NULL, 0U);
}

HAL_StatusTypeDef CO5300_AllPixelsOn(bool on)
{
  return WriteReg(on ? 0x23U : 0x13U, NULL, 0U);
}

HAL_StatusTypeDef CO5300_Sleep(void)
{
  HAL_StatusTypeDef st = CO5300_DisplayOff();

  if (st == HAL_OK)
  {
    st = WriteReg(CMD_SLPIN, NULL, 0U);
  }
  HAL_Delay(120U);
  return st;
}

HAL_StatusTypeDef CO5300_Wakeup(void)
{
  HAL_StatusTypeDef st = WriteReg(CMD_SLPOUT, NULL, 0U);

  HAL_Delay(120U);
  if (st == HAL_OK)
  {
    st = CO5300_DisplayOn();
  }
  return st;
}

/* ------------------------------------------------ bit-banged read-back -- */

static void BbDelay(void)
{
  for (volatile uint32_t i = 0U; i < 100U; i++)
  {
  }
}

static void BbPinMode(GPIO_TypeDef *port, uint16_t pin, uint32_t mode, uint32_t pull)
{
  GPIO_InitTypeDef gpio = {0};

  gpio.Pin       = pin;
  gpio.Mode      = mode;
  gpio.Pull      = pull;
  gpio.Speed     = GPIO_SPEED_FREQ_LOW;       /* keep in sync with the OCTOSPI1 pins in CubeMX */
  gpio.Alternate = GPIO_AF10_OCTOSPI1;
  HAL_GPIO_Init(port, &gpio);
}

static void BbClockPulse(void)
{
  BbDelay();
  HAL_GPIO_WritePin(LCD_CLK_PORT, LCD_CLK_PIN, GPIO_PIN_SET);
  BbDelay();
  HAL_GPIO_WritePin(LCD_CLK_PORT, LCD_CLK_PIN, GPIO_PIN_RESET);
}

void CO5300_ReadReg(uint8_t cmd, uint8_t *io0Data, uint8_t *io1Data, uint32_t len)
{
  if (s_frameOpen)
  {
    (void)StreamAbort();
  }

  const uint32_t header = ((uint32_t)QSPI_INST_READ << 24) | ((uint32_t)cmd << 8);

  /* take the pins away from OCTOSPI1 (SPI mode 0: clock idles low) */
  HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LCD_CLK_PORT, LCD_CLK_PIN, GPIO_PIN_RESET);
  BbPinMode(LCD_CS_PORT, LCD_CS_PIN, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);
  BbPinMode(LCD_CLK_PORT, LCD_CLK_PIN, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);
  BbPinMode(LCD_IO0_PORT, LCD_IO0_PIN, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);
  BbPinMode(LCD_IO1_PORT, LCD_IO1_PIN, GPIO_MODE_INPUT, GPIO_PULLDOWN);
  BbDelay();

  HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_RESET);

  /* [03h] [00 CMD 00], MSB first */
  for (int32_t bit = 31; bit >= 0; bit--)
  {
    HAL_GPIO_WritePin(LCD_IO0_PORT, LCD_IO0_PIN,
                      ((header >> bit) & 1U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    BbClockPulse();
  }

  /* release IO0, the panel now shifts data out on falling clock edges */
  BbPinMode(LCD_IO0_PORT, LCD_IO0_PIN, GPIO_MODE_INPUT, GPIO_PULLDOWN);

  for (uint32_t n = 0U; n < len; n++)
  {
    uint8_t d0 = 0U;
    uint8_t d1 = 0U;

    for (uint32_t bit = 0U; bit < 8U; bit++)
    {
      BbDelay();
      d0 = (uint8_t)((d0 << 1) | (uint8_t)HAL_GPIO_ReadPin(LCD_IO0_PORT, LCD_IO0_PIN));
      d1 = (uint8_t)((d1 << 1) | (uint8_t)HAL_GPIO_ReadPin(LCD_IO1_PORT, LCD_IO1_PIN));
      BbClockPulse();
    }
    if (io0Data != NULL) { io0Data[n] = d0; }
    if (io1Data != NULL) { io1Data[n] = d1; }
  }

  HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_SET);
  BbDelay();

  /* hand the pins back to OCTOSPI1 */
  BbPinMode(LCD_IO0_PORT, LCD_IO0_PIN, GPIO_MODE_AF_PP, GPIO_NOPULL);
  BbPinMode(LCD_IO1_PORT, LCD_IO1_PIN, GPIO_MODE_AF_PP, GPIO_NOPULL);
  BbPinMode(LCD_CLK_PORT, LCD_CLK_PIN, GPIO_MODE_AF_PP, GPIO_NOPULL);
  BbPinMode(LCD_CS_PORT, LCD_CS_PIN, GPIO_MODE_AF_PP, GPIO_NOPULL);
}

HAL_StatusTypeDef CO5300_Init(void)
{
  HAL_StatusTypeDef st;

#if defined(LCD_EN_PORT) && defined(LCD_EN_PIN)
  AMOLED_GpioOutputInit(LCD_EN_PORT, LCD_EN_PIN, GPIO_PIN_SET);
  HAL_Delay(20U);
#endif

#if defined(LCD_RST_PORT) && defined(LCD_RST_PIN)
  /* Hardware reset: >= 10 us low, then 120 ms before Sleep Out (datasheet 6.6) */
  AMOLED_GpioOutputInit(LCD_RST_PORT, LCD_RST_PIN, GPIO_PIN_SET);
  HAL_Delay(10U);
  HAL_GPIO_WritePin(LCD_RST_PORT, LCD_RST_PIN, GPIO_PIN_RESET);
  HAL_Delay(10U);
  HAL_GPIO_WritePin(LCD_RST_PORT, LCD_RST_PIN, GPIO_PIN_SET);
  HAL_Delay(120U);
#else
  st = WriteReg(CMD_SWRESET, NULL, 0U);
  if (st != HAL_OK)
  {
    return st;
  }
  HAL_Delay(120U);
#endif

  st = WriteReg(CMD_SLPOUT, NULL, 0U);
  if (st != HAL_OK)
  {
    return st;
  }
  HAL_Delay(120U);

  if (st == HAL_OK) { st = WriteReg8(CMD_PAGE, 0x00U); }         /* user command set (CMD1) */
  if (st == HAL_OK) { st = WriteReg8(CMD_SETSPIMODE, 0x80U); }   /* allow GRAM writes over SPI */
  if (st == HAL_OK) { st = WriteReg8(CMD_COLMOD, 0x55U); }       /* 16 bit/pixel RGB565 */
  if (st == HAL_OK) { st = WriteReg8(CMD_TEON, 0x00U); }         /* TE pin: V-blank pulses */
  if (st == HAL_OK) { st = WriteReg8(CMD_WRCTRLD, 0x20U); }      /* brightness control on */
  if (st == HAL_OK) { st = WriteReg8(CMD_WRHBMDISBV, 0xFFU); }   /* HBM brightness */
  if (st == HAL_OK) { st = WriteReg8(CMD_WRIMGEHCCTR, 0x00U); }  /* sunlight enhancement off */
  if (st == HAL_OK) { st = WriteReg8(CMD_WRDISBV, 0x00U); }      /* dark until GRAM is cleared */
  if (st == HAL_OK) { st = CO5300_FillScreen(CO5300_BLACK); }    /* GRAM is random after power on */
  if (st == HAL_OK) { st = CO5300_DisplayOn(); }
  HAL_Delay(10U);
  if (st == HAL_OK) { st = CO5300_SetBrightness(AMOLED_DEFAULT_BRIGHTNESS); }

  return st;
}
