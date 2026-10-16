/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ===== LoRa GCS: jembatan USB (Mission Planner) <-> E32 =====
   Parameter E32 - air rate, address, channel HARUS SAMA di GCS dan AIR.
   SPED = paritas(bit7-6) | baud UART(bit5-3) | air rate(bit2-0)
     baud UART : 0x18=9600  0x20=19200  0x28=38400  0x30=57600  0x38=115200
     air rate  : 0x02=2.4k  0x03=4.8k   0x04=9.6k   0x05=19.2k
   Air rate makin tinggi = telemetri makin lancar, tapi jangkauan makin pendek. */
#define E32_ADDH    0x00
#define E32_ADDL    0x00
#define E32_SPED    (0x30 | 0x04)   /* UART 8N1 57600 bps, air rate 9.6 kbps */
#define E32_CHAN    0x17            /* 410 + 0x17 = 433 MHz */
#define E32_OPTION  0x44            /* transparan, push-pull, FEC on, daya 30 dBm (1 W) */
#define E32_UART_BAUD  57600        /* harus sesuai bit baud di E32_SPED */

#define E32_BURST     58    /* maks. byte per kiriman ke E32 (= 1 sub-paket radio) */
#define RING_SIZE     4096  /* ukuran antrean (harus pangkat 2) */
#define USB_CHUNK     63    /* < 64 agar Windows tidak menunggu paket kosong (ZLP) */

/* Nilai e32_status (lihat di Live Expressions) */
#define E32_ST_BELUM        0
#define E32_ST_OK           1
#define E32_ST_AUX_TIMEOUT  2   /* AUX tidak pernah HIGH: cek kabel AUX / power */
#define E32_ST_NO_REPLY     3   /* tidak ada balasan: cek TX/RX tertukar, M0/M1 */
#define E32_ST_WRITE_FAIL   4   /* parameter gagal ditulis */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
extern USBD_HandleTypeDef hUsbDeviceFS;

uint8_t e32_cfg[6];               // hasil baca parameter E32 (harus C0 00 00 34 17 44)
volatile uint8_t e32_status = E32_ST_BELUM;
uint8_t e32_ver[4];               // versi modul (C3 32 xx yy, 0x32 = 433 MHz)

/* Antrean satu-produsen satu-konsumen (aman dipakai interrupt + main loop) */
typedef struct {
  uint8_t buf[RING_SIZE];
  volatile uint32_t head;   /* ditulis produsen */
  volatile uint32_t tail;   /* ditulis konsumen */
} Ring;

static Ring up;     /* uplink  : USB (Mission Planner) -> E32 */
static Ring down;   /* downlink: E32 -> USB (Mission Planner) */

uint8_t usb_tx[USB_CHUNK];        // buffer kirim USB (harus tetap utuh selama transfer)
uint8_t e32_tx[E32_BURST];        // buffer kirim ke E32

/* Statistik - pantau di Live Expressions atau terminal PA9 (115200) */
volatile uint32_t up_bytes = 0;     // byte Mission Planner -> radio
volatile uint32_t down_bytes = 0;   // byte radio -> Mission Planner
volatile uint32_t up_drop = 0;      // byte uplink dibuang karena antrean penuh
volatile uint32_t down_drop = 0;    // byte downlink dibuang karena antrean penuh
volatile uint32_t uart_overrun = 0; // overrun USART2 (seharusnya 0)
volatile uint32_t uart_noise = 0;   // byte rusak dari E32 dibuang (seharusnya 0)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Tunggu AUX HIGH (modul siap). Return 1 jika siap, 0 jika timeout. */
static uint8_t E32_WaitAux(uint32_t timeout)
{
  uint32_t t0 = HAL_GetTick();
  while (HAL_GPIO_ReadPin(E32_AUX_GPIO_Port, E32_AUX_Pin) == GPIO_PIN_RESET)
  {
    if (HAL_GetTick() - t0 > timeout) return 0;
  }
  return 1;
}

/* Buang byte sisa & flag overrun di USART2, supaya balasan E32 tidak bergeser */
static void E32_FlushRx(void)
{
  for (int i = 0; i < 64; i++)
  {
    if (!__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE) && !__HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE)) break;
    (void)huart2.Instance->DR;   /* baca SR lalu DR = clear RXNE & ORE */
  }
}

/* Ganti mode: (0,0)=normal, (1,1)=sleep/konfigurasi */
static void E32_SetMode(uint8_t m0, uint8_t m1)
{
  E32_WaitAux(1000);   /* jangan ganti mode saat modul masih sibuk */
  HAL_GPIO_WritePin(E32_M0_GPIO_Port, E32_M0_Pin, m0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(E32_M1_GPIO_Port, E32_M1_Pin, m1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_Delay(10);
  E32_WaitAux(1000);
  HAL_Delay(5);        /* datasheet: tunggu >= 2 ms setelah AUX HIGH */
  E32_FlushRx();
}

/* Kirim perintah konfigurasi, terima 6 byte balasan (diawali 0xC0) */
static uint8_t E32_Command(const uint8_t *cmd, uint16_t len, uint8_t *resp)
{
  /* E32 menolak perintah (balas FF FF FF) jika dikirim terlalu cepat setelah perintah sebelumnya */
  E32_WaitAux(1000);
  HAL_Delay(100);
  E32_FlushRx();
  HAL_UART_Transmit(&huart2, (uint8_t *)cmd, len, 100);
  if (HAL_UART_Receive(&huart2, resp, 6, 1000) != HAL_OK) return 0;
  return resp[0] == 0xC0;
}

/* Baca parameter E32; jika beda dari yang diinginkan, tulis (disimpan permanen) */
static void E32_Init(void)
{
  const uint8_t want[6] = {0xC0, E32_ADDH, E32_ADDL, E32_SPED, E32_CHAN, E32_OPTION};
  const uint8_t rd[3] = {0xC1, 0xC1, 0xC1};
  uint8_t ok = 0;

  if (!E32_WaitAux(2000))
  {
    e32_status = E32_ST_AUX_TIMEOUT;
    return;
  }

  E32_SetMode(1, 1);
  for (int i = 0; i < 3 && !ok; i++) ok = E32_Command(rd, 3, e32_cfg);

  if (!ok)
  {
    e32_status = E32_ST_NO_REPLY;
  }
  else if (memcmp(e32_cfg, want, 6) != 0)
  {
    E32_Command(want, 6, e32_cfg);
    E32_WaitAux(1000);
    HAL_Delay(50);
    ok = E32_Command(rd, 3, e32_cfg);
    e32_status = (ok && memcmp(e32_cfg, want, 6) == 0) ? E32_ST_OK : E32_ST_WRITE_FAIL;
  }
  else
  {
    e32_status = E32_ST_OK;
  }

  /* Baca versi modul */
  const uint8_t ver[3] = {0xC3, 0xC3, 0xC3};
  E32_WaitAux(1000);
  HAL_Delay(100);
  E32_FlushRx();
  HAL_UART_Transmit(&huart2, (uint8_t *)ver, 3, 100);
  for (int i = 0; i < 4; i++)
  {
    if (HAL_UART_Receive(&huart2, &e32_ver[i], 1, 300) != HAL_OK) break;
  }

  E32_SetMode(0, 0);
}

/* ---------- Antrean ---------- */
static inline uint32_t Ring_Count(const Ring *r) { return (r->head - r->tail) & (RING_SIZE - 1); }
static inline uint32_t Ring_Free(const Ring *r)  { return RING_SIZE - 1 - Ring_Count(r); }

static inline uint8_t Ring_Put(Ring *r, uint8_t b)
{
  uint32_t next = (r->head + 1) & (RING_SIZE - 1);
  if (next == r->tail) return 0;   /* penuh */
  r->buf[r->head] = b;
  r->head = next;
  return 1;
}

/* Salin maks. n byte ke dst TANPA mengambilnya (diambil dengan Ring_Drop) */
static uint32_t Ring_Peek(const Ring *r, uint8_t *dst, uint32_t n)
{
  uint32_t cnt = Ring_Count(r), t = r->tail;
  if (n > cnt) n = cnt;
  for (uint32_t i = 0; i < n; i++) dst[i] = r->buf[(t + i) & (RING_SIZE - 1)];
  return n;
}

static inline void Ring_Drop(Ring *r, uint32_t n) { r->tail = (r->tail + n) & (RING_SIZE - 1); }

/* Dipanggil dari usbd_cdc_if.c (interrupt USB) */
void GCS_UsbReceived(const uint8_t *buf, uint32_t len)
{
  for (uint32_t i = 0; i < len; i++)
  {
    if (!Ring_Put(&up, buf[i])) up_drop++;
  }
}

uint32_t GCS_UplinkFree(void) { return Ring_Free(&up); }

/* Pindah USART2 ke baud kerja + aktifkan interrupt terima (setelah konfigurasi E32 di 9600) */
static void E32_StartBridge(void)
{
  huart2.Init.BaudRate = E32_UART_BAUD;
  if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
  E32_FlushRx();
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
}

static uint8_t Usb_Ready(void)
{
  return hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED;
}
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

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(100);
  E32_Init();

  // Konfigurasi gagal -> LED kedip cepat 2 detik sebagai tanda
  if (e32_status != E32_ST_OK)
  {
    for (int i = 0; i < 20; i++)
    {
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      HAL_Delay(100);
    }
  }

  E32_StartBridge();

  uint32_t last_burst = 0;    // waktu kiriman terakhir ke E32
  uint32_t last_act = 0;      // waktu ada lalu lintas data (untuk LED)
  uint32_t last_report = 0;   // waktu laporan statistik ke PA9
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */
	  uint32_t now = HAL_GetTick();

	  // 1) Uplink: Mission Planner -> E32.
	  //    Kirim hanya saat AUX HIGH (buffer E32 kosong), maks. 1 sub-paket per kali.
	  //    Jeda 3 ms memberi waktu AUX turun setelah kiriman sebelumnya.
	  if (Ring_Count(&up) > 0 && now - last_burst >= 3 &&
	      HAL_GPIO_ReadPin(E32_AUX_GPIO_Port, E32_AUX_Pin) == GPIO_PIN_SET)
	  {
	      uint32_t n = Ring_Peek(&up, e32_tx, E32_BURST);
	      HAL_UART_Transmit(&huart2, e32_tx, n, 50);
	      Ring_Drop(&up, n);
	      up_bytes += n;
	      last_burst = last_act = HAL_GetTick();
	  }
	  CDC_ResumeRx_FS();   // lanjutkan terima USB bila antrean sudah longgar

	  // 2) Downlink: E32 -> Mission Planner (data diisi oleh interrupt USART2)
	  if (Ring_Count(&down) > 0 && Usb_Ready())
	  {
	      uint32_t n = Ring_Peek(&down, usb_tx, USB_CHUNK);
	      if (CDC_Transmit_FS(usb_tx, n) == USBD_OK)
	      {
	          Ring_Drop(&down, n);
	          down_bytes += n;
	          last_act = now;
	      }
	  }

	  // 3) LED: nyala saat ada data lewat; kedip lambat jika konfigurasi E32 gagal
	  if (e32_status != E32_ST_OK)
	      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, (now / 250) & 1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
	  else
	      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, (now - last_act < 30) ? GPIO_PIN_RESET : GPIO_PIN_SET);

	  // 4) Laporan statistik tiap 2 detik ke PA9 (115200) - TIDAK lewat USB agar MAVLink bersih
	  if (now - last_report >= 2000)
	  {
	      last_report = now;
	      char line[128];
	      int len = snprintf(line, sizeof(line),
	              "[GCS] st=%u cfg=%02X %02X %02X %02X %02X %02X up=%lu down=%lu drop=%lu/%lu ovr=%lu noise=%lu\r\n",
	              e32_status, e32_cfg[0], e32_cfg[1], e32_cfg[2], e32_cfg[3], e32_cfg[4], e32_cfg[5],
	              up_bytes, down_bytes, up_drop, down_drop, uart_overrun, uart_noise);
	      HAL_UART_Transmit(&huart1, (uint8_t *)line, len, 20);
	  }
	/* USER CODE END 3 */
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, E32_M0_Pin|E32_M1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : E32_M0_Pin E32_M1_Pin */
  GPIO_InitStruct.Pin = E32_M0_Pin|E32_M1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : E32_AUX_Pin */
  GPIO_InitStruct.Pin = E32_AUX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(E32_AUX_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* Interrupt USART2: setiap byte dari E32 langsung masuk antrean downlink.
   CATATAN: jangan centang "USART2 global interrupt" di CubeMX, karena handler ini
   sudah ditulis manual di sini (bisa bentrok dengan yang dibuat CubeMX). */
void USART2_IRQHandler(void)
{
  uint32_t sr = USART2->SR;
  if (sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE))
  {
    uint8_t b = (uint8_t)USART2->DR;   /* baca SR lalu DR = clear semua flag */
    if (sr & USART_SR_ORE) uart_overrun++;
    if (sr & (USART_SR_FE | USART_SR_NE)) uart_noise++;   /* byte rusak -> buang */
    else if (sr & USART_SR_RXNE)
    {
      if (!Ring_Put(&down, b)) down_drop++;
    }
  }
}
/* USER CODE END 4 */

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
