/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct __attribute__((packed)) {
    uint8_t sof;
    float temperature;
    float humidity;
    float pressure;
    float voc;
    float co2;
    float lux;
    uint8_t crc;
    uint8_t eof;
} datapacket_t;

typedef struct {
    double par_t1, par_t2, par_t3, par_p1, par_p2, par_p3, par_p4, par_p5,
		par_p6, par_p7, par_p8, par_p9, par_p10, par_p11;
} bmp_calib_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PACKET_SOF 0x55
#define PACKET_EOF 0xff

#define BMP_ADDRESS 0x76
#define MICS_ADDRESS 0x70
#define HIH_ADDRESS 0x27

#define MICS_CMD_GET_STATUS 0x0C
#define MICS_VOC_VALUE 0
#define MICS_CO2_VALUE 1
#define MICS_RAW_VALUES 2
#define MICS_ERROR 5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define BE_TO_LE(x) (x >> 8) | ((x & 0xff) << 8)  // Swap bytes

#define likely(x)       __builtin_expect(!!(x), 1)
#define unlikely(x)     __builtin_expect(!!(x), 0)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1; // ESP
UART_HandleTypeDef huart2; // Debug

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

static uint8_t calcCRC(uint8_t* buffer, size_t size) {
	uint16_t sum = 0;
	for (uint8_t i = 0; i < size; i++)
		sum += buffer[i];

	uint8_t crc = sum & 0xff;
	crc += (sum / 0x0100); 	// Add with carry
	crc = 0xFF-crc; 		// Complement

	return crc;
}

// BMP390
static inline HAL_StatusTypeDef BMP_WriteRegister(uint8_t reg, uint8_t data) {
	return HAL_I2C_Mem_Write(&hi2c1, BMP_ADDRESS << 1, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
}

static inline HAL_StatusTypeDef BMP_ConfigSensor(void) {
	HAL_StatusTypeDef ret;

	// Soft reset the sensor
	ret = BMP_WriteRegister(0xE0, 0b10110110);
	if (unlikely(ret != HAL_OK))
		return ret;
	HAL_Delay(20);

	// Use 16X oversampling for pressure measurement, 2X for temp
	ret = BMP_WriteRegister(0x1c, 0b00001100);
	if (unlikely(ret != HAL_OK))
		return ret;

	// Use IIR filter coefficient 15
	ret = BMP_WriteRegister(0x1f, 0b00001000);
	if (unlikely(ret != HAL_OK))
		return ret;

	// 25Hz sample rate
	ret = BMP_WriteRegister(0x1d, 0b00000111);
	if (unlikely(ret != HAL_OK))
		return ret;

	// Normal mode, enable pressure and temperature
	ret = BMP_WriteRegister(0x1b, 0b00110011);

	return ret;
}

static inline HAL_StatusTypeDef BMP_GetCalibrationData(bmp_calib_t* calib) {
	uint8_t buf[21];
	HAL_StatusTypeDef ret;

	ret = HAL_I2C_Mem_Read(
		&hi2c1,
		BMP_ADDRESS << 1,
		0x31,
		I2C_MEMADD_SIZE_8BIT,
		buf,
		21,
		1000
	);
	if (unlikely(ret != HAL_OK))
		return ret;

	/* 1 / 2^8 */
	calib->par_t1 = (buf[1] << 8) | buf[0];
	calib->par_t1 /= 0.00390625f;
	calib->par_t2 = (buf[3] << 8) | buf[2];
	calib->par_t2 /= 1073741824.0f;
	calib->par_t3 = buf[4];
	calib->par_t3 /= 281474976710656.0f;
	calib->par_p1 = (buf[6] << 8) | buf[5];
	calib->par_p1 -= 16384.0f;
	calib->par_p1 /= 1048576.0f;
	calib->par_p2 = (buf[8] << 8) | buf[7];
	calib->par_p2 -= 16384.0f;
	calib->par_p2 /= 536870912.0f;
	calib->par_p3 = buf[9];
	calib->par_p3 /= 4294967296.0f;
	calib->par_p4 = buf[10];
	calib->par_p4 /= 137438953472.0f;

	/* 1 / 2^3 */
	calib->par_p5 = (buf[12] << 8) | buf[11];
	calib->par_p5 /= 0.125f;
	calib->par_p6 = (buf[14] << 8) | buf[13];
	calib->par_p6 /= 64.0f;
	calib->par_p7 = buf[15];
	calib->par_p7 /= 256.0f;
	calib->par_p8 = buf[16];
	calib->par_p8 /= 32768.0f;
	calib->par_p9 = (buf[18] << 8) | buf[17];
	calib->par_p9 /= 281474976710656.0f;
	calib->par_p10 = buf[19];
	calib->par_p10 /= 281474976710656.0f;
	calib->par_p11 = buf[20];
	calib->par_p11 /= 36893488147419103232.0f;

	return HAL_OK;
}

static double BMP_CompensateTemperature(uint32_t uncomp_temp, bmp_calib_t* calib) {
	double partial_data1;
	double partial_data2;

	partial_data1 = (double)uncomp_temp - calib->par_t1;
	partial_data2 = partial_data1 * calib->par_t2;
	double t_lin = partial_data2 + (partial_data1 * partial_data1) * calib->par_t3;

	return t_lin;
}

static double BMP_CompensatePressure(uint32_t uncomp_press, double comp_temp, bmp_calib_t* calib) {
	// Variable to store the compensated pressure
	double comp_press;

	// Temporary variables used for compensation
	double partial_data1;
	double partial_data2;
	double partial_data3;
	double partial_data4;
	double partial_out1;
	double partial_out2;

	// Calibration data
	partial_data1 = calib->par_p6 * comp_temp;
	partial_data2 = calib->par_p7 * (comp_temp * comp_temp);
	partial_data3 = calib->par_p8 * (comp_temp * comp_temp * comp_temp);
	partial_out1 = calib->par_p5 + partial_data1 + partial_data2 + partial_data3;
	partial_data1 = calib->par_p2 * comp_temp;
	partial_data2 = calib->par_p3 * (comp_temp * comp_temp);
	partial_data3 = calib->par_p4 * (comp_temp * comp_temp * comp_temp);
	partial_out2 = (double)uncomp_press * (calib->par_p1 + partial_data1 + partial_data2 + partial_data3);
	partial_data1 = (double)uncomp_press * (double)uncomp_press;
	partial_data2 = calib->par_p9 + calib->par_p10 * comp_temp;
	partial_data3 = partial_data1 * partial_data2;
	partial_data4 = partial_data3 + ((double)uncomp_press * (double)uncomp_press * (double)uncomp_press) * calib->par_p11;
	comp_press = partial_out1 + partial_out2 + partial_data4;

	return comp_press;
}

static HAL_StatusTypeDef BMP_ReadData(uint32_t* raw_temp, uint32_t* raw_press) {
	uint8_t buffer[6];
	HAL_StatusTypeDef ret;

	// Fetch data
	ret = HAL_I2C_Mem_Read(
		&hi2c1,
		BMP_ADDRESS << 1,
		0x04,
		I2C_MEMADD_SIZE_8BIT,
		buffer,
		6,
		1000
	);
	if (unlikely(ret != HAL_OK))
		return ret;

	*raw_press = buffer[0] | (buffer[1] << 8) | (buffer[2] << 16);
	*raw_temp = buffer[3] | (buffer[4] << 8) | (buffer[5] << 16);

	return HAL_OK;
}

// HIH7000
static inline void HIH_SendWakeup(void) {
	HAL_StatusTypeDef ret;

	ret = HAL_I2C_IsDeviceReady(&hi2c1, HIH_ADDRESS << 1, 1, 1000);
	if (unlikely(ret != HAL_OK)) {
		// This is not supposed to happen
		// Communication between MCU and HIH7120 is lost
		puts("\x1B[91mERROR: HIH7120 unreachable, check wiring!\x1B[0m\r");
		Error_Handler();
	}
}

static HAL_StatusTypeDef HIH_ReadData(float* hum, float* temp) {
	uint8_t buffer[4];
	HAL_StatusTypeDef ret;

	// Fetch data
	ret = HAL_I2C_Master_Receive(
		&hi2c1,
		HIH_ADDRESS << 1,
		buffer,
		4,
		1000
	);
	if (unlikely(ret != HAL_OK))
	  return ret;

	uint8_t status = buffer[0] >> 6;
	if (unlikely(status >= 2))
		return HAL_ERROR;	// Invalid flag combination

	if (status == 0x00) {
		// New sample, update data
		uint16_t raw_humidity = (uint16_t)buffer[1] | (((uint16_t)buffer[0] & 0x3f) << 8);
		uint16_t raw_temperature = ((uint16_t)buffer[3] >> 2) | (uint16_t)(buffer[2] << 6);
		*hum = (float)raw_humidity / ((1<<14) - 2.0f) * 100.0f;
		*temp = (float)raw_temperature / ((1<<14) - 2.0f) * 165.0f - 40.0f;
	} else {
		// Either old sample or invalid flag combination
		if (likely(status == 0x01)) {
			puts("WARN: Read old sample from HIH7120, data not updated\r");
		} else {
			puts("ERROR: HIH7120 status invalid\r");
			return HAL_ERROR;
		}
	}
	return HAL_OK;
}

static inline HAL_StatusTypeDef MICS_SendCmd(uint8_t cmd) {
	uint8_t cmd_frame[6] = { cmd, 0x00, 0x00, 0x00, 0x00, 0x00 };

	cmd_frame[5] = calcCRC(cmd_frame, 5); // CRC includes command and 4 databytes, 6th byte is CRC
	return HAL_I2C_Master_Transmit(&hi2c1, MICS_ADDRESS << 1, cmd_frame, 6, 1000);
}

static HAL_StatusTypeDef MICS_ReadData(float* voc, float* co2) {
	uint8_t buffer[7];
	HAL_StatusTypeDef ret;

	// Fetch data
	ret = HAL_I2C_Master_Receive(&hi2c1, MICS_ADDRESS << 1, buffer, 7, 1000);
	if (unlikely(ret != HAL_OK))
		return ret;

	// Check frame CRC
	if (calcCRC(buffer, 6) != buffer[6])
		return HAL_ERROR; // Checksum error

	// Calculate CO2 and VOC
	*voc = ((float)buffer[MICS_VOC_VALUE] - 13.0f) * (1000.0f / 229.0f);
	*co2 = ((float)buffer[MICS_CO2_VALUE] - 13.0f) * (1600.0f / 229.0f) + 400.0f;

	return HAL_OK;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Required in order to use prinf-function
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  puts("STM32 starting...\r");
  HAL_Delay(2000); // Give all sensors and ESP time to start up
  puts("Checking if all I2C-sensors are reachable...\r");

  uint8_t error = 0;

  // HIH7120: temperature and rel. humidity
  if ( HAL_I2C_IsDeviceReady(&hi2c1, HIH_ADDRESS << 1, 1, 1000) != HAL_OK ) {
	  puts("HIH7120 - FAILED\r");
	  error = -1;
  } else {
	  puts("HIH7120 - OK\r");
  }

  // BMP390: pressure and temperature
  if ( HAL_I2C_IsDeviceReady(&hi2c1, BMP_ADDRESS << 1, 1, 1000) != HAL_OK ) {
	  puts("BMP390 - FAILED\r");
  	  error = -1;
  } else {
  	  puts("BMP390 - OK\r");
  }

  // MiCS-VZ-89TE: CO2 and VOC level
  if ( HAL_I2C_IsDeviceReady(&hi2c1, MICS_ADDRESS << 1, 1, 1000) != HAL_OK ) {
	  puts("MiCS-VZ-89TE - FAILED\r");
		  error = -1;
  } else {
	  puts("MiCS-VZ-89TE - OK\r");
  }

  if (error) {
	  // Critical error - print in (bright) red
	  puts("\x1B[91mERROR: Not all I2C-sensors are reachable, check wiring!\x1B[0m\r");
	  Error_Handler();
  } else {
	  puts("All I2C-sensors reachable\r");
  }

  // Init BMP390
  bmp_calib_t bmp_calib_data;

  puts("Configuring BMP390...\r");
  if (unlikely(BMP_ConfigSensor() != HAL_OK)) {
	  // Critical error - print in (bright) red
	  puts("\x1B[91mERROR: Failed to configure BMP390\x1B[0m\r");
	  Error_Handler();
  }

  puts("Reading BMP390's calibration data...\r");
  if (unlikely(BMP_GetCalibrationData(&bmp_calib_data) != HAL_OK)) {
	  // Critical error - print in (bright) red
  	  puts("\x1B[91mERROR: Failed to read BMP390's calibration data\x1B[0m\r");
  	  Error_Handler();
  }

  // Ready to start sampling data
  datapacket_t sample;
  sample.sof = PACKET_SOF;
  sample.eof = PACKET_EOF;

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
  puts("Starting to sample data...\r");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	// Prepare reading
	HAL_ADC_Start(&hadc1);
	HIH_SendWakeup();
	if (unlikely(MICS_SendCmd(MICS_CMD_GET_STATUS) != HAL_OK)) {
		puts("WARN: Failed to send get-status command to MICS, sample will be skipped\r");
		continue;
	}
	HAL_Delay(100);

	// Fetch data
	float voc, co2, humidity, temperature, pressure, lux;
	uint32_t bmp_raw_pressure, bmp_raw_temperature;
	double bmp_comp_temperature;

	if (unlikely(HAL_ADC_PollForConversion(&hadc1, 50) != HAL_OK)) {
		puts("WARN: Failed to read ADC, sample will be skipped\r");
		continue;
	}
	lux = (float)HAL_ADC_GetValue(&hadc1) * 0.244140625f;

	if (unlikely(MICS_ReadData(&voc, &co2) != HAL_OK)) {
		puts("WARN: Failed to read data from MICS, sample will be skipped\r");
		continue;
	}
	if (unlikely(HIH_ReadData(&humidity, &temperature) != HAL_OK)) {
		puts("WARN: Failed to read data from HIH7120, sample will be skipped\r");
		continue;
	}

	if (unlikely(BMP_ReadData(&bmp_raw_temperature, &bmp_raw_pressure) != HAL_OK)) {
		puts("WARN: Failed to read data from BMP390, sample will be skipped\r");
		continue;
	}
	bmp_comp_temperature = BMP_CompensateTemperature(bmp_raw_temperature, &bmp_calib_data);
	pressure = (float)BMP_CompensatePressure(bmp_raw_pressure, bmp_comp_temperature, &bmp_calib_data);

	printf("Temp BMP: %ld\n\r", (long)bmp_comp_temperature);
	printf("Temp HIH: %ld\n\r", (long)temperature);
	printf("Pressure: %ld\n\r", (long)pressure);
	printf("Humidity: %ld\n\r", (long)humidity);
	printf("CO2: %ld\n\r", (long)co2);
	printf("VOC: %ld\n\r", (long)voc);
	printf("Lux: %ld\n\r", (long)lux);

	// Get average temperature
	//temperature += (float)bmp_comp_temperature;
	//temperature /= 2;

	// Prepare transmission to ESP
	sample.voc = voc;
	sample.co2 = co2;
	sample.humidity = humidity;
	sample.temperature = temperature;
	sample.pressure = pressure;
	sample.lux = lux;
	sample.crc = calcCRC((void *)&sample, sizeof(datapacket_t) - 2);

	// Transmit sample to ESP
	puts("Sending sample to ESP\r");
	HAL_UART_Transmit(&huart1, (void *)&sample, sizeof(datapacket_t), HAL_MAX_DELAY);

	HAL_Delay(900);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  huart2.Init.BaudRate = 115200;
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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
  while (1) {
	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	  // Systick interrupt is disabled, so HAL_Delay won't work
	  // Executing NOP's to mimic delay
	  for (volatile uint32_t i = 0; i < 750000; i++);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
