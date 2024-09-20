/* USER CODE BEGIN Header */
/**
  * @file           : main.c
  * @brief          : Main program body with PID control, sensor integration, and logging
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h" // STM32 HAL kütüphanesi
#include "main.h" // Ana başlık dosyası
#include "i2c.h" // I2C iletişim protokolü için başlık dosyası
#include "i2s.h" // I2S iletişim protokolü için başlık dosyası
#include "spi.h" // SPI iletişim protokolü için başlık dosyası
#include "tim.h" // Timer modülü için başlık dosyası
#include "usart.h"  // UART iletişimi için başlık dosyası
#include "usb_host.h" // USB Host işlemleri için başlık dosyası
#include "gpio.h" // GPIO işlemleri için başlık dosyası
#include <string.h>  // String işlemleri için standart kütüphane
#include <stdlib.h> // Standart kütüphane (sayısal işlemler için)

/* Private variables ---------------------------------------------------------*/
char receivedCommand[7]; // Gelen komut için 6 byte + 1 byte checksum

// PID parametreleri ve değişkenleri
float Kp = 1.0f;  // Orantısal kazanç
float Ki = 0.0f;  // İntegral kazanç
float Kd = 0.0f;  // Türevsel kazanç

float previousError = 0.0f;
float integral = 0.0f;

uint16_t targetSpeed = 50;  // Hedef hız setpoint
uint16_t currentSpeed = 0;   // Ölçülen motor hızı

// Sensör entegrasyonu
volatile uint32_t encoderCount = 0;  // Motor pozisyonu için encoder sayısı
uint16_t encoderResolution = 1024;   // Encoder çözünürlüğü

// Motor ve ADC parametreleri
uint16_t maxMotorSpeed = 100;  // Maksimum motor hızı
uint16_t currentLimit = 10;    // Maksimum izin verilen akım
#define ADC_RESOLUTION 4096.0f  // 12-bit ADC için örnek çözünürlük
#define ADC_MAX_VALUE 4095.0f  // 12-bit ADC için maksimum değer
#define CURRENT_SENSOR_GAIN 1.0f  // Akım sensörü kazancı

extern ADC_HandleTypeDef hadc1;  // ADC yapı tanımlaması

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_USB_HOST_Process(void);
void setMotorPWM(uint16_t dutyCycle);
void processCommand(char* command);
void emergencyStop(void);
void measureMotorCurrent(void);
uint8_t calculateChecksum(uint8_t* data, uint8_t length);
int validateChecksum(uint8_t* data, uint8_t length);
int isEmergencyButtonPressed(void);
void logError(const char* message);
void MX_ADC1_Init(void); // Prototipi burada tanımla

// assert_failed fonksiyonu
void assert_failed(uint8_t *file, uint32_t line)
{
    // Hata durumu için sonsuz döngü
    while (1)
    {
        // Hata durumu işlemleri burada yapılabilir
    }
}

// Error_Handler fonksiyonu
void Error_Handler(void)
{
    // Hata durumu için sonsuz döngü
    while (1)
    {
        // Hata durumunda yapılacak işlemler burada olabilir
    }
}

// SystemClock_Config fonksiyonu
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    // HSI'yi başlat
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16; // HSI 16 MHz
    RCC_OscInitStruct.PLL.PLLN = 336; // 336 MHz
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2; // 168 MHz
    RCC_OscInitStruct.PLL.PLLQ = 7; // 48 MHz
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    // Saat kaynağını ve saat ağaçlarını ayarla
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

// PID Kontrol Fonksiyonu
uint16_t PID_Control(uint16_t setpoint, uint16_t measuredSpeed) {
    float error = setpoint - measuredSpeed;
    integral += error;
    float derivative = error - previousError;
    float output = (Kp * error) + (Ki * integral) + (Kd * derivative);

    previousError = error;

    // Çıkışı geçerli PWM aralığında tut
    if (output > 1000) {
        output = 1000;
    } else if (output < 0) {
        output = 0;
    }

    return (uint16_t)output;
}

// Encoder interrupt handler
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {  // TIM3'ün encoder girişine ayarlandığını varsayalım
        encoderCount++;  // Her pulse'ta encoder sayısını artır
    }
}

// Hata kaydetme fonksiyonu
void logError(const char* message) {
    HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
}

void setMotorPWM(uint16_t dutyCycle) {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, dutyCycle);
}

void processCommand(char* command) {
    if (strlen(command) == 6 && command[5] == 'F') {
        if (command[0] == 'S') {
            char direction = command[1];
            char duty[4];
            strncpy(duty, &command[2], 3);
            duty[3] = '\0';
            int dutyCycle = atoi(duty);

            if (dutyCycle >= 0 && dutyCycle <= 100) {
                uint16_t pidOutput = PID_Control(targetSpeed, currentSpeed);

                if (direction == '1') {
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
                } else if (direction == '2') {
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
                } else {
                    logError("A1050F: Geçersiz yön.\r\n");
                    setMotorPWM(0);
                    return;
                }
                setMotorPWM(pidOutput);  // PID çıkışına göre PWM ayarla
            } else {
                logError("A1050F: Geçersiz duty cycle.\r\n");
                setMotorPWM(0);
            }
        } else if (strncmp(command, "S1000F", 6) == 0 || strncmp(command, "S2000F", 6) == 0) {
            setMotorPWM(0);
        } else {
            logError("A1050F: Geçersiz mesaj formatı.\r\n");
            setMotorPWM(0);
        }
    } else {
        logError("A1050F: Geçersiz mesaj formatı.\r\n");
        setMotorPWM(0);
    }
}

void emergencyStop(void) {
    setMotorPWM(0);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);  // Motoru devre dışı bırak
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);  // Motoru devre dışı bırak
    logError("Acil durdurma aktif.\r\n");
}

void measureMotorCurrent(void) {
    HAL_ADC_Start(&hadc1);  // ADC dönüşümünü başlat
    if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
        uint16_t currentSensorValue = HAL_ADC_GetValue(&hadc1);  // Akım sensöründen ADC okuması
        HAL_ADC_Stop(&hadc1);  // ADC dönüşümünü durdur

        float current = (float)currentSensorValue * ADC_RESOLUTION / ADC_MAX_VALUE * CURRENT_SENSOR_GAIN;
        if (current > currentLimit) {
            logError("Aşırı akım tespit edildi.\r\n");
            emergencyStop();
        }
    }
}

uint8_t calculateChecksum(uint8_t* data, uint8_t length) {
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < length; i++) {
        checksum += data[i];
    }
    return checksum;
}

int validateChecksum(uint8_t* data, uint8_t length) {
    uint8_t receivedChecksum = data[length];
    uint8_t calculatedChecksum = calculateChecksum(data, length);
    return receivedChecksum == calculatedChecksum;
}

int isEmergencyButtonPressed(void) {
    return HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);  // GPIO_PIN_13 acil durdurma butonunu varsayalım
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        if (isEmergencyButtonPressed()) {
            emergencyStop();
        } else {
            if (validateChecksum((uint8_t*)receivedCommand, 6)) {
                processCommand(receivedCommand);
            } else {
                logError("Checksum hatası.\r\n");
                setMotorPWM(0);
            }
        }
        HAL_UART_Receive_IT(&huart2, (uint8_t*)receivedCommand, 7);
    }
}

/**
  * @brief  Uygulama giriş noktası.
  * @retval int
  */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_I2S3_Init();
    MX_SPI1_Init();
    MX_USB_HOST_Init();
    MX_TIM2_Init();
    MX_USART2_UART_Init();
    MX_ADC1_Init();  // ADC başlatıldı

    HAL_UART_Receive_IT(&huart2, (uint8_t*)receivedCommand, 7);

    while (1)
    {
        MX_USB_HOST_Process();
        measureMotorCurrent();  // Motor akımını sürekli izle
    }
}
