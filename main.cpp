#include <TinyGPS++.h>
#include "stm32f4xx_hal.h"

// Definição dos pinos dos sensores
#define flow_PIN1 GPIO_PIN_2  // Pino do sensor de fluxo 1
#define flow_PIN2 GPIO_PIN_3  // Pino do sensor de fluxo 2
#define flow_PIN3 GPIO_PIN_4  // Pino do sensor de fluxo 3
#define flow_PIN4 GPIO_PIN_5  // Pino do sensor de fluxo 4

#define pressure_PIN GPIO_PIN_6 // Pino do sensor de pressão
#define level_PIN GPIO_PIN_7 // Pino do sensor de nível

#define trigPin1 GPIO_PIN_8
#define echoPin1 GPIO_PIN_9
#define trigPin2 GPIO_PIN_10
#define echoPin2 GPIO_PIN_11
#define trigPin3 GPIO_PIN_12
#define echoPin3 GPIO_PIN_13

#define flowRateConverter 0.078201369 // Conversão para fluxo em L/min
#define pressureRateConverter 0.006739775 // Conversão para pressão em Bar

#define UPPER_LIMIT 18.5 // Limite superior para leitura dos sensores de fluxo L/min
#define LOWER_LIMIT 17.0 // Limite inferior para leitura dos sensores de fluxo L/min

#define pulseDuration 0.0034 // Tempo de viagem do som no ar em cm

#define RXPin PA3
#define TXPin PA2

static const uint32_t GPSBaud = 9600;   // Baudrate do módulo GPS

UART_HandleTypeDef huart2;
TinyGPSPlus gps;

void SystemClock_Config(void);
static void MX_USART2_UART_Init(void);
static void MX_GPIO_Init(void);

void setup() {
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();

  Serial.begin(9600);
}

void loop() {
  // Leitura dos sensores de fluxo
  float flow1 = (map(analogRead(flow_PIN1), 0, 1023, 0, 255)) * flowRateConverter; 
  float flow2 = (map(analogRead(flow_PIN2), 0, 1023, 0, 255)) * flowRateConverter; 
  float flow3 = (map(analogRead(flow_PIN3), 0, 1023, 0, 255)) * flowRateConverter; 
  float flow4 = (map(analogRead(flow_PIN4), 0, 1023, 0, 255)) * flowRateConverter;

  // Verificação dos limites dos sensores de fluxo
  if (flow1 > UPPER_LIMIT || flow1 < LOWER_LIMIT) {
    Serial.print("flow out of limit");
  } else {
    Serial.print("Flow Sensor 01 = ");
    Serial.print(flow1);
  }
  delay(1000); // espera 1s

  if (flow2 > UPPER_LIMIT || flow2 < LOWER_LIMIT) {
    Serial.print("flow out of limit");
  } else {
    Serial.print("Flow Sensor 02 = ");
    Serial.print(flow2);
  }
  delay(1000); // espera 1s

  if (flow3 > UPPER_LIMIT || flow3 < LOWER_LIMIT) {
    Serial.print("flow out of limit");
  } else {
    Serial.print("Flow Sensor 03 = ");
    Serial.print(flow3);
  }
  delay(1000); // espera 1s

  if (flow4 > UPPER_LIMIT || flow4 < LOWER_LIMIT) {
    Serial.print("flow out of limit");
  } else {
    Serial.print("Flow Sensor 04 = ");
    Serial.print(flow4);
  }
  delay(1000); // espera 1s

  // Leitura e impressão do sensor de pressão
  float pressure = (map(analogRead(pressure_PIN), 0, 1023, 0, 255)) * pressureRateConverter;
  Serial.print("Pressure = ");
  Serial.print(pressure);
  delay(1000); // espera 1s

  // Leitura e impressão do sensor de nível
  float level = map(analogRead(level_PIN), 0, 100, 0, 255);
  Serial.print("Level = ");
  Serial.print(level);
  delay(1000); // espera 1s

  // Sensores de localização usando ultrassom
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);
  long duration1 = pulseIn(echoPin1, HIGH);
  long distance1 = duration1 * pulseDuration / 2;

  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);
  long duration2 = pulseIn(echoPin2, HIGH);
  long distance2 = duration2 * pulseDuration / 2;

  digitalWrite(trigPin3, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin3, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin3, LOW);
  long duration3 = pulseIn(echoPin3, HIGH);
  long distance3 = duration3 * pulseDuration / 2;

  Serial.print("Distance from Sensor 1: ");
  Serial.print(distance1);
  Serial.println(" cm");

  Serial.print("Distance from Sensor 2: ");
  Serial.print(distance2);
  Serial.println(" cm");

  Serial.print("Distance from Sensor 3: ");
  Serial.print(distance3);
  Serial.println(" cm");

  // Cálculo de posição relativa
  if (distance1 < distance2 && distance1 < distance3) {
    Serial.println("Near Sensor 1");
  } else if (distance2 < distance1 && distance2 < distance3) {
    Serial.println("Near Sensor 2");
  } else if (distance3 < distance1 && distance3 < distance2) {
    Serial.println("Near Sensor 3");
  } else {
    Serial.println("No position");
  }

  float x = (distance1 * distance1 - distance2 * distance2 + 10000) / 200;
  float y = (distance1 * distance1 - distance3 * distance3 + 10000) / 200;

  Serial.print("Relative Position (x, y): ");
  Serial.print(x);
  Serial.print(", ");
  Serial.println(y);
  delay(1000);

  // Leitura e processamento do GPS
  while (Serial.available() > 0) {
    if (gps.encode(Serial.read())) {
      if (gps.location.isValid()) {
        double latitude = gps.location.lat();
        double longitude = gps.location.lng();

        Serial.print("Latitude: ");
        Serial.println(latitude, 6);
        Serial.print("Longitude: ");
        Serial.println(longitude, 6);
      } else {
        Serial.println("GPS signal not valid");
      }
    }
  }
}

// Inicialização da UART
static void MX_USART2_UART_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK) {
    Custom_Error_Handler();
  }
}

// Inicialização dos GPIOs
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // Ativar o relógio dos GPIOs
  __HAL_RCC_GPIOA_CLK_ENABLE();

  // Configuração dos pinos dos sensores
  GPIO_InitStruct.Pin = flow_PIN1 | flow_PIN2 | flow_PIN3 | flow_PIN4 | pressure_PIN | level_PIN | trigPin1 | trigPin2 | trigPin3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = echoPin1 | echoPin2 | echoPin3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

// Função de tratamento de erros
void Custom_Error_Handler() {
  while (1) {
    // Loop infinito para indicar erro
  }
}

// Configuração do relógio do sistema
void SystemClock_Config(void) {
  // Configuração do relógio do sistema
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Custom_Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
    Custom_Error_Handler();
  }
}