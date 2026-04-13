#include "main.h"
#include "stm32l4xx_ll_usart.h"
#include "stm32l4xx_ll_tim.h"
#include "stm32l4xx_ll_gpio.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define RX_BUFFER_SIZE 128

volatile uint8_t rx_buffer[RX_BUFFER_SIZE];
volatile uint16_t rx_index = 0;
volatile uint8_t bluetooth_data_ready = 0;

volatile uint16_t scan_x = 1500;
volatile int16_t scan_direction_x = 1;
volatile uint8_t scan_enabled = 0;
volatile uint32_t scan_speed = 2;
volatile uint32_t scan_counter = 0;

void USART1_IRQHandler(void)
{
    if (LL_USART_IsActiveFlag_RXNE(USART1))
    {
        uint8_t rx_byte = LL_USART_ReceiveData8(USART1);
        rx_buffer[rx_index] = rx_byte;
        rx_index = (rx_index + 1) % RX_BUFFER_SIZE;
        
        if (rx_byte == '\n' || rx_byte == '\r')
        {
            bluetooth_data_ready = 1;
        }
    }
}

void TIM2_IRQHandler(void)
{
    if (LL_TIM_IsActiveFlag_UPDATE(TIM2))
    {
        LL_TIM_ClearFlag_UPDATE(TIM2);
        
        if (scan_enabled)
        {
            scan_counter++;
            
            if (scan_counter % 4 == 0)
            {
                scan_x += scan_direction_x * scan_speed;
                
                if (scan_x >= 1800)
                    scan_direction_x = -1;
                else if (scan_x <= 1200)
                    scan_direction_x = 1;
            }
            
            LL_TIM_OC_SetCompareCH1(TIM1, scan_x);
        }
        else
        {
            uint16_t center = 1500;
            if (scan_x < center) scan_x++;
            if (scan_x > center) scan_x--;
            LL_TIM_OC_SetCompareCH1(TIM1, scan_x);
        }
    }
}

void read_bluetooth_response(char* output_buffer, uint16_t max_len)
{
    uint16_t i = 0;
    while (i < max_len && i < rx_index)
    {
        output_buffer[i] = (char)rx_buffer[i];
        i++;
    }
    output_buffer[i] = '\0';
    rx_index = 0;
    bluetooth_data_ready = 0;
}

void send_bluetooth_command(const char* command)
{
    while (*command)
    {
        LL_USART_TransmitData8(USART1, (uint8_t)*command);
        while (!LL_USART_IsActiveFlag_TXE(USART1));
        command++;
    }
    LL_USART_TransmitData8(USART1, '\r');
    while (!LL_USART_IsActiveFlag_TXE(USART1));
    LL_USART_TransmitData8(USART1, '\n');
    while (!LL_USART_IsActiveFlag_TXE(USART1));
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART1_UART_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    
    LL_USART_SetBaudRate(USART1, SystemCoreClock, LL_USART_OVERSAMPLING_16, 115200);
    LL_USART_Enable(USART1);
    LL_USART_EnableIT_RXNE(USART1);
    
    NVIC_SetPriority(TIM2_IRQn, 1);
    NVIC_SetPriority(USART1_IRQn, 2);
    NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_EnableIRQ(USART1_IRQn);
    
    LL_TIM_EnableIT_UPDATE(TIM2);
    LL_TIM_EnableCounter(TIM2);
    
    LL_TIM_OC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
    LL_TIM_EnableCounter(TIM1);
    LL_TIM_OC_SetCompareCH1(TIM1, 1500);
    
    HAL_Delay(1000);
    
    send_bluetooth_command("AT");
    HAL_Delay(100);
    char response[50];
    read_bluetooth_response(response, 50);
    
    send_bluetooth_command("AT+NAME=MEMS_Mirror_ECE491");
    HAL_Delay(100);
    read_bluetooth_response(response, 50);
    
    send_bluetooth_command("AT+BAUD=115200");
    HAL_Delay(100);
    read_bluetooth_response(response, 50);
    
    send_bluetooth_command("AT+EXIT");
    HAL_Delay(100);
    read_bluetooth_response(response, 50);
    
    while (1)
    {
        if (bluetooth_data_ready)
        {
            char cmd[32];
            read_bluetooth_response(cmd, 32);
            
            if (strstr(cmd, "START"))
            {
                scan_enabled = 1;
            }
            else if (strstr(cmd, "STOP"))
            {
                scan_enabled = 0;
            }
            else if (strstr(cmd, "SPEED"))
            {
                int speed = atoi(&cmd[6]);
                if (speed > 0 && speed < 100)
                    scan_speed = speed;
            }
            else if (strstr(cmd, "QUERY"))
            {
                char telemetry[32];
                sprintf(telemetry, "X:%d,EN:%d\r\n", scan_x, scan_enabled);
                send_bluetooth_command(telemetry);
            }
        }
        
        HAL_Delay(10);
    }
}

void SystemClock_Config(void)
{
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
    while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_1)
    {
    }
    LL_RCC_MSI_Enable();
    while(LL_RCC_MSI_IsReady() != 1)
    {
    }
    LL_RCC_MSI_EnableRangeSelection();
    LL_RCC_MSI_SetRange(LL_RCC_MSIRANGE_6);
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_MSI);
    while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_MSI)
    {
    }
}

static void MX_GPIO_Init(void)
{
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    
    GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void MX_USART1_UART_Init(void)
{
    LL_USART_InitTypeDef USART_InitStruct = {0};
    
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
    
    USART_InitStruct.BaudRate = 115200;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection = LL_USART_DIR_TX_RX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
    LL_USART_Init(USART1, &USART_InitStruct);
    LL_USART_EnableIT_RXNE(USART1);
    LL_USART_Enable(USART1);
}

static void MX_TIM1_Init(void)
{
    LL_TIM_InitTypeDef TIM_InitStruct = {0};
    LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
    
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);
    
    TIM_InitStruct.Prescaler = 319;
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload = 1999;
    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    TIM_InitStruct.RepetitionCounter = 0;
    LL_TIM_Init(TIM1, &TIM_InitStruct);
    LL_TIM_DisableARRPreload(TIM1);
    
    LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);
    LL_TIM_SetTriggerOutput2(TIM1, LL_TIM_TRGO2_RESET);
    LL_TIM_DisableMasterSlaveMode(TIM1);
    
    TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
    TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
    TIM_OC_InitStruct.OCNState = LL_TIM_OCNSTATE_DISABLE;
    TIM_OC_InitStruct.CompareValue = 1500;
    TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
    TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCNPOLARITY_HIGH;
    TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
    TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCNIDLESTATE_LOW;
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
    LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH1);
    
    LL_TIM_SetCounter(TIM1, 0);
    LL_TIM_EnableAllOutputs(TIM1);
}

static void MX_TIM2_Init(void)
{
    LL_TIM_InitTypeDef TIM_InitStruct = {0};
    
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
    
    TIM_InitStruct.Prescaler = 7;
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload = 999;
    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    LL_TIM_Init(TIM2, &TIM_InitStruct);
    LL_TIM_DisableARRPreload(TIM2);
    LL_TIM_SetClockSource(TIM2, LL_TIM_CLOCKSOURCE_INTERNAL);
    LL_TIM_DisableMasterSlaveMode(TIM2);
    LL_TIM_SetCounter(TIM2, 0);
}
