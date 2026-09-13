/**
 * @file main.c
 * Minimal example of MPU-6050 driver for STM32F4xx microcontrollers.
 * This code is based on the MPU-6050 datasheet.
 * Not a full project, thus supposed to be used as a part of an existing CubeIDE project.
 */

#include "stm32f4xx.h"
#include "mpu6050.h"

static volatile uint32_t g_tick_ms = 0;
static mpu6050_t g_mpu;

void SysTick_Handler(void)
{
    g_tick_ms++;
}

static void systick_init(uint32_t core_clock_hz)
{
    // Configure SysTick to generate an interrupt every 1 ms
    SysTick_Config(core_clock_hz / 1000u);
}

// NVIC interrupt handlers for DMA and I2C events
void DMA1_Stream0_IRQHandler(void) // DMA1 Stream 0 for I2C1 RX
{
    if (DMA1->LISR & DMA_LISR_TCIF0)
    {
        DMA1->LIFCR = DMA_LIFCR_CTCIF0;
        MPU6050_DMA_RX_IRQHandler(&g_mpu);
    }
}

void DMA1_Stream6_IRQHandler(void) // DMA1 Stream 6 for I2C1 TX
{
    if (DMA1->HISR & DMA_HISR_TCIF6)
    {
        DMA1->HIFCR = DMA_HIFCR_CTCIF6;
        MPU6050_DMA_TX_IRQHandler(&g_mpu);
    }
}

void I2C1_EV_IRQHandler(void)
{
     MPU6050_I2C_EV_IRQHandler(&g_mpu);
}

void I2C1_ER_IRQHandler(void)
{
     MPU6050_I2C_ER_IRQHandler(&g_mpu);
}

int main(void)
{
    // Assuming the core clock is 84 MHz for STM32F4xx (should be adjusted according to your clock configuration)
    systick_init(84000000u);

    // Initialize the MPU6050 with desired settings
    MPU6050_Init(&g_mpu, I2C1, DMA1_Stream0, DMA1_Stream6, MPU6050_GYRO_FS_500, MPU6050_ACCEL_FS_4G, 7u, 3u);

    NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    NVIC_EnableIRQ(DMA1_Stream6_IRQn);
    NVIC_EnableIRQ(I2C1_EV_IRQn);
    NVIC_EnableIRQ(I2C1_ER_IRQn);

    uint32_t last_sample_ms = 0;

    while (1)
    {
        uint32_t now = g_tick_ms;

        // Process the MPU6050 state machine
        MPU6050_Process(&g_mpu, now);

        // If the MPU6050 is ready and it's time to sample, start a read operation
        if (MPU6050_IsReady(&g_mpu) && (now - last_sample_ms) >= 20u)
        {
            if (MPU6050_StartRead(&g_mpu))
            {
                last_sample_ms = now;
            }
        }

        if (MPU6050_DataAvailable(&g_mpu))
        {
            mpu6050_scaled_t s;
            MPU6050_GetScaled(&g_mpu, &s);
            /* Here you can use the scaled data (s) as needed
             * use s.accel_x_g, s.accel_y_g, s.accel_z_g for acceleration in g
             * use s.temp_c for temperature in Celsius
             * use s.gyro_x_dps, s.gyro_y_dps, s.gyro_z_dps for gyroscope in degrees per second
             * e.g. send it over UART, log it, process it, etc.
             */
        }

        if (MPU6050_HasError(&g_mpu))
        {
            // Handle the error, e.g., reset the MPU6050, log the error, etc.
        }
    }
}