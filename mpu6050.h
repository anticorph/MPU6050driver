/**
 * @file  mpu6050.h
 * @brief Bare-metal driver for the MPU6050 IMU sensor using I2C and DMA
 * on STM32 microcontrollers. No HAL/LL, no delay().
 * 
 * Target peripherals (adjust for your board in the MPU6050_Init() call):
 *   - I2C1 (PB = SCL, PB7 = SDA, AF4, open-drain + external/internal pull-ups)
 *   - DMA1 Stream0 Channel1 -> I2C1_RX
 *   - DMA1 Stream6 Channel1 -> I2C1_TX
 * 
 * Design:
 *   - Everything is driven by a state machine advanced by MPU6050_Process(),
 *     which you call from your main loop (or a periodic task) every cycle.
 *   - Register writes during init are short (2-byte) transfers handled by
 *     polling I2C flags with a bounded spin-count timeout (microseconds) --
 *     this is NOT a delay() call, it never sleeps, it just guards against a
 *     wedged bus.
 *   - The only genuinely time-based wait (the ~100 ms MPU-6050 power-up /
 *     reset settle time) is implemented as a non-blocking tick comparison,
 *     so the CPU is completely free to do other work while waiting.
 *   - The 14-byte sensor burst read (accel + temp + gyro) is done with DMA,
 *     so the CPU issues the request and is interrupted only once when the
 *     whole burst has landed in memory.
 */

#ifndef MPU6050_H
#define MPU6050_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>

// Use your exact device header
#include "stm32f4xx.h"

// Register addresses and constants for MPU6050
#define MPU6050_I2C_ADDRESS_7BIT    0x68u // Default I2C address for MPU6050
#define MPU6050_I2C_ADDRESS_W       (MPU6050_I2C_ADDRESS_7BIT << 1)
#define MPU6050_I2C_ADDRESS_R       ((MPU6050_I2C_ADDRESS_W << 1) | 0x01u)

#define MPU6050_REG_SMPLRT_DIV      0x19u
#define MPU6050_REG_CONFIG          0x1Au
#define MPU6050_REG_GYRO_CONFIG     0x1Bu
#define MPU6050_REG_ACCEL_CONFIG    0x1Cu
#define MPU6050_REG_ACCEL_XOUT_H    0x3Bu
#define MPU6050_REG_PWR_MGMT_1      0x6Bu
#define MPU6050_REG_WHO_AM_I        0x75u

#define MPU6050_WHO_AM_I_VALUE      0x68u

// Full-scale range selects
#define MPU6050_GYRO_FS_250         0x00u // 250  deg/s -> 131.0 LSB/(deg/s)
#define MPU6050_GYRO_FS_500         0x08u // 500  deg/s -> 65.5  LSB/(deg/s)
#define MPU6050_GYRO_FS_1000        0x10u // 1000 deg/s -> 32.8  LSB/(deg/s)
#define MPU6050_GYRO_FS_2000        0x18u // 2000 deg/s -> 16.4  LSB/(deg/s)

#define MPU6050_ACCEL_FS_2G         0x00u // +-2  g -> 16384 LSB/g
#define MPU6050_ACCEL_FS_4G         0x08u // +-4  g -> 8192  LSB/g
#define MPU6050_ACCEL_FS_8G         0x10u // +-8  g -> 4096  LSB/g
#define MPU6050_ACCEL_FS_16G        0x18u // +-16 g -> 2048  LSB/g

typedef enum
{
    MPU_ST_RESET = 0,
    MPU_ST_WAIT_POWERUP, 
    MPU_ST_WAKE_WRITE,      // PWR_MGMT_1 = 0x00 (leave sleep, pick clock)
    MPU_ST_WAKE_WAIT,
    MPU_ST_SMPLRT_WRITE,
    MPU_ST_SMPLRT_WAIT,
    MPU_ST_CONFIG_WRITE,    // DLPF config
    MPU_ST_CONFIG_WAIT,
    MPU_ST_GYRO_CFG_WRITE,
    MPU_ST_GYRO_CFG_WAIT,
    MPU_ST_ACCEL_CFG_WRITE,
    MPU_ST_ACCEL_CFG_WAIT,
    MPU_ST_WHOAMI_REQUEST,  // Verify cip is present
    MPU_ST_WHOAMI_WAIT,
    MPU_ST_READY,           // Idle, call MPU6050_StartRead() to sample
    MPU_ST_READ_SETUP,      // Sending register pointer (ACCEL_XOUT_H)
    MPU_ST_READ_DMA,        // DMA burst in flight
    MPU_ST_READ_DONE,       // Fresh data available, one-shot flag
    MPU_ST_ERROR
} mpu6050_state_t;

typedef struct
{
    int16_t accel_x, accel_y, accel_z;
    int16_t temp_raw;
    int16_t gyro_x, gyro_y, gyro_z;
} mpu6050_raw_t;

typedef struct
{
    float accel_x_g, accel_y_g, accel_z_g;    // Acceleration in g
    float temp_c;                             // Temperature in Celsius
    float gyro_x_dps, gyro_y_dps, gyro_z_dps; // Gyroscope in degrees per second
} mpu6050_scaled_t;

typedef struct mpu6050_dev
{
    I2C_TypeDef *i2c;
    DMA_Stream_TypeDef *dma_rx; // I2C RX stream (DMA1_Stream0)
    DMA_Stream_TypeDef *dma_tx; // I2C TX stream (DMA1_Stream6)

    uint8_t gyro_fs;
    uint8_t accel_fs;
    uint8_t smplrt_div;
    uint8_t dlpf_config;

    // Internal state machine
    volatile mpu6050_state_t state;
    volatile bool i2c_error;
    uint32_t wait_start_ms;
    uint32_t wait_len_ms;

    uint8_t ctrl_tx[2]; // Control bytes for I2C write operations
    uint8_t ctrl_rx[1]; // Control byte for I2C read operations

    uint8_t dma_rx_buffer[14];

    mpu6050_raw_t raw_data;

    uint32_t error_count;
} mpu6050_t;

// Call MPU6050_Process() repeatedly, then run through the async init sequence
void MPU6050_Init(mpu6050_t *dev, I2C_TypeDef *i2c,
                  DMA_Stream_TypeDef *dma_rx,
                  DMA_Stream_TypeDef *dma_tx,
                  uint8_t gyro_fs, uint8_t accel_fs,
                  uint8_t smplrt_div, uint8_t dlpf_config);

// Must be called periodically with the current millisecond tick count (from SysTick)
void MPU6050_Process(mpu6050_t *dev, uint32_t current_time_ms);

// True once init has finished successfully and the driver is idle
bool MPU6050_IsReady(const mpu6050_t *dev);

// True if init/read failed after retries (set to 3; see mpu6050.c MAX_RETRIES)
bool MPU6050_HasError(const mpu6050_t *dev);

// Returns false if the driver isn't in MPU_ST_READY (still busy or not initialised yet)
bool MPU6050_StartRead(mpu6050_t *dev);

// True only once per completed read; reading it clears the flag
bool MPU6050_DataAvailable(mpu6050_t *dev);

// Copy out the last completed raw sample
void MPU6050_GetRawData(const mpu6050_t *dev, mpu6050_raw_t *data);

// Copy out the last completed sample converted to physical units
void MPU6050_GetScaledData(const mpu6050_t *dev, mpu6050_scaled_t *data);

// Interrupt hools should be called form the actual vector table handlers
void MPU6050_I2C_EV_IRQHandler(mpu6050_t *dev);
void MPU6050_I2C_ER_IRQHandler(mpu6050_t *dev);
void MPU6050_DMA_RX_IRQHandler(mpu6050_t *dev);
void MPU6050_DMA_TX_IRQHandler(mpu6050_t *dev);

#ifdef __cplusplus
}
#endif

#endif // MPU6050_H