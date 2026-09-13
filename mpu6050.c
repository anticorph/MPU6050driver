/**
 * @file  mpu6050.c
 * @brief Bare-metal driver for the MPU6050 IMU sensor using I2C and DMA
 * on STM32 microcontrollers. No HAL/LL, no delay().
 * 
 * This file talks to raw peripheral registers (CMSIS level), dependency-free.
 *   - RCC clocks for GPIOB, I2C1 and DMA1 enabled before/inside MPU6050_Init
 *   - NVIC interrupts enabled for I2C1_EV, I2C1_ER, DMA1_Stream0, DMA1_Stream6
 *   - a millisecond tick source (SysTick) whose count you pass into MPU6050_Process()
 */

#include "mpu6050.h"

// Tunables
#define MPU6050_POWERUP_WAIT_MS     100u    // Delay for MPU6050 power-up in milliseconds
#define MPU6050_I2C_SPIN_LIMIT      50000u  // Limit for I2C spin-wait loops
#define MPU6050_MAX_RETRIES         3u      // Maximum number of retries for I2C operations
#define I2C_APB1_CLK_MHZ            42u     // for STM32F4xx; adjust according to your clock config
#define I2C_TARGET_FREQ_HZ          100000u // Target I2C frequency in Hz (100 kHz standard mode)

/* Small internal helpers
 * Returns true if the flag became the requested value before the spin limit was hit
 */
static bool spin_wait_for_flag(volatile uint32_t reg, uint32_t mask, bool want_set)
{
    uint32_t guard = MPU6050_I2C_SPIN_LIMIT;
    while (guard--)
    {
        bool is_set = (reg & mask) != 0u;
        if (is_set == want_set)
        {
            return true; // Desired flag state achieved
        }
    }
    return false; // Flag state not achieved within limit
}

static void mpu6050_enter_error(mpu6050_t *dev)
{
    dev->state = MPU_ST_ERROR;
    dev->i2c_error = true;
    dev->error_count++;
}

// Low-level peripheral configuration
static void mpu6050_gpio_init(void)
{
    // Enable GPIO clocks for I2C pins (adjust according to your hardware configuration)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; // Enable GPIOB clock

    // Configure I2C1 SCL (PB6) and SDA (PB7) pins
    GPIOB->MODER    &= ~((3u << (6 * 2)) | (3u << (7 * 2)));     // Clear mode bits
    GPIOB->MODER    |=  ((2u << (6 * 2)) | (2u << (7 * 2)));     // Set to alternate function mode
    GPIOB->OTYPER   |=  ((1u << 6) | (1u << 7));                 // Open-drain
    GPIOB->OSPEEDR  |=  ((3u << (6 * 2)) | (3u << (7 * 2)));     // High speed
    GPIOB->PUPDR    &= ~((3u << (6 * 2)) | (3u << (7 * 2)));     // No pull-up/pull-down
    GPIOB->PUPDR    |=  ((1u << (6 * 2)) | (1u << (7 * 2)));     // Pull-up
    GPIOB->AFR[0]   &= ~((0xFu << (6 * 4)) | (0xFu << (7 * 4))); // Clear alternate function bits
    GPIOB->AFR[0]   |=  ((4u << (6 * 4)) | (4u << (7 * 4)));     // AF4 for I2C1
}

static void mpu6050_i2c_init(I2C_TypeDef *i2c)
{
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    i2c->CR1 |= I2C_CR1_SWRST;
    i2c->CR1 &= ~I2C_CR1_SWRST;

    i2c->CR2 = (I2C_APB1_CLK_MHZ & 0x3Fu);           // peripheral clock, MHz

    // Standard mode 100 kHz: CCR = Fpclk / (2 * Fi2c)
    uint16_t ccr = (uint16_t)(I2C_APB1_CLK_MHZ * 1000000u / (2u * I2C_TARGET_FREQ_HZ));
    i2c->CCR = ccr & 0x0FFFu;                        // standard mode, bit15 = 0

    i2c->TRISE = I2C_APB1_CLK_MHZ + 1u;              // max rise time / Tpclk +1

    i2c->CR2 |= I2C_CR2_ITEVTEN | I2C_CR2_ITERREN;   // enable event/error IRQs

    i2c->CR1 |= I2C_CR1_PE;                          // peripheral enable
    i2c->CR1 |= I2C_CR1_ACK;
}

static void mpu6050_dma_init(mpu6050_t *dev)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

    // RX stream: I2C1_RX -> memory (dma_rx_buf), channel 1
    dev->dma_rx->CR &= ~DMA_SxCR_EN;
    while (dev->dma_rx->CR & DMA_SxCR_EN)
    {
        // wait for HW to actually disable
    }
    dev->dma_rx->CR  = (1u << 25);                   // CHSEL = channel 1
    dev->dma_rx->CR |= (0u << 6);                    // DIR = periph-to-mem
    dev->dma_rx->CR |= DMA_SxCR_MINC;                // memory increment
    dev->dma_rx->CR |= DMA_SxCR_TCIE;                // transfer-complete IRQ
    dev->dma_rx->PAR  = (uint32_t)&dev->i2c->DR;

    // TX stream: memory -> I2C1_TX, channel 1
    dev->dma_tx->CR &= ~DMA_SxCR_EN;
    while (dev->dma_tx->CR & DMA_SxCR_EN)
    {
        // wait
    }
    dev->dma_tx->CR  = (1u << 25);
    dev->dma_tx->CR |= (1u << 6);                    // DIR = mem-to-periph
    dev->dma_tx->CR |= DMA_SxCR_MINC;
    dev->dma_tx->CR |= DMA_SxCR_TCIE;
    dev->dma_tx->PAR  = (uint32_t)&dev->i2c->DR;
}

// Control-phase (small) I2C transfers - polled with a bounded spin only
static bool i2c_write_reg(I2C_TypeDef *i2c, uint8_t reg, uint8_t value)
{
    i2c->CR1 |= I2C_CR1_START;
    if (!spin_wait_flag(i2c->SR1, I2C_SR1_SB, true)) return false;

    i2c->DR = MPU6050_I2C_ADDR_W;
    if (!spin_wait_flag(i2c->SR1, I2C_SR1_ADDR, true)) return false;
    (void)i2c->SR1; (void)i2c->SR2;                  // clear ADDR

    if (!spin_wait_flag(i2c->SR1, I2C_SR1_TXE, true)) return false;
    i2c->DR = reg;
    if (!spin_wait_flag(i2c->SR1, I2C_SR1_TXE, true)) return false;
    i2c->DR = value;
    if (!spin_wait_flag(i2c->SR1, I2C_SR1_BTF, true)) return false;

    i2c->CR1 |= I2C_CR1_STOP;
    return true;
}

/* Requests a single register value with a repeated-start read (used only
 * for the WHO_AM_I sanity check, which is a one-shot, one-byte read)
 */
static bool i2c_read_reg(I2C_TypeDef *i2c, uint8_t reg, uint8_t *out)
{
    i2c->CR1 |= I2C_CR1_START;
    if (!spin_wait_flag(i2c->SR1, I2C_SR1_SB, true)) return false;
    i2c->DR = MPU6050_I2C_ADDR_W;
    if (!spin_wait_flag(i2c->SR1, I2C_SR1_ADDR, true)) return false;
    (void)i2c->SR1; (void)i2c->SR2;

    if (!spin_wait_flag(i2c->SR1, I2C_SR1_TXE, true)) return false;
    i2c->DR = reg;
    if (!spin_wait_flag(i2c->SR1, I2C_SR1_BTF, true)) return false;

    i2c->CR1 &= ~I2C_CR1_ACK;
    i2c->CR1 |= I2C_CR1_START;                       // repeated start
    if (!spin_wait_flag(i2c->SR1, I2C_SR1_SB, true)) return false;
    i2c->DR = MPU6050_I2C_ADDR_R;
    if (!spin_wait_flag(i2c->SR1, I2C_SR1_ADDR, true)) return false;
    (void)i2c->SR1; (void)i2c->SR2;

    i2c->CR1 |= I2C_CR1_STOP;
    if (!spin_wait_flag(i2c->SR1, I2C_SR1_RXNE, true)) return false;
    *out = (uint8_t)i2c->DR;
    return true;
}

/* Non-blocking kick-off of the DMA burst read: send device addr + register pointer,
 * then arm DMA on the RX stream so the 14 data bytes stream into memory
 * without further CPU involvement. STOP is issued from the DMA transfer-complete ISR
 */
static bool i2c_start_dma_burst_read(mpu6050_t *dev, uint8_t start_reg, uint8_t *buf, uint16_t len)
{
    I2C_TypeDef *i2c = dev->i2c;

    i2c->CR1 |= I2C_CR1_START;
    if (!spin_wait_flag(i2c->SR1, I2C_SR1_SB, true)) return false;
    i2c->DR = MPU6050_I2C_ADDR_W;
    if (!spin_wait_flag(i2c->SR1, I2C_SR1_ADDR, true)) return false;
    (void)i2c->SR1; (void)i2c->SR2;

    if (!spin_wait_flag(i2c->SR1, I2C_SR1_TXE, true)) return false;
    i2c->DR = start_reg;
    if (!spin_wait_flag(i2c->SR1, I2C_SR1_BTF, true)) return false;

    i2c->CR1 |= I2C_CR1_ACK;
    i2c->CR1 |= I2C_CR1_START;                       // repeated start, read
    if (!spin_wait_flag(i2c->SR1, I2C_SR1_SB, true)) return false;
    i2c->DR = MPU6050_I2C_ADDR_R;
    if (!spin_wait_flag(i2c->SR1, I2C_SR1_ADDR, true)) return false;

    // Arm DMA before clearing ADDR so the first byte isn't missed
    dev->dma_rx->M0AR = (uint32_t)buf;
    dev->dma_rx->NDTR = len;
    i2c->CR2 |= I2C_CR2_LAST;                        // auto-NACK on last byte
    i2c->CR2 |= I2C_CR2_DMAEN;
    dev->dma_rx->CR |= DMA_SxCR_EN;

    (void)i2c->SR1; (void)i2c->SR2;                  // clear ADDR, transfer starts
    return true;
}

void MPU6050_Init(mpu6050_t *dev, I2C_TypeDef *i2c,
                  DMA_Stream_TypeDef *dma_rx_stream,
                  DMA_Stream_TypeDef *dma_tx_stream,
                  uint8_t gyro_fs, uint8_t accel_fs,
                  uint8_t sample_rate_div, uint8_t dlpf_cfg)
{
    dev->i2c             = i2c;
    dev->dma_rx          = dma_rx_stream;
    dev->dma_tx          = dma_tx_stream;
    dev->gyro_fs         = gyro_fs;
    dev->accel_fs        = accel_fs;
    dev->sample_rate_div = sample_rate_div;
    dev->dlpf_cfg        = dlpf_cfg;
    dev->i2c_error       = false;
    dev->error_count     = 0u;
    dev->state           = MPU_ST_RESET;

    mpu6050_gpio_init();
    mpu6050_i2c_init(i2c);
    mpu6050_dma_init(dev);
}

bool MPU6050_IsReady(const mpu6050_t *dev)
{
    return dev->state == MPU_ST_READY;
}

bool MPU6050_HasError(const mpu6050_t *dev)
{
    return dev->state == MPU_ST_ERROR;
}

bool MPU6050_StartRead(mpu6050_t *dev)
{
    if (dev->state != MPU_ST_READY)
    {
        return false;
    }
    dev->state = MPU_ST_READ_SETUP;
    return true;
}

bool MPU6050_DataAvailable(mpu6050_t *dev)
{
    if (dev->state == MPU_ST_READ_DONE)
    {
        dev->state = MPU_ST_READY;
        return true;
    }
    return false;
}

void MPU6050_GetRaw(const mpu6050_t *dev, mpu6050_raw_t *out)
{
    *out = dev->raw;
}

void MPU6050_GetScaled(const mpu6050_t *dev, mpu6050_scaled_t *out)
{
    static const float accel_lsb[4] = {16384.0f, 8192.0f, 4096.0f, 2048.0f};
    static const float gyro_lsb[4]  = {131.0f, 65.5f, 32.8f, 16.4f};

    float a_scale = accel_lsb[(dev->accel_fs >> 3) & 0x03u];
    float g_scale = gyro_lsb[(dev->gyro_fs >> 3) & 0x03u];

    out->ax_g   =  dev->raw.accel_x / a_scale;
    out->ay_g   =  dev->raw.accel_y / a_scale;
    out->az_g   =  dev->raw.accel_z / a_scale;
    out->temp_c = (dev->raw.temp_raw / 340.0f) + 36.53f;
    out->gx_dps =  dev->raw.gyro_x / g_scale;
    out->gy_dps =  dev->raw.gyro_y / g_scale;
    out->gz_dps =  dev->raw.gyro_z / g_scale;
}

// The state machine - call this every loop iteration / scheduler tick
void MPU6050_Process(mpu6050_t *dev, uint32_t now_ms)
{
    switch (dev->state)
    {
    case MPU_ST_RESET:
        dev->wait_start_ms = now_ms;
        dev->wait_len_ms   = MPU6050_POWERUP_WAIT_MS;
        dev->state = MPU_ST_WAIT_POWERUP;
        break;

    case MPU_ST_WAIT_POWERUP:
        if ((uint32_t)(now_ms - dev->wait_start_ms) >= dev->wait_len_ms)
        {
            dev->state = MPU_ST_WAKE_WRITE;
        }
        break;

    case MPU_ST_WAKE_WRITE:
        if (i2c_write_reg(dev->i2c, MPU6050_REG_PWR_MGMT_1, 0x00u))
        {
            dev->state = MPU_ST_SMPLRT_WRITE;
        } else {
            mpu6050_enter_error(dev);
        }
        break;

    case MPU_ST_SMPLRT_WRITE:
        if (i2c_write_reg(dev->i2c, MPU6050_REG_SMPLRT_DIV, dev->sample_rate_div))
        {
            dev->state = MPU_ST_CONFIG_WRITE;
        } else {
            mpu6050_enter_error(dev);
        }
        break;

    case MPU_ST_CONFIG_WRITE:
        if (i2c_write_reg(dev->i2c, MPU6050_REG_CONFIG, dev->dlpf_cfg))
        {
            dev->state = MPU_ST_GYRO_CFG_WRITE;
        } else {
            mpu6050_enter_error(dev);
        }
        break;

    case MPU_ST_GYRO_CFG_WRITE:
        if (i2c_write_reg(dev->i2c, MPU6050_REG_GYRO_CONFIG, dev->gyro_fs))
        {
            dev->state = MPU_ST_ACCEL_CFG_WRITE;
        } else {
            mpu6050_enter_error(dev);
        }
        break;

    case MPU_ST_ACCEL_CFG_WRITE:
        if (i2c_write_reg(dev->i2c, MPU6050_REG_ACCEL_CONFIG, dev->accel_fs))
        {
            dev->state = MPU_ST_WHOAMI_REQUEST;
        } else {
            mpu6050_enter_error(dev);
        }
        break;

    case MPU_ST_WHOAMI_REQUEST:
    {
        uint8_t who = 0;
        if (i2c_read_reg(dev->i2c, MPU6050_REG_WHO_AM_I, &who) && who == MPU6050_WHO_AM_I_VALUE)
        {
            dev->state = MPU_ST_READY;
        } else {
            mpu6050_enter_error(dev);
        }
        break;
    }

    case MPU_ST_READY:
        // Idle - waiting for MPU6050_StartRead()
        break;

    case MPU_ST_READ_SETUP:
        if (i2c_start_dma_burst_read(dev, MPU6050_REG_ACCEL_XOUT_H,
            dev->dma_rx_buf, sizeof(dev->dma_rx_buf)))
        {
            dev->state = MPU_ST_READ_DMA;
        } else {
            mpu6050_enter_error(dev);
        }
        break;

    case MPU_ST_READ_DMA:
        // Waiting for MPU6050_DMA_RX_IRQHandler() to move us to READ_DONE
        break;

    case MPU_ST_READ_DONE:
        // Left as-is until the application calls MPU6050_DataAvailable()
        break;

    case MPU_ST_ERROR:
    default:
        /* Re-run init from scratch a bounded number of times,
         * then stay latched in ERROR for the app to handle
         * (e.g. power-cycle the sensor, flag a fault LED, etc)
         */
        if (dev->error_count < MPU6050_MAX_RETRIES)
        {
            dev->i2c_error = false;
            dev->state = MPU_ST_RESET;
        }
        break;
    }
}

// Interrupt handlers
void MPU6050_DMA_RX_IRQHandler(mpu6050_t *dev)
{
    /* Caller's actual ISR should check/clear the correct DMA1 stream's
     * TCIF flag in the LIFCR/HIFCR register before/after calling this -
     * kept out of this function so one handler works for any stream
     * number the user wires up
     */
    dev->dma_rx->CR  &= ~DMA_SxCR_EN;
    dev->i2c->CR2    &= ~(I2C_CR2_DMAEN | I2C_CR2_LAST);
    dev->i2c->CR1    |=  I2C_CR1_STOP;

    uint8_t *b = dev->dma_rx_buf;
    dev->raw.accel_x  = (int16_t)((b[0]  << 8) | b[1]);
    dev->raw.accel_y  = (int16_t)((b[2]  << 8) | b[3]);
    dev->raw.accel_z  = (int16_t)((b[4]  << 8) | b[5]);
    dev->raw.temp_raw = (int16_t)((b[6]  << 8) | b[7]);
    dev->raw.gyro_x   = (int16_t)((b[8]  << 8) | b[9]);
    dev->raw.gyro_y   = (int16_t)((b[10] << 8) | b[11]);
    dev->raw.gyro_z   = (int16_t)((b[12] << 8) | b[13]);

    dev->state = MPU_ST_READ_DONE;
}

void MPU6050_DMA_TX_IRQHandler(mpu6050_t *dev)
{
    /* Not used by the current read/write sequences (writes are short enough
     * to be done with polled TXE/BTF), but provided so the API is symmetric
     * if you extend the driver to DMA-write multi-register blocks
     * (e.g. writing an entire config block in one shot)
     */
    dev->dma_tx->CR &= ~DMA_SxCR_EN;
    dev->i2c->CR2 &= ~I2C_CR2_DMAEN;
}

void MPU6050_I2C_EV_IRQHandler(mpu6050_t *dev)
{
    /* Present for API completeness / future extension (e.g. moving the control-phase
     * writes from polled to fully interrupt-driven). The current implementation
     * only relies on I2C interrupts being enabled so that DMA-triggering event
     * flags are latched correctly; no action is required here for the read/write paths above
     */
    (void)dev;
}

void MPU6050_I2C_ER_IRQHandler(mpu6050_t *dev)
{
    uint32_t sr1 = dev->i2c->SR1;

    if (sr1 & (I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_AF | I2C_SR1_OVR | I2C_SR1_TIMEOUT))
    {
        dev->i2c->SR1 &= ~(I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_AF | I2C_SR1_OVR | I2C_SR1_TIMEOUT);
        dev->i2c->CR1 |= I2C_CR1_STOP;
        mpu6050_enter_error(dev);
    }
}