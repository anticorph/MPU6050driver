# MPU-6050 Driver for STM32

A register-level (no HAL/LL) driver for the InvenSense MPU-6050 6-axis
IMU, built around:

- **DMA** for the 14-byte accel+temp+gyro burst read, so the CPU issues one
  request and is interrupted only when the whole transfer has landed in
  memory.
- **A non-blocking state machine** for everything else (init sequence,
  sampling cadence). There is no `delay()` / `HAL_Delay()` anywhere in the
  driver — the one genuinely time-based wait (the sensor's ~100 ms
  power-up settle time) is a tick comparison you drive from your own
  SysTick, not a busy sleep.

Reference target: **STM32F4** (I2C1 on PB6/PB7, DMA1 Stream0 for I2C1_RX,
DMA1 Stream6 for I2C1_TX).

## Files

| File               | Purpose                                                   |
|--------------------|------------------------------------------------------------|
| `mpu6050.h`        | Public API, register map, state machine types              |
| `mpu6050.c`        | GPIO/I2C/DMA setup + the non-blocking driver implementation |
| `main.c`           | Minimal wiring: NVIC, SysTick, main loop usage              |

## Wiring

| MPU-6050 pin | STM32 pin | Notes                                |
|--------------|-----------|---------------------------------------|
| VCC          | 3.3V      | Power Supply                           |
| GND          | GND       | Ground                                 |
| SCL          | PB6       | I2C1_SCL, AF4, open-drain              |
| SDA          | PB7       | I2C1_SDA, AF4, open-drain              |
| AD0          | GND       | sets 7-bit address to `0x68`           |
| INT          | (unused)  | not used by this driver, optional      |

Internal pull-ups are enabled in `mpu6050_gpio_init()`, but for reliable
100 kHz operation you should still add external 4.7 kΩ pull-ups to 3.3V
on both SDA and SCL.

## Full-scale range reference

| Constant               | Range     | Sensitivity          |
|-------------------------|-----------|-----------------------|
| `MPU6050_ACCEL_FS_2G`   | ±2 g      | 16384 LSB/g           |
| `MPU6050_ACCEL_FS_4G`   | ±4 g      | 8192 LSB/g            |
| `MPU6050_ACCEL_FS_8G`   | ±8 g      | 4096 LSB/g            |
| `MPU6050_ACCEL_FS_16G`  | ±16 g     | 2048 LSB/g            |
| `MPU6050_GYRO_FS_250`   | ±250 °/s  | 131.0 LSB/(°/s)       |
| `MPU6050_GYRO_FS_500`   | ±500 °/s  | 65.5 LSB/(°/s)        |
| `MPU6050_GYRO_FS_1000`  | ±1000 °/s | 32.8 LSB/(°/s)        |
| `MPU6050_GYRO_FS_2000`  | ±2000 °/s | 16.4 LSB/(°/s)        |

`MPU6050_GetScaled()` uses whichever `gyro_fs`/`accel_fs` you passed to
`MPU6050_Init()`.

## Error handling

- I2C bus errors (`BERR`, `ARLO`, `AF`, `OVR`, timeout) are caught in
  `MPU6050_I2C_ER_IRQHandler()`, which issues STOP and moves the state
  machine to `MPU_ST_ERROR`.
- A failed control-phase write/read (spin-wait exceeded) also lands in
  `MPU_ST_ERROR`.
- From `MPU_ST_ERROR`, the driver automatically retries the full init
  sequence up to `MPU6050_MAX_RETRIES` (default 3) before staying latched
  in the error state. Check `MPU6050_HasError()` in your app to surface a
  fault (LED, log, safe-state the control loop, etc).

## Porting to other STM32 families

This driver was written against the STM32F4 I2C1/DMA1 peripheral map.
To port:

1. **Header include**: swap `stm32f4xx.h` in `mpu6050.h` for your
   family's CMSIS header (`stm32f1xx.h`, `stm32f7xx.h`, `stm32h7xx.h`, ...).
2. **Clock assumption**: `I2C_APB1_CLK_MHZ` in `mpu6050.c` assumes a
   42 MHz APB1 (typical 84 MHz SYSCLK STM32F401/F411 setup). Recompute
   `CCR`/`TRISE` for your actual APB1 frequency, or replace the fixed
   macro with a runtime `SystemCoreClock`-derived calculation.
3. **DMA peripheral**: F1/F3 use a single shared DMA controller with
   different stream/channel naming (`DMA_Channel_TypeDef`, no `CHSEL`
   field — channels are fixed per peripheral instead of selectable).
   F7/H7 add a DMA MUX / different stream-request model. You'll need to
   adjust `mpu6050_dma_init()` and the `DMA_SxCR_*` register field names
   accordingly; the state machine and public API do not need to change.
4. **NVIC IRQ names**: update the vector names in `main.c` to
   match your family's startup file (e.g. `DMA1_Channel*_IRQHandler` on
   F1 instead of `DMA1_Stream*_IRQHandler` on F4).

## Known limitations / things to verify on your hardware

- This was written and reviewed at the register-sequence level but not
  compiled/flashed against physical hardware in this environment — please
  build against your exact CMSIS device header and confirm on a scope/
  logic analyzer before relying on it in production.
- The I2C peripheral on STM32F4 has well-documented errata around single-
  and two-byte DMA reads; this driver only ever DMAs the 14-byte burst
  (well above the errata's affected N=1/N=2 cases), so it should be clear
  of those specific issues, but always confirm against the errata sheet
  for your exact part number/silicon revision.
- No FIFO/multi-sample buffering is implemented — each `MPU6050_StartRead()`
  overwrites the previous sample once the new one completes.
