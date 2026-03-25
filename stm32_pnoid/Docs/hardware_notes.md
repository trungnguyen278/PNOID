# Hardware Notes - STM32 PNOID

## I2C1 Bus (PB8=SCL, PB9=SDA) — 400kHz Fast Mode

| Device | Address | Ghi chu |
|--------|---------|---------|
| PCA9685 #1 | 0x41 | A0 soldered |
| PCA9685 #2 | 0x42 | A1 soldered |
| ICM-20948 | 0x68 | AD0=LOW (default) |
| AK09916 (mag) | 0x0C | Internal to ICM-20948, accessed via I2C master |
| 0x70 | — | PCA9685 All-Call address (normal) |

### I2C1 Speed
- **Must use Fast Mode (400kHz)**, NOT Fast Mode Plus
- PCA9685 clone modules fail at FM+ (~1MHz) — NACK on data phase
- CubeMX: I2C1 Speed Mode = Fast Mode, Timing = 0x00B03FDB

## ICM-20948 IMU Mounting

- Module mounted **upside-down** (IC facing down)
- Firmware compensates by flipping X and Z axes in `icm20948.cpp::read()`
- Standing upright: Az ≈ +960mg, Roll ≈ 0, Pitch ≈ 0
- **Temperature reads ~43°C** — this is die temperature, NOT ambient. Normal.
- Yaw=0 if magnetometer not calibrated

## PCA9685 Servo Drivers

- Servo: SG92R (or compatible)
- PWM: 50Hz, prescale=121
- Pulse range: 500us (0°) — 2500us (180°)
- After setting angle, servo **draws current continuously** to hold position (gets hot)
- Stop PWM (`sleep()`) to release and cool down

## SPI Peripherals

| SPI | Pins | Usage | Speed |
|-----|------|-------|-------|
| SPI1 (I2S1) | PA15(WS), PB3(CK), PB4(SDI), PB5(SDO) | Audio DAC | — |
| SPI2 | PB13(SCK), PB14(MISO), PB15(MOSI) | LCD (240x240) | 30MHz |
| SPI4 | PE12(SCK), PE13(MISO), PE14(MOSI), PE11(CS) | ESP comms (DMA) | 15MHz |

## UART

| UART | Pins | Usage | Baud |
|------|------|-------|------|
| USART1 | PA9(TX), PA10(RX) | Debug log (CP2102) | 115200 |
| USART2 | PA2(TX), PA3(RX) | ESP command/control | 115200 |

## GPIO — Multiplexer (Analog Decoder)

| Pin | Label | Note |
|-----|-------|------|
| PD4 | MUX_S0 | |
| PD5 | MUX_S1 | |
| PD6 | MUX_S2 | |
| PD7 | MUX_S3 | |
| PD8 | MUX_S4 | 5th bit for 32-channel decode |

## GPIO — IMU Control

| Pin | Label | Note |
|-----|-------|------|
| PE7 | BNO_INT | ICM-20948 INT (active low, pull-up). Not used yet (polling). |
| PE8 | BNO_RST | ICM-20948 RST (active low, default high). Not used yet. |
