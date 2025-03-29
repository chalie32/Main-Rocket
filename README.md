# 🚀 Rocket Avionics System

## Overview

This project is dedicated to developing an advanced avionics system for a high-powered rocket. The system integrates multiple sensors with the [Teensy 4.1](https://www.pjrc.com/store/teensy41.html) microcontroller to collect and process flight data in real-time.

## Features

- **Sensor Integration:**
  - **MPU6050:** Captures accelerometer and gyroscope data.
  - **BMP280:** Measures barometric pressure and temperature.
- **Data Processing:** Implements a **Kalman Filter** for sensor fusion, enhancing the accuracy of flight data.

## Project Structure

The `src` directory contains the core components:

- `BMP280Handler.cpp` / `BMP280Handler.h`: Manages BMP280 sensor interactions.
- `MPU6050Handler.cpp` / `MPU6050Handler.h`: Manages MPU6050 sensor interactions.
- `KalmanFilter.cpp` / `KalmanFilter.h`: Implements the Kalman Filter algorithm for data fusion.
- `main.cpp`: Initializes the system and orchestrates sensor data acquisition and processing.

---

## 📝 更新日誌（Changelog）

### 2025-03-30
- 初始化 GitHub 專案。
- 完成 MPU6050 與 BMP280 感測器在 Teensy 4.1 上的整合。
- 建立模組化架構（BMP280Handler、MPU6050Handler、KalmanFilter）。
- 測試 I2C2 (pin 16/17) 通訊穩定性。
- 撰寫 README 初版。

---
