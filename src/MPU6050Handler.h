#ifndef MPU6050_HANDLER_H
#define MPU6050_HANDLER_H

#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "KalmanFilter.h"

class MPU6050Handler {
private:
    MPU6050 mpu;
    
    // MPU control/status vars
    bool dmpReady;        // set true if DMP init was successful
    uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
    uint8_t devStatus;    // return status after each device operation (0 = success, !0 = error)
    uint16_t packetSize;  // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount;   // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64]; // FIFO storage buffer

    Quaternion q;        // [w, x, y, z]         quaternion container
    VectorFloat gravity; // [x, y, z]            gravity vector
    float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
    int16_t ax, ay, az;  // acceleration values
    int16_t gx, gy, gz;  // gyro values

    // Kalman filter instances for each axis
    KalmanFilter kalmanX;
    KalmanFilter kalmanY;
    KalmanFilter kalmanZ;

    // Timing for Kalman filter
    unsigned long timer;

    // Constants for conversion
    const float ACCEL_SCALE = 9.81f / 16384.0f; // Convert to m/s² (9.81 / 2^14)
    const float GYRO_SCALE = 0.0174533f / 131.0f; // Convert to rad/s (π/180 / 131)

public:
    MPU6050Handler();
    
    bool initialize();
    void update();
    
    // Getters for filtered values
    float getYaw();
    float getPitch();
    float getRoll();
    float getAccelX();
    float getAccelY();
    float getAccelZ();
    
    // Interrupt handling
    void setInterruptFlag();
    bool isReady();
};

// Extern declaration for the global instance
extern MPU6050Handler mpuHandler;
extern volatile bool mpuInterrupt;

#endif // MPU6050_HANDLER_H 