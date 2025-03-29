#include "MPU6050Handler.h"
#include <Wire.h>

// Define the interrupt pin
#define INTERRUPT_PIN 2

// Create the global instance
MPU6050Handler mpuHandler;
volatile bool mpuInterrupt = false;

// Interrupt service routine
void dmpDataReady() {
    mpuInterrupt = true;
}

MPU6050Handler::MPU6050Handler() : 
    dmpReady(false),
    mpuIntStatus(0),
    devStatus(0),
    packetSize(0),
    fifoCount(0),
    timer(0) {
    // Initialize arrays
    for (int i = 0; i < 3; i++) {
        ypr[i] = 0;
    }
    ax = ay = az = 0;
    gx = gy = gz = 0;
}

bool MPU6050Handler::initialize() {
    // Initialize I2C - use the default Wire for MPU6050
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    
    // Initialize serial communication
    Serial.begin(115200);

    // Initialize device
    Serial.println("Initializing I2C devices...");
    mpu.initialize();

    // Verify connection
    Serial.println("Testing device connections...");
    Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    
    // Load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // Supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);
    
    // Make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        
        // Turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // Enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // Set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // Get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
        
        // Initialize timer for Kalman filter
        timer = millis();
        
        return true;
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
        
        return false;
    }
}

void MPU6050Handler::update() {
    // If programming failed, don't try to do anything
    if (!dmpReady) return;
    
    // Wait for MPU interrupt or extra packet(s) available
    if (!mpuInterrupt && fifoCount < packetSize) return;
    
    // Reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    
    // Get current FIFO count
    fifoCount = mpu.getFIFOCount();
    
    // Check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // Reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
    } 
    // Otherwise, check for DMP data ready interrupt (this should happen frequently)
    else if (mpuIntStatus & 0x02) {
        // Wait for correct available data length
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        
        // Read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // Track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
        
        // Get sensor data
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        
        // Calculate delta time for Kalman filter
        float dt = (millis() - timer) / 1000.0f;
        timer = millis();
        
        // Apply Kalman filter to angles
        float rollDeg = ypr[2] * 180 / M_PI;
        float pitchDeg = ypr[1] * 180 / M_PI;
        float yawDeg = ypr[0] * 180 / M_PI;
        
        float gyroX = gx * GYRO_SCALE;
        float gyroY = gy * GYRO_SCALE;
        float gyroZ = gz * GYRO_SCALE;
        
        kalmanX.getAngle(rollDeg, gyroX, dt);
        kalmanY.getAngle(pitchDeg, gyroY, dt);
        kalmanZ.getAngle(yawDeg, gyroZ, dt);
    }
}

float MPU6050Handler::getYaw() {
    return kalmanZ.getAngle();
}

float MPU6050Handler::getPitch() {
    return kalmanY.getAngle();
}

float MPU6050Handler::getRoll() {
    return kalmanX.getAngle();
}

float MPU6050Handler::getAccelX() {
    return ax * ACCEL_SCALE;
}

float MPU6050Handler::getAccelY() {
    return ay * ACCEL_SCALE;
}

float MPU6050Handler::getAccelZ() {
    return az * ACCEL_SCALE;
}

void MPU6050Handler::setInterruptFlag() {
    mpuInterrupt = true;
}

bool MPU6050Handler::isReady() {
    return dmpReady;
} 