#ifndef BMP280_HANDLER_H
#define BMP280_HANDLER_H

#include <Arduino.h>
#include <Adafruit_BMP280.h>
#include <Wire.h>
#include "KalmanFilter.h"

class BMP280Handler {
private:
    Adafruit_BMP280 bmp;
    
    // Kalman filter for altitude readings
    KalmanFilter kalmanAltitude;
    
    // Store the sea level pressure for altitude calculations
    float seaLevelPressure;
    
    // Last reading time for Kalman filter
    unsigned long lastReadTime;
    
    // Filtered values
    float filteredAltitude;
    float temperature;
    
    // Flag to track initialization status
    bool initialized;

public:
    BMP280Handler();
    
    // Initialize the BMP280 sensor
    bool initialize();
    
    // Update sensor readings
    void update();
    
    // Set the sea level pressure (in hPa) for more accurate altitude
    void setSeaLevelPressure(float pressure);
    
    // Get the current temperature in Celsius
    float getTemperature();
    
    // Get the current altitude in meters
    float getAltitude();
    
    // Check if sensor is initialized
    bool isReady();
};

// Extern declaration for the global instance
extern BMP280Handler bmpHandler;

#endif // BMP280_HANDLER_H 