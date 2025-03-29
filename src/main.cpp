#include <Arduino.h>
#include "MPU6050Handler.h"
#include "BMP280Handler.h"

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  
  // Wait for serial port to connect (needed for native USB port only)
  while (!Serial) {
    delay(10);
  }
  
  Serial.println(F("Starting sensor initialization..."));
  
  // Initialize the MPU6050
  if (mpuHandler.initialize()) {
    Serial.println(F("MPU6050 initialization successful"));
  } else {
    Serial.println(F("MPU6050 initialization failed"));
  }
  
  // Initialize the BMP280
  if (bmpHandler.initialize()) {
    Serial.println(F("BMP280 initialization successful"));
    
    // Optional: Set the sea level pressure for more accurate altitude readings
    // You can calibrate this value based on your local weather report
    bmpHandler.setSeaLevelPressure(1013.25); // Standard sea level pressure in hPa
  } else {
    Serial.println(F("BMP280 initialization failed - continuing without BMP280"));
  }
  
  Serial.println(F("Setup complete"));
}

void loop() {
  // Update MPU6050 sensor readings and apply Kalman filter
  mpuHandler.update();
  
  // Update BMP280 sensor readings
  bmpHandler.update();
  
  // Get filtered values from MPU6050
  float yaw = mpuHandler.getYaw();
  float pitch = mpuHandler.getPitch();
  float roll = mpuHandler.getRoll();
  float accX = mpuHandler.getAccelX();
  float accY = mpuHandler.getAccelY();
  float accZ = mpuHandler.getAccelZ();
  
  // Get values from BMP280
  float temperature = bmpHandler.getTemperature();
  float altitude = bmpHandler.getAltitude();
  
  // Output formatted data
  Serial.print("Yaw: "); Serial.print(yaw);
  Serial.print(" Pitch: "); Serial.print(pitch);
  Serial.print(" Roll: "); Serial.print(roll);
  Serial.print(" X: "); Serial.print(accX);
  Serial.print(" Y: "); Serial.print(accY);
  Serial.print(" Z: "); Serial.print(accZ);
  
  // Only print BMP280 data if it's initialized
  if (bmpHandler.isReady()) {
    Serial.print(" Temp: "); Serial.print(temperature);
    Serial.print("°C Alt: "); Serial.print(altitude);
    Serial.println("m");
  } else {
    Serial.println(" (BMP280 not available)");
  }
  
  delay(10); // Small delay for stability
}

