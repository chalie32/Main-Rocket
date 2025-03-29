#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

class KalmanFilter {
private:
    float Q_angle;    // Process noise variance for the accelerometer
    float Q_bias;     // Process noise variance for the gyro bias
    float R_measure;  // Measurement noise variance

    float angle;      // The angle calculated by the Kalman filter
    float bias;       // The gyro bias calculated by the Kalman filter
    float rate;       // Unbiased rate calculated from the rate and the calculated bias

    float P[2][2];    // Error covariance matrix - This is a 2x2 matrix
    
public:
    KalmanFilter();
    
    // The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
    float getAngle(float newAngle, float newRate, float dt);
    
    void setAngle(float angle);
    float getRate();
    float getAngle();
};

#endif // KALMAN_FILTER_H 