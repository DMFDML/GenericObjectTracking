import numpy as np
import matplotlib.pyplot as plt

class KalmanFilter:
    def __init__(self, initial_angle, process_variance, measurement_variance):
        self.angle = initial_angle
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimate_error_covariance = 1.0
        
    def update(self, measurement):
        kalman_gain = self.estimate_error_covariance / (self.estimate_error_covariance + self.measurement_variance)
        self.angle += kalman_gain * (measurement - self.angle)
        self.estimate_error_covariance *= (1.0 - kalman_gain)
        
    def predict(self, motion):
        self.angle += motion
        self.estimate_error_covariance += self.process_variance

# Example usage
if __name__ == '__main__':
    # Initialize the Kalman filter
    initial_angle = 45.0
    process_variance = 0.3  # Process noise variance
    measurement_variance = 0.5  # Measurement noise variance
    kalman_filter = KalmanFilter(initial_angle, process_variance, measurement_variance)
    
    # Simulate noisy angle measurements
    true_angle = 45.0  # True angle (ground truth)
    num_measurements = 100
    measurements = np.random.normal(true_angle, measurement_variance, num_measurements)
    
    # Apply the Kalman filter to smooth the measurements
    smoothed_angles = []
    for measurement in measurements:
        kalman_filter.update(measurement)
        kalman_filter.predict(0.0)  # Assume no motion
        
        smoothed_angles.append(kalman_filter.angle)
    
    # Print the smoothed angles
    # for i in range(len(smoothed_angles)):
    #     print(f"Measurement {i+1}: {measurements[i]}, Smoothed Angle: {smoothed_angles[i]}")

    plt.plot(range(len(measurements)), measurements, label='Measurements')
    plt.plot(range(len(smoothed_angles)), smoothed_angles, label='Smoothed Angles')
    plt.show()
