#!/usr/bin/env python3
"""
Adaptive PID Controller with gain scheduling and feedforward control
"""
import numpy as np

class AdaptivePID:
    """
    Advanced PID controller with:
    - Gain scheduling based on boat speed
    - Feedforward control
    - Advanced anti-windup
    - Derivative filtering
    """
    
    def __init__(self, Kp_base=70.0, Ki_base=0.5, Kd_base=35.0):
        self.Kp_base = Kp_base
        self.Ki_base = Ki_base
        self.Kd_base = Kd_base
        
        self.integral = 0.0
        self.last_error = 0.0
        self.last_output = 0.0
        self.filtered_derivative = 0.0
        
        # Anti-windup parameters
        self.integral_max = 100.0
        self.output_min = -21.0
        self.output_max = 21.0
    
    def get_scheduled_gains(self, boat_speed):
        """Adjust PID gains based on boat speed"""
        # Speed-based scaling
        # Assume boat speed range: 0.5 - 4.0 m/s
        speed_factor = np.clip(boat_speed / 2.0, 0.5, 2.0)
        
        Kp = self.Kp_base * speed_factor
        Ki = self.Ki_base * speed_factor * 0.5  # Less aggressive integral
        Kd = self.Kd_base * speed_factor
        
        return Kp, Ki, Kd
    
    def calculate_feedforward(self, error, boat_speed):
        """Predict required rudder angle based on physics"""
        # Estimate turn rate needed (correct error in 10 seconds)
        desired_turn_rate = error / 10.0  # deg/s
        
        # Rudder effectiveness increases with boat speed
        k = 50.0  # Tuning parameter
        
        if boat_speed > 0.5:
            feedforward_rudder = k * desired_turn_rate / boat_speed
        else:
            feedforward_rudder = 0.0
        
        # Clamp to reasonable range
        feedforward_rudder = np.clip(feedforward_rudder, -10, 10)
        
        return feedforward_rudder
    
    def update(self, error, dt, boat_speed, use_gain_scheduling=True, use_feedforward=True):
        """Update PID controller with new error value"""
        # Get gains
        if use_gain_scheduling:
            Kp, Ki, Kd = self.get_scheduled_gains(boat_speed)
        else:
            Kp, Ki, Kd = self.Kp_base, self.Ki_base, self.Kd_base
        
        # Proportional term
        P = Kp * error
        
        # Integral term with anti-windup
        self.integral += error * dt
        self.integral = np.clip(self.integral, -self.integral_max, self.integral_max)
        I = Ki * self.integral
        
        # Derivative term with filtering
        if dt > 0:
            derivative = (error - self.last_error) / dt
        else:
            derivative = 0.0
        
        # Low-pass filter on derivative (alpha = 0.1)
        self.filtered_derivative = 0.1 * derivative + 0.9 * self.filtered_derivative
        D = Kd * self.filtered_derivative
        
        # Calculate output
        output = P + I + D
        
        # Add feedforward
        if use_feedforward:
            ff = self.calculate_feedforward(error, boat_speed)
            output += ff
        else:
            ff = 0.0
        
        # Apply output limits
        output_clamped = np.clip(output, self.output_min, self.output_max)
        
        # Back-calculation anti-windup
        if output != output_clamped:
            saturation_error = output_clamped - output
            self.integral += saturation_error / (Ki + 1e-6) * 0.5
        
        # Store for next iteration
        self.last_error = error
        self.last_output = output_clamped
        
        return output_clamped, {'P': P, 'I': I, 'D': D, 'FF': ff if use_feedforward else 0.0, 'Kp': Kp, 'Ki': Ki, 'Kd': Kd}
    
    def reset(self):
        """Reset the PID controller state"""
        self.integral = 0.0
        self.last_error = 0.0
        self.last_output = 0.0
        self.filtered_derivative = 0.0


def normalize_angle(angle):
    """Normalize angle to [-180, 180]"""
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


def calculate_heading_error(target_heading, current_heading):
    """Calculate shortest angular distance between two headings"""
    error = target_heading - current_heading
    return normalize_angle(error)

