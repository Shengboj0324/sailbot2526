#!/usr/bin/env python3
"""Pure wind-direction smoother used by regression tests and non-ROS demos."""
from collections import deque

import numpy as np


class AdaptiveWindKalmanFilter:
    """Adaptive Kalman filter for circular wind direction measurements."""

    def __init__(self):
        self.state = np.array([0.0, 0.0])
        self.P = np.eye(2) * 10.0
        self.Q_base = np.diag([0.01, 0.001])
        self.Q_gust = np.diag([1.0, 0.1])
        self.Q = self.Q_base.copy()
        self.R = 0.1
        self.innovation_threshold = 0.26
        self.gust_detected = False
        self.gust_counter = 0
        self.gust_threshold_count = 3
        self.steady_wind = 0.0
        self.steady_wind_alpha = 0.003
        self.innovation_history = deque(maxlen=10)
        self.last_time = None
        self._has_measurement = False

    def predict(self, dt):
        F = np.array([[1.0, dt], [0.0, 1.0]])
        self.state = F @ self.state
        self.state[0] = np.arctan2(np.sin(self.state[0]), np.cos(self.state[0]))
        self.P = F @ self.P @ F.T + self.Q

    def update(self, measurement_deg, current_time=None, dt=None):
        if dt is None:
            if isinstance(current_time, (int, float)):
                dt = float(current_time)
            elif self.last_time is not None and current_time is not None:
                dt = (current_time - self.last_time).nanoseconds / 1e9
            else:
                dt = 0.1
        self.last_time = current_time

        measurement_rad = np.radians(measurement_deg)
        measurement_rad = np.arctan2(np.sin(measurement_rad), np.cos(measurement_rad))
        if not self._has_measurement:
            self.state[0] = measurement_rad
            self.steady_wind = measurement_rad
            self._has_measurement = True
            return
        self.predict(dt)

        H = np.array([[1.0, 0.0]])
        innovation = measurement_rad - self.state[0]
        innovation = np.arctan2(np.sin(innovation), np.cos(innovation))
        self.innovation_history.append(abs(innovation))

        steady_innovation = measurement_rad - self.steady_wind
        steady_innovation = np.arctan2(np.sin(steady_innovation), np.cos(steady_innovation))

        if abs(innovation) > self.innovation_threshold or abs(steady_innovation) > self.innovation_threshold:
            self.gust_counter += 1
        else:
            self.gust_counter = max(0, self.gust_counter - 1)

        self.gust_detected = self.gust_counter >= self.gust_threshold_count
        self.Q = self.Q_gust.copy() if self.gust_detected else self.Q_base.copy()
        if len(self.innovation_history) >= 5:
            innovation_std = np.std(self.innovation_history)
            self.Q = self.Q_base * (1.0 + innovation_std / 0.1)

        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T / S
        self.state = self.state + K.flatten() * innovation
        self.state[0] = np.arctan2(np.sin(self.state[0]), np.cos(self.state[0]))
        self.P = (np.eye(2) - np.outer(K, H)) @ self.P

        if not self.gust_detected:
            diff = self.state[0] - self.steady_wind
            diff = np.arctan2(np.sin(diff), np.cos(diff))
            self.steady_wind += self.steady_wind_alpha * diff
            self.steady_wind = np.arctan2(np.sin(self.steady_wind), np.cos(self.steady_wind))

    def get_wind(self):
        current_deg = np.degrees(self.state[0]) % 360.0
        steady_deg = np.degrees(self.steady_wind) % 360.0
        return {
            "current": current_deg,
            "steady": steady_deg,
            "rate": np.degrees(self.state[1]),
            "gust_detected": self.gust_detected,
            "uncertainty": np.degrees(np.sqrt(max(self.P[0, 0], 0.0))),
            "innovation_std": np.degrees(np.std(self.innovation_history)) if self.innovation_history else 0.0,
        }
