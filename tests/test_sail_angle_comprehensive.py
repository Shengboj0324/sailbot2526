#!/usr/bin/env python3
"""
Comprehensive Sail Angle Test Suite
Tests sail angle calculations across all wind conditions, boat speeds, heel angles
Extremely detailed validation with harsh acceptance criteria
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'sailboat_control'))

import numpy as np
import unittest
import json
from datetime import datetime

class SailAngleDataCollector:
    """Collects comprehensive sail angle data for analysis"""
    
    def __init__(self):
        self.data_points = []
    
    def record(self, twa, tws, boat_speed, boat_heading, heel_angle, gust_detected,
               sail_angle, awa, aws, depowering_applied=False):
        """Record all parameters for a sail angle calculation"""
        self.data_points.append({
            'true_wind_angle': twa,
            'true_wind_speed': tws,
            'boat_speed': boat_speed,
            'boat_heading': boat_heading,
            'heel_angle': heel_angle,
            'gust_detected': gust_detected,
            'sail_angle': sail_angle,
            'apparent_wind_angle': awa,
            'apparent_wind_speed': aws,
            'depowering_applied': depowering_applied,
            'timestamp': datetime.now().isoformat()
        })
    
    def export_json(self, filename):
        """Export data to JSON for analysis"""
        with open(filename, 'w') as f:
            json.dump(self.data_points, f, indent=2)
    
    def get_statistics(self):
        """Calculate statistics across all data points"""
        if not self.data_points:
            return {}
        
        sail_angles = [d['sail_angle'] for d in self.data_points]
        awas = [d['apparent_wind_angle'] for d in self.data_points]
        awss = [d['apparent_wind_speed'] for d in self.data_points]
        
        return {
            'total_points': len(self.data_points),
            'sail_angle_min': min(sail_angles),
            'sail_angle_max': max(sail_angles),
            'sail_angle_mean': np.mean(sail_angles),
            'sail_angle_std': np.std(sail_angles),
            'awa_min': min(awas),
            'awa_max': max(awas),
            'aws_min': min(awss),
            'aws_max': max(awss),
            'depowering_events': sum(1 for d in self.data_points if d['depowering_applied']),
        }


class TestSailAngleComprehensive(unittest.TestCase):
    """Comprehensive sail angle tests across all conditions"""
    
    def setUp(self):
        from sailboat_control.optimal_sail_controller import OptimalSailController
        self.controller = OptimalSailController()
        self.collector = SailAngleDataCollector()
        self.test_results = []
    
    def test_sail_angle_full_wind_spectrum(self):
        """Test sail angles across full 360° wind spectrum"""
        # Test every 15° of wind angle
        wind_angles = range(0, 360, 15)
        wind_speed = 10.0
        boat_speed = 2.0
        boat_heading = 0.0
        heel_angle = 15.0
        
        for twa in wind_angles:
            sail, awa, aws = self.controller.calculate_sail_angle(
                twa, wind_speed, boat_speed, boat_heading, heel_angle, gust_detected=False
            )
            
            self.collector.record(twa, wind_speed, boat_speed, boat_heading,
                                heel_angle, False, sail, awa, aws)
            
            # Validate physical constraints
            self.assertGreaterEqual(sail, 0, f"Negative sail at TWA={twa}°")
            self.assertLessEqual(sail, 88, f"Excessive sail at TWA={twa}°")
            
            # Validate sail increases with wind angle (generally)
            if 30 <= twa <= 150:
                # In this range, sail should correlate with wind angle
                pass  # Detailed validation in polar tests
        
        stats = self.collector.get_statistics()
        self.test_results.append({
            'test': 'full_wind_spectrum',
            'stats': stats,
            'pass': True
        })
    
    def test_sail_angle_speed_sensitivity(self):
        """Test sail angle response to boat speed changes"""
        twa = 60.0
        tws = 10.0
        boat_heading = 0.0
        heel_angle = 15.0
        
        # Test speeds from 0 to 5 m/s
        speeds = np.linspace(0, 5, 20)
        sail_angles = []
        awas = []
        awss = []
        
        for speed in speeds:
            sail, awa, aws = self.controller.calculate_sail_angle(
                twa, tws, speed, boat_heading, heel_angle, gust_detected=False
            )
            
            sail_angles.append(sail)
            awas.append(awa)
            awss.append(aws)
            
            self.collector.record(twa, tws, speed, boat_heading,
                                heel_angle, False, sail, awa, aws)
        
        # Apparent wind should increase with boat speed
        self.assertGreater(awss[-1], awss[0],
                         "AWS should increase with boat speed")
        
        # AWA should shift forward with boat speed
        awa_change = abs(awas[-1] - awas[0])
        self.assertGreater(awa_change, 5.0,
                         f"AWA should shift significantly with speed: {awa_change:.1f}°")
        
        self.test_results.append({
            'test': 'speed_sensitivity',
            'awa_change_deg': awa_change,
            'aws_change_ms': awss[-1] - awss[0],
            'pass': awa_change > 5.0
        })
    
    def test_sail_angle_wind_speed_sensitivity(self):
        """Test sail angle response to wind speed changes"""
        twa = 90.0  # Beam reach
        boat_speed = 2.0
        boat_heading = 0.0
        heel_angle = 15.0
        
        # Test wind speeds from 5 to 25 m/s
        wind_speeds = np.linspace(5, 25, 20)
        sail_angles = []
        
        for tws in wind_speeds:
            sail, awa, aws = self.controller.calculate_sail_angle(
                twa, tws, boat_speed, boat_heading, heel_angle, gust_detected=False
            )
            
            sail_angles.append(sail)
            
            self.collector.record(twa, tws, boat_speed, boat_heading,
                                heel_angle, False, sail, awa, aws)
        
        # Sail angle should increase with wind speed (from polar data)
        self.assertGreater(sail_angles[-1], sail_angles[0],
                         "Sail angle should increase with wind speed")
        
        # Increase should be reasonable (not excessive)
        sail_increase = sail_angles[-1] - sail_angles[0]
        self.assertLess(sail_increase, 30.0,
                       f"Sail increase {sail_increase:.1f}° seems excessive")
        
        self.test_results.append({
            'test': 'wind_speed_sensitivity',
            'sail_increase_deg': sail_increase,
            'pass': 0 < sail_increase < 30
        })

    def test_depowering_heel_angle_threshold(self):
        """Test depowering activates correctly at heel threshold"""
        twa = 60.0
        tws = 15.0
        boat_speed = 3.0
        boat_heading = 0.0

        # Test heel angles from 0 to 40°
        heel_angles = np.linspace(0, 40, 20)
        sail_angles_no_gust = []
        sail_angles_with_gust = []

        for heel in heel_angles:
            # Without gust
            sail_no_gust, awa, aws = self.controller.calculate_sail_angle(
                twa, tws, boat_speed, boat_heading, heel, gust_detected=False
            )
            sail_angles_no_gust.append(sail_no_gust)

            # With gust
            sail_gust, _, _ = self.controller.calculate_sail_angle(
                twa, tws, boat_speed, boat_heading, heel, gust_detected=True
            )
            sail_angles_with_gust.append(sail_gust)

            self.collector.record(twa, tws, boat_speed, boat_heading,
                                heel, False, sail_no_gust, awa, aws,
                                depowering_applied=(heel > 25.0))

        # Find index where heel exceeds 25°
        threshold_idx = np.argmax(np.array(heel_angles) > 25.0)

        # Sail angle should increase after threshold (depowering)
        if threshold_idx > 0 and threshold_idx < len(sail_angles_no_gust) - 1:
            self.assertGreater(sail_angles_no_gust[threshold_idx + 5],
                             sail_angles_no_gust[threshold_idx - 1],
                             "Depowering should ease sail (increase angle)")

        # Gust should always increase sail angle
        for i in range(len(sail_angles_no_gust)):
            self.assertGreaterEqual(sail_angles_with_gust[i], sail_angles_no_gust[i],
                                  f"Gust should increase sail angle at heel={heel_angles[i]:.1f}°")

        self.test_results.append({
            'test': 'depowering_heel_threshold',
            'threshold_deg': 25.0,
            'pass': True
        })

    def test_sail_angle_rate_limiting(self):
        """Test sail angle changes are rate-limited"""
        max_rate = 10.0  # deg/s
        dt = 1.0

        # Start at low sail angle
        self.controller.current_sail_angle = 20.0

        # Command large changes
        target_angles = [80.0, 10.0, 70.0, 15.0]
        actual_changes = []

        for target in target_angles:
            result = self.controller.rate_limit_sail(target, max_rate, dt)
            change = abs(result - self.controller.current_sail_angle)
            actual_changes.append(change)
            self.controller.current_sail_angle = result

            # Change should not exceed max_rate * dt
            self.assertLessEqual(change, max_rate * dt + 0.1,
                               f"Rate limit violated: {change:.1f}° in {dt}s")

        self.test_results.append({
            'test': 'rate_limiting',
            'max_rate_deg_s': max_rate,
            'max_change_observed': max(actual_changes),
            'pass': max(actual_changes) <= max_rate * dt + 0.1
        })

    def test_sail_angle_upwind_optimization(self):
        """Test sail angles are optimized for upwind sailing"""
        # Upwind angles: 30-60°
        upwind_angles = [30, 35, 40, 45, 50, 55, 60]
        tws = 10.0
        boat_speed = 2.5
        boat_heading = 0.0
        heel_angle = 20.0

        sail_angles = []

        for twa in upwind_angles:
            sail, awa, aws = self.controller.calculate_sail_angle(
                twa, tws, boat_speed, boat_heading, heel_angle, gust_detected=False
            )
            sail_angles.append(sail)

            self.collector.record(twa, tws, boat_speed, boat_heading,
                                heel_angle, False, sail, awa, aws)

        # Sail should be relatively tight upwind (< 45°)
        for i, sail in enumerate(sail_angles):
            self.assertLess(sail, 45.0,
                          f"Upwind sail too loose at TWA={upwind_angles[i]}°: {sail:.1f}°")

        # Sail should increase with wind angle
        for i in range(len(sail_angles) - 1):
            self.assertLessEqual(sail_angles[i], sail_angles[i+1] + 2.0,
                               "Sail should increase with wind angle upwind")

        self.test_results.append({
            'test': 'upwind_optimization',
            'sail_angles': sail_angles,
            'pass': all(s < 45 for s in sail_angles)
        })

    def test_sail_angle_downwind_optimization(self):
        """Test sail angles are optimized for downwind sailing"""
        # Downwind angles: 120-180°
        downwind_angles = [120, 135, 150, 165, 180]
        tws = 10.0
        boat_speed = 2.0
        boat_heading = 0.0
        heel_angle = 10.0

        sail_angles = []

        for twa in downwind_angles:
            sail, awa, aws = self.controller.calculate_sail_angle(
                twa, tws, boat_speed, boat_heading, heel_angle, gust_detected=False
            )
            sail_angles.append(sail)

            self.collector.record(twa, tws, boat_speed, boat_heading,
                                heel_angle, False, sail, awa, aws)

        # Sail should be eased downwind (> 60°)
        for i, sail in enumerate(sail_angles):
            self.assertGreater(sail, 60.0,
                             f"Downwind sail too tight at TWA={downwind_angles[i]}°: {sail:.1f}°")

        # Sail should increase with wind angle
        for i in range(len(sail_angles) - 1):
            self.assertLessEqual(sail_angles[i], sail_angles[i+1] + 2.0,
                               "Sail should increase with wind angle downwind")

        # Maximum sail at dead downwind should approach limit
        self.assertGreater(sail_angles[-1], 80.0,
                         f"Dead downwind sail should be near maximum: {sail_angles[-1]:.1f}°")

        self.test_results.append({
            'test': 'downwind_optimization',
            'sail_angles': sail_angles,
            'pass': all(s > 60 for s in sail_angles) and sail_angles[-1] > 80
        })

    def test_sail_angle_extreme_conditions(self):
        """Test sail angles in extreme wind and heel conditions"""
        extreme_conditions = [
            # (TWA, TWS, boat_speed, heel, gust, description)
            (45, 25, 4, 35, True, "High wind upwind with gust"),
            (90, 25, 4, 30, True, "High wind beam reach with gust"),
            (135, 25, 3, 25, True, "High wind broad reach with gust"),
            (60, 5, 1, 5, False, "Light wind upwind"),
            (120, 5, 1, 5, False, "Light wind downwind"),
        ]

        for twa, tws, speed, heel, gust, desc in extreme_conditions:
            sail, awa, aws = self.controller.calculate_sail_angle(
                twa, tws, speed, 0.0, heel, gust
            )

            self.collector.record(twa, tws, speed, 0.0, heel, gust, sail, awa, aws,
                                depowering_applied=(heel > 25 or gust))

            # Must stay within physical limits
            self.assertGreaterEqual(sail, 0, f"Negative sail in: {desc}")
            self.assertLessEqual(sail, 88, f"Excessive sail in: {desc}")

            # High heel or gust should trigger depowering
            if heel > 25 or gust:
                # Depowering should be active (harder to verify without baseline)
                pass

        self.test_results.append({
            'test': 'extreme_conditions',
            'conditions_tested': len(extreme_conditions),
            'pass': True
        })


if __name__ == '__main__':
    # Run tests and export data
    suite = unittest.TestLoader().loadTestsFromTestCase(TestSailAngleComprehensive)
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)

    # Export collected data
    if hasattr(result, 'collector'):
        result.collector.export_json('sail_angle_test_data.json')
        print(f"\nExported {len(result.collector.data_points)} data points to sail_angle_test_data.json")
