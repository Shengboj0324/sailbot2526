# Comprehensive Sailbot2526 Test Report
**Generated:** 2026-03-16 08:53:59  
**Duration:** 0.42 seconds  
**Test Environment:** Python 3.9.13

---

## Executive Summary

**Overall Results:**
- **Total Tests:** 40
- **Passed:** 7 ✅
- **Failed:** 33 ❌
- **Success Rate:** 17.5%

**Status:** ❌ FAIL

---

## Test Suite Results

### ❌ Comprehensive Regression Tests
- Tests Run: 26
- Passed: 7
- Failed: 3
- Errors: [{'test': 'test_covariance_positive_definite (test_comprehensive_regression.TestExtendedKalmanFilterHarsh)', 'traceback': 'Traceback (most recent call last):\n  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_comprehensive_regression.py", line 164, in setUp\n    from sailboat_control.state_estimator import ExtendedKalmanFilter\n  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/../src/sailboat_control/sailboat_control/state_estimator.py", line 7, in <module>\n    import rclpy\nModuleNotFoundError: No module named \'rclpy\'\n'}, {'test': 'test_gps_fusion_accuracy (test_comprehensive_regression.TestExtendedKalmanFilterHarsh)', 'traceback': 'Traceback (most recent call last):\n  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_comprehensive_regression.py", line 164, in setUp\n    from sailboat_control.state_estimator import ExtendedKalmanFilter\n  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/../src/sailboat_control/sailboat_control/state_estimator.py", line 7, in <module>\n    import rclpy\nModuleNotFoundError: No module named \'rclpy\'\n'}, {'test': 'test_imu_heading_fusion_accuracy (test_comprehensive_regression.TestExtendedKalmanFilterHarsh)', 'traceback': 'Traceback (most recent call last):\n  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_comprehensive_regression.py", line 164, in setUp\n    from sailboat_control.state_estimator import ExtendedKalmanFilter\n  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/../src/sailboat_control/sailboat_control/state_estimator.py", line 7, in <module>\n    import rclpy\nModuleNotFoundError: No module named \'rclpy\'\n'}, {'test': 'test_initial_state_zero (test_comprehensive_regression.TestExtendedKalmanFilterHarsh)', 'traceback': 'Traceback (most recent call last):\n  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_comprehensive_regression.py", line 164, in setUp\n    from sailboat_control.state_estimator import ExtendedKalmanFilter\n  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/../src/sailboat_control/sailboat_control/state_estimator.py", line 7, in <module>\n    import rclpy\nModuleNotFoundError: No module named \'rclpy\'\n'}, {'test': 'test_prediction_step_stability (test_comprehensive_regression.TestExtendedKalmanFilterHarsh)', 'traceback': 'Traceback (most recent call last):\n  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_comprehensive_regression.py", line 164, in setUp\n    from sailboat_control.state_estimator import ExtendedKalmanFilter\n  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/../src/sailboat_control/sailboat_control/state_estimator.py", line 7, in <module>\n    import rclpy\nModuleNotFoundError: No module named \'rclpy\'\n'}, {'test': 'test_velocity_estimation_accuracy (test_comprehensive_regression.TestExtendedKalmanFilterHarsh)', 'traceback': 'Traceback (most recent call last):\n  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_comprehensive_regression.py", line 164, in setUp\n    from sailboat_control.state_estimator import ExtendedKalmanFilter\n  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/../src/sailboat_control/sailboat_control/state_estimator.py", line 7, in <module>\n    import rclpy\nModuleNotFoundError: No module named \'rclpy\'\n'}, {'test': 'test_apparent_wind_calculation_accuracy (test_comprehensive_regression.TestOptimalSailControllerHarsh)', 'traceback': 'Traceback (most recent call last):\n  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_comprehensive_regression.py", line 263, in setUp\n    from sailboat_control.optimal_sail_controller import OptimalSailController\n  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/../src/sailboat_control/sailboat_control/optimal_sail_controller.py", line 6, in <module>\n    import rclpy\nModuleNotFoundError: No module named \'rclpy\'\n'}, {'test': 'test_depowering_logic_heel_threshold (test_comprehensive_regression.TestOptimalSailControllerHarsh)', 'traceback': 'Traceback (most recent call last):\n  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_comprehensive_regression.py", line 263, in setUp\n    from sailboat_control.optimal_sail_controller import OptimalSailController\n  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/../src/sailboat_control/sailboat_control/optimal_sail_controller.py", line 6, in <module>\n    import rclpy\nModuleNotFoundError: No module named \'rclpy\'\n'}, {'test': 'test_polar_interpolation_boundaries (test_comprehensive_regression.TestOptimalSailControllerHarsh)', 'traceback': 'Traceback (most recent call last):\n  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_comprehensive_regression.py", line 263, in setUp\n    from sailboat_control.optimal_sail_controller import OptimalSailController\n  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/../src/sailboat_control/sailboat_control/optimal_sail_controller.py", line 6, in <module>\n    import rclpy\nModuleNotFoundError: No module named \'rclpy\'\n'}, {'test': 'test_rate_limiting_prevents_shock_loads (test_comprehensive_regression.TestOptimalSailControllerHarsh)', 'traceback': 'Traceback (most recent call last):\n  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_comprehensive_regression.py", line 263, in setUp\n    from sailboat_control.optimal_sail_controller import OptimalSailController\n  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/../src/sailboat_control/sailboat_control/optimal_sail_controller.py", line 6, in <module>\n    import rclpy\nModuleNotFoundError: No module named \'rclpy\'\n'}, {'test': 'test_sail_angle_physical_limits (test_comprehensive_regression.TestOptimalSailControllerHarsh)', 'traceback': 'Traceback (most recent call last):\n  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_comprehensive_regression.py", line 263, in setUp\n    from sailboat_control.optimal_sail_controller import OptimalSailController\n  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/../src/sailboat_control/sailboat_control/optimal_sail_controller.py", line 6, in <module>\n    import rclpy\nModuleNotFoundError: No module named \'rclpy\'\n'}, {'test': 'test_circular_statistics_wraparound (test_comprehensive_regression.TestWindFilterHarsh)', 'traceback': 'Traceback (most recent call last):\n  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_comprehensive_regression.py", line 363, in setUp\n    from sailboat_control.wind_smoother import AdaptiveWindKalmanFilter\nModuleNotFoundError: No module named \'sailboat_control.wind_smoother\'\n'}, {'test': 'test_gust_detection_persistence (test_comprehensive_regression.TestWindFilterHarsh)', 'traceback': 'Traceback (most recent call last):\n  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_comprehensive_regression.py", line 363, in setUp\n    from sailboat_control.wind_smoother import AdaptiveWindKalmanFilter\nModuleNotFoundError: No module named \'sailboat_control.wind_smoother\'\n'}, {'test': 'test_gust_detection_threshold (test_comprehensive_regression.TestWindFilterHarsh)', 'traceback': 'Traceback (most recent call last):\n  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_comprehensive_regression.py", line 363, in setUp\n    from sailboat_control.wind_smoother import AdaptiveWindKalmanFilter\nModuleNotFoundError: No module named \'sailboat_control.wind_smoother\'\n'}, {'test': 'test_innovation_variance_adaptation (test_comprehensive_regression.TestWindFilterHarsh)', 'traceback': 'Traceback (most recent call last):\n  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_comprehensive_regression.py", line 363, in setUp\n    from sailboat_control.wind_smoother import AdaptiveWindKalmanFilter\nModuleNotFoundError: No module named \'sailboat_control.wind_smoother\'\n'}, {'test': 'test_steady_wind_low_pass_filtering (test_comprehensive_regression.TestWindFilterHarsh)', 'traceback': 'Traceback (most recent call last):\n  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_comprehensive_regression.py", line 363, in setUp\n    from sailboat_control.wind_smoother import AdaptiveWindKalmanFilter\nModuleNotFoundError: No module named \'sailboat_control.wind_smoother\'\n'}]
- Success Rate: 26.9%

**Failures:**
```
test_gain_scheduling_speed_range (test_comprehensive_regression.TestAdaptivePIDHarsh)
Traceback (most recent call last):
  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_comprehensive_regression.py", line 56, in test_gain_scheduling_speed_range
    self.assertLess(outputs[i], outputs[i+1],
AssertionError: 21.0 not less than 21.0 : Output should increase with speed: 0.5 vs 1.0


test_heading_error_wraparound (test_comprehensive_regression.TestAdaptivePIDHarsh)
Traceback (most recent call last):
  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_comprehensive_regression.py", line 122, in test_heading_error_wraparound
    self.assertAlmostEqual(error, expected, places=1,
AssertionError: -180 != 180 within 1 places (360 difference) : Heading error wrong: 0° - 180° = -180° (expected 180°)


test_integral_windup_protection (test_comprehensive_regression.TestAdaptivePIDHarsh)
Traceback (most recent call last):
  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_comprehensive_regression.py", line 69, in test_integral_windup_protection
    self.assertLessEqual(abs(self.pid.integral), self.pid.integral_max,
AssertionError: 7028.07558202668 not less than or equal to 100.0 : Integral term exceeded maximum


```

**Errors:**
```
test_covariance_positive_definite (test_comprehensive_regression.TestExtendedKalmanFilterHarsh)
Traceback (most recent call last):
  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_comprehensive_regression.py", line 164, in setUp
    from sailboat_control.state_estimator import ExtendedKalmanFilter
  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/../src/sailboat_control/sailboat_control/state_estimator.py", line 7, in <module>
    import rclpy
ModuleNotFoundError: No module named 'rclpy'


test_gps_fusion_accuracy (test_comprehensive_regression.TestExtendedKalmanFilterHarsh)
Traceback (most recent call last):
  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_comprehensive_regression.py", line 164, in setUp
    from sailboat_control.state_estimator import ExtendedKalmanFilter
  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/../src/sailboat_control/sailboat_control/state_estimator.py", line 7, in <module>
    import rclpy
ModuleNotFoundError: No module named 'rclpy'


test_imu_heading_fusion_accuracy (test_comprehensive_regression.TestExtendedKalmanFilterHarsh)
Traceback (most recent call last):
  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_comprehensive_regression.py", line 164, in setUp
    from sailboat_control.state_estimator import ExtendedKalmanFilter
  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/../src/sailboat_control/sailboat_control/state_estimator.py", line 7, in <module>
    import rclpy
ModuleNotFoundError: No module named 'rclpy'


test_initial_state_zero (test_comprehensive_regression.TestExtendedKalmanFilterHarsh)
Traceback (most recent call last):
  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_comprehensive_regression.py", line 164, in setUp
    from sailboat_control.state_estimator import ExtendedKalmanFilter
  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/../src/sailboat_control/sailboat_control/state_estimator.py", line 7, in <module>
    import rclpy
ModuleNotFoundError: No module named 'rclpy'


test_prediction_step_stability (test_comprehensive_regression.TestExtendedKalmanFilterHarsh)
Traceback (most recent call last):
  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_comprehensive_regression.py", line 164, in setUp
    from sailboat_control.state_estimator import ExtendedKalmanFilter
  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/../src/sailboat_control/sailboat_control/state_estimator.py", line 7, in <module>
    import rclpy
ModuleNotFoundError: No module named 'rclpy'


test_velocity_estimation_accuracy (test_comprehensive_regression.TestExtendedKalmanFilterHarsh)
Traceback (most recent call last):
  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_comprehensive_regression.py", line 164, in setUp
    from sailboat_control.state_estimator import ExtendedKalmanFilter
  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/../src/sailboat_control/sailboat_control/state_estimator.py", line 7, in <module>
    import rclpy
ModuleNotFoundError: No module named 'rclpy'


test_apparent_wind_calculation_accuracy (test_comprehensive_regression.TestOptimalSailControllerHarsh)
Traceback (most recent call last):
  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_comprehensive_regression.py", line 263, in setUp
    from sailboat_control.optimal_sail_controller import OptimalSailController
  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/../src/sailboat_control/sailboat_control/optimal_sail_controller.py", line 6, in <module>
    import rclpy
ModuleNotFoundError: No module named 'rclpy'


test_depowering_logic_heel_threshold (test_comprehensive_regression.TestOptimalSailControllerHarsh)
Traceback (most recent call last):
  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_comprehensive_regression.py", line 263, in setUp
    from sailboat_control.optimal_sail_controller import OptimalSailController
  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/../src/sailboat_control/sailboat_control/optimal_sail_controller.py", line 6, in <module>
    import rclpy
ModuleNotFoundError: No module named 'rclpy'


test_polar_interpolation_boundaries (test_comprehensive_regression.TestOptimalSailControllerHarsh)
Traceback (most recent call last):
  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_comprehensive_regression.py", line 263, in setUp
    from sailboat_control.optimal_sail_controller import OptimalSailController
  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/../src/sailboat_control/sailboat_control/optimal_sail_controller.py", line 6, in <module>
    import rclpy
ModuleNotFoundError: No module named 'rclpy'


test_rate_limiting_prevents_shock_loads (test_comprehensive_regression.TestOptimalSailControllerHarsh)
Traceback (most recent call last):
  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_comprehensive_regression.py", line 263, in setUp
    from sailboat_control.optimal_sail_controller import OptimalSailController
  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/../src/sailboat_control/sailboat_control/optimal_sail_controller.py", line 6, in <module>
    import rclpy
ModuleNotFoundError: No module named 'rclpy'


test_sail_angle_physical_limits (test_comprehensive_regression.TestOptimalSailControllerHarsh)
Traceback (most recent call last):
  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_comprehensive_regression.py", line 263, in setUp
    from sailboat_control.optimal_sail_controller import OptimalSailController
  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/../src/sailboat_control/sailboat_control/optimal_sail_controller.py", line 6, in <module>
    import rclpy
ModuleNotFoundError: No module named 'rclpy'


test_circular_statistics_wraparound (test_comprehensive_regression.TestWindFilterHarsh)
Traceback (most recent call last):
  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_comprehensive_regression.py", line 363, in setUp
    from sailboat_control.wind_smoother import AdaptiveWindKalmanFilter
ModuleNotFoundError: No module named 'sailboat_control.wind_smoother'


test_gust_detection_persistence (test_comprehensive_regression.TestWindFilterHarsh)
Traceback (most recent call last):
  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_comprehensive_regression.py", line 363, in setUp
    from sailboat_control.wind_smoother import AdaptiveWindKalmanFilter
ModuleNotFoundError: No module named 'sailboat_control.wind_smoother'


test_gust_detection_threshold (test_comprehensive_regression.TestWindFilterHarsh)
Traceback (most recent call last):
  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_comprehensive_regression.py", line 363, in setUp
    from sailboat_control.wind_smoother import AdaptiveWindKalmanFilter
ModuleNotFoundError: No module named 'sailboat_control.wind_smoother'


test_innovation_variance_adaptation (test_comprehensive_regression.TestWindFilterHarsh)
Traceback (most recent call last):
  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_comprehensive_regression.py", line 363, in setUp
    from sailboat_control.wind_smoother import AdaptiveWindKalmanFilter
ModuleNotFoundError: No module named 'sailboat_control.wind_smoother'


test_steady_wind_low_pass_filtering (test_comprehensive_regression.TestWindFilterHarsh)
Traceback (most recent call last):
  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_comprehensive_regression.py", line 363, in setUp
    from sailboat_control.wind_smoother import AdaptiveWindKalmanFilter
ModuleNotFoundError: No module named 'sailboat_control.wind_smoother'


```

### ❌ Position Tracking Tests
- Tests Run: 6
- Passed: 0
- Failed: 0
- Errors: [{'test': 'test_circular_path_tracking_accuracy (test_position_tracking.TestPositionTrackingHarsh)', 'traceback': 'Traceback (most recent call last):\n  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_position_tracking.py", line 101, in setUp\n    from sailboat_control.state_estimator import ExtendedKalmanFilter\n  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/../src/sailboat_control/sailboat_control/state_estimator.py", line 7, in <module>\n    import rclpy\nModuleNotFoundError: No module named \'rclpy\'\n'}, {'test': 'test_gps_dropout_recovery (test_position_tracking.TestPositionTrackingHarsh)', 'traceback': 'Traceback (most recent call last):\n  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_position_tracking.py", line 101, in setUp\n    from sailboat_control.state_estimator import ExtendedKalmanFilter\n  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/../src/sailboat_control/sailboat_control/state_estimator.py", line 7, in <module>\n    import rclpy\nModuleNotFoundError: No module named \'rclpy\'\n'}, {'test': 'test_straight_line_tracking_accuracy (test_position_tracking.TestPositionTrackingHarsh)', 'traceback': 'Traceback (most recent call last):\n  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_position_tracking.py", line 101, in setUp\n    from sailboat_control.state_estimator import ExtendedKalmanFilter\n  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/../src/sailboat_control/sailboat_control/state_estimator.py", line 7, in <module>\n    import rclpy\nModuleNotFoundError: No module named \'rclpy\'\n'}, {'test': 'test_zigzag_maneuver_tracking (test_position_tracking.TestPositionTrackingHarsh)', 'traceback': 'Traceback (most recent call last):\n  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_position_tracking.py", line 101, in setUp\n    from sailboat_control.state_estimator import ExtendedKalmanFilter\n  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/../src/sailboat_control/sailboat_control/state_estimator.py", line 7, in <module>\n    import rclpy\nModuleNotFoundError: No module named \'rclpy\'\n'}, {'test': 'test_direct_waypoint_approach_accuracy (test_position_tracking.TestWaypointNavigationHarsh)', 'traceback': 'Traceback (most recent call last):\n  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_position_tracking.py", line 298, in setUp\n    from path_planning.path_planning.leg import Leg\nModuleNotFoundError: No module named \'path_planning.path_planning\'\n'}, {'test': 'test_upwind_tacking_waypoint_accuracy (test_position_tracking.TestWaypointNavigationHarsh)', 'traceback': 'Traceback (most recent call last):\n  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_position_tracking.py", line 298, in setUp\n    from path_planning.path_planning.leg import Leg\nModuleNotFoundError: No module named \'path_planning.path_planning\'\n'}]
- Success Rate: 0.0%

**Errors:**
```
test_circular_path_tracking_accuracy (test_position_tracking.TestPositionTrackingHarsh)
Traceback (most recent call last):
  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_position_tracking.py", line 101, in setUp
    from sailboat_control.state_estimator import ExtendedKalmanFilter
  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/../src/sailboat_control/sailboat_control/state_estimator.py", line 7, in <module>
    import rclpy
ModuleNotFoundError: No module named 'rclpy'


test_gps_dropout_recovery (test_position_tracking.TestPositionTrackingHarsh)
Traceback (most recent call last):
  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_position_tracking.py", line 101, in setUp
    from sailboat_control.state_estimator import ExtendedKalmanFilter
  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/../src/sailboat_control/sailboat_control/state_estimator.py", line 7, in <module>
    import rclpy
ModuleNotFoundError: No module named 'rclpy'


test_straight_line_tracking_accuracy (test_position_tracking.TestPositionTrackingHarsh)
Traceback (most recent call last):
  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_position_tracking.py", line 101, in setUp
    from sailboat_control.state_estimator import ExtendedKalmanFilter
  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/../src/sailboat_control/sailboat_control/state_estimator.py", line 7, in <module>
    import rclpy
ModuleNotFoundError: No module named 'rclpy'


test_zigzag_maneuver_tracking (test_position_tracking.TestPositionTrackingHarsh)
Traceback (most recent call last):
  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_position_tracking.py", line 101, in setUp
    from sailboat_control.state_estimator import ExtendedKalmanFilter
  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/../src/sailboat_control/sailboat_control/state_estimator.py", line 7, in <module>
    import rclpy
ModuleNotFoundError: No module named 'rclpy'


test_direct_waypoint_approach_accuracy (test_position_tracking.TestWaypointNavigationHarsh)
Traceback (most recent call last):
  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_position_tracking.py", line 298, in setUp
    from path_planning.path_planning.leg import Leg
ModuleNotFoundError: No module named 'path_planning.path_planning'


test_upwind_tacking_waypoint_accuracy (test_position_tracking.TestWaypointNavigationHarsh)
Traceback (most recent call last):
  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_position_tracking.py", line 298, in setUp
    from path_planning.path_planning.leg import Leg
ModuleNotFoundError: No module named 'path_planning.path_planning'


```

### ❌ Sail Angle Comprehensive Tests
- Tests Run: 8
- Passed: 0
- Failed: 0
- Errors: [{'test': 'test_depowering_heel_angle_threshold (test_sail_angle_comprehensive.TestSailAngleComprehensive)', 'traceback': 'Traceback (most recent call last):\n  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_sail_angle_comprehensive.py", line 71, in setUp\n    from sailboat_control.optimal_sail_controller import OptimalSailController\n  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/../src/sailboat_control/sailboat_control/optimal_sail_controller.py", line 6, in <module>\n    import rclpy\nModuleNotFoundError: No module named \'rclpy\'\n'}, {'test': 'test_sail_angle_downwind_optimization (test_sail_angle_comprehensive.TestSailAngleComprehensive)', 'traceback': 'Traceback (most recent call last):\n  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_sail_angle_comprehensive.py", line 71, in setUp\n    from sailboat_control.optimal_sail_controller import OptimalSailController\n  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/../src/sailboat_control/sailboat_control/optimal_sail_controller.py", line 6, in <module>\n    import rclpy\nModuleNotFoundError: No module named \'rclpy\'\n'}, {'test': 'test_sail_angle_extreme_conditions (test_sail_angle_comprehensive.TestSailAngleComprehensive)', 'traceback': 'Traceback (most recent call last):\n  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_sail_angle_comprehensive.py", line 71, in setUp\n    from sailboat_control.optimal_sail_controller import OptimalSailController\n  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/../src/sailboat_control/sailboat_control/optimal_sail_controller.py", line 6, in <module>\n    import rclpy\nModuleNotFoundError: No module named \'rclpy\'\n'}, {'test': 'test_sail_angle_full_wind_spectrum (test_sail_angle_comprehensive.TestSailAngleComprehensive)', 'traceback': 'Traceback (most recent call last):\n  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_sail_angle_comprehensive.py", line 71, in setUp\n    from sailboat_control.optimal_sail_controller import OptimalSailController\n  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/../src/sailboat_control/sailboat_control/optimal_sail_controller.py", line 6, in <module>\n    import rclpy\nModuleNotFoundError: No module named \'rclpy\'\n'}, {'test': 'test_sail_angle_rate_limiting (test_sail_angle_comprehensive.TestSailAngleComprehensive)', 'traceback': 'Traceback (most recent call last):\n  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_sail_angle_comprehensive.py", line 71, in setUp\n    from sailboat_control.optimal_sail_controller import OptimalSailController\n  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/../src/sailboat_control/sailboat_control/optimal_sail_controller.py", line 6, in <module>\n    import rclpy\nModuleNotFoundError: No module named \'rclpy\'\n'}, {'test': 'test_sail_angle_speed_sensitivity (test_sail_angle_comprehensive.TestSailAngleComprehensive)', 'traceback': 'Traceback (most recent call last):\n  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_sail_angle_comprehensive.py", line 71, in setUp\n    from sailboat_control.optimal_sail_controller import OptimalSailController\n  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/../src/sailboat_control/sailboat_control/optimal_sail_controller.py", line 6, in <module>\n    import rclpy\nModuleNotFoundError: No module named \'rclpy\'\n'}, {'test': 'test_sail_angle_upwind_optimization (test_sail_angle_comprehensive.TestSailAngleComprehensive)', 'traceback': 'Traceback (most recent call last):\n  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_sail_angle_comprehensive.py", line 71, in setUp\n    from sailboat_control.optimal_sail_controller import OptimalSailController\n  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/../src/sailboat_control/sailboat_control/optimal_sail_controller.py", line 6, in <module>\n    import rclpy\nModuleNotFoundError: No module named \'rclpy\'\n'}, {'test': 'test_sail_angle_wind_speed_sensitivity (test_sail_angle_comprehensive.TestSailAngleComprehensive)', 'traceback': 'Traceback (most recent call last):\n  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_sail_angle_comprehensive.py", line 71, in setUp\n    from sailboat_control.optimal_sail_controller import OptimalSailController\n  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/../src/sailboat_control/sailboat_control/optimal_sail_controller.py", line 6, in <module>\n    import rclpy\nModuleNotFoundError: No module named \'rclpy\'\n'}]
- Success Rate: 0.0%

**Errors:**
```
test_depowering_heel_angle_threshold (test_sail_angle_comprehensive.TestSailAngleComprehensive)
Traceback (most recent call last):
  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_sail_angle_comprehensive.py", line 71, in setUp
    from sailboat_control.optimal_sail_controller import OptimalSailController
  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/../src/sailboat_control/sailboat_control/optimal_sail_controller.py", line 6, in <module>
    import rclpy
ModuleNotFoundError: No module named 'rclpy'


test_sail_angle_downwind_optimization (test_sail_angle_comprehensive.TestSailAngleComprehensive)
Traceback (most recent call last):
  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_sail_angle_comprehensive.py", line 71, in setUp
    from sailboat_control.optimal_sail_controller import OptimalSailController
  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/../src/sailboat_control/sailboat_control/optimal_sail_controller.py", line 6, in <module>
    import rclpy
ModuleNotFoundError: No module named 'rclpy'


test_sail_angle_extreme_conditions (test_sail_angle_comprehensive.TestSailAngleComprehensive)
Traceback (most recent call last):
  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_sail_angle_comprehensive.py", line 71, in setUp
    from sailboat_control.optimal_sail_controller import OptimalSailController
  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/../src/sailboat_control/sailboat_control/optimal_sail_controller.py", line 6, in <module>
    import rclpy
ModuleNotFoundError: No module named 'rclpy'


test_sail_angle_full_wind_spectrum (test_sail_angle_comprehensive.TestSailAngleComprehensive)
Traceback (most recent call last):
  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_sail_angle_comprehensive.py", line 71, in setUp
    from sailboat_control.optimal_sail_controller import OptimalSailController
  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/../src/sailboat_control/sailboat_control/optimal_sail_controller.py", line 6, in <module>
    import rclpy
ModuleNotFoundError: No module named 'rclpy'


test_sail_angle_rate_limiting (test_sail_angle_comprehensive.TestSailAngleComprehensive)
Traceback (most recent call last):
  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_sail_angle_comprehensive.py", line 71, in setUp
    from sailboat_control.optimal_sail_controller import OptimalSailController
  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/../src/sailboat_control/sailboat_control/optimal_sail_controller.py", line 6, in <module>
    import rclpy
ModuleNotFoundError: No module named 'rclpy'


test_sail_angle_speed_sensitivity (test_sail_angle_comprehensive.TestSailAngleComprehensive)
Traceback (most recent call last):
  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_sail_angle_comprehensive.py", line 71, in setUp
    from sailboat_control.optimal_sail_controller import OptimalSailController
  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/../src/sailboat_control/sailboat_control/optimal_sail_controller.py", line 6, in <module>
    import rclpy
ModuleNotFoundError: No module named 'rclpy'


test_sail_angle_upwind_optimization (test_sail_angle_comprehensive.TestSailAngleComprehensive)
Traceback (most recent call last):
  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_sail_angle_comprehensive.py", line 71, in setUp
    from sailboat_control.optimal_sail_controller import OptimalSailController
  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/../src/sailboat_control/sailboat_control/optimal_sail_controller.py", line 6, in <module>
    import rclpy
ModuleNotFoundError: No module named 'rclpy'


test_sail_angle_wind_speed_sensitivity (test_sail_angle_comprehensive.TestSailAngleComprehensive)
Traceback (most recent call last):
  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/test_sail_angle_comprehensive.py", line 71, in setUp
    from sailboat_control.optimal_sail_controller import OptimalSailController
  File "/Users/jiangshengbo/Desktop/sailbot2526/tests/../src/sailboat_control/sailboat_control/optimal_sail_controller.py", line 6, in <module>
    import rclpy
ModuleNotFoundError: No module named 'rclpy'


```

---

## Detailed Test Metrics

### Performance Metrics
- Average test execution time: 0.011s per test
- Total test coverage: 40 test cases
- Test suites executed: 3

### Quality Metrics
- Pass rate: 17.5%
- Failure rate: 82.5%
- Critical failures: 3

---

## Recommendations

❌ **CRITICAL** - Major issues require immediate attention

---

## Test Data Export

Detailed test data has been exported to JSON files:
- `test_results.json` - Complete test results
- `sail_angle_test_data.json` - Sail angle test data (if available)
- `position_tracking_data.json` - Position tracking data (if available)

---

**Report End**
