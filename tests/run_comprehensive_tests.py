#!/usr/bin/env python3
"""
Master Test Runner for Comprehensive Sailbot Regression Suite
Executes all test suites and generates detailed academic-quality report
"""
import sys
import os
import unittest
import json
from datetime import datetime
import traceback
# Add paths
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'sailboat_control'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'path_planning'))

class ComprehensiveTestReport:
    """Generates comprehensive test report for academic presentation"""
    
    def __init__(self):
        self.start_time = datetime.now()
        self.test_suites = []
        self.total_tests = 0
        self.passed_tests = 0
        self.failed_tests = 0
        self.errors = []
        self.detailed_results = []
    
    def add_suite_result(self, suite_name, result, test_data=None):
        """Add results from a test suite"""
        tests_run = result.testsRun
        failures = len(result.failures)
        errors = len(result.errors)
        passed = tests_run - failures - errors
        
        self.total_tests += tests_run
        self.passed_tests += passed
        self.failed_tests += failures + errors
        
        suite_info = {
            'name': suite_name,
            'tests_run': tests_run,
            'passed': passed,
            'failed': failures,
            'errors': errors,
            'success_rate': (passed / tests_run * 100) if tests_run > 0 else 0,
            'timestamp': datetime.now().isoformat()
        }
        
        # Add failure details
        if result.failures:
            suite_info['failures'] = [
                {'test': str(test), 'traceback': tb}
                for test, tb in result.failures
            ]
        
        if result.errors:
            suite_info['errors'] = [
                {'test': str(test), 'traceback': tb}
                for test, tb in result.errors
            ]
        
        if test_data:
            suite_info['test_data'] = test_data
        
        self.test_suites.append(suite_info)
    
    def generate_report(self, filename='COMPREHENSIVE_TEST_REPORT.md'):
        """Generate markdown report"""
        end_time = datetime.now()
        duration = (end_time - self.start_time).total_seconds()
        
        overall_success_rate = (self.passed_tests / self.total_tests * 100) if self.total_tests > 0 else 0
        
        report = f"""# Comprehensive Sailbot2526 Test Report
**Generated:** {end_time.strftime('%Y-%m-%d %H:%M:%S')}  
**Duration:** {duration:.2f} seconds  
**Test Environment:** Python {sys.version.split()[0]}

---

## Executive Summary

**Overall Results:**
- **Total Tests:** {self.total_tests}
- **Passed:** {self.passed_tests} ✅
- **Failed:** {self.failed_tests} ❌
- **Success Rate:** {overall_success_rate:.1f}%

**Status:** {'✅ PASS' if overall_success_rate >= 70 else '❌ FAIL'}

---

## Test Suite Results

"""
        
        for suite in self.test_suites:
            status_icon = '✅' if suite['success_rate'] >= 70 else '❌'
            report += f"""### {status_icon} {suite['name']}
- Tests Run: {suite['tests_run']}
- Passed: {suite['passed']}
- Failed: {suite['failed']}
- Errors: {suite['errors']}
- Success Rate: {suite['success_rate']:.1f}%

"""
            
            # Add failure details
            if 'failures' in suite:
                report += "**Failures:**\n```\n"
                for failure in suite['failures']:
                    report += f"{failure['test']}\n{failure['traceback']}\n\n"
                report += "```\n\n"
            
            if 'errors' in suite:
                report += "**Errors:**\n```\n"
                for error in suite['errors']:
                    report += f"{error['test']}\n{error['traceback']}\n\n"
                report += "```\n\n"
        
        report += f"""---

## Detailed Test Metrics

### Performance Metrics
- Average test execution time: {duration / self.total_tests:.3f}s per test
- Total test coverage: {self.total_tests} test cases
- Test suites executed: {len(self.test_suites)}

### Quality Metrics
- Pass rate: {overall_success_rate:.1f}%
- Failure rate: {(self.failed_tests / self.total_tests * 100) if self.total_tests > 0 else 0:.1f}%
- Critical failures: {sum(1 for s in self.test_suites if s['success_rate'] < 50)}

---

## Recommendations

"""
        
        if overall_success_rate >= 90:
            report += "✅ **EXCELLENT** - System ready for deployment\n"
        elif overall_success_rate >= 70:
            report += "⚠️ **ACCEPTABLE** - Minor issues to address before deployment\n"
        else:
            report += "❌ **CRITICAL** - Major issues require immediate attention\n"
        
        report += f"""
---

## Test Data Export

Detailed test data has been exported to JSON files:
- `test_results.json` - Complete test results
- `sail_angle_test_data.json` - Sail angle test data (if available)
- `position_tracking_data.json` - Position tracking data (if available)

---

**Report End**
"""
        
        # Write report
        with open(filename, 'w') as f:
            f.write(report)
        
        # Export JSON data
        json_data = {
            'summary': {
                'total_tests': self.total_tests,
                'passed': self.passed_tests,
                'failed': self.failed_tests,
                'success_rate': overall_success_rate,
                'duration_seconds': duration,
                'timestamp': end_time.isoformat()
            },
            'test_suites': self.test_suites
        }
        
        with open('test_results.json', 'w') as f:
            json.dump(json_data, f, indent=2)
        
        return filename


def main():
    """Run all comprehensive tests"""
    print("="*80)
    print("COMPREHENSIVE SAILBOT2526 REGRESSION TEST SUITE")
    print("="*80)
    print()
    
    report = ComprehensiveTestReport()
    
    # Test Suite 1: Comprehensive Regression Tests
    print("\n" + "="*80)
    print("TEST SUITE 1: Comprehensive Regression Tests")
    print("="*80)
    try:
        from test_comprehensive_regression import (
            TestAdaptivePIDHarsh,
            TestExtendedKalmanFilterHarsh,
            TestOptimalSailControllerHarsh,
            TestWindFilterHarsh,
            TestDriftEstimatorHarsh
        )
        
        suite = unittest.TestSuite()
        suite.addTests(unittest.TestLoader().loadTestsFromTestCase(TestAdaptivePIDHarsh))
        suite.addTests(unittest.TestLoader().loadTestsFromTestCase(TestExtendedKalmanFilterHarsh))
        suite.addTests(unittest.TestLoader().loadTestsFromTestCase(TestOptimalSailControllerHarsh))
        suite.addTests(unittest.TestLoader().loadTestsFromTestCase(TestWindFilterHarsh))
        suite.addTests(unittest.TestLoader().loadTestsFromTestCase(TestDriftEstimatorHarsh))
        
        runner = unittest.TextTestRunner(verbosity=2)
        result = runner.run(suite)
        report.add_suite_result("Comprehensive Regression Tests", result)
    except Exception as e:
        print(f"ERROR loading regression tests: {e}")
        traceback.print_exc()
    
    # Test Suite 2: Position Tracking Tests
    print("\n" + "="*80)
    print("TEST SUITE 2: Position Tracking Tests")
    print("="*80)
    try:
        from test_position_tracking import TestPositionTrackingHarsh, TestWaypointNavigationHarsh
        
        suite = unittest.TestSuite()
        suite.addTests(unittest.TestLoader().loadTestsFromTestCase(TestPositionTrackingHarsh))
        suite.addTests(unittest.TestLoader().loadTestsFromTestCase(TestWaypointNavigationHarsh))
        
        runner = unittest.TextTestRunner(verbosity=2)
        result = runner.run(suite)
        report.add_suite_result("Position Tracking Tests", result)
    except Exception as e:
        print(f"ERROR loading position tracking tests: {e}")
        traceback.print_exc()
    
    # Test Suite 3: Sail Angle Comprehensive Tests
    print("\n" + "="*80)
    print("TEST SUITE 3: Sail Angle Comprehensive Tests")
    print("="*80)
    try:
        from test_sail_angle_comprehensive import TestSailAngleComprehensive
        
        suite = unittest.TestLoader().loadTestsFromTestCase(TestSailAngleComprehensive)
        runner = unittest.TextTestRunner(verbosity=2)
        result = runner.run(suite)
        report.add_suite_result("Sail Angle Comprehensive Tests", result)
    except Exception as e:
        print(f"ERROR loading sail angle tests: {e}")
        traceback.print_exc()
    
    # Generate report
    print("\n" + "="*80)
    print("GENERATING COMPREHENSIVE REPORT")
    print("="*80)
    
    report_file = report.generate_report()
    
    print(f"\n✅ Report generated: {report_file}")
    print(f"✅ JSON data exported: test_results.json")
    print(f"\n📊 FINAL RESULTS:")
    print(f"   Total Tests: {report.total_tests}")
    print(f"   Passed: {report.passed_tests}")
    print(f"   Failed: {report.failed_tests}")
    print(f"   Success Rate: {(report.passed_tests/report.total_tests*100) if report.total_tests > 0 else 0:.1f}%")
    print("\n" + "="*80)


if __name__ == '__main__':
    main()

