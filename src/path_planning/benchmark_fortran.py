#!/usr/bin/env python3
"""
Comprehensive benchmark comparing different path planning implementations
"""

import time
import numpy as np
import sys
import os
from typing import List, Tuple
import multiprocessing

# Add path for imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

class BenchmarkRunner:
    def __init__(self):
        self.implementations = {}
        self.load_implementations()
        
    def load_implementations(self):
        """Load all available implementations"""
        # Original Python implementation
        try:
            from path_planning.leg_original import Leg as LegOriginal
            self.implementations['Python Original'] = LegOriginal
            print("✓ Loaded Python Original implementation")
        except:
            print("✗ Could not load Python Original")
            
        # Current f2py Fortran implementation
        try:
            from path_planning.leg import Leg as LegCurrent
            self.implementations['Fortran f2py'] = LegCurrent
            print("✓ Loaded Fortran f2py implementation")
        except:
            print("✗ Could not load Fortran f2py")
            
        # Modern Fortran with ctypes
        try:
            from path_planning.leg_modern_wrapper import ModernLeg
            self.implementations['Fortran Modern'] = ModernLeg
            print("✓ Loaded Modern Fortran implementation")
        except Exception as e:
            print(f"✗ Could not load Modern Fortran: {e}")
    
    def generate_test_cases(self, n_cases: int) -> List[Tuple]:
        """Generate random test cases"""
        np.random.seed(42)  # Reproducible results
        cases = []
        
        for _ in range(n_cases):
            # Random positions within reasonable bounds
            start_lat = np.random.uniform(40.0, 45.0)
            start_lon = np.random.uniform(-75.0, -70.0)
            end_lat = start_lat + np.random.uniform(-1.0, 1.0)
            end_lon = start_lon + np.random.uniform(-1.0, 1.0)
            
            # Random wind and heading
            wind_angle = np.random.uniform(0, 360)
            boat_heading = np.random.uniform(0, 360)
            
            # Random first maneuver
            first_starboard = np.random.choice([True, False])
            
            cases.append((
                (start_lat, start_lon),
                (end_lat, end_lon),
                wind_angle,
                boat_heading,
                first_starboard
            ))
        
        return cases
    
    def benchmark_single_implementation(self, name: str, impl_class, test_cases: List[Tuple]):
        """Benchmark a single implementation"""
        try:
            impl = impl_class()
            
            # Warmup
            for case in test_cases[:10]:
                impl.calculate_path(*case)
            
            # Actual benchmark
            start_time = time.perf_counter()
            results = []
            
            for case in test_cases:
                result = impl.calculate_path(*case)
                results.append(result)
            
            end_time = time.perf_counter()
            total_time = end_time - start_time
            avg_time = total_time / len(test_cases) * 1000  # ms per calculation
            
            return {
                'name': name,
                'total_time': total_time,
                'avg_time_ms': avg_time,
                'calculations_per_second': len(test_cases) / total_time,
                'success': True
            }
        except Exception as e:
            return {
                'name': name,
                'error': str(e),
                'success': False
            }
    
    def run_benchmarks(self, n_cases: int = 10000):
        """Run all benchmarks"""
        print(f"\n{'='*60}")
        print(f"Running benchmarks with {n_cases} test cases")
        print(f"{'='*60}\n")
        
        # Generate test cases
        test_cases = self.generate_test_cases(n_cases)
        
        # Run benchmarks
        results = []
        for name, impl_class in self.implementations.items():
            print(f"Benchmarking {name}...", end='', flush=True)
            result = self.benchmark_single_implementation(name, impl_class, test_cases)
            results.append(result)
            if result['success']:
                print(f" ✓ {result['avg_time_ms']:.3f} ms/calc")
            else:
                print(f" ✗ Error: {result['error']}")
        
        # Display results
        self.display_results(results)
        
    def display_results(self, results):
        """Display benchmark results in a nice format"""
        print(f"\n{'='*60}")
        print("BENCHMARK RESULTS")
        print(f"{'='*60}")
        
        # Filter successful results
        successful = [r for r in results if r['success']]
        if not successful:
            print("No successful benchmarks!")
            return
        
        # Sort by performance
        successful.sort(key=lambda x: x['avg_time_ms'])
        
        # Find baseline (slowest)
        baseline = successful[-1]
        
        print(f"\n{'Implementation':<20} {'Avg Time':<12} {'Calc/sec':<12} {'Speedup':<10}")
        print("-" * 60)
        
        for result in successful:
            speedup = baseline['avg_time_ms'] / result['avg_time_ms']
            print(f"{result['name']:<20} "
                  f"{result['avg_time_ms']:>8.3f} ms  "
                  f"{result['calculations_per_second']:>10.0f}  "
                  f"{speedup:>8.1f}x")
        
        print("\nDetailed timings:")
        print("-" * 60)
        for result in successful:
            print(f"{result['name']}: {result['total_time']:.3f} seconds total")

def run_parallel_benchmark():
    """Test parallel processing capabilities"""
    print("\n" + "="*60)
    print("PARALLEL PROCESSING BENCHMARK")
    print("="*60)
    
    try:
        # This would test the parallel Fortran implementation
        # if we had the proper Python wrapper for it
        print("Parallel benchmark not yet implemented")
    except:
        print("Parallel implementation not available")

def main():
    """Main benchmark runner"""
    print("Sailboat Path Planning Performance Benchmark")
    print("=" * 60)
    print(f"CPU cores available: {multiprocessing.cpu_count()}")
    print(f"Python version: {sys.version.split()[0]}")
    
    runner = BenchmarkRunner()
    
    # Run different benchmark sizes
    for n_cases in [100, 1000, 10000]:
        runner.run_benchmarks(n_cases)
    
    # Run parallel benchmark
    run_parallel_benchmark()

if __name__ == "__main__":
    main()