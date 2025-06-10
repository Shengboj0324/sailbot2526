from setuptools import setup, find_packages
from numpy.distutils.core import setup as numpy_setup, Extension
import os

package_name = 'path_planning'

# Define the Fortran extension
fortran_extension = Extension(
    name=f'{package_name}.{package_name}.leg_fortran_module',
    sources=[os.path.join(package_name, 'leg_fortran.f90')],
    extra_f90_compile_args=['-O3', '-fPIC'],
)

# Note: We use numpy.distutils for Fortran support
numpy_setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, f'{package_name}.{package_name}'], 
    package_dir={
        '': '.',
        f'{package_name}.{package_name}': package_name
    },
    ext_modules=[fortran_extension],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'data'),
            [os.path.join(package_name, 'data', 'test.pol')]),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=False,
    maintainer='kehillah',
    maintainer_email='kehillah@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
    package_data={
        f'{package_name}.{package_name}': ['data/test.pol']
    }
)