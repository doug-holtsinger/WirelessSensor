from distutils.core import setup, Extension
from Cython.Build import cythonize

PROJ_DIR = '../../../Sensor/src'
PROJ_INC_DIR = PROJ_DIR
IMU_INC_DIR = PROJ_DIR + '/IMU'
AHRS_INC_DIR = PROJ_DIR + '/AHRS'
CONFIG_DIR = PROJ_DIR + '/../config'

#SRC = ['ahrs.pyx', 'AHRS.cpp', 'MahonyAHRS.cpp', 'MadgwickAHRS.cpp']
SRC = ['ahrs.pyx']
INC_DIR = [ PROJ_INC_DIR, IMU_INC_DIR, AHRS_INC_DIR , CONFIG_DIR]

extension1 = Extension("ahrs", SRC, include_dirs = INC_DIR)

setup(name = 'ahrs',
      version = '1.0',
      description = 'AHRS package',
      author = 'Douglas Holtsinger',
      ext_modules = cythonize(
         extension1,         # Cython and C++ source
         language="c++",     # generate C++ code
         ))

