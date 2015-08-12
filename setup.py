import ez_setup
ez_setup.use_setuptools()

from setuptools import setup

with open('README.rst') as file:
    long_description = file.read()

setup(name='GeminiMotorDrive',
      version='0.2',
      description='Utilities to control Parker Hannifin Gemini stepper/servo motor drives.',
      long_description=long_description,
      author='Freja Nordsiek',
      author_email='fnordsie at gmail dt com',
      url='https://github.com/frejanordsiek/GeminiMotorDrive',
      py_modules=['GeminiMotorDrive'],
      requires=['serial'],
      license='Apache',
      keywords='Parker Hannifin Gemini stepper servo motor',
      classifiers=[
          "Programming Language :: Python :: 3",
          "Development Status :: 3 - Alpha",
          "License :: OSI Approved :: Apache Software License",
          "Operating System :: OS Independent",
          "Intended Audience :: Manufacturing",
          "Intended Audience :: Science/Research",
          "Topic :: System :: Hardware :: Hardware Drivers",
          ],
      )
