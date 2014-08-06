Overview
========

This Python package provides an interface to Parker Hannifin Gemini
stepper/servo motor drives.

.. note::
   
   Only the ASCII protocol is supported at this time, meaning that only
   newer GV6 and GT6 drives are supported.

The package's documetation is found at
http://pythonhosted.org/GeminiMotorDrive/

The package's source code is found at
https://github.com/frejanordsiek/GeminiMotorDrive

Installation
============

This package may not work on Python < 3.0.

This package requires the serial package.

To install GeminiMotorDrive, download the package and run the command::

    python3 setup.py install


Versions
========

0.1.1. Bugfix release fixing one bug.
       * Fixed bug in how ``GeminiG6.move_time`` calculated the time
         a movement would take in the case that the drive cannot reach
         its maximum velocity in the motion.

0.1. Initial version.
