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

The package uses the Apache 2.0 license
http://www.apache.org/licenses/LICENSE-2.0

Installation
============

This package requires Python >= 3.0 at this time. It still needs
additional development to support Python 2.7 and 2.6.

This package requires the serial package.

To install GeminiMotorDrive, download the package and run the command::

    python3 setup.py install

Need New Maintainer
===================

As of 2016-02-12, I (Freja Nordsiek) no longer have access to a Gemini
stepper/servo motor. Thus, I can no longer continue development of this
project other than fixing obvious small bugs. If someone is interested
in taking over this package, please contact me.

Versions
========

0.2. Major release changing many things.
     * Changed license from 2-clause BSD to Apache 2.0.
     * Major API rewrite - breaks compatability with 0.1.x versions.
     * Main module split into several submodules.
     * ``GeminiG6`` and ``LinearMotorGV6`` classes combined together
       as ``GeminiG6`` and several of its methods split off into their
       own classes in the submodules.
     * A lot of documentation added.

0.1.1. Bugfix release fixing one bug.
       * Fixed bug in how ``GeminiG6.move_time`` calculated the time
         a movement would take in the case that the drive cannot reach
         its maximum velocity in the motion.

0.1. Initial version.
