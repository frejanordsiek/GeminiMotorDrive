===============
Getting Started
===============

Let's suppose that one has a Gemini GV-6 linear servo motor drive
supporting the ASCII protocol connected to the RS232 port
``'/dev/ttyS1'`` (would look more like ``'COM1'`` on Windows).

Accessing The Drive Directly
============================

The drive is accessed directly by using drivers in the
:py:mod:`GeminiMotorDrive.drivers` module. Right now, there is only one
driver there, which is :py:class:`GeminiMotorDrive.drivers.ASCII_RS232`
for the ASCII protocol over an RS232 connection (sometimes known as a
serial connection).

Connecting to The Drive
-----------------------

The first step is to connect to the drive using
:py:class:`GeminiMotorDrive.drivers.ASCII_RS232`. There are two ways to
access it. One is to access it directly as 

    >>> import GeminiMotorDrive
    >>> from GeminiMotorDrive.drivers import ASCII_RS232
    >>> dr = ASCII_RS232('/dev/ttyS1')

and another is to use the :py:func:`GeminiMotorDrive.get_driver`
function in the main module.

    >>> import GeminiMotorDrive
    >>> dr = GeminiMotorDrive.get_driver(driver='ASCII_RS232')

Sending Commands to The Drive
-----------------------------

Using :py:meth:`GeminiMotorDrive.drivers.ASCII_RS232.send_command` and
:py:meth:`GeminiMotorDrive.drivers.ASCII_RS232.send_commands`, one can
send commands to the drive using the drive's ASCII language. For
example, the drive would be energized (motor turned on) by doing

    >>> dr.send_command('DRIVE1', immediate=False, timeout=1.0)
    ['DRIVE1', 'DRIVE1\r\r\n', 'DRIVE1', None, []]

The command was sent to the drive's command buffer to be executed
when the drive is finished moving (``immediate=False``) with a
timeout of one second to get a response from the drive. The returned
``list`` gives the command sent to the drive (after sanitizing it a
bit), the full response from the drive, the echoed command (``dr`` was
initialized so that the drive would echo back commands to make it easier
to send commands correctly), whether there was an error (``None`` means
there was not), and the drive's repsonse separated into its individual
lines. From the return values, there was no error meaning that the drive
was successfully energized.

If the same command had been sent but with a typo,

    >>> dr.send_command('DRIV1', immediate=False, timeout=1.0)
    ['DRIV1', 'DRIV1\r*UNDEFINED_LABEL\r\r\n', 'DRIV1',
     'UNDEFINED_LABEL', []]

The command produced an error since it was recognized. Specifically, the
drive reported and ``'UNDEFINED_LABEL'`` error.

An easier way to check whether there was an error or not is to use the
:py:meth:`GeminiMotorDrive.drivers.ASCII_RS232.command_error` function,
which returns ``'True'`` if a command response indicates an error. Doing
this on the previous incorrect command,

    >>> resp = dr.send_command('DRIV1', immediate=False, timeout=1.0)
    >>> dr.command_error(resp)
    True

A ``list`` of multiple commands can be sent using
:py:meth:`GeminiMotorDrive.drivers.ASCII_RS232.send_commands`. Let's
energize the drive, move the motor 10000 motor units, make it oscillate
10000 motor units 4 times, and then de-energize the drive.

    >>> dr.send_commands(['DRIVE1', 'D-10000', 'GO']
    ...                  + ['D-10000','GO','D10000','GO']*4
    ...                  + [ 'DRIVE0'])
    [['DRIVE1', 'DRIVE1\r', 'DRIVE1', None, []],
     ['D-10000', 'D-10000\r', 'D-10000', None, []],
     ['GO', 'GO\r', 'GO', None, []],
     ['D-10000', 'D-10000\r', 'D-10000', None, []],
     ['GO', 'GO\r', 'GO', None, []],
     ['D10000', 'D10000\r', 'D10000', None, []],
     ['GO', 'GO\r', 'GO', None, []],
     ['D-10000', 'D-10000\r', 'D-10000', None, []],
     ['GO', 'GO\r', 'GO', None, []],
     ['D10000', 'D10000\r', 'D10000', None, []],
     ['GO', 'GO\r', 'GO', None, []],
     ['D-10000', 'D-10000\r', 'D-10000', None, []],
     ['GO', 'GO\r', 'GO', None, []],
     ['D10000', 'D10000\r', 'D10000', None, []],
     ['GO', 'GO\r', 'GO', None, []],
     ['D-10000', 'D-10000\r', 'D-10000', None, []],
     ['GO', 'GO\r', 'GO', None, []],
     ['D10000', 'D10000\r', 'D10000', None, []],
     ['GO', 'GO\r', 'GO', None, []],
     ['DRIVE0', 'DRIVE0\r', 'DRIVE0', None, []]]

The returned ``list`` is just a ``list`` of the outputs for the
individual commands.

Using The Convenience Class GeminiG6
====================================

A major downside to using
:py:class:`GeminiMotorDrive.drivers.ASCII_RS232` like this is that one
must give the drive the exact commands for each action in the drive's
language and then parse the output. This can be inconvenient at times
and is also error prone.

The :py:class:`GeminiMotorDrive.GeminiG6` class wraps around the driver
to provide a more convenient interface for many commands.

Wrapping The Driver
-------------------

An instance is made by passing a suitable driver (``dr`` in this case)

    >>> gem = GeminiMotorDrive.GeminiG6(dr)

The driver can still be accessed directly with ``dr`` or as
``gem.driver`` (:py:attr:`GeminiMotorDrive.GeminiG6.driver`).

Using Properties to Control And Query
-------------------------------------

Now, energizing the drive is as simple as setting the
:py:attr:`GeminiMotorDrive.GeminiG6.energized` property.

    >>> gem.energized = True

And reading the property will return whether the drive is energized or
not.

    >>> gem.energized
    True

Using Programs And Profiles
---------------------------

Many times, it is undesirable to control the motor's motion one command
at a time. For repeated patterns of motion, doing it each time is error
prone and can suffer from transmission errors. In addition, the
transmission time for each command can slow down operation. A useful
feature of the Gemini G6 series is that they can store sequences of
commands as programs and sequences of movement as motion profiles in
their internal memory. The commands can then be run locally one after
another without having to wait for a new command to be sent.

Programs and profiles are a bit different though. Programs allow nearly
all commands and operate as if the commands had been sent, interpretting
each command in the program one after another. Profiles allow a much
more limited set commands that essentially only describe movement
patterns. But profiles are not interpreted each time they are
run. Instead, the drive compiles them to an internal motion description
when they are received. Profiles run faster since there is no
interpretting at each step. Moreover, the drive can receive and run
commands while a profile is running.

Programs and profiles are numbered starting from one.

.. warning::

   Gemini G6 drives have a limited amount of memory. This means that
   they can not necessarily store the full numerical number of programs
   and profiles indicated in the manual. Larger programs and profiles
   consume more memory.

Let's take the earlier example motion of moving the motor 10000 motor
units and then making it oscillate 10000 motor units 4 times. But this
time, we will make the motor pause for one second before it starts to
oscillate. For the program version, we will make it energize the drive
at the beginning and de-energize the drive at the end (this cannot be
done in a profile). The program version of this looks like

    >>> pgm = ['DRIVE1', 'D-10000', 'GO', 'WAIT(AS.1=b0)', 'T1']
    ...       + ['D-10000','GO','D10000','GO']*4
    ...       + [ 'DRIVE0']

and the profile version looks like

    >>> pfl = ['D-10000', 'GOBUF', 'GOWHEN(T=1000)']
    ...       + ['D-10000','GOBUF','D10000','GOBUF']*4

Programs and profiles are set using
:py:meth:`GeminiMotorDrive.GeminiG6.set_program_profile`. To set program
1,

    >>> gem.set_program_profile(1, pgm, program_or_profile='program')
    True

We can then read the program back with
:py:meth:`GeminiMotorDrive.GeminiG6.get_program`

    >>> gem.get_program(1)
    ['Drive1',
     'D-10000',
     'GO1',
     'WAIT(AS.1=b0)',
     'T1',
     'D-10000',
     'GO1',
     'D10000',
     'GO1',
     'D-10000',
     'GO1',
     'D10000',
     'GO1',
     'D-10000',
     'GO1',
     'D10000',
     'GO1',
     'D-10000',
     'GO1',
     'D10000',
     'GO1',
     'DRIVE0']

This looks a bit different than the program that was sent. A lot of
commands have default values for arguments or can interpret many strings
to be the same. When a program is read from the drive, all arguments are
given, even if they have the default value. And they can be rendered
into strings differently than a human would.

Setting profile 1 is similar

    >>> gem.set_program_profile(1, pfl, program_or_profile='profile')
    True

except that we cannot read a profile from the drive. This is something
that Gemini drives does not support.

Programs and profiles are run the same way using
:py:meth:`GeminiMotorDrive.GeminiG6.run_program_profile`. Running
program 1 is done by

    >>> gem.run_program_profile(1, program_or_profile='program',
    ...                         timeout=20.0)

and running profile 1 is done by

    >>> gem.run_program_profile(1, program_or_profile='profile',
    ...                         timeout=2.0)

Which are nearly identical. It is just a matter of specifying whether to
run a ``'program'`` or a ``'profile'``. The `timeout` argument does
deserve special attention. It is used to specify how many seconds to
wait to collect the output from the drive. When running a program, each
command that is run is returned as it is run, meaning that the
`timeout` needs to be longer than the program takes to run if one
wants to collect all of its output. When running a profile, just the
command to run a profile is returned. Thus, for a profile, the
`timeout` can be relatively short.

:py:meth:`GeminiMotorDrive.GeminiG6.run_program_profile` returns the
output that it captures within the window of the `timeout`.

Generating Programs and Profiles
================================

Programs and profiles give a large level of convenience, but they still
have to be written without introducing errors. For simple motion
composed of sequences of parabolic motion (parobolic position vs. time
graphs) coming to a stop at the end of each step with optional pauses in
beteen, a sort of higher level language
description of the motion is provided. It can be compiled to the program
and profile language of the drive using the module
:py:mod:`GeminiMotorDrive.compilers.move_sequence`.

Defining Movement Sequences (Syntax)
------------------------------------

A move sequence is formatted as an iterable of ``dict``. Each element of
the iterable defines a block of movement steps. The blocks are done one
after another.

Each block defines the movements to perform, the time to wait after each
movement, and the number of to do the motion of the block. These are
defined using the ``'moves'``, ``'wait_times'``, and ``'iterations'``
keys respectively.

  * ``'moves'`` is an iterable of the movements to do, which are
    themselves ``dict``.
  * ``'wait_times'`` is an iterable of the number of seconds to wait
    after each movement. Values of 0 mean no wait.
  * ``'iterations'`` is a positive integer specifying how many times to
    do the block. Values > 1 imply doing a loop.

A block consisting of movements ``M1`` and ``M2``, with waits of ``1.0``
and ``0.3`` seconds after the first and second movements, and is done 20 times would look like

``{'iterations': 20, 'wait_times': [1.0, 0.3], 'moves': [M1, M2]}``

Movements need to specify a distance to move, a maximum velocity that
will the motor will attempt to reach, the acceleration magnitude to use
when speeding up, and the accleration magnitude (deceleration) to use
when slowing down to a stop. To match the program language of the drive,
these are given as the keys ``'D'``, ``'V'``, ``'A'``, and ``'AD'``
respecively. They each take single numerical values, which can be
floating point.

  * ``'D'`` The distance to move including sign.
  * ``'V'`` Maximum velocity for the motor to attempt to reach.
  * ``'A'`` Acceleration magnitude to use when speeding up.
  * ``'AD'`` Acceleration magnitude to use when slowing down to a stop.

Notice that the units have not been defined yet. At this stage, they do
not have to be. One could use the motor's unit of distance, meters,
centimeters, etc. And one could be using the motor's units of velocity,
m/s, cm/s, etc.

A movement going a distance of ``120`` backwards (negative) with a
maximum velocity of ``3.3``, an acceleration of ``0.1``, and a
deceleration of ``30`` would be

``{'D': -120, 'V': 3.3, 'A': 0.1, 'AD': 30}``

Now, let's define a movement sequence consisting of one block where
there are two motions inside it with a pause of 1 second in between
movements.

    >>> cycles = [{'iterations':1, 'wait_times':[1, 0],
    ...           'moves':[{'A':100, 'AD':0, 'D':-1000, 'V':100},
    ...           {'A':90, 'AD':0, 'D':-1000, 'V':100}]}]

And another longer movement sequence consisting of a single movement
with a 1 second paus, then oscillating 100 times, and then moving back
to the starting position.

    >>> long_cycles = [{'iterations':1, 'wait_times':[1],
    ...                'moves':[{'A':100, 'AD':0, 'D':-1000, 'V':100}]},
    ...                {'iterations':100, 'wait_times':[0, 0],
    ...                'moves':[{'A':50, 'AD':40, 'D':-1000, 'V':30},
    ...                {'A':50, 'AD':40, 'D':1000, 'V':30}]},
    ...                {'iterations':1, 'wait_times':[0],
    ...                'moves':[{'A':100, 'AD':0, 'D':1000, 'V':100}]}]

Units
-----

The units in the motion sequence are thus far undefined. The Gemini
drive has its own particular set of units it uses, much like many
stepper and servo motor drives.

If the sequence was written using the motor's units, then the sequence
can be compiled and the motion will be performed truthfully. But if say
we are using units of meters, m/s, and m/s**2; this will not be the
case and we will have to do a unit conversion first.

The one thing that is fixed in the units is the unit of time, which is
fixed to seconds.

Now, the motor's units are defined by two variables, called DMEPIT and
ERES. They are the electrical pitch and encoder resolution of the motor
respectively. For Gemini drives, the conversions between motor units and
MKS units (distance in meters, velocity in m/s, and acceleration in
m/s**2) is

  * ``D_motor = 1e3 * D * ERES / DMEPIT``
  * ``V_motor = 1e3 * V / DMEPIT``
  * ``A_motor = 1e3 * A / DMEPIT``

The module :py:mod:`GeminiMotorDrive.utilities` provides a class for
converting units called
:py:class:`GeminiMotorDrive.utilities.UnitConverter`. If ``dmepit`` and
``eres`` are the DMEPIT and ERES values for the motor, a unit converter
for MKS can be created by

    >>> from GeminiMotorDrive.utilities import UnitConverter
    >>> uc = UnitConverter(dmepit=dmepit, eres=eres,
    ...                    unit_in_meters=1.0)

The `unit_in_meters` parameter is used to specify the unit of distance
(remember, the unit of time is fixed as seconds) in meters. So, if one
wanted to do centimeters, one would use ``unit_in_meters=0.01``. If one
wanted to do inches, one would use ``unit_in_meters=0.0254``.

Now, ``uc`` (:py:class:`GeminiMotorDrive.utilities.UnitConverter`) is
not really meant to be used by the user directly. It is instead an
object passed as an argument when compiling the move sequence or
calculating how long it will take to complete. However, the move
squence's units can be converted directly using
:py:func:`GeminiMotorDrive.compilers.move_sequence.convert_sequence_to_motor_units`
in the module :py:mod:`GeminiMotorDrive.compilers.move_sequence`.
    
    >>> from GeminiMotorDrive.compilers.move_sequence import convert_sequence_to_motor_units
    >>> convert_sequence_to_motor_units(cycles, unit_converter=uc)

Compiling the Program or Profile
--------------------------------

Move sequences are compiled into programs and profiles using the module
:py:mod:`GeminiMotorDrive.compilers.move_sequence`. Specifically, the
function
:py:func:`GeminiMotorDrive.compilers.move_sequence.compile_sequence`
is used. Import it

    >>> from GeminiMotorDrive.compilers.move_sequence import compile_sequence

If we want to compile the move sequence ``ms`` to a program using
``uc``, one would do

    >>> compile_sequence(ms, program_or_profile='program',
    ...                  unit_converter=uc)

or to compile it to a profile

    >>> compile_sequence(ms, program_or_profile='profile',
    ...                  unit_converter=uc)

By default, the function assumes that the move sequence is given in
motor units.

Now, lets compile ``cycles`` into a program and then a profile assuming
that it is in motor units. First, a program

    >>> compile_sequence(cycles, program_or_profile='program')
    ['A100',
     'AD0',
     'V100',
     'D-1000',
     'GO1',
     'WAIT(AS.1=b0)',
     'T1',
     'A90',
     'GO1',
     'WAIT(AS.1=b0)']

and now a profile

    >>> compile_sequence(cycles, program_or_profile='profile')
    ['A100',
     'AD100',
     'V100',
     'D-1000',
     'VF0',
     'GOBUF1',
     'GOWHEN(T=1000)',
     'A90',
     'AD90',
     'VF0',
     'GOBUF1']

Compiling the longer program, which had oscillation, into a program
results in

    >>> compile_sequence(long_cycles, program_or_profile='program')
    ['A100',
     'AD0',
     'V100',
     'D-1000',
     'GO1',
     'WAIT(AS.1=b0)',
     'T1',
     'L100',
     'A50',
     'AD40',
     'V30',
     'D-1000',
     'GO1',
     'WAIT(AS.1=b0)',
     'D~',
     'GO1',
     'WAIT(AS.1=b0)',
     'LN',
     'A100',
     'AD0',
     'V100',
     'GO1',
     'WAIT(AS.1=b0)']

The programs and profiles can be sent to the drive and run.
     
Movement Completion Time
------------------------

Sometimes it is useful to know how long a movement sequence is going to
take. This is done using the function
:py:func:`GeminiMotorDrive.compilers.move_sequence.get_sequence_time`
in the module :py:mod:`GeminiMotorDrive.compilers.move_sequence`. It can
either be given a unit convert ``uc`` directory or the encoder
resolution (ERES stored in ``eres``). It then returns the time the move
sequence takes in seconds to perform.

    >>> from GeminiMotorDrive.compilers.move_sequence import get_sequence_time
    >>> get_sequence_time(cycles, unit_converter=uc)

or using ERES

    >>> from GeminiMotorDrive.compilers.move_sequence import get_sequence_time
    >>> get_sequence_time(cycles, eres=eres)

Due to drive latency and motor latencies, it usually takes a bit longer
for the motion sequence to complete when it is run by the drive. This is
especially true for programs where each command in the program has to be
interpreted. 
