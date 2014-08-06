# Copyright (c) 2014, Freja Nordsiek
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:
#
# 1. Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import math
import copy


def compile_sequence(cycles, program_or_profile='program',
                     unit_converter=None):
    """ Makes the command list for a move sequence.

    Constructs the list of commands to execute the given sequence of
    motion. Program/command line commands or profile commands can be
    generated depending on the value of `program_or_profile` so that the
    commands can be used to construct a program or profile later. Types
    of motion supported (see Notes for how to specify) are moves from
    one position to another (the motion will always come to a stop
    before doing the next motion), waiting a given interval of time till
    starting the next move, and looping over a sequence of moves.

    Parameters
    ----------
    cycles : iterable of dicts
        The iterable of cycles of motion to do one after another. See
        Notes for format.
    program_or_profile : {'program', 'profile'}, optional
        Whether program or profile motion commands should be used.
        Anything other than these two values implies the default.
    unit_converter : UnitConverter, optional
        ``GeminiMotorDrive.utilities.UnitConverter`` to use to convert
        the units in `cycles` to motor units. ``None`` indicates that
        they are already in motor units.

    Returns
    -------
    commands : list of str
        ``list`` of ``str`` commands making up the move sequence.

    Notes
    -----
    `cycles` is an iterable of individual cycles of motion. Each cycle
    is a ``dict`` that represents a sequence of moves that could
    possibly be looped over. The field ``'iterations'`` gives how many
    times the sequence of moves should be done (a value > 1 implies a
    loop). Then the field ``'moves'`` is an iterable of the individual
    moves. Each individual move is a ``dict`` with the acceleration
    (``'A'``), deceleration (``'AD'`` with 0 meaning the value of the
    acceleration is used), velocity (``'V'``), and the distance/position
    (``'D'``). Back in the cycle, the field ``'wait_times'`` is an
    iterable of numbers giving the time in seconds to wait after each
    move before going onto the next.

    See Also
    --------
    get_sequence_time
    convert_sequence_to_motor_units
    GeminiMotorDrive.utilities.UnitConverter

    Examples
    --------

    Simple program style two motions with a pause in between.

    >>> cycles = [{'iterations':1, 'wait_times':[1, 0],
                  'moves':[{'A':100, 'AD':0, 'D':-1000, 'V':100},
                  {'A':90, 'AD':0, 'D':-1000, 'V':100}]}]
    >>> compile_sequence(cycles)
    ['A100.0',
     'AD0.0',
     'V100.0',
     'D-1000.0',
     'GO1',
     'WAIT(AS.1=b0)',
     'T1.0',
     'A90.0',
     'GO1',
     'WAIT(AS.1=b0)']

    The same motion but in profile style commands

    >>> cycles = [{'iterations':1, 'wait_times':[1, 0],
                  'moves':[{'A':100, 'AD':0, 'D':-1000, 'V':100},
                  {'A':90, 'AD':0, 'D':-1000, 'V':100}]}]
    >>> compile_sequence(cycles, program_or_profile='profile')
    ['A100.0',
     'AD0.0',
     'V100.0',
     'D-1000.0',
     'VF0',
     'GOBUF1',
     'GOWHEN(T=1000)',
     'A90.0',
     'VF0',
     'GOBUF1']

    Another motion with a back and forth loop (100 iterations) in the
    middle, done in program style commands.

    >>> cycles = [{'iterations':1, 'wait_times':[1],
                  'moves':[{'A':100, 'AD':0, 'D':-1000, 'V':100}]},
                  {'iterations':100, 'wait_times':[0, 0],
                  'moves':[{'A':50, 'AD':40, 'D':-1000, 'V':30},
                  {'A':50, 'AD':40, 'D':1000, 'V':30}]},
                  {'iterations':1, 'wait_times':[0],
                  'moves':[{'A':100, 'AD':0, 'D':1000, 'V':100}]}]
    >>> compile_sequence(cycles)
    ['A100.0',
     'AD0.0',
     'V100.0',
     'D-1000.0',
     'GO1',
     'WAIT(AS.1=b0)',
     'T1.0',
     'L100',
     'A50.0',
     'AD40.0',
     'V30.0',
     'D-1000.0',
     'GO1',
     'WAIT(AS.1=b0)',
     'D~',
     'GO1',
     'WAIT(AS.1=b0)',
     'LN',
     'A100.0',
     'AD0.0',
     'V100.0',
     'GO1',
     'WAIT(AS.1=b0)']

    """
    # If needed, cycles needs to be converted to motor units.
    if unit_converter is None:
        cv_cycles = cycles
    else:
        cv_cycles = convert_sequence_to_motor_units(cycles, \
            unit_converter=unit_converter)

    # Initially, we have no commands in our command list.
    commands = []

    # The A, AD, D, and V parameters of the previous motion should be
    # kept track of because if they don't change from one motion to the
    # next, the commands to set them don't need to be included. They
    # will be started blank since there are no previous motions yet.
    previous_motion = {'A': [], 'AD': [], 'D': [], 'V': []}

    # Construct each cycle one by one.
    for cycle in cv_cycles:
        # If more than one iteration is being done, a loop needs to be
        # setup. It will be either 'L' or 'PLOOP' with the number of
        # iterations attached if it is a program or a profile
        # respectively. Since it will be tough to keep track of what
        # motion changed from the end of a loop to the beginning of it,
        # it is easier to just forget all previous motion values and set
        # them all at the beginning of the loop (clear previous_motion).
        iterations = int(cycle['iterations'])
        if iterations > 1:
            previous_motion = {'A': [], 'AD': [], 'D': [], 'V': []}
            if program_or_profile != 'profile':
                commands.append('L' + str(iterations))
            else:
                commands.append('PLOOP' + str(iterations))

        # Construct each individual move in the cycle.
        for i in range(0, len(cycle['moves'])):
            # Grab the motion indicated by the current move.
            new_motion = cycle['moves'][i]

            # If we are doing a profile, AD must be set explicitly
            # to A if it is 0.
            if program_or_profile == 'profile' \
                    and new_motion['AD'] == 0.0:
                new_motion['AD'] = new_motion['A']

            # Set A, AD, and V if they have changed.
            for k in ('A', 'AD', 'V'):
                if previous_motion[k] != new_motion[k]:
                    # Grab it and round it to 4 places after the decimal
                    # point because that is the most that is
                    # supported. Then, if it is an integer value,
                    # convert it to an integer because that is what the
                    # drive will send back if requested (makes
                    # comparisons easier). Then add the command.
                    val = round(float(new_motion[k]), 4)
                    if val == int(val):
                        val = int(val)
                    commands.append(k + str(val))

            # If the sign of D has flipped, we just need to issue a 'D~'
            # command. If the value has changed in another way, it needs
            # to be reset.
            if previous_motion['D'] != new_motion['D']:
                if previous_motion['D'] == -new_motion['D']:
                    commands.append('D~')
                else:
                    commands.append('D'
                                    + str(int(new_motion['D'])))

            # Grab the amount of time that should be waited after the
            # move is done.
            wait_time = cycle['wait_times'][i]

            # Give the motion command (GO or GOBUF), tell the drive to
            # wait till the motor has stopped (a WAIT command if it is a
            # program and a VF0 command if it is a profile), and make it
            # wait the period of time wait_time (T and GOWHEN commands).
            if program_or_profile != 'profile':
                commands.append('GO1')
                commands.append('WAIT(AS.1=b0)')
                if wait_time != 0:
                    # The wait time needs to be rounded to 3 places
                    # after the decimal. If it is an integer, it should
                    # be converted to an int so that the drive will send
                    # back what we send (makes compairisons easier).
                    wait_time = round(float(wait_time), 3)
                    if wait_time == int(wait_time):
                        wait_time = int(wait_time)
                    commands.append('T' + str(wait_time))
            else:
                commands.append('VF0')
                commands.append('GOBUF1')
                if wait_time != 0:
                    commands.append('GOWHEN(T='
                                    + str(int(1000*wait_time))
                                    + ')')

            # Before going onto the next move, previous_motion needs to
            # be set to the one just done.
            previous_motion = new_motion

        # Done with all the moves of the cycle. If we are looping, the
        # loop end needs to be put in.
        if iterations > 1:
            if program_or_profile != 'profile':
                commands.append('LN')
            else:
                commands.append('PLN')

    # Done constructing the command list.
    return commands


def get_sequence_time(cycles, unit_converter=None, eres=None):
    """ Calculates the time the move sequence will take to complete.

    Calculates the amount of time it will take to complete the given
    move sequence. Types of motion supported are moves from one position
    to another (the motion will always come to a stop before doing the
    next motion), waiting a given interval of time till starting the
    next move, and looping over a sequence of moves.

    Parameters
    ----------
    cycles : list of dicts
        The ``list`` of cycles of motion to do one after another. See
        ``compile_sequence`` for format.
    unit_converter : UnitConverter, optional
        ``GeminiMotorDrive.utilities.UnitConverter`` to use to convert
        the units in `cycles` to motor units. ``None`` indicates that
        they are already in motor units.
    eres : int
        Encoder resolution. Only relevant if `unit_converter` is
        ``None``.


    Returns
    -------
    time : float
        Time the move sequence will take.

    See Also
    --------
    compile_sequence
    GeminiMotorDrive.utilities.UnitConverter
    move_time

    """
    # If we are doing unit conversion, then that is equivalent to motor
    # units but with eres equal to one.
    if unit_converter is not None:
        eres = 1
    # Starting with 0 time, steadily add the time of each movement.
    tme = 0.0
    # Go through each cycle and collect times.
    for cycle in cycles:
        # Add all the wait times.
        tme += cycle['iterations']*sum(cycle['wait_times'])
        # Add the time for each individual move.
        for move in cycle['moves']:
            tme += cycle['iterations'] \
                * move_time(move, eres=eres)
    # Done.
    return tme


def move_time(move, eres):
    """ Calculates the time it takes to do a move.

    Calculates how long it will take to complete a move of the motor. It
    is assumed that the motor will decerate to a stop for the end of the
    move as opposed to keep moving at velocity.

    Everything is in motor units which are encoder counts for distance,
    pitches/s for velocity, and pitches/s^2 for acceleration.

    Parameters
    ----------
    move : dict
        Contains the move parameters in its fields: acceleration ('A'),
        deceleration ('AD' with 0 meaning the value of the acceleration
        is used), velocity ('V'), and the distance/position ('D').
    eres : int
        Encoder resolution.

    Returns
    -------
    time : float
        Time the move will take.

    See Also
    --------
    compile_sequence
    get_sequence_time

    """
    # Grab the move parameters. If the deceleration is given as zero,
    # that means it has the same value as the acceleration. Distance is
    # converted to the same units as the others by dividing by the
    # encoder resolution. The absolute value of everything is taken for
    # simplicity.
    A = abs(move['A'])
    AD = abs(move['AD'])
    if AD == 0.0:
        AD = A
    V = abs(move['V'])
    D = abs(move['D'])/eres

    # Calculate the times it would take to accelerate from stop to V and
    # decelerate to stop at rates A and AD respectively.
    accel_times = [V/A, V/AD]

    # Calculate the distances that would be moved in those times.
    dists = [0.5*A*(accel_times[0]**2), 0.5*AD*(accel_times[1]**2)]

    # If the sum of those dists is greater than D, then the velocity V
    # is never reached. The way the time is calculated depends on which
    # case it is.
    if sum(dists) <= D:
        # The time is just the sum of the acceleration times plus the
        # remaining distance divided by V.
        return (sum(accel_times) + (D-sum(dists))/V)
    else:
        # We need to find the time it takes for the acceleration path
        # and deceleration path to meet and have the same speeds.
        #
        # (1) t = t_1 + t_2
        # (2) A*t_1 = AD*t_2
        # (3) D = 0.5*A*(t_1**2) + 0.5*AD*(t_2**2)
        #
        # Re-writing t_2 in terms of t_1 using (2)
        # (4) t_2 = (A / AD) * t_1
        #
        # Putting that into (1) and (3)
        # (4) t = (1 + (A / AD)) * t_1
        # (5) D = 0.5*A*(1 + (A / AD)) * (t_1**2)
        #
        # Solving (5) for t_1,
        # (6) t_1 = sqrt( 2*D / (A * (1 + (A / AD))))
        #
        # Putting that into (4),
        # t = sqrt(2*D*(1 + (A / AD)) / A)
        return math.sqrt(2*D * (1 + (A / AD)) / A)

def convert_sequence_to_motor_units(cycles, unit_converter):
    """ Converts a move sequence to motor units.

    Converts a move sequence to motor units using the provied converter.

    Parameters
    ----------
    cycles : iterable of dicts
        The iterable of cycles of motion to do one after another. See
        ``compile_sequence`` for format.
    unit_converter : UnitConverter, optional
        ``GeminiMotorDrive.utilities.UnitConverter`` to use to convert
        the units in `cycles` to motor units.

    Returns
    -------
    motor_cycles : list of dicts
        A deep copy of `cycles` with all units converted to motor units.

    See Also
    --------
    compile_sequence
    GeminiMotorDrive.utilities.UnitConverter

    """
    # Make a deep copy of cycles so that the conversions don't damage
    # the original one.
    cv_cycles = copy.deepcopy(cycles)

    # Go through each cycle and do the conversions.
    for cycle in cv_cycles:
        # Go through each of the moves and do the conversions.
        for move in cycle['moves']:
            move['A'] = unit_converter.to_motor_velocity_acceleration( \
                move['A'])
            move['AD'] = \
                unit_converter.to_motor_velocity_acceleration( \
                move['AD'])
            move['V'] = unit_converter.to_motor_velocity_acceleration( \
                move['V'])
            move['D'] = int(unit_converter.to_motor_distance(move['D']))

    # Now return the converted move sequence.
    return cv_cycles
