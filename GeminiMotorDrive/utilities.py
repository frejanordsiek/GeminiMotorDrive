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


import collections


def strip_commands(commands):
    """ Strips a sequence of commands.

    Strips down the sequence of commands by removing comments and
    surrounding whitespace around each individual command and then
    removing blank commands.

    Parameters
    ----------
    commands : iterable of strings
        Iterable of commands to strip.

    Returns
    -------
    stripped_commands : list of str
        The stripped commands with blank ones removed.

    """
    # Go through each command one by one, stripping it and adding it to
    # a growing list if it is not blank. Each command needs to be
    # converted to an str if it is a bytes.
    stripped_commands = []
    for v in commands:
        if isinstance(v, bytes):
            v = v.decode(errors='replace')
        v = v.split(';')[0].strip()
        if len(v) != 0:
            stripped_commands.append(v)
    return stripped_commands


class UnitConverter(object):
    """ Converter between physical units and motor units.

    Class to convert between the desired physical unit of length and
    motor units; which are encoder counts for distance, motor pitches
    per second for velocity, and motor pitches per second squared for
    acceleration.

    Parameters
    ----------
    dmepit : float
        Electrical pitch of the motor.
    eres : int
        Encoder resolution.
    unit_in_meters : float, optional
        The length in meters of the desired physical unit of length for
        this convert to convert between. The default, ``1.0``,
        corresponds to converting between meters and motor units.

    """
    def __init__(self, dmepit, eres, unit_in_meters=1.0):
        # Construct the multipliers to convert distances and
        # velocities/accelerations to motor units.
        self._distance_to_motor = 1e3*unit_in_meters*eres/dmepit
        self._va_to_motor = 1e3*unit_in_meters/dmepit

    def to_motor_distance(self, distance):
        """ Convert distance/s to motor units.

        Converts distance/s to units of motor encoder counts, which is
        what the drive requires when given move instructions.

        Parameters
        ----------
        distance : int, float, or iterable of ints and floats
            The distance/s to convert.

        Returns
        -------
        converted_distance : float or list of floats
            The converted distance/s.

        """
        if isinstance(distance, collections.Iterable):
            return [(x * self._distance_to_motor) for x in distance]
        else:
            return distance * self._distance_to_motor

    def to_motor_velocity_acceleration(self, va):
        """ Convert velocities/accelerations to motor units.

        Converts velocity/ies and/or acceleration/s to units of motor
        pitch per second (or second squared), which is what the drive
        requires when given move instructions.

        Parameters
        ----------
        va : int, float, or iterable of ints and floats
            The velocities/accelerations to convert.

        Returns
        -------
        converted_va : float or list of floats
            The converted velocities/accelerations.

        """
        if isinstance(va, collections.Iterable):
            return [(x * self._va_to_motor) for x in va]
        else:
            return va * self._va_to_motor

    def to_unit_distance(self, distance):
        """ Convert distance/s to units of UnitConverter.

        Converts distance/s from motor encoder counts to that of this
        UnitConverter.

        Parameters
        ----------
        distance : int, float, or iterable of ints and floats
            The distance/s to convert.

        Returns
        -------
        converted_distance : float or list of floats
            The converted distance/s.

        """
        if isinstance(distance, collections.Iterable):
            return [(x / self._distance_to_motor) for x in distance]
        else:
            return distance / self._distance_to_motor

    def to_unit_velocity_acceleration(self, va):
        """ Convert velocities/accelerations to units of UnitConverter.

        Converts velocity/ies and/or acceleration/s from units of motor
        pitch per second (or second squared) to that of this
        UnitConverter.

        Parameters
        ----------
        va : int, float, or iterable of ints and floats
            The velocities/accelerations to convert.

        Returns
        -------
        converted_va : float or list of floats
            The converted velocities/accelerations.

        """
        if isinstance(va, collections.Iterable):
            return [(x / self._va_to_motor) for x in va]
        else:
            return va / self._va_to_motor
