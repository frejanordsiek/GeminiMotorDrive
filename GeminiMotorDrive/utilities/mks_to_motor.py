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

""" Module of functions to convert MKS measures to motor units.

Functions convert distances, velocities, and accelerations in MKS units
to motor units which the Gemini drives expect.

"""


import collections


def convert_distance(distance, dmepit, eres):
    """ Convert distances from MKS to motor units.

    Converts the distance/s given in MKS units to units of motor encoder
    counts, which is what the drive requires when given move
    instructions.

    Parameters
    ----------
    distance : int, float, or iterable of ints and floats
        The distance/s in MKS units to convert.
    dmepit : float
        Electrical pitch of the motor.
    eres : int
        Encoder resolution.

    Returns
    -------
    motor_distance : float or list of floats
        The distance/s converted to units of encoder counts.

    See Also
    --------
    convert_velocity_acceleration

    """
    if isinstance(distance, collections.Iterable):
        return [(x*1e3*eres/dmepit) for x in distance]
    else:
        return distance*1e3*eres/dmepit


def convert_velocity_acceleration(va, dmepit):
    """ Convert velocities/accelerations from MKS to motor units.

    Converts the velocity/ies and/or acceleration/s given in MKS units
    to units of motor pitch per second (or second squared), which is
    what the drive requires when given move instructions.

    Parameters
    ----------
    va : int, float, or iterable of ints and floats
        The velocities/accelerations in MKS units to convert.
    dmepit : float
        Electrical pitch of the motor.

    Returns
    -------
    motor_va : float or list of floats
        The velocities/accelerations converted to motor units.

    See Also
    --------
    convert_distance

    """
    if isinstance(va, collections.Iterable):
        return [(x*1e3/dmepit) for x in va]
    else:
        return va*1e3/dmepit
