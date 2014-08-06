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

""" Module for controlling a Parker Hannifin Gemini GV-6 or GT-6.

Version 0.1.1
"""

__version__ = "0.1.1"

import math
import copy
import re

from . import drivers, utilities


class GeminiError(IOError):
    """Base exception class for this module."""
    pass

class CommandError(GeminiError):
    """Exception executing a command."""
    pass


def get_driver(driver='ASCII_RS232', *args, **keywords):
    """ Gets a driver for a Parker Motion Gemini drive.

    Gets and connects a particular driver in ``drivers`` to a Parker
    Motion Gemini GV-6 or GT-6 servo/stepper motor drive.

    The only driver currently supported is the ``'ASCII_RS232'`` driver
    which corresponds to ``drivers.ASCII_RS232``.

    Parameters
    ----------
    driver : str, optional
        The driver to communicate to the particular driver with, which
        includes the hardware connection and possibly the communications
        protocol. The only driver currently supported is the
        ``'ASCII_RS232'`` driver which corresponds to
        ``drivers.ASCII_RS232``.
    *args : additional positional arguments
        Additional positional arguments to pass onto the constructor for
        the driver.
    **keywords : additional keyword arguments
        Additional keyword arguments to pass onto the constructor for
        the driver.

    Raises
    ------
    NotImplementedError
        If the `driver` is not supported.

    See Also
    --------
    drivers
    drivers.ASCII_RS232

    """
    if driver.upper() == 'ASCII_RS232':
        return drivers.ASCII_RS232(*args, **keywords)
    else:
        raise NotImplementedError('Driver not supported: '
                                      + str(driver))


class GeminiG6(object):
    """ Controller for a Parker Motion Gemini GV-6 or GT-6.

    An object to connect to and control a Parker Motion Gemini GV-6 or
    GT-6 servo/stepper motor drive already connected to with a
    particular `driver`.

    Parameters
    ----------
    driver : driver
        Connected instance of a class in ``drivers``. Use ``get_driver``
        to load one. Is stored in the attribute ``driver``.

    Raises
    ------
    GeminiError
        If the attached device is not a Gemini GV-6 or GT-6.

    Attributes
    ----------
    driver : driver
        Driver for communicating to the drive.
    energized : bool
    denergize_on_kill : bool
    encoder_resolution : int
    electrical_pitch : float
    motion_commanded : bool

    See Also
    --------
    get_driver

    """
    def __init__(self, driver):
        #: Driver for communicating to the drive.
        #:
        #: driver
        #:
        #: A class from ``GeminiMotorDriver.drivers``. Can be loaded
        #: using ``get_driver``.
        #:
        #: See Also
        #: --------
        #: get_driver
        self.driver = driver

        # Make sure that it is indeed a GV/T6, and throw an exception
        # otherwise. It should respond to the 'TREV' command with 'TREV'
        # echoed and '*TREV-GV6-L3E_D1.50_F1.00' where everything after
        # the 'GV6' (possibly replaced with a 'GT6') part is model
        # dependent.
        response = self.driver.send_command('TREV', timeout=1.0,
                                         immediate=True)
        if re.search('^!TREV\r\\*TREV-G[VT]{1}6', response[1]) is None:
            raise GeminiError('Not a valid Gemini GV-6 or GT-6 device.')

    def _get_parameter(self, name, tp, timeout=1.0, max_retries=2):
        """ Gets the specified drive parameter.

        Gets a parameter from the drive. Only supports ``bool``,
        ``int``, and ``float`` parameters.

        Parameters
        ----------
        name : str
            Name of the parameter to check. It is always the command to
            set it but without the value.
        tp : type {bool, int, float}
            The type of the parameter.
        timeout : number, optional
            Optional timeout in seconds to use when reading the
            response. A negative value or ``None`` indicates that the
            an infinite timeout should be used.
        max_retries : int, optional
            Maximum number of retries to do per command in the case of
            errors.

        Raises
        ------
        TypeError
            If 'tp' is not an allowed type (``bool``, ``int``,
            ``float``).
        CommandError
            If the command to retrieve the parameter returned an error.
        ValueError
            If the value returned to the drive cannot be converted to
            the proper type.

        Returns
        -------
        bool, int, or float
            The value of the specified parameter.

        See Also
        --------
        _set_parameter : Set a parameter.

        """
        # Raise a TypeError if tp isn't one of the valid types.
        if tp not in (bool, int, float):
            raise TypeError('Only supports bool, int, and float; not '
                      + str(tp))

        # Sending a command of name queries the state for that
        # parameter. The response will have name preceeded by an '*' and
        # then followed by a number which will have to be converted.
        response = self.driver.send_command(name, timeout=timeout,
                                            immediate=True,
                                            max_retries=max_retries)

        # If the response has an error, there are no response lines, or
        # the first response line isn't '*'+name; then there was an
        # error and an exception needs to be thrown.
        if self.driver.command_error(response) \
                or len(response[4]) == 0 \
                or not response[4][0].startswith('*' + name):
            raise CommandError('Couldn''t retrieve parameter '
                               + name)

        # Extract the string representation of the value, which is after
        # the '*'+name.
        value_str = response[4][0][(len(name)+1):]

        # Convert the value string to the appropriate type and return
        # it. Throw an error if it is not supported.
        if tp == bool:
            return (value_str == '1')
        elif tp == int:
            return int(value_str)
        elif tp == float:
            return float(value_str)

    def _set_parameter(self, name, value, tp, timeout=1.0,
                       max_retries=2):
        """ Sets the specified drive parameter.

        Sets a parameter on the drive. Only supports ``bool``,
        ``int``, and ``float`` parameters.

        Parameters
        ----------
        name : str
            Name of the parameter to set. It is always the command to
            set it when followed by the value.
        value : bool, int, or float
            Value to set the parameter to.
        tp : type {bool, int, float}
            The type of the parameter.
        timeout : number, optional
            Optional timeout in seconds to use when reading the
            response. A negative value or ``None`` indicates that the
            an infinite timeout should be used.
        max_retries : int, optional
            Maximum number of retries to do per command in the case of
            errors.

        Returns
        -------
        bool
            Whether the last attempt to set the parameter was successful
            (``True``) or not (``False`` meaning it had an error).

        See Also
        --------
        _get_parameter : Get a parameter.

        """
        # Return False if tp isn't one of the valid types.
        if tp not in (bool, int, float):
            return False

        # Convert value to the string that the drive will expect. value
        # must first be converted to the proper type before getting
        # converted to str in the usual fasion. As bools need to be a
        # '1' or a '0', it must be converted to int before going through
        # str.
        if tp == bool:
            value_str = str(int(bool(value)))
        elif tp == int:
            value_str = str(int(value))
        elif tp == float:
            value_str = str(float(value))

        # Immediately set the named parameter of the drive. The command
        # is just the parameter name followed by the value string.
        response = self.driver.send_command(name+value_str, \
            timeout=timeout, immediate=True, max_retries=max_retries)

        # Return whether the setting was successful or not.
        return not self.driver.command_error(response)

    def pause(self, max_retries=0):
        """ Pauses the drive (execution of commands).

        Causes the drive to pause execution of commands till it is
        unpaused. Commands will be queued until it is unpaused. Motion
        is not stopped.

        Parameters
        ----------
        max_retries : int, optional
            Maximum number of retries to do to pause the drive in the
            case of errors.

        Returns
        -------
        bool
            Whether the last pause command (last try or retry) returned
            any errors or not.

        Notes
        -----
        The command sent to the drive is '!C'.

        See Also
        --------
        unpause : Unpause the drive.

        """
        return (not self.driver.command_error(
                self.driver.send_command('PS', timeout=1.0,
                immediate=True, max_retries=max_retries)))

    def unpause(self, max_retries=0):
        """ Unpauses the drive.

        Unpauses the drive. Commands queued while it is paused will then
        be executed.

        Parameters
        ----------
        max_retries : int, optional
            Maximum number of retries to do to unpause the drive in the
            case of errors.

        Returns
        -------
        bool
            Whether the last unpause command (last try or retry)
            returned any errors or not.

        Notes
        -----
        The command sent to the drive is '!C'.

        See Also
        --------
        pause : Pause the drive.

        """
        return (not self.driver.command_error(
                self.driver.send_command('C',
                timeout=1.0, immediate=True, max_retries=max_retries)))

    def stop(self, max_retries=0):
        """ Stops motion.

        The drive stops the motor.

        Parameters
        ----------
        max_retries : int, optional
            Maximum number of retries to do to kill the drive in the
            case of errors.

        Returns
        -------
        bool
            Whether the last stop command (last try or retry) returned
            any errors or not.

        Notes
        -----
        The command sent to the drive is '!S1'.

        """
        return (not self.driver.command_error(
                self.driver.send_command('S1',
                timeout=1.0, immediate=True, max_retries=max_retries)))

    def kill(self, max_retries=0):
        """ Kills the drive.

        The drive stops the motor and any running program. The motor
        will de-energize depending on the state of
        :py:attr:`denergize_on_kill`.

        Parameters
        ----------
        max_retries : int, optional
            Maximum number of retries to do to kill the drive in the
            case of errors.

        Returns
        -------
        bool
            Whether the last kill command (last try or retry) returned
            any errors or not.

        Notes
        -----
        The command sent to the drive is '!K'.

        See Also
        --------
        denergize_on_kill : Controls whether the motor de-energizes
            after the drive is killed or not.

        """
        return (not self.driver.command_error(
                self.driver.send_command('K',
                timeout=1.0, immediate=True, max_retries=max_retries)))

    def reset(self, max_retries=0):
        """ Resets the drive.

        Resets the drive, which is equivalent to a power cycling.

        Parameters
        ----------
        max_retries : int, optional
            Maximum number of retries to do to reset the drive in the
            case of errors.

        Returns
        -------
        bool
            Whether the last reset command (last try or retry) returned
            any errors or not.

        Notes
        -----
        The command sent to the drive is '!RESET'.

        """
        return (not self.driver.command_error(
                self.driver.send_command('RESET',
                timeout=10.0, immediate=True, max_retries=max_retries)))

    def get_program(self, n, timeout=2.0, max_retries=2):
        """ Get a program from the drive.

        Gets program 'n' from the drive and returns its commands.

        Parameters
        ----------
        n : int
            Which program to get.
        timeout : number, optional
            Optional timeout in seconds to use when reading the
            response. A negative value or ``None`` indicates that the
            an infinite timeout should be used.
        max_retries : int, optional
            Maximum number of retries to do per command in the case of
            errors.

        Returns
        -------
        list
            ``list`` of ``str`` commands making up the program. The
            trailing 'END' is removed.

        Notes
        -----
        The command sent to the drive is '!TPROG PROGn'.

        See Also
        --------
        set_program_profile : Sets a program or profile.
        run_program_profile : Runs a program or profile.

        """
        # Send the 'TPROG PROGn' command to read the program.
        response = self.driver.send_command( \
            'TPROG PROG' + str(int(n)), timeout=timeout, \
            immediate=True, max_retries=max_retries)

        # If there was an error, then return empty. Otherwise, return
        # the response lines but strip the leading '*' first and the
        # 'END' at the end of the list.
        if self.driver.command_error(response) \
                or len(response[4]) == 0:
            return []
        else:
            if '*END' in response[4]:
                response[4].remove('*END')
            return [line[1:] for line in response[4]]

    def set_program_profile(self, n, commands,
                            program_or_profile='program',
                            timeout=1.0, max_retries=0):
        """ Sets a program/profile on the drive.

        Sets program or profile 'n' on the drive to the sequence of
        commands in 'commands'. If the existing program is identical, it
        is not overwritten (can't check this for a profile). Returns
        whether the program or profile was successfully set or not (if
        the existing one is identical, it is considered a success).

        Parameters
        ----------
        n : int
            Which program to get.
        commands : list or tuple of strings
            ``list`` or ``tuple`` of commands to send to the drive. Each
            command must be a string.
        program_or_profile : {'program', 'profile'}, optional
            Whether to read a program or a profile. Anything other than
            these two values implies the default.
        timeout : number, optional
            Optional timeout in seconds to use when reading the
            response. A negative value or ``None`` indicates that the
            an infinite timeout should be used.
        max_retries : int, optional
            Maximum number of retries to do per command in the case of
            errors.

        Returns
        -------
        bool
            Whether the program or profile was successfully set or not
            (an identical program already existing on the drive is
            considered a success).

        Notes
        -----
        'commands' gets wrapped between ['DEL PROGn', 'DEF PROGn'] and
        'END' or the equivalent profile ones.

        See Also
        --------
        get_program : Gets a program.
        run_program_profile : Runs a program or profile.

        """
        # Grab the n'th program on the drive and strip commands. If we
        # are doing a profile, None will be used as a placeholder.
        if program_or_profile != 'profile':
            current_program = self.get_program(n, timeout=timeout, \
                max_retries=max_retries+2)
        else:
            current_program = None
        stripped_commands = utilities.strip_commands(commands)

        # If the two are identical and we are doing a program, then
        # nothing needs to be done and the program is already set
        # (return True). Otherwise, it needs to be overwritten. If there
        # were no errors on the last command, then it was written
        # successfully. Otherwise, the program or profile needs to be
        # terminated and then deleted.

        if current_program is not None \
                and current_program == stripped_commands:
            return True
        else:
            # Construct the End Of Responses for each command that will
            # be sent. They are '\n' for deletion and ending, but are
            # '\n- ' for the rest.
            eor = ['\n'] + (['\n- '] * (1 + len(stripped_commands))) \
                + ['\n']

            # The commands consist of a header that tells which program
            # or profile to set, the stripped commands, followed by an
            # 'END'.
            if program_or_profile != 'profile':
                header = ['DEL PROG'+str(int(n)),
                          'DEF PROG'+str(int(n))]
            else:
                header = ['DEL PROF'+str(int(n)),
                          'DEF PROF'+str(int(n))]
            responses = self.driver.send_commands(\
                header + stripped_commands + ['END'], \
                timeout=timeout, max_retries=max_retries, eor=eor)

            # Check to see if it was set successfully. If it was (the
            # last command had no errors), return True. Otherwise, the
            # program or profile needs to be ended and deleted before
            # returning False.
            if not self.driver.command_error(responses[-1]):
                return True
            else:
                if program_or_profile != 'profile':
                    cmds = ['END', 'DEL PROG'+str(int(n))]
                else:
                    cmds = ['END', 'DEL PROF'+str(int(n))]
                self.driver.send_commands(cmds, timeout=timeout,
                                           max_retries=max_retries+2)
                return False

    def run_program_profile(self, n, program_or_profile='program',
                            timeout=10.0):
        """ Runs a program/profile on the drive.

        Runs program or profile 'n' on the drive, grabs its output, and
        processes the output. The response from the drive is broken down
        into the echoed command (drive echoes it back), any error
        returned by the drive (leading '*' is stripped), and the
        different lines of the response; which are all returned.

        It is **VERY IMPORTANT** that 'timeout' is long enough for the
        program to run if all the output from the drive is to be
        collected.

        Parameters
        ----------
        n : int
            Which program to get.
        program_or_profile : {'program', 'profile'}, optional
            Whether to read a program or a profile. Anything other than
            these two values implies the default.
        timeout : number, optional
            Optional timeout in seconds to use when reading the
            response for running a program (set to 1.0 for a profile
            regardless of what is given). A negative value or ``None``
            indicates that the an infinite timeout should be used.

        Returns
        -------
        list
            A 5-element ``list``. The elements, in order, are the
            sanitized command (``str``), the full response (``str``),
            the echoed command (``str``), any error response (``None``
            if none, or the ``str`` of the error), and the lines of the
            response that are not the echo or error line (``list`` of
            ``str`` with newlines stripped).

        Notes
        -----
        Sends 'RUN PROGn' (program) or 'PRUN PROFn' (profile) as the
        command to the drive. For a profile, the only output is that
        command echoed back. For a program, it will echo back each
        command in the program (preceeded by an '*' and followed by a
        line feed as opposed to a carriage return).

        See Also
        --------
        get_program : Gets a program.
        set_program_profile : Sets a program or profile.

        """
        if program_or_profile != 'profile':
            return self.driver.send_command('RUN PROG' + str(int(n)), \
                timeout=timeout, immediate=True, eor='*END\n')
        else:
            return self.driver.send_command( \
                'PRUN PROF' + str(int(n)), timeout=1.0, immediate=True)

    @property
    def energized(self):
        """ Energized state of the motor.

        ``bool`` with energized being ``True``.

        Setting it sends an immediate command to the drive to energize
        the motor.

        Notes
        -----
        This uses the 'DRIVE' command.

        """
        return self._get_parameter('DRIVE', bool)

    @energized.setter
    def energized(self, value):
        self._set_parameter('DRIVE', value, bool)

    @property
    def denergize_on_kill(self):
        """ De-energize motor when the drive is killed.

        ``bool`` with ``True`` meaning that whenever the drive is given
        the kill signal, the motor will de-energize.

        Setting it sends an immediate command to the drive to set it.

        Notes
        -----
        This uses the 'KDRIVE' command.

        See Also
        --------
        energized : Get/set the motor energized state.
        kill : Kill the drive.

        """
        return self._get_parameter('KDRIVE', bool)

    @denergize_on_kill.setter
    def denergize_on_kill(self, value):
        self._set_parameter('KDRIVE', value, bool)

    @property
    def encoder_resolution(self):
        """ Encoder/Resolver resolution.

        ``int`` with units counts/rev (servo) or counts/pitch (linear)

        Setting it sends an immediate command to the drive to change the
        encoder/resolver resolution.

        Notes
        -----
        This uses the 'ERES' command.

        """
        return self._get_parameter('ERES', int)

    @encoder_resolution.setter
    def encoder_resolution(self, value):
        self._set_parameter('ERES', value, int)
        self.reset()

    @property
    def electrical_pitch(self):
        """ The motor's electrical pitch.

        float with units of mm

        It gives the spacing between two magnets (full magnetic cycle)
        on a linear motor. Velocities and accelerations are in units of
        pitches/s and pitches/s^2, so it is important.

        Setting it sends an immediate command to the drive to change the
        electrical pitch.

        Notes
        -----
        This uses the 'DMEPIT' command.

        """
        return self._get_parameter('DMEPIT', float)

    @electrical_pitch.setter
    def electrical_pitch(self, value):
        self._set_parameter('DMEPIT', value, float)
        self.reset()

    @property
    def max_velocity(self):
        """ The motor's velocity limit.

        ``float`` in motor units

        Notes
        -----
        This uses the 'DMVLIM' command.

        """
        return self._get_parameter('DMVLIM', float)

    @max_velocity.setter
    def max_velocity(self, value):
        self._set_parameter('DMVLIM', value, float)

    @property
    def motion_commanded(self):
        """ Whether motion is commanded or not.

        ``bool``

        Can't be set.

        Notes
        -----
        It is the value of the first bit of the 'TAS' command.

        """
        rsp = self.driver.send_command('TAS', immediate=True)
        if self.driver.command_error(rsp) or len(rsp[4]) != 1 \
                or rsp[4][0][0:4] != '*TAS':
            return False
        else:
            return (rsp[4][0][4] == '1')
