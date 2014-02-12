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

Version 0.1
"""

__version__ = "0.1"

import math
import copy
import io
import time
import threading
import re

import serial


class GeminiError(IOError):
    """Base exception class for this module."""
    pass

class CommandError(GeminiError):
    """Exception executing a command."""
    pass


class GeminiVersions(object):
    """ Representation of supported Parker Motion Gemini drive versions.

    Represents the minimum versions of the different Parker Motion
    Gemini drives to support something. An ``int`` denotes the first
    version to support something while ``None`` means the whole type of
    drive is not supported.

    Parameters
    ----------
    GT : int or None
        Gemini GT version.
    GV : int or None
        Gemini GV version.
    GT6 : int or None
        Gemini GT6 version.
    GV6 : int or None
        Gemini GV6 version.

    Attributes
    ----------
    versions : tuple
    GT : int or None
    GV : int or None
    GT6 : int or None
    GV6 : int or None

    """
    def __init__(self, GT, GV, GT6, GV6):
        self._versions = copy.deepcopy((GT, GV, GT6, GV6))

    @property
    def versions(self):
        """ Drive versions.

        ``tuple`` of drive versions in GT, GV, GT6, GV6 order. Each are
        either an ``int`` denoting the versions they are first supported
         in, or ``None`` if they are not supported.

        """
        return copy.deepcopy(self._versions)

    @property
    def GT(self):
        """ GT version.

        ``int`` of the first version to support it or ``None`` if it
        isn't supported.

        """
        return self._versions[0]

    @property
    def GV(self):
        """ GV version.

        ``int`` of the first version to support it or ``None`` if it
        isn't supported.

        """
        return self._versions[1]

    @property
    def GT6(self):
        """ GT6 version.

        ``int`` of the first version to support it or ``None`` if it
        isn't supported.

        """
        return self._versions[2]

    @property
    def GV6(self):
        """ GV6 version.

        ``int`` of the first version to support it or ``None`` if it
        isn't supported.

        """
        return self._versions[3]


class GeminiG6(object):
    """ Controller for a Parker Motion Gemini GV-6 or GT-6.

    An object to connect to and control a Parker Motion Gemini GV-6 or
    GT-6 servo/stepper motor drive via RS232 (serial port). The drive
    must be setup to be running in ASCII communications mode.

    Parameters
    ----------
    port : serial port string
        The serial port (RS232) that the Gemini drive is connected to.

    Raises
    ------
    serial.SerialException
        If `port` does not correspond to an available RS232 port or
        can't be opened, the drive can't be successfully communicated
        with, or if the attached device is not a Gemini GV-6 or GT-6.
        The communications settings are restored to their defaults.

    Attributes
    ----------
    energized : bool
    denergize_on_kill : bool
    encoder_resolution : int
    electrical_pitch : float
    motion_commanded : bool

    Notes
    -----
    The ASCII communications settings of the Gemini drive are changed
    while this object is connected and are returned to the default
    values when this object is deleted. Thus, the values of the
    communications settings before this object is created are lost.

    """
    def __init__(self, port):
        # Initialize the serial port to connect to the Gemini GV6. The
        # only timeout being explicitly set right now is the write
        # timeout. Read timeouts are handled in a more manual fasion.
        self._ser = serial.Serial(port, baudrate=9600,
                                  bytesize=serial.EIGHTBITS,
                                  parity=serial.PARITY_NONE,
                                  stopbits=serial.STOPBITS_ONE,
                                  timeout=None, writeTimeout=1.0,
                                  interCharTimeout=0.002,
                                  xonxoff=True, rtscts=False,
                                  dsrdtr=False)

        # It is convenient to have a text wrapper around the serial
        # port for reading.

        self._sio = io.TextIOWrapper(io.BufferedRWPair(self._ser,
                                     self._ser, 1), newline='\n',
                                     encoding='ASCII')

        # Change the communications parameters so that commands are
        # echoed, on error level 4, no characters are used to preceed
        # each response, carriage returns are used for newlines in
        # responses, responses are terminated by a '\n', and there are
        # no prompts (there are separate prompts depending on whether
        # the previous command had an error or not). The echo command is
        # the one command that echo checking cannot be done on since
        # echo may not be enabled yet.

        self._send_command('ECHO1', check_echo=False, immediate=True)
        self._send_command('ERRLVL4', immediate=True)
        self._send_command('BOT0,0,0', immediate=True)
        self._send_command('EOT10,0,0', immediate=True)
        self._send_command('EOL13,0,0', immediate=True)
        self._send_command('ERRBAD0,0,0,0', immediate=True)
        self._send_command('ERROK0,0,0,0', immediate=True)

        # Wait a little while for the commands to be processed and then
        # discard all the responses.

        time.sleep(2)
        self._ser.read(self._ser.inWaiting())

        # Make sure that it is indeed a GV6, and throw an exception
        # otherwise. It should respond to the 'TREV' command with 'TREV'
        # echoed and '*TREV-GV6-L3E_D1.50_F1.00' where everything after
        # the 'GV6' (possibly replaced with a 'GT6') part is model
        # dependent.

        response = self.send_command('TREV', timeout=1.0,
                                     immediate=True)
        if re.search('^!TREV\r\\*TREV-G[VT]{1}6', response[1]) is None:
            # Return all communicatsions parameters to their default
            # values (from the manual).
            self._send_command('ECHO1', immediate=True)
            self._send_command('ERRLVL4', immediate=True)
            self._send_command('BOT0,0,0', immediate=True)
            self._send_command('EOT13,0,0', immediate=True)
            self._send_command('EOL13,10,0', immediate=True)
            self._send_command('ERRBAD13,10,63,32', immediate=True)
            self._send_command('ERROK13,10,62,32', immediate=True)
            # Wait a little while for the commands to be processed and
            # then discard all the responses.
            time.sleep(2)
            self._ser.read(self._ser.inWaiting())
            raise serial.SerialException('Not a valid Gemini GV-6 '
                                         + 'or GT-6 device.')

    def __del__(self):
        """ Returns all communications settings to their defaults.
        """
        # Return all communicatsions parameters to their default values
        # (from the manual).
        self._send_command('ECHO1', immediate=True)
        self._send_command('ERRLVL4', immediate=True)
        self._send_command('BOT0,0,0', immediate=True)
        self._send_command('EOT13,0,0', immediate=True)
        self._send_command('EOL13,10,0', immediate=True)
        self._send_command('ERRBAD13,10,63,32', immediate=True)
        self._send_command('ERROK13,10,62,32', immediate=True)
        # Wait a little while for the commands to be processed and then
        # discard all the responses.
        time.sleep(2)
        self._ser.read(self._ser.inWaiting())


    def _send_command(self, command, immediate=False, timeout=1.0,
                      check_echo=True):
        """ Send a single command to the drive after sanitizing it.

        Takes a single given `command`, sanitizes it (strips out
        comments, extra whitespace, and newlines), sends the command to
        the drive, and returns the sanitized command. The validity of
        the command is **NOT** checked.

        Parameters
        ----------
        command : str
            The command to send to the Gemini drive.
        timeout : number, optional
            Optional timeout in seconds to use to get the command right
            when we are doing echo checking. A negative value or
            ``None`` indicates that the an infinite timeout should be
            used.
        immediate : bool, optional
            Whether to make it so the command is executed immediately or
            not.
        check_echo : bool, optional
            Whether the echoing of the command as it is being written to
            the drive should be used to correct mistakes in what the
            drive is seeing.

        Returns
        -------
        str
            The sanitized command that was sent to the drive.

        """
        # Convert to bytes and then strip comments, whitespace, and
        # newlines.
        c = bytes(command, encoding='ASCII')
        c = c.split(b';')[0].strip()

        # If the command is supposed to be immediate, insure that it
        # starts with an '!'.
        if immediate and not c.startswith(b'!'):
            c = b'!' + c

        # Read out any junk on the serial port before we start.
        self._ser.read(self._ser.inWaiting())

        # The command needs to be written a character at a time with
        # pauses between them to make sure nothing gets lost or
        # corrupted. This is a simple loop if we are not checking the
        # echo. If we are, it is more complicated.
        if not check_echo:
            for i in range(0, len(c)):
                self._ser.write(bytes([c[i]]))
                time.sleep(0.01)
        else:
            # If we are doing an infinite timeout, set it to the maximum
            # a threading.Timer is allowed.
            if timeout is None or timeout <= 0:
                timeout = threading.TIMEOUT_MAX

            # A timer will be made that take timeout to finish. Then, it
            # is a matter of checking whether it is alive or not to know
            # whether the timeout was exceeded or not. min does have to
            # be used to make sure that a timeout greater than the
            # maximum allowed TIMEOUT_MAX is not used. Then, the timer
            # is started.
            tm = threading.Timer(min(timeout, threading.TIMEOUT_MAX),
                                 lambda : None)
            tm.start()

            # Each character needs to be written one by one while the
            # echo is collected. If any mistakes occur, they need to be
            # corrected with backspaces b'\x08'. The echo starts out
            # empty. We go until either the echo is identical to the
            # command or the timeout is exceeded.
            echo = b''
            while c != echo and tm.is_alive():
                # If there are no mistakes, then echo will be the
                # beginning of c meaning the next character can be
                # written. Otherwise, there is a mistake and a backspace
                # needs to be written.
                if c.startswith(echo):
                    self._ser.write(bytes([c[len(echo)]]))
                else:
                    self._ser.write(b'\x08')

                # Pause for a bit to make sure nothing gets lost. Then
                # read the drive's output add it to the echo.
                time.sleep(0.01)
                echo += self._ser.read(self._ser.inWaiting())

                # All backspaces in echo need to be processed. Each
                # backspace deletes itself and the character before it
                # (if any).
                while b'\x08' in echo:
                    index = echo.index(b'\x08')
                    if index == 0:
                        echo = echo[1:]
                    else:
                        echo = echo[0:(index-1)] + echo[(index+1):]

            # Turn off the timer in the case that it is still running
            # (command completely written before timeout).
            tm.cancel()

        # Write the carriage return to enter the command and then return
        # the sanitized command.
        self._ser.write(b'\r')
        return c.decode(errors='replace')

    def _get_response(self, timeout=1.0, eor=('\n', '\n- ')):
        """ Reads a response from the drive.

        Reads the response returned by the drive with an optional
        timeout. All carriage returns and linefeeds are kept.

        Parameters
        ----------
        timeout : number, optional
            Optional timeout in seconds to use when reading the
            response. A negative value or ``None`` indicates that the
            an infinite timeout should be used.
        eor : str or tuple of str, optional
            ``str`` or ``tuple`` of ``str`` that denote the allowed
            End Of Response. For most commands, it should be
            ``('\\n', '\\n- ')``, but for running a program, it should
            be ``'*END\\n'``. The default is ``('\\n', '\\n- ')``.

        Returns
        -------
        str
            The response obtained from the drive. Carriage returns and
            linefeeds are preserved.

        """
        # If no timeout is given or it is invalid and we are using '\n'
        # as the eor, use the wrapper to read a line with an infinite
        # timeout. Otherwise, the reading and timeout must be
        # implemented manually.
        if (timeout is None or timeout < 0) and eor == '\n':
            return self._sio.readline()
        else:
            # A timer will be made that take timeout to finish. Then, it
            # is a matter of checking whether it is alive or not to know
            # whether the timeout was exceeded or not. min does have to
            # be used to make sure that a timeout greater than the
            # maximum allowed TIMEOUT_MAX is not used. Then, the timer
            # is started.
            tm = threading.Timer(min(timeout, threading.TIMEOUT_MAX),
                                 lambda : None)
            tm.start()

            # eor needs to be converted to bytes. If it is just an str,
            # it needs to be wrapped in a tuple.
            if isinstance(eor, str):
                eor = tuple([eor])
            eor = tuple([s.encode(encoding='ASCII') for s in eor])

            # Read from the serial port into buf until the EOR is found
            # or the timer has stopped. A small pause is done each time
            # so that this thread doesn't hog the CPU.
            buf = b''
            while not any([(x in buf) for x in eor]) and tm.is_alive():
                time.sleep(0.001)
                buf += self._ser.read(self._ser.inWaiting())

            # Just in case the timer has not stopped (EOR was found),
            # stop it.
            tm.cancel()

            # Remove anything after the EOR if there is one. First, a
            # set of matches (index, eor_str) for each string in eor
            # needs to be constructed. Sorting the matches by their
            # index puts all the ones that were not found (index of -1)
            # at the front. Then a list of bools that are True for each
            # index that isn't -1 is made, converted to a bytes (True
            # goes to b'\x01' and False goes to b'\x00'), and then the
            # index of the first True value found. If it is not -1, then
            # there was a successful match and  all the characters are
            # dropped after that eor_str.
            matches = [(buf.find(x), x) for x in eor]
            matches.sort(key=lambda x: x[0])
            index = bytes([x[0] != -1 for x in matches]).find(b'\x01')
            if index != -1:
                buf = buf[:(matches[index][0] + len(matches[index][1]))]

            # Convert to an str before returning.
            return buf.decode(errors='replace')

    def _process_response(self, response):
        """ Processes a response from the drive.

        Processes the response returned from the drive. It is broken
        down into the echoed command (drive echoes it back), any error
        returned by the drive (leading '*' is stripped), and the
        different lines of the response.

        Parameters
        ----------
        response : str
            The response returned by the drive.

        Returns
        -------
        list
            A 4-element ``list``. The elements, in order, are `response`
            (``str``), the echoed command (``str``), any error response
            (``None`` if none, or the ``str`` of the error), and the
            lines of the response that are not the echo or error line
            (``list`` of ``str`` with newlines stripped).

        """
        # Strip the trailing newline and split the response into lines
        # by carriage returns.
        rsp_lines = response.rstrip('\r\n').split('\r')

        # If we have at least one line, the first one is the echoed
        # command. If available, it needs to be grabbed and that line
        # removed from rsp_lines since it is just the echoing, not the
        # actual response to the command. None will be used to denote a
        # non-existent echo.
        if len(rsp_lines) > 0:
            echoed_command = rsp_lines[0]
            del rsp_lines[0]
        else:
            echoed_command = None

        # If the next line is one of the different possible error
        # strings, then there was an error that must be grabbed (leading
        # '*' is stripped). If there was an error, remove that line from
        # the response. None will be used to denote the lack of an error.
        if len(rsp_lines) > 0 and \
                rsp_lines[0] in ('*INVALID_ADDRESS', '*INVALID_DATA', \
                '*INVALID_DATA_HIGH', '*INVALID_DATA_LOW', \
                '*UNDEFINED_LABEL'):
            err = rsp_lines[0][1:]
            del rsp_lines[0]
        else:
            err = None

        return [response, echoed_command, err, rsp_lines]

    def command_error(self, response):
        """ Checks whether a command produced an error.

        Checks whether a command procuded an error based on its
        processed response. The two types of errors are an error
        returned by the drive and the command that the drive received
        being different than the one that was sent (error in
        transmission).

        Parameters
        ----------
        response : processed response (list)
            The processed response ``list`` for the command that was
            executed.

        Returns
        -------
        bool
            ``True`` if there was an error and ``False`` otherwise.

        """
        # The command should be echoed back accurately (might be
        # preceeded by a '- ' if it is part of a program definition) and
        # no errors should be returned, if it has no errors.
        return (response[2] not in [response[0], '- ' + response[0]]
                or response[3] is not None)

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
        response = self.send_command(name, timeout=timeout,
                                     immediate=True,
                                     max_retries=max_retries)

        # If the response has an error, there are no response lines, or
        # the first response line isn't '*'+name; then there was an
        # error and an exception needs to be thrown.
        if self.command_error(response) or len(response[4]) == 0 \
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
        _get_parameter :Get a parameter.

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
        response = self.send_command(name+value_str,
                                     timeout=1.0,
                                     immediate=True, max_retries=2)

        # Return whether the setting was successful or not.
        return not self.command_error(response)

    def strip_commands(self, commands):
        """ Strips a list of commands.

        Strips down the list of commands by removing comments and
        surrounding whitespace around each individual command and then
        removing blank commands.

        Parameters
        ----------
        commands : list or tuple of strings
            ``list`` or ``tuple`` of commands to send to the drive. Each
            command must be a string.

        Returns
        -------
        list
            ``list`` of ``str``. Contains the stripped commands with
            blank ones removed.
        """
        # Go through each command one by one, stripping it and adding it
        # to a growing list if it is not blank. Each command needs to be
        # converted to an str if it is a bytes.
        stripped_commands = []
        for i, v in enumerate(commands):
            if isinstance(v, bytes):
                v = v.decode(errors='replace')
            v = v.split(';')[0].strip()
            if len(v) != 0:
                stripped_commands.append(v)
        return stripped_commands


    def send_command(self, command, immediate=False, timeout=1.0,
                     check_echo=True, max_retries=0,
                     eor=('\n', '\n- ')):
        """ Sends a single command to the drive and returns output.

        Takes a single given `command`, sanitizes it, sends it to the
        drive, reads the response, and returns the processed response.
        The command is first sanitized by removing comments, extra
        whitespace, and newline characters. If `immediate` is set, the
        command is made to be an immediate command. Note, the command is
        **NOT** checked for validity. If the drive returns an error, the
        command is re-executed up to `max_tries` more times. The
        response from the final execution is processed and returned. The
        response from the drive is broken down into the echoed command
        (drive echoes it back), any error returned by the drive (leading
        '*' is stripped), and the different lines of the response; which
        are all returned.

        Parameters
        ----------
        command : str
            The command to send to the Gemini drive.
        immediate : bool, optional
            Whether to make it so the command is executed immediately or
            not.
        timeout : number, optional
            Optional timeout in seconds to use when reading the
            response. A negative value or ``None`` indicates that the
            an infinite timeout should be used.
        check_echo : bool, optional
            Whether the echoing of the command as it is being written to
            the drive should be used to correct mistakes in what the
            drive is seeing.
        max_retries : int, optional
            Maximum number of retries to do per command in the case of
            errors.
        eor : str or tuple of str, optional
            ``str`` or ``tuple`` of ``str`` that denote the allowed
            End Of Response. For most commands, it should be
            ``('\\n', '\\n- ')``, but for running a program, it should
            be ``'*END\\n'``. The default is ``('\\n', '\\n- ')``.

        Returns
        -------
        list
            A 5-element ``list``. The elements, in order, are the
            sanitized command (``str``), the full response (``str``),
            the echoed command (``str``), any error response (``None``
            if none, or the ``str`` of the error), and the lines of the
            response that are not the echo or error line (``list`` of
            ``str`` with newlines stripped).

        See Also
        --------
        send_commands : Send multiple commands.

        Examples
        --------

        Simple command energizing the motor with no response and no
        errors.

        >>> g = GeminiG6('/dev/ttyS1')
        >>> g.send_command('DRIVE1', immediate=False, timeout=1.0)
        ['DRIVE1', 'DRIVE1\\r', 'DRIVE1', None, []]

        Same command but made immediate.

        >>> g = GeminiG6('/dev/ttyS1')
        >>> g.send_command('DRIVE1', immediate=True, timeout=1.0)
        ['!DRIVE1', '!DRIVE1\\r', '!DRIVE1', None, []]

        Same command with a typo.

        >>> g = GeminiG6('/dev/ttyS1')
        >>> g.send_command('DRIV1', immediate=False, timeout=1.0)
        ['DRIV1', 'DRIV1\\r*UNDEFINED_LABEL\\n', 'DRIV1',
         'UNDEFINED_LABEL', []]

        Simple command asking whether the motor is energized or not.

        >>> g = GeminiG6('/dev/ttyS1')
        >>> g.send_command('DRIVE', immediate=False, timeout=1.0)
        ['DRIVE', 'DRIVE\\r*DRIVE1\\n', 'DRIVE', None, ['*DRIVE1']]

        """
        # Execute the command till it either doesn't have an error or
        # the maximum number of retries is exceeded.
        for i in range(0, max_retries+1):
            # Send the command and stuff the sanitized version in a
            # list. Then process the response and add it to the list.
            response = [self._send_command(command,
                        immediate=immediate, check_echo=check_echo)]
            output = self._get_response(timeout=timeout, eor=eor)
            # If echo checking was done, the echo was already grabbed,
            # is identical to the command, and needs to be placed back
            # in front of the output so that it can be processed
            # properly.
            if check_echo:
                output = response[0] + output
            response.extend(self._process_response(output))
            # We are done if there is no error.
            if not self.command_error(response):
                break
            # Put in a slight pause so the drive has a bit of breathing
            # time between retries.
            time.sleep(0.25)
        return response

    def send_commands(self, commands, timeout=1.0, check_echo=True,
                      max_retries=1, eor=('\n', '\n- ')):
        """ Send a sequence of commands to the drive and collect output.

        Takes a sequence of many commands and executes them one by one
        till either all are executed or one runs out of retries
        (`max_retries`). Retries are optionally performed if a command's
        repsonse indicates that there was an error. Remaining commands
        are not executed. The processed output of the final execution
        (last try or retry) of each command that was actually executed
        is returned.

        This function basically feeds commands one by one to
        :py:func:`send_command` and collates the outputs.

        Parameters
        ----------
        commands : list or tuple of strings
            ``list`` or ``tuple`` of commands to send to the drive. Each
            command must be a string.
        timeout : number, optional
            Optional timeout in seconds to use when reading the
            response. A negative value or ``None`` indicates that the
            an infinite timeout should be used.
        check_echo : bool, optional
            Whether the echoing of the commands as they are being
            written to the drive should be used to correct mistakes in
            what the drive is seeing.
        max_retries : int, optional
            Maximum number of retries to do per command in the case of
            errors.
        eor : optional
            End Of Resonse. An EOR is either a ``str`` or a ``tuple``
            of ``str`` that denote the possible endings of a response.
            'eor' can be a single EOR, in which case it is used for all
            commands, or it can be a ``list`` of EOR to use for each
            individual command. For most commands, it should be
            ``('\\n', '\\n- ')``, but for running a program, it should
            be ``'*END\\n'``. The default is ``('\\n', '\\n- ')``.

        Returns
        -------
        list
            ``list`` composed of the processed responses of each command
            in the order that they were done up to and including the
            last command executed. See :py:func:`send_command` for the
            format of processed responses.

        See Also
        --------
        send_command : Send a single command.

        Examples
        --------

        A sequence of commands to energize the motor, move it a bit away
        from the starting position, and then do 4 forward/reverse
        cycles, and de-energize the motor. **DO NOT** try these specific
        movement distances without checking that the motion won't damage
        something (very motor and application specific).

        >>> g = GeminiG6('/dev/ttyS1')
        >>> g.send_commands(['DRIVE1', 'D-10000', 'GO']
                            + ['D-10000','GO','D10000','GO']*4
                            + [ 'DRIVE0'])
        [['DRIVE1', 'DRIVE1\\r', 'DRIVE1', None, []],
         ['D-10000', 'D-10000\\r', 'D-10000', None, []],
         ['GO', 'GO\\r', 'GO', None, []],
         ['D-10000', 'D-10000\\r', 'D-10000', None, []],
         ['GO', 'GO\\r', 'GO', None, []],
         ['D10000', 'D10000\\r', 'D10000', None, []],
         ['GO', 'GO\\r', 'GO', None, []],
         ['D-10000', 'D-10000\\r', 'D-10000', None, []],
         ['GO', 'GO\\r', 'GO', None, []],
         ['D10000', 'D10000\\r', 'D10000', None, []],
         ['GO', 'GO\\r', 'GO', None, []],
         ['D-10000', 'D-10000\\r', 'D-10000', None, []],
         ['GO', 'GO\\r', 'GO', None, []],
         ['D10000', 'D10000\\r', 'D10000', None, []],
         ['GO', 'GO\\r', 'GO', None, []],
         ['D-10000', 'D-10000\\r', 'D-10000', None, []],
         ['GO', 'GO\\r', 'GO', None, []],
         ['D10000', 'D10000\\r', 'D10000', None, []],
         ['GO', 'GO\\r', 'GO', None, []],
         ['DRIVE0', 'DRIVE0\\r', 'DRIVE0', None, []]]


        """
        # If eor is not a list, make a list of it replicated enough for
        # every command.
        if not isinstance(eor, list):
            eor = [eor]*len(commands)
        # Do every command one by one, collecting the responses and
        # stuffing them in a list. Commands that failed are retried, and
        # we stop if the last retry is exhausted.
        responses = []
        for i, command in enumerate(commands):
            rsp = self.send_command(command, timeout=timeout,
                                    check_echo=check_echo,
                                    max_retries=max_retries,
                                    eor=eor[i])
            responses.append(rsp)
            if self.command_error(rsp):
                break
            # Put in a slight pause so the drive has a bit of breathing
            # time between commands.
            time.sleep(0.25)
        return responses

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
        return (not self.command_error(self.send_command('PS',
                timeout=1.0, immediate=True, max_retries=max_retries)))

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
        return (not self.command_error(self.send_command('C',
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
        return (not self.command_error(self.send_command('K', \
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
        return (not self.command_error(self.send_command('RESET',
                timeout=10.0, immediate=True, max_retries=max_retries)))

    def get_program(self, n, timeout=2.0, max_retries=2):
        """ Get a program from the drive.

        Gets program 'n' from the drive and returns its commands.

        Parameters
        ---------
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
        response = self.send_command('TPROG PROG' + str(int(n)),
                                     timeout=timeout,
                                     immediate=True,
                                     max_retries=max_retries)

        # If there was an error, then return empty. Otherwise, return
        # the response lines but strip the leading '*' first and the
        # 'END' at the end of the list.
        if self.command_error(response) or len(response[4]) == 0:
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
        ---------
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
        stripped_commands = self.strip_commands(commands)

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
            responses = self.send_commands(header
                                           + stripped_commands
                                           + ['END'],
                                           timeout=timeout,
                                           max_retries=max_retries,
                                           eor=eor)

            # Check to see if it was set successfully. If it was (the
            # last command had no errors), return True. Otherwise, the
            # program or profile needs to be ended and deleted before
            # returning False.
            if not self.command_error(responses[-1]):
                return True
            else:
                if program_or_profile != 'profile':
                    cmds = ['END', 'DEL PROG'+str(int(n))]
                else:
                    cmds = ['END', 'DEL PROF'+str(int(n))]
                self.send_commands(cmds, timeout=timeout,
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
        ---------
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
            return self.send_command('RUN PROG' + str(int(n)),
                                     timeout=timeout, immediate=True,
                                     eor='*END\n')
        else:
            return self.send_command('PRUN PROF' + str(int(n)),
                                     timeout=1.0, immediate=True)

    def make_move_sequence(self, cycles, program_or_profile='program'):
        """ Makes the command list for a move sequence.

        Constructs the list of commands to execute the given sequence of
        motion. Program/command line commands or profile commands can be
        generated depending on the value of `program_or_profile` so that
        the commands can be used to construct a program or profile
        later. Types of motion supported (see Notes for how to specify)
        are moves from one position to another (the motion will always
        come to a stop before doing the next motion), waiting a given
        interval of time till starting the next move, and looping over a
        sequence of moves.

        Everything is in motor units which are encoder counts for
        distance, pitches/s for velocity, and pitches/s^2 for
        acceleration.

        Parameters
        ----------
        cycles : list of dicts
            The ``list`` of cycles of motion to do one after another.
            See Notes for format.
        program_or_profile : {'program', 'profile'}, optional
            Whether program or profile motion commands should be used.
            Anything other than these two values implies the default.

        Returns
        -------
        list
            ``list`` of ``str`` commands making up the move sequence.

        Notes
        -----
        `cycles` is a ``list`` of individual cycles of motion. Each
        cycle is a ``dict`` that represents a sequence of moves that
        could possibly be looped over. The field 'iterations' gives how
        many times the sequence of moves should be done (a value > 1
        implies a loop). Then the field 'moves' is a ``list`` of the
        individual moves. Each individual move is a ``dict`` with the
        acceleration ('A'), deceleration ('AD' with 0 meaning the
        value of the acceleration is used), velocity ('V'), and the
        distance/position ('D'). Back in the cycle, the field
        'wait_times' is a ``list`` of numbers giving the time in seconds
        to wait after each move before going onto the next.

        Examples
        --------

        Simple program style two motions with a pause in between.

        >>> g = gemini.GeminiG6('/dev/ttyS1')
        >>> cycles = [{'iterations':1, 'wait_times':[1, 0],
                      'moves':[{'A':100, 'AD':0, 'D':-1000, 'V':100},
                      {'A':90, 'AD':0, 'D':-1000, 'V':100}]}]
        >>> g.make_move_sequence(cycles)
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

        >>> g = gemini.GeminiG6('/dev/ttyS1')
        >>> cycles = [{'iterations':1, 'wait_times':[1, 0],
                      'moves':[{'A':100, 'AD':0, 'D':-1000, 'V':100},
                      {'A':90, 'AD':0, 'D':-1000, 'V':100}]}]
        >>> g.make_move_sequence(cycles, program_or_profile='profile')
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

        Another motion with a back and forth loop (100 iterations) in
        the middle, done in program style commands.

        >>> g = gemini.GeminiG6('/dev/ttyS1')
        >>> cycles = [{'iterations':1, 'wait_times':[1],
                      'moves':[{'A':100, 'AD':0, 'D':-1000, 'V':100}]},
                      {'iterations':100, 'wait_times':[0, 0],
                      'moves':[{'A':50, 'AD':40, 'D':-1000, 'V':30},
                      {'A':50, 'AD':40, 'D':1000, 'V':30}]},
                      {'iterations':1, 'wait_times':[0],
                      'moves':[{'A':100, 'AD':0, 'D':1000, 'V':100}]}]
        >>> g.make_move_sequence(cycles)
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
        # Initially, we have no commands in our command list.
        commands = []

        # The A, AD, D, and V parameters of the previous motion should
        # be kept track of because if they don't change from one motion
        # to the next, the commands to set them don't need to be
        # included. They will be started blank since there are no
        # previous motions yet.
        previous_motion = {'A': [], 'AD': [], 'D': [], 'V': []}

        # Construct each cycle one by one.
        for cycle in cycles:
            # If more than one iteration is being done, a loop needs to
            # be setup. It will be either 'L' or 'PLOOP' with the number
            # of iterations attached if it is a program or a profile
            # respectively. Since it will be tough to keep track of what
            # motion changed from the end of a loop to the beginning of
            # it, it is easier to just forget all previous motion values
            # and set them all at the beginning of the loop (clear
            # previous_motion).
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
                        # Grab it and round it to 4 places after the
                        # decimal point because that is the most that is
                        # supported. Then, if it is an integer value,
                        # convert it to an integer because that is what
                        # the drive will send back if requested (makes
                        # comparisons easier). Then add the command.
                        val = round(float(new_motion[k]), 4)
                        if val == int(val):
                            val = int(val)
                        commands.append(k + str(val))

                # If the sign of D has flipped, we just need to issue a
                # 'D~' command. If the value has changed in another way,
                # it needs to be reset.
                if previous_motion['D'] != new_motion['D']:
                    if previous_motion['D'] == -new_motion['D']:
                        commands.append('D~')
                    else:
                        commands.append('D'
                                        + str(int(new_motion['D'])))

                # Grab the amount of time that should be waited after
                # the move is done.
                wait_time = cycle['wait_times'][i]

                # Give the motion command (GO or GOBUF), tell the drive
                # to wait till the motor has stopped (a WAIT command if
                # it is a program and a VF0 command if it is a profile),
                # and make it wait the period of time wait_time (T and
                # GOWHEN commands).
                if program_or_profile != 'profile':
                    commands.append('GO1')
                    commands.append('WAIT(AS.1=b0)')
                    if wait_time != 0:
                        # The wait time needs to be rounded to 3 places
                        # after the decimal. If it is an integer, it
                        # should be converted to an int so that the
                        # drive will send back what we send (makes
                        # compairisons easier).
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

                # Before going onto the next move, previous_motion needs
                # to be set to the one just done.
                previous_motion = new_motion

            # Done with all the moves of the cycle. If we are looping,
            # the loop end needs to be put in.
            if iterations > 1:
                if program_or_profile != 'profile':
                    commands.append('LN')
                else:
                    commands.append('PLN')

        # Done constructing the command list.
        return commands

    def get_move_sequence_time(self, cycles, eres=None):
        """ Calculates the time the move sequence will take to complete.

        Calculates the amount of time it will take to complete the given
        move sequence. Types of motion supported are moves from one
        position to another (the motion will always come to a stop
        before doing the next motion), waiting a given interval of time
        till starting the next move, and looping over a sequence of
        moves.

        Everything is in motor units which are encoder counts for
        distance, pitches/s for velocity, and pitches/s^2 for
        acceleration.

        Parameters
        ----------
        cycles : list of dicts
            The ``list`` of cycles of motion to do one after another.
            See :py:meth:`make_move_sequence` for format.
        eres : int, optional
            Encoder resolution. Set to ``None`` to use the value
            obtained from the drive.

        Returns
        -------
        float
            Time the move sequence will take.

        See Also
        --------
        make_move_sequence

        """
        # Grab the encoder resolution.
        if eres is None:
            eres = self.encoder_resolution

        # Starting with 0 time, steadily add the time of each movement.
        tme = 0.0

        # Go through each cycle and collect times.
        for cycle in cycles:
            # Add all the wait times.
            tme += cycle['iterations']*sum(cycle['wait_times'])

            # Add the time for each individual move.
            for move in cycle['moves']:
                tme += cycle['iterations'] \
                    * self.move_time(move, eres=eres)

        # Done.
        return tme

    def move_time(self, move, eres=None):
        """ Calculates the time it takes to do a move.

        Calculates how long it will take to complete a move of the
        motor. It is assumed that the motor will decerate to a stop for
        the end of the move as opposed to keep moving at velocity.

        Everything is in motor units which are encoder counts for
        distance, pitches/s for velocity, and pitches/s^2 for
        acceleration.

        Parameters
        ----------
        move : dict
            Contains the move parameters in its fields: acceleration
            ('A'), deceleration ('AD' with 0 meaning the value of the
            acceleration is used), velocity ('V'), and the
            distance/position ('D').

        eres : int, optional
            Encoder resolution. Set to ``None`` to use the value
            obtained from the drive.

        Returns
        -------
        float
            Time the move will take.

        See Also
        --------
        make_move_sequence

        """
        # Grab the encoder resolution.
        if eres is None:
            eres = self.encoder_resolution

        # Grab the move parameters. If the deceleration is given as
        # zero, that means it has the same value as the
        # acceleration. Distance is converted to the same units as the
        # others by dividing by the encoder resolution. The absolute
        # value of everything is taken for simplicity.
        A = abs(move['A'])
        AD = abs(move['AD'])
        if AD == 0.0:
            AD = A
        V = abs(move['V'])
        D = abs(move['D'])/eres

        # Calculate the times it would take to accelerate from stop to V
        # and decelerate to stop at rates A and AD respectively.

        accel_times = [V/A, V/AD]

        # Calculate the distances that would moved in those times.

        dists = [0.5*A*(accel_times[0]**2), 0.5*AD*(accel_times[1]**2)]

        # If the sum of those dists is greater than D, then the velocity
        # V is never reached. The way the time is calculated depends on
        # which case it is.

        if sum(dists) <= D:
            # The time is just the sum of the acceleration times plus
            # the remaining distance divided by V.
            return (sum(accel_times) + (D-sum(dists))/V)
        else:
            # We need to find the time it takes for the acceleration
            # path and deceleration path to meet, or in other words,
            # when the sum of their distances is D.
            #
            # D = 0.5*A*(t**2) + 0.5*AD*(t**2)
            # t**2 = D / (0.5 * (A + AD))
            return math.sqrt(2*D / (A + AD))


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
        rsp = self.send_command('TAS', immediate=True)
        if self.command_error(rsp) or len(rsp[4]) != 1 \
                or rsp[4][0][0:4] != '*TAS':
            return False
        else:
            return (rsp[4][0][4] == '1')


class LinearMotorGV6(GeminiG6):
    """ Controller for a linear motor attached to a Gemini GV-6.

    An object to connect to and control a linear motor attatched to a
    Parker Motion Gemini GV-6 motor drive via RS232 (serial port). The
    drive must be setup to be running in ASCII communications mode.

    Parameters
    ----------
    port : serial port string
        The serial port (RS232) that the Gemini drive is connected to.

    Raises
    ------
    serial.SerialException
        If `port` does not correspond to an available RS232 port or
        can't be opened, the drive can't be successfully communicated
        with, or if the attached device is not a Gemini GV-6.
        The communications settings are restored to their defaults.

    """
    def __init__(self, port):
        GeminiG6.__init__(self, port)

    def convert_distances(self, distances, dmepit=None, eres=None):
        """ Convert distances from MKS to motor units.

        Converts the distance/s given in MKS units to units of motor
        ecnoder counts, which is what the drive requires when given move
        instructions.

        Parameters
        ----------
        distances : int, float, or iterable of ints and floats
            The distance/s in MKS units to convert.
        dmepit : float, optional
            Electrical pitch of the motor to use. Set to ``None`` to
            use the value obtained from the drive.
        eres : int, optional
            Encoder resolution. Set to ``None`` to use the value
            obtained from the drive.

        Returns
        -------
        float or list of floats
            The distance/s converted to units of encoder counts.

        See Also
        --------
        electrical_pitch
        encoder_resolution

        """
        if dmepit is None:
            dmepit = self.electrical_pitch
        if eres is None:
            eres = self.encoder_resolution
        if isinstance(distances, (int, float)):
            return distances*1e3*eres/dmepit
        else:
            return [(x*1e3*eres/dmepit) for x in distances]

    def convert_velocities_accelerations(self, vas, dmepit=None):
        """ Convert velocities/accelerations from MKS to motor units.

        Converts the velocities and/or accelerations given in MKS units
        to units of motor pitch per second (or second squared), which is
        what the drive requires when given move instructions.

        Parameters
        ----------
        vas : int, float, or iterable of ints and floats
            The velocities/accelerations in MKS units to convert.
        dmepit : float, optional
            Electrical pitch of the motor to use. Set to ``None`` to
            use the value obtained from the drive.

        Returns
        -------
        float or list of floats
            The velocities/accelerations converted to motor units.

        See Also
        --------
        electrical_pitch

        """
        if dmepit is None:
            dmepit = self.electrical_pitch
        if isinstance(vas, (int, float)):
            return vas*1e3/dmepit
        else:
            return [(x*1e3/dmepit) for x in vas]

    def convert_move_sequence(self, cycles, dmepit=None, eres=None):
        """ Converts a move sequence from MKS units to motor units.

        Converts a move sequence from MKS units to motor units. Types of
        motion supported are moves from one position to another (the
        motion will always come to a stop before doing the next motion),
        waiting a given interval of time till starting the next move,
        and looping over a sequence of moves.

        Parameters
        ----------
        cycles : list of dicts
            The ``list`` of cycles of motion to do one after another.
            See :py:meth:`GeminiGV6.make_move_sequence` for format.
        dmepit : float, optional
            Electrical pitch of the motor to use. Set to ``None`` to
            use the value obtained from the drive.
        eres : int, optional
            Encoder resolution. Set to ``None`` to use the value
            obtained from the drive.

        Returns
        -------
        cycles
            A deep copy of `cycles` with all units converted to motor
            units.

        See Also
        --------
        GeminiGV6.make_move_sequence

        """
        # Grab the electrical pith and encoder resolution so that the
        # converters don't keep grabbing them and taking up a lot of
        # time.
        if dmepit is None:
            dmepit = self.electrical_pitch
        if eres is None:
            eres = self.encoder_resolution

        # Make a deep copy of cycles so that the conversions don't
        # damage the original one.

        cv_cycles = copy.deepcopy(cycles)

        # Go through each cycle and do the conversions.
        for cycle in cv_cycles:
            # Go through each of the moves and do the conversions.
            for move in cycle['moves']:
                move['A'] = self.convert_velocities_accelerations( \
                    move['A'], dmepit=dmepit)
                move['AD'] = self.convert_velocities_accelerations( \
                    move['AD'], dmepit=dmepit)
                move['V'] = self.convert_velocities_accelerations( \
                    move['V'], dmepit=dmepit)
                move['D'] = int(self.convert_distances(move['D'],
                                                       dmepit=dmepit,
                                                       eres=eres))

        # Now return the converted move sequence.
        return cv_cycles

    def make_move_sequence_mks(self, cycles,
                               program_or_profile='program',
                               dmepit=None, eres=None):
        """ Makes the command list for a move sequence in MKS units.

        Constructs the list of commands to execute the given sequence of
        motion. All conversions of MKS units to motor units are handled
        automatically. Program/command line commands or profile commands
        can be generated depending on the value of `program_or_profile`
        so that the commands can be used to construct a program or
        profile later. Types of motion supported (see Notes for how to
        specify) are moves from one position to another (the motion will
        always come to a stop before doing the next motion), waiting a
        given interval of time till starting the next move, and looping
        over a sequence of moves.

        Parameters
        ----------
        cycles : list of dicts
            The ``list`` of cycles of motion to do one after another.
            See :py:meth:`GeminiGV6.make_move_sequence` for format.
        program_or_profile : {'program', 'profile'}, optional
            Whether program or profile motion commands should be used.
            Anything other than these two values implies the default.
        dmepit : float, optional
            Electrical pitch of the motor to use. Set to ``None`` to
            use the value obtained from the drive.
        eres : int, optional
            Encoder resolution. Set to ``None`` to use the value
            obtained from the drive.

        Returns
        -------
        list
            ``list`` of ``str`` commands making up the move sequence.

        See Also
        --------
        GeminiGV6.make_move_sequence

        """
        # Grab the electrical pith and encoder resolution so that the
        # converters don't keep grabbing them and taking up a lot of
        # time.
        if dmepit is None:
            dmepit = self.electrical_pitch
        if eres is None:
            eres = self.encoder_resolution

        # Make the move sequence.
        return (self.make_move_sequence(
                self.convert_move_sequence(cycles, dmepit=dmepit,
                eres=eres), program_or_profile=program_or_profile))

    def get_move_sequence_time_mks(self, cycles):
        """ Calculates the time the move sequence will take to complete.

        Calculates the amount of time it will take to complete the given
        move sequence (in MKS units). Types of motion supported are
        moves from one position to another (the motion will always come
        to a stop before doing the next motion), waiting a given
        interval of time till starting the next move, and looping over a
        sequence of moves.

        Parameters
        ----------
        cycles : list of dicts
            The ``list`` of cycles of motion to do one after another.
            See :py:meth:`GeminiG6.make_move_sequence` for format.

        Returns
        -------
        float
            Time the move sequence will take.

        See Also
        --------
        GeminiG6.make_move_sequence

        """
        # We can just use cycles as is with an eres of 1 since
        # distances, velocities, and accelerations are already using the
        # same units for distance.
        return self.get_move_sequence_time(cycles, eres=1.0)

    @property
    def max_velocity_mks(self):
        """ The motor's velocity limit.

        ``float`` in m/s

        Notes
        -----
        This uses the 'DMVLIM' command.

        """
        return (self.electrical_pitch
                * self._get_parameter('DMVLIM', float)
                / 1e3)

    @max_velocity_mks.setter
    def max_velocity_mks(self, value):
        self._set_parameter('DMVLIM', \
            self.convert_velocities_accelerations(value), float)
