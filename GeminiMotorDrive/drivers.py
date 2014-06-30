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

"""
"""

import io
import time
import threading

import serial


class ASCII_RS232(object):
    """ ASCII RS232 comm.  driver for a Parker Motion Gemini drive.

    Communications driver to talk to a Parker Motion Gemini drive in
    ASCII mode over RS232.

    Parameters
    ----------
    port : serial port string
        The serial port (RS232) that the Gemini drive is connected to.
    check_echo : bool, optional
        Whether the echoing of the commands as they are being written
        to the drive should be used to correct mistakes in what the
        drive is seeing or not as the default.
    writeTimout : float, optional
        The write timeout for the RS232 port. See ``serial.Serial``.
    interCharTimeout : float or None, optional
        The inter-character timeout for writing on the RS232 port.
        ``None`` disables. See ``serial.Serial``.

    Raises
    ------
    serial.SerialException
        If `port` does not correspond to an available RS232 port or
        can't be opened.

    Notes
    -----
    The ASCII communications settings of the Gemini drive are changed
    while this object is connected and are returned to the default
    values when this object is deleted. Thus, the values of the
    communications settings before this object is created are lost.

    See Also
    --------
    serial.Serial

    """
    def __init__(self, port, check_echo=True, writeTimeout=1.0,
                 interCharTimeout=0.002):
        # Set private variable holding the echo parameters.
        self._check_echo = check_echo

        # Initialize the serial port to connect to the Gemini drive. The
        # only timeout being explicitly set right now is the write
        # timeout. Read timeouts are handled in a more manual fasion.
        self._ser = serial.Serial(port, baudrate=9600,
                                  bytesize=serial.EIGHTBITS,
                                  parity=serial.PARITY_NONE,
                                  stopbits=serial.STOPBITS_ONE,
                                  timeout=None,
                                  writeTimeout=writeTimeout,
                                  interCharTimeout=interCharTimeout,
                                  xonxoff=True, rtscts=False,
                                  dsrdtr=False)

        # It is convenient to have a text wrapper around the serial
        # port for reading and writing.
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
                      check_echo=None):
        """ Send a single command to the drive after sanitizing it.

        Takes a single given `command`, sanitizes it (strips out
        comments, extra whitespace, and newlines), sends the command to
        the drive, and returns the sanitized command. The validity of
        the command is **NOT** checked.

        Parameters
        ----------
        command : str
            The command to send to the Gemini drive.
        immediate : bool, optional
            Whether to make it so the command is executed immediately or
            not.
        timeout : number, optional
            Optional timeout in seconds to use to get the command right
            when we are doing echo checking. A negative value or
            ``None`` indicates that the an infinite timeout should be
            used.
        check_echo : bool or None, optional
            Whether the echoing of the command as it is being written to
            the drive should be used to correct mistakes in what the
            drive is seeing, or whether the default set when the
            instance of this class was created shoudl be used
            (``None``).

        Returns
        -------
        sanitized_command : str
            The sanitized command that was sent to the drive.

        """
        # Use the default echo checking if None was given.
        if check_echo is None:
            check_echo = self._check_echo

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

            # A timer will be made that takes timeout to finish. Then,
            # it is a matter of checking whether it is alive or not to
            # know whether the timeout was exceeded or not. min does
            # have to be used to make sure that a timeout greater than
            # the maximum allowed TIMEOUT_MAX is not used. Then, the
            # timer is started.
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
        eor : str or iterable of str, optional
            ``str`` or iterable of ``str`` that denote the allowed
            End Of Response. For most commands, it should be
            ``('\\n', '\\n- ')``, but for running a program, it should
            be ``'*END\\n'``. The default is ``('\\n', '\\n- ')``.

        Returns
        -------
        response : str
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
            # A timer will be made that takes timeout to finish. Then,
            # it is a matter of checking whether it is alive or not to
            # know whether the timeout was exceeded or not. min does
            # have to be used to make sure that a timeout greater than
            # the maximum allowed TIMEOUT_MAX is not used. Then, the
            # timer is started.
            tm = threading.Timer(min(timeout, threading.TIMEOUT_MAX),
                                 lambda : None)
            tm.start()

            # eor needs to be converted to bytes. If it is just an str,
            # it needs to be wrapped in a tuple.
            if isinstance(eor, str):
                eor = tuple([eor])
            eor = [s.encode(encoding='ASCII') for s in eor]

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
        processed_response : list
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
        error : bool
            ``True`` if there was an error and ``False`` otherwise.

        """
        # The command should be echoed back accurately (might be
        # preceeded by a '- ' if it is part of a program definition) and
        # no errors should be returned, if it has no errors.
        return (response[2] not in [response[0], '- ' + response[0]]
                or response[3] is not None)

    def send_command(self, command, immediate=False, timeout=1.0,
                     max_retries=0, eor=('\n', '\n- ')):
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
        timeout : float or None, optional
            Optional timeout in seconds to use when reading the
            response. A negative value or ``None`` indicates that the
            an infinite timeout should be used.
        max_retries : int, optional
            Maximum number of retries to do per command in the case of
            errors.
        eor : str or iterable of str, optional
            ``str`` or an iterable of ``str`` that denote the allowed
            End Of Response. For most commands, it should be
            ``('\\n', '\\n- ')``, but for running a program, it should
            be ``'*END\\n'``. The default is ``('\\n', '\\n- ')``.

        Returns
        -------
        output : list
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

        >>> ra = rs232_ascii('/dev/ttyS1')
        >>> ra.send_command('DRIVE1', immediate=False, timeout=1.0)
        ['DRIVE1', 'DRIVE1\\r', 'DRIVE1', None, []]

        Same command but made immediate.

        >>> ra = rs232_ascii('/dev/ttyS1')
        >>> ra.send_command('DRIVE1', immediate=True, timeout=1.0)
        ['!DRIVE1', '!DRIVE1\\r', '!DRIVE1', None, []]

        Same command with a typo.

        >>> ra = rs232_ascii('/dev/ttyS1')
        >>> ra.send_command('DRIV1', immediate=False, timeout=1.0)
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
                        immediate=immediate)]
            output = self._get_response(timeout=timeout, eor=eor)
            # If echo checking was done, the echo was already grabbed,
            # is identical to the command, and needs to be placed back
            # in front of the output so that it can be processed
            # properly.
            if self._check_echo:
                output = response[0] + output
            response.extend(self._process_response(output))
            # We are done if there is no error.
            if not self.command_error(response):
                break
            # Put in a slight pause so the drive has a bit of breathing
            # time between retries.
            time.sleep(0.25)
        return response

    def send_commands(self, commands, timeout=1.0,
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
        ``send_command`` and collates the outputs.

        Parameters
        ----------
        commands : iterable of str
            Iterable of commands to send to the drive. Each command must
            be an ``str``.
        timeout : float or None, optional
            Optional timeout in seconds to use when reading the
            response. A negative value or ``None`` indicates that the
            an infinite timeout should be used.
        max_retries : int, optional
            Maximum number of retries to do per command in the case of
            errors.
        eor : str or iterable of str, optional
            End Of Resonse. An EOR is either a ``str`` or an iterable
            of ``str`` that denote the possible endings of a response.
            'eor' can be a single EOR, in which case it is used for all
            commands, or it can be an iterable of EOR to use for each
            individual command. For most commands, it should be
            ``('\\n', '\\n- ')``, but for running a program, it should
            be ``'*END\\n'``. The default is ``('\\n', '\\n- ')``.

        Returns
        -------
        outputs : list of lists
            ``list`` composed of the processed responses of each command
            in the order that they were done up to and including the
            last command executed. See ``send_command`` for the format
            of processed responses.

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

        >>> ra = rs232_ascii('/dev/ttyS1')
        >>> ra.send_commands(['DRIVE1', 'D-10000', 'GO']
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
                                    max_retries=max_retries,
                                    eor=eor[i])
            responses.append(rsp)
            if self.command_error(rsp):
                break
            # Put in a slight pause so the drive has a bit of breathing
            # time between commands.
            time.sleep(0.25)
        return responses
