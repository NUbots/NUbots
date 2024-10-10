#!/usr/bin/env python3
#
# MIT License
#
# Copyright (c) 2019 NUbots
#
# This file is part of the NUbots codebase.
# See https://github.com/NUbots/NUbots for further info.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#

import array
import fcntl
import os
import pty
import signal
import sys
import termios
import tty


# Based on the implementation at https://gist.github.com/jathanism/4489235
class WrapPty:
    def __init__(self):
        self.fd = None
        self.mode = None

        # Old signal handlers
        self.old_winch = None
        self.old_abort = None
        self.old_int = None
        self.old_term = None

    def _set_pty_size(self):
        buf = array.array("h", [0, 0, 0, 0])
        fcntl.ioctl(pty.STDOUT_FILENO, termios.TIOCGWINSZ, buf, True)
        fcntl.ioctl(self.fd, termios.TIOCSWINSZ, buf)

    def _signal_winch(self, signum, frame):
        """
        Signal handler for SIGWINCH - window size has changed.
        """
        self._set_pty_size()

    def _signal_int(self, signum, frame):
        """
        Signal handler for
            SIGABRT - Abort signal
            SIGINT - Interrupt from keyboard (Ctrl + C)
            SIGKILL - Kill signal
            SIGTERM - Termination signal
        Reset the terminal mode and undo signal handlers, then quit
        """
        if self.mode:
            try:
                tty.tcsetattr(pty.STDIN_FILENO, tty.TCSAFLUSH, self.mode)
                self.mode = None
            except tty.error:  # This is the same as termios.error
                pass
        os.close(self.fd)

        # Reset signal handlers
        if sys.stdout.isatty():
            signal.signal(signal.SIGWINCH, self.old_winch)

        signal.signal(signal.SIGABRT, self.old_abort)
        signal.signal(signal.SIGINT, self.old_int)
        signal.signal(signal.SIGTERM, self.old_term)

    def spawn(self, args, env=os.environ, cwd=None):
        if len(args) == 0:
            args = ["/bin/bash"]

        pid, self.fd = pty.fork()

        if pid == pty.CHILD:

            if cwd is not None:
                os.chdir(cwd)

            # If no arguments run bash
            os.execlpe(args[0], *args, env)

        # Catch important signals
        self.old_abort = signal.signal(signal.SIGABRT, self._signal_int)
        self.old_int = signal.signal(signal.SIGINT, self._signal_int)
        self.old_term = signal.signal(signal.SIGTERM, self._signal_int)

        if sys.stdout.isatty():
            self.old_winch = signal.signal(signal.SIGWINCH, self._signal_winch)

        try:
            self.mode = tty.tcgetattr(pty.STDIN_FILENO)
            tty.setraw(pty.STDIN_FILENO)
        except tty.error:  # This is the same as termios.error
            pass

        if sys.stdout.isatty():
            self._set_pty_size()

        try:
            pty._copy(self.fd)
        except (IOError, OSError):
            pass

        # Restore the terminals original settings
        if self.mode:
            tty.tcsetattr(pty.STDIN_FILENO, tty.TCSAFLUSH, self.mode)
            self.mode = None

        os.close(self.fd)

        # Reset signal handlers
        if sys.stdout.isatty():
            signal.signal(signal.SIGWINCH, self.old_winch)

        signal.signal(signal.SIGABRT, self.old_abort)
        signal.signal(signal.SIGINT, self.old_int)
        signal.signal(signal.SIGTERM, self.old_term)

        self.fd = None
        return os.waitpid(pid, 0)[1] >> 8
