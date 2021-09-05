#!/usr/bin/env python3

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

    def _set_pty_size(self):
        buf = array.array("h", [0, 0, 0, 0])
        fcntl.ioctl(pty.STDOUT_FILENO, termios.TIOCGWINSZ, buf, True)
        fcntl.ioctl(self.fd, termios.TIOCSWINSZ, buf)

    def _signal_winch(self, signum, frame):
        """
        Signal handler for SIGWINCH - window size has changed.
        """
        self._set_pty_size()

    def spawn(self, args, env=os.environ):
        pid, self.fd = pty.fork()

        if len(args) == 0:
            args = ["/bin/bash"]

        if pid == pty.CHILD:
            # If no arguments run bash
            os.execlpe(args[0], *args, env)

        if sys.stdout.isatty():
            old_handler = signal.signal(signal.SIGWINCH, self._signal_winch)

        try:
            mode = tty.tcgetattr(pty.STDIN_FILENO)
            tty.setraw(pty.STDIN_FILENO)
            restore = True
        except tty.error:  # This is the same as termios.error
            restore = False

        if sys.stdout.isatty():
            self._set_pty_size()

        try:
            pty._copy(self.fd)
        except (IOError, OSError):
            if restore:
                tty.tcsetattr(pty.STDIN_FILENO, tty.TCSAFLUSH, mode)

        os.close(self.fd)

        if sys.stdout.isatty():
            signal.signal(signal.SIGWINCH, old_handler)

        self.fd = None
        return os.waitpid(pid, 0)[1] >> 8
