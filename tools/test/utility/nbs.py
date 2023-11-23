#
# MIT License
#
# Copyright (c) 2023 NUbots
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
import os.path
import unittest
from tempfile import gettempdir

from numpy.lib.function_base import _select_dispatcher

from utility.nbs import Encoder, LinearDecoder, MessageTypes

Say = MessageTypes["message.output.Say"].type


class nbs_io(unittest.TestCase):
    """Tests the logic in utility.nbs"""

    def setUp(self):
        self.path = os.path.join(gettempdir(), "test.nbs")
        self.true_data = []
        for i in range(256):
            say = Say()
            say.message = str(i)
            self.true_data.append(say)

    def test_write_read_data(self):
        with Encoder(self.path) as out:
            # Write
            for m in self.true_data:
                # We set the timestamp to 1, as we don't care about it
                out.write(1, m)

        decoder = LinearDecoder(self.path)
        # Read
        data = [x.msg for x in decoder]

        # Compare
        self.assertEqual(data, self.true_data)
