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
