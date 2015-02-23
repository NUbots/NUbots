#!/usr/bin/python

import sys
import os
import re
from subprocess import Popen, PIPE

# Get the role that we are to be using
role = sys.argv[1];

# Change to the same directory as it
os.chdir(os.path.dirname(sys.argv[1]));

# Run ldd on the program to work out which modules it uses
process = Popen(["ldd", role], stdout=PIPE)
(output, err) = process.communicate()
exit_code = process.wait()

# Get all of our modules
modules = [re.sub(".*\\s+(lib/[A-Za-z]+\\.so).*", "\\1", x) for x in str(output).split('\n') if re.search("\\s+lib/[A-Za-z]+\\.so", x)]

# Run nm on each of our modules
for module in modules:
	process = Popen(["nm", "-g", "-C", module], stdout=PIPE)
	(output, err) = process.communicate()
	exit_code = process.wait()

	# Get our emit related lines
	emits = [re.sub(".*\\w+\\s+\\w+\\s+NUClear::PowerPlant::Emit<(.+)>::emit.*", "\\1", x) for x in str(output).split('\n') if re.search("\\w+\\s+\\w+\\s+NUClear::PowerPlant::Emit<.+>::emit", x)]
	listens = [re.sub(".*\\w+\\s+V\\s+typeinfo\\s+name\\s+for\\s+std::tuple<(.+)>.*", "\\1", x) for x in str(output).split('\n') if re.search("\\w+\\s+V\\s+typeinfo\\s+name\\s+for\\s+std::tuple<(.+)>", x)]

	print emits
	print listens