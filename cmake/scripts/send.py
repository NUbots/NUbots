#!/usr/bin/python

import os
import fnmatch
import tempfile
import argparse
import hashlib
import glob
from termcolor import cprint
from subprocess import call

parser = argparse.ArgumentParser()

parser.add_argument('--robot_ip')
parser.add_argument('--config')
parser.add_argument('--username')

args = parser.parse_args()

robot_ip = args.robot_ip
robot_id = robot_ip[-1] # TODO: unhack this

config = args.config
username = args.username if args.username else 'darwin'

if not robot_ip:
    raise 'Robot IP not given (robot=ip)'

target_dir = '{0}@{1}:/home/{0}/'.format(username, robot_ip)

# Copy the binaries over
cprint('Installing binaries', 'blue', attrs=['bold'])
files = glob.glob('bin/*')
call(['rsync', '-avzPl', '--checksum', '-e ssh'] + files + [target_dir])

# Get all of our required shared libraries in our toolchain and send them
cprint('Installing toolchain library files', 'blue', attrs=['bold'])
libs = glob.glob('/nubots/toolchain/lib/*.so*')
call(['rsync', '-avzPl', '--checksum', '-e ssh'] + libs + [target_dir + 'toolchain'])

# Overwrite configuration files
if config in ['overwrite', 'o']:
    cprint('Updating configuration files', 'blue', attrs=['bold'])
    call(['rsync', '-avzPL', '--checksum', '-e ssh', 'config', target_dir])

    # Override robot specific versions
    cprint('Installing robot specific configuration', 'blue', attrs=['bold'])
    robot_config = glob.glob('config/robot_' + robot_id + '/*')
    call(['rsync', '-avzPL', '--checksum', '-e ssh'] + robot_config + [target_dir + 'config'])

# Update configuration files
elif config in ['update', 'u']:
    cprint('Adding new configuration files only', 'blue', attrs=['bold'])
    call(['rsync', '-avzuPL', '--checksum', '-e ssh', 'config', target_dir])

    # Override robot specific versions
    cprint('Installing robot specific configuration', 'blue', attrs=['bold'])
    robot_config = glob.glob('config/robot_' + robot_id + '/*')
    call(['rsync', '-avzuPL', '--checksum', '-e ssh'] + robot_config + [target_dir + 'config'])

# Pull configuration files off the robot
elif config in ['pull', 'p']:
    cprint('Pulling config from the robot', 'blue', attrs=['bold'])

    # Rsync to a local directory
    path = tempfile.mkdtemp()
    call(['rsync', '-avzuPL', '--checksum', '-e ssh', target_dir + 'config', path])

    # Copy the data over to the original files
    for root, dirnames, filenames in os.walk(path):
        for filename in fnmatch.filter(filenames, '*.yaml'):

            src = os.path.join(root, filename)
            dst = os.path.relpath(src, path)

            if os.path.isfile(dst):
                # Check our hashes
                srcHash = hashlib.md5(open(src, 'rb').read()).hexdigest()
                dstHash = hashlib.md5(open(dst, 'rb').read()).hexdigest()

                if srcHash != dstHash:
                    cprint('Pulling updated file ' + dst, 'green', attrs=['bold'])
                    with open(dst, 'wb') as o:
                        with open(src, 'rb') as i:
                            o.write(i.read())

            else:
                with open(dst, 'wb') as o:
                        with open(src, 'rb') as i:
                            o.write(i.read())

                cprint('File ' + dst + ' does not belong to any module, you will need to manually move it into place', 'red', attrs=['bold'])

# Add new configuration files
elif not config or config in ['new', 'n']:
    cprint('Adding new configuration files only', 'blue', attrs=['bold'])
    call(['rsync', '-avzPL', '--checksum', '--ignore-existing', '-e ssh', 'config', target_dir])

    # Override robot specific versions
    cprint('Installing robot specific configuration', 'blue', attrs=['bold'])
    robot_config = glob.glob('config/robot_' + robot_id + '/*')
    call(['rsync', '-avzPL', '--checksum', '--ignore-existing', '-e ssh'] + robot_config + [target_dir + 'config'])

# Ignore configuration files
elif config in ['ignore', 'i']:
    cprint('Ignoring configuration changes', 'blue', attrs=['bold'])

# Bad configuration parameters
else:
    cprint('The configuration parameters must be either one of update, new, pull, bidirectional or ignore (u, n, p, b or i)', 'red', attrs=['bold'])
