#!/usr/bin/env python

import os
import fnmatch
import glob
import hashlib
import tempfile
import b

from termcolor import cprint
from subprocess import call, STDOUT

def register(command):

    # Install help
    command.help = 'Install the system onto the target drone'

    # Drone arguments
    command.add_argument('ip_addr'
        , metavar='ip_addr'
        , help='the IP address of the target to install to')

    command.add_argument('hostname'
        , metavar='hostname'
        , help='the hostname of the target to install to')

    command.add_argument('-c', '--config'
        , metavar='config'
        , choices=['', 'new', 'update', 'overwrite', 'pull', 'ignore']
        , default='new'
        , help='method to use for configuration files when installing')

    command.add_argument('-u', '--user'
        , metavar='username'
        , default='hive'
        , help='the username to use when installing')

def run(ip_addr, hostname, config, user, **kwargs):

    # Target location to install to
    target_dir   = '{0}@{1}:/home/{0}/'.format(user, ip_addr)
    build_dir    = b.binary_dir
    config_dir   = os.path.join(build_dir, 'config')
    platform_dir = '/nubots/toolchain/{0}'.format(b.cmake_cache["PLATFORM"])
    roles        = b.cmake_cache["NUCLEAR_ROLES"].split(';')

    cprint('Installing binaries to ' + target_dir, 'blue', attrs=['bold'])
    files = glob.glob(os.path.join(build_dir, 'bin', '*'))
    call(['rsync', '-avzPl', '--checksum', '-e ssh'] + files + [target_dir])

    # Get all of our required shared libraries in our toolchain and send them
    cprint('Installing toolchain library files', 'blue', attrs=['bold'])
    libs = glob.glob('{0}/lib/*.so*'.format(platform_dir))
    call(['rsync', '-avzPl', '--checksum', '-e ssh'] + libs + [target_dir + 'toolchain'])

    # Find all data files and send them
    # Data files are located in the build directory (mixed in with the make files and CMake cache).
    cprint('Installing data files to ' + target_dir, 'blue', attrs=['bold'])
    files = [fn for fn in glob.glob(os.path.join(build_dir, '*'))
             if not os.path.basename(fn).endswith('ninja') and 
                not os.path.basename(fn).lower().startswith('cmake') and 
                not os.path.isdir(fn)]
    call(['rsync', '-avzPL', '--checksum', '-e ssh'] + files + [target_dir])

    # Set rpath for all libs on the remote machine
    cprint('Setting rpath for all toolchain libs to {0}'.format(target_dir + 'toolchain'), 'blue', attrs=['bold'])
    command = 'for lib in /home/{0}/toolchain/*.so*; do patchelf --set-rpath /home/{0}/toolchain $lib; done'.format(user)
    host = '{0}@{1}'.format(user, ip_addr)
    cprint('Running {0} on {1}'.format(command, host), 'blue', attrs=['bold'])
    FNULL = open(os.devnull, 'w')
    call(['ssh', host, command], stdout=FNULL, stderr=STDOUT)
    FNULL.close()

    # Get list of config files.
    config_files = glob.glob('{0}/*.yaml'.format(config_dir))
                 + glob.glob('{0}/{1}/*.yaml'.format(config_dir, hostname))
                 + [glob.glob('{0}/{1}/*.yaml'.format(config_dir, role)) for role in roles]

    if config in ['overwrite', 'o']:
        cprint('Overwriting configuration files on target', 'blue', attrs=['bold'])
        call(['rsync', '-avzPL', '--checksum', '-e ssh'] + config_files + [target_dir])

    if config in ['update', 'u']:
        cprint('Adding new configuration files to target', 'blue', attrs=['bold'])
        call(['rsync', '-avzuPL', '--checksum', '-e ssh'] + config_files + [target_dir])

    if not config or config in ['new', 'n']:
        cprint('Adding new configuration files to the target', 'blue', attrs=['bold'])
        call(['rsync', '-avzPL', '--checksum', '--ignore-existing', '-e ssh'] + config_files + [target_dir])

    if config in ['ignore', 'i']:
        cprint('Ignoring configuration changes', 'blue', attrs=['bold'])

    if config in ['pull', 'p']:
        cprint('Pulling updated configuration files from the target to a temporary directory', 'blue', attrs=['bold'])

        # Rsync to a local directory
        path = tempfile.mkdtemp()
        call(['rsync', '-avzuPL', '--checksum', '-e ssh', target_dir + 'config', path])


        cprint('Updating original configuraiton files', 'blue', attrs=['bold'])
        # Copy the data over to the original files
        for root, dirnames, filenames in os.walk(path):
            for filename in fnmatch.filter(filenames, '*.yaml'):

                src = os.path.join(root, filename)
                dst = os.path.relpath(src, path)

                if os.path.isfile(dst):
                    # Check our hashes
                    src_hash = hashlib.md5(open(src, 'rb').read()).hexdigest()
                    dst_hash = hashlib.md5(open(dst, 'rb').read()).hexdigest()

                    if src_hash != dst_hash:
                        cprint('Pulling updated file ' + dst, 'green', attrs=['bold'])
                        with open(dst, 'wb') as o:
                            with open(src, 'rb') as i:
                                o.write(i.read())

                else:
                    with open(dst, 'wb') as o:
                            with open(src, 'rb') as i:
                                o.write(i.read())

                    cprint('File ' + dst + ' does not belong to any module, you will need to manually move it into place', 'red', attrs=['bold'])
