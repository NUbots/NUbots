import sys
import glob
import argparse
import subprocess
from docker import Docker

def register(command):

    # Set our commands help
    command.help = 'Send the binary and/or configuration files to a robot'

    # Docker subcommands
    command.add_argument('target', metavar='target', help='the target robot to send to, either an IP address, or a shorthand address (e.g. d5 for Darwin 5)')
    command.add_argument('-c', '--configuration', help='What configuration settings to') # TODO better help
    command.add_argument('-u', '--user', help='The user to connect to the robot as') # TODO better help

def run(target, configuration=None, user=None, **kwargs):

    # Setup docker
    docker = Docker()

    # Run ourselves in dockerland to give us unix things!
    docker.run('python', '../tools/build/send.py', *sys.argv[2:], interactive=True)

# When we are re-run from within the dockerland we can do fun things!
if __name__ == '__main__':

    # Rebuild our arguments using argparse
    command = argparse.ArgumentParser()
    register(command)
    args = command.parse_args()

    # TODO work out our robot's ip
    # d = darwin
    # then number
    # then e is ethernet else blah


    # Work out our target address
    target_address = '{0}@{1}:/home/{0}/'.format(args.user if args.user else 'darwin', args.target)
    print target_address
    # # Copy the binaries over
    # files = glob.glob('bin/*')
    # subprocess.call(['rsync', '-avzP', '--checksum', '-e ssh'] + files + [target_dir])

    # # Overwrite configuration files
    # if args.configuration in ['overwrite', 'o']:
    #     print 'Updating configuration files'
    #     call(['rsync', '-avzP', '--checksum', '-e ssh', 'config', target_dir])

    # # Update configuration files
    # elif args.configuration in ['update', 'u']:
    #     print 'Adding new configuration files only'
    #     call(['rsync', '-avzuP', '--checksum', '-e ssh', 'config', target_dir])

    # # Add new configuration files
    # elif not args.configuration or args.configuration in ['new', 'n']:
    #     print 'Adding new configuration files only'
    #     call(['rsync', '-avzP', '--checksum', '--ignore-existing', '-e ssh', 'config', target_dir])

    # # Ignore configuration files
    # elif args.configuration in ['ignore', 'i']:
    #     print "Ignoring configuration changes"

    # # Bad configuration parameters
    # else:
    #     print 'The configuration parameters must be either one of update, new or ignore (u, n or i)'


#     # TODO do something with rsync or something