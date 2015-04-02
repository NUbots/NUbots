#!/usr/bin/python

from subprocess import PIPE,Popen

for f in xrange(25, 30, 5):
    intensity = "%.2f" % (f * 0.01)

    for d in xrange(5, 35, 5):

        print 'Running test for Intensity: {} Duration: {}'.format(intensity, d)

        # Write our path to the log file to run
        with open('build/config/NBZPlayer.yaml', 'w') as f:
            f.write('file: /nubots/NUbots/logs3/Intensity{}Duration{}.nbs\n'.format(intensity, d))
            f.write('replay: false'.format(intensity))

        # Execute our three programs
        process = Popen(['./b', 'role', 'run', 'nbzautoclassifiernone'], stdout=PIPE, stderr=PIPE)
        out, err = process.communicate()
        process.wait()

        with open('newRes2/StaticI{}D{}.csv'.format(intensity, d), 'w') as f:
            f.write('\n'.join([l for l in out.split('\n') if l[:4] == 'Ball' or l[:4] == 'Goal']))

        # Execute our layer test
        process = Popen(['./b', 'role', 'run', 'nbzautoclassifierlayer'], stdout=PIPE, stderr=PIPE)
        out, err = process.communicate()
        process.wait()

        with open('newRes2/LayerI{}D{}.csv'.format(intensity, d), 'w') as f:
            f.write('\n'.join([l for l in out.split('\n') if l[:4] == 'Ball' or l[:4] == 'Goal']))

        # Execute our pressure test
        process = Popen(['./b', 'role', 'run', 'nbzautoclassifierpressure'], stdout=PIPE, stderr=PIPE)
        out, err = process.communicate()
        process.wait()

        with open('newRes2/PressureI{}D{}.csv'.format(intensity, d), 'w') as f:
            f.write('\n'.join([l for l in out.split('\n') if l[:4] == 'Ball' or l[:4] == 'Goal']))
