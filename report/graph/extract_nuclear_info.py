#!/usr/bin/python

import sys
import re
from subprocess import Popen, PIPE

if sys.argv[1]:
    input_file = sys.argv[1];
else:
    print 'You must specify an input file\n';
    sys.exit(1);

if sys.argv[2]:
    output_file = sys.argv[2];
else:
    print 'You must specify an output file\n';
    sys.exit(1);

# Open our output file for writing
with open(output_file, 'w') as file:

    # Run nm on the file
    process = Popen(["nm", "-a", "--radix=d", "--demangle", "--format=bsd", input_file], stdout=PIPE);
    (output, err) = process.communicate();
    exit_code = process.wait();

    # If our nm command failed then exit with the error
    if(exit_code != 0):
        print err;
        exit(exit_code);

    # Get all the lines
    lines = str(output).split('\n');

    # Regexes to extract the On<> related information
    on_regex = [
        re.compile(r'^\d+ V typeinfo name for std::tuple<(.+)>$'),
        re.compile(r'^\d+ V typeinfo for std::tuple<(.+)>$')
    ];

    # Regexes to extract the emit information
    emit_regex = [
        re.compile(r'^\d+ W NUClear::PowerPlant::Emit<(.+)>::emit\(.+\)$'),
        re.compile(r'^\d+ W void NUClear::PowerPlant::ReactorMaster::directEmit<(.+)>\(.+\)$'),
        re.compile(r'^\d+ W void NUClear::PowerPlant::ReactorMaster::emitOnStart<(.+)>\(.+\)$'),
        re.compile(r'^\d+ W void NUClear::PowerPlant::ReactorMaster::emit<(.+)>\(.+\)$'),
        re.compile(r'^\d+ W void NUClear::PowerPlant::emit<(.+)>\(.+\)$'),
        re.compile(r'^\d+ W void NUClear::Reactor::emit<(.+)>\(.+\)$')
    ];

    # Regexes to exctract the cache information
    cache_regex = [
        re.compile(r'^\d+ W void NUClear::PowerPlant::CacheMaster::cache<(.+)>\(.+\)$'),
        re.compile(r'^\d+ W NUClear::metaprogramming::TypeMap<(NUClear::PowerPlant::CacheMaster,.+)>::get\(\)$'),
        re.compile(r'^\d+ W NUClear::metaprogramming::TypeMap<(NUClear::PowerPlant::CacheMaster,.+)>::set\(.+\)$'),
        re.compile(r'^\d+ u NUClear::metaprogramming::TypeMap<(NUClear::PowerPlant::CacheMaster,.+)>::data$'),
        re.compile(r'^\d+ u NUClear::metaprogramming::TypeMap<(NUClear::PowerPlant::CacheMaster,.+)>::mutex$')
    ];

    # Regexes to extract the TypeList <Trigger> information
    trigger_list_regex = [
        re.compile(r'^\d+ W NUClear::metaprogramming::TypeList<(NUClear::Reactor,.+)>::get\(\)$'),
        re.compile(r'^\d+ u NUClear::metaprogramming::TypeList<(NUClear::Reactor,.+)>::data$')
    ];

    # Regexes to extract the exists information
    exists_regex = [
        re.compile(r'^\d+ W void NUClear::Reactor::Exists<(.+)>::exists\(.+\)$')
    ];

    # Regexes to extract the Get<> information
    get_regex = [
        re.compile(r'^\d+ W NUClear::PowerPlant::CacheMaster::Get<(.+)>::get\(.+\)$')
    ];

    # Declare our lists
    raw_on           = [[r.sub(r'\1', x) for x in lines if r.match(x)] for r in on_regex];
    raw_emit         = [[r.sub(r'\1', x) for x in lines if r.match(x)] for r in emit_regex];
    raw_cache        = [[r.sub(r'\1', x) for x in lines if r.match(x)] for r in cache_regex];
    raw_trigger_list = [[r.sub(r'\1', x) for x in lines if r.match(x)] for r in trigger_list_regex];
    raw_exists       = [[r.sub(r'\1', x) for x in lines if r.match(x)] for r in exists_regex];
    raw_get          = [[r.sub(r'\1', x) for x in lines if r.match(x)] for r in get_regex];

    # TODO a lot of this information is redundant and should be merged together
    # TODO however existing in different regex groups means different things
    # TODO do some processing to make a json file of the interaces on this file
    for i, val in enumerate(raw_on):
        file.write("\n\nOn<> Statements " + str(i) + " \n\t");
        file.write('\n\t'.join(val));

    for i, val in enumerate(raw_emit):
        file.write("\n\nEmit<> Statements " + str(i) + " \n\t");
        file.write('\n\t'.join(val));

    for i, val in enumerate(raw_cache):
        file.write("\n\nCache<> Statements " + str(i) + " \n\t");
        file.write('\n\t'.join(val));

    for i, val in enumerate(raw_trigger_list):
        file.write("\n\nTrigger<> Statements " + str(i) + " \n\t");
        file.write('\n\t'.join(val));

    for i, val in enumerate(raw_exists):
        file.write("\n\nExists<> Statements " + str(i) + " \n\t");
        file.write('\n\t'.join(val));

    for i, val in enumerate(raw_get):
        file.write("\n\nGet<> Statements " + str(i) + " \n\t");
        file.write('\n\t'.join(val));
