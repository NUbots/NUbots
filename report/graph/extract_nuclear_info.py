#!/usr/bin/python

import sys
import re
import ctypes
import pyparsing as pp
from subprocess import Popen, PIPE


if sys.argv[1]:
    input_file = sys.argv[1]
else:
    print 'You must specify an input file\n'
    sys.exit(1)

if sys.argv[2]:
    output_file = sys.argv[2]
else:
    print 'You must specify an output file\n'
    sys.exit(1)

if sys.argv[3]:
    demangler = sys.argv[3]

    # Start up our demangler
    demangler = ctypes.cdll.LoadLibrary(demangler)
    demangler.demangle.argtypes = [ctypes.c_char_p]
    demangler.demangle.restype = ctypes.c_char_p
else:
    print 'You must provide a demangler\n'
    sys.exit(1)

# Our namespaced type (potentially containing templates) e.g. `a::b::c<x::y>`
nsType = pp.Forward()

# Fundamental types (types built into c++)
fundamentalType = (pp.Literal('bool')
                 | pp.Literal('unsigned char')
                 | pp.Literal('signed char')
                 | pp.Literal('char')
                 | pp.Literal('short int')
                 | pp.Literal('short')
                 | pp.Literal('int')
                 | pp.Literal('signed short int')
                 | pp.Literal('signed short')
                 | pp.Literal('signed int')
                 | pp.Literal('signed')
                 | pp.Literal('unsigned short int')
                 | pp.Literal('unsigned short')
                 | pp.Literal('unsigned int')
                 | pp.Literal('unsigned')
                 | pp.Literal('long long int')
                 | pp.Literal('long long')
                 | pp.Literal('long int')
                 | pp.Literal('long')
                 | pp.Literal('signed long long int')
                 | pp.Literal('signed long long')
                 | pp.Literal('signed long int')
                 | pp.Literal('signed long')
                 | pp.Literal('unsigned long long int')
                 | pp.Literal('unsigned long long')
                 | pp.Literal('unsigned long int')
                 | pp.Literal('unsigned long'))

# An enum type e.g. `(a::b::c)1`
enumType = pp.Suppress('(') + nsType + pp.Suppress(')') + pp.Word(pp.nums)

# Match a template
templateType = pp.Group(pp.Suppress('<') + pp.Optional(pp.delimitedList(pp.Group(nsType | enumType | pp.Empty()))) + pp.Suppress('>'))

# Match a cType (text then maybe a template)
cType = pp.Word(pp.alphanums + '_') + pp.Optional(templateType)

# A function type (type followed by function arguments)
funcType = cType + pp.Suppress('(') + pp.Optional(pp.delimitedList(pp.Group(nsType))) + pp.Suppress(')')

# A lambda type (looks like) {lambda(a,b,c)#1}
lambdaType = pp.Suppress('{') + funcType + pp.Suppress('#') + pp.Word(pp.nums) + pp.Suppress('}')

# Fill our ns type (which is made up of several cTypes separated by ::)
nsType << pp.delimitedList(fundamentalType | funcType | lambdaType | cType, '::') + pp.ZeroOrMore(pp.Literal('const') | pp.Word('*&'))

noRetFuncParser = pp.Group(nsType)
funcParser = pp.Group(nsType) + pp.Group(nsType)

# Open our output file for writing
with open(output_file, 'w') as file:

    # Attempting a disassembing version
    process = Popen(["objdump", "-t", "-d", input_file], stdout=PIPE);

    (output, err) = process.communicate()
    exit_code = process.wait()

    # If our nm command failed then exit with the error
    if(exit_code != 0):
        print err
        exit(exit_code)

    # Get all the lines
    lines = str(output).split('\n')

    # Regular expressions to get both symbols and calls to symbols
    symbol_table_regex = re.compile(r'^([0-9A-Fa-f]+)\s+([A-Za-z])\s+([A-Za-z])\s+(\S+)\s+([0-9A-Fa-f]+)\s+([0-9-A-Z-a-z_]+).*$')
    symbol_regex       = re.compile(r'^([0-9A-Fa-f]+)\s+<([0-9-A-Z-a-z_]+).*?>:$')
    call_regex         = re.compile(r'^\s+([0-9A-Fa-f]+):\s+(?:[0-9A-Fa-f]+\s+){5}call\s+([0-9A-Fa-f]+)\s+<.*>$')

    symbol_table = []
    symbols = []
    calls = []

    # Loop through our lines looking for useful symbols
    for line in lines:

        # See if it's a call line
        if call_regex.match(line):
            # We only need the indicies
            call = call_regex.match(line)
            calls.append( (int(call.group(1), 16), int(call.group(2), 16)) )

        # See if it's a symbol line
        elif symbol_regex.match(line):
            # Demangle the symbol
            symbol = symbol_regex.match(line)
            dm = demangler.demangle(symbol.group(2))
            if dm != None:
                symbols.append( (int(symbol.group(1), 16), dm) )

        # See if it's a symbol table line
        elif symbol_table_regex.match(line):
            # Demangle the symbol
            table = symbol_table_regex.match(line)
            dm = demangler.demangle(table.group(6))
            if dm != None:
                symbol_table.append( (int(table.group(1), 16), table.group(2), table.group(3), table.group(4), int(table.group(5), 16), dm) )

    # Build our call map
    callmap = dict()
    namemap = dict()

    # Attach our calls to the symbols that called them
    elem = 0
    for a in range(0, len(symbols) - 1):
        us = symbols[a]
        ne = symbols[a + 1]

        namemap[us[0]] = us[1];

        # If we don't have an entry in the call map add one
        if not us[0] in callmap:
            callmap[us[0]] = []

        # Insert all the calls into the map
        while calls[elem][0] < ne[0]:
            callmap[us[0]].append(calls[elem][1])
            elem += 1

    # Parse all of our emit symbols

    # Emit types (should cover most cases)
    r = re.compile(r'^NUClear::PowerPlant::Emit<(.+)>::emit\(.+\)$')
    for id in [i for i in namemap if r.match(namemap[i])]:
        file.write('{} {}\n'.format(id, namemap[id]))
        p = noRetFuncParser.parseString(namemap[id]).asList()
        file.write('{} {}\n'.format(id, str(p)))

    # Direct emits
    r = re.compile(r'^void NUClear::PowerPlant::ReactorMaster::directEmit<.+>\(.+\)$')
    for id in [i for i in namemap if r.match(namemap[i])]:
        file.write('{} {}\n'.format(id, namemap[id]))
        p = funcParser.parseString(namemap[id]).asList()
        file.write('{} {}\n'.format(id, str(p)))

    # Initialize emits
    r = re.compile(r'^void NUClear::PowerPlant::ReactorMaster::emitOnStart<.+>\(.+\)$')
    for id in [i for i in namemap if r.match(namemap[i])]:
        file.write('{} {}\n'.format(id, namemap[id]))
        p = funcParser.parseString(namemap[id]).asList()
        file.write('{} {}\n'.format(id, str(p)))

    # Powerplant emits
    r = re.compile(r'^void NUClear::PowerPlant::ReactorMaster::emit<.+>\(.+\)$')
    for id in [i for i in namemap if r.match(namemap[i])]:
        file.write('{} {}\n'.format(id, namemap[id]))
        p = funcParser.parseString(namemap[id]).asList()
        file.write('{} {}\n'.format(id, str(p)))

    # Reactor Emits
    r = re.compile(r'^void NUClear::Reactor::emit<.+>\(.+\)$')
    for id in [i for i in namemap if r.match(namemap[i])]:
        file.write('{} {}\n'.format(id, namemap[id]))
        p = funcParser.parseString(namemap[id]).asList()
        file.write('{} {}\n'.format(id, str(p)))


    # Parse all of our cache symbols
    r = re.compile(r'^void NUClear::PowerPlant::CacheMaster::cache<.+>\(.+\)$')
    for id in [i for i in namemap if r.match(namemap[i])]:
        file.write('{} {}\n'.format(id, namemap[id]))
        p = funcParser.parseString(namemap[id]).asList()
        file.write('{} {}\n'.format(id, str(p)))

    r = re.compile(r'^NUClear::metaprogramming::TypeMap<NUClear::PowerPlant::CacheMaster,.+>::get\(\)$')
    for id in [i for i in namemap if r.match(namemap[i])]:
        file.write('{} {}\n'.format(id, namemap[id]))
        p = noRetFuncParser.parseString(namemap[id]).asList()
        file.write('{} {}\n'.format(id, str(p)))

    r = re.compile(r'^NUClear::metaprogramming::TypeMap<NUClear::PowerPlant::CacheMaster,.+>::set\(.+\)$')
    for id in [i for i in namemap if r.match(namemap[i])]:
        file.write('{} {}\n'.format(id, namemap[id]))
        p = noRetFuncParser.parseString(namemap[id]).asList()
        file.write('{} {}\n'.format(id, str(p)))

    # THESE ONLY EXIST IN THE SYMBOL TABLE!!!
    # r = re.compile(r'^NUClear::metaprogramming::TypeMap<(NUClear::PowerPlant::CacheMaster,.+)>::data$')
    # for id in [i for i in namemap if r.match(namemap[i])]:
    #     file.write('{} {}\n'.format(id, namemap[id]))

    # r = re.compile(r'^NUClear::metaprogramming::TypeMap<(NUClear::PowerPlant::CacheMaster,.+)>::mutex$')
    # for id in [i for i in namemap if r.match(namemap[i])]:
    #     file.write('{} {}\n'.format(id, namemap[id]))

    # Parse all of our typelist symbols
    r = re.compile(r'^NUClear::metaprogramming::TypeList<NUClear::Reactor,.+>::get\(\)$')
    for id in [i for i in namemap if r.match(namemap[i])]:
        file.write('{} {}\n'.format(id, namemap[id]))
        p = noRetFuncParser.parseString(namemap[id]).asList()
        file.write('{} {}\n'.format(id, str(p)))

    # THESE ONLY EXIST IN THE SYMBOL TABLE!!!
    # r = re.compile(r'^\d+ u NUClear::metaprogramming::TypeList<(NUClear::Reactor,[^{]+)>::data$')
    # for id in [i for i in namemap if r.match(namemap[i])]:
    #     file.write('{} {}\n'.format(id, namemap[id]))

    # Parse our Exists symbols
    r = re.compile(r'^NUClear::Reactor::Exists<.+>::exists\(.+\)$')
    for id in [i for i in namemap if r.match(namemap[i])]:
        file.write('{} {}\n'.format(id, namemap[id]))
        p = noRetFuncParser.parseString(namemap[id]).asList()
        file.write('{} {}\n'.format(id, str(p)))

    # Parse our Get symbols
    r = re.compile(r'^NUClear::PowerPlant::CacheMaster::Get<.+>::get\(.+\)$')
    for id in [i for i in namemap if r.match(namemap[i])]:
        file.write('{} {}\n'.format(id, namemap[id]))
        p = noRetFuncParser.parseString(namemap[id]).asList()
        file.write('{} {}\n'.format(id, str(p)))

    # Parse our On symbols
    r = re.compile(r'^NUClear::Reactor::On<.+>::on\(.+\)$')
    for id in [i for i in namemap if r.match(namemap[i])]:
        file.write('{} {}\n'.format(id, namemap[id]))
        p = noRetFuncParser.parseString(namemap[id]).asList()
        file.write('{} {}\n'.format(id, str(p)))

    r = re.compile(r'^NUClear::threading::ReactionHandle NUClear::Reactor::on<.+>\(.+\)$')
    for id in [i for i in namemap if r.match(namemap[i])]:
        file.write('{} {}\n'.format(id, namemap[id]))
        p = funcParser.parseString(namemap[id]).asList()
        file.write('{} {}\n'.format(id, str(p)))

