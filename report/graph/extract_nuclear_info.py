#!/usr/bin/python

import json
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

# Enable packrat for fastness!
pp.ParserElement.enablePackrat()

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

locate = False
locator = pp.Empty().setParseAction(lambda s,l,t: l if locate else None)

# An enum type e.g. `(a::b::c)1`
enumType = pp.Suppress('(') + nsType + pp.Suppress(')') + pp.Word(pp.nums)

# Match a template (list of types, enums and empties)
templateType = pp.Group(pp.Suppress('<') + pp.Optional(pp.delimitedList(pp.Group(nsType | enumType | pp.Empty()))) + pp.Suppress('>'))

# Match a cType (text then maybe a template)
cType = pp.Word(pp.alphanums + '_') + pp.Optional(templateType)

# A function type (type followed by function arguments)
funcType = cType + pp.Suppress('(') + pp.Optional(pp.Group(pp.delimitedList(pp.Group(nsType)))) + pp.Suppress(')')

# A lambda type (looks like) {lambda(a,b,c)#1}
lambdaType = pp.Suppress('{') + funcType + pp.Suppress('#') + pp.Word(pp.nums) + pp.Suppress('}')

# Fill our ns type (which is made up of several types separated by ::)
nsType << locator + pp.delimitedList(fundamentalType | funcType | lambdaType | cType, '::') + locator + pp.ZeroOrMore(pp.Literal('const') | pp.Word('*&'))

noRetFuncParser = nsType
funcParser = pp.Group(nsType) + pp.Group(nsType)

# Open our output file for writing
with open(output_file, 'w') as file:

    # Attempting a disassembing version
    process = Popen(["objdump", "-d", input_file], stdout=PIPE);

    (output, err) = process.communicate()
    exit_code = process.wait()

    # If our nm command failed then exit with the error
    if(exit_code != 0):
        print err
        exit(exit_code)

    # Get all the lines
    lines = str(output).split('\n')

    # Regular expressions to get both symbols and calls to symbols
    symbol_regex       = re.compile(r'^([0-9A-Fa-f]+)\s+<([0-9-A-Z-a-z_]+)>:$')
    call_regex         = re.compile(r'^\s+([0-9A-Fa-f]+):\s+(?:[0-9A-Fa-f]+\s+){5}call\s+([0-9A-Fa-f]+)\s+<(.*)>$')

    raw_symbols = []
    raw_calls = []

    # Loop through our lines looking for useful symbols
    for line in lines:

        # See if it's a call line
        if call_regex.match(line):
            # We only need the indicies
            call = call_regex.match(line)
            raw_calls.append( (int(call.group(1), 16), int(call.group(2), 16), call.group(3)) )

        # See if it's a symbol line
        elif symbol_regex.match(line):
            # Demangle the symbol
            symbol = symbol_regex.match(line)
            raw_symbols.append( (int(symbol.group(1), 16), symbol.group(2)) )

    symbols = []
    calls = []

    # Process our calls
    for call in raw_calls:
        # If our call is a plt call (dynamic library)
        if call[2].endswith('@plt'):

            # Find the actual call and append it
            for symb in raw_symbols:
                if(symb[1] == call[2][:-4]):
                    calls.append( (call[0], symb[0]) )
                    break
        else:
            # Our call is ok as is
            calls.append( (call[0], call[1]) )

    # Process our symbols (demangle the token)
    for symbol in raw_symbols:
        dm = demangler.demangle(symbol[1])
        if dm != None:
            symbols.append( (symbol[0], str(dm)) )

    # Our callmap, holds a map of functions to functions they call
    callmap = {}
    # Our inverse callmap holds a map of functions to functions they are called by
    calledbymap = {}
    # Our map that maps function calls to their name
    namemap = {}

    # Attach our calls to the symbols that called them (build our callmap)
    elem = 0
    # Loop through every element pair
    for us, ne in zip(symbols, symbols[1:]):

        namemap[us[0]] = us[1];

        # If we don't have an entry in the call map add one
        if not us[0] in callmap:
            callmap[us[0]] = []

        # Insert all the calls into the map
        while calls[elem][0] < ne[0]:
            callmap[us[0]].append(calls[elem][1])
            elem += 1

    # Build our inverse lookup list (find usages)
    for caller in callmap:
        for callee in callmap[caller]:
            if callee not in calledbymap:
                calledbymap[callee] = []

            calledbymap[callee].append(caller)

    emit_re = [
        # Emit types (should cover most cases)
        re.compile(r'^NUClear::PowerPlant::Emit<(.+)>::emit\(.+\)$'),
        # Direct emits
        re.compile(r'^void NUClear::PowerPlant::ReactorMaster::directEmit<.+>\(.+\)$'),
        # Initialize emits
        re.compile(r'^void NUClear::PowerPlant::ReactorMaster::emitOnStart<.+>\(.+\)$'),
        # Powerplant emits
        re.compile(r'^void NUClear::PowerPlant::ReactorMaster::emit<.+>\(.+\)$'),
        # Reactor Emits
        re.compile(r'^void NUClear::Reactor::emit<.+>\(.+\)$')
    ]

    cache_re = [
        # Happens when a cache function is called (is a set)
        re.compile(r'^void NUClear::PowerPlant::CacheMaster::cache<.+>\(.+\)$'),
        # Happens when something is retrived from the cache (a get)
        re.compile(r'^NUClear::metaprogramming::TypeMap<NUClear::PowerPlant::CacheMaster,.+>::get\(\)$'),
        # Happens when something is put into the cache (a set)
        re.compile(r'^NUClear::metaprogramming::TypeMap<NUClear::PowerPlant::CacheMaster,.+>::set\(.+\)$'),
        # Happens when the cache exists at all (only exists in symbol table)
        re.compile(r'^NUClear::metaprogramming::TypeMap<(NUClear::PowerPlant::CacheMaster,.+)>::data$'),
        # Happens when the cache exists at all (only exists in symbol table)
        re.compile(r'^NUClear::metaprogramming::TypeMap<(NUClear::PowerPlant::CacheMaster,.+)>::mutex$')
    ]

    typelist_re = [
        # Happens when a type list is gotten from the TypeList (a Trigger being set or triggered)
        re.compile(r'^NUClear::metaprogramming::TypeList<NUClear::Reactor,.+>::get\(\)$'),
        # Happens when a type list exists at all (holds triggers)
        re.compile(r'^\d+ u NUClear::metaprogramming::TypeList<(NUClear::Reactor,[^{]+)>::data$')
    ]

    exists_re = [
        # Is called when a type exists (in a trigger or with)
        re.compile(r'^NUClear::Reactor::Exists<.+>::exists\(.+\)$')
    ]

    get_re = [
        # Is called when a reaction wants to get a type
        re.compile(r'^NUClear::PowerPlant::CacheMaster::Get<.+>::get\(.+\)$')
    ]

    on_re = [
        # Is the result of the On<> metaprogram (contains lots of information)
        re.compile(r'^NUClear::Reactor::On<.+>::on\(.+\)$'),
        # Is the call of the On<> metaprogramm (contains the dsl used)
        re.compile(r'^NUClear::threading::ReactionHandle NUClear::Reactor::on<.+>\(.+\)$')
    ]

    outputs = []
    isolated_outputs = []
    inputs = []

    # Look through all our symbols and parse them
    for id in namemap:
        parsed = None

        # Emit types
        if emit_re[0].match(namemap[id]):

            parsed = noRetFuncParser.parseString(namemap[id]).asList()
            outputs.append({
                'address': id,
                'type': parsed[3][-1],
                'scopes': parsed[3][:-1]
            })

        # Direct emits
        elif emit_re[1].match(namemap[id]):
            parsed = funcParser.parseString(namemap[id]).asList()
            outputs.append({
                'address': id,
                'type': parsed[1][4],
                'scopes': [['NUClear', 'dsl', 'Scope', 'DIRECT']]
            })

        # Initialize emits
        elif emit_re[2].match(namemap[id]):
            parsed = funcParser.parseString(namemap[id]).asList()
            outputs.append({
                'address': id,
                'type': parsed[1][4],
                'scopes': [['NUClear', 'dsl', 'Scope', 'INITIALIZE']]
            })

        # Local emits
        elif emit_re[3].match(namemap[id]):
            parsed = funcParser.parseString(namemap[id]).asList()
            outputs.append({
                'address': id,
                'type': parsed[1][4],
                'scopes': [['NUClear', 'dsl', 'Scope', 'LOCAL']]
            })

        # Reactor Emits
        elif emit_re[4].match(namemap[id]):
            parsed = funcParser.parseString(namemap[id]).asList()
            outputs.append({
                'address': id,
                'type': parsed[1][3][-1],
                'scopes': parsed[1][3][:-1] if len(parsed[1][3][0:-1][0]) > 0 else [['NUClear', 'dsl', 'Scope', 'LOCAL']]
            })

        # Happens when a cache function is called (is a set) this is an output
        elif cache_re[0].match(namemap[id]):
            parsed = funcParser.parseString(namemap[id]).asList()
            outputs.append({
                'address': id,
                'type': parsed[1][4],
                'scopes': []
            })

        elif cache_re[1].match(namemap[id]):
            # Happens when something is retrived from the cache (a get)
            parsed = noRetFuncParser.parseString(namemap[id]).asList()
            outputs.append({
                'address': id,
                'type': parsed[3][2],
                'scopes': []
            })

        elif cache_re[2].match(namemap[id]):
            # Happens when something is put into the cache (a set)
            parsed = noRetFuncParser.parseString(namemap[id]).asList()

        elif typelist_re[0].match(namemap[id]):
            # Happens when a type list is gotten from the TypeList (a Trigger being set or triggered)
            parsed = noRetFuncParser.parseString(namemap[id]).asList()

        elif exists_re[0].match(namemap[id]):
            # Is called when a type exists (in a trigger or with)
            parsed = noRetFuncParser.parseString(namemap[id]).asList()

        elif get_re[0].match(namemap[id]):
            # Is called when a reaction wants to get a type
            parsed = noRetFuncParser.parseString(namemap[id]).asList()

        elif on_re[0].match(namemap[id]):
            # Is the result of the On<> metaprogram (contains lots of information)
            parsed = noRetFuncParser.parseString(namemap[id]).asList()

        # Is the call of the On<> metaprogramm (contains the dsl used)
        elif on_re[1].match(namemap[id]):

            # Enable our locator tags so we can access the original function type
            locate = True
            parsed = funcParser.parseString(namemap[id]).asList()
            # Disable our locator tags
            locate = False

            # Extract our function name
            start = parsed[1][-2][-1][0]
            end = parsed[1][-2][-1][-1]
            func = namemap[id][start:end]

            # Work out which function is probably ours
            candidates = [
                [x[0] for x in symbols if x[1].startswith(func + '::operator()')],
                [x[0] for x in symbols if x[1].startswith(func)]
            ]

            if len(candidates[0]) == 1:
                candidates = candidates[0][0]
            elif len(candidates[1]) == 1:
                candidates = candidates[1][0]
            else:
                candidates = None

            # Reparse without our locator tags
            parsed = funcParser.parseString(namemap[id]).asList()
            inputs.append({
                'address': candidates,
                'types': parsed[1][3][:-1],
                'outputs': []
            })

    # For each of our outputs, trace it back to an input
    for output in outputs:
        searched = set()
        search = [output['address']]
        joined = False

        # While something is in our search list
        while search:
            # Get our search vector
            top = search.pop(0)

            # Find our input if we can
            input = [i for i in inputs if i['address'] == top]

            if input:
                joined = True
                for i in input:
                    i['outputs'].append(output)
            elif top in calledbymap:
                for c in calledbymap[top]:
                    if c not in searched:
                        search.append(c)
                        searched.add(c)

        if not joined:
            isolated_outputs.append(output)



    # Now make our json output
    jsonOutput = {
        'module_name': '',
        'reactions': [],
        'outputs': []
    }

    for input in inputs:
        jsonOutput['reactions'].append({
            'name': '',
            'inputs': [],
            'outputs': []
        })

        elem = jsonOutput['reactions'][-1]

        for type in input['types']:
            if type[2] == 'Trigger':
                elem['inputs'].append({
                    'type': type[3][0],
                    'scope': 'TRIGGER'
                })

            elif type[2] == 'With':
                for t in type[3]:
                    elem['inputs'].append({
                        'type': t,
                        'scope': 'WITH'
                    })
            # Option

        for output in input['outputs']:
            elem['outputs'].append({
                'type': output['type'],
                'scopes': output['scopes']
            })

    for output in isolated_outputs:
        jsonOutput['outputs'].append({
            'type': output['type'],
            'scopes': output['scopes']
        })

    json.dump(jsonOutput, file, sort_keys=True, indent=4, separators=(',', ': '))


