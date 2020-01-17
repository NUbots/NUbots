#!/usr/bin/python

import bisect
import copy
import ctypes
import json
import logging
import re
import struct
import sys
from subprocess import PIPE, Popen

import pyparsing as pp

import pybfd.bfd
import pybfd.opcodes
from info.collapse_output_type import collapse_output_type
from info.parse_dsl_type import parse_dsl_type
from info.parse_output_type import parse_output_type
from info.symbol_parser import SymbolParser

if sys.argv[1]:
    module_name = sys.argv[1]
else:
    logging.error("You must specify a module name")
    sys.exit(1)

if sys.argv[2]:
    input_file = sys.argv[2]
else:
    logging.error("You must specify an input file")
    sys.exit(1)

if sys.argv[3]:
    output_file = sys.argv[3]
else:
    logging.error("You must specify an output file")
    sys.exit(1)

if sys.argv[4]:
    demangler = sys.argv[4]

    # Start up demangler
    demangler = ctypes.cdll.LoadLibrary(demangler)
    demangler.demangle.argtypes = [ctypes.c_char_p]
    demangler.demangle.restype = ctypes.c_char_p
else:
    logging.error("You must provide a demangler")
    sys.exit(1)

parser = SymbolParser()
symbols = {}
assembly_histogram = {}
symbol_relocation = {}
symbol_keys = []
reactions = {}
outputs = {}
isolated_outputs = []
binders = {}
memonic_regex = re.compile(r"^(\w+)")
call_regex = re.compile(r"^call\s+0x([0-9A-Fa-f]+)$")
push_regex = re.compile(r"^push\s+\$0x([0-9A-Fa-f]+)$")
# This regular expression specifically finds values loaded into register eax ebx ecx or edx
lea_regex = re.compile(r"lea\s+(-?0x[0-9a-fA-F]+)\(%ebx\),%e[abcd]x")

# Open the file using BFD and open a disassembler instance
bfd = pybfd.bfd.Bfd(input_file)
opcodes = pybfd.opcodes.Opcodes(bfd)

# Get our global offset table address for relative jumps
got_addr = bfd.sections[".got.plt"].vma

# Get list of symbols and their addresses
for symbol_address in bfd.symbols:
    symbol = bfd.symbols[symbol_address]

    d = demangler.demangle(symbol.name)
    d = symbol.name if d == None else str(d)
    symbols[symbol_address] = {"symbol": symbol.name, "name": d, "string_args": [], "calls": [], "called_by": []}

# Extract the strings from the read only data section so we can find our reaction names
s_data = bfd.sections[".rodata"].content
s_data_addr_range = (bfd.sections[".rodata"].vma, bfd.sections[".rodata"].vma + bfd.sections[".rodata"].size)

# Space to tilde matches all printable chars, 4, matches all of length 4 or more
# It also must be null terminated (the \x00)
strings_re = re.compile(r"[ -~]{4,}(?=\x00)")

# Get all of our strings in rodata mapped by their address
ro_strings = dict([(m.start() + s_data_addr_range[0], m.group()) for m in strings_re.finditer(s_data)])

# Resolve plt dynamic symbols so we can follow them
s_dynsym = bfd.sections[".dynsym"].content
s_dynstr = bfd.sections[".dynstr"].content
s_rel_plt = bfd.sections[".rel.plt"].content
s_plt = bfd.sections[".plt"]
for vma, size, disasm in opcodes.disassemble(s_plt.content, s_plt.vma):
    c = push_regex.match(disasm)
    if c:
        # Work out pushed info
        plt_index = int(c.group(1), 16)
        inst_index = vma - size - 1

        # Read relocation information
        rel_plt = struct.unpack("II", s_rel_plt[plt_index : plt_index + 8])
        rel_plt = (rel_plt[0], rel_plt[1] >> 8, rel_plt[1] & 0xFF)

        # Find corresponding dynamic symbol
        sym = s_dynsym[rel_plt[1] * 16 : rel_plt[1] * 16 + 16]
        sym = struct.unpack("IIIBBH", sym)

        # Store this relocation if it is meaningful
        if sym[1] != 0:
            symbol_relocation[inst_index] = sym[1]
        # Otherwise add a new symbol for this address with dynstr string
        else:
            # Get the symbol name from dynstr
            end = s_dynstr[sym[0] :].index("\x00")
            symbol_name = s_dynstr[sym[0] : sym[0] + end]

            d = demangler.demangle(symbol_name)
            d = symbol_name if d == None else str(d)
            symbols[inst_index] = {"symbol": symbol_name, "name": d, "string_args": [], "calls": [], "called_by": []}

# Make a list of keys so we can bisect the list (lower bound)
symbol_keys = sorted(list(symbols.keys()))

# Process text (program) section to find call statements
s_text = bfd.sections[".text"]
string_args = []
for vma, size, disasm in opcodes.disassemble(s_text.content, s_text.vma):

    # Get the assembly memonic that is used and increment it
    try:
        memonic = memonic_regex.match(disasm).group(1)
        assembly_histogram[memonic] = assembly_histogram.get(memonic, 0) + 1
    except:
        pass

    # Find a string arguments for the next call if they exist
    lea = lea_regex.match(disasm)
    if lea:
        arg_addr = int(lea.group(1), 16) + got_addr

        # If this address is for a string store it for the next call
        if arg_addr in ro_strings:
            string_args.append(ro_strings[arg_addr])

    call = call_regex.match(disasm)
    if call:  # This will ignore calls that are not to a specific function (e.g. call from register)
        call_addr = int(call.group(1), 16)

        # If call_addr is a relocated symbol, resolve to the real one
        if call_addr in symbol_relocation:
            call_addr = symbol_relocation[call_addr]

        # Find the symbol that is probably doing the calling (the previously declared one)
        # and put this call in it
        symbol = symbols[symbol_keys[bisect.bisect(symbol_keys, vma) - 1]]
        symbol["calls"].append(call_addr)

        # If we have some string arguments for this call, add them to the callee
        # Provided this isn't the string constructor... that eats lots of const char*s
        if string_args:
            symbols[call_addr]["string_args"].append(string_args)

            # Clear our args, we used them
            string_args = []

# Reverse link symbols with the symbols that call them
for caller_addr in symbols:

    # Dedupe our string arguments to this function
    symbols[caller_addr]["string_args"] = [list(x) for x in set(tuple(x) for x in symbols[caller_addr]["string_args"])]

    # Reverse link our symbols
    for callee_addr in symbols[caller_addr]["calls"]:
        if callee_addr in symbols:
            symbols[callee_addr]["called_by"].append(caller_addr)

# Define regular expressions for "interesting" emit symbols
emit_re = [
    # Powerplant emits
    re.compile(r"^void NUClear::PowerPlant::emit<.+>\(.+\)$"),
    # Reactor Emits
    re.compile(r"^void NUClear::Reactor::emit<.+>\(.+\)$"),
]

# Define regular expressions for "interesting" on symbols
on_re = [
    # Our then calls
    re.compile(r"^decltype\s+\(.+?\.then.+?\).+?NUClear::Reactor::Binder<.+?>::then<.+?>.+?$")
]

# Find interesting symbols and add information to them
for symbol_address in symbols:
    symbol = symbols[symbol_address]

    if emit_re[0].match(symbol["name"]):
        parsed = parser.parse_symbol(symbol["name"])

        # Remove the template arguments that represent the actual arguments
        # TODO note there is a bug here that if an argument is the same as the type
        # getting emitted this will remove it and break it
        info = [x for x in parsed[1][3] if x not in parsed[1][-1][1:] and x != []]

        # If this is a NUClear CommandLineArgs or ReactionStatisitcs ignore it
        if len(info[-1]) == 3 and info[-1][:2] in [
            ["NUClear", "message", "CommandLineArgs"],
            ["NUClear", "message", "ReactionStatistics"],
        ]:
            continue

        outputs[symbol_address] = {
            "type": info[-1],  # The type in the unique_ptr
            "scopes": [x[-1] for x in info[:-1]],
            "children": [],
        }
        # If empty then local scope
        outputs[symbol_address]["scopes"] = (
            outputs[symbol_address]["scopes"] if outputs[symbol_address]["scopes"] != [] else ["Local"]
        )

    elif emit_re[1].match(symbol["name"]):
        parsed = parser.parse_symbol(symbol["name"])

        # Remove the template arguments that represent the actual arguments
        # TODO note there is a bug here that if an argument is the same as the type
        # getting emitted this will remove it and break it
        info = [x for x in parsed[1][3] if x not in parsed[1][-1][1:] and x != []]

        # If this is a NUClear CommandLineArgs or ReactionStatisitcs ignore it
        if len(info[-1]) == 3 and info[-1][:2] in [
            ["NUClear", "message", "CommandLineArgs"],
            ["NUClear", "message", "ReactionStatistics"],
        ]:
            continue

        outputs[symbol_address] = {
            "type": info[-1],  # The type in the unique_ptr
            "scopes": [x[-1] for x in info[:-1]],
            "children": [],
        }
        # If empty then local scope
        outputs[symbol_address]["scopes"] = (
            outputs[symbol_address]["scopes"] if outputs[symbol_address]["scopes"] != [] else ["Local"]
        )

    elif on_re[0].match(symbol["name"]):

        # Run a parse with our locator tags on to get the function we are using
        location = parser.parse_symbol(symbol["name"], locateTags=True)

        # check and double check we are on the string version of on
        if len(location[1][-3]) == 2 and len(location[1][-2]) == 2:

            # Extract our function
            func = symbol["name"][location[1][-3][1][0] : location[1][-3][1][-1]]

            # Extract our binder instance
            # This is the on call related to this reaction which has binder arguments
            binder = symbol["name"][location[0][2][0][1][1][1][2][0][0] : location[0][2][0][1][1][1][2][0][-1]]
            binder = [x for x in symbols if symbols[x]["name"].startswith(binder)]
            binder_args = []

            # If we have one or more, pick the shortest
            if binder:
                binder = min(binder, key=lambda addr: len(symbols[addr]["name"]))
                binder_args = symbols[binder]["string_args"]

            # Find our candidates (start with ::operator() version)
            candidates = [x for x in symbols if symbols[x]["name"].startswith(func + "::operator")]

            # Fallback to a direct match
            if not candidates:
                candidates = [x for x in symbols if symbols[x]["name"].startswith(func)]

            # Fallback to trying to fix the symbol
            if not candidates:
                # Sometimes something stupid happens with the function and we get 'then' and 'Parse' in our symbol...
                # We need to fix this otherwise it can't find the real underlying symbol

                # Try one that replaces with then
                then_rep = re.sub(r"([A-Za-z0-9]+)::then\(", "\\1::\\1(", func, 1)
                parse_rep = re.sub(r"([A-Za-z0-9]+)::Parse\(", "\\1::\\1(", func, 1)

                if then_rep != func:
                    # Find our candidates (start with ::operator() version)
                    candidates = [x for x in symbols if symbols[x]["name"].startswith(then_rep + "::operator")]

                    # Fallback to a direct match
                    if not candidates:
                        candidates = [x for x in symbols if symbols[x]["name"].startswith(then_rep)]

                elif parse_rep != func:

                    # Find our candidates (start with ::operator() version)
                    candidates = [x for x in symbols if symbols[x]["name"].startswith(parse_rep + "::operator")]

                    # Fallback to a direct match
                    if not candidates:
                        candidates = [x for x in symbols if symbols[x]["name"].startswith(parse_rep)]

            # If we still can't find it give up
            if not candidates:
                print("Error could not find function for on statement")
                print(func, "\n\n")

            else:
                # There could be more than one candidiate here
                # this can happen if there is a lambda in a lambda (we want the outer one)
                # So pick the shortest string in the list
                candidate = min(candidates, key=lambda addr: len(symbols[addr]["name"]))

                # Reparse without our locator tags
                parsed = parser.parse_symbol(symbol["name"])

                # Check if this is probably a reaction owned by NUClear
                # We don't want to include those because they will get
                # included in every nuclear_info file and make the graphs
                # huge and complex. Fortunatly all of them hide in the
                # NUClear extension namespace
                is_nuclear = func.startswith("NUClear::extension")

                if candidate not in reactions:
                    reactions[candidate] = []

                reactions[candidate].append(
                    {
                        "dsl": parsed[1][3][0][3],
                        "outputs": [],
                        "binder_args": binder_args,
                        "binding_outputs": [],
                        "user_name": symbol["string_args"],
                        "is_nuclear": is_nuclear,
                    }
                )

                # Store that this binder binds this candidate
                binders[symbol_address] = reactions[candidate][-1]

# Now collapse the call tree to work out which reactions send which outputs
for output_address in outputs:

    output = outputs[output_address]

    # Do a bfs search
    searched = set()
    # todo search from symbols[output]['called_by']
    search = set(symbols[output_address]["called_by"])

    while search:
        # Get our next search element
        top = search.pop()
        searched.add(top)

        # If this is one of our reactions we don't need to search any
        # further on this path, we found one add it to our reactions
        # outputs
        if top in reactions:
            for reaction in reactions[top]:
                reaction["outputs"].append(copy.deepcopy(output))

        # If this has been called by another emit then add this to its list
        elif top in outputs:
            outputs[top]["children"].append(copy.deepcopy(output))

        # If this emit is done from a binding call (then call) we can trace it
        # to the reaction that called it. It is probably a setup reaction
        elif top in binders:
            binders[top]["binding_outputs"].append(copy.deepcopy(output))

        # Otherwise add this functions callers to the search list
        elif top in symbols:
            search |= set(symbols[top]["called_by"]) - searched

            # If we reached an end this way, this reaction must be called from
            # somewhere that does not match up to a reaction, another emit, or a binder
            if len(search) == 0:
                isolated_outputs.append(output)


# Build our output information structure
jsonOutput = {"name": module_name, "assembly": assembly_histogram, "reactions": [], "output_data": []}

for reaction_address in reactions:
    for reaction in reactions[reaction_address]:

        # We don't want to include reactions from NUClear
        if not reaction["is_nuclear"]:

            r = {
                # The name of the reaction, if the user provided use that
                "name": reaction["user_name"][0][0]
                if reaction["user_name"]
                else None,  # TODO find a better name here if you can
                # The DSL that this is generated from
                "dsl": reaction["dsl"],
                # The memory address for the reaction function
                "address": reaction_address,
                # The data that this reaction gets
                # Consists of a list of event descriptions, and modifiers applied to the input
                "input_data": [],  # { 'scope': 'S', 'value': 'T', 'modifiers': {'last':n, 'optional':True} }
                # A list of the output data from this reaction
                # Consists of a list of event descriptions, and scopes
                "output_data": [],  # { 'scope': 'S', 'value': 'T' }
                # Modifiers that influence how the reaction as a whole runs
                # Consists of a set of properties and values
                "modifiers": {},  # 'single': True, 'sync': 'T'
            }

            # Go through our DSL word elements
            for word in reaction["dsl"]:

                # Get our reaction delta from this word
                delta = parse_dsl_type(word, reaction["binder_args"])

                # Fuse in the data to the reaction
                for key in delta:
                    if key == "execution":
                        r[key].extend(delta[key])
                    elif key == "input_data":
                        r[key].extend(delta[key])
                    elif key == "modifiers":
                        r[key].update(delta[key])

            # Go through all our outputs
            for output in reaction["outputs"]:

                # Collapse our identical outputs
                output = collapse_output_type(output)

                # Parse our output
                output = parse_output_type(output)

                # Add our parsed list
                r["output_data"].append(output)

            # Go through all our binding outputs
            for output in reaction["binding_outputs"]:

                # Collapse our identical outputs
                output = collapse_output_type(output)

                # Parse our output
                output = parse_output_type(output)

                for o in output:
                    o["modifiers"]["binding"] = True

                # Add our parsed list
                r["output_data"].append(output)

            # Add this reaction to our list
            jsonOutput["reactions"].append(r)

        # Process all our isolated outputs
        for output in isolated_outputs:

            # Collapse our identical outputs
            output = collapse_output_type(output)

            # Parse our output
            output = parse_output_type(output)

            # Add our parsed list
            jsonOutput["output_data"].append(output)

vals = []
for o in jsonOutput["output_data"]:
    if o not in vals:
        vals.append(o)
jsonOutput["output_data"] = vals

for reaction in jsonOutput["reactions"]:
    vals = []
    for o in reaction["output_data"]:
        if o not in vals:
            vals.append(o)
    reaction["output_data"] = vals

# Open output file for writing
with open(output_file, "w") as file:
    json.dump(jsonOutput, file, sort_keys=True, indent=4, separators=(",", ": "))
