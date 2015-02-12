#!/usr/bin/python

import sys
import re
from subprocess import Popen, PIPE

def demangle(demangler, symbol):

    # Demangle our symbol
    process = Popen([demangler, symbol], stdout=PIPE, stdin=PIPE);

    # Read our result
    (output, err) = process.communicate();
    exit_code = process.wait();

    # If our nm command failed then exit with the error
    if(exit_code != 0):
        print err;
        exit(exit_code);

    return str(output);


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

if sys.argv[3]:
    demangler = sys.argv[3];

    # Start up our demangler
    demangler = Popen([demangler], stdout=PIPE, stdin=PIPE);
else:
    print 'You must provide a demangler\n';
    sys.exit(1);



# Open our output file for writing
with open(output_file, 'w') as file:

    # Attempting a disassembing version
    process = Popen(["objdump", "-t", "-d", input_file], stdout=PIPE);

    (output, err) = process.communicate();
    exit_code = process.wait();

    # If our nm command failed then exit with the error
    if(exit_code != 0):
        print err;
        exit(exit_code);

    # Get all the lines
    lines = str(output).split('\n');

    # Regular expressions to get both symbols and calls to symbols
    symbol_regex = re.compile(r'^([0-9A-Fa-f]+)\s+<([0-9-A-Z-a-z_]+).*?>:$');
    call_regex = re.compile(r'^\s+([0-9A-Fa-f]+):\s+(?:[0-9A-Fa-f]+\s+){5}call\s+([0-9A-Fa-f]+)\s+<([0-9-A-Z-a-z_]+).*?>$');

    symbols = [];
    calls = [];

    # Loop through our lines looking for useful symbols
    for line in lines:
        symbol = symbol_regex.match(line);
        call = call_regex.match(line);

        if symbol != None:
            demangler.stdin.write(symbol.group(2) + '\n');
            dm = demangler.stdout.readline();
            symbols.append( (symbol.group(1), dm) );

        if call != None:
            demangler.stdin.write(call.group(3) + '\n');
            dm = demangler.stdout.readline().strip();
            calls.append( (call.group(1), call.group(2), dm) );

    # Find the symbols we are looking for
    emit_regex = re.compile(r'^NUClear::PowerPlant::Emit<(.+)>::emit\(.+\)$');

    emits = [emit_regex.sub(r'\1', x[1]) for x in symbols if emit_regex.match(x[1])];

    file.write('\n'.join(emits));

# 0008fce0 NUClear::Reactor::Exists<NUClear::dsl::Raw<messages::vision::ClassifiedImage<messages::vision::ObjectClass> > >::exists(NUClear::Reactor&)
# 00091930 void NUClear::Reactor::emit<, std::vector<messages::vision::Ball, std::allocator<messages::vision::Ball> > >(std::unique_ptr<std::vector<messages::vision::Ball, std::allocator<messages::vision::Ball> >, std::default_delete<std::vector<messages::vision::Ball, std::allocator<messages::vision::Ball> > > >&&)
# 00091df0 void NUClear::PowerPlant::emit<, messages::support::Configuration<modules::vision::BallDetector> >(std::unique_ptr<messages::support::Configuration<modules::vision::BallDetector>, std::default_delete<messages::support::Configuration<modules::vision::BallDetector> > >&&)
# 000926c0 std::_Function_base::_Base_manager<void NUClear::PowerPlant::ReactorMaster::emitOnStart<messages::support::ConfigurationConfiguration>(std::shared_ptr<messages::support::ConfigurationConfiguration>)::{lambda()#1}>::_M_init_functor(std::_Any_data&, {lambda()#1}&&, std::integral_constant<bool, false>)
# 00092e00 void NUClear::PowerPlant::ReactorMaster::directEmit<NUClear::ReactionStatistics>(std::shared_ptr<NUClear::ReactionStatistics>)
# 00092ee0 bool std::_Function_base::_Base_manager<void NUClear::PowerPlant::ReactorMaster::emitOnStart<messages::support::ConfigurationConfiguration>(std::shared_ptr<messages::support::ConfigurationConfiguration>)::{lambda()#1}>::_M_not_empty_function<{lambda()#1}>({lambda()#1} const&)
# 000b9492 void NUClear::PowerPlant::ReactorMaster::emit<messages::support::nubugger::proto::DataPoint>(std::shared_ptr<messages::support::nubugger::proto::DataPoint>)

# 000b9f0d NUClear::PowerPlant::CacheMaster::Get<NUClear::dsl::Raw<messages::vision::ClassifiedImage<messages::vision::ObjectClass> > >::get(NUClear::PowerPlant&)

# 000bd707 NUClear::metaprogramming::TypeMap<NUClear::PowerPlant::CacheMaster, messages::vision::ClassifiedImage<messages::vision::ObjectClass>, messages::vision::ClassifiedImage<messages::vision::ObjectClass> >::get()
# 000bbf92 void NUClear::PowerPlant::CacheMaster::cache<NUClear::LogMessage>(std::shared_ptr<NUClear::LogMessage>)
# 000c4f6d NUClear::metaprogramming::TypeList<NUClear::Reactor, messages::support::ConfigurationConfiguration, std::unique_ptr<NUClear::threading::Reaction, std::default_delete<NUClear::threading::Reaction> > >::get()

# 9caf9 95ea0 NUClear::Reactor::Exists<NUClear::dsl::Optional<messages::support::FieldDescription> >::exists(NUClear::Reactor&)
# 9cb04 946d0 NUClear::Reactor::Exists<messages::input::CameraParameters>::exists(NUClear::Reactor&)
# 9cb0f 8fce0 NUClear::Reactor::Exists<NUClear::dsl::Raw<messages::vision::ClassifiedImage<messages::vision::ObjectClass> > >::exists(NUClear::Reactor&)
# 0008ed70 NUClear::Reactor::Exists<messages::support::Configuration<modules::vision::BallDetector> >::exists(NUClear::Reactor&)::{lambda(NUClear::Reactor*, std::string const&, YAML::Node const&)#2}*& std::_Any_data::_M_access<NUClear::Reactor::Exists<messages::support::Configuration<modules::vision::BallDetector> >::exists(NUClear::Reactor&)::{lambda(NUClear::Reactor*, std::string const&, YAML::Node const&)#2}*>()
# 0008fe00 decltype (NUClear::PowerPlant::CacheMaster::Get<NUClear::dsl::Optional<messages::support::FieldDescription> >::get((*this).parent)) NUClear::PowerPlant::CacheMaster::get<NUClear::dsl::Optional<messages::support::FieldDescription> >()




    # # Regexes to extract the On<> related information
    # on_regex = [
    #     # Seems to include NUClear's On<>s as well as ours
    #     re.compile(r'^\d+ V typeinfo name for std::tuple<(.+)>$'),
    #     # Seems to be only On<>s in this module
    #     re.compile(r'^\d+ V typeinfo for std::tuple<(.+)>$')
    # ];

    # # Regexes to extract the emit information
    # emit_regex = [
    #     # Emits that have generated a type
    #     re.compile(r'^\d+ W NUClear::PowerPlant::Emit<([^{]+)>::emit\(.+?\)$'),
    #     # Direct emits only
    #     re.compile(r'^\d+ W void NUClear::PowerPlant::ReactorMaster::directEmit<([^{]+)>\(.+?\)$'),
    #     # Initialize emits only
    #     re.compile(r'^\d+ W void NUClear::PowerPlant::ReactorMaster::emitOnStart<([^{]+)>\(.+?\)$'),
    #     # Regular emits only
    #     re.compile(r'^\d+ W void NUClear::PowerPlant::ReactorMaster::emit<(.+)>\(.+?\)$'),
    #     # Emits that come through powerplant
    #     re.compile(r'^\d+ W void NUClear::PowerPlant::emit<([^{]+)>\(.+?\)$'),
    #     # Emits that come through a reactor
    #     re.compile(r'^\d+ W void NUClear::Reactor::emit<([^{]+)>\(.+?\)$')
    # ];

    # # Regexes to exctract the cache information
    # cache_regex = [
    #     # Caches that have a cache function called from the CacheMaster
    #     re.compile(r'^\d+ W void NUClear::PowerPlant::CacheMaster::cache<([^{]+)>\(.+?\)$'),
    #     # Caches that have a get function called (things that are gotten)
    #     re.compile(r'^\d+ W NUClear::metaprogramming::TypeMap<(NUClear::PowerPlant::CacheMaster,[^{]+)>::get\(\)$'),
    #     # Caches that have a set function called (things that are emitted)
    #     re.compile(r'^\d+ W NUClear::metaprogramming::TypeMap<(NUClear::PowerPlant::CacheMaster,[^{]+)>::set\(.+?\)$'),
    #     # Caches that have data but are unused
    #     re.compile(r'^\d+ u NUClear::metaprogramming::TypeMap<(NUClear::PowerPlant::CacheMaster,[^{]+)>::data$'),
    #     # Caches that have a mutex but are unused
    #     re.compile(r'^\d+ u NUClear::metaprogramming::TypeMap<(NUClear::PowerPlant::CacheMaster,[^{]+)>::mutex$')
    # ];

    # # Regexes to extract the TypeList <Trigger> information
    # trigger_list_regex = [
    #     # Triggers that are executed by this module (triggers that used and emitted by this module?)
    #     re.compile(r'^\d+ W NUClear::metaprogramming::TypeList<(NUClear::Reactor,[^{]+)>::get\(\)$'),
    #     # Triggers that are executed external to this module (triggers that are not handled internally)
    #     re.compile(r'^\d+ u NUClear::metaprogramming::TypeList<(NUClear::Reactor,[^{]+)>::data$')
    # ];

    # # Regexes to extract the exists information
    # exists_regex = [
    #     # Types that fire an exists handler (triggers and withs)
    #     re.compile(r'^\d+ W NUClear::Reactor::Exists<([^{]+)>::exists\(.+?\)$')
    # ];

    # # Regexes to extract the Get<> information
    # get_regex = [
    #     # Types that use a Get<> handler (types in trigger and with)
    #     re.compile(r'^\d+ W NUClear::PowerPlant::CacheMaster::Get<([^{]+)>::get\(.+?\)$')
    # ];

    # # Declare our lists
    # raw_on           = [[r.sub(r'\1', x) for x in lines if r.match(x)] for r in on_regex];
    # raw_emit         = [[r.sub(r'\1', x) for x in lines if r.match(x)] for r in emit_regex];
    # raw_cache        = [[r.sub(r'\1', x) for x in lines if r.match(x)] for r in cache_regex];
    # raw_trigger_list = [[r.sub(r'\1', x) for x in lines if r.match(x)] for r in trigger_list_regex];
    # raw_exists       = [[r.sub(r'\1', x) for x in lines if r.match(x)] for r in exists_regex];
    # raw_get          = [[r.sub(r'\1', x) for x in lines if r.match(x)] for r in get_regex];

    # # TODO a lot of this information is redundant and should be merged together
    # # TODO however existing in different regex groups means different things
    # # TODO do some processing to make a json file of the interaces on this file
    # for i, val in enumerate(raw_on):
    #     file.write("\n\nOn<> Statements " + str(i) + " \n\t");
    #     file.write('\n\t'.join(val));

    # for i, val in enumerate(raw_emit):
    #     file.write("\n\nEmit<> Statements " + str(i) + " \n\t");
    #     file.write('\n\t'.join(val));

    # for i, val in enumerate(raw_cache):
    #     file.write("\n\nCache<> Statements " + str(i) + " \n\t");
    #     file.write('\n\t'.join(val));

    # for i, val in enumerate(raw_trigger_list):
    #     file.write("\n\nTrigger<> Statements " + str(i) + " \n\t");
    #     file.write('\n\t'.join(val));

    # for i, val in enumerate(raw_exists):
    #     file.write("\n\nExists<> Statements " + str(i) + " \n\t");
    #     file.write('\n\t'.join(val));

    # for i, val in enumerate(raw_get):
    #     file.write("\n\nGet<> Statements " + str(i) + " \n\t");
    #     file.write('\n\t'.join(val));

