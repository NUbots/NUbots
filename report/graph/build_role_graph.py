#!/usr/bin/python

import sys
import json
import re
from pydotplus.graphviz import Dot, Node, Edge, Cluster

class NUClearGraphBuilder:
    def __init__(self):
        self.modules = {}

    def add_module(self, filename):

        with open(filename, 'r') as file:
            data = json.load(file)

            # Our relevant fields
            module_name = data['module_name']
            module_outputs = data['outputs']
            module_reactions = data['reactions']

            # TODO NUbugger is crazy!
            if module_name == 'support::NUbugger':
                return

            # Add our outputs
            for output in module_outputs:

                # Our output may actually belong to someone else!
                if output['type'] and output['type'][0] == 'NUClear':
                    target_module = 'NUClear'
                elif len(output['type']) > 3 and output['type'][:3] == ['messages', 'support', 'Configuration']:
                    target_module = 'support::configuration::ConfigSystem'
                else:
                    target_module = module_name

                # Get our module (or create it)
                module = self.modules.setdefault(target_module, {
                    'name': target_module,
                    'outputs': set(),
                    'reactions': []
                })

                # Add this parsed output to the module
                module['outputs'].add(self.type_to_str(output['type']))

            # Add our reactions
            for reaction in module_reactions:
                # Work out a name for our reaction

                dsl = ['on'] + [[ x['scope'] + [[ x['type']]] for x in reaction['inputs']]]
                reaction_name = self.type_to_str(dsl);

                module = self.modules.setdefault(module_name, {
                    'name': module_name,
                    'outputs': set(),
                    'reactions': []
                })

                module['reactions'].append({
                    'name': reaction_name,
                    'inputs': set(),
                    'outputs': set()
                })

                r = module['reactions'][-1]

                # Add our reaction outputs
                for output in reaction['outputs']:
                    # Parse our output type into a sensible name
                    r['outputs'].add(self.type_to_str(output['type']))

                # Add our reaction inputs
                for input in reaction['inputs']:
                    # Parse our input type into a sensible name
                    r['inputs'].add(self.type_to_str(input['type']))

    def type_to_str(self, type_list):

        # Extract datatypes out of DataFor types
        if len(type_list) > 2 and type_list[:2] == ['NUClear', 'DataFor']:
            type_list = type_list[2][0]

        # Remove the allocator from vectors
        if len(type_list) > 2 and type_list[:2] == ['std', 'vector']:
            type_list = type_list[:2] + [[type_list[2][0]]]

        # Parse Every into sanity
        if len(type_list) > 3 and type_list[:3] == ['NUClear', 'dsl', 'Every']:
            # Get the number of ticks we are doing
            ticks = int(type_list[3][0][0])

            # Extract our period information
            period = type_list[3][1]
            is_per = len(type_list[3][1]) > 3 and period[:3] == ['NUClear', 'dsl', 'Per']
            if is_per:
                period = period[3][0]
            period = period[3][-1][-1]
            period = ['std', 'chrono'] + {
                1e-9 : ['nanoseconds'],
                1e-6 : ['microseconds'],
                1e-3 : ['milliseconds'],
                1    : ['seconds'],
                60   : ['minutes'],
                3600 : ['hours']
            }[float(period[0][0][:-2]) / float(period[1][0][:-2])]

            if is_per:
                type_list = type_list[:3] + [[[str(ticks)], ['NUClear', 'dsl', 'Per', [period]]]]
            else:
                type_list = type_list[:3] + [[[str(ticks)], period]]
                pass

        # Optional and Raw won't link as they are, pull out the real types
        if len(type_list) > 3 and type_list[:3] == ['NUClear', 'dsl', 'Optional'] or type_list[:3] == ['NUClear', 'dsl', 'Raw']:
            type_list = type_list[3][0]

        flat_list = []
        short_type = False
        previous = type_list[0]

        for item in type_list:
            if isinstance(item, basestring) and (short_type or (item[0].isupper() and item != 'NUClear')):
                short_type = True
                flat_list.append(item)
            elif not isinstance(item, basestring):
                # If we didn't have a type yet we must use the last one
                if not short_type:
                    flat_list.append(previous)
                    short_type = True
                flat_list.append('<{}>'.format(', '.join([self.type_to_str(subitem) for subitem in item])))
            else:
                previous = item

        if not short_type:
            flat_list.append(previous)


        return '::'.join(flat_list)

    def build_reaction_graph(self):

        # Our graph
        graph = Dot(graph_type='digraph', suppress_disconnected=False, splines=True, overlap=False)

        # Add our nodes
        id = 0
        for index in self.modules:
            module = self.modules[index]
            # Make a unique group for our cluster
            cluster = Cluster(graph_name=str(id), label='"{}"'.format(module['name']))
            id += 1
            graph.add_subgraph(cluster);

            for reaction in module['reactions']:
                fqn = module['name'] + '::' + reaction['name']
                node = Node(name='"{}"'.format(fqn), label='"{}"'.format(reaction['name']), shape='rect')

                cluster.add_node(node)

        # Loop through all our reactions
        for m1 in self.modules:

            src_module = self.modules[m1]
            for us in src_module['reactions']:

                # Our fully qualified unique name
                src_fqn = src_module['name'] + '::' + us['name']

                # Look for nodes that have our outputs in their inputs
                for m2 in self.modules:
                    dst_module = self.modules[m2]
                    for them in dst_module['reactions']:

                        # Our fully qualified unique name
                        dst_fqn = dst_module['name'] + '::' + them['name']

                        # Go through every type that overlaps our outputs to their inputs
                        for type in (us['outputs'] & them['inputs']):
                            graph.add_edge(Edge(src='"{}"'.format(src_fqn), dst='"{}"'.format(dst_fqn), label='"{}"'.format(type)))

        return graph

    def build_module_graph(self):

        # Our graph
        graph = Dot(graph_type='digraph', suppress_disconnected=False, splines=True, overlap=False, layout='neato')

        # Build up a compressed modules list
        graph_modules = {}

        # Make a version of our data where all the modules are compressed into one
        for m1 in self.modules:

            # Skip several modules because they are so connected
            if m1 == 'support::configuration::ConfigSystem' or m1 == 'support::logging::ConsoleLogHandler':
                continue

            module = self.modules[m1]

            # Get our module
            gm = graph_modules.setdefault(module['name'], {
                'name': module['name'],
                'inputs': set(),
                'outputs': set()
            })

            # Add in our global outputs
            gm['outputs'] |= module['outputs']

            # Add in our reaction outputs
            for reaction in module['reactions']:
                gm['outputs'] |= reaction['outputs']
                gm['inputs'] |= reaction['inputs']

        # Loop through to make nodes
        for m1 in graph_modules:
            module = graph_modules[m1]

            node = Node(name='"{}"'.format(module['name']), shape='rect')
            graph.add_node(node)

        # Loop through to link edges
        for m1 in graph_modules:
            us = graph_modules[m1]
            for m2 in graph_modules:
                them = graph_modules[m2]

                # Go through every type that overlaps our outputs to their inputs
                for type in (us['outputs'] & them['inputs']):
                    graph.add_edge(Edge(src='"{}"'.format(us['name']), dst='"{}"'.format(them['name']), label='"{}"'.format(type), weight=0.2))

        return graph


if __name__ == "__main__":

    converter = NUClearGraphBuilder()

    out = sys.argv[1]
    filenames = sys.argv[2:]

    # Add all of our modules
    for filename in filenames:
        converter.add_module(filename)

    # Build our graph and save
    converter.build_module_graph().write("{}_module.dot".format(out))
    converter.build_reaction_graph().write("{}_reaction.dot".format(out))
