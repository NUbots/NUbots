#!/usr/bin/python

from info.type_to_string import type_to_string

import pdb
import sys
import json
import re
from pydotplus.graphviz import Dot, Node, Edge, Cluster

class NUClearGraphBuilder:
    def __init__(self):
        self.modules = []

    def add_module(self, filename):

        with open(filename, 'r') as file:
            module = json.load(file)

            # Don't load some super connected modules
            if module['name'] in [
                'support::logging::ConsoleLogHandler',
                'support::NUbugger',
                'support::extension::FileWatcher',
            ]:
                return

            self.modules.append(module)

    def build_reaction_graph(self, group_clusters=True):

        # A unique identifier for use in the clusters
        id = 0

        # Our graph
        graph = Dot(graph_type='digraph', suppress_disconnected=False, splines=True, overlap=False, layout='neato')

        # Loop through each of our modules
        for m1 in self.modules:
            if group_clusters:
                # Add a cluster for the module
                cluster = Cluster(graph_name=str(id), label='"{}"'.format(m1['name']))
                graph.add_subgraph(cluster);
                id += 1
            else:
                cluster = graph

            # Add a node for our module itself
            node = {
                'name': '"{}"'.format(m1['name']),
                'label': '"{}"'.format(m1['name']),
                'shape': 'rect'
            }
            cluster.add_node(Node(**node))

            for outputs in m1['output_data']:
                for output in outputs:
                    # Loop through our reactions in all the other modules
                    for m2 in self.modules:
                        for r2 in m2['reactions']:

                            # Get our destination fqn
                            dst_text_dsl = type_to_string(['DSL', r2['dsl']])[4:-1]
                            dst_reaction_identifier = '0x{0:x}<{1}>'.format(r2['address'], dst_text_dsl)
                            dst_fqn = m2['name'] + '::' + dst_reaction_identifier

                            for input in r2['input_data']:

                                # TODO make a function here that links edges
                                # edge = self.getEdge(input_name, input, output_name, output)
                                # if edge: graph.add_edge(edge)

                                if input['scope'] == output['scope'] and input['type'] == output['type']:
                                    if input['scope'] == 'type':

                                        # Get our type name
                                        type_name = type_to_string(input['type'])

                                        # give our global arguments
                                        edge = {
                                            'src':   '"{}"'.format(m1['name']),
                                            'dst':   '"{}"'.format(dst_fqn),
                                            'label': '"{}"'.format(type_name),
                                            # Red for triggering edges, blue for data only edges
                                            'color': '#FF0000' if input['modifiers'].get('execution', False) else '#0000FF'
                                        }

                                        # TODO make this function get modifiers for the edge
                                        # self.edgeModifiers(input['modifiers'], output['modifiers'])

                                        if input['modifiers'].get('optional', False):
                                            edge['style'] = 'dashed'

                                        if output['modifiers'].get('direct', False):
                                            edge['arrowtail'] = 'dot'

                                        if output['modifiers'].get('initialize', False):
                                            edge['arrowtail'] = 'odot'

                                        graph.add_edge(Edge(**edge))


            # Loop through the reactions of each module
            for r1 in m1['reactions']:

                # Get identifiers for this reaction
                src_text_dsl = type_to_string(['DSL', r1['dsl']])[4:-1]
                src_reaction_identifier = '0x{0:x}<{1}>'.format(r1['address'], src_text_dsl)
                src_fqn = m1['name'] + '::' + src_reaction_identifier
                label = r1['name'] if r1['name'] else src_text_dsl

                # TODO if the label is Configuration, we need to get the .yaml file to help the label

                node = {
                    'name': '"{}"'.format(src_fqn),
                    'label': '"{}"'.format(label),
                    'shape': 'rect'
                }

                # Add the node
                cluster.add_node(Node(**node))

                # TODO look at the modifiers for this reaction
                # sync
                # single
                # priority
                # And modifiy their colour to signifiy that

                # Go through our outputs
                for outputs in r1['output_data']:
                    for output in outputs:

                        # Loop through our reactions in all the other modules
                        for m2 in self.modules:
                            for r2 in m2['reactions']:

                                # Get our destination fqn
                                dst_text_dsl = type_to_string(['DSL', r2['dsl']])[4:-1]
                                dst_reaction_identifier = '0x{0:x}<{1}>'.format(r2['address'], dst_text_dsl)
                                dst_fqn = m2['name'] + '::' + dst_reaction_identifier

                                for input in r2['input_data']:


                                    if input['scope'] == output['scope'] and input['type'] == output['type']:
                                        if input['scope'] == 'type':

                                            # Get our type name
                                            type_name = type_to_string(input['type'])

                                            # give our global arguments
                                            edge = {
                                                'src':   '"{}"'.format(src_fqn),
                                                'dst':   '"{}"'.format(dst_fqn),
                                                'label': '"{}"'.format(type_name),
                                                # Red for triggering edges, blue for data only edges
                                                'color': '#FF0000' if input['modifiers'].get('execution', False) else '#0000FF'
                                            }

                                            if input['modifiers'].get('optional', False):
                                                edge['style'] = 'dashed'

                                            if input['modifiers'].get('binding', False):
                                                edge['style'] = 'dotted'

                                            if output['modifiers'].get('direct', False):
                                                edge['dir'] = 'both'
                                                edge['arrowtail'] = 'dot'

                                            if output['modifiers'].get('initialize', False):
                                                edge['dir'] = 'both'
                                                edge['arrowtail'] = 'odot'

                                            graph.add_edge(Edge(**edge))

                                            # Add our edge
                                            # graph.add_edge(Edge(src='"{}"'.format(src_fqn), dst='"{}"'.format(dst_fqn), label='"{}"'.format(type_name)))

                                            # TODO
                                            # modifiers
                                            #   'optional' -> style=dashed
                                            #   'last'     -> lastthings
                                            #   'binding'  -> things
                                            #   'execution' -> RED
                                            #   'data'      -> BLUE
                                    # pass
                                    # pdb.set_trace()
                                    #     print 1

                        # We have input modifiers and output modifiers

                        # Check what scope it is

                        # executing in one colour
                        # data in another colour
                        # the optional option makes it dashed
                        # the last n option makes it doubleline?

                        # TODO check if these two link together

                        # TODO make an edge

                        # todo make an Edge element here if it matches


                # add to the subgraph?




                # scopes
                #   'type'
                #   'every'
                #   'always'
                #   'system_event'
                #   'io'
                #   'network'
                #   'tcp'
                #   'udp'
                #   'udp_broadcast'
                #   'udp_multicast'
                #   'configuration'

                # modifiers
                #   'optional'
                #   'last'
                #   'binding'

                # Reaction modifiers
                #   'sync'
                #   'single'
                #   'priority'


        # Loop through each of our modules

        # Loop through each modules reactions

        return graph

    def build_module_graph(self):

        # Build up a list of compressed modules
        compressed_modules = []

        for module in self.modules:

            compressed = {
                'name': module['name'],
                'input_data': [],
                'output_data': []
            }

            # Add all of our reaction's inputs and outputs
            for reaction in module['reactions']:
                for input in reaction['input_data']:
                    if input not in compressed['input_data']:
                        compressed['input_data'].append(input)

                for outputs in reaction['output_data']:
                    for output in outputs:
                        if output not in compressed['output_data']:
                            compressed['output_data'].append(output)

            # Add our modules floating inputs and outputs
            for outputs in module['output_data']:
                for output in outputs:
                    if output not in compressed['output_data']:
                        compressed['output_data'].append(output)


            compressed_modules.append(compressed)


        # Our graph
        graph = Dot(graph_type='digraph', suppress_disconnected=False, splines=True, overlap=False, layout='neato')

        for m1 in compressed_modules:

            # Add a node for our module itself
            node = {
                'name': '"{}"'.format(m1['name']),
                'label': '"{}"'.format(m1['name']),
                'shape': 'rect'
            }
            graph.add_node(Node(**node))

            # Loop through our outputs
            for output in m1['output_data']:

                # Loop through the other modules inputs
                for m2 in compressed_modules:

                    for input in m2['input_data']:

                        # Make an edge if it matches
                        if input['scope'] == output['scope'] and input['type'] == output['type']:
                            if input['scope'] == 'type':

                                # Get our type name
                                type_name = type_to_string(input['type'])

                                # give our global arguments
                                edge = {
                                    'src':   '"{}"'.format(m1['name']),
                                    'dst':   '"{}"'.format(m2['name']),
                                    'label': '"{}"'.format(type_name),
                                    # Red for triggering edges, blue for data only edges
                                    'color': '#FF0000' if input['modifiers'].get('execution', False) else '#0000FF'
                                }

                                if input['modifiers'].get('optional', False):
                                    edge['style'] = 'dashed'

                                if input['modifiers'].get('binding', False):
                                    edge['style'] = 'dotted'

                                if output['modifiers'].get('direct', False):
                                    edge['dir'] = 'both'
                                    edge['arrowtail'] = 'dot'

                                if output['modifiers'].get('initialize', False):
                                    edge['dir'] = 'both'
                                    edge['arrowtail'] = 'odot'

                                graph.add_edge(Edge(**edge))

        return graph

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
    converter.build_reaction_graph(group_clusters=True).write("{}_reaction.dot".format(out))
