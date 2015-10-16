#!/usr/bin/python

from info.type_to_string import type_to_string

import random
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

    def make_edge(self, output_id, output, input_id, input):
        # If these two types are in the same scope
        if input['scope'] == output['scope']:

            # If the two types are type scopes of the same type
            if input['scope'] == 'type' and input['type'] == output['type']:

                # Get our type name
                type_name = type_to_string(input['type'])

                # give our global arguments
                edge = {
                    'src':   '"{}"'.format(output_id),
                    'dst':   '"{}"'.format(input_id),
                    'label': '"{}"'.format(type_name),
                    # Red for triggering edges, blue for data only edges
                    'color': '#FF0000' if input['modifiers'].get('execution', False) else '#0000FF'
                }

                if input['modifiers'].get('optional', False):
                    edge['style'] = 'dashed'

                if input['modifiers'].get('last', False):
                    # TODO do something here
                    # Try to make the line a different style
                    pass

                if output['modifiers'].get('binding', False):
                    edge['style'] = 'dotted'

                if output['modifiers'].get('direct', False):
                    edge['dir'] = 'both'
                    edge['arrowtail'] = 'dot'

                if output['modifiers'].get('initialize', False):
                    edge['dir'] = 'both'
                    edge['arrowtail'] = 'odot'

                return Edge(**edge)

            if input['scope'] == 'network' and input['type'] == output['type']:
                pass
                # This is a network type

        # TODO we want to have only one edge for these
        # But they will be called here for every alternative
        if input['scope'] == 'system_event':
            # TODO this could be Startup or Shutdown
            # TODO work out what to do with these events
            pass

        if input['scope'] == 'every':
            # TODO work out what to do with these events
            pass

        if input['scope'] == 'always':
            # TODO work out what to do with these events
            pass

        if input['scope'] == 'io':
            # TODO work out what to do with these events
            pass

        if input['scope'] == 'tcp':
            # TODO work out what to do with these events
            pass

        if input['scope'] == 'udp':
            # TODO work out what to do with these events
            pass

        if input['scope'] == 'udp_broadcast':
            # TODO work out what to do with these events
            pass

        if input['scope'] == 'udp_multicast':
            # TODO work out what to do with these events
            pass

        if input['scope'] == 'configuration':
            # TODO work out what to do with these events
            pass

        if input['scope'] == 'file_watch':
            # TODO work out what to do with these events
            pass

        # We are not going to make an edge for this
        return None

    def build_reaction_graph(self, group_clusters=True):

        # A unique identifier for use in the clusters
        id = 0

        # Our graph
        graph = {
            'label': 'Reactions',
            'graph_type': 'digraph',
            'suppress_disconnected': False,
            'splines': 'polyline',
            'overlap': 'prism10000',
            'layout': 'fdp',
            'epsilon': 0.01,
            'start': int(random.random() * 2**32)
        }

        graph = Dot(**graph)

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
            if m1['output_data']:
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

                                    edge = self.make_edge(m1['name'], output, dst_fqn, input)
                                    if edge:
                                        graph.add_edge(edge)


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

                # If we are not grouping clusters, put the owner module here
                if not group_clusters:
                    node['label'] = '{0}\\n{1}'.format(m1['name'], label)

                if r1['modifiers'].get('single', False):
                    # TODO make the outline style be dashed?
                    pass # TODO modify the reaction
                if r1['modifiers'].get('sync', False):
                    # TODO make the outline style be something??
                    pass # TODO modify the reaction
                if r1['modifiers'].get('priority', False):
                    if r1['modifiers']['priority'] == 'LOW':
                        node['style'] = 'filled'
                        node['fillcolor'] = '#44FF4444'

                    elif r1['modifiers']['priority'] == 'HIGH':
                        node['style'] = 'filled'
                        node['fillcolor'] = '#FF444444'

                    # elif r1['modifiers']['priority'] == 'NORMAL':

                    # TODO make the background colour depend on priority
                    # Normal = none
                    # High = #FF444444
                    # Low = #44FF4444
                    pass # TODO modify the reaction

                # Add the node
                cluster.add_node(Node(**node))

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
                                    edge = self.make_edge(src_fqn, output, dst_fqn, input)
                                    if edge:
                                        graph.add_edge(edge)

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
        graph = Dot(graph_type='digraph', suppress_disconnected=True, splines=True, overlap='prism10000', layout='neato', epsilon=0.01, start=int(random.random() * 2**32))

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
                        edge = self.make_edge(m1['name'], output, m2['name'], input)
                        if edge:
                            graph.add_edge(edge)

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
