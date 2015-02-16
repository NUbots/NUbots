#!/usr/bin/python

import sys
import json
import re
from pydotplus.graphviz import Dot, Node, Edge


"""
Sample inputs:

vision.json
{
    "module_name": "Vision",
    "reactions": [{
        "name": "SomeReaction",
        "inputs": [{
            "name": "message::input::Vision",
            "trigger": true,
            "with": false
        }],
        "outputs": [{
            "name": "message::input::ClassifiedImage",
            "scopes": [
                "INITIALIZE"
            ]
        }]
    }]
}


classified.json
{
    "module_name": "ClassifiedImage",
    "reactions": [{
        "name": "SomeOtherReaction",
        "inputs": [{
            "name": "message::input::ClassifiedImage",
            "trigger": false,
            "with": true
        }],
        "outputs": []
    }]
}

Produces graph: "Vision" --(message::input::ClassifiedImage)--> "ClassifiedImage"
"""

class JsonToDot:
    def __init__(self):
        self.graph = Dot(graph_name='Emit Graph', graph_type='digraph', suppress_disconnected=True, splines=False)
        self.graph.set_node_defaults(shape='box')
        self.half_edges = {}

    def parse(self, filename):

        with open(filename, 'r') as file:
            data = json.load(file)
            module_name = data['module_name']

            if module_name == 'support::NUbugger':
                return

            node = Node(name=module_name)
            self.graph.add_node(node)

            for reaction_output in data['outputs']:

                effective_module_name = module_name

                # Our output may actually belong to us
                if reaction_output['type'] and reaction_output['type'][0] == 'NUClear':
                    effective_module_name = 'NUClear'
                elif (reaction_output['type']) > 3 and reaction_output['type'][:3] == ['messages', 'support', 'Configuration']:
                    effective_module_name = 'support::configuration::ConfigSystem'

                output_name = self.type_to_str(reaction_output['type'])

                if self.is_valid_edge(output_name):
                    edge = self._add_output(effective_module_name, output_name)
                    self._check_edge(output_name, edge)

            for reaction in data['reactions']:

                for reaction_input in reaction['inputs']:
                    input_name = self.type_to_str(reaction_input['type'])
                    if self.is_valid_edge(input_name):
                        edge = self._add_input(module_name, input_name)
                        self._check_edge(input_name, edge)

                for reaction_output in reaction['outputs']:
                    effective_module_name = module_name
                    # Our output may actually belong to us
                    if reaction_output['type'] and reaction_output['type'][0] == 'NUClear':
                        effective_module_name = 'NUClear'
                    elif (reaction_output['type']) > 3 and reaction_output['type'][:3] == ['messages', 'support', 'Configuration']:
                        effective_module_name = 'support::configuration::ConfigSystem'

                    output_name = self.type_to_str(reaction_output['type'])
                    if self.is_valid_edge(output_name):
                        edge = self._add_output(effective_module_name, output_name)
                        self._check_edge(output_name, edge)

    def is_valid_edge(self, name):
        return True

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

            sys.stderr.write("T: {}\nP: {}\nP: {}\n\n".format(ticks, is_per, period))

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
                flat_list.append('<{}>'.format(','.join([self.type_to_str(subitem) for subitem in item])))
            else:
                previous = item

        if not short_type:
            flat_list.append(previous)


        return '::'.join(flat_list)

    def _add_input(self, module_name, input_name):
        edge = None
        try:
            edge = self.half_edges[input_name]
            edge['dst'] = '"{}"'.format(module_name)
        except KeyError:
            edge = self.half_edges[input_name] = {
                'dst': '"{}"'.format(module_name),
                'src': None,
                'label': '"{}"'.format(input_name)
            }
        return edge

    def _add_output(self, module_name, output_name):
        edge = None
        try:
            edge = self.half_edges[output_name]
            edge['src'] = '"{}"'.format(module_name)
        except KeyError:
            edge = self.half_edges[output_name] = {
                'dst': None,
                'src': '"{}"'.format(module_name),
                'label': '"{}"'.format(output_name)
            }
        return edge

    def _check_edge(self, type_name, edge):
        if edge['src'] and edge['dst']:
            self.graph.add_edge(Edge(**edge))
            del self.half_edges[type_name]


    def save(self, filename):
        self.graph.write(filename)


if __name__ == "__main__":
    converter = JsonToDot()

    out = sys.argv[1]
    filenames = sys.argv[2:]
    for filename in filenames:
        converter.parse(filename)

    converter.save(out);
