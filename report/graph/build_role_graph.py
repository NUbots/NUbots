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
                output_name = self.type_to_str(reaction_output['type'])
                if self.is_valid_edge(output_name):
                    edge = self._add_output(module_name, output_name)
                    self._check_edge(output_name, edge)

            for reaction in data['reactions']:

                for reaction_input in reaction['inputs']:
                    input_name = self.type_to_str(reaction_input['type'])
                    if self.is_valid_edge(input_name):
                        edge = self._add_input(module_name, input_name)
                        self._check_edge(input_name, edge)

                for reaction_output in reaction['outputs']:
                    output_name = self.type_to_str(reaction_output['type'])
                    if self.is_valid_edge(output_name):
                        edge = self._add_output(module_name, output_name)
                        self._check_edge(output_name, edge)

    def is_valid_edge(self, name):
        if 'configuration' in name.lower():
            return False
        if 'nuclear' in name.lower():
            return False
        return True

    def type_to_str(self, type_list):
        flat_list = []
        for item in type_list:
            if isinstance(item, basestring):
                flat_list.append(item)
            else:
                flat_list.append('<{}>'.format(','.join([self.type_to_str(subitem) for subitem in item])))

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
