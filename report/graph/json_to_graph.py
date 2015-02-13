#!/usr/bin/python

import glob
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
        self.graph = None
        self.half_edges = {}

    def parse(self, filename):

        with open(filename, 'r') as file:
            data = json.load(file)

            self.graph = Dot(graph_name='Emit Graph', graph_type='digraph', suppress_disconnected=True, ratio="compress", splines=False)

            module_name = data['module_name']

            if module_name == 'NUbugger':
                return

            node = Node(name=module_name, shape='box')
            self.graph.add_node(node)

            for reaction in data['reactions']:
                reaction_name = reaction['name']

                for reaction_input in reaction['inputs']:
                    input_name = reaction_input['name']
                    edge = self._add_input(module_name, input_name)
                    self._check_edge(input_name, edge)

                for reaction_output in reaction['outputs']:
                    output_name = reaction_output['name']
                    edge = self._add_output(module_name, output_name)
                    self._check_edge(output_name, edge)

    def _add_input(self, module_name, input_name):
        edge = None
        try:
            edge = self.half_edges[input_name]
            edge['src'] = module_name
        except KeyError:
            edge = self.half_edges[input_name] = {
                'src': module_name,
                'dst': None,
                'label': '"{}"'.format(input_name)
            }
        return edge

    def _add_output(self, module_name, output_name):
        edge = None
        try:
            edge = self.half_edges[output_name]
            edge['dst'] = module_name
        except KeyError:
            edge = self.half_edges[output_name] = {
                'src': None,
                'dst': module_name,
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

    filenames = glob.glob('*.json')
    for filename in filenames:
        converter.parse(filename)

    converter.save('graph.dot');
