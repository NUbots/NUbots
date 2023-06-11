#!/usr/bin/env python3

import os


def get_nusight_ports(yarn_command):
    port_vite = "3000"
    port_vite_hmr = "3010"
    port_node_debugger = "9229"

    command_ports = {
        "dev": [port_vite, port_vite_hmr],
        "start": [port_vite, port_vite_hmr],
        "dev:debug": [port_vite, port_vite_hmr, port_node_debugger],
        "start:debug": [port_vite, port_vite_hmr, port_node_debugger],
        "prod": ["9090"],
        "storybook": ["9001"],
        "storybook:prod": ["9002"],
    }

    return [f"{p}:{p}" for p in command_ports.get(yarn_command, [])]
