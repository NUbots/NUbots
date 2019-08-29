#!/usr/bin/env python3

import nbs_tools.json
import nbs_tools.stats
import nbs_tools.extract_images


def register(command):
    command.help = "Tools for working with nbs or nbz files"
    subcommands = command.add_subparsers(dest="nbs_command", help="Subcommand to call for working with nbs files")
    subcommands.required = True

    nbs_tools.json.register(subcommands.add_parser("json"))
    nbs_tools.stats.register(subcommands.add_parser("stats"))
    nbs_tools.extract_images.register(subcommands.add_parser("extract_images"))


def run(nbs_command, **kwargs):
    {"stats": nbs_tools.stats.run, "json": nbs_tools.json.run, "extract_images": nbs_tools.extract_images.run}[
        nbs_command
    ](**kwargs)
