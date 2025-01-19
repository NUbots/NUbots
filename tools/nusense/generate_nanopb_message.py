import subprocess
import re
import os
from utility.dockerise import run_on_docker

@run_on_docker
def register(command):
    command.description = ("Generate nanopb messages given a path with a .proto file and possibly a .options file. Mainly used for NUSense.")

    # Command arguments
    command.add_argument("proto_file", help="The path to the proto file. This module assumes that you only have one proto message in the directory.")
    command.add_argument("-s",
                         "--selections",
                         nargs="+",
                         help="""Arguments for the options file, if any.
                                The expected format is <message>:<property>:<option>:<option argument>. Be wary of formatting as the generator will not be able to account for typed mistakes. This file will live in the same directory as the proto file.""")

@run_on_docker
def run(proto_file, selections, **kwargs):
    # Find the prefix of the package
    with open(proto_file, 'r') as f:
        prefix = re.search(r'package\s+(.*?);', f.read()).group(1)

    # If options were specified, write the options file to the same directory as the proto file
    if (len(selections) > 0):
        proto_dir = os.path.dirname(proto_file)
        file_name_no_ext = os.path.splitext(os.path.split(proto_file)[-1])[0]
        options_file_name = f"{file_name_no_ext}.options" if not proto_dir else f"{proto_dir}/{file_name_no_ext}.options"

        with open(options_file_name, 'w') as f:
            # Parse each selection then write to the options file
            for selection in selections:
                message, property, option, argument = selection.split(':')
                f.write(f"{prefix}.{message}.{property} {option}:{argument}\n")

    subprocess.call(["nanopb_generator", proto_file])
