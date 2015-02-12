#!/usr/bin/python

import sys
import os
import re

def pathComponents(path):
    path = os.path.dirname(path);
    folders = [];
    while 1:
        path,folder=os.path.split(path);

        if folder != "":
            folders.append(folder);
        else:
            if path != "":
                folders.append(path);

            break;
    folders.reverse();
    return folders;

def fixName(name, leaf):
    # If it's a leaf node, camelcase to words
    if leaf:
        # Split up based on caps runs
        words = filter(None, re.split("([A-Z]+[a-z]+)", name));
        return ' '.join(words);
    else:
        words = [item.capitalize() for item in name.split("_")]
        return ' '.join(words);

    # Otherwise underscore to words

def addModuleContent(file, tree, depth):

    # Print our tex content if we have any
    if tree[1] != "":

        file.write(re.sub("^(.*)$", (("\t") * depth) + "\\1", tree[1], flags=re.MULTILINE));
        file.write("\n");

    # Loop through each of this level sorted so that subsections come after
    for elem in sorted(tree[0], key=lambda item: (len(item[1][0]) == 0, item[0])):
        # Fix our name to be wordy
        name = fixName(elem, len(tree[0][elem][0]) == 0);

        # Write our section level
        file.write({ 0: "\n\part",
                     1: "\n\t\chapter",
                     2: "\n\t\t\section",
                     3: "\n\t\t\t\subsection",
                     4: "\n\t\t\t\t\subsection",
                     5: "\n\t\t\t\t\t\subsubsection" }[depth]);
        file.write("{" + name + "}\n")
        addModuleContent(file, tree[0][elem], depth + 1);


if sys.argv[1]:
    output_file = sys.argv[1];
else:
    print 'You must specify an output file\n';
    sys.exit(1);

if sys.argv[2]:
    header = sys.argv[2];
else:
    print 'You must specify a document header\n';
    sys.exit(1);

if sys.argv[3]:
    footer = sys.argv[3];
else:
    print 'You must specify a document footer\n';
    sys.exit(1);

if sys.argv[4:]:
    modules = sys.argv[4:];
    modules = zip(modules[:(len(modules)/2)], modules[(len(modules)/2):])
else:
    print 'You must specify some report modules\n';
    sys.exit(1);

# Open our output file for writing
with open(output_file, 'w') as file:

    # Open and write the header
    with open(header, 'r') as h:
        file.write(h.read() + "\n");

    # Make our tree
    tree = [ dict(), "" ];

    # Look through all of our source files
    for module in modules:

        path = tree;

        folders = pathComponents(module[1]);
        for folder in folders[:-1]:
            # Create the element if it doesn't exist
            if folder not in path[0]:
                path[0][folder] = [ dict(), "" ];

            # Move to our new path in the tree
            path = path[0][folder];

        # Add our tex content to the path
        with open(module[0], 'r') as tex:
            path[1] += tex.read();

    # Add the content from the modules to the file
    addModuleContent(file, tree, 0);

    # Open and write the footer
    with open(footer, 'r') as f:
        file.write("\n" + f.read());


