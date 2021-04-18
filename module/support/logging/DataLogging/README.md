# DataLogging

## Description

Logs serialisable neutrons in nbs files. It also creates nbs.idx files.

## Usage

Add the neutron types that you want to record in the config file in the messages list field. Specify the directory you
want the log files to be written in the output directories field, the actual nbs and idx files will be created in a
subdirectory of the given directory with the same name as the role that these messages were created by running. To
specify a maximum size before which the current nbs and idx files will be closed and new nbs and idx files to be created
use the output output/split_size field.

Filenames will be of the format year, month, day, T, hours, \_, minutes, \_, seconds e.g.
`20201110T13_31_50`.

Files that are currently being written to will have an underscore prepended to the name, which will be removed when the
file is closed.

## Emits

## Dependencies
