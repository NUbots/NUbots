#!/usr/bin/python

import sys
import re

toRead = open(sys.argv[1], "r")
toWrite = open(sys.argv[2], "w")

textToParse = toRead.read()

# add '(//.*|/\*.*\*/)' for comments?
# Find an on statement and grab the DSL, then grab the arguments, pass over the .then, grab the capture, then the
# arguments, then optionally have a -> then pass through the body and finally end with });.
onPattern = re.compile(r"on<(.*)>\((.*)\)\.then\((\[.*\])(\(.*\))?\s*(-> (ret)?)?{(.|\n)*?}\);")
# Start with a , or a ( then check if const, then grab the type, then grab the name
paramsPattern = re.compile(r"(\(|, ?)(const)? ([^,) ]*?) ([^,) ]*)")
# Start with emit, then grab DSL, then the things we are passing in, then end with );
emitPattern = re.compile(r"emit<(.*?)>\((.*)\);")
argsPattern = re.compile(r"[^,\s]+")

for thing in re.findall(onPattern, textToParse):
    toWrite.write("On " + thing[0].replace("<", "\<") + "\n")
    toWrite.write("* Params: " + thing[1] + "\n")
    toWrite.write("* Capture: " + thing[2] + "\n")
    toWrite.write("* Lambda Params: \n")
    for param in re.findall(paramsPattern, thing[3]):
        toWrite.write("    * " + param[3] + " : " + param[1] + " " + param[2].replace("<", "\<") + "\n")
    toWrite.write("\n")

for thing in re.findall(emitPattern, textToParse):
    toWrite.write("Emit " + thing[0].replace("<", "\<") + "\n")
    toWrite.write("* Params: \n")
    for arg in re.findall(argsPattern, thing[1]):
        print(arg)
        toWrite.write("    * " + arg + "\n")
    toWrite.write("\n")


toRead.close()
toWrite.close()
