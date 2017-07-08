import os
import re

rolesPath = os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..', 'roles'))
modulesPath = os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..', 'module'))

basePath = os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..', 'module')) + os.sep

existingModules = {}
missingModules = {}
commentedModules = {}
usedModules = []
unusedButCommentedModules = []

# a list of folders that could be within a module
skip_list = ['src', 'data', 'config', 'tests', 'report']
skip_list = [s + os.sep for s in skip_list]

# find all the modules that are available
for root, dirs, files in os.walk(modulesPath):
    if len(files) != 0:
        if not any(substring in (root + os.sep) for substring in skip_list):
            existingModules[root.replace(basePath, '')] = 0 # make sure we remove the base path from the module


# compare our roles to our existing modules
for root, dirs, files in os.walk(rolesPath):
    for file in files:
        with open(os.path.join(rolesPath, file), 'r') as role:
            for line in role:
                line = line.strip()

                if '#' in line: # a commented
                    line = re.sub(r"##+", '#', line) # remove multiple # that are next to each other
                    location = line.find('#')

                    if location == 0:
                        line = line[1:] # remove the comment and continue to process
                        if '::' in line: # the commented line is a commented out module
                            additionalCommentLoc = line.find('#')
                            if additionalCommentLoc > 0:
                                line = line[:additionalCommentLoc]

                            line = line.replace('::', os.sep)
                            commentedModules[line.strip()] = 1
                        continue
                    else:
                        # the comment was just a description, remove the comment and continue
                        line = line[:location].strip()

                if '::' in line: # the line is a module
                    line = line.replace('::', os.sep)
                    if line in existingModules:
                        existingModules[line] += 1
                        usedModules.append(line)
                    else:
                        missingModules[line] = 1

print('Unused modules:\n')
for key in sorted(existingModules.keys()):
    if existingModules[key] == 0:
        if key not in commentedModules.keys():
            print('\t', key)
        else:
            unusedButCommentedModules.append(key)

print('\nUnused but commented out modules:\n')
for key in sorted(commentedModules):
    if key not in usedModules:
        if key in existingModules:
            if existingModules[key] > 0:
                continue

        print('\t', key)
