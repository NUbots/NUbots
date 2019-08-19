# How wide to allow formatted cmake files
line_width = 120

# How many spaces to tab for indent
tab_size = 2

# If arglists are longer than this, break them always
max_subargs_per_line = 5

# If true, separate flow control names from their parentheses with a space
separate_ctrl_name_with_space = False

# If true, separate function names from parentheses with a space
separate_fn_name_with_space = False

# If a statement is wrapped to more than one line, than dangle the closing
# parenthesis on it's own line
dangle_parens = True

# What style line endings to use in the output.
line_ending = "unix"

# Format command names consistently as 'lower' or 'upper' case
command_case = "unchanged"

# Format keywords consistently as 'lower' or 'upper' case
keyword_case = "upper"

# Setup all the additional commands
additional_commands = {
    "HeaderLibrary": {"kwargs": {"NAME": "*", "HEADER": "*", "PATH_SUFFIX": "*", "URL": "*"}},
    "ToolchainLibraryFinder": {
        "kwargs": {
            "NAME": "*",
            "HEADER": "*",
            "LIBRARY": "*",
            "PATH_SUFFIX": "*",
            "BINARY": "*",
            "VERSION_FILE": "*",
            "VERSION_BINARY_ARGUMENTS": "*",
            "VERSION_REGEX": "*",
        }
    },
}
