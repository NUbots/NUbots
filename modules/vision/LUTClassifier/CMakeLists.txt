# Find the QUEX Library
FIND_PACKAGE(QUEX REQUIRED)

# Our lexer file
SET(lexer "src/Lexer.qx")

# Get the filename without the extension
GET_FILENAME_COMPONENT(file_we ${lexer} NAME_WE)

# Get the absolute file location
GET_FILENAME_COMPONENT(abs_file ${lexer} ABSOLUTE)

# Work out our output path
FILE(RELATIVE_PATH outputpath ${CMAKE_CURRENT_SOURCE_DIR} ${abs_file})
GET_FILENAME_COMPONENT(outputpath ${outputpath} PATH)
SET(outputpath "${CMAKE_CURRENT_BINARY_DIR}/${outputpath}")

# Make our output directory
FILE(MAKE_DIRECTORY ${outputpath})

# Add the command to generate the lexer
# Make the quex
ADD_CUSTOM_COMMAND(
    OUTPUT "${outputpath}/Lexer-token_ids.hpp"
           "${outputpath}/Lexer.cpp"
           "${outputpath}/Lexer.hpp"
           "${outputpath}/Lexer-token.hpp"
           "${outputpath}/Lexer-configuration.hpp"
    COMMAND "quex"
    ARGS -i ${abs_file} --file-extension-scheme pp --output-directory ${outputpath} --language C++ --engine Lexer --token-policy single

    # TODO to try and improve performanec
    # Enable computed gotos (QUEX_OPTION_COMPUTED_GOTOS)
    # Enable template compression
    # Enable path compression
    # Disable the string accumulator (--no-string-accumulator)
    # Disable line counting (--no-count-lines)
    # Enable Single Mode Analyser (--single-mode-analyzer)
    # Disable Transition check (--no-mode-transition-check)
    # Disable DOS eol checks (probably won't help) (--no-DOS)
    # Set the codec to ascii (rather then the default utf8) (--codec ASCII)
    # disable setting a termination zero (maybe miniscule speedup?) QUEX_OPTION_TERMINATION_ZERO_DISABLED

    DEPENDS ${abs_file}
    COMMENT "Generating Quex lexer for vision"
    VERBATIM)

# Build our NUClear module
NUCLEAR_MODULE(SOURCES
    ${outputpath}/Lexer-token_ids.hpp
    ${outputpath}/Lexer.cpp
    ${outputpath}/Lexer.hpp
    ${outputpath}/Lexer-token.hpp
    ${outputpath}/Lexer-configuration.hpp)