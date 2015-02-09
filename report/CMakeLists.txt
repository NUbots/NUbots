# We need python!
FIND_PACKAGE(PythonInterp REQUIRED)

# We need latex!
FIND_PACKAGE(LATEX REQUIRED)

# We need bibtool!
FIND_PACKAGE(Bibtool REQUIRED)

# Make our arguments for our module files
UNSET(report_rel_paths)
FOREACH(report_file ${report_tex_files})
    FILE(RELATIVE_PATH rel_path ${CMAKE_SOURCE_DIR} ${report_file})
    SET(report_rel_paths ${report_rel_paths} ${rel_path})
ENDFOREACH(report_file)

# Get all our standalone report files
FILE(GLOB_RECURSE standalone_files "${CMAKE_CURRENT_SOURCE_DIR}/**.tex")
# Add those onto the report_files
SET(report_tex_files ${report_tex_files} ${standalone_files})

# Get their relative paths for the standalone files
UNSET(standalone_report_rel_paths)
FOREACH(report_file ${standalone_files})
    FILE(RELATIVE_PATH rel_path ${CMAKE_CURRENT_SOURCE_DIR} ${report_file})
    SET(standalone_report_rel_paths ${standalone_report_rel_paths} ${rel_path})
ENDFOREACH(report_file)

# Add the new relative paths to the list
SET(report_rel_paths ${report_rel_paths} ${standalone_report_rel_paths})

# Build our main tex file that includes the others
ADD_CUSTOM_COMMAND(
    OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/main.tex"
    COMMAND ${PYTHON_EXECUTABLE}
            ${CMAKE_SOURCE_DIR}/cmake/scripts/generate_report_tex.py
            ${CMAKE_CURRENT_BINARY_DIR}/main.tex
            ${CMAKE_CURRENT_SOURCE_DIR}/header.tex
            ${report_tex_files}
            ${report_rel_paths}
    DEPENDS ${CMAKE_SOURCE_DIR}/cmake/Scripts/generate_report_tex.py
            ${CMAKE_CURRENT_SOURCE_DIR}/header.tex
            ${report_tex_files}
    COMMENT "Generating main .tex file for the report"
)

UNSET(bibtool_args)
FOREACH(bib_file ${report_bib_files})
    SET(bibtool_args ${bibtool_args} "-i" ${bib_file})
ENDFOREACH(bib_file)

# Build our main tex file that includes the others
ADD_CUSTOM_COMMAND(
    OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/references.bib"
    COMMAND ${BIBTOOL_EXEC}
            "-d" "-q"
            ${bibtool_args}
            "-o" "${CMAKE_CURRENT_BINARY_DIR}/references.bib"

    DEPENDS ${report_bib_files}
    COMMENT "Generating bibliography file for the report"
)

ADD_CUSTOM_TARGET(report
                  DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/main.tex"
                          "${CMAKE_CURRENT_BINARY_DIR}/references.bib")


 # Make our installer


# Get all the .tex files

# Find out what section they should be in
#	e.g. behaviour/Controller

# For every section there should be a label ref so that other modules can link to it

# Generate the master tex file

# Merge the bibliography sections

# Call all of the things
# latex
# bibtex
# makeglosseries
# make index
# latex
# latex


# DEPENDENCIES

# The generateReport