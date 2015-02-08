# We need python!
FIND_PACKAGE(PythonInterp REQUIRED)

# We need latex!
FIND_PACKAGE(LATEX REQUIRED)

# Make our arguments!
UNSET(report_rel_paths)
FOREACH(report_file ${report_files})
	FILE(RELATIVE_PATH rel_path ${CMAKE_SOURCE_DIR} ${report_file})
	SET(report_rel_paths ${report_rel_paths} ${rel_path})
ENDFOREACH(report_file)

# Build our main tex file that includes the others
ADD_CUSTOM_COMMAND(
    OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/main.tex"
    COMMAND ${PYTHON_EXECUTABLE}
            ${CMAKE_SOURCE_DIR}/cmake/scripts/generate_report_tex.py
            ${CMAKE_CURRENT_BINARY_DIR}/main.tex
            ${CMAKE_CURRENT_SOURCE_DIR}/header.tex
            ${CMAKE_CURRENT_SOURCE_DIR}/footer.tex
            ${report_files}
            ${report_rel_paths}
    DEPENDS ${CMAKE_SOURCE_DIR}/cmake/Scripts/generate_report_tex.py
            ${CMAKE_CURRENT_SOURCE_DIR}/header.tex
            ${CMAKE_CURRENT_SOURCE_DIR}/footer.tex
            ${report_files}
    COMMENT "Generating main .tex file for the report"
)

ADD_CUSTOM_TARGET(report
                  DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/main.tex")


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