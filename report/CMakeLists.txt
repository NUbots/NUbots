# We need python!
find_package(PythonInterp REQUIRED)

# We need latex!
find_package(LATEX REQUIRED)

# We need bibtool!
find_package(Bibtool REQUIRED)

# Build the graphs!
add_subdirectory(graph)

# Make our arguments for our module files
unset(report_rel_paths)
foreach(report_file ${report_tex_files})
  file(RELATIVE_PATH rel_path ${CMAKE_SOURCE_DIR} ${report_file})
  set(report_rel_paths ${report_rel_paths} ${rel_path})
endforeach(report_file)

# Get all our standalone report files
file(GLOB_RECURSE standalone_files "${CMAKE_CURRENT_SOURCE_DIR}/src/**.tex")
# Add those onto the report_files
set(report_tex_files ${report_tex_files} ${standalone_files})

# Get their relative paths for the standalone files
unset(standalone_report_rel_paths)
foreach(report_file ${standalone_files})
  file(RELATIVE_PATH rel_path ${CMAKE_CURRENT_SOURCE_DIR}/src/ ${report_file})
  set(standalone_report_rel_paths ${standalone_report_rel_paths} ${rel_path})
endforeach(report_file)

# Add the new relative paths to the list
set(report_rel_paths ${report_rel_paths} ${standalone_report_rel_paths})

# Build our main tex file that includes the others
add_custom_command(
  OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/main.tex"
  COMMAND
    ${PYTHON_EXECUTABLE}
    ${CMAKE_SOURCE_DIR}/cmake/scripts/generate_report_tex.py
    ${CMAKE_CURRENT_BINARY_DIR}/main.tex
    ${CMAKE_CURRENT_SOURCE_DIR}/header.tex
    ${CMAKE_CURRENT_SOURCE_DIR}/footer.tex
    ${report_tex_files}
    ${report_rel_paths}
  DEPENDS
    ${CMAKE_SOURCE_DIR}/tools/scripts/generate_report_tex.py ${CMAKE_CURRENT_SOURCE_DIR}/header.tex ${report_tex_files}
  COMMENT "Generating main .tex file for the report"
)

unset(bibtool_args)
foreach(bib_file ${report_bib_files})
  set(bibtool_args ${bibtool_args} "-i" ${bib_file})
endforeach(bib_file)

# Build our main tex file that includes the others
add_custom_command(
  OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/references.bib"
  COMMAND
    ${BIBTOOL_EXEC}
    "-d"
    "-q"
    ${bibtool_args}
    "-o"
    "${CMAKE_CURRENT_BINARY_DIR}/references.bib"
  DEPENDS ${report_bib_files}
  COMMENT "Generating bibliography file for the report"
)

# Build our first latex run
add_custom_command(
  OUTPUT
    "${CMAKE_CURRENT_BINARY_DIR}/main.aux" "${CMAKE_CURRENT_BINARY_DIR}/main.log" "${CMAKE_CURRENT_BINARY_DIR}/main.idx"
    "${CMAKE_CURRENT_BINARY_DIR}/main.toc"
  COMMAND "${PDFLATEX_COMPILER}" "-interaction=batchmode" "--shell-escape" "--file-line-error" "main.tex"
  DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/main.tex"
  COMMENT "Running the first pass of the latex compiler"
)

# Build our bibliographies
add_custom_command(
  OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/main.bbl" "${CMAKE_CURRENT_BINARY_DIR}/main.blg"
  COMMAND "${BIBTEX_COMPILER}" "-terse" "main"
  DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/references.bib" "${CMAKE_CURRENT_BINARY_DIR}/main.aux"
  COMMENT "Running the bibtex bibliography compiler"
)

# Build our indicies
add_custom_command(
  OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/main.ilg" "${CMAKE_CURRENT_BINARY_DIR}/main.ind"
  COMMAND "${MAKEINDEX_COMPILER}" "-q" "main.idx"
  DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/main.idx"
  COMMENT "Running the makeindex index generator"
)

# Do our final latex run
add_custom_command(
  OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/main.pdf"
  COMMAND
    "${PDFLATEX_COMPILER}" "-interaction=batchmode" "--shell-escape" "--file-line-error"
    "${CMAKE_CURRENT_BINARY_DIR}/main.tex"
  COMMAND
    "${PDFLATEX_COMPILER}" "-interaction=batchmode" "--shell-escape" "--file-line-error"
    "${CMAKE_CURRENT_BINARY_DIR}/main.tex"
  DEPENDS
    "${CMAKE_CURRENT_BINARY_DIR}/main.tex" "${CMAKE_CURRENT_BINARY_DIR}/main.ind" "${CMAKE_CURRENT_BINARY_DIR}/main.bbl"
  COMMENT "Running the final pass of the latex compiler"
)

add_custom_target(report DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/main.pdf")
set_property(TARGET report PROPERTY EXCLUDE_FROM_ALL TRUE)
set_property(TARGET report PROPERTY EXCLUDE_FROM_DEFAULT_BUILD TRUE)

# Make our installer

# Get all the .tex files

# Find out what section they should be in e.g. behaviour/Controller

# For every section there should be a label ref so that other modules can link to it

# Generate the master tex file

# Merge the bibliography sections

# Call all of the things latex bibtex makeglosseries make index latex latex

# DEPENDENCIES

# The generateReport
