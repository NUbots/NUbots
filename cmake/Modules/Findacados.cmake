# acados (https://github.com/acados/acados), the optimal control solver MPCWalkPath's generated code runs on, and the
# two libraries it is built on: HPIPM (the QP solver) and BLASFEO (the linear algebra). The Docker image builds all
# three as static archives. acados installs the HPIPM and BLASFEO headers under include/hpipm/include and
# include/blasfeo/include, and its own headers include them by bare name, so all three directories go on the path.
find_path(
  acados_INCLUDE_DIR
  NAMES acados_c/ocp_nlp_interface.h
  DOC "The acados include directory"
)
find_path(
  acados_blasfeo_INCLUDE_DIR
  NAMES blasfeo_target.h
  PATH_SUFFIXES blasfeo/include
  DOC "The BLASFEO include directory installed by acados"
)
find_path(
  acados_hpipm_INCLUDE_DIR
  NAMES hpipm_common.h
  PATH_SUFFIXES hpipm/include
  DOC "The HPIPM include directory installed by acados"
)
find_library(
  acados_LIBRARY
  NAMES acados
  DOC "The acados library"
)
find_library(
  acados_hpipm_LIBRARY
  NAMES hpipm
  DOC "The HPIPM library installed by acados"
)
find_library(
  acados_blasfeo_LIBRARY
  NAMES blasfeo
  DOC "The BLASFEO library installed by acados"
)
mark_as_advanced(
  acados_INCLUDE_DIR acados_blasfeo_INCLUDE_DIR acados_hpipm_INCLUDE_DIR acados_LIBRARY acados_hpipm_LIBRARY
  acados_blasfeo_LIBRARY
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
  acados REQUIRED_VARS acados_LIBRARY acados_hpipm_LIBRARY acados_blasfeo_LIBRARY acados_INCLUDE_DIR
                       acados_blasfeo_INCLUDE_DIR acados_hpipm_INCLUDE_DIR
)

if(acados_FOUND AND NOT TARGET acados::acados)
  add_library(acados::acados INTERFACE IMPORTED)
  target_include_directories(
    acados::acados SYSTEM INTERFACE "${acados_INCLUDE_DIR}" "${acados_blasfeo_INCLUDE_DIR}"
                                    "${acados_hpipm_INCLUDE_DIR}"
  )
  # Link order matters for static archives: acados uses HPIPM, which uses BLASFEO
  target_link_libraries(
    acados::acados INTERFACE "${acados_LIBRARY}" "${acados_hpipm_LIBRARY}" "${acados_blasfeo_LIBRARY}" m
  )
  set(acados_INCLUDE_DIRS "${acados_INCLUDE_DIR}" "${acados_blasfeo_INCLUDE_DIR}" "${acados_hpipm_INCLUDE_DIR}")
  set(acados_LIBRARIES acados::acados)
endif()
