# Find our globally shared libraries:
find_package(Armadillo REQUIRED)
find_package(LibGFortran REQUIRED)
find_package(OpenBLAS)
find_package(Protobuf REQUIRED)
find_package(CATCH REQUIRED)
find_package(YAML-CPP REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)

# Resolve problems when OpenBLAS isn't found:
if(OpenBLAS_FOUND)
  set(BLAS_LIBRARIES ${OpenBLAS_LIBRARIES})
  set(BLAS_INCLUDE_DIRS ${OpenBLAS_INCLUDE_DIRS})
else()
  find_package(BLAS REQUIRED)
  message(WARNING "OpenBLAS was not found. Using BLAS instead.")
endif()

# Some definitions for armadillo
add_definitions(-DARMA_DONT_USE_WRAPPER -DARMA_32BIT_WORD)

# Set include directories and libraries:
include_directories(SYSTEM ${BLAS_INCLUDE_DIRS})
include_directories(SYSTEM ${PROTOBUF_INCLUDE_DIRS})
include_directories(SYSTEM ${CATCH_INCLUDE_DIRS})
include_directories(SYSTEM ${YAML-CPP_INCLUDE_DIRS})
include_directories(SYSTEM ${fmt_INCLUDE_DIRS})

set(NUCLEAR_ADDITIONAL_SHARED_LIBRARIES
    ${BLAS_LIBRARIES}
    ${LIBGFORTRAN_LIBRARIES}
    ${PROTOBUF_LIBRARIES}
    ${YAML-CPP_LIBRARIES}
    ${fmt_LIBRARIES}
    Eigen3::Eigen
    -ldl
    -lbacktrace
    CACHE PATH "Additional libraries used when linking roles, extensions, and utilities" FORCE
)
