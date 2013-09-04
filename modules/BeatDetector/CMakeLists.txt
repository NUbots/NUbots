# Find the FFTW Library (needed by Aubio)
FIND_PACKAGE(Aubio REQUIRED)
# Find the Aubio Library
FIND_PACKAGE(Aubio REQUIRED)

# Build our NUClear module
NUCLEAR_MODULE(INCLUDES ${FFTW_INCLUDE_DIRS} ${AUBIO_INCLUDE_DIRS}
               LIBRARIES ${FFTW_LIBRARIES} ${AUBIO_LIBRARIES})
