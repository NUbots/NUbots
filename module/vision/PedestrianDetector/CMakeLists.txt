FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(tensorflow REQUIRED)

# Build our NUClear module
NUCLEAR_MODULE(LANGUAGE "CPP"
               INCLUDES ${tensorflow_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
               LIBRARIES ${tensorflow_LIBRARIES} ${OpenCV_LIBRARIES})
