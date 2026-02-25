# OpenVINO
find_package(OpenVINO REQUIRED)
find_package(TBB REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# Extracting normalisation parameters from pytorch model at build-time
set(PT_MODEL_PATH "/home/nubots/NUbots/module/skill/RLWalk/data/model_standing_normalisation.pt")
set(OUTPUT_PATH "/home/nubots/NUbots/module/skill/RLWalk/data/config/normalisation_params.yaml")
set(EXTRACT_SCRIPT "/home/nubots/NUbots/module/skill/RLWalk/src/extract_normalisation.py")

# Run Python script to extract normalization stats at configure time
if(EXISTS ${PT_MODEL_PATH})
    message(STATUS "Extracting normalization stats from ${PT_MODEL_PATH}")
    message(STATUS "Using Python: ${Python3_EXECUTABLE}")
    execute_process(
        COMMAND ${Python3_EXECUTABLE} ${EXTRACT_SCRIPT} ${PT_MODEL_PATH} ${OUTPUT_PATH}
        RESULT_VARIABLE EXTRACT_RESULT
        OUTPUT_VARIABLE EXTRACT_OUTPUT
        ERROR_VARIABLE EXTRACT_ERROR
    )

    if(NOT EXTRACT_RESULT EQUAL 0)
        message(WARNING "Failed to extract normalization stats")
        message(WARNING "Exit code: ${EXTRACT_RESULT}")
        message(WARNING "Error: ${EXTRACT_ERROR}")
    else()
        message(STATUS "Normalization stats extracted successfully")
        message(STATUS "${EXTRACT_OUTPUT}")
    endif()
else()
    message(WARNING "Checkpoint file not found: ${PT_MODEL_PATH}")
    message(WARNING "Normalization stats will not be extracted")
endif()

# Build our NUClear module
nuclear_module(LIBRARIES TBB::tbb openvino::runtime ${OpenVINO_LIBRARIES})
