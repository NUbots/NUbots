# Build our NUClear module

find_package(OpenVINO REQUIRED)
find_package(TBB REQUIRED)

# Define paths
set(ONNX_MODEL_PATH "${CMAKE_CURRENT_SOURCE_DIR}/data/model.onnx")
set(IR_OUTPUT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/data")
set(IR_MODEL_XML "${IR_OUTPUT_DIR}/model.xml")
set(IR_MODEL_BIN "${IR_OUTPUT_DIR}/model.bin")

# Check if OpenVINO model optimizer (mo) is available
find_program(OPENVINO_MO mo)
if(NOT OPENVINO_MO)
    # Try alternative names/locations
    find_program(OPENVINO_MO ovc)
    if(NOT OPENVINO_MO)
        message(WARNING "OpenVINO Model Optimizer not found. You may need to install openvino-dev: pip install openvino-dev")
    endif()
endif()

# Convert ONNX to OpenVINO IR format at build time (only if ONNX exists)
if(EXISTS ${ONNX_MODEL_PATH} AND OPENVINO_MO)
    add_custom_command(
        OUTPUT ${IR_MODEL_XML} ${IR_MODEL_BIN}
        COMMAND ${OPENVINO_MO}
            --input_model ${ONNX_MODEL_PATH}
            --output_dir ${IR_OUTPUT_DIR}
            --model_name model
        DEPENDS ${ONNX_MODEL_PATH}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        COMMENT "Converting ONNX model to OpenVINO IR format..."
        VERBATIM
    )

    add_custom_target(convert_rlwalk_model ALL
        DEPENDS ${IR_MODEL_XML} ${IR_MODEL_BIN}
    )

    message(STATUS "RLWalk: ONNX model will be converted to OpenVINO IR at build time")
else()
    if(NOT EXISTS ${ONNX_MODEL_PATH})
        message(WARNING "RLWalk: ONNX model not found at ${ONNX_MODEL_PATH}")
    endif()

    # Create a dummy target so build doesn't fail
    add_custom_target(convert_rlwalk_model)
endif()

# Build our NUClear module
nuclear_module(LIBRARIES TBB::tbb openvino::runtime ${OpenVINO_LIBRARIES})

# Make sure model conversion happens before module build
if(TARGET module_skill_RLWalk)
    add_dependencies(module_skill_RLWalk convert_rlwalk_model)
endif()
