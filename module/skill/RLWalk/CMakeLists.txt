# OpenVINO
find_package(OpenVINO REQUIRED)
find_package(TBB REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(RLWALK_ONNX_MODEL "${CMAKE_CURRENT_SOURCE_DIR}/data/walk_model.onnx")
set(RLWALK_IR_XML "${PROJECT_BINARY_DIR}/walk_model.xml")
set(RLWALK_IR_BIN "${PROJECT_BINARY_DIR}/walk_model.bin")

add_custom_command(
	OUTPUT ${RLWALK_IR_XML} ${RLWALK_IR_BIN}
	COMMAND ${Python3_EXECUTABLE} -m openvino.tools.ovc ${RLWALK_ONNX_MODEL} --output_model ${RLWALK_IR_XML}
	DEPENDS ${RLWALK_ONNX_MODEL}
	COMMENT "Converting RLWalk ONNX model to OpenVINO IR"
	VERBATIM
)

# Build our NUClear module
nuclear_module(SOURCES ${RLWALK_IR_XML} ${RLWALK_IR_BIN} LIBRARIES TBB::tbb openvino::runtime ${OpenVINO_LIBRARIES})
