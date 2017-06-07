SET(dependencies
	Eigen3
	YAML-CPP
	muParserX
	NUBOTS_PROJECT
	)

ADD_UTILITY_LIBRARY(${CMAKE_CURRENT_SOURCE_DIR} ${dependencies})

