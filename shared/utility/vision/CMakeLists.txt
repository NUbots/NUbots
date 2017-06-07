SET(dependencies
	Eigen3
	NUbots_PROJECT
	)

ADD_UTILITY_LIBRARY(${CMAKE_CURRENT_SOURCE_DIR} ${dependencies})

