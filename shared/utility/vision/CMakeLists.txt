SET(dependencies
	ARMADILLO
    Aravis
    glib2
	NUbots_PROJECT
	)

ADD_UTILITY_LIBRARY(${CMAKE_CURRENT_SOURCE_DIR} ${dependencies})

