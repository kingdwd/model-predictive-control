CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/cpack.cmake)
INCLUDE(cmake/eigen.cmake)

SET(PROJECT_NAME model-predictive-control)
SET(PROJECT_DESCRIPTION "MPC")
SET(PROJECT_URL "")
SET(PROJECT_DEBUG_POSTFIX "_d")

SET(CXX_DISABLE_WERROR 1)
SETUP_PROJECT()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -pedantic")
#########################
# External dependencies
SEARCH_FOR_EIGEN()
SET(BOOST_REQUIRED 1.48)
SET(BOOST_COMPONENTS system unit_test_framework)

ADD_REQUIRED_DEPENDENCY("yaml-cpp >= 0.5")
ADD_REQUIRED_DEPENDENCY(EigenQP EigenQP_d)

########################

ADD_DEFINITIONS(-DLOGS_DATA_DIR="${PROJECT_SOURCE_DIR}/logs")

ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(tests)

# Add dependency towards the library in the pkg-config file.
PKG_CONFIG_APPEND_LIBS(model-predictive-control)

SETUP_PROJECT_FINALIZE()
#SETUP_PROJECT_CPACK()
