INCLUDE_DIRECTORIES(BEFORE ${Boost_INCLUDE_DIR})

SET(SOURCES
  Controller.cc
  Problem.cc
  logger.cc
  StepPlan.cc
  )
SET(HEADERS
  ../include/model-predictive-control/Controller.hh
  ../include/model-predictive-control/logger.hh
  ../include/model-predictive-control/Problem.hh
  ../include/model-predictive-control/Step.hh
  ../include/model-predictive-control/StepPlan.hh
  )

ADD_LIBRARY(model-predictive-control SHARED ${SOURCES} ${HEADERS})

PKG_CONFIG_USE_DEPENDENCY(model-predictive-control "yaml-cpp")

TARGET_LINK_LIBRARIES(model-predictive-control ${Boost_LIBRARIES})

SET(INSTALL_PATH include/model-predictive-control)
INSTALL(FILES ${HEADERS} DESTINATION ${INSTALL_PATH})
INSTALL(TARGETS model-predictive-control DESTINATION "lib")

