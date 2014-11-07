cmake_minimum_required(VERSION 2.6)

PROJECT(kuka_communication_tools)

execute_process(COMMAND /usr/xenomai/bin/xeno-config --skin=native --cflags OUTPUT_VARIABLE XENO_NATIVE_CFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND /usr/xenomai/bin/xeno-config --skin=native --ldflags OUTPUT_VARIABLE XENO_NATIVE_LDFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)


SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g -Wall -O3 ${XENO_NATIVE_CFLAGS} -DHAVE_SOCKLEN_T")
#SET(CMAKE_VERBOSE_MAKEFILE on )



find_package(Boost COMPONENTS system filesystem thread REQUIRED)

SET( FRI_REMOTE fri_remote fri_remote_rt)

INCLUDE_DIRECTORIES(include /usr/local/rtnet/include $ENV{LAB_ROOT}/include ${FRI_REMOTE})
LINK_DIRECTORIES(${PROJECT_NAME_BIN}/lib ${Boost_LIBRARY_DIR} $ENV{LAB_ROOT}/lib/x86_64xeno/)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

ADD_LIBRARY(${PROJECT_NAME}
  fri_remote/friremote.cpp  
  fri_remote/friudp.cpp  
)

ADD_LIBRARY(${PROJECT_NAME}_RT
  fri_remote_rt/friremote_rt.cpp  
  fri_remote_rt/friudp_rt.cpp  
)

ADD_EXECUTABLE(friFirstApp
  fri_firstapp/frifirstapp.cpp)
target_link_libraries(friFirstApp
  ${PROJECT_NAME}
  )

ADD_EXECUTABLE(friSecondApp
  fri_secondapp/fri_secondapp.cpp)
target_link_libraries(friSecondApp
  ${PROJECT_NAME}
)


ADD_EXECUTABLE(friTestRT
  fri_test_rt/fritest_rt.cpp)
target_link_libraries(friTestRT
  ${PROJECT_NAME}_RT
  ${XENO_NATIVE_LDFLAGS} -lrtdm
  ${Boost_LIBRARIES}
  )

ADD_EXECUTABLE(friBiManualRT
  fri_dual_rt/fridual_rt.cpp)
target_link_libraries(friBiManualRT
  ${PROJECT_NAME}_RT
  ${XENO_NATIVE_LDFLAGS} -lrtdm
  ${Boost_LIBRARIES}
  )

ADD_EXECUTABLE(friBiManualSeqRT
  fri_dual_rt/fridualseq_rt.cpp)
target_link_libraries(friBiManualSeqRT
  ${PROJECT_NAME}_RT
  ${XENO_NATIVE_LDFLAGS} -lrtdm
  ${Boost_LIBRARIES}
  )

ADD_EXECUTABLE(friFirstAppRT
  fri_firstapp_rt/frifirstapp_rt.cpp)
target_link_libraries(friFirstAppRT
  ${PROJECT_NAME}_RT
  ${XENO_NATIVE_LDFLAGS} -lrtdm
  ${Boost_LIBRARIES}
  )

ADD_EXECUTABLE(friSecondAppRT
  fri_secondapp_rt/frisecondapp_rt.cpp)
target_link_libraries(friSecondAppRT
  ${PROJECT_NAME}_RT
  ${XENO_NATIVE_LDFLAGS} -lrtdm
  ${Boost_LIBRARIES}
  )

ADD_EXECUTABLE(friGravCompRT
  fri_test_rt/fri_gravitycomp_rt.cpp)
target_link_libraries(friGravCompRT
  ${PROJECT_NAME}_RT
  ${XENO_NATIVE_LDFLAGS} -lrtdm
  ${Boost_LIBRARIES}
  )

# add a target to generate API documentation with Doxygen
# find_package(Doxygen)
# if(DOXYGEN_FOUND)
# add_custom_target(doc
# ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/doc/Doxyfile
# WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
# COMMENT "Generating API documentation with Doxygen" VERBATIM
# )
# endif(DOXYGEN_FOUND)
