add_library(utils stl_to_string.cpp clock.cpp config.cpp logging.cpp)
target_link_libraries(utils ${Boost_PROGRAM_OPTIONS_LIBRARY})

#install(TARGETS utils
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#  )
