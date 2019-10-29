project(cartesio_planning_examples)
cmake_minimum_required(VERSION 3.0)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY configs/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/configs
)

install(DIRECTORY utils/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/utils
)
