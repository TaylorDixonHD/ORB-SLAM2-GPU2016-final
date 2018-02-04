find_package(PCL 1.7 REQUIRED)
find_package(Threads REQUIRED)

include_directories(
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(tx1 tx1.cpp)
target_link_libraries(tx1 ${PROJECT_NAME} ${PCL_LIBRARIES})
