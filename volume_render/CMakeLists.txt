cmake_minimum_required(VERSION 3.10)

PROJECT(final)
SET(VTK_DIR /Users/jordan/Desktop/cis410/build)
SET(CMAKE_VERBOSE_MAKEFILE ON)

find_package(VTK REQUIRED)

add_executable(final MACOSX_BUNDLE final)

SET(CMAKE_CXX_LINK_FLAGS "-framework OpenGL")
if(VTK_LIBRARIES)
  target_link_libraries(final ${VTK_LIBRARIES})
else()
  target_link_libraries(final vtkHybrid)
endif()

vtk_module_autoinit(
  TARGETS final
  MODULES ${VTK_LIBRARIES})