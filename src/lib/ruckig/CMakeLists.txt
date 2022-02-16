cmake_minimum_required(VERSION 3.10)


project(ruckig VERSION 0.6.0 LANGUAGES CXX)


list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(GNUInstallDirs)

option(BUILD_SHARED_LIBS "Build as shared library" ON)
option(EXPOSE_INTERNAL "Expose some internal classes and methods" OFF)

if (WIN32 AND BUILD_SHARED_LIBS)
  option(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS "On Windows, export all symbols when building a shared library." ON)
endif ()


find_package(Reflexxes QUIET)


add_library(rotools_ruckig
  src/brake.cpp
  src/position-step1.cpp
  src/position-step2.cpp
  src/velocity-step1.cpp
  src/velocity-step2.cpp
  )
add_library(rotools_ruckig::ruckig ALIAS rotools_ruckig)

target_compile_features(rotools_ruckig PUBLIC cxx_std_17)
target_include_directories(rotools_ruckig PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  )
target_link_libraries(rotools_ruckig PUBLIC)


if (MSVC)
  target_compile_definitions(rotools_ruckig PUBLIC _USE_MATH_DEFINES)
  target_compile_options(rotools_ruckig PRIVATE /W4 /WX)

else ()
  target_compile_options(rotools_ruckig PRIVATE -Werror -Wall -Wextra)
endif ()

if (EXPOSE_INTERNAL)
  target_compile_definitions(rotools_ruckig PUBLIC WITH_EXPOSE_INTERNAL)
endif ()

if (Reflexxes)
  set(REFLEXXES_TYPE "ReflexxesTypeII" CACHE STRING "Type of Reflexxes library") # or ReflexxesTypeIV
  message("Found Reflexxes ${REFLEXXES_TYPE}")

  target_compile_definitions(ruckig PUBLIC WITH_REFLEXXES)
  target_link_libraries(ruckig PUBLIC Reflexxes::Reflexxes)
endif ()
