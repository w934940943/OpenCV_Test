cmake_minimum_required(VERSION 3.12.2)

project(sample_test)

set(CMAKE_CONFIGURATION_TYPES "Debug;Release"
    CACHE STRING "Configuration types" FORCE)

add_executable(sample_test
  hist.cpp
  kmeans.cpp
  mouse_callback.cpp
  test.cpp
  test2.cpp
  test2.h
  threshold.cpp
)
# Preprocessor definitions
target_compile_definitions(sample_test PRIVATE
  $<$<CONFIG:Debug>:_UNICODE;_DEBUG;_CONSOLE>
  $<$<CONFIG:Release>:_UNICODE;NDEBUG;_CONSOLE>
)

# Minimal rebuild
if (MSVC)
  target_compile_options(sample_test PRIVATE
    "$<$<CONFIG:Debug>:/Gm->"
    "$<$<CONFIG:Release>:/Gm->"
  )
endif ()

# Precompiled header files
if (MSVC)
  target_compile_options(sample_test PRIVATE
    ""
  )
endif ()
