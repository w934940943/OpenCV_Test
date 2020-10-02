cmake_minimum_required(VERSION 3.12.2)

project(pupil_test)

set(CMAKE_CONFIGURATION_TYPES "Debug;Release"
    CACHE STRING "Configuration types" FORCE)

add_executable(pupil_test
  ../../../mylib/my_utilityfunctions.h
  DEPRECSTED.h
  PupilDetectorHaar.cpp
  PupilDetectorHaar.h
  test.cpp
)
# Preprocessor definitions
target_compile_definitions(pupil_test PRIVATE
  $<$<CONFIG:Debug>:_UNICODE;_DEBUG;_CONSOLE>
  $<$<CONFIG:Release>:_UNICODE;NDEBUG;_CONSOLE>
)

# SDL check
target_compile_options(pupil_test PRIVATE
  "$<$<CONFIG:Debug>:/sdl>"
  "$<$<CONFIG:Release>:/sdl>"
)

# Minimal rebuild
if (MSVC)
  target_compile_options(pupil_test PRIVATE
    "$<$<CONFIG:Debug>:/Gm->"
    "$<$<CONFIG:Release>:/Gm->"
  )
endif ()

# Precompiled header files
if (MSVC)
  target_compile_options(pupil_test PRIVATE
    ""
  )
endif ()
