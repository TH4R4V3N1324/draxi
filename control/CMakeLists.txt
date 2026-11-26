# The robot's controllers and the HAL they talk through. No MuJoCo, no heap: the same code runs in
# the sim and on the Teensy.
add_library(draxi_control STATIC
  src/stand_controller.cpp
)
add_library(draxi::control ALIAS draxi_control)
target_include_directories(draxi_control PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(draxi_control PUBLIC draxi::shared)
target_compile_features(draxi_control PUBLIC cxx_std_17)

if(MSVC)
  target_compile_options(draxi_control PRIVATE /W4)
else()
  target_compile_options(draxi_control PRIVATE -Wall -Wextra -Wpedantic -Wdouble-promotion)
endif()
