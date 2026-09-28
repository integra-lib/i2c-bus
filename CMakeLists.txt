cmake_minimum_required(VERSION 3.21)

project(
  hwlib_i2c_bus
  VERSION 0.1.0
  LANGUAGES CXX)

add_library(hwlib_i2c_bus INTERFACE)
add_library(Hwlib::i2c_bus ALIAS hwlib_i2c_bus)

target_include_directories(hwlib_i2c_bus
                           INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(hwlib_i2c_bus INTERFACE cxx_std_20)

# Dependants read the version from this property rather than from a variable: a
# variable set here is invisible in a sibling directory, and a stale one left in
# the parent scope would be read instead — a version check that silently passes.
set_target_properties(hwlib_i2c_bus PROPERTIES HWLIB_VERSION ${PROJECT_VERSION})

if(PROJECT_IS_TOP_LEVEL)
  include(FetchContent)
  FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.15.2)
  FetchContent_MakeAvailable(googletest)

  include(GoogleTest)
  enable_testing()

  add_executable(i2c-bus-test tests/i2c_bus_test.cpp)
  target_link_libraries(i2c-bus-test PRIVATE Hwlib::i2c_bus GTest::gtest_main)
  gtest_discover_tests(i2c-bus-test)
endif()
