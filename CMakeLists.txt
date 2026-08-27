cmake_minimum_required(VERSION 3.21)
project(SSA VERSION 0.8.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT XPLANE_SDK_ROOT)
  message(FATAL_ERROR "Set XPLANE_SDK_ROOT to the extracted X-Plane SDK directory")
endif()

include(FetchContent)
FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz)
FetchContent_MakeAvailable(json)

add_library(SSA SHARED
  src/plugin.cpp
  src/datarefs.cpp
  src/scenery.cpp
  src/tablet.cpp
)

target_include_directories(SSA PRIVATE
  include
  "${XPLANE_SDK_ROOT}/CHeaders/XPLM"
  "${XPLANE_SDK_ROOT}/CHeaders/Widgets"
)
target_link_libraries(SSA PRIVATE nlohmann_json::nlohmann_json)
target_compile_definitions(SSA PRIVATE XPLM303 XPLM301 XPLM300 XPLM210 XPLM200)
set_target_properties(SSA PROPERTIES PREFIX "" SUFFIX ".xpl")

if(WIN32)
  # XPLM pulls in Windows headers. Prevent their min/max macros from replacing
  # calls such as std::min and std::max in portable C++ code.
  target_compile_definitions(SSA PRIVATE IBM=1 NOMINMAX WIN32_LEAN_AND_MEAN)
  target_link_libraries(SSA PRIVATE "${XPLANE_SDK_ROOT}/Libraries/Win/XPLM_64.lib")
elseif(APPLE)
  target_compile_definitions(SSA PRIVATE APL=1)
  target_link_options(SSA PRIVATE "-F${XPLANE_SDK_ROOT}/Libraries/Mac" "-framework" "XPLM")
elseif(UNIX)
  target_compile_definitions(SSA PRIVATE LIN=1)
  target_link_options(SSA PRIVATE "-Wl,--unresolved-symbols=ignore-all")
endif()

if(MSVC)
  target_compile_options(SSA PRIVATE /W4 /EHsc /utf-8)
else()
  target_compile_options(SSA PRIVATE -Wall -Wextra -Wpedantic)
endif()
