include(FetchContent)

# ---------------------------------------------------------------------------
# MuJoCo: the official prebuilt release for this platform, downloaded at configure time and checked
# against its SHA-256. To try a different version, change MUJOCO_VERSION and the hashes below
# (each release lists them on its GitHub page).
# ---------------------------------------------------------------------------
set(MUJOCO_VERSION 4.0.1)
set(MUJOCO_RELEASE_URL https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION})

if(WIN32 AND CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(MUJOCO_ARCHIVE mujoco-${MUJOCO_VERSION}-windows-x86_64.zip)
  set(MUJOCO_SHA256 abe3104a2ae0ca18a4b50fa1e79aaf57699496c7ee78ec7214104a0d1593fc75)
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
  set(MUJOCO_ARCHIVE mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz)
  set(MUJOCO_SHA256 8df1308b2900a4ed6c18717ebbc9d66d53ab3fe7a4da860a7d5b21f94607299d)
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  set(MUJOCO_ARCHIVE mujoco-${MUJOCO_VERSION}-linux-aarch64.tar.gz)
  set(MUJOCO_SHA256 e16dc0ed24d31279bce674bd03b9f8ef75f209d1189befe67b7c3b366978a6b3)
else()
  message(FATAL_ERROR "No prebuilt MuJoCo ${MUJOCO_VERSION} for ${CMAKE_SYSTEM_NAME}/${CMAKE_SYSTEM_PROCESSOR}")
endif()

FetchContent_Declare(mujoco_bin
  URL ${MUJOCO_RELEASE_URL}/${MUJOCO_ARCHIVE}
  URL_HASH SHA256=${MUJOCO_SHA256}
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(mujoco_bin)
set(MUJOCO_ROOT ${mujoco_bin_SOURCE_DIR})

add_library(mujoco::mujoco SHARED IMPORTED GLOBAL)
set_target_properties(mujoco::mujoco PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${MUJOCO_ROOT}/include)
if(WIN32)
  set_target_properties(mujoco::mujoco PROPERTIES
    IMPORTED_IMPLIB ${MUJOCO_ROOT}/lib/mujoco.lib
    IMPORTED_LOCATION ${MUJOCO_ROOT}/bin/mujoco.dll
  )
else()
  file(GLOB MUJOCO_SO ${MUJOCO_ROOT}/lib/libmujoco.so.*)
  list(GET MUJOCO_SO 0 MUJOCO_SO)
  set_target_properties(mujoco::mujoco PROPERTIES IMPORTED_LOCATION ${MUJOCO_SO})
endif()

# ---------------------------------------------------------------------------
# GLFW for the viewer window (the version MuJoCo's own samples pin), linked statically
# ---------------------------------------------------------------------------
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
FetchContent_Declare(glfw
  GIT_REPOSITORY https://github.com/glfw/glfw.git
  GIT_TAG 7b6aead9fb88b3623e3b3725ebb42670cbe4c579 # 3.4
)
FetchContent_MakeAvailable(glfw)

# ---------------------------------------------------------------------------
# draxi_sim
# ---------------------------------------------------------------------------
add_executable(draxi_sim main.cpp)
target_compile_features(draxi_sim PRIVATE cxx_std_17)
target_link_libraries(draxi_sim PRIVATE mujoco::mujoco glfw gait_core::gait_core)
target_compile_definitions(draxi_sim PRIVATE DRAXI_DEFAULT_MODEL="${CMAKE_SOURCE_DIR}/cad/draxi.xml")
if(NOT WIN32)
  set_target_properties(draxi_sim PROPERTIES BUILD_RPATH ${MUJOCO_ROOT}/lib)
endif()

# Put mujoco.dll next to the exe so it runs straight from the build folder
if(WIN32)
  add_custom_command(TARGET draxi_sim POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_RUNTIME_DLLS:draxi_sim> $<TARGET_FILE_DIR:draxi_sim>
    COMMAND_EXPAND_LISTS
  )
endif()
