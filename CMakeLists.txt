cmake_minimum_required(VERSION 3.0)
project(allolib)

macro(MACRO_ENSURE_OUT_OF_SOURCE_BUILD MSG)
     STRING(COMPARE EQUAL "${CMAKE_SOURCE_DIR}"
     "${CMAKE_BINARY_DIR}" insource)
     GET_FILENAME_COMPONENT(PARENTDIR ${CMAKE_SOURCE_DIR} PATH)
     STRING(COMPARE EQUAL "${CMAKE_SOURCE_DIR}"
     "${PARENTDIR}" insourcesubdir)
    IF(insource OR insourcesubdir)
        MESSAGE(FATAL_ERROR "${MSG}")
    ENDIF(insource OR insourcesubdir)
ENDMACRO(MACRO_ENSURE_OUT_OF_SOURCE_BUILD)

MACRO_ENSURE_OUT_OF_SOURCE_BUILD(
    "${CMAKE_PROJECT_NAME} requires an out of source build. Make a directory and run cmake there."
)

option(AL_VERBOSE_OUTPUT "" OFF)
option(ALLOLIB_BUILD_EXAMPLES "" OFF)
option(TRAVIS_BUILD "" OFF)
option(APPVEYOR_BUILD "" OFF)
option(ALLOLIB_BUILD_TESTS "" OFF)
option(ALLOLIB_USE_PORTAUDIO "Use PortAudio instead of RtAudio" OFF)
option(ALLOLIB_USE_DUMMY_AUDIO "Use Dummy Audio I/O" OFF)
option(ALLOLIB_BUILD_SHARED "Build all libraries as shared libraries" OFF)

option(CULT_ALLOLIB_ENABLE_AUDIO "Build cult-allolib audio I/O support" ON)
option(CULT_ALLOLIB_ENABLE_OSC "Build cult-allolib OSC protocol support" ON)
option(CULT_ALLOLIB_ENABLE_PARAMETERS "Build cult-allolib parameter support" ON)
option(CULT_ALLOLIB_ENABLE_SPATIAL "Build cult-allolib spatialization support" ON)
option(CULT_ALLOLIB_ENABLE_GRAPHICS "Build cult-allolib OpenGL graphics support" OFF)
option(CULT_ALLOLIB_ENABLE_APP "Build cult-allolib app/domain infrastructure" OFF)
option(CULT_ALLOLIB_ENABLE_VR "Build cult-allolib VR/omni renderer support" OFF)
option(CULT_ALLOLIB_ENABLE_MIDI "Build cult-allolib MIDI support" OFF)
option(CULT_ALLOLIB_ENABLE_SERIAL "Build cult-allolib serial I/O support" OFF)
option(CULT_ALLOLIB_ENABLE_FILE "Build cult-allolib file/path utilities" ON)
option(CULT_ALLOLIB_ENABLE_TIMING "Build cult-allolib time/thread utilities" ON)
option(CULT_ALLOLIB_ENABLE_TYPES "Build cult-allolib runtime type utilities" ON)
option(CULT_ALLOLIB_ENABLE_EXAMPLES "Build cult-allolib examples" OFF)
option(CULT_ALLOLIB_ENABLE_TESTS "Build cult-allolib tests" OFF)
option(CULT_ALLOLIB_ENABLE_DOCS "Build cult-allolib docs/helpful extras" OFF)

set(ALLOLIB_BUILD_EXAMPLES ${CULT_ALLOLIB_ENABLE_EXAMPLES} CACHE BOOL "" FORCE)
set(ALLOLIB_BUILD_TESTS ${CULT_ALLOLIB_ENABLE_TESTS} CACHE BOOL "" FORCE)

if(CULT_ALLOLIB_ENABLE_PARAMETERS AND NOT CULT_ALLOLIB_ENABLE_OSC)
  message(FATAL_ERROR "CULT_ALLOLIB_ENABLE_PARAMETERS requires CULT_ALLOLIB_ENABLE_OSC=ON")
endif()

if(CULT_ALLOLIB_ENABLE_PARAMETERS)
  set(CULT_ALLOLIB_ENABLE_FILE ON CACHE BOOL "" FORCE)
  set(CULT_ALLOLIB_ENABLE_TIMING ON CACHE BOOL "" FORCE)
  set(CULT_ALLOLIB_ENABLE_TYPES ON CACHE BOOL "" FORCE)
endif()

if(CULT_ALLOLIB_ENABLE_SPATIAL)
  set(CULT_ALLOLIB_ENABLE_TYPES ON CACHE BOOL "" FORCE)
endif()

if(CULT_ALLOLIB_ENABLE_OSC)
  set(CULT_ALLOLIB_ENABLE_TIMING ON CACHE BOOL "" FORCE)
endif()

if(CULT_ALLOLIB_ENABLE_GRAPHICS)
  set(CULT_ALLOLIB_ENABLE_APP ON CACHE BOOL "" FORCE)
endif()

if(CULT_ALLOLIB_ENABLE_VR)
  set(CULT_ALLOLIB_ENABLE_APP ON CACHE BOOL "" FORCE)
  set(CULT_ALLOLIB_ENABLE_GRAPHICS ON CACHE BOOL "" FORCE)
endif()

if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  set(AL_MACOS 1 CACHE BOOL "Building on OS X")
  set(PLATFORM_DEFINITION -DAL_OSX)
elseif (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  set(AL_LINUX 1 CACHE BOOL "Building on Linux")
  set(PLATFORM_DEFINITION -DAL_LINUX)
elseif (${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  set(AL_WINDOWS 1 CACHE BOOL "Building on Windows")
  set(PLATFORM_DEFINITION -DAL_WINDOWS)
  if (NOT ${CMAKE_GENERATOR_PLATFORM} STREQUAL "x64")
    message("Warning! Not building for Win64 : ${CMAKE_GENERATOR_PLATFORM}")
  endif()
else ()
  message(FATAL_ERROR "system platform not defined")
endif ()

if(TRAVIS_BUILD)
  add_definitions(-DTRAVIS_BUILD)
endif()

if(ALLOLIB_BUILD_SHARED)
  set(AL_LIBRARY_TYPE "SHARED")
else()
  set(AL_LIBRARY_TYPE "STATIC")
endif()

if(CULT_ALLOLIB_ENABLE_GRAPHICS)
  find_package(OpenGL REQUIRED)
endif()
find_package(Threads REQUIRED)

add_subdirectory(external)

set(cult_allolib_public_include_dirs
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/external/Gamma
  ${CMAKE_CURRENT_SOURCE_DIR}/external/json/include
)

set(cult_allolib_platform_compile_defs ${PLATFORM_DEFINITION})

if(SNDFILE_LIBRARY)
  list(APPEND cult_allolib_platform_compile_defs AL_LIBSNDFILE)
endif()

function(cult_allolib_apply_common_settings target_name)
  set_target_properties(${target_name} PROPERTIES
    DEBUG_POSTFIX d
    CXX_STANDARD 14
    CXX_STANDARD_REQUIRED ON
  )

  if (AL_WINDOWS)
    target_compile_definitions(${target_name} PUBLIC NOMINMAX)
    target_compile_options(${target_name} PUBLIC /wd4221 /wd4224 /wd4244 /wd4305 /wd4267 /wd4005 /wd4996)
    set_target_properties(${target_name} PROPERTIES LINK_FLAGS "/ignore:4099")
  else()
    target_compile_options(${target_name} PRIVATE "-Wall")
  endif()
endfunction()

add_library(cult_allolib_core ${AL_LIBRARY_TYPE}
  include/al/math/al_Constants.hpp
  include/al/math/al_Mat.hpp
  include/al/math/al_Matrix4.hpp
  include/al/math/al_Quat.hpp
  include/al/math/al_Vec.hpp
  include/al/spatial/al_Curve.hpp
  include/al/spatial/al_Pose.hpp
  include/al/system/al_Printing.hpp
  include/al/system/al_Thread.hpp
  include/al/system/al_Time.hpp
  include/al/types/al_Color.hpp
  include/al/types/al_ValueSource.hpp
  include/al/types/al_VariantValue.hpp
  src/spatial/al_Pose.cpp
  src/system/al_Printing.cpp
  src/system/al_ThreadNative.cpp
  src/system/al_Time.cpp
  src/types/al_Color.cpp
  src/types/al_VariantValue.cpp
)
target_include_directories(cult_allolib_core PUBLIC ${cult_allolib_public_include_dirs})
target_compile_definitions(cult_allolib_core PUBLIC ${cult_allolib_platform_compile_defs})
target_link_libraries(cult_allolib_core PUBLIC Gamma Threads::Threads)
cult_allolib_apply_common_settings(cult_allolib_core)

if(CULT_ALLOLIB_ENABLE_FILE)
  add_library(cult_allolib_file ${AL_LIBRARY_TYPE}
    include/al/io/al_File.hpp
    src/io/al_File.cpp
    src/io/minFileSys.hpp
  )
  target_include_directories(cult_allolib_file PUBLIC ${cult_allolib_public_include_dirs})
  target_compile_definitions(cult_allolib_file PUBLIC ${cult_allolib_platform_compile_defs})
  target_link_libraries(cult_allolib_file PUBLIC cult_allolib_core)
  cult_allolib_apply_common_settings(cult_allolib_file)
endif()

if(CULT_ALLOLIB_ENABLE_AUDIO)
  add_library(cult_allolib_audio ${AL_LIBRARY_TYPE}
    include/al/io/al_AudioIO.hpp
    include/al/io/al_AudioIOData.hpp
    src/io/al_AudioIO.cpp
    src/io/al_AudioIOData.cpp
  )
  target_include_directories(cult_allolib_audio PUBLIC
    ${cult_allolib_public_include_dirs}
    ${CMAKE_CURRENT_SOURCE_DIR}/external/rtaudio
  )
  target_link_libraries(cult_allolib_audio PUBLIC cult_allolib_core)
  if (ALLOLIB_USE_PORTAUDIO)
    if(ALLOLIB_USE_DUMMY_AUDIO)
      message("ALLOLIB_USE_DUMMY_AUDIO ignored. Using PortAudio")
    endif()
    target_compile_definitions(cult_allolib_audio PUBLIC ${cult_allolib_platform_compile_defs} AL_AUDIO_PORTAUDIO)
    target_link_libraries(cult_allolib_audio PUBLIC portaudio)
  else()
    if (ALLOLIB_USE_DUMMY_AUDIO)
      target_compile_definitions(cult_allolib_audio PUBLIC ${cult_allolib_platform_compile_defs} AL_AUDIO_DUMMY)
    else()
      target_compile_definitions(cult_allolib_audio PUBLIC ${cult_allolib_platform_compile_defs} AL_AUDIO_RTAUDIO)
      target_link_libraries(cult_allolib_audio PUBLIC rtaudio)
    endif()
  endif()
  cult_allolib_apply_common_settings(cult_allolib_audio)
endif()

if(CULT_ALLOLIB_ENABLE_OSC)
  add_library(cult_allolib_osc ${AL_LIBRARY_TYPE}
    include/al/protocol/al_OSC.hpp
    src/protocol/al_OSC.cpp
  )
  target_include_directories(cult_allolib_osc PUBLIC ${cult_allolib_public_include_dirs})
  target_compile_definitions(cult_allolib_osc PUBLIC ${cult_allolib_platform_compile_defs})
  target_link_libraries(cult_allolib_osc PUBLIC cult_allolib_core oscpack)
  cult_allolib_apply_common_settings(cult_allolib_osc)
endif()

if(CULT_ALLOLIB_ENABLE_SPATIAL)
  add_library(cult_allolib_spatial ${AL_LIBRARY_TYPE}
    include/al/sound/al_Dbap.hpp
    include/al/sound/al_Lbap.hpp
    include/al/sound/al_Spatializer.hpp
    include/al/sound/al_Speaker.hpp
    include/al/sound/al_Vbap.hpp
    include/al/spatial/al_DistAtten.hpp
    src/sound/al_Dbap.cpp
    src/sound/al_Lbap.cpp
    src/sound/al_Spatializer.cpp
    src/sound/al_Speaker.cpp
    src/sound/al_Vbap.cpp
  )
  target_include_directories(cult_allolib_spatial PUBLIC ${cult_allolib_public_include_dirs})
  target_compile_definitions(cult_allolib_spatial PUBLIC ${cult_allolib_platform_compile_defs})
  target_link_libraries(cult_allolib_spatial PUBLIC cult_allolib_core)
  if(TARGET cult_allolib_audio)
    target_link_libraries(cult_allolib_spatial PUBLIC cult_allolib_audio)
  endif()
  cult_allolib_apply_common_settings(cult_allolib_spatial)
endif()

if(CULT_ALLOLIB_ENABLE_PARAMETERS)
  add_library(cult_allolib_params ${AL_LIBRARY_TYPE}
    include/al/io/al_Socket.hpp
    include/al/ui/al_Parameter.hpp
    include/al/ui/al_ParameterBundle.hpp
    include/al/ui/al_ParameterServer.hpp
    src/io/al_Socket.cpp
    src/ui/al_Parameter.cpp
    src/ui/al_ParameterBundle.cpp
    src/ui/al_ParameterServer.cpp
  )
  target_include_directories(cult_allolib_params PUBLIC ${cult_allolib_public_include_dirs})
  target_compile_definitions(cult_allolib_params PUBLIC ${cult_allolib_platform_compile_defs})
  target_link_libraries(cult_allolib_params
    PUBLIC
      cult_allolib_core
      cult_allolib_osc
      serial
  )
  if(TARGET cult_allolib_file)
    target_link_libraries(cult_allolib_params PUBLIC cult_allolib_file)
  endif()
  cult_allolib_apply_common_settings(cult_allolib_params)
endif()

if(CULT_ALLOLIB_ENABLE_GRAPHICS)
  add_library(cult_allolib_graphics ${AL_LIBRARY_TYPE}
    include/al/graphics/al_BufferObject.hpp
    include/al/graphics/al_DefaultShaders.hpp
    include/al/graphics/al_DefaultShaderString.hpp
    include/al/graphics/al_EasyFBO.hpp
    include/al/graphics/al_EasyVAO.hpp
    include/al/graphics/al_FBO.hpp
    include/al/graphics/al_Font.hpp
    include/al/graphics/al_GPUObject.hpp
    include/al/graphics/al_Graphics.hpp
    include/al/graphics/al_Image.hpp
    include/al/graphics/al_Isosurface.hpp
    include/al/graphics/al_Lens.hpp
    include/al/graphics/al_Light.hpp
    include/al/graphics/al_Mesh.hpp
    include/al/graphics/al_OpenGL.hpp
    include/al/graphics/al_RenderManager.hpp
    include/al/graphics/al_Shader.hpp
    include/al/graphics/al_Shapes.hpp
    include/al/graphics/al_Texture.hpp
    include/al/graphics/al_VAO.hpp
    include/al/graphics/al_VAOMesh.hpp
    include/al/graphics/al_Viewpoint.hpp
    include/al/io/al_Window.hpp
    src/graphics/al_BufferObject.cpp
    src/graphics/al_DefaultShaders.cpp
    src/graphics/al_DefaultShaderString.cpp
    src/graphics/al_EasyFBO.cpp
    src/graphics/al_EasyVAO.cpp
    src/graphics/al_FBO.cpp
    src/graphics/al_Font.cpp
    src/graphics/al_GPUObject.cpp
    src/graphics/al_Graphics.cpp
    src/graphics/al_Image.cpp
    src/graphics/al_Isosurface.cpp
    src/graphics/al_Lens.cpp
    src/graphics/al_Light.cpp
    src/graphics/al_Mesh.cpp
    src/graphics/al_OpenGL.cpp
    src/graphics/al_RenderManager.cpp
    src/graphics/al_Shader.cpp
    src/graphics/al_Shapes.cpp
    src/graphics/al_Texture.cpp
    src/graphics/al_VAO.cpp
    src/graphics/al_VAOMesh.cpp
    src/graphics/al_Viewpoint.cpp
    src/graphics/al_stb_font.cpp
    src/graphics/al_stb_image.cpp
    src/io/al_Window.cpp
    src/io/al_WindowGLFW.cpp
  )
  target_include_directories(cult_allolib_graphics PUBLIC
    ${cult_allolib_public_include_dirs}
    ${CMAKE_CURRENT_SOURCE_DIR}/external/glad/include
    ${CMAKE_CURRENT_SOURCE_DIR}/external/imgui
  )
  target_compile_definitions(cult_allolib_graphics PUBLIC ${cult_allolib_platform_compile_defs})
  target_link_libraries(cult_allolib_graphics
    PUBLIC
      cult_allolib_core
      ${OPENGL_gl_LIBRARY}
      glfw
      glad
      imgui
      stb
      dr_libs
  )
  cult_allolib_apply_common_settings(cult_allolib_graphics)
endif()

if(CULT_ALLOLIB_ENABLE_APP)
  add_library(cult_allolib_app ${AL_LIBRARY_TYPE}
    include/al/app/al_App.hpp
    include/al/app/al_AppRecorder.hpp
    include/al/app/al_AudioDomain.hpp
    include/al/app/al_ComputationDomain.hpp
    include/al/app/al_ConsoleDomain.hpp
    include/al/app/al_DistributedApp.hpp
    include/al/app/al_FPS.hpp
    include/al/app/al_GUIDomain.hpp
    include/al/app/al_NodeConfiguration.hpp
    include/al/app/al_OmniRendererDomain.hpp
    include/al/app/al_OpenGLGraphicsDomain.hpp
    include/al/app/al_OSCDomain.hpp
    include/al/app/al_SimulationDomain.hpp
    include/al/app/al_StateDistributionDomain.hpp
    src/app/al_App.cpp
    src/app/al_AppRecorder.cpp
    src/app/al_AudioDomain.cpp
    src/app/al_ComputationDomain.cpp
    src/app/al_DistributedApp.cpp
    src/app/al_FPS.cpp
    src/app/al_GUIDomain.cpp
    src/app/al_OmniRendererDomain.cpp
    src/app/al_OpenGLGraphicsDomain.cpp
    src/app/al_OSCDomain.cpp
    src/app/al_SimulationDomain.cpp
    src/app/al_StateDistributionDomain.cpp
  )
  target_include_directories(cult_allolib_app PUBLIC ${cult_allolib_public_include_dirs})
  target_compile_definitions(cult_allolib_app PUBLIC ${cult_allolib_platform_compile_defs})
  target_link_libraries(cult_allolib_app
    PUBLIC
      cult_allolib_core
      cult_allolib_graphics
  )
  if(TARGET cult_allolib_audio)
    target_link_libraries(cult_allolib_app PUBLIC cult_allolib_audio)
  endif()
  if(TARGET cult_allolib_osc)
    target_link_libraries(cult_allolib_app PUBLIC cult_allolib_osc)
  endif()
  if(TARGET cult_allolib_params)
    target_link_libraries(cult_allolib_app PUBLIC cult_allolib_params)
  endif()
  cult_allolib_apply_common_settings(cult_allolib_app)
endif()

add_library(al INTERFACE)
target_include_directories(al INTERFACE ${cult_allolib_public_include_dirs})
target_compile_definitions(al INTERFACE ${cult_allolib_platform_compile_defs})
target_link_libraries(al INTERFACE cult_allolib_core)
if(TARGET cult_allolib_file)
  target_link_libraries(al INTERFACE cult_allolib_file)
endif()
if(TARGET cult_allolib_audio)
  target_link_libraries(al INTERFACE cult_allolib_audio)
endif()
if(TARGET cult_allolib_osc)
  target_link_libraries(al INTERFACE cult_allolib_osc)
endif()
if(TARGET cult_allolib_spatial)
  target_link_libraries(al INTERFACE cult_allolib_spatial)
endif()
if(TARGET cult_allolib_params)
  target_link_libraries(al INTERFACE cult_allolib_params)
endif()
if(TARGET cult_allolib_graphics)
  target_link_libraries(al INTERFACE cult_allolib_graphics)
endif()
if(TARGET cult_allolib_app)
  target_link_libraries(al INTERFACE cult_allolib_app)
endif()

add_library(cult_allolib_core_alias ALIAS cult_allolib_core)

if (ALLOLIB_BUILD_TESTS)
  message(FATAL_ERROR "cult-allolib tests were removed from this lean fork. Reintroduce the test tree before enabling ALLOLIB_BUILD_TESTS/CULT_ALLOLIB_ENABLE_TESTS.")
endif()

if (ALLOLIB_BUILD_EXAMPLES)
  message(FATAL_ERROR "cult-allolib examples were removed from this lean fork. Reintroduce the examples tree before enabling ALLOLIB_BUILD_EXAMPLES/CULT_ALLOLIB_ENABLE_EXAMPLES.")
endif()
