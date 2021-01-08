#
#   This file is part of Magnum.
#
#   Original authors — credit is appreciated but not required:
#
#       2010, 2011, 2012, 2013, 2014, 2015, 2016, 2017, 2018, 2019, 2020 —
#           Vladimír Vondruš <mosra@centrum.cz>
#
#   This is free and unencumbered software released into the public domain.
#
#   Anyone is free to copy, modify, publish, use, compile, sell, or distribute
#   this software, either in source code form or as a compiled binary, for any
#   purpose, commercial or non-commercial, and by any means.
#
#   In jurisdictions that recognize copyright laws, the author or authors of
#   this software dedicate any and all copyright interest in the software to
#   the public domain. We make this dedication for the benefit of the public
#   at large and to the detriment of our heirs and successors. We intend this
#   dedication to be an overt act of relinquishment in perpetuity of all
#   present and future rights to this software under copyright law.
#
#   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
#   THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
#   IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
#   CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#

cmake_minimum_required(VERSION 3.4)

project(MagnumExamples CXX)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/modules/" ${CMAKE_MODULE_PATH})
find_package(Magnum REQUIRED)

include(CMakeDependentOption)
option(WITH_ARCBALL_EXAMPLE "Build ArcBall Camera example" ON)
option(WITH_ANIMATED_GIF_EXAMPLE "Build Animated Gif example" ON)
option(WITH_AREALIGHTS_EXAMPLE "Build Area Lights example (requires the Ui library and some TTF font plugin)" ON)
option(WITH_AUDIO_EXAMPLE "Build Audio example (requires the Audio library and the StbVorbisAudioImporter plugin)" ON)
option(WITH_BOX2D_EXAMPLE "Build Box2D integration example" ON)
option(WITH_BULLET_EXAMPLE "Build Bullet integration example (requires the BulletIntegration library)" ON)
cmake_dependent_option(WITH_CUBEMAP_EXAMPLE "Build CubeMap example (requires some JPEG importer plugin)" ON "NOT MAGNUM_TARGET_GLES" ON)
option(WITH_DART_EXAMPLE "Build DART integration example (requires the DartIntegration library)" OFF)
option(WITH_FLUIDSIMULATION2D_EXAMPLE "Build 2D Fluid Simulation example (requires the ImGui integration)" ON)
option(WITH_FLUIDSIMULATION3D_EXAMPLE "Build 3D Fluid Simulation example (requires the ImGui integration)" ON)
option(WITH_IMGUI_EXAMPLE "Build ImGui example" ON)
option(WITH_LEAPMOTION_EXAMPLE "Build LeapMotion example" OFF)
option(WITH_MOUSEINTERACTION_EXAMPLE "Build Mouse Interaction example" ON)
cmake_dependent_option(WITH_MOTIONBLUR_EXAMPLE "Build Motion Blur example" ON "NOT MAGNUM_TARGET_GLES" ON)
option(WITH_OCTREE_EXAMPLE "Build Octree example" OFF)
option(WITH_OVR_EXAMPLE "Build OVR example" OFF)
option(WITH_PICKING_EXAMPLE "Build Picking example" ON)
option(WITH_PRIMITIVES_EXAMPLE "Build Primitives example" ON)
option(WITH_RAYTRACING_EXAMPLE "Build Ray Tracing example" ON)
option(WITH_SHADOWS_EXAMPLE "Build Shadow Mapping example" ON)
option(WITH_TEXT_EXAMPLE "Build Text example (requires some TTF font plugin)" ON)
cmake_dependent_option(WITH_TEXTUREDTRIANGLE_EXAMPLE "Build TexturedTriangle example (requires some TGA importer plugin)" ON "NOT MAGNUM_TARGET_GLES" ON)
option(WITH_TRIANGLE_EXAMPLE "Build Triangle example" ON)
option(WITH_TRIANGLE_PLAIN_GLFW_EXAMPLE "Build Plain GLFW Triangle example" ON)
option(WITH_TRIANGLE_SOKOL_EXAMPLE "Build sokol_gfx Triangle example" ON)
option(WITH_TRIANGLE_VULKAN_EXAMPLE "Build Vulkan Triangle example" OFF)
option(WITH_VIEWER_EXAMPLE "Build Viewer example (requires the AnySceneImporter plugin)" OFF)
if(CORRADE_TARGET_EMSCRIPTEN)
    message("CORRADE_TARGET_EMSCRIPTEN enabled")
    option(WITH_WEBXR_EXAMPLE "Build WebXR example" OFF)
endif()

# A single output location. After a decade of saying NO THIS IS A NON-SOLUTION
# TO A NON-PROBLEM I reconsidered my views and enabled this, because:
#
# - On Windows (which don't have RPATH), this makes test execution finally
#   possible without having to install all the stuff first (including the
#   test-only libs, which is ugh).
# - With CMake subprojects, this makes it finally possible to use dynamic
#   plugins directly from the build dir (again without installing anything) ---
#   all plugins are put into the same place, so PluginManager has a single
#   place to look into; and thanks to the dynamic libraries being there as
#   well, this location can be automagically detected as relative to
#   Directory::libraryLocation().
# - Thanks to the $<CONFIG> being part of the output path, you are always sure
#   you never accidentally mix up debug/release libraries when switching
#   CMAKE_BUILD_TYPE in an existing build dir.
#
# The runtime location is set to CMAKE_BINARY_DIR and not PROJECT_BINARY_DIR
# because have one runtime location per CMake subproject would not solve much
# either. If the user already provides CMAKE_RUNTIME_OUTPUT_DIRECTORY (even
# empty), it's respected and nothing is being done.
#
# Explicitly using a generator expression to ensure plugins are added to e.g.
# <CONFIG>/lib/magnum/importers/ instead of lib/magnum/importers/<CONFIG>. Also
# adding this to cache, making superprojects pick that up implicitly as well,
# without forcing them to explicitly mirror this setting.
if(NOT DEFINED CMAKE_RUNTIME_OUTPUT_DIRECTORY AND NOT DEFINED CMAKE_LIBRARY_OUTPUT_DIRECTORY AND NOT DEFINED CMAKE_ARCHIVE_OUTPUT_DIRECTORY)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/$<CONFIG>/bin CACHE PATH "" FORCE)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/$<CONFIG>/lib CACHE PATH "" FORCE)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/$<CONFIG>/lib CACHE PATH "" FORCE)
endif()

set(BASIS_UNIVERSAL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/basis-universal)
set(BUILD_PLUGINS_STATIC ON CACHE BOOL "" FORCE)
set(WITH_STBIMAGEIMPORTER ON CACHE BOOL "" FORCE) # enable what you need
set(WITH_STBIMAGECONVERTER ON CACHE BOOL "" FORCE) # enable what you need
set(WITH_STLIMPORTER ON CACHE BOOL "" FORCE) # enable what you need
set(WITH_ASSIMPIMPORTER ON CACHE BOOL "" FORCE) # enable what you need
set(WITH_ANYAUDIOIMPORTER ON CACHE BOOL "" FORCE)
set(WITH_ANYIMAGEIMPORTER ON CACHE BOOL "" FORCE)
set(WITH_ANYSCENECONVERTER ON CACHE BOOL "" FORCE)
set(WITH_ANYSCENEIMPORTER ON CACHE BOOL "" FORCE)
set(WITH_OBJIMPORTER ON CACHE BOOL "" FORCE)
set(WITH_MAGNUMFONT ON CACHE BOOL "" FORCE)
set(WITH_TGAIMPORTER ON CACHE BOOL "" FORCE)
set(WITH_TGAIMAGECONVERTER ON CACHE BOOL "" FORCE)
set(WITH_DDSIMPORTER ON CACHE BOOL "" FORCE)
set(WITH_PNGIMPORTER ON CACHE BOOL "" FORCE)
set(WITH_PLAYER ON CACHE BOOL "" FORCE)
set(WITH_JPEGIMPORTER ON CACHE BOOL "" FORCE)
set(WITH_BASISIMPORTER ON CACHE BOOL "" FORCE)
set(WITH_STBIMAGEIMPORTER ON CACHE BOOL "" FORCE)
set(WITH_STBTRUETYPEFONT ON CACHE BOOL "" FORCE)
set(WITH_TINYGLTFIMPORTER ON CACHE BOOL "" FORCE)
set(WITH_CUBEMAP_EXAMPLE ON CACHE BOOL "" FORCE)
set(WITH_SHADOWS_EXAMPLE ON CACHE BOOL "" FORCE)
# Disable Assimp tests, tools and exporter functionality
set(ASSIMP_BUILD_ASSIMP_TOOLS OFF CACHE BOOL "" FORCE)
set(ASSIMP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(ASSIMP_NO_EXPORT ON CACHE BOOL "" FORCE)
# If you won't be accessing Assimp outside of the plugin, build it as static to
# have the plugin binary self-contained
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
# The following is important to avoid Assimp appending `d` to all your
# binaries. You need Assimp >= 5.0.0 for this to work, also note that after
# 5.0.1 this option is prefixed with ASSIMP_, so better set both variants.
set(INJECT_DEBUG_POSTFIX OFF CACHE BOOL "" FORCE)
set(ASSIMP_INJECT_DEBUG_POSTFIX OFF CACHE BOOL "" FORCE)
set(WITH_UI ON CACHE BOOL "" FORCE)
add_subdirectory(magnum)
add_subdirectory(magnum-integration)
add_subdirectory(magnum-plugins)
add_subdirectory(magnum-extras)

add_subdirectory(src)
