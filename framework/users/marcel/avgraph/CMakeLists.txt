cmake_minimum_required(VERSION 2.6)

set(CMAKE_CXX_STANDARD 14)

set(HAS_LIBGG 0)
set(HAS_LIBOSC 0)
set(HAS_FRAMEWORK 0)

# --- dependencies ---

if (NOT HAS_LIBGG)
	message("including libgg")
	add_subdirectory(../../../libgg libgg)
endif (NOT HAS_LIBGG)
if (NOT HAS_LIBOSC)
	message("including libosc")
	add_subdirectory(../../../libosc libosc)
endif (NOT HAS_LIBOSC)
if (NOT HAS_FRAMEWORK)
	message("including framework")
	add_subdirectory(../framework framework)
endif (NOT HAS_FRAMEWORK)

#

project(avgraph)

# --- avgraph ---

file(GLOB_RECURSE source "*.c" "*.cpp" "*.h")
file(GLOB_RECURSE source_video "../avpaint/video.cpp" "../avpaint/video.h")
file(GLOB_RECURSE source_mediaplayer_new "../vfxpro/mediaplayer_new/*.*")
file(GLOB_RECURSE source_ffmpeg "../vfxpro/ffmpeg/*.*")

file(GLOB_RECURSE source_libfreenect "../../../libfreenect/src/*.*")

file(GLOB_RECURSE source_vfxnodes "vfxNodes/*.*")

file(GLOB_RECURSE source_ui "../libparticle/ui.cpp" "../libparticle/ui.h" "../libparticle/textfield.cpp" "../libparticle/textfield.h" "../libparticle/colorwheel.cpp" "../libparticle/colorwheel.h" "../libparticle/particle.cpp" "../libparticle/particle.h")

list(APPEND source ${source_video})
list(APPEND source ${source_mediaplayer_new})
list(APPEND source ${source_ffmpeg})
list(APPEND source ${source_ui})
list(APPEND source ${source_libfreenect})

source_group("Source Files" FILES ${source})
source_group("video" FILES ${source_video})
source_group("mediaplayer_new" FILES ${source_mediaplayer_new})
source_group("ffmpeg" FILES ${source_ffmpeg})
source_group("vfxNodes" FILES ${source_vfxnodes})
source_group("ui" FILES ${source_ui})
source_group("freenect" FILES ${source_libfreenect})

add_executable(avgraph ${source})

target_compile_definitions(avgraph PRIVATE NO_SDL_GLEXT HAVE_M_PI _USE_MATH_DEFINES _CRT_SECURE_NO_WARNINGS NOMINMAX)

target_include_directories(avgraph PRIVATE "${PROJECT_SOURCE_DIR}")
target_include_directories(avgraph PRIVATE "${PROJECT_SOURCE_DIR}/../../../libusb-1.0/include/libusb-01")
target_include_directories(avgraph PRIVATE "${PROJECT_SOURCE_DIR}/../../../libfreenect/src")

if (APPLE)
	target_include_directories(avgraph PRIVATE "../vfxpro/macos/include")
	target_include_directories(avgraph PRIVATE "../vfxpro/")
endif (APPLE)

if (WIN32)
	target_include_directories(avgraph PRIVATE "../vfxpro/win32/include")
	target_include_directories(avgraph PRIVATE "../vfxpro/")
endif (WIN32)

target_link_libraries(avgraph PRIVATE framework)
target_link_libraries(avgraph PRIVATE libosc)
target_link_libraries(avgraph PRIVATE libgg)

if (MSVC)
	target_link_libraries(avgraph PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../vfxpro/win32/lib/avcodec.lib)
	target_link_libraries(avgraph PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../vfxpro/win32/lib/avdevice.lib)
	target_link_libraries(avgraph PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../vfxpro/win32/lib/avfilter.lib)
	target_link_libraries(avgraph PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../vfxpro/win32/lib/avformat.lib)
	target_link_libraries(avgraph PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../vfxpro/win32/lib/avutil.lib)
	target_link_libraries(avgraph PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../vfxpro/win32/lib/swresample.lib)
	target_link_libraries(avgraph PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../vfxpro/win32/lib/swscale.lib)
	target_link_libraries(avgraph PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../vfxpro/win32/lib/Leap.lib)
	
	target_link_libraries(avgraph PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../framework/win32/lib/FreeImage.lib)
	target_link_libraries(avgraph PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../framework/win32/lib/freetype252.lib)
	target_link_libraries(avgraph PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../framework/win32/lib/glew32.lib)
	target_link_libraries(avgraph PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../framework/win32/lib/OpenAL32.lib)
	target_link_libraries(avgraph PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../framework/win32/lib/SDL2.lib)
	target_link_libraries(avgraph PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../framework/win32/lib/SDL2main.lib)
	target_link_libraries(avgraph PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../framework/win32/lib/XInput9_1_0.lib)
	target_link_libraries(avgraph PRIVATE Winmm.lib OpenGL32.lib WSock32.lib Ws2_32.lib)
endif (MSVC)

if (APPLE)
	target_compile_definitions(avgraph PUBLIC MACOS)

	find_library(COCOA_LIBRARY Cocoa)
	find_library(CARBON_LIBRARY Carbon)
	find_library(COREAUDIO_LIBRARY CoreAudio)
	find_library(COREVIDEO_LIBRARY CoreVideo)
	find_library(COREMEDIA_LIBRARY CoreMedia)
	find_library(IOKIT_LIBRARY IOKit)
	find_library(AUDIOTOOLBOX_LIBRARY AudioToolbox)
	find_library(AUDIOUNIT_LIBRARY AudioUnit)
	find_library(VIDEOTOOLBOX_LIBRARY VideoToolbox)
	find_library(VDA_LIBRARY VideoDecodeAcceleration)
	find_library(OPENAL_LIBRARY OpenAL)
	find_library(OPENGL_LIBRARY OpenGL)
	find_library(FF_LIBRARY ForceFeedback)
	find_library(ICONV_LIBRARY iconv)
	find_library(FREETYPE_LIBRARY freetype2)
	find_library(BZ2_LIBRARY bz2)
	find_library(SECURITY_LIBRARY Security)
	
	target_link_libraries(avgraph PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../../libusb-1.0/lib/osx/usb-1.0.a)

	target_link_libraries(avgraph PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../framework/macos/glew/lib/libGLEW.a)
	target_link_libraries(avgraph PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../framework/macos/lib/libfreeimage.a)
	target_link_libraries(avgraph PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../framework/macos/lib/libfreetype.a)
	target_link_libraries(avgraph PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../framework/macos/lib/libSDL2.a)
	target_link_libraries(avgraph PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../framework/macos/lib/libSDL2main.a)
	target_link_libraries(avgraph PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../vfxpro/macos/lib/libavcodec.a)
	target_link_libraries(avgraph PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../vfxpro/macos/lib/libavdevice.a)
	target_link_libraries(avgraph PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../vfxpro/macos/lib/libavfilter.a)
	target_link_libraries(avgraph PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../vfxpro/macos/lib/libavformat.a)
	target_link_libraries(avgraph PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../vfxpro/macos/lib/libavutil.a)
	target_link_libraries(avgraph PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../vfxpro/macos/lib/libswresample.a)
	target_link_libraries(avgraph PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../vfxpro/macos/lib/libswscale.a)
	target_link_libraries(avgraph PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../avpaint/macos/lib/libLeap.dylib)
	target_link_libraries(avgraph PRIVATE ${COCOA_LIBRARY})
	target_link_libraries(avgraph PRIVATE ${CARBON_LIBRARY})
	target_link_libraries(avgraph PRIVATE ${COREAUDIO_LIBRARY})
	target_link_libraries(avgraph PRIVATE ${COREVIDEO_LIBRARY})
	target_link_libraries(avgraph PRIVATE ${COREMEDIA_LIBRARY})
	target_link_libraries(avgraph PRIVATE ${IOKIT_LIBRARY})
	target_link_libraries(avgraph PRIVATE ${AUDIOTOOLBOX_LIBRARY})
	target_link_libraries(avgraph PRIVATE ${AUDIOUNIT_LIBRARY})
	target_link_libraries(avgraph PRIVATE ${VIDEOTOOLBOX_LIBRARY})
	target_link_libraries(avgraph PRIVATE ${VDA_LIBRARY})
	target_link_libraries(avgraph PRIVATE ${OPENAL_LIBRARY})
	target_link_libraries(avgraph PRIVATE ${OPENGL_LIBRARY})
	target_link_libraries(avgraph PRIVATE ${FF_LIBRARY})
	target_link_libraries(avgraph PRIVATE ${ICONV_LIBRARY})
	target_link_libraries(avgraph PRIVATE ${BZ2_LIBRARY})
	target_link_libraries(avgraph PRIVATE ${SECURITY_LIBRARY})
endif (APPLE)

if (MSVC)
	set_property(TARGET avgraph APPEND_STRING PROPERTY COMPILE_FLAGS /wd4244)
	set_property(TARGET avgraph APPEND_STRING PROPERTY LINK_FLAGS /SAFESEH:NO)
endif (MSVC)
