cmake_minimum_required(VERSION 2.8.12)
project(Latency_test)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
###
# Configuration Options
###

###
# CMake Modules
###
include(EnableExtraCompilerWarnings)

include(SetDefaultBuildType)
set_default_build_type(RelWithDebInfo)

include(MSVCMultipleProcessCompile)
include(CopyImportedTarget)

include(InstallRequiredSystemLibraries)
if(WIN32)
    # Flat install for docs on Windows.
    set(CMAKE_INSTALL_DOCDIR .)
endif()
include(GNUInstallDirs)
if(NOT CMAKE_INSTALL_DOCDIR)
    set(CMAKE_INSTALL_DOCDIR ${CMAKE_INSTALL_DATAROOTDIR}/doc/${PROJECT_NAME})
endif()

###
# Dependencies
###
# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
list(APPEND CMAKE_PREFIX_PATH "C:\\Qt\\5.4\\msvc2013_64_opengl\\")
set(CMAKE_AUTOMOC ON)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5OpenGL REQUIRED)
find_package(OpenGL REQUIRED)

if(MSVC)
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)
endif()

set(SOURCES
    main.cpp
    mainwindow.cpp
    mainwindow.h
    opengl_widget.cpp
    opengl_widget.h)
qt5_wrap_ui(UI_HEADERS mainwindow.ui)

add_executable(Latency_test ${SOURCES} ${UI_HEADERS})

target_link_libraries(Latency_test Qt5::Widgets Qt5::OpenGL ${OPENGL_LIBRARIES})
install(TARGETS Latency_test
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

set(deps Qt5::Core Qt5::Gui Qt5::Widgets Qt5::OpenGL)

#if(TARGET GLEW::GLEW_static)
#    target_link_libraries(Latency_test GLEW::GLEW_static)
#elseif(TARGET GLEW::GLEW)
#    target_link_libraries(Latency_test GLEW::GLEW)
#    list(APPEND deps GLEW::GLEW)
#endif()

foreach(dep ${deps})
    copy_imported_targets(Latency_test ${dep})
    install_imported_target(${dep} DESTINATION ${CMAKE_INSTALL_BINDIR})
endforeach()

if(WIN32)
    install_imported_target(Qt5::QWindowsIntegrationPlugin DESTINATION ${CMAKE_INSTALL_BINDIR}/platforms)
    list(APPEND deps Qt5::QWindowsIntegrationPlugin)
endif()

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/GetSearchDirs.cmake)

get_search_dirs(libSearchDirs ${deps})

set(APPS
    "\${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}/Latency_test${CMAKE_EXECUTABLE_SUFFIX}")
install(CODE "include(BundleUtilities)
    fixup_bundle(\"${APPS}\" \"\" \"${libSearchDirs}\")")
