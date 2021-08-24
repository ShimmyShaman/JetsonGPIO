# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

#[=======================================================================[.rst:
FindJetsonGPIO
-------

Finds the JetsonGPIO library.

Imported Targets
^^^^^^^^^^^^^^^^

This module provides the following imported targets, if found:

``JetsonGPIO::JetsonGPIO``
  The JetsonGPIO library

Result Variables
^^^^^^^^^^^^^^^^

This will define the following variables:

``JetsonGPIO_FOUND``
  True if the system has the JetsonGPIO library.
``JetsonGPIO_VERSION``
  The version of the JetsonGPIO library which was found.
``JetsonGPIO_INCLUDE_DIRS``
  Include directories needed to use JetsonGPIO.
``JetsonGPIO_LIBRARIES``
  Libraries needed to link to JetsonGPIO.

Cache Variables
^^^^^^^^^^^^^^^

The following cache variables may also be set:

``JetsonGPIO_INCLUDE_DIR``
  The directory containing ``JetsonGPIO.h``.
``JetsonGPIO_LIBRARY``
  The path to the JetsonGPIO library.

#]=======================================================================]
find_package(PkgConfig)
pkg_check_modules(PC_JetsonGPIO QUIET JetsonGPIO)

find_path(JetsonGPIO_INCLUDE_DIR
  NAMES JetsonGPIO.h
  PATHS ${PC_JetsonGPIO_INCLUDE_DIRS}
  PATH_SUFFIXES JetsonGPIO
)
find_library(JetsonGPIO_LIBRARY
  NAMES JetsonGPIO
  PATHS ${PC_JetsonGPIO_LIBRARY_DIRS}
)

set(JetsonGPIO_VERSION ${PC_JetsonGPIO_VERSION})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(JetsonGPIO
  FOUND_VAR JetsonGPIO_FOUND
  REQUIRED_VARS
    JetsonGPIO_LIBRARY
    JetsonGPIO_INCLUDE_DIR
  VERSION_VAR JetsonGPIO_VERSION
)

if(JetsonGPIO_FOUND)
  set(JetsonGPIO_LIBRARIES ${JetsonGPIO_LIBRARY})
  set(JetsonGPIO_INCLUDE_DIRS ${JetsonGPIO_INCLUDE_DIR})
  set(JetsonGPIO_DEFINITIONS ${PC_JetsonGPIO_CFLAGS_OTHER})
endif()