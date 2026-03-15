# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file LICENSE.rst or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "/home/omen16/workspace/grad_ws/build/kiss_icp/_deps/tessil-src")
  file(MAKE_DIRECTORY "/home/omen16/workspace/grad_ws/build/kiss_icp/_deps/tessil-src")
endif()
file(MAKE_DIRECTORY
  "/home/omen16/workspace/grad_ws/build/kiss_icp/_deps/tessil-build"
  "/home/omen16/workspace/grad_ws/build/kiss_icp/_deps/tessil-subbuild/tessil-populate-prefix"
  "/home/omen16/workspace/grad_ws/build/kiss_icp/_deps/tessil-subbuild/tessil-populate-prefix/tmp"
  "/home/omen16/workspace/grad_ws/build/kiss_icp/_deps/tessil-subbuild/tessil-populate-prefix/src/tessil-populate-stamp"
  "/home/omen16/workspace/grad_ws/build/kiss_icp/_deps/tessil-subbuild/tessil-populate-prefix/src"
  "/home/omen16/workspace/grad_ws/build/kiss_icp/_deps/tessil-subbuild/tessil-populate-prefix/src/tessil-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/omen16/workspace/grad_ws/build/kiss_icp/_deps/tessil-subbuild/tessil-populate-prefix/src/tessil-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/omen16/workspace/grad_ws/build/kiss_icp/_deps/tessil-subbuild/tessil-populate-prefix/src/tessil-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
