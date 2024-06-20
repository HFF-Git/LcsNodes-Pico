# This is the CMAKE file at the LcsNodes directory level
# 
#
#
#
# A library that needs an external library code, will simply include the source files in the directory of 
# the including library as a subdirectory. It would be better to keep tem outside the including library tree.
# To be investigated.
# 
# The include for the pico-sdk cmake file depends on the PICO_SDK_PATH. This can be set via an ENV variable or
# in the .vscode settings. This is still a bit flaky. What exactly is the path ? For now I just put in the 
# path. Bit this is not very flexible...
#
# What is also missing is that the targets created are in a nice place vs. the build folder.
#
# Also: what shell scripts would make our typing life easier ? VsCode does seem to do a nice job in calling 
# cmake whenever you change something....
#
# 
#
#
#

cmake_minimum_required( VERSION 3.29 )

Message ( "LcsNodes Cmake" )

message ( "SDK-PATH: $ENV{PICO_SDK_PATH}" )
include( $ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake )

project( LcsNodes C CXX ASM )

pico_sdk_init( )

set( CMAKE_C_STANDARD 11 )
set( CMAKE_CXX_STANDARD 17 )

# add the sub directories for each project. Sub directories need to have a CmakeLists.txt file!

add_subdirectory(LcsCdcLib-Test)
add_subdirectory(LcsRuntimeLib-Test)

add_subdirectory(LcsRuntimeLib)
add_subdirectory(LcsCdcLib)


