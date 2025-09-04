# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# "Trim" the build. Include the minimal set of components, main, and anything it depends on.
idf_build_set_property(MINIMAL_BUILD ON)
project(KeyChain)

# SPIFFS partition image from folder "spiffs"
set(SPIFFS_IMAGE_DIR ${CMAKE_SOURCE_DIR}/spiffs)
set(SPIFFS_PARTITION_NAME storage)

if(EXISTS ${SPIFFS_IMAGE_DIR})
	# Build SPIFFS image, but DO NOT auto-flash it with the app.
	# This preserves existing files (e.g., uploaded JPGs) across normal flashes.
	# To (re)flash storage manually: idf.py -T storage-flash
	spiffs_create_partition_image(${SPIFFS_PARTITION_NAME} ${SPIFFS_IMAGE_DIR})
endif()

# Use custom partition table
idf_build_set_property(PARTITION_TABLE_CUSTOM_FILENAME ${CMAKE_SOURCE_DIR}/partition.csv)
idf_build_set_property(PARTITION_TABLE_CUSTOM ON)
