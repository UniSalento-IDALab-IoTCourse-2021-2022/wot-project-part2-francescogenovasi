# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

# Log level debug bluetooth stack
#add_compile_options(-DGATT_INITIAL_TRACE_LEVEL=6 -DSMP_INITIAL_TRACE_LEVEL=6 -DAPPL_INITIAL_TRACE_LEVEL=6 -DHID_INITIAL_TRACE_LEVEL=6 -DMCA_INITIAL_TRACE_LEVEL=6 -DAVRC_INITIAL_TRACE_LEVEL=6 -DAVCT_INITIAL_TRACE_LEVEL=6 -DAVDT_INITIAL_TRACE_LEVEL=6 -DA2D_INITIAL_TRACE_LEVEL=6 -DPAN_INITIAL_TRACE_LEVEL=6 -DBNEP_INITIAL_TRACE_LEVEL=6 -DGAP_INITIAL_TRACE_LEVEL=6 -DSDP_INITIAL_TRACE_LEVEL=6 -DRFCOMM_INITIAL_TRACE_LEVEL=6 -DL2CAP_INITIAL_TRACE_LEVEL=6 -DBTM_INITIAL_TRACE_LEVEL=6 -DHCI_INITIAL_TRACE_LEVEL=6 -DRFCOMM_INITIAL_TRACE_LEVEL=6 -DL2CAP_INITIAL_TRACE_LEVEL=6 -DBTM_INITIAL_TRACE_LEVEL=6)

if ( DEV_NAME MATCHES prova)
    add_definitions(-DDEVICE_ID="esp1")
endif()
if (DEV_NAME MATCHES test)
    add_definitions(-DDEVICE_ID="esp2")
endif()
if (DEV_NAME MATCHES esperimento)
    add_definitions(-DDEVICE_ID="esp3")
endif()
if (DEV_NAME MATCHES basta)
    add_definitions(-DDEVICE_ID="esp4")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(mesh_network_beacon)