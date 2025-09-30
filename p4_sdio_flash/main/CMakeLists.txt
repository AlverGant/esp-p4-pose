set(srcs main.c example_common.c)
set(include_dirs .)

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS ${include_dirs})
set(target ${COMPONENT_LIB})

include(${CMAKE_CURRENT_LIST_DIR}/bin2array.cmake)
create_resources(${CMAKE_CURRENT_LIST_DIR}/../target_binaries ${CMAKE_BINARY_DIR}/binaries.c)
set_property(SOURCE ${CMAKE_BINARY_DIR}/binaries.c PROPERTY GENERATED 1)
target_sources(${target} PRIVATE ${CMAKE_BINARY_DIR}/binaries.c)

