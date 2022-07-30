
file(GLOB_RECURSE SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c)

idf_component_register(SRCS "hw_ow.c"
                    INCLUDE_DIRS "src")
