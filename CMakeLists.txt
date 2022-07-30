
file(GLOB_RECURSE SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c)

idf_component_register(SRCS ${SRC}
                    INCLUDE_DIRS "src"
                    REQUIRES main driver )
