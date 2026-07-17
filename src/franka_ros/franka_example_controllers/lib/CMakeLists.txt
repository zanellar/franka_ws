cmake_minimum_required(VERSION 3.16)
add_compile_options(-w)

add_subdirectory(osqp-cpp)
list(APPEND LIB_TARGETS osqp-cpp)
list(APPEND LIB_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/osqp-cpp/include")


if(LIB_INCLUDE_DIRS)
        list(REMOVE_DUPLICATES LIB_INCLUDE_DIRS)
        target_include_directories(${PROJECT_NAME} PRIVATE ${LIB_INCLUDE_DIRS})
        if(MSVC)
                target_compile_options(${PROJECT_NAME} PRIVATE
                        "$<$<CXX_COMPILER_ID:Clang>:SHELL:-Xclang -isystem$<JOIN:${LIB_INCLUDE_DIRS}, -Xclang -isystem>>"
                        "$<$<CXX_COMPILER_ID:MSVC>:SHELL:-experimental:external -external:W0 -external:anglebrackets -external:I$<JOIN:${LIB_INCLUDE_DIRS}, -external:I>>")
        else()
                target_compile_options(${PROJECT_NAME} PRIVATE "SHELL:-isystem $<JOIN:${LIB_INCLUDE_DIRS}, -isystem>")
        endif()
endif()

target_link_libraries(${PROJECT_NAME} PRIVATE ${LIB_TARGETS})

