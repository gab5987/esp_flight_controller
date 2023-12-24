file(GLOB MAIN_SOURCES "*.c*")
set(SOURCES ${MAIN_SOURCES})
idf_component_register(SRCS ${SOURCES} INCLUDE_DIRS "." "../include")

