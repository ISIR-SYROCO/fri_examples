
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libfri_examples-<target>.so
# and installs in the directory lib/orocos/fri_examples/
#
include_directories(${PROJECT_SOURCE_DIR}/include)
add_library(friExampleAbstract friExampleAbstract.cpp)
add_library(friRTNetExampleAbstract friRTNetExampleAbstract.cpp)

#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/fri_examples/ directory
#
orocos_install_headers(friExampleAbstract.hpp friRTNetExampleAbstract.hpp) # ...you may add multiple header files
