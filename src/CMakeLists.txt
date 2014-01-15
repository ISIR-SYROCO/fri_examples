
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
orocos_component(FriExample friExample.cpp) # ...you may add multiple source files
orocos_component(FriExampleJointReader friExampleJointReader.cpp) # ...you may add multiple source files
orocos_component(FriExampleCartesianPosReader friExampleCartesianPosReader.cpp) # ...you may add multiple source files

#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/fri_examples/ directory
#
orocos_install_headers(friExampleAbstract.hpp) # ...you may add multiple header files
orocos_install_headers(friExample.hpp) # ...you may add multiple header files
orocos_install_headers(friExampleJointReader.hpp) # ...you may add multiple header files
orocos_install_headers(friExampleCartesianPosReader.hpp) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

