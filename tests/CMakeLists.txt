include_directories(${PROJECT_SOURCE_DIR}/tests/include)
orocos_component(FriExample src/friExample.cpp) # ...you may add multiple source files
orocos_component(FriExampleKinematic src/friExampleKinematic.cpp) # ...you may add multiple source files
target_link_libraries(FriExample friExampleAbstract)
target_link_libraries(FriExampleKinematic friExampleAbstract)

#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/fri_examples/ directory
#
orocos_install_headers(friExample.hpp) # ...you may add multiple header files
orocos_install_headers(friExampleKinematic.hpp) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

