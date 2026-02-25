cmake_minimum_required( VERSION 3.10)

project( HDK_Project )

# CMAKE_PREFIX_PATH must contain the path to the toolkit/cmake subdirectory of
# the Houdini installation. See the "Compiling with CMake" section of the HDK
# documentation for more details, which describes several options for
# specifying this path.
list( APPEND CMAKE_PREFIX_PATH "$ENV{HFS}/toolkit/cmake" )

# Locate Houdini's libraries and header files.
# Registers an imported library target named 'Houdini'.
find_package( Houdini REQUIRED )

set( library_name SOP_PBD )

# Code generation for the embedded DS file 
houdini_generate_proto_headers( 
    FILES   SOP_ProjectConstraints.cpp
            SOP_CreateCollisionConstraints.cpp
            SOP_CreateAttachmentConstraints.cpp
            SOP_CreateDistanceConstraints.cpp
            SOP_CreateRodConstraints.cpp
            SOP_Orientation.cpp
            SOP_RodIO.cpp
            SOP_IntegrateStep.cpp
            SOP_ApplyUpdates.cpp
)

# Add a library and its source files.
add_library( ${library_name} SHARED
    SOP_ProjectConstraints.cpp
    MathUtils.cpp
    PBD.cpp
    SOP_CreateCollisionConstraints.cpp
    SOP_CreateAttachmentConstraints.cpp
    SOP_CreateDistanceConstraints.cpp
    SOP_SolveVelocity.cpp
    SOP_CreateRodConstraints.cpp
    SOP_Orientation.cpp
    SOP_RodIO.cpp
    SOP_IntegrateStep.cpp
    SOP_ApplyUpdates.cpp
)

# Link against the Houdini libraries, and add required include directories and
# compile definitions.
target_link_libraries( ${library_name} Houdini )

# Include ${CMAKE_CURRENT_BINARY_DIR} for the generated header.
target_include_directories( ${library_name} PRIVATE
    ${CMAKE_CURRENT_BINARY_DIR}
)

target_compile_options(${library_name} PRIVATE -g)
include_directories(${PROJECT_SOURCE_DIR})

# Sets several common target properties, such as the library's output directory.
houdini_configure_target( ${library_name} )

# Find Eigen path to include
find_path(EIGEN_DIR NAMES signature_of_eigen3_matrix_library
        PATHS
        /opt/homebrew/Cellar/eigen
#        /usr/local/include/eigen3
        PATH_SUFFIXES eigen3 eigen)
if(NOT EXISTS ${EIGEN_DIR})
    message(FATAL_ERROR "Please check that the set Eigen directory is valid!")
endif()
include_directories(${EIGEN_DIR})
include_directories(include)

add_executable(sop_pbd_test
    test/test.cpp
    MathUtils.cpp
    extern/intg-pbd/MathFunctions.cpp
    extern/intg-pbd/PositionBasedElasticRods.cpp
    extern/intg-pbd/TimeIntegration.cpp
)

target_link_libraries(sop_pbd_test Houdini)
