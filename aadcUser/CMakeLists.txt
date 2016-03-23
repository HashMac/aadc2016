cmake_minimum_required(VERSION 2.8.4 FATAL_ERROR)

cmake_policy(SET CMP0002 NEW)
cmake_policy(SET CMP0003 NEW)
cmake_policy(SET CMP0011 NEW)

# The project and cmake vars must only be defined in case that only the base
# filter should be build.
if(NOT AADC_BUILD_ALL_IN_ONE)
    # define a project name
    project(aadc_user)
	Message("compiler")
    #set the install prefix if not already done.
	if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
		if(MSVC10)
			set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/../../bin/${CMAKE_SYSTEM_NAME}/vc10" CACHE PATH "The install directory" FORCE)
		elseif(CMAKE_COMPILER_IS_GNUCXX)
			set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/../../bin/${CMAKE_SYSTEM_NAME}/gcc" CACHE PATH "The install directory" FORCE)
		else()
			set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/../../bin/${CMAKE_SYSTEM_NAME}" CACHE PATH "The install directory" FORCE)
		endif()
		set(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT FALSE)
		message("Install dir is ${CMAKE_INSTALL_PREFIX}")
	endif(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    # CMAKE_BUILD_TYPE is empty on Unix systems, but needed by ADTF. Otherwise the ADTF libs
    # can't be found and linked.
    if(UNIX AND NOT CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
                "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
                FORCE)
    endif(UNIX AND NOT CMAKE_BUILD_TYPE)

    # set toolchain file on Unix
    if(UNIX AND NOT CMAKE_TOOLCHAINFILE)
        set(CMAKE_TOOLCHAIN_FILE "/opt/adtf/toolchainfiles/ARM_toolchain_build_on_target.cmake")
    endif(UNIX AND NOT CMAKE_TOOLCHAINFILE)

endif(NOT AADC_BUILD_ALL_IN_ONE)

find_package(ADTF REQUIRED)
#-------General------------


set(AADC_INSTALL_DIR .)
set(AADC_INSTALL_DIR_LIBS .)

set(AADC_DIR ../)
find_package(AADC REQUIRED)
FIND_PACKAGE( icl_core REQUIRED )
FIND_PACKAGE( oadrive REQUIRED )
FIND_PACKAGE( Eigen3 REQUIRED )
FIND_PACKAGE(redisclient PATHS "../../../FZI/ic_workspace/packages/oadrive/")

#--------------sources----------------------------
include_directories(
	./include
	${EIGEN3_INCLUDE_DIR}
	${icl_core_INCLUDE_DIRS} 
	${oadrive_INCLUDE_DIRS}
)

add_subdirectory(src/KACADU_SaveImage)
#add_subdirectory(src/MomenTUM_LaneDetection)
#add_subdirectory(src/MomenTUM_ImageProcessing)
#add_subdirectory(src/LaneDetection)

add_subdirectory(src/KACADU_BirdViewCalibration)
add_subdirectory(src/KACADU_BirdViewFilter)
add_subdirectory(src/KACADU_InheritTestThroughput)
add_subdirectory(src/KACADU_ConverterWheels)
add_subdirectory(src/KACADU_WorldCoordCalculator)
#add_subdirectory(src/KACADU_WorldCoordCalculatorAbs)
add_subdirectory(src/KACADU_WorldCoordVisualizer)
add_subdirectory(src/KACADU_SteeringController)
add_subdirectory(src/KACADU_EmergencyBreak)
#add_subdirectory(src/KACADU_USView)
#add_subdirectory(src/KACADU_LaneFollowing)
add_subdirectory(src/KACADU_MonsterFilter)
add_subdirectory(src/KACADU_WheelSpeedController)
