# add the include directory of the filter
include_directories(../include)
find_package(octomap REQUIRED)
# declare our filter
set(FILTER_NAME KACADU_USView)

adtf_add_filter(${FILTER_NAME}
    stdafx.h
    stdafx.cpp

    USView.h
    USView.cpp
)

target_link_libraries( ${FILTER_NAME}
        ${oadrive_LIBRARIES} ${OCTOMAP_LIBRARIES}
)
include_directories( ${OCTOMAP_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)
# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
