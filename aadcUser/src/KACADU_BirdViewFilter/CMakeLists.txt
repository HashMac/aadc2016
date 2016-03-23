
# declare our filter
set(FILTER_NAME KACADU_BirdViewFilter)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(../KACADU_InheritanceTemplates)
include_directories(../Util/BirdViewPosConv/)
include_directories(${oadrive_INCLUDE_DIRS})

adtf_add_filter(${FILTER_NAME}
    ../KACADU_InheritanceTemplates/InheritThroughput.h
    ../KACADU_InheritanceTemplates/InheritThroughput.cpp
	../Util/BirdViewPosConv/BirdViewPosConv.h
	../Util/BirdViewPosConv/BirdViewPosConv.cpp	
    BirdViewFilter.h
    BirdViewFilter.cpp
)

# linking libs
target_link_libraries( ${FILTER_NAME}
	${OpenCV_LIBS}
	${oadrive_LIBRARIES}
)

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
