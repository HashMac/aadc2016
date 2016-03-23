
# declare our filter
set(FILTER_NAME KACADU_WorldCoordVisualizer)

include_directories(
	${OpenCV_INCLUDE_DIRS}
	../KACADU_InheritanceTemplates
	${oadrive_INCLUDE_DIRS}
)

adtf_add_filter(${FILTER_NAME}
    WorldCoordVisualizer.h
    WorldCoordVisualizer.cpp
)

# linking libs
target_link_libraries( ${FILTER_NAME}
	${OpenCV_LIBS}
	#${icl_core_LIBRARIES}
	${oadrive_LIBRARIES}
)

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
