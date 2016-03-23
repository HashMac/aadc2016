
# declare our filter
set(FILTER_NAME KACADU_InheritTestThroughput)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(../KACADU_InheritanceTemplates)

adtf_add_filter(${FILTER_NAME}
    ../KACADU_InheritanceTemplates/InheritThroughput.h
    ../KACADU_InheritanceTemplates/InheritThroughput.cpp
    InheritTestThroughput.h
    InheritTestThroughput.cpp
)

# linking libs
target_link_libraries(${FILTER_NAME} ${OpenCV_LIBS})

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
