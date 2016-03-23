
# declare our filter
set(FILTER_NAME katana_saveImg)

include_directories(${OpenCV_INCLUDE_DIRS})

adtf_add_filter(${FILTER_NAME}
    SaveImage.h
    SaveImage.cpp
)

# linking libs
target_link_libraries(${FILTER_NAME} ${OpenCV_LIBS})

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
