
# declare our filter
set(FILTER_NAME KACADU_BirdViewCalibration)
adtf_use_qt(QtCore QtGui)

#we build this example only if qt was found
if(QT_FOUND)
    include_directories(${OpenCV_INCLUDE_DIRS})
     set(HEADER_MOC
            displaywidget.h
            BirdViewCalibration.h

            )

        adtf_add_moc(${FILTER_NAME}
                    ${HEADER_MOC}
        )

    adtf_add_filter(${FILTER_NAME}

        displaywidget.cpp
        BirdViewCalibration.h
        BirdViewCalibration.cpp
    )


    # linking libs
    target_link_libraries(${FILTER_NAME} ${OpenCV_LIBS})

    # Specify where it should be installed to
    adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
endif(QT_FOUND)
