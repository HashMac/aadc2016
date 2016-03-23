
set(FILTER_NAME KACADU_LaneFollowing)

#FIND_PACKAGE( OpenCV REQUIRED )
#FIND_PACKAGE( icl_core REQUIRED )
#FIND_PACKAGE( oadrive REQUIRED )
#FIND_PACKAGE( Eigen3 REQUIRED )

set ( NN_LINK_LIBS libluajit.so libluaT.so libTH.so )
link_directories(/home/aadc/torch/install/lib/)


include_directories( 
	${OpenCV_INCLUDE_DIRS} 
	#${EIGEN3_INCLUDE_DIR}
	./NNLaneTracker
	/home/aadc/torch/install/include/
	../KACADU_InheritanceTemplates		# For inheritance
	../Util/BirdViewPosConv
	../Util/BirdViewPosConv
)

#message(STATUS "\n\n\nicl_core_INCLUDE_DIRS:" ${icl_core_INCLUDE_DIRS} " \n\n\n")
adtf_add_filter(${FILTER_NAME}
    ../KACADU_InheritanceTemplates/InheritThroughput.h
    ../KACADU_InheritanceTemplates/InheritThroughput.cpp
	NNLaneTracker/NNPredictor.cpp
	NNLaneTracker/NNPredictor.h
	../Util/BirdViewPosConv/BirdViewPosConv.h
	../Util/BirdViewPosConv/BirdViewPosConv.cpp
	LaneFollowing.cpp
	LaneFollowing.h
)
target_link_libraries( ${FILTER_NAME}
	${NN_LINK_LIBS}
	${OpenCV_LIBS}
	#${icl_core_LIBRARIES}
	${oadrive_LIBRARIES}
)

#message(STATUS "\n\n" )
#message(STATUS ${oadrive_LIBRARIES} )

# Dependencies
#add_dependencies(${FILTER_NAME}
  #lanetracker
#)

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})

