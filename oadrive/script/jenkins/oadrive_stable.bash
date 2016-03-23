#!/bin/bash
set -e

if [ "${JENKINS_HOME}" == "" ]; then
  cd $(mktemp -d)
  echo "#
# Not on Jenkins!
#"
  HOME=$(mktemp -d)
else
  echo "#
# Jenkins Build
#"
  test -d ic_workspace && rm -rf ic_workspace
  test -d ${HOME}/.cmake && rm -rf ${HOME}/.cmake
fi


echo "Using workdir $PWD"

#
# Checkout
#
git clone git://idsgit.fzi.de/core/ic_workspace.git
cd ic_workspace

# manually clone oadrive to be able to checkout a specific branch before grabbing all dependencies.
#mkdir packages
#cd packages
#git clone git://idsgit.fzi.de/~kuhnt/cognitive-cars/kacadu-oadrive.git oadrive
#cd oadrive
#git checkout master
#cd ../.. # to ic_workspace folder
./IcWorkspace.py grab oadrive -a # this grabs all the dependencies

cd packages/oadrive
git checkout opencv2   # this is the current stable branch to test
cd ../..

#
# Build
#
mkdir build && cd build
cmake ..
make

#
# Also test the build
#
make test

#
# Run additional tests here. For example checking if files exist.
#
test -f bin/test_oadrive_birdViewCal
test -f bin/test_oadrive_core_trajectory
test -f bin/test_oadrive_core_velocity_interpolator
test -f bin/test_oadrive_depth
test -f bin/test_oadrive_drivermodule
test -f bin/test_oadrive_eventRegion
test -f bin/test_oadrive_haarFeatureEvaluation
test -f bin/test_oadrive_haarfeatures
test -f bin/test_oadrive_haarfilter
#test -f bin/test_oadrive_interface
test -f bin/test_oadrive_lanedetection
#test -f bin/test_oadrive_world
test -f bin/test_oadrive_log
#test -f bin/test_oadrive_missioncontrol
test -f bin/test_oadrive_missioncontrol_man_parser
test -f bin/test_oadrive_obstacle
test -f bin/test_oadrive_patches
test -f bin/test_oadrive_patch_stitching
#test -f bin/test_oadrive_patch_traj
#test -f bin/test_oadrive_timer
test -f bin/test_oadrive_util_BirdViewConv
test -f bin/test_oadrive_util_BirdViewPosConv
test -f bin/test_oadrive_util_ultrasonic
test -f bin/test_oadrive_writedouble



cd ../..
