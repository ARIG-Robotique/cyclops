#!/bin/bash
echo "Déverouillage admin, ca sert pour installer"
export STARTDIR=`pwd`
source /etc/os-release
if echo $PRETTY_NAME | grep -q Debian; then
  ./InstallRequirementDebian.sh
fi
if echo $PRETTY_NAME | grep -q Fedora; then
  ./InstallRequirementFedora.sh
fi
cd ..
#sudo usermod -a -G dialout $USER
#git clone https://gitlab.kitware.com/vtk/vtk.git || echo "Failed to clone vtk : already exists"
#cd vtk/
#git checkout v9.2.2
#mkdir build || echo "Build folder already existed, skipping..."
#cd build
#cmake -G Ninja -D CMAKE_BUILD_TYPE=RELEASE ../
#sudo -v
#ninja
#sudo ninja install
#cd ../..
#sudo -v

OPENCV_VERSION=4.10.0

#git clone https://github.com/opencv/opencv_contrib.git || echo "Failed to clone opencv_contrib : already exists"
#cd opencv_contrib/
#git reset --hard
#git fetch
#git checkout $OPENCV_VERSION
#sudo -v
#cd ..
git clone https://github.com/opencv/opencv.git || echo "Failed to clone opencv : already exists"
cd opencv/
git reset --hard
git fetch
git checkout $OPENCV_VERSION
cd ..
cd $STARTDIR
time ./InstallOpenCV.sh