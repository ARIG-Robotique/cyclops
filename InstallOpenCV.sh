#!/bin/bash

#refresh admin unlock
sudo -v

#exit script if fail
set -e

cd ..

# set install dir
cd opencv/
mkdir build || echo "Build folder already existed, skipping..."
cd build

#-D WITH_NVCUVID=ON \ potetiellement pour cudacodec::VideoCapture
#-D VIDEOIO_PLUGIN_LIST=ffmpeg,gstreamer \
#-D EIGEN_INCLUDE_PATH=/usr/include/eigen3 \

# Ajouter les lignes suivantes si compilation avec CUDA/CuDNN
#-D WITH_CUDA=OFF \
#-D WITH_NVCUVID=ON \
#-D WITH_CUBLAS=ON \
#-D CUDA_FAST_MATH=ON \
#-D WITH_CUDNN=OFF \
#-D OPENCV_DNN_CUDA=ON \


#-D PYTHON3_PACKAGES_PATH=/usr/lib/python3/dist-packages \

# CUDA_ARCH_BIN : 53 pour la jetson Nano, 50,52,61,75,86 pour les GPU
# run cmake

#pour installer les extra opencv
# -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \

CUDA_EN=OFF

if [ ! -z $1 ]; then
    CUDA_EN=$1
fi

if [ "$CUDA_EN" = "ON" ]; then
    EXTRA_MODULES="-D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules"
    echo "Using CUDA and extra modules"
else
    EXTRA_MODULES=""
    echo "Not using CUDA"
fi

cmake -G Ninja \
-D CMAKE_BUILD_TYPE=Release \
-D CMAKE_INSTALL_PREFIX=/usr \
-D WITH_OPENCL=ON \
-D WITH_OPENMP=ON \
-D WITH_OPENGL=OFF \
-D WITH_EIGEN=ON \
-D ENABLE_FAST_MATH=ON \
-D CUDA_GENERATION=Auto \
-D WITH_TBB=ON \
\
-D OPENCV_ENABLE_NONFREE=ON \
\
-D WITH_CUDA=$CUDA_EN \
-D WITH_NVCUVID=$CUDA_EN \
-D WITH_CUBLAS=$CUDA_EN \
-D CUDA_FAST_MATH=$CUDA_EN \
-D WITH_CUDNN=$CUDA_EN \
-D OPENCV_DNN_CUDA=$CUDA_EN \
\
-D WITH_V4L=ON \
-D WITH_LIBV4L=ON \
-D WITH_FFMPEG=ON \
-D WITH_GSTREAMER=ON \
-D WITH_1394=OFF \
\
-D HIGHGUI_ENABLE_PLUGINS=OFF \
-D WITH_QT=OFF \
-D WITH_GTK=ON \
-D WITH_GTK_2_X=OFF \
-D WITH_VTK=OFF \
\
-D BUILD_opencv_python2=OFF \
-D BUILD_opencv_python3=OFF \
-D BUILD_JAVA=OFF \
-D BUILD_opencv_apps=OFF \
\
-D INSTALL_C_EXAMPLES=OFF \
-D INSTALL_PYTHON_EXAMPLES=OFF \
\
-D ENABLE_PROFILING=OFF \
-D OPENCV_GENERATE_PKGCONFIG=ON \
-D BUILD_TESTS=OFF \
-D BUILD_PERF_TESTS=OFF \
-D BUILD_EXAMPLES=OFF \
\
-D BUILD_wechat_qrcode=OFF \
-D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules/ximgproc \
$EXTRA_MODULES \
../

ninja

sudo rm -r /usr/include/opencv4/opencv2 || echo "No previous installation of opencv, skipping..."
sudo ninja install
sudo ldconfig

clear
echo "Done!"
