#rpmfusion
sudo dnf install https://mirrors.rpmfusion.org/free/fedora/rpmfusion-free-release-$(rpm -E %fedora).noarch.rpm \
https://mirrors.rpmfusion.org/nonfree/fedora/rpmfusion-nonfree-release-$(rpm -E %fedora).noarch.rpm
#build
sudo dnf install -y cmake ninja-build
#opengl
sudo dnf install -y assimp-devel freeglut-devel glew-devel glm-devel glfw-devel
#image formats
#sudo apt-get install -y libjpeg-dev libjpeg8-dev libjpeg-turbo8-dev libpng-dev libtiff-dev
#video formats
#sudo apt-get install -y libxvidcore-dev libx264-dev libxine2-dev
#multithreading
#sudo apt-get install -y libtbb2 libtbb2-dev
#ui
#sudo apt-get install -y libgtk-3-dev
#sudo apt-get install -y qtbase5-dev
#v4l2
sudo dnf install -y libv4l-devel v4l-utils qv4l2
#ffmpeg
sudo dnf install -y ffmpeg-devel
sudo dnf install -y libavcodec-freeworld openh264-devel
#ffmpeg + cuda
#sudo apt-get install -y libffmpeg-nvenc-dev
#gstreamer
sudo dnf install -y gst-devtools gstreamer1-devel gstreamer1-plugins-base-devel
#sudo apt-get install -y libgstreamer-plugins-good1.0-dev libgstreamer-plugins-bad1.0-dev
#math
sudo dnf install -y openblas-devel atlas-devel blas-devel
sudo dnf install -y lapack-devel eigen3-devel
#audio
#sudo apt-get install -y libvorbis-dev libtesseract-dev
#sudo apt-get install -y libfaac-dev libmp3lame-dev libtheora-dev libpostproc-dev
#speech recognition
#sudo apt-get install -y libopencore-amrnb-dev libopencore-amrwb-dev
#serialisation
#sudo apt-get install -y libhdf5-dev protobuf-compiler
#sudo apt-get install -y libprotobuf-dev libgoogle-glog-dev libgflags-dev