#config file
sudo apt-get install -y libconfig++-dev
#build
sudo apt-get install -y build-essential git unzip pkg-config zlib1g-dev
sudo apt-get install -y gcc-multilib g++-multilib
sudo apt-get install -y cmake ninja-build
#opengl
sudo apt-get install -y libassimp-dev freeglut3-dev libglew-dev libglm-dev libglfw3-dev
#image formats
sudo apt-get install -y libjpeg-dev libjpeg8-dev libjpeg-turbo8-dev libpng-dev libtiff-dev
#video formats
sudo apt-get install -y libxvidcore-dev libx264-dev libxine2-dev
#multithreading
sudo apt-get install -y libtbb2 libtbb2-dev 
#ui
sudo apt-get install -y libgtk-3-dev
sudo apt-get install -y qtbase5-dev
#v4l2
sudo apt-get install -y libv4l-dev v4l-utils qv4l2 
#ffmpeg
sudo apt-get install -y ffmpeg libavcodec-dev libavformat-dev libswscale-dev libavutil-dev libavresample-dev
#ffmpeg + cuda
#sudo apt-get install -y libffmpeg-nvenc-dev
#gstreamer
sudo apt-get install -y gstreamer1.0-tools libgstreamer-plugins-base1.0-dev 
sudo apt-get install -y libgstreamer-plugins-good1.0-dev libgstreamer-plugins-bad1.0-dev
#math
sudo apt-get install -y libopenblas-dev libatlas-base-dev libblas-dev
sudo apt-get install -y liblapack-dev liblapacke-dev libeigen3-dev gfortran
#audio
sudo apt-get install -y libvorbis-dev libtesseract-dev
sudo apt-get install -y libfaac-dev libmp3lame-dev libtheora-dev libpostproc-dev
#speech recognition
sudo apt-get install -y libopencore-amrnb-dev libopencore-amrwb-dev
#serialisation
sudo apt-get install -y libhdf5-dev protobuf-compiler
sudo apt-get install -y libprotobuf-dev libgoogle-glog-dev libgflags-dev