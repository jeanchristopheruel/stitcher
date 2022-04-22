#!/bin/sh

# Note: It is best to install libraries in /prefix to make the use of docker multistage easier.
apt-get update
apt-get clean
apt-get install -y --no-install-recommends g++ git

# install OPENCV WITH GSTREAMER
apt-get purge *gstreamer* -y
apt-get install -y --no-install-recommends libgstreamer1.0-0 gstreamer1.0-plugins-base gstreamer1.0-plugins-good libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-ugly gstreamer1.0-libav

git clone --depth 1 --branch 4.3.0 https://github.com/opencv/opencv
cd opencv
mkdir build && cd build

/prefix/bin/cmake \
  -D CMAKE_BUILD_TYPE=RELEASE \
  -D CMAKE_INSTALL_PREFIX=/prefix/ \
  -D OPENCV_ENABLE_NONFREE=OFF \
  -D WITH_GSTREAMER=ON \
  -D ENABLE_FAST_MATH=ON \
  -D WITH_TBB=ON \
  -D WITH_QT=OFF \
  -D WITH_OPENGL=ON \
  -D WITH_LIBV4L=OFF \
  -D WITH_CUDA=OFF \
  -D PYTHON_EXECUTABLE=$(which python3) \
  -D BUILD_OPENCV_PYTHON2=OFF \
  -D BUILD_OPENCV_PYTHON3=ON \
  -D PYTHON3_EXECUTABLE=$(which python3) \
  -D PYTHON3_INCLUDE_DIR=$(python3 -c "from distutils.sysconfig import get_python_inc; print(get_python_inc())") \
  -D PYTHON3_PACKAGES_PATH=$(python3 -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())") \
  -D BUILD_TESTS=OFF \
  -D BUILD_OPENCV_JAVA=OFF \
  -D BUILD_PERF_TESTS=OFF \
  -D BUILD_EXAMPLES=OFF \
  -D BUILD_opencv_core=ON\
  -D BUILD_opencv_imgproc=ON\
  -D BUILD_opencv_imgcodecs=ON\
  -D BUILD_opencv_videoio=ON\
  -D BUILD_opencv_highgui=ON\
  -D BUILD_opencv_video=OFF\
  -D BUILD_opencv_calib3d=ON \
  -D BUILD_opencv_features2d=ON \
  -D BUILD_opencv_flann=ON \
  -D BUILD_opencv_ml=OFF \
  -D BUILD_opencv_objdetect=OFF \
  -D BUILD_opencv_photo=OFF \
  -D BUILD_opencv_stitching=ON \
  -D BUILD_opencv_dnn=OFF\
  -D BUILD_opencv_ml=OFF\
  ..

make -j$(nproc) 
make install -j$(nproc) 
cd .. && mkdir ../licenses/opencv && cp LICENSE ../licenses/opencv/
cd .. && rm -rf opencv

RVAL=false
OPENCV_FILE="/prefix/lib/libopencv_core.so"
if [ ! -f $OPENCV_FILE ]; then
    echo "OPENCV was not correctly installed."
    exit 1
fi

ldconfig 
exit 0